//! Force feedback for the MOZA wheel base.
//!
//! The steering torque comes from the vehicle model: the front tires' aligning
//! torque and lateral force acting through the steering geometry, scaled from
//! steering column N m to device output. Friction stands in for tire scrub at
//! parking speeds, which the tire model does not cover.
//!
//! Each frame the game computes a steering torque (positive = pushes the wheel
//! clockwise) and a friction amount, and sends them to a writer thread that
//! owns its own handle to the wheel's evdev node. The writer keeps one
//! constant-force effect and one friction effect playing and updates them in
//! place. All effects belong to that handle, so the kernel stops them when it
//! closes, including when the game crashes.

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::thread;
use std::time::Duration;

use bevy::prelude::*;
use evdev::{Device, FFCondition, FFEffectData, FFEffectKind, FFEnvelope, FFReplay, FFTrigger};

use crate::vehicle::{Vehicle, VehicleParams};
use crate::wheel::find_wheel;

/// Flips the sign of the torque sent to the device.
/// Set from the direction test: holding T must turn the wheel clockwise.
/// On the MOZA R12 with hid-universal-pidff, a positive level turns it counter-clockwise.
const FFB_DIRECTION_SIGN: f32 = -1.0;

/// Linux FF direction for a single-axis wheel (90 degrees).
const WHEEL_DIRECTION: u16 = 0x4000;

/// Force used by the direction test, as a fraction of full device torque.
const TEST_TORQUE: f32 = 0.15;

/// If the game stops sending commands for this long, the force is zeroed.
const WATCHDOG_TIMEOUT: Duration = Duration::from_millis(200);

const DEFAULT_STRENGTH: f32 = 0.5;
const STRENGTH_STEP: f32 = 0.05;

/// Steering column torque (N m) that maps to full device torque at 100%
/// strength. Without power steering a road car reaches about 15-25 N m at
/// the grip limit.
const FULL_SCALE_COLUMN_TORQUE: f32 = 25.0;
/// Friction at standstill (tire scrub) and at speed, as a fraction of full
/// device torque at 100% strength.
const STANDSTILL_FRICTION: f32 = 0.25;
const MOVING_FRICTION: f32 = 0.03;
/// Speed (m/s) over which scrub friction fades out once rolling.
const SCRUB_FADE_SPEED: f32 = 3.0;

pub struct FfbPlugin;

impl Plugin for FfbPlugin {
    fn build(&self, app: &mut App) {
        let (sender, receiver) = mpsc::channel();
        spawn_writer_thread(receiver);

        app.insert_resource(FfbSender(sender))
            .init_resource::<ForceFeedback>()
            .add_systems(Update, (adjust_ffb_settings, compute_ffb, send_ffb).chain());
    }
}

/// Force feedback settings and the last computed output.
#[derive(Resource, Debug)]
pub struct ForceFeedback {
    pub enabled: bool,
    /// Master strength, 0.0..=1.0 of the device's maximum torque.
    pub strength: f32,
    /// True while the direction test key is held.
    pub direction_test: bool,
    /// Steering column torque from the vehicle model, N m; positive clockwise.
    pub column_torque: f32,
    /// Torque sent this frame, -1.0..=1.0; positive pushes the wheel clockwise.
    pub torque: f32,
    /// True when the requested torque exceeds what the device can output.
    pub clipping: bool,
    /// Friction sent this frame, 0.0..=1.0.
    pub friction: f32,
}

impl Default for ForceFeedback {
    fn default() -> Self {
        Self {
            enabled: true,
            strength: DEFAULT_STRENGTH,
            direction_test: false,
            column_torque: 0.0,
            torque: 0.0,
            clipping: false,
            friction: 0.0,
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Debug)]
struct FfbCommand {
    constant_level: i16,
    friction_coefficient: i16,
}

#[derive(Resource)]
struct FfbSender(Sender<FfbCommand>);

/// F toggles FFB, Left/Right change strength, holding T runs the direction test.
fn adjust_ffb_settings(keys: Res<ButtonInput<KeyCode>>, mut ffb: ResMut<ForceFeedback>) {
    if keys.just_pressed(KeyCode::KeyF) {
        ffb.enabled = !ffb.enabled;
    }
    if keys.just_pressed(KeyCode::ArrowRight) {
        ffb.strength += STRENGTH_STEP;
    }
    if keys.just_pressed(KeyCode::ArrowLeft) {
        ffb.strength -= STRENGTH_STEP;
    }
    // Round away float drift from repeated steps.
    ffb.strength = (ffb.strength.clamp(0.0, 1.0) / STRENGTH_STEP).round() * STRENGTH_STEP;
    ffb.direction_test = keys.pressed(KeyCode::KeyT);
}

fn compute_ffb(car: Res<Vehicle>, params: Res<VehicleParams>, mut ffb: ResMut<ForceFeedback>) {
    ffb.column_torque = car.steering_column_torque(&params);
    ffb.clipping = false;

    if ffb.direction_test {
        ffb.torque = TEST_TORQUE;
        ffb.friction = 0.0;
        return;
    }
    if !ffb.enabled {
        ffb.torque = 0.0;
        ffb.friction = 0.0;
        return;
    }

    let torque = ffb.column_torque / FULL_SCALE_COLUMN_TORQUE * ffb.strength;
    ffb.clipping = torque.abs() > 1.0;
    ffb.torque = torque.clamp(-1.0, 1.0);

    let rolling = 1.0 - (-car.velocity.length() / SCRUB_FADE_SPEED).exp();
    let friction = STANDSTILL_FRICTION.lerp(MOVING_FRICTION, rolling);
    ffb.friction = (friction * ffb.strength).clamp(0.0, 1.0);
}

fn send_ffb(ffb: Res<ForceFeedback>, sender: Res<FfbSender>) {
    let command = FfbCommand {
        constant_level: (ffb.torque * FFB_DIRECTION_SIGN * i16::MAX as f32) as i16,
        friction_coefficient: (ffb.friction * i16::MAX as f32) as i16,
    };
    // The writer thread only exits if it panicked; nothing useful to do then.
    let _ = sender.0.send(command);
}

fn spawn_writer_thread(receiver: Receiver<FfbCommand>) {
    thread::Builder::new()
        .name("ffb-writer".into())
        .spawn(move || {
            loop {
                if let Some(device) = find_wheel() {
                    match drive_device(device, &receiver) {
                        Ok(()) => return,
                        Err(err) => warn!("Force feedback stopped: {err}"),
                    }
                }
                thread::sleep(Duration::from_secs(1));
                if !discard_pending(&receiver) {
                    return;
                }
            }
        })
        .expect("failed to spawn ffb writer thread");
}

/// Drops commands sent while no device was usable.
/// Returns false once the game has exited.
fn discard_pending(receiver: &Receiver<FfbCommand>) -> bool {
    loop {
        match receiver.try_recv() {
            Ok(_) => {}
            Err(TryRecvError::Empty) => return true,
            Err(TryRecvError::Disconnected) => return false,
        }
    }
}

/// Plays effects on `device` until it errors. Returns `Ok` when the game exits.
fn drive_device(mut device: Device, receiver: &Receiver<FfbCommand>) -> std::io::Result<()> {
    device.set_ff_gain(u16::MAX)?;

    let mut current = FfbCommand::default();
    let mut constant = device.upload_ff_effect(constant_effect(current.constant_level))?;
    let mut friction = device.upload_ff_effect(friction_effect(current.friction_coefficient))?;
    constant.play(1)?;
    friction.play(1)?;
    info!("Force feedback ready");

    loop {
        let command = match receiver.recv_timeout(WATCHDOG_TIMEOUT) {
            // Skip stale commands if the device is slower than the game.
            Ok(command) => receiver.try_iter().last().unwrap_or(command),
            Err(RecvTimeoutError::Timeout) => FfbCommand::default(),
            Err(RecvTimeoutError::Disconnected) => return Ok(()),
        };

        if command.constant_level != current.constant_level {
            constant.update(constant_effect(command.constant_level))?;
        }
        if command.friction_coefficient != current.friction_coefficient {
            friction.update(friction_effect(command.friction_coefficient))?;
        }
        current = command;
    }
}

fn constant_effect(level: i16) -> FFEffectData {
    effect_data(FFEffectKind::Constant {
        level,
        envelope: FFEnvelope {
            attack_length: 0,
            attack_level: 0,
            fade_length: 0,
            fade_level: 0,
        },
    })
}

fn friction_effect(coefficient: i16) -> FFEffectData {
    let axis = FFCondition {
        right_saturation: u16::MAX,
        left_saturation: u16::MAX,
        right_coefficient: coefficient,
        left_coefficient: coefficient,
        deadband: 0,
        center: 0,
    };
    let unused_axis = FFCondition {
        right_coefficient: 0,
        left_coefficient: 0,
        ..axis
    };
    effect_data(FFEffectKind::Friction {
        condition: [axis, unused_axis],
    })
}

fn effect_data(kind: FFEffectKind) -> FFEffectData {
    FFEffectData {
        direction: WHEEL_DIRECTION,
        trigger: FFTrigger::default(),
        // A length of 0 plays the effect until it is stopped or removed.
        replay: FFReplay {
            length: 0,
            delay: 0,
        },
        kind,
    }
}
