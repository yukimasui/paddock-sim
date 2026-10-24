//! Wheel base, pedal and shift paddle input read directly from the Linux
//! evdev interface.
//!
//! A background thread finds the MOZA wheel base, reads its steering axis and
//! the axes of pedals attached to the base, and publishes the latest values
//! into a shared state that a Bevy system copies into the [`Steering`] and
//! [`Pedals`] resources every frame. The thread keeps retrying, so the wheel
//! can be plugged in or reconnected while the game runs.

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use bevy::prelude::*;
use evdev::{AbsInfo, AbsoluteAxisCode, Device, EventSummary, KeyCode};

/// USB vendor ID of Gudsen (MOZA Racing).
const MOZA_VENDOR_ID: u16 = 0x346e;

/// Axis that carries the steering position on MOZA wheel bases.
const STEERING_AXIS: AbsoluteAxisCode = AbsoluteAxisCode::ABS_X;
/// Axes of CRP2 pedals connected to the wheel base's pedal port.
const THROTTLE_AXIS: AbsoluteAxisCode = AbsoluteAxisCode::ABS_Z;
const BRAKE_AXIS: AbsoluteAxisCode = AbsoluteAxisCode::ABS_RZ;
/// Shift paddles on the wheel rim, reported as buttons by the base.
const UPSHIFT_BUTTON: KeyCode = KeyCode::new(301);
const DOWNSHIFT_BUTTON: KeyCode = KeyCode::new(300);

/// Lock-to-lock rotation range assumed until changed at runtime.
/// Must match the "steering wheel rotation angle" configured in MOZA Pit House,
/// since the device scales the axis over that range.
pub const DEFAULT_ROTATION_RANGE_DEG: f32 = 900.0;

pub struct WheelPlugin;

impl Plugin for WheelPlugin {
    fn build(&self, app: &mut App) {
        let shared = WheelShared::default();
        spawn_reader_thread(shared.clone());

        app.insert_resource(shared)
            .insert_resource(Steering::default())
            .insert_resource(Pedals::default())
            .insert_resource(Paddles::default())
            .add_systems(PreUpdate, sync_inputs);
    }
}

/// Latest steering state, updated once per frame.
#[derive(Resource, Debug)]
pub struct Steering {
    pub device_name: Option<String>,
    pub connected: bool,
    pub raw: AxisState,
    /// -1.0 = full left lock, 0.0 = center, 1.0 = full right lock.
    pub normalized: f32,
    pub rotation_range_deg: f32,
}

impl Default for Steering {
    fn default() -> Self {
        Self {
            device_name: None,
            connected: false,
            raw: AxisState::default(),
            normalized: 0.0,
            rotation_range_deg: DEFAULT_ROTATION_RANGE_DEG,
        }
    }
}

impl Steering {
    /// Steering angle in degrees; positive means turned to the right.
    pub fn angle_deg(&self) -> f32 {
        self.normalized * self.rotation_range_deg * 0.5
    }
}

/// Latest pedal state, updated once per frame.
/// Each pedal is 0.0 when released and 1.0 when fully pressed.
#[derive(Resource, Default, Debug)]
pub struct Pedals {
    pub throttle: f32,
    pub brake: f32,
    pub raw_throttle: AxisState,
    pub raw_brake: AxisState,
}

/// How many times each shift paddle has been pulled since start-up.
/// Compare with earlier values to find new pulls.
#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct Paddles {
    pub upshifts: u32,
    pub downshifts: u32,
}

/// Raw value and reported range of one evdev axis.
#[derive(Default, Clone, Copy, Debug)]
pub struct AxisState {
    pub value: i32,
    pub min: i32,
    pub max: i32,
}

impl AxisState {
    fn from_absinfo(info: &AbsInfo) -> Self {
        Self {
            value: info.value(),
            min: info.minimum(),
            max: info.maximum(),
        }
    }

    /// Position within the range: 0.0 at `min`, 1.0 at `max`.
    fn unit(&self) -> f32 {
        if self.max <= self.min {
            return 0.0;
        }
        ((self.value - self.min) as f32 / (self.max - self.min) as f32).clamp(0.0, 1.0)
    }
}

#[derive(Default, Clone, Debug)]
struct RawWheel {
    device_name: Option<String>,
    connected: bool,
    steering: AxisState,
    throttle: AxisState,
    brake: AxisState,
    paddles: Paddles,
}

#[derive(Resource, Default, Clone)]
struct WheelShared(Arc<Mutex<RawWheel>>);

fn sync_inputs(
    shared: Res<WheelShared>,
    mut steering: ResMut<Steering>,
    mut pedals: ResMut<Pedals>,
    mut paddles: ResMut<Paddles>,
) {
    let raw = shared.0.lock().unwrap().clone();
    *paddles = raw.paddles;

    steering.device_name = raw.device_name;
    steering.connected = raw.connected;
    steering.raw = raw.steering;
    pedals.raw_throttle = raw.throttle;
    pedals.raw_brake = raw.brake;

    if raw.connected {
        steering.normalized = raw.steering.unit() * 2.0 - 1.0;
        pedals.throttle = raw.throttle.unit();
        pedals.brake = raw.brake.unit();
    } else {
        steering.normalized = 0.0;
        pedals.throttle = 0.0;
        pedals.brake = 0.0;
    }
}

fn spawn_reader_thread(shared: WheelShared) {
    thread::Builder::new()
        .name("wheel-reader".into())
        .spawn(move || {
            loop {
                match find_wheel() {
                    Some(device) => {
                        if let Err(err) = read_wheel(device, &shared) {
                            warn!("Steering wheel disconnected: {err}");
                        }
                        shared.0.lock().unwrap().connected = false;
                    }
                    None => thread::sleep(Duration::from_secs(1)),
                }
            }
        })
        .expect("failed to spawn wheel reader thread");
}

pub(crate) fn find_wheel() -> Option<Device> {
    evdev::enumerate().map(|(_, device)| device).find(|device| {
        device.input_id().vendor() == MOZA_VENDOR_ID
            && device
                .supported_absolute_axes()
                .is_some_and(|axes| axes.contains(STEERING_AXIS))
    })
}

/// Blocks reading events from `device` until it errors (e.g. is unplugged).
fn read_wheel(mut device: Device, shared: &WheelShared) -> std::io::Result<()> {
    let mut state = RawWheel {
        device_name: device.name().map(str::to_owned),
        connected: true,
        // Paddle counts carry on across reconnects so they never go backwards.
        paddles: shared.0.lock().unwrap().paddles,
        ..default()
    };
    let mut has_steering = false;
    for (code, info) in device.get_absinfo()? {
        match code {
            STEERING_AXIS => {
                state.steering = AxisState::from_absinfo(&info);
                has_steering = true;
            }
            THROTTLE_AXIS => state.throttle = AxisState::from_absinfo(&info),
            BRAKE_AXIS => state.brake = AxisState::from_absinfo(&info),
            _ => {}
        }
    }
    if !has_steering {
        return Err(std::io::Error::other("steering axis not found"));
    }

    info!(
        "Steering wheel connected: {} (steering {}..{})",
        state.device_name.as_deref().unwrap_or("unknown"),
        state.steering.min,
        state.steering.max
    );
    *shared.0.lock().unwrap() = state;

    loop {
        for event in device.fetch_events()? {
            let mut state = shared.0.lock().unwrap();
            match event.destructure() {
                EventSummary::AbsoluteAxis(_, code, value) => match code {
                    STEERING_AXIS => state.steering.value = value,
                    THROTTLE_AXIS => state.throttle.value = value,
                    BRAKE_AXIS => state.brake.value = value,
                    _ => {}
                },
                // A value of 1 is a press; 0 is a release and 2 a repeat.
                EventSummary::Key(_, UPSHIFT_BUTTON, 1) => state.paddles.upshifts += 1,
                EventSummary::Key(_, DOWNSHIFT_BUTTON, 1) => state.paddles.downshifts += 1,
                _ => {}
            }
        }
    }
}
