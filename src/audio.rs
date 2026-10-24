//! Engine sound output.
//!
//! The engine itself runs in the vehicle physics (it is coupled to the
//! wheels through the gearbox). After each physics step its speed, load and
//! throttle are passed to the audio callback through atomics, where the
//! sound is synthesized sample by sample with the values smoothed so the
//! 1 kHz updates don't step audibly.

use std::error::Error;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::thread;

use bevy::prelude::*;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, SampleFormat, SizedSample, StreamConfig};

use crate::car::{Car, SelectedCar};
use crate::engine_sound::{EngineSound, SoundInputs};
use crate::vehicle::{Vehicle, step_vehicle};

/// Smoothing applied in the audio thread to values from the physics, s.
const PARAMETER_SMOOTHING: f32 = 0.008;

pub struct EngineAudioPlugin;

impl Plugin for EngineAudioPlugin {
    fn build(&self, app: &mut App) {
        let audio = EngineAudio::default();
        spawn_audio_thread(audio.clone());

        app.insert_resource(audio)
            .add_systems(FixedUpdate, send_engine_state.after(step_vehicle))
            .add_systems(Update, (send_selected_car, toggle_mute));
    }
}

/// An f32 shared between threads.
#[derive(Clone)]
struct SharedValue(Arc<AtomicU32>);

impl SharedValue {
    fn new(value: f32) -> Self {
        Self(Arc::new(AtomicU32::new(value.to_bits())))
    }

    fn get(&self) -> f32 {
        f32::from_bits(self.0.load(Ordering::Relaxed))
    }

    fn set(&self, value: f32) {
        self.0.store(value.to_bits(), Ordering::Relaxed);
    }
}

/// Engine state shared with the audio thread.
#[derive(Resource, Clone)]
pub struct EngineAudio {
    rpm: SharedValue,
    load: SharedValue,
    throttle: SharedValue,
    speed: SharedValue,
    /// Index into `Car::ALL` of the car whose engine to play.
    car: Arc<AtomicUsize>,
    muted: Arc<AtomicBool>,
}

impl Default for EngineAudio {
    fn default() -> Self {
        Self {
            rpm: SharedValue::new(Car::default().spec().engine.idle_rpm),
            load: SharedValue::new(0.0),
            throttle: SharedValue::new(0.0),
            speed: SharedValue::new(0.0),
            car: Arc::new(AtomicUsize::new(0)),
            muted: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl EngineAudio {
    pub fn muted(&self) -> bool {
        self.muted.load(Ordering::Relaxed)
    }
}

fn send_selected_car(selected: Res<SelectedCar>, audio: Res<EngineAudio>) {
    let index = Car::ALL
        .iter()
        .position(|&car| car == selected.0)
        .unwrap_or(0);
    audio.car.store(index, Ordering::Relaxed);
}

fn send_engine_state(vehicle: Res<Vehicle>, audio: Res<EngineAudio>) {
    let engine = &vehicle.powertrain.engine;
    audio.rpm.set(engine.rpm);
    audio.load.set(engine.load);
    audio.throttle.set(engine.throttle);
    audio.speed.set(vehicle.velocity.length());
}

/// M mutes and unmutes the engine.
fn toggle_mute(keys: Res<ButtonInput<KeyCode>>, audio: Res<EngineAudio>) {
    if keys.just_pressed(KeyCode::KeyM) {
        audio.muted.fetch_xor(true, Ordering::Relaxed);
    }
}

fn spawn_audio_thread(audio: EngineAudio) {
    thread::Builder::new()
        .name("engine-audio".into())
        .spawn(move || match start_stream(audio) {
            // The stream plays for as long as it is alive, so keep it here.
            Ok(_stream) => loop {
                thread::park();
            },
            Err(err) => warn!("Engine sound unavailable: {err}"),
        })
        .expect("failed to spawn audio thread");
}

fn start_stream(audio: EngineAudio) -> Result<cpal::Stream, Box<dyn Error>> {
    let device = cpal::default_host()
        .default_output_device()
        .ok_or("no audio output device")?;
    let supported = device.default_output_config()?;
    let config = supported.config();
    let stream = match supported.sample_format() {
        SampleFormat::F32 => build_stream::<f32>(&device, config, audio)?,
        SampleFormat::I16 => build_stream::<i16>(&device, config, audio)?,
        SampleFormat::U16 => build_stream::<u16>(&device, config, audio)?,
        other => return Err(format!("unsupported sample format {other}").into()),
    };
    stream.play()?;
    info!(
        "Engine sound playing at {} Hz, {} channel(s)",
        supported.sample_rate(),
        supported.channels()
    );
    Ok(stream)
}

fn build_stream<T: SizedSample + FromSample<f32>>(
    device: &cpal::Device,
    config: StreamConfig,
    audio: EngineAudio,
) -> Result<cpal::Stream, Box<dyn Error>> {
    let channels = config.channels as usize;
    let sample_rate = config.sample_rate as f32;
    let smoothing = 1.0 - (-1.0 / (sample_rate * PARAMETER_SMOOTHING)).exp();
    let mut car = audio.car.load(Ordering::Relaxed);
    let mut sound = EngineSound::new(sample_rate, Car::ALL[car].spec().sound);
    let mut smoothed = SoundInputs {
        rpm: Car::ALL[car].spec().engine.idle_rpm,
        ..SoundInputs::default()
    };

    let stream = device.build_output_stream(
        config,
        move |data: &mut [T], _| {
            // A different car means a different engine and exhaust.
            let selected = audio.car.load(Ordering::Relaxed);
            if selected != car {
                car = selected;
                sound = EngineSound::new(sample_rate, Car::ALL[car].spec().sound);
            }
            let target = SoundInputs {
                rpm: audio.rpm.get(),
                load: audio.load.get(),
                throttle: audio.throttle.get(),
                speed: audio.speed.get(),
            };
            let muted = audio.muted();
            for frame in data.chunks_mut(channels) {
                smoothed.rpm += (target.rpm - smoothed.rpm) * smoothing;
                smoothed.load += (target.load - smoothed.load) * smoothing;
                smoothed.throttle += (target.throttle - smoothed.throttle) * smoothing;
                smoothed.speed += (target.speed - smoothed.speed) * smoothing;
                let sample = sound.sample(smoothed);
                frame.fill(T::from_sample(if muted { 0.0 } else { sample }));
            }
        },
        |err| warn!("Engine sound stream error: {err}"),
        None,
    )?;
    Ok(stream)
}
