//! Procedural engine sound for the BRZ / GR86's flat-four, as heard from the
//! driver's seat, built as source, transfer paths and receiver.
//!
//! Source: each cylinder's exhaust valve opens and releases a pressure
//! pulse whose strength follows the burnt charge, with the cycle-to-cycle
//! variation, spark timing scatter, crank speed ripple and idle speed drift
//! of a real engine. Gas temperature follows load with a lag and tunes the
//! exhaust. Intake valves draw noisy pulses through the airbox; injectors
//! and valvetrain tick.
//!
//! Paths: the pulses travel the exhaust waveguide network (`exhaust`), and
//! every source then takes its own path into the cabin (`cabin`), where the
//! cabin resonances, reverberation and road and wind noise complete it. As
//! in the BRZ / GR86, Active Sound Control adds crank-locked engine orders through
//! the cabin speakers, mostly at high revs under throttle.

mod cabin;
mod exhaust;
mod filters;
mod waveguide;

use std::f32::consts::TAU;

use cabin::{Cabin, CabinInputs};
use exhaust::Exhaust;
use filters::{Biquad, Noise, OnePoleLowPass, smoothing_coefficient};

/// Firing points in the 720 degree cycle (fraction), firing order 1-3-2-4
/// evenly spaced, with tiny per-cylinder differences.
const FIRING_POINTS: [f32; 4] = [0.0, 0.2503, 0.4998, 0.7502];
/// Relative pulse strength per cylinder: equal-length headers and good
/// balance keep these close.
const CYLINDER_GAIN: [f32; 4] = [1.0, 0.96, 0.98, 0.95];
/// Exhaust valve opens and closes, as a fraction of the cycle after firing
/// (about 130 and 380 crank degrees).
const EXHAUST_OPENS: f32 = 0.18;
const EXHAUST_CLOSES: f32 = 0.53;
/// Intake valve open window, fraction of the cycle after firing.
const INTAKE_OPENS: f32 = 0.5;
const INTAKE_CLOSES: f32 = 0.85;
/// Spark timing scatter, fraction of the cycle (about 0.7 crank degrees).
const SPARK_JITTER: f32 = 0.001;
/// Point in each cylinder's cycle, clear of all its events, where the next
/// spark scatter is drawn, so the small phase jump can't re-trigger one.
const SPARK_JITTER_UPDATE: f32 = 0.95;

/// Cycle-to-cycle variation of each cylinder's charge: correlation from one
/// cycle to the next, and spread at light load and at full load.
const CHARGE_CORRELATION: f32 = 0.4;
const CHARGE_SPREAD: (f32, f32) = (0.08, 0.025);
/// Pulse strength with fuel cut, relative to a full charge: only pumping.
const PUMPING_PULSE: f32 = 0.15;
/// Exhaust pulse length as a fraction of the time between firings, and limits (s).
const PULSE_LENGTH: f32 = 0.3;
const PULSE_LENGTH_LIMITS: (f32, f32) = (0.0004, 0.003);
/// Each pulse carries out the burnt charge, so a shorter pulse is taller.
/// Heights are relative to a pulse of this length (s).
const REFERENCE_PULSE_LENGTH: f32 = 0.003;
/// Pulse rise time as a fraction of its length, at light and full load:
/// a full charge blows down more abruptly.
const PULSE_RISE: (f32, f32) = (0.15, 0.06);
/// Turbulent noise riding on each pulse.
const PULSE_TURBULENCE: f32 = 0.3;
/// Steady airflow noise through the exhaust on overrun.
const OVERRUN_AIRFLOW: f32 = 0.02;

/// Crank speed ripple at the firing frequency, at idle; it shrinks with rpm squared.
const CRANK_RIPPLE: f32 = 0.012;
const IDLE_RPM: f32 = 750.0;
/// Idle speed wander: rpm at idle plus a fraction of engine speed, and its bandwidth (Hz).
const RPM_DRIFT: (f32, f32) = (5.0, 0.0008);
const RPM_DRIFT_BANDWIDTH: f32 = 2.0;

/// Header gas temperature, K: base, rise at full load, rise per rpm.
const GAS_TEMPERATURE: (f32, f32, f32) = (600.0, 450.0, 0.035);
/// Time constant with which gas temperature follows load, s.
const GAS_TEMPERATURE_LAG: f32 = 1.5;

/// Intake: airbox (Helmholtz) and runner resonances (Hz, Q), level.
const AIRBOX: (f32, f32) = (180.0, 4.0);
const INTAKE_RUNNER: (f32, f32) = (600.0, 2.0);
const INTAKE_LEVEL: f32 = 0.5;

/// Injector and valvetrain ticks: resonances (Hz, Q) and level.
const TICK_RESONANCES: [(f32, f32); 2] = [(3500.0, 10.0), (5500.0, 12.0)];
const TICK_LEVEL: f32 = 0.004;

/// Active Sound Control: engine orders and their levels.
const ASC_ORDERS: [(f32, f32); 3] = [(2.0, 1.0), (4.0, 0.4), (6.0, 0.15)];
/// ASC level rises from this rpm to the next, scaled by throttle.
const ASC_RPM_RANGE: (f32, f32) = (2500.0, 7000.0);
const ASC_LEVEL: f32 = 0.08;
const ASC_LAG: f32 = 0.05;

/// Torque pulses reach the mounts in proportion to the charge burnt.
const STRUCTURE_LEVEL: f32 = 1.0;

const MASTER_GAIN: f32 = 0.8;

/// Engine and vehicle state the sound depends on, for one sample.
#[derive(Clone, Copy, Debug, Default)]
pub struct SoundInputs {
    pub rpm: f32,
    /// Fraction of a full cylinder charge burnt, 0..=1 (0 with fuel cut).
    pub load: f32,
    /// Throttle plate opening, 0..=1.
    pub throttle: f32,
    /// Vehicle speed, m/s.
    pub speed: f32,
}

pub struct EngineSound {
    sample_rate: f32,
    /// Position in the 720 degree four-stroke cycle, 0..1.
    cycle: f32,
    /// Each cylinder's phase after its firing point, last sample.
    previous_phase: [f32; 4],
    /// Each cylinder's current charge variation and spark scatter.
    charge_variation: [f32; 4],
    spark_offset: [f32; 4],
    /// Rising and falling envelopes whose difference shapes each pulse.
    pulse_fast: [f32; 4],
    pulse_slow: [f32; 4],
    rpm_drift: OnePoleLowPass,
    load_for_temperature: f32,
    asc_level: f32,
    airbox: Biquad,
    intake_runner: Biquad,
    ticks: [Biquad; 2],
    exhaust: Exhaust,
    cabin: Cabin,
    noise: Noise,
}

impl EngineSound {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            sample_rate,
            cycle: 0.0,
            previous_phase: [0.0; 4],
            charge_variation: [0.0; 4],
            spark_offset: [0.0; 4],
            pulse_fast: [0.0; 4],
            pulse_slow: [0.0; 4],
            rpm_drift: OnePoleLowPass::new(sample_rate, RPM_DRIFT_BANDWIDTH),
            load_for_temperature: 0.0,
            asc_level: 0.0,
            airbox: Biquad::band_pass(sample_rate, AIRBOX.0, AIRBOX.1),
            intake_runner: Biquad::band_pass(sample_rate, INTAKE_RUNNER.0, INTAKE_RUNNER.1),
            ticks: TICK_RESONANCES.map(|(hz, q)| Biquad::band_pass(sample_rate, hz, q)),
            exhaust: Exhaust::new(sample_rate),
            cabin: Cabin::new(sample_rate),
            noise: Noise::new(0x2545_f491),
        }
    }

    /// Produces the next sample (about -1..1).
    pub fn sample(&mut self, inputs: SoundInputs) -> f32 {
        let dt = 1.0 / self.sample_rate;
        let SoundInputs {
            rpm,
            load,
            throttle,
            speed,
        } = inputs;

        // The crank never turns perfectly steadily: the idle speed wanders,
        // and each firing kicks it forward a little.
        // A 2 Hz one-pole low-pass keeps about 1/88 of unit noise's spread
        // at 48 kHz; scale back up to unit size.
        let wander = self.rpm_drift.process(self.noise.gaussian()) * 88.0;
        let rpm = (rpm + wander * (RPM_DRIFT.0 + RPM_DRIFT.1 * rpm)).max(100.0);
        let ripple = (CRANK_RIPPLE * (IDLE_RPM / rpm).powi(2)).min(0.02);
        let ripple = 1.0 + ripple * (TAU * 4.0 * self.cycle).sin();
        self.cycle = (self.cycle + rpm / 120.0 * dt * ripple).fract();

        let firing_rate = rpm / 30.0;
        let length =
            (PULSE_LENGTH / firing_rate).clamp(PULSE_LENGTH_LIMITS.0, PULSE_LENGTH_LIMITS.1);
        let rise = length * (PULSE_RISE.0 + (PULSE_RISE.1 - PULSE_RISE.0) * load);
        let slow_decay = (-dt / length).exp();
        let fast_decay = (-dt / rise).exp();

        let mut pulses = [0.0; 4];
        let mut valves_open = [false; 4];
        let mut intake_open = 0.0;
        let mut tick = 0.0;
        let mut torque = 0.0;
        for cylinder in 0..4 {
            let phase = (self.cycle - FIRING_POINTS[cylinder] - self.spark_offset[cylinder])
                .rem_euclid(1.0);
            let previous = self.previous_phase[cylinder];
            self.previous_phase[cylinder] = phase;
            let crossed = |at: f32| previous < at && phase >= at;

            if crossed(EXHAUST_OPENS) {
                self.blow_down(cylinder, load, REFERENCE_PULSE_LENGTH / length);
                tick += 1.0;
            }
            if crossed(INTAKE_OPENS) {
                // The direct injector fires for the next charge.
                tick += 0.7;
            }
            if crossed(SPARK_JITTER_UPDATE) {
                self.spark_offset[cylinder] = SPARK_JITTER * self.noise.gaussian();
            }

            self.pulse_slow[cylinder] *= slow_decay;
            self.pulse_fast[cylinder] *= fast_decay;
            let pulse = self.pulse_slow[cylinder] - self.pulse_fast[cylinder];
            torque += pulse;
            valves_open[cylinder] = (EXHAUST_OPENS..EXHAUST_CLOSES).contains(&phase);
            let mut wave = pulse * (1.0 + PULSE_TURBULENCE * self.noise.white());
            if load == 0.0 && valves_open[cylinder] {
                wave += OVERRUN_AIRFLOW * (rpm / 7000.0) * self.noise.white();
            }
            pulses[cylinder] = wave;
            if (INTAKE_OPENS..INTAKE_CLOSES).contains(&phase) {
                intake_open += 1.0;
            }
        }

        // Hotter gas at higher load and revs speeds up sound in the exhaust.
        self.load_for_temperature += (load - self.load_for_temperature)
            * smoothing_coefficient(self.sample_rate, GAS_TEMPERATURE_LAG);
        self.exhaust.set_gas_temperature(
            GAS_TEMPERATURE.0
                + GAS_TEMPERATURE.1 * self.load_for_temperature
                + GAS_TEMPERATURE.2 * rpm,
        );
        let exhaust = self.exhaust.process(pulses, valves_open);

        let suction = intake_open * self.noise.white() * (0.2 + throttle);
        let intake = INTAKE_LEVEL
            * throttle.powf(0.7)
            * (rpm / 7000.0)
            * (self.airbox.process(suction) + 0.5 * self.intake_runner.process(suction));

        let mechanical = TICK_LEVEL
            * self
                .ticks
                .iter_mut()
                .map(|resonance| resonance.process(tick))
                .sum::<f32>();

        let asc_target = throttle
            * ((rpm - ASC_RPM_RANGE.0) / (ASC_RPM_RANGE.1 - ASC_RPM_RANGE.0)).clamp(0.0, 1.0);
        self.asc_level +=
            (asc_target - self.asc_level) * smoothing_coefficient(self.sample_rate, ASC_LAG);
        let asc = ASC_LEVEL
            * self.asc_level
            * ASC_ORDERS
                .iter()
                .map(|&(order, level)| level * (TAU * 2.0 * order * self.cycle).sin())
                .sum::<f32>();

        let cabin = self.cabin.process(CabinInputs {
            exhaust,
            intake,
            structure: STRUCTURE_LEVEL * torque,
            mechanical,
            asc,
            speed,
        });
        (cabin * MASTER_GAIN).tanh()
    }

    /// The exhaust valve of `cylinder` opens: release a pulse sized by the
    /// charge burnt this cycle and scaled by `height` for its length, and set
    /// up the next cycle's variation.
    fn blow_down(&mut self, cylinder: usize, load: f32, height: f32) {
        let spread = CHARGE_SPREAD.0 + (CHARGE_SPREAD.1 - CHARGE_SPREAD.0) * load;
        let variation = &mut self.charge_variation[cylinder];
        *variation = CHARGE_CORRELATION * *variation + spread * self.noise.gaussian();
        let charge = PUMPING_PULSE + (1.0 - PUMPING_PULSE) * load;
        let strength =
            CYLINDER_GAIN[cylinder] * charge * height * (1.0 + self.charge_variation[cylinder]);
        self.pulse_slow[cylinder] += strength;
        self.pulse_fast[cylinder] += strength;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::Surface;
    use crate::vehicle::{Controls, Vehicle, VehicleParams};

    const RATE: f32 = 48_000.0;

    fn steady(rpm: f32, load: f32, speed: f32) -> SoundInputs {
        SoundInputs {
            rpm,
            load,
            throttle: load,
            speed,
        }
    }

    fn render(inputs: SoundInputs, seconds: f32) -> Vec<f32> {
        let mut sound = EngineSound::new(RATE);
        // Let the gas temperature and filters settle before recording.
        for _ in 0..(RATE * 3.0) as usize {
            sound.sample(inputs);
        }
        (0..(RATE * seconds) as usize)
            .map(|_| sound.sample(inputs))
            .collect()
    }

    fn rms(samples: &[f32]) -> f32 {
        (samples.iter().map(|x| x * x).sum::<f32>() / samples.len() as f32).sqrt()
    }

    /// Magnitude at `frequency` (Goertzel algorithm).
    fn magnitude_at(samples: &[f32], frequency: f32) -> f32 {
        let coefficient = 2.0 * (TAU * frequency / RATE).cos();
        let (mut s1, mut s2) = (0.0, 0.0);
        for &x in samples {
            let s = x + coefficient * s1 - s2;
            s2 = s1;
            s1 = s;
        }
        (s1 * s1 + s2 * s2 - coefficient * s1 * s2).max(0.0).sqrt() / samples.len() as f32
    }

    #[test]
    fn output_is_finite_and_bounded() {
        for inputs in [
            steady(750.0, 0.05, 0.0),
            steady(3000.0, 0.5, 15.0),
            steady(7400.0, 1.0, 40.0),
            steady(6000.0, 0.0, 30.0),
        ] {
            let samples = render(inputs, 1.0);
            assert!(samples.iter().all(|x| x.is_finite() && x.abs() <= 1.0));
            assert!(rms(&samples) > 0.01, "silent for {inputs:?}");
        }
    }

    #[test]
    fn engine_sound_follows_the_firing_frequency() {
        // A four-stroke four fires twice per revolution, so its sound is a
        // harmonic series on rpm / 30 Hz.
        let rpm = 3000.0;
        let firing = rpm / 30.0;
        let samples = render(steady(rpm, 0.6, 0.0), 2.0);
        let harmonic: f32 = (1..=4)
            .map(|k| magnitude_at(&samples, firing * k as f32))
            .sum();
        let between: f32 = [1.37, 2.41, 3.29, 4.63]
            .iter()
            .map(|k| magnitude_at(&samples, firing * k))
            .sum();
        assert!(
            harmonic > 3.0 * between,
            "harmonic {harmonic}, between {between}"
        );
    }

    #[test]
    fn full_load_is_louder_than_overrun() {
        let loud = rms(&render(steady(5000.0, 1.0, 0.0), 1.0));
        let quiet = rms(&render(steady(5000.0, 0.0, 0.0), 1.0));
        assert!(loud > 2.0 * quiet, "full load {loud}, overrun {quiet}");
    }

    #[test]
    fn no_two_engine_cycles_are_identical() {
        // Combustion varies from cycle to cycle, so successive cycles differ.
        let rpm = 1500.0;
        let samples = render(steady(rpm, 0.3, 0.0), 1.0);
        let cycle = (RATE * 120.0 / rpm) as usize;
        let difference = rms(&samples[..cycle]
            .iter()
            .zip(&samples[cycle..2 * cycle])
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>());
        assert!(difference > 0.05 * rms(&samples), "cycles too alike");
    }

    #[test]
    fn road_and_wind_noise_grow_with_speed() {
        let parked = rms(&render(steady(800.0, 0.05, 0.0), 1.0));
        let fast = rms(&render(steady(800.0, 0.05, 40.0), 1.0));
        assert!(fast > 1.5 * parked, "parked {parked}, 144 km/h {fast}");
    }

    /// Writes a WAV of a drive to $ENGINE_PREVIEW (a file path): idle, a
    /// full-throttle pull through the gears on the paddles, braking with
    /// downshifts, then cruising.
    /// Run with `ENGINE_PREVIEW=engine.wav cargo test render_preview -- --ignored`.
    #[test]
    #[ignore]
    fn render_preview() {
        let path = std::env::var("ENGINE_PREVIEW").expect("set ENGINE_PREVIEW to a .wav path");
        let params = VehicleParams::default();
        let mut vehicle = Vehicle::default();
        let mut sound = EngineSound::new(RATE);
        let smoothing = smoothing_coefficient(RATE, 0.008);
        let mut smoothed = SoundInputs {
            rpm: 750.0,
            ..SoundInputs::default()
        };
        let physics_steps_per_sample = 1000.0 / RATE;
        let mut physics_time = 0.0;
        let mut samples = Vec::new();
        let mut shift_cooldown = 0.0;
        for i in 0..(RATE * 22.0) as usize {
            let t = i as f32 / RATE;
            physics_time += physics_steps_per_sample;
            while physics_time >= 1.0 {
                physics_time -= 1.0;
                let rpm = vehicle.powertrain.engine.rpm;
                let gear = vehicle.powertrain.gear;
                shift_cooldown -= 0.001;
                let controls = match t {
                    t if t < 1.5 => Controls {
                        brake: 1.0,
                        ..Controls::default()
                    },
                    t if t < 12.0 => {
                        if rpm > 7000.0 && gear < 3 && shift_cooldown <= 0.0 {
                            vehicle.powertrain.request_shift(1);
                            shift_cooldown = 0.5;
                        }
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        }
                    }
                    t if t < 16.0 => {
                        if rpm < 3200.0 && gear > 1 && shift_cooldown <= 0.0 {
                            vehicle.powertrain.request_shift(-1);
                            shift_cooldown = 0.6;
                        }
                        Controls {
                            brake: 0.5,
                            ..Controls::default()
                        }
                    }
                    _ => {
                        let speed = vehicle.speed();
                        Controls {
                            throttle: ((60.0 / 3.6 - speed) * 0.3).clamp(0.0, 0.35),
                            ..Controls::default()
                        }
                    }
                };
                vehicle.integrate(&params, controls, [Surface::ASPHALT; 4], 0.001);
            }
            let engine = &vehicle.powertrain.engine;
            smoothed.rpm += (engine.rpm - smoothed.rpm) * smoothing;
            smoothed.load += (engine.load - smoothed.load) * smoothing;
            smoothed.throttle += (engine.throttle - smoothed.throttle) * smoothing;
            smoothed.speed += (vehicle.velocity.length() - smoothed.speed) * smoothing;
            samples.push(sound.sample(smoothed));
        }
        write_wav(&path, &samples, RATE as u32).unwrap();
    }

    fn write_wav(path: &str, samples: &[f32], rate: u32) -> std::io::Result<()> {
        let data_len = (samples.len() * 2) as u32;
        let mut bytes = Vec::with_capacity(44 + data_len as usize);
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36 + data_len).to_le_bytes());
        bytes.extend_from_slice(b"WAVEfmt ");
        bytes.extend_from_slice(&16u32.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes()); // PCM
        bytes.extend_from_slice(&1u16.to_le_bytes()); // mono
        bytes.extend_from_slice(&rate.to_le_bytes());
        bytes.extend_from_slice(&(rate * 2).to_le_bytes());
        bytes.extend_from_slice(&2u16.to_le_bytes());
        bytes.extend_from_slice(&16u16.to_le_bytes());
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&data_len.to_le_bytes());
        for &s in samples {
            bytes.extend_from_slice(&((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16).to_le_bytes());
        }
        std::fs::write(path, bytes)
    }
}
