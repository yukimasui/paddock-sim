//! Procedural engine sound as heard from the driver's seat, built as
//! source, transfer paths and receiver, and parameterised per car by a
//! [`SoundSpec`].
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
//! cabin resonances, reverberation and road and wind noise complete it. Cars
//! with Active Sound Control (the BRZ / GR86) add crank-locked engine orders
//! through the cabin speakers, mostly at high revs under throttle.

mod cabin;
mod exhaust;
mod filters;
mod waveguide;

use std::f32::consts::TAU;

use cabin::{Cabin, CabinInputs, CabinSpec};
use exhaust::{Exhaust, ExhaustSpec, Section};
use filters::{Biquad, Noise, OnePoleLowPass, smoothing_coefficient};

/// One cylinder, listed in firing order.
#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
    /// Firing point in the 720 degree cycle, as a fraction.
    pub fires_at: f32,
    /// Exhaust bank (collector) the cylinder feeds.
    pub bank: usize,
    /// Relative pulse strength.
    pub gain: f32,
}

/// What distinguishes one car's engine sound from another's.
#[derive(Debug)]
pub struct SoundSpec {
    pub cylinders: &'static [Cylinder],
    pub idle_rpm: f32,
    pub exhaust: ExhaustSpec,
    /// Intake airbox (Helmholtz) and runner resonances (Hz, Q).
    pub airbox: (f32, f32),
    pub intake_runner: (f32, f32),
    pub intake_level: f32,
    /// Torque pulses reach the mounts in proportion to the charge burnt.
    pub structure_level: f32,
    pub cabin: CabinSpec,
}

/// Subaru BRZ / Toyota GR86 FA24 flat-four, firing order 1-3-2-4, with
/// equal-length headers into one collector and Active Sound Control.
pub static FA24_SOUND: SoundSpec = SoundSpec {
    cylinders: &[
        Cylinder {
            fires_at: 0.0,
            bank: 0,
            gain: 1.0,
        },
        Cylinder {
            fires_at: 0.2503,
            bank: 0,
            gain: 0.96,
        },
        Cylinder {
            fires_at: 0.4998,
            bank: 0,
            gain: 0.98,
        },
        Cylinder {
            fires_at: 0.7502,
            bank: 0,
            gain: 0.95,
        },
    ],
    idle_rpm: 750.0,
    exhaust: ExhaustSpec {
        runner: Section {
            length: 0.45,
            area: 1.0,
            loss: 0.997,
            damping_hz: 10_000.0,
        },
        runner_detune: &[1.0, 1.02, 0.985, 1.012],
        collector: Section {
            length: 0.3,
            area: 2.2,
            loss: 0.995,
            damping_hz: 9000.0,
        },
        center_pipe: Section {
            length: 1.5,
            area: 1.8,
            loss: 0.993,
            damping_hz: 7000.0,
        },
        first_chamber: Section {
            length: 0.35,
            area: 14.0,
            loss: 0.9,
            damping_hz: 2500.0,
        },
        link_pipe: Section {
            length: 0.5,
            area: 1.8,
            loss: 0.993,
            damping_hz: 6000.0,
        },
        second_chamber: Section {
            length: 0.3,
            area: 10.0,
            loss: 0.92,
            damping_hz: 3000.0,
        },
        tailpipe: Section {
            length: 0.7,
            area: 1.8,
            loss: 0.995,
            damping_hz: 8000.0,
        },
    },
    airbox: (180.0, 4.0),
    intake_runner: (600.0, 2.0),
    intake_level: 0.5,
    structure_level: 1.0,
    cabin: CabinSpec {
        exhaust_gain: 180.0,
        body_transmission: (180.0, 0.7),
        exhaust_leak: (0.01, 1500.0),
        intake_gain: 4.5,
        intake_transmission: (350.0, 0.7),
        structure_gain: 1.2,
        mount_transmission: (150.0, 1.2),
        mechanical_gain: 40.0,
        asc_gain: 1.4,
    },
};

/// Lexus LFA 1LR-GUE V10, firing order 1-6-5-10-2-7-3-8-4-9 every 72
/// degrees, alternating between the banks. Each bank's five equal-length
/// runners meet in their own collector, both collectors merge, and the
/// titanium muffler is free-flowing. Yamaha's sound channel pipes intake
/// noise straight into the cabin; there is no Active Sound Control.
pub static ONE_LR_GUE_SOUND: SoundSpec = SoundSpec {
    cylinders: &[
        Cylinder {
            fires_at: 0.0,
            bank: 0,
            gain: 1.0,
        },
        Cylinder {
            fires_at: 0.1002,
            bank: 1,
            gain: 0.98,
        },
        Cylinder {
            fires_at: 0.1999,
            bank: 0,
            gain: 0.99,
        },
        Cylinder {
            fires_at: 0.3001,
            bank: 1,
            gain: 0.97,
        },
        Cylinder {
            fires_at: 0.4,
            bank: 0,
            gain: 1.0,
        },
        Cylinder {
            fires_at: 0.4998,
            bank: 1,
            gain: 0.99,
        },
        Cylinder {
            fires_at: 0.6001,
            bank: 0,
            gain: 0.98,
        },
        Cylinder {
            fires_at: 0.7,
            bank: 1,
            gain: 1.0,
        },
        Cylinder {
            fires_at: 0.7999,
            bank: 0,
            gain: 0.97,
        },
        Cylinder {
            fires_at: 0.9002,
            bank: 1,
            gain: 0.99,
        },
    ],
    idle_rpm: 950.0,
    exhaust: ExhaustSpec {
        runner: Section {
            length: 0.75,
            area: 1.0,
            loss: 0.997,
            damping_hz: 12_000.0,
        },
        runner_detune: &[1.0, 1.01, 0.99, 1.015, 0.995, 1.005, 0.985, 1.0, 1.01, 0.99],
        collector: Section {
            length: 0.35,
            area: 2.4,
            loss: 0.995,
            damping_hz: 9000.0,
        },
        center_pipe: Section {
            length: 1.2,
            area: 2.6,
            loss: 0.994,
            damping_hz: 8000.0,
        },
        first_chamber: Section {
            length: 0.3,
            area: 7.0,
            loss: 0.95,
            damping_hz: 4500.0,
        },
        link_pipe: Section {
            length: 0.4,
            area: 2.6,
            loss: 0.994,
            damping_hz: 7000.0,
        },
        second_chamber: Section {
            length: 0.25,
            area: 6.0,
            loss: 0.95,
            damping_hz: 5000.0,
        },
        tailpipe: Section {
            length: 0.45,
            area: 2.6,
            loss: 0.996,
            damping_hz: 9000.0,
        },
    },
    airbox: (260.0, 3.0),
    intake_runner: (900.0, 2.0),
    intake_level: 0.8,
    structure_level: 0.6,
    cabin: CabinSpec {
        exhaust_gain: 180.0,
        body_transmission: (180.0, 0.7),
        exhaust_leak: (0.01, 1500.0),
        intake_gain: 0.8,
        intake_transmission: (1500.0, 0.9),
        structure_gain: 1.2,
        mount_transmission: (150.0, 1.2),
        mechanical_gain: 30.0,
        asc_gain: 0.0,
    },
};

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
/// Idle speed wander: rpm at idle plus a fraction of engine speed, and its bandwidth (Hz).
const RPM_DRIFT: (f32, f32) = (5.0, 0.0008);
const RPM_DRIFT_BANDWIDTH: f32 = 2.0;

/// Header gas temperature, K: base, rise at full load, rise per rpm.
const GAS_TEMPERATURE: (f32, f32, f32) = (600.0, 450.0, 0.035);
/// Time constant with which gas temperature follows load, s.
const GAS_TEMPERATURE_LAG: f32 = 1.5;

/// Injector and valvetrain ticks: resonances (Hz, Q) and level.
const TICK_RESONANCES: [(f32, f32); 2] = [(3500.0, 10.0), (5500.0, 12.0)];
const TICK_LEVEL: f32 = 0.004;

/// Active Sound Control: engine orders and their levels.
const ASC_ORDERS: [(f32, f32); 3] = [(2.0, 1.0), (4.0, 0.4), (6.0, 0.15)];
/// ASC level rises from this rpm to the next, scaled by throttle.
const ASC_RPM_RANGE: (f32, f32) = (2500.0, 7000.0);
const ASC_LEVEL: f32 = 0.08;
const ASC_LAG: f32 = 0.05;

const MASTER_GAIN: f32 = 0.45;

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

/// Per-cylinder state.
#[derive(Clone, Copy, Default)]
struct CylinderState {
    /// Phase after the firing point, last sample.
    previous_phase: f32,
    /// Current charge variation and spark scatter.
    charge_variation: f32,
    spark_offset: f32,
    /// Rising and falling envelopes whose difference shapes each pulse.
    pulse_fast: f32,
    pulse_slow: f32,
}

pub struct EngineSound {
    spec: &'static SoundSpec,
    sample_rate: f32,
    /// Position in the 720 degree four-stroke cycle, 0..1.
    cycle: f32,
    cylinders: Vec<CylinderState>,
    /// Per-cylinder scratch for one sample.
    pulses: Vec<f32>,
    valves_open: Vec<bool>,
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
    pub fn new(sample_rate: f32, spec: &'static SoundSpec) -> Self {
        let count = spec.cylinders.len();
        let banks: Vec<usize> = spec.cylinders.iter().map(|c| c.bank).collect();
        Self {
            spec,
            sample_rate,
            cycle: 0.0,
            cylinders: vec![CylinderState::default(); count],
            pulses: vec![0.0; count],
            valves_open: vec![false; count],
            rpm_drift: OnePoleLowPass::new(sample_rate, RPM_DRIFT_BANDWIDTH),
            load_for_temperature: 0.0,
            asc_level: 0.0,
            airbox: Biquad::band_pass(sample_rate, spec.airbox.0, spec.airbox.1),
            intake_runner: Biquad::band_pass(
                sample_rate,
                spec.intake_runner.0,
                spec.intake_runner.1,
            ),
            ticks: TICK_RESONANCES.map(|(hz, q)| Biquad::band_pass(sample_rate, hz, q)),
            exhaust: Exhaust::new(sample_rate, &spec.exhaust, &banks),
            cabin: Cabin::new(sample_rate, &spec.cabin),
            noise: Noise::new(0x2545_f491),
        }
    }

    /// Produces the next sample (about -1..1).
    pub fn sample(&mut self, inputs: SoundInputs) -> f32 {
        let spec = self.spec;
        let dt = 1.0 / self.sample_rate;
        let SoundInputs {
            rpm,
            load,
            throttle,
            speed,
        } = inputs;
        let firings_per_cycle = spec.cylinders.len() as f32;

        // The crank never turns perfectly steadily: the idle speed wanders,
        // and each firing kicks it forward a little.
        // A 2 Hz one-pole low-pass keeps about 1/88 of unit noise's spread
        // at 48 kHz; scale back up to unit size.
        let wander = self.rpm_drift.process(self.noise.gaussian()) * 88.0;
        let rpm = (rpm + wander * (RPM_DRIFT.0 + RPM_DRIFT.1 * rpm)).max(100.0);
        let ripple = (CRANK_RIPPLE * (spec.idle_rpm / rpm).powi(2)).min(0.02);
        let ripple = 1.0 + ripple * (TAU * firings_per_cycle * self.cycle).sin();
        self.cycle = (self.cycle + rpm / 120.0 * dt * ripple).fract();

        let firing_rate = rpm / 120.0 * firings_per_cycle;
        let length =
            (PULSE_LENGTH / firing_rate).clamp(PULSE_LENGTH_LIMITS.0, PULSE_LENGTH_LIMITS.1);
        let rise = length * (PULSE_RISE.0 + (PULSE_RISE.1 - PULSE_RISE.0) * load);
        let slow_decay = (-dt / length).exp();
        let fast_decay = (-dt / rise).exp();

        let mut intake_open = 0.0;
        let mut tick = 0.0;
        let mut torque = 0.0;
        for (index, cylinder) in spec.cylinders.iter().enumerate() {
            let state = self.cylinders[index];
            let phase = (self.cycle - cylinder.fires_at - state.spark_offset).rem_euclid(1.0);
            let previous = state.previous_phase;
            self.cylinders[index].previous_phase = phase;
            let crossed = |at: f32| previous < at && phase >= at;

            if crossed(EXHAUST_OPENS) {
                self.blow_down(index, load, REFERENCE_PULSE_LENGTH / length);
                tick += 1.0;
            }
            if crossed(INTAKE_OPENS) {
                // The direct injector fires for the next charge.
                tick += 0.7;
            }
            if crossed(SPARK_JITTER_UPDATE) {
                self.cylinders[index].spark_offset = SPARK_JITTER * self.noise.gaussian();
            }

            let state = &mut self.cylinders[index];
            state.pulse_slow *= slow_decay;
            state.pulse_fast *= fast_decay;
            let pulse = state.pulse_slow - state.pulse_fast;
            torque += pulse;
            let open = (EXHAUST_OPENS..EXHAUST_CLOSES).contains(&phase);
            self.valves_open[index] = open;
            let mut wave = pulse * (1.0 + PULSE_TURBULENCE * self.noise.white());
            if load == 0.0 && open {
                wave += OVERRUN_AIRFLOW * (rpm / 7000.0) * self.noise.white();
            }
            self.pulses[index] = wave;
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
        let exhaust = self.exhaust.process(&self.pulses, &self.valves_open);

        let suction = intake_open * self.noise.white() * (0.2 + throttle);
        let intake = spec.intake_level
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
            structure: spec.structure_level * torque,
            mechanical,
            asc,
            speed,
        });
        (cabin * MASTER_GAIN).tanh()
    }

    /// The exhaust valve of cylinder `index` opens: release a pulse sized by
    /// the charge burnt this cycle and scaled by `height` for its length,
    /// and set up the next cycle's variation.
    fn blow_down(&mut self, index: usize, load: f32, height: f32) {
        let spread = CHARGE_SPREAD.0 + (CHARGE_SPREAD.1 - CHARGE_SPREAD.0) * load;
        let gaussian = self.noise.gaussian();
        let state = &mut self.cylinders[index];
        state.charge_variation = CHARGE_CORRELATION * state.charge_variation + spread * gaussian;
        let charge = PUMPING_PULSE + (1.0 - PUMPING_PULSE) * load;
        let strength =
            self.spec.cylinders[index].gain * charge * height * (1.0 + state.charge_variation);
        state.pulse_slow += strength;
        state.pulse_fast += strength;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::car::Car;
    use crate::powertrain::Powertrain;
    use crate::surface::Surface;
    use crate::vehicle::{Controls, Ground, Vehicle, VehicleParams};

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
        let mut sound = EngineSound::new(RATE, &FA24_SOUND);
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
        // Kept well below the engine, but always growing with speed.
        let at = |speed: f32| rms(&render(steady(800.0, 0.05, speed), 1.0));
        let (parked, town, motorway) = (at(0.0), at(60.0 / 3.6), at(144.0 / 3.6));
        assert!(
            parked < town && town < motorway && motorway > 1.3 * parked,
            "parked {parked}, 60 km/h {town}, 144 km/h {motorway}"
        );
    }

    /// Writes a WAV of a drive to $ENGINE_PREVIEW (a file path): idle, a
    /// full-throttle pull through the gears on the paddles, braking with
    /// downshifts, then cruising. Set $ENGINE_PREVIEW_CAR to `lfa` for the
    /// LFA instead of the BRZ.
    /// Run with `ENGINE_PREVIEW=engine.wav cargo test render_preview -- --ignored`.
    #[test]
    #[ignore]
    fn render_preview() {
        let path = std::env::var("ENGINE_PREVIEW").expect("set ENGINE_PREVIEW to a .wav path");
        let params = VehicleParams::default();
        let car = match std::env::var("ENGINE_PREVIEW_CAR").as_deref() {
            Ok("lfa") => Car::Lfa,
            _ => Car::Brz,
        }
        .spec();
        let mut vehicle = Vehicle {
            powertrain: Powertrain::new(car.engine, car.gearbox),
            ..Vehicle::default()
        };
        let mut sound = EngineSound::new(RATE, car.sound);
        let upshift_rpm = car.engine.redline_rpm - 400.0;
        let downshift_rpm = car.engine.redline_rpm * 0.45;
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
                        if rpm > upshift_rpm && gear < 3 && shift_cooldown <= 0.0 {
                            vehicle.powertrain.request_shift(1);
                            shift_cooldown = 0.5;
                        }
                        Controls {
                            throttle: 1.0,
                            ..Controls::default()
                        }
                    }
                    t if t < 16.0 => {
                        if rpm < downshift_rpm && gear > 1 && shift_cooldown <= 0.0 {
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
                vehicle.integrate(&params, controls, Ground::default(), 0.001);
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

    #[test]
    fn v10_sound_follows_its_firing_frequency() {
        // A V10 fires five times per revolution: harmonics of rpm / 12 Hz.
        let rpm = 4800.0;
        let firing = rpm / 12.0;
        let mut sound = EngineSound::new(RATE, &ONE_LR_GUE_SOUND);
        let inputs = steady(rpm, 0.6, 0.0);
        for _ in 0..(RATE * 3.0) as usize {
            sound.sample(inputs);
        }
        let samples: Vec<f32> = (0..(RATE * 2.0) as usize)
            .map(|_| sound.sample(inputs))
            .collect();
        assert!(samples.iter().all(|x| x.is_finite() && x.abs() <= 1.0));
        let harmonic: f32 = (1..=3)
            .map(|k| magnitude_at(&samples, firing * k as f32))
            .sum();
        let between: f32 = [1.37, 2.41, 2.63]
            .iter()
            .map(|k| magnitude_at(&samples, firing * k))
            .sum();
        assert!(
            harmonic > 3.0 * between,
            "harmonic {harmonic}, between {between}"
        );
    }
}
