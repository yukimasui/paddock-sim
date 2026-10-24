//! What reaches the driver's ears: each engine sound source through its own
//! transfer path into the cabin, plus road and wind noise, coloured by the
//! cabin's low resonances and a short reverberation.
//!
//! - Exhaust noise is airborne: the body blocks it more the higher its
//!   frequency, with a small leak through seals and glass.
//! - Intake noise comes through the firewall, muffled, unless the car pipes
//!   it into the cabin on purpose (as the LFA's sound channel does).
//! - Engine torque pulses shake the engine on its mounts, and the body
//!   radiates the vibration (structure-borne); this is the low boom.
//! - Mechanical ticks (injectors, valvetrain) leak through faintly.
//!
//! Below its first acoustic mode the closed cabin acts as a pressure
//! chamber: the panels compress the air inside rather than radiate into
//! open space, which lifts the bass (the "cabin gain" of car audio).
//!
//! The car's body is shared by every car for now, so the cabin itself, the
//! road and the wind are fixed here; only the engine-side paths are per car.

use super::filters::{Biquad, Noise, OnePoleHighPass, OnePoleLowPass};

/// How each engine source reaches the cabin of a particular car. Filters
/// are (cutoff Hz, Q) second-order low-passes.
#[derive(Debug)]
pub struct CabinSpec {
    pub exhaust_gain: f32,
    /// Body transmission of airborne exhaust noise. With the tailpipe's
    /// rising radiation it leaves the exhaust falling about 6 dB per octave
    /// above the cutoff.
    pub body_transmission: (f32, f32),
    /// Fraction of exhaust noise leaking through seals and glass, and the
    /// cutoff (Hz) of that leak.
    pub exhaust_leak: (f32, f32),
    pub intake_gain: f32,
    /// Firewall (or sound channel) transmission of intake noise.
    pub intake_transmission: (f32, f32),
    pub structure_gain: f32,
    /// Engine mounts and body panels pass the low part of the vibration,
    /// with a broad resonance near the top of that range.
    pub mount_transmission: (f32, f32),
    pub mechanical_gain: f32,
    /// Active Sound Control speaker level (zero for cars without it).
    pub asc_gain: f32,
}

/// Active Sound Control speaker band, Hz.
const ASC_BAND: (f32, f32) = (80.0, 600.0);
/// Cabin pressure-chamber bass lift: shelf corner (Hz) and gain (dB).
const CABIN_GAIN: (f32, f32) = (80.0, 10.0);
/// Cabin modes: (Hz, Q, boost dB). The first is the front-to-back mode of a
/// cabin about 1.9 m long.
const CABIN_MODES: [(f32, f32, f32); 3] = [(90.0, 3.0, 5.0), (140.0, 3.0, 4.0), (210.0, 3.0, 3.0)];
const REVERB_TIME: f32 = 0.08; // s, RT60
const REVERB_MIX: f32 = 0.2;

/// Road noise at 100 km/h, and its growth with speed (power of speed ratio).
const ROAD_GAIN: f32 = 0.1;
const ROAD_EXPONENT: f32 = 1.5;
/// Wind noise at 100 km/h; it grows much faster with speed. Band (Hz).
const WIND_GAIN: f32 = 0.025;
const WIND_EXPONENT: f32 = 2.5;
const WIND_BAND: (f32, f32) = (400.0, 2500.0);
const REFERENCE_SPEED: f32 = 100.0 / 3.6;

const DC_BLOCK_HZ: f32 = 20.0;

/// Sources reaching the cabin, for one sample.
pub struct CabinInputs {
    pub exhaust: f32,
    pub intake: f32,
    /// Engine torque fluctuation driving the mounts.
    pub structure: f32,
    pub mechanical: f32,
    /// Active Sound Control signal before its speaker band limit.
    pub asc: f32,
    /// Vehicle speed, m/s.
    pub speed: f32,
}

pub struct Cabin {
    spec: &'static CabinSpec,
    body: Biquad,
    leak: OnePoleLowPass,
    intake_path: Biquad,
    mounts: Biquad,
    asc_band: [Biquad; 2],
    road_band: Biquad,
    road_rumble: Biquad,
    wind_band: [Biquad; 2],
    cabin_gain: Biquad,
    modes: [Biquad; 3],
    reverb: Reverb,
    dc_block: OnePoleHighPass,
    noise: Noise,
}

impl Cabin {
    pub fn new(sample_rate: f32, spec: &'static CabinSpec) -> Self {
        let low_pass = |(hz, q): (f32, f32)| Biquad::low_pass(sample_rate, hz, q);
        Self {
            spec,
            body: low_pass(spec.body_transmission),
            leak: OnePoleLowPass::new(sample_rate, spec.exhaust_leak.1),
            intake_path: low_pass(spec.intake_transmission),
            mounts: low_pass(spec.mount_transmission),
            asc_band: [
                Biquad::high_pass(sample_rate, ASC_BAND.0, 0.7),
                Biquad::low_pass(sample_rate, ASC_BAND.1, 0.7),
            ],
            road_band: Biquad::band_pass(sample_rate, 350.0, 0.8),
            road_rumble: Biquad::low_pass(sample_rate, 90.0, 0.7),
            wind_band: [
                Biquad::high_pass(sample_rate, WIND_BAND.0, 0.7),
                Biquad::low_pass(sample_rate, WIND_BAND.1, 0.7),
            ],
            cabin_gain: Biquad::low_shelf(sample_rate, CABIN_GAIN.0, CABIN_GAIN.1),
            modes: CABIN_MODES.map(|(hz, q, db)| Biquad::peaking(sample_rate, hz, q, db)),
            reverb: Reverb::new(sample_rate, REVERB_TIME),
            dc_block: OnePoleHighPass::new(sample_rate, DC_BLOCK_HZ),
            noise: Noise::new(0x51ed_270b),
        }
    }

    pub fn process(&mut self, inputs: CabinInputs) -> f32 {
        let spec = self.spec;
        let exhaust = spec.exhaust_gain
            * (self.body.process(inputs.exhaust)
                + spec.exhaust_leak.0 * self.leak.process(inputs.exhaust));
        let intake = spec.intake_gain * self.intake_path.process(inputs.intake);
        let structure = spec.structure_gain * self.mounts.process(inputs.structure);
        let mechanical = spec.mechanical_gain * inputs.mechanical;
        let asc = spec.asc_gain
            * self
                .asc_band
                .iter_mut()
                .fold(inputs.asc, |signal, filter| filter.process(signal));

        let speed = inputs.speed.abs() / REFERENCE_SPEED;
        let pink = self.noise.pink();
        let road = ROAD_GAIN
            * speed.powf(ROAD_EXPONENT)
            * (self.road_band.process(pink) + self.road_rumble.process(pink));
        let white = self.noise.white();
        let wind = WIND_GAIN
            * speed.powf(WIND_EXPONENT)
            * self
                .wind_band
                .iter_mut()
                .fold(white, |signal, filter| filter.process(signal));

        let mut cabin = self
            .cabin_gain
            .process(exhaust + intake + structure + mechanical + asc + road + wind);
        for mode in &mut self.modes {
            cabin = mode.process(cabin);
        }
        let cabin = cabin + REVERB_MIX * self.reverb.process(cabin);
        self.dc_block.process(cabin)
    }
}

/// A small feedback delay network: four delay lines mixed through a
/// Hadamard matrix, each feeding back with damping.
struct Reverb {
    lines: [ReverbLine; 4],
}

struct ReverbLine {
    buffer: Vec<f32>,
    position: usize,
    feedback: f32,
    damping: OnePoleLowPass,
}

impl Reverb {
    fn new(sample_rate: f32, rt60: f32) -> Self {
        // Mutually prime lengths of about 7 to 15 ms.
        let lines = [331, 467, 593, 709].map(|samples_at_48k: usize| {
            let length = (samples_at_48k as f32 * sample_rate / 48_000.0).round() as usize;
            ReverbLine {
                buffer: vec![0.0; length],
                position: 0,
                // Each pass decays so the whole tail falls 60 dB in rt60.
                feedback: 10f32.powf(-3.0 * length as f32 / (rt60 * sample_rate)),
                damping: OnePoleLowPass::new(sample_rate, 3000.0),
            }
        });
        Self { lines }
    }

    fn process(&mut self, input: f32) -> f32 {
        let out = self.lines.each_ref().map(|line| line.buffer[line.position]);
        let mixed = [
            out[0] + out[1] + out[2] + out[3],
            out[0] - out[1] + out[2] - out[3],
            out[0] + out[1] - out[2] - out[3],
            out[0] - out[1] - out[2] + out[3],
        ];
        for (line, mixed) in self.lines.iter_mut().zip(mixed) {
            let fed_back = line.damping.process(mixed * 0.5) * line.feedback;
            line.buffer[line.position] = input + fed_back;
            line.position = (line.position + 1) % line.buffer.len();
        }
        out.iter().sum::<f32>() * 0.25
    }
}
