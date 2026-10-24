//! What reaches the driver's ears: each engine sound source through its own
//! transfer path into the cabin, plus road and wind noise, coloured by the
//! cabin's low resonances and a short reverberation.
//!
//! - Exhaust noise is airborne: the body blocks it more the higher its
//!   frequency (mass law), with a small leak through seals and glass.
//! - Intake noise comes through the firewall, muffled.
//! - Engine torque pulses shake the engine on its mounts, and the body
//!   radiates the vibration (structure-borne); this is the low boom.
//! - Mechanical ticks (injectors, valvetrain) leak through faintly.

use super::filters::{Biquad, Noise, OnePoleHighPass, OnePoleLowPass};

/// Path gains, set so that at 3000 rpm, half load and 60 km/h the exhaust
/// leads, the structure-borne boom sits just below it, intake and road noise
/// below that, and mechanical ticks are barely audible.
const EXHAUST_GAIN: f32 = 180.0;
/// Fraction of exhaust noise that leaks in unfiltered.
const EXHAUST_LEAK: f32 = 0.05;
const BODY_CUTOFF: f32 = 200.0;
const INTAKE_GAIN: f32 = 4.5;
const FIREWALL_CUTOFF: f32 = 500.0;
const STRUCTURE_GAIN: f32 = 0.7;
/// Engine mount and body panel resonances: (Hz, Q).
const MOUNT_RESONANCES: [(f32, f32); 3] = [(40.0, 2.0), (75.0, 2.5), (130.0, 3.0)];
const MECHANICAL_GAIN: f32 = 100.0;
const ASC_GAIN: f32 = 1.4;
/// Active Sound Control speaker band, Hz.
const ASC_BAND: (f32, f32) = (80.0, 600.0);

/// Cabin modes: (Hz, Q, boost dB). The first is the front-to-back mode of a
/// cabin about 1.9 m long.
const CABIN_MODES: [(f32, f32, f32); 3] = [(90.0, 3.0, 5.0), (140.0, 3.0, 4.0), (210.0, 3.0, 3.0)];
const REVERB_TIME: f32 = 0.08; // s, RT60
const REVERB_MIX: f32 = 0.2;

/// Road noise at 100 km/h, and its growth with speed (power of speed ratio).
const ROAD_GAIN: f32 = 0.14;
const ROAD_EXPONENT: f32 = 1.5;
/// Wind noise at 100 km/h; it grows much faster with speed.
const WIND_GAIN: f32 = 0.09;
const WIND_EXPONENT: f32 = 2.5;
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
    body: OnePoleLowPass,
    firewall: OnePoleLowPass,
    mounts: [Biquad; 3],
    asc_band: [Biquad; 2],
    road_band: Biquad,
    road_rumble: Biquad,
    wind_band: [Biquad; 2],
    modes: [Biquad; 3],
    reverb: Reverb,
    dc_block: OnePoleHighPass,
    noise: Noise,
}

impl Cabin {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            body: OnePoleLowPass::new(sample_rate, BODY_CUTOFF),
            firewall: OnePoleLowPass::new(sample_rate, FIREWALL_CUTOFF),
            mounts: MOUNT_RESONANCES.map(|(hz, q)| Biquad::band_pass(sample_rate, hz, q)),
            asc_band: [
                Biquad::high_pass(sample_rate, ASC_BAND.0, 0.7),
                Biquad::low_pass(sample_rate, ASC_BAND.1, 0.7),
            ],
            road_band: Biquad::band_pass(sample_rate, 350.0, 0.8),
            road_rumble: Biquad::low_pass(sample_rate, 90.0, 0.7),
            wind_band: [
                Biquad::high_pass(sample_rate, 700.0, 0.7),
                Biquad::low_pass(sample_rate, 4000.0, 0.7),
            ],
            modes: CABIN_MODES.map(|(hz, q, db)| Biquad::peaking(sample_rate, hz, q, db)),
            reverb: Reverb::new(sample_rate, REVERB_TIME),
            dc_block: OnePoleHighPass::new(sample_rate, DC_BLOCK_HZ),
            noise: Noise::new(0x51ed_270b),
        }
    }

    pub fn process(&mut self, inputs: CabinInputs) -> f32 {
        let exhaust =
            EXHAUST_GAIN * (self.body.process(inputs.exhaust) + EXHAUST_LEAK * inputs.exhaust);
        let intake = INTAKE_GAIN * self.firewall.process(inputs.intake);
        let structure = STRUCTURE_GAIN
            * self
                .mounts
                .iter_mut()
                .map(|mount| mount.process(inputs.structure))
                .sum::<f32>();
        let mechanical = MECHANICAL_GAIN * inputs.mechanical;
        let asc = ASC_GAIN
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

        let mut cabin = exhaust + intake + structure + mechanical + asc + road + wind;
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
