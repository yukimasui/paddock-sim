//! Digital waveguide pipes for the exhaust.
//!
//! Each pipe carries pressure waves in both directions through two delay
//! lines, with a little frequency-dependent loss per pass. Pipes meet at
//! junctions, where waves scatter according to the pipes' cross-sections:
//! pressure is common to all branches and volume flow is conserved.

use super::filters::OnePoleLowPass;

/// A delay line read at a fractional delay, which may change over time.
struct DelayLine {
    buffer: Vec<f32>,
    write: usize,
}

impl DelayLine {
    fn new(capacity: usize) -> Self {
        Self {
            buffer: vec![0.0; capacity.max(4)],
            write: 0,
        }
    }

    /// The sample written `delay` samples ago (at least 1), interpolated.
    fn read(&self, delay: f32) -> f32 {
        let len = self.buffer.len();
        let delay = delay.clamp(1.0, (len - 2) as f32);
        let whole = delay.floor();
        let fraction = delay - whole;
        let newer = (self.write + len - whole as usize) % len;
        let older = (newer + len - 1) % len;
        self.buffer[newer] * (1.0 - fraction) + self.buffer[older] * fraction
    }

    fn push(&mut self, value: f32) {
        self.buffer[self.write] = value;
        self.write = (self.write + 1) % self.buffer.len();
    }
}

/// A uniform pipe section. "Upstream" is the end nearer the engine.
pub struct Pipe {
    pub length: f32, // m
    pub area: f32,   // relative cross-section
    sample_rate: f32,
    delay: f32, // samples, one way
    downstream_wave: DelayLine,
    upstream_wave: DelayLine,
    loss: f32,
    downstream_damping: OnePoleLowPass,
    upstream_damping: OnePoleLowPass,
}

/// Slowest speed of sound the pipes must hold delay for, m/s.
const MIN_SOUND_SPEED: f32 = 300.0;

impl Pipe {
    /// `loss` is the gain of one pass (below 1); `damping_hz` the cutoff
    /// of the low-pass that models wall friction and heat loss.
    pub fn new(sample_rate: f32, length: f32, area: f32, loss: f32, damping_hz: f32) -> Self {
        let capacity = (length / MIN_SOUND_SPEED * sample_rate).ceil() as usize + 4;
        let mut pipe = Self {
            length,
            area,
            sample_rate,
            delay: 1.0,
            downstream_wave: DelayLine::new(capacity),
            upstream_wave: DelayLine::new(capacity),
            loss,
            downstream_damping: OnePoleLowPass::new(sample_rate, damping_hz),
            upstream_damping: OnePoleLowPass::new(sample_rate, damping_hz),
        };
        pipe.set_sound_speed(500.0);
        pipe
    }

    /// Hotter gas carries sound faster, shortening the pipe acoustically.
    pub fn set_sound_speed(&mut self, speed: f32) {
        self.delay = self.length / speed.max(MIN_SOUND_SPEED) * self.sample_rate;
    }

    /// Waves arriving this sample at the (upstream, downstream) ends.
    pub fn arrivals(&mut self) -> (f32, f32) {
        let at_upstream = self
            .upstream_damping
            .process(self.upstream_wave.read(self.delay));
        let at_downstream = self
            .downstream_damping
            .process(self.downstream_wave.read(self.delay));
        (at_upstream * self.loss, at_downstream * self.loss)
    }

    /// Sends waves into the pipe from its (upstream, downstream) ends and
    /// advances one sample.
    pub fn inject(&mut self, at_upstream: f32, at_downstream: f32) {
        self.downstream_wave.push(at_upstream);
        self.upstream_wave.push(at_downstream);
    }
}

/// Scattering at a junction of pipes. `arriving` holds each branch's
/// arriving wave and cross-section; returns the wave leaving into each.
pub fn scatter<const N: usize>(arriving: [(f32, f32); N]) -> [f32; N] {
    let total_area: f32 = arriving.iter().map(|&(_, area)| area).sum();
    let junction_pressure = 2.0
        * arriving
            .iter()
            .map(|&(wave, area)| wave * area)
            .sum::<f32>()
        / total_area;
    arriving.map(|(wave, _)| junction_pressure - wave)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equal_pipes_pass_waves_straight_through() {
        let out = scatter([(1.0, 2.0), (0.0, 2.0)]);
        assert!((out[0] - 0.0).abs() < 1e-6, "no reflection");
        assert!((out[1] - 1.0).abs() < 1e-6, "full transmission");
    }

    #[test]
    fn widening_reflects_with_inverted_sign() {
        // Into a much wider pipe, like an open end, the reflection inverts.
        let out = scatter([(1.0, 1.0), (0.0, 9.0)]);
        assert!((out[0] - (-0.8)).abs() < 1e-6, "reflection {}", out[0]);
        assert!((out[1] - 0.2).abs() < 1e-6, "transmission {}", out[1]);
    }

    #[test]
    fn closed_open_pipe_resonates_at_a_quarter_wavelength() {
        // A pipe closed at one end and open at the other has its first
        // resonance where it is a quarter wavelength long.
        let rate = 48_000.0;
        let speed = 500.0;
        let length = 1.0;
        let mut pipe = Pipe::new(rate, length, 1.0, 0.999, 15_000.0);
        pipe.set_sound_speed(speed);
        let mut output = Vec::new();
        for n in 0..(rate as usize) {
            let (at_closed, at_open) = pipe.arrivals();
            let excitation = if n == 0 { 1.0 } else { 0.0 };
            pipe.inject(at_closed + excitation, -at_open);
            output.push(at_open);
        }
        let expected = speed / (4.0 * length);
        let magnitude = |f: f32| {
            let (mut re, mut im) = (0.0, 0.0);
            for (n, x) in output.iter().enumerate() {
                let phase = std::f32::consts::TAU * f * n as f32 / rate;
                re += x * phase.cos();
                im += x * phase.sin();
            }
            (re * re + im * im).sqrt()
        };
        let peak = (50..300)
            .map(|f| f as f32)
            .max_by(|a, b| magnitude(*a).total_cmp(&magnitude(*b)))
            .unwrap();
        assert!(
            (peak - expected).abs() < 3.0,
            "peak at {peak} Hz, expected {expected}"
        );
    }
}
