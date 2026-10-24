//! Small DSP building blocks: filters, smoothing and noise.

use std::f32::consts::TAU;

/// Second-order filter (RBJ audio EQ cookbook), direct form I.
#[derive(Clone)]
pub struct Biquad {
    b: [f32; 3],
    a: [f32; 2],
    x: [f32; 2],
    y: [f32; 2],
}

impl Biquad {
    fn from_coefficients(b: [f32; 3], a: [f32; 3]) -> Self {
        Self {
            b: b.map(|b| b / a[0]),
            a: [a[1] / a[0], a[2] / a[0]],
            x: [0.0; 2],
            y: [0.0; 2],
        }
    }

    fn prewarp(sample_rate: f32, frequency: f32, q: f32) -> (f32, f32) {
        let omega = TAU * frequency.min(sample_rate * 0.45) / sample_rate;
        (omega.cos(), omega.sin() / (2.0 * q))
    }

    pub fn low_pass(sample_rate: f32, cutoff: f32, q: f32) -> Self {
        let (cos, alpha) = Self::prewarp(sample_rate, cutoff, q);
        Self::from_coefficients(
            [(1.0 - cos) / 2.0, 1.0 - cos, (1.0 - cos) / 2.0],
            [1.0 + alpha, -2.0 * cos, 1.0 - alpha],
        )
    }

    pub fn high_pass(sample_rate: f32, cutoff: f32, q: f32) -> Self {
        let (cos, alpha) = Self::prewarp(sample_rate, cutoff, q);
        Self::from_coefficients(
            [(1.0 + cos) / 2.0, -(1.0 + cos), (1.0 + cos) / 2.0],
            [1.0 + alpha, -2.0 * cos, 1.0 - alpha],
        )
    }

    /// Band-pass with 0 dB gain at the centre frequency.
    pub fn band_pass(sample_rate: f32, center: f32, q: f32) -> Self {
        let (cos, alpha) = Self::prewarp(sample_rate, center, q);
        Self::from_coefficients([alpha, 0.0, -alpha], [1.0 + alpha, -2.0 * cos, 1.0 - alpha])
    }

    /// Peaking EQ boosting (or cutting) `gain_db` around `center`.
    pub fn peaking(sample_rate: f32, center: f32, q: f32, gain_db: f32) -> Self {
        let (cos, alpha) = Self::prewarp(sample_rate, center, q);
        let a = 10f32.powf(gain_db / 40.0);
        Self::from_coefficients(
            [1.0 + alpha * a, -2.0 * cos, 1.0 - alpha * a],
            [1.0 + alpha / a, -2.0 * cos, 1.0 - alpha / a],
        )
    }

    pub fn process(&mut self, input: f32) -> f32 {
        let output = self.b[0] * input + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[0] * self.y[0]
            - self.a[1] * self.y[1];
        self.x = [input, self.x[0]];
        self.y = [output, self.y[0]];
        output
    }
}

/// First-order low-pass (6 dB per octave).
#[derive(Clone)]
pub struct OnePoleLowPass {
    coefficient: f32,
    state: f32,
}

impl OnePoleLowPass {
    pub fn new(sample_rate: f32, cutoff: f32) -> Self {
        Self {
            coefficient: 1.0 - (-TAU * cutoff / sample_rate).exp(),
            state: 0.0,
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        self.state += (input - self.state) * self.coefficient;
        self.state
    }
}

/// First-order high-pass, used to remove DC and subsonic rumble.
pub struct OnePoleHighPass {
    coefficient: f32,
    previous_input: f32,
    previous_output: f32,
}

impl OnePoleHighPass {
    pub fn new(sample_rate: f32, cutoff: f32) -> Self {
        Self {
            coefficient: 1.0 / (1.0 + TAU * cutoff / sample_rate),
            previous_input: 0.0,
            previous_output: 0.0,
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        let output = self.coefficient * (self.previous_output + input - self.previous_input);
        self.previous_input = input;
        self.previous_output = output;
        output
    }
}

/// Exponential smoothing towards a target with time constant `time` (s).
pub fn smoothing_coefficient(sample_rate: f32, time: f32) -> f32 {
    1.0 - (-1.0 / (sample_rate * time)).exp()
}

/// Uniform, Gaussian-ish and pink noise from one xorshift generator.
pub struct Noise {
    state: u32,
    pink: [f32; 3],
}

impl Noise {
    pub fn new(seed: u32) -> Self {
        Self {
            state: seed.max(1),
            pink: [0.0; 3],
        }
    }

    /// Uniform in 0..1.
    pub fn uniform(&mut self) -> f32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        self.state as f32 / u32::MAX as f32
    }

    /// Uniform in -1..1 (white).
    pub fn white(&mut self) -> f32 {
        self.uniform() * 2.0 - 1.0
    }

    /// Roughly unit-variance, bell-shaped (sum of uniforms).
    pub fn gaussian(&mut self) -> f32 {
        (self.uniform() + self.uniform() + self.uniform() - 1.5) * 2.0
    }

    /// Pink (-3 dB per octave) noise, Paul Kellet's economy filter.
    pub fn pink(&mut self) -> f32 {
        let white = self.white();
        self.pink[0] = 0.99765 * self.pink[0] + white * 0.099_046;
        self.pink[1] = 0.963 * self.pink[1] + white * 0.296_516_4;
        self.pink[2] = 0.57 * self.pink[2] + white * 1.052_691_3;
        (self.pink[0] + self.pink[1] + self.pink[2] + white * 0.1848) * 0.25
    }
}
