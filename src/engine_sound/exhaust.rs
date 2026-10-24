//! The exhaust system as a network of waveguide pipes: four equal-length
//! header runners into a collector, a centre pipe, a two-chamber muffler
//! and the tailpipe, which radiates into the air.
//!
//! Exhaust pulses enter at the cylinder end of each runner. That end
//! reflects strongly while the exhaust valve is shut and hardly at all while
//! it is open. The gas cools along the system, so sound travels slower
//! downstream; all of it speeds up as the engine works harder and the gas
//! gets hotter.

use super::filters::OnePoleLowPass;
use super::waveguide::{Pipe, scatter};

/// Header runner length, m, and the small manufacturing differences
/// between the four.
const RUNNER_LENGTH: f32 = 0.45;
const RUNNER_DETUNE: [f32; 4] = [1.0, 1.02, 0.985, 1.012];
const RUNNER_AREA: f32 = 1.0;

/// Downstream sections: (length m, relative area, loss per pass, damping cutoff Hz).
/// The muffler chambers are wide and lined with packing, so they absorb.
const COLLECTOR: (f32, f32, f32, f32) = (0.3, 2.2, 0.995, 9000.0);
const CENTER_PIPE: (f32, f32, f32, f32) = (1.5, 1.8, 0.993, 7000.0);
const FIRST_CHAMBER: (f32, f32, f32, f32) = (0.35, 14.0, 0.9, 2500.0);
const LINK_PIPE: (f32, f32, f32, f32) = (0.5, 1.8, 0.993, 6000.0);
const SECOND_CHAMBER: (f32, f32, f32, f32) = (0.3, 10.0, 0.92, 3000.0);
const TAILPIPE: (f32, f32, f32, f32) = (0.7, 1.8, 0.995, 8000.0);

/// Gas temperature relative to the header, per section, as it cools.
const CENTER_COOLING: f32 = 0.85;
const MUFFLER_COOLING: f32 = 0.7;
const TAILPIPE_COOLING: f32 = 0.6;

const VALVE_CLOSED_REFLECTION: f32 = 0.9;
const VALVE_OPEN_REFLECTION: f32 = 0.15;
/// The open end reflects low frequencies (inverted) and lets high ones out.
const OPEN_END_REFLECTION: f32 = 0.98;
const OPEN_END_CUTOFF: f32 = 1500.0;

pub struct Exhaust {
    runners: [Pipe; 4],
    collector: Pipe,
    center: Pipe,
    first_chamber: Pipe,
    link: Pipe,
    second_chamber: Pipe,
    tailpipe: Pipe,
    open_end: OnePoleLowPass,
    previous_flow: f32,
}

fn pipe(sample_rate: f32, (length, area, loss, damping): (f32, f32, f32, f32)) -> Pipe {
    Pipe::new(sample_rate, length, area, loss, damping)
}

impl Exhaust {
    pub fn new(sample_rate: f32) -> Self {
        Self {
            runners: RUNNER_DETUNE.map(|detune| {
                Pipe::new(
                    sample_rate,
                    RUNNER_LENGTH * detune,
                    RUNNER_AREA,
                    0.997,
                    10_000.0,
                )
            }),
            collector: pipe(sample_rate, COLLECTOR),
            center: pipe(sample_rate, CENTER_PIPE),
            first_chamber: pipe(sample_rate, FIRST_CHAMBER),
            link: pipe(sample_rate, LINK_PIPE),
            second_chamber: pipe(sample_rate, SECOND_CHAMBER),
            tailpipe: pipe(sample_rate, TAILPIPE),
            open_end: OnePoleLowPass::new(sample_rate, OPEN_END_CUTOFF),
            previous_flow: 0.0,
        }
    }

    /// Sets the speed of sound in each section from the header gas
    /// temperature, K.
    pub fn set_gas_temperature(&mut self, kelvin: f32) {
        let speed = |t: f32| 20.05 * t.sqrt();
        for runner in &mut self.runners {
            runner.set_sound_speed(speed(kelvin));
        }
        self.collector.set_sound_speed(speed(kelvin));
        self.center.set_sound_speed(speed(kelvin * CENTER_COOLING));
        for muffler in [
            &mut self.first_chamber,
            &mut self.link,
            &mut self.second_chamber,
        ] {
            muffler.set_sound_speed(speed(kelvin * MUFFLER_COOLING));
        }
        self.tailpipe
            .set_sound_speed(speed(kelvin * TAILPIPE_COOLING));
    }

    /// Advances one sample with `pulses` entering each runner and each
    /// exhaust valve open or shut. Returns the sound radiated from the
    /// tailpipe.
    pub fn process(&mut self, pulses: [f32; 4], valves_open: [bool; 4]) -> f32 {
        let runners = self.runners.each_mut().map(|runner| runner.arrivals());
        let collector = self.collector.arrivals();
        let center = self.center.arrivals();
        let first_chamber = self.first_chamber.arrivals();
        let link = self.link.arrivals();
        let second_chamber = self.second_chamber.arrivals();
        let tailpipe = self.tailpipe.arrivals();

        // Runners into the collector.
        let header = scatter([
            (runners[0].1, self.runners[0].area),
            (runners[1].1, self.runners[1].area),
            (runners[2].1, self.runners[2].area),
            (runners[3].1, self.runners[3].area),
            (collector.0, self.collector.area),
        ]);
        let [collector_out, center_in] = scatter([
            (collector.1, self.collector.area),
            (center.0, self.center.area),
        ]);
        let [center_out, first_in] = scatter([
            (center.1, self.center.area),
            (first_chamber.0, self.first_chamber.area),
        ]);
        let [first_out, link_in] = scatter([
            (first_chamber.1, self.first_chamber.area),
            (link.0, self.link.area),
        ]);
        let [link_out, second_in] = scatter([
            (link.1, self.link.area),
            (second_chamber.0, self.second_chamber.area),
        ]);
        let [second_out, tail_in] = scatter([
            (second_chamber.1, self.second_chamber.area),
            (tailpipe.0, self.tailpipe.area),
        ]);

        // Open end: what is not reflected leaves as flow, and the sound
        // radiated is proportional to how fast that flow changes.
        let reflected = -OPEN_END_REFLECTION * self.open_end.process(tailpipe.1);
        let flow = tailpipe.1 - reflected;
        let radiated = flow - self.previous_flow;
        self.previous_flow = flow;

        for (i, runner) in self.runners.iter_mut().enumerate() {
            let reflection = if valves_open[i] {
                VALVE_OPEN_REFLECTION
            } else {
                VALVE_CLOSED_REFLECTION
            };
            runner.inject(reflection * runners[i].0 + pulses[i], header[i]);
        }
        self.collector.inject(header[4], collector_out);
        self.center.inject(center_in, center_out);
        self.first_chamber.inject(first_in, first_out);
        self.link.inject(link_in, link_out);
        self.second_chamber.inject(second_in, second_out);
        self.tailpipe.inject(tail_in, reflected);

        radiated
    }
}
