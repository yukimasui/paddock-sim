//! The exhaust system as a network of waveguide pipes: equal-length header
//! runners, one collector per bank, a centre pipe, a two-chamber muffler and
//! the tailpipe, which radiates into the air.
//!
//! Exhaust pulses enter at the cylinder end of each runner. That end
//! reflects strongly while the exhaust valve is shut and hardly at all while
//! it is open. The gas cools along the system, so sound travels slower
//! downstream; all of it speeds up as the engine works harder and the gas
//! gets hotter.

use super::filters::OnePoleLowPass;
use super::waveguide::{Pipe, scatter, scatter_into};

/// A straight pipe section: length (m), relative cross-section, gain per
/// pass (below 1) and the cutoff (Hz) of its frequency-dependent damping.
/// Muffler chambers are wide and lined with packing, so they absorb.
#[derive(Clone, Copy, Debug)]
pub struct Section {
    pub length: f32,
    pub area: f32,
    pub loss: f32,
    pub damping_hz: f32,
}

#[derive(Debug)]
pub struct ExhaustSpec {
    pub runner: Section,
    /// Small manufacturing differences in runner length, one per cylinder
    /// in firing order.
    pub runner_detune: &'static [f32],
    pub collector: Section,
    pub center_pipe: Section,
    pub first_chamber: Section,
    pub link_pipe: Section,
    pub second_chamber: Section,
    pub tailpipe: Section,
}

/// Gas temperature relative to the header, per section, as it cools.
const CENTER_COOLING: f32 = 0.85;
const MUFFLER_COOLING: f32 = 0.7;
const TAILPIPE_COOLING: f32 = 0.6;

const VALVE_CLOSED_REFLECTION: f32 = 0.9;
const VALVE_OPEN_REFLECTION: f32 = 0.15;
/// The open end reflects low frequencies (inverted) and lets high ones out.
const OPEN_END_REFLECTION: f32 = 0.98;
const OPEN_END_CUTOFF: f32 = 1500.0;
/// Most pipes meeting at one junction: five runners and their collector.
const MAX_BRANCHES: usize = 8;

pub struct Exhaust {
    runners: Vec<Pipe>,
    /// Bank (collector) each runner feeds.
    runner_bank: Vec<usize>,
    collectors: Vec<Pipe>,
    center: Pipe,
    first_chamber: Pipe,
    link: Pipe,
    second_chamber: Pipe,
    tailpipe: Pipe,
    open_end: OnePoleLowPass,
    previous_flow: f32,
    /// Per-runner scratch for one sample: arriving (cylinder end, collector
    /// end) waves, and the wave leaving each runner into its collector.
    runner_arrivals: Vec<(f32, f32)>,
    runner_inflow: Vec<f32>,
}

fn pipe(sample_rate: f32, section: Section) -> Pipe {
    Pipe::new(
        sample_rate,
        section.length,
        section.area,
        section.loss,
        section.damping_hz,
    )
}

impl Exhaust {
    /// An exhaust for runners feeding the banks in `runner_bank`.
    pub fn new(sample_rate: f32, spec: &ExhaustSpec, runner_bank: &[usize]) -> Self {
        let banks = runner_bank.iter().max().map_or(1, |&bank| bank + 1);
        let runners: Vec<Pipe> = spec
            .runner_detune
            .iter()
            .map(|&detune| {
                pipe(
                    sample_rate,
                    Section {
                        length: spec.runner.length * detune,
                        ..spec.runner
                    },
                )
            })
            .collect();
        let count = runners.len();
        Self {
            runners,
            runner_bank: runner_bank.to_vec(),
            collectors: (0..banks)
                .map(|_| pipe(sample_rate, spec.collector))
                .collect(),
            center: pipe(sample_rate, spec.center_pipe),
            first_chamber: pipe(sample_rate, spec.first_chamber),
            link: pipe(sample_rate, spec.link_pipe),
            second_chamber: pipe(sample_rate, spec.second_chamber),
            tailpipe: pipe(sample_rate, spec.tailpipe),
            open_end: OnePoleLowPass::new(sample_rate, OPEN_END_CUTOFF),
            previous_flow: 0.0,
            runner_arrivals: vec![(0.0, 0.0); count],
            runner_inflow: vec![0.0; count],
        }
    }

    /// Sets the speed of sound in each section from the header gas
    /// temperature, K.
    pub fn set_gas_temperature(&mut self, kelvin: f32) {
        let speed = |t: f32| 20.05 * t.sqrt();
        for pipe in self.runners.iter_mut().chain(&mut self.collectors) {
            pipe.set_sound_speed(speed(kelvin));
        }
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
    pub fn process(&mut self, pulses: &[f32], valves_open: &[bool]) -> f32 {
        for (arrival, runner) in self.runner_arrivals.iter_mut().zip(&mut self.runners) {
            *arrival = runner.arrivals();
        }
        let banks = self.collectors.len();
        let mut collector_arrivals = [(0.0, 0.0); MAX_BRANCHES];
        for (arrival, collector) in collector_arrivals.iter_mut().zip(&mut self.collectors) {
            *arrival = collector.arrivals();
        }
        let center = self.center.arrivals();
        let first_chamber = self.first_chamber.arrivals();
        let link = self.link.arrivals();
        let second_chamber = self.second_chamber.arrivals();
        let tailpipe = self.tailpipe.arrivals();

        // Each bank's runners meet its collector.
        let mut collector_inflow = [0.0; MAX_BRANCHES];
        for bank in 0..banks {
            let mut arriving = [(0.0, 0.0); MAX_BRANCHES];
            let mut members = [0; MAX_BRANCHES];
            let mut count = 0;
            for (runner, &runner_bank) in self.runner_bank.iter().enumerate() {
                if runner_bank == bank {
                    arriving[count] = (self.runner_arrivals[runner].1, self.runners[runner].area);
                    members[count] = runner;
                    count += 1;
                }
            }
            arriving[count] = (collector_arrivals[bank].0, self.collectors[bank].area);
            let mut leaving = [0.0; MAX_BRANCHES];
            scatter_into(&arriving[..=count], &mut leaving[..=count]);
            for (&runner, &wave) in members[..count].iter().zip(&leaving) {
                self.runner_inflow[runner] = wave;
            }
            collector_inflow[bank] = leaving[count];
        }

        // The collectors merge into the centre pipe.
        let mut arriving = [(0.0, 0.0); MAX_BRANCHES];
        for bank in 0..banks {
            arriving[bank] = (collector_arrivals[bank].1, self.collectors[bank].area);
        }
        arriving[banks] = (center.0, self.center.area);
        let mut merged = [0.0; MAX_BRANCHES];
        scatter_into(&arriving[..=banks], &mut merged[..=banks]);

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
            runner.inject(
                reflection * self.runner_arrivals[i].0 + pulses[i],
                self.runner_inflow[i],
            );
        }
        for (bank, collector) in self.collectors.iter_mut().enumerate() {
            collector.inject(collector_inflow[bank], merged[bank]);
        }
        self.center.inject(merged[banks], center_out);
        self.first_chamber.inject(first_in, first_out);
        self.link.inject(link_in, link_out);
        self.second_chamber.inject(second_in, second_out);
        self.tailpipe.inject(tail_in, reflected);

        radiated
    }
}
