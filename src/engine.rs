//! A naturally aspirated piston engine, parameterised per car by an
//! [`EngineSpec`].
//!
//! The engine turns against a load torque from whatever it drives (the
//! transmission), or freely as in neutral. Torque comes from a full-throttle
//! torque curve scaled by how well the throttle lets the cylinders fill with
//! air, which falls as engine speed rises. The response lags the pedal twice:
//! the drive-by-wire throttle is smoothed by the ECU, and the intake manifold
//! takes time to fill or empty. Friction and pumping losses, an idle speed
//! controller and a fuel-cut rev limiter complete the model.

use std::f32::consts::TAU;

/// What distinguishes one engine from another.
#[derive(Debug)]
pub struct EngineSpec {
    /// Full-throttle torque curve (rpm, N m), in increasing rpm.
    pub torque_curve: &'static [(f32, f32)],
    /// Rotating inertia of crank, flywheel and torque converter impeller, kg m^2.
    pub inertia: f32,
    /// Time constant with which the ECU lets the throttle follow the pedal, s.
    pub throttle_lag: f32,
    /// Intake volume (plenum and runners) over engine displacement. The
    /// manifold fills or empties over the time the engine takes to swallow
    /// this many displacements.
    pub manifold_volume_ratio: f32,
    /// Friction torque, N m: constant part and part per rpm.
    pub friction_base: f32,
    pub friction_per_rpm: f32,
    /// Extra pumping loss with the throttle shut, N m at `pumping_reference_rpm`.
    pub pumping_loss: f32,
    pub pumping_reference_rpm: f32,
    /// Engine speed up to which the wide-open throttle still fills the
    /// cylinders fully.
    pub full_flow_rpm: f32,
    pub idle_rpm: f32,
    pub redline_rpm: f32,
    /// Fuel is cut above the first speed and restored below the second.
    pub limiter_rpm: (f32, f32),
}

/// Subaru BRZ / Toyota GR86 FA24: 2.4 L flat-four, 250 N m at 3700 rpm and
/// 173 kW at 7000 rpm.
pub static FA24: EngineSpec = EngineSpec {
    torque_curve: &[
        (500.0, 130.0),
        (1000.0, 180.0),
        (2000.0, 205.0),
        (3000.0, 235.0),
        (3700.0, 250.0),
        (5000.0, 248.0),
        (6000.0, 245.0),
        (7000.0, 236.0),
        (7600.0, 215.0),
    ],
    inertia: 0.2,
    throttle_lag: 0.12,
    manifold_volume_ratio: 1.25,
    friction_base: 12.0,
    friction_per_rpm: 0.004,
    pumping_loss: 60.0,
    pumping_reference_rpm: 7000.0,
    full_flow_rpm: 8000.0,
    idle_rpm: 750.0,
    redline_rpm: 7400.0,
    limiter_rpm: (7450.0, 7300.0),
};

/// Lexus LFA 1LR-GUE: 4.8 L 72-degree V10, 480 N m at 6800 rpm and 412 kW
/// at 8700 rpm, redline 9000 rpm. Individual throttle bodies and a light
/// rotating assembly make it famously quick to rev.
pub static ONE_LR_GUE: EngineSpec = EngineSpec {
    torque_curve: &[
        (500.0, 250.0),
        (1000.0, 300.0),
        (2000.0, 350.0),
        (3000.0, 400.0),
        (3700.0, 432.0),
        (5000.0, 455.0),
        (6800.0, 480.0),
        (8000.0, 470.0),
        (8700.0, 452.0),
        (9000.0, 440.0),
        (9600.0, 400.0),
    ],
    inertia: 0.16,
    throttle_lag: 0.05,
    manifold_volume_ratio: 0.4,
    friction_base: 20.0,
    friction_per_rpm: 0.005,
    pumping_loss: 90.0,
    pumping_reference_rpm: 9000.0,
    full_flow_rpm: 10_000.0,
    idle_rpm: 950.0,
    redline_rpm: 9000.0,
    limiter_rpm: (9500.0, 9350.0),
};

/// Air that leaks past the closed throttle, as a fraction of full flow.
const CLOSED_THROTTLE_FLOW: f32 = 0.005;
/// Extra air the idle controller can add, as a fraction of full flow.
const IDLE_CONTROL_FLOW: f32 = 0.04;
/// Below idle by this many rpm the idle controller is fully open.
const IDLE_CONTROL_RANGE: f32 = 250.0;
/// With the pedal up and this far above idle, the ECU cuts fuel, rpm.
const OVERRUN_FUEL_CUT_MARGIN: f32 = 400.0;
const MIN_RPM: f32 = 400.0;

#[derive(Clone, Debug)]
pub struct Engine {
    pub spec: &'static EngineSpec,
    pub rpm: f32,
    /// Throttle plate opening, 0..=1, after the ECU's smoothing.
    pub throttle: f32,
    /// Cylinder filling the manifold pressure currently allows, 0..=1.
    pub fill: f32,
    /// Fraction of a full cylinder charge burnt in each firing, 0..=1. Zero
    /// while fuel is cut.
    pub load: f32,
    /// True while the rev limiter has cut the fuel.
    pub limiter: bool,
    /// Share of combustion torque the ECU allows, 0..=1. Traction control
    /// lowers it (in a real car by retarding the spark and closing the
    /// throttle).
    pub torque_limit: f32,
}

impl Engine {
    pub fn new(spec: &'static EngineSpec) -> Self {
        Self {
            spec,
            rpm: spec.idle_rpm,
            throttle: 0.0,
            fill: 0.0,
            load: 0.0,
            limiter: false,
            torque_limit: 1.0,
        }
    }

    /// Advances a free-revving engine (as in neutral) by `dt` seconds with
    /// the pedal at `pedal` (0..=1).
    #[cfg(test)]
    pub fn update(&mut self, pedal: f32, dt: f32) {
        self.update_with_load(pedal, 0.0, dt);
    }

    /// Advances the engine by `dt` seconds with the pedal at `pedal` (0..=1)
    /// while whatever it drives resists with `load_torque` (N m).
    pub fn update_with_load(&mut self, pedal: f32, load_torque: f32, dt: f32) {
        let spec = self.spec;
        // Drive-by-wire maps the pedal progressively, so the first part of
        // its travel opens the throttle only a little, and smooths it.
        let requested = pedal.clamp(0.0, 1.0).powi(2);
        self.throttle += (requested - self.throttle) * (1.0 - (-dt / spec.throttle_lag).exp());

        // The idle controller opens a bypass as the engine drops towards idle.
        let idle_air =
            ((spec.idle_rpm - self.rpm) / IDLE_CONTROL_RANGE).clamp(0.0, 1.0) * IDLE_CONTROL_FLOW;
        let flow = (CLOSED_THROTTLE_FLOW + idle_air + self.throttle).min(1.0);
        // The same airflow spread over more intake strokes fills each less.
        let steady_fill = (flow * spec.full_flow_rpm / self.rpm).min(1.0);
        // A four-stroke swallows one displacement every two revolutions.
        let manifold_time = spec.manifold_volume_ratio * 120.0 / self.rpm;
        self.fill += (steady_fill - self.fill) * (1.0 - (-dt / manifold_time).exp());
        let fill = self.fill;

        if self.rpm > spec.limiter_rpm.0 {
            self.limiter = true;
        } else if self.rpm < spec.limiter_rpm.1 {
            self.limiter = false;
        }
        let overrun = requested == 0.0 && self.rpm > spec.idle_rpm + OVERRUN_FUEL_CUT_MARGIN;
        self.load = if self.limiter || overrun { 0.0 } else { fill };

        let combustion = self.full_throttle_torque() * self.load * self.torque_limit;
        let losses = spec.friction_base
            + spec.friction_per_rpm * self.rpm
            + spec.pumping_loss * (1.0 - fill) * self.rpm / spec.pumping_reference_rpm;
        let angular_acceleration = (combustion - losses - load_torque) / spec.inertia;
        self.rpm = (self.rpm + angular_acceleration * 60.0 / TAU * dt).max(MIN_RPM);
    }

    /// Crankshaft speed, rad/s.
    pub fn angular_velocity(&self) -> f32 {
        self.rpm * TAU / 60.0
    }

    /// Peak of the full-throttle torque curve, N m.
    pub fn peak_torque(&self) -> f32 {
        self.spec
            .torque_curve
            .iter()
            .map(|&(_, torque)| torque)
            .fold(0.0, f32::max)
    }

    /// Wide-open-throttle torque at the current speed, N m.
    fn full_throttle_torque(&self) -> f32 {
        let curve = self.spec.torque_curve;
        let rpm = self.rpm;
        let (first, last) = (curve[0], curve[curve.len() - 1]);
        if rpm <= first.0 {
            return first.1;
        }
        if rpm >= last.0 {
            return last.1;
        }
        let i = curve.partition_point(|&(r, _)| r <= rpm);
        let (r0, t0) = curve[i - 1];
        let (r1, t1) = curve[i];
        t0 + (t1 - t0) * (rpm - r0) / (r1 - r0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;
    const ENGINES: [&EngineSpec; 2] = [&FA24, &ONE_LR_GUE];

    fn run(engine: &mut Engine, pedal: f32, seconds: f32) {
        for _ in 0..(seconds / DT) as usize {
            engine.update(pedal, DT);
        }
    }

    #[test]
    fn settles_at_idle_with_the_pedal_up() {
        for spec in ENGINES {
            let mut engine = Engine {
                rpm: 2000.0,
                ..Engine::new(spec)
            };
            run(&mut engine, 0.0, 5.0);
            assert!(
                (engine.rpm - spec.idle_rpm).abs() < 60.0,
                "idle at {:.0} rpm",
                engine.rpm
            );
        }
    }

    #[test]
    fn full_throttle_revs_to_the_limiter_quickly() {
        // Free-revving from idle to redline: about a second for the FA24
        // with its throttle and manifold lag, a well-known 0.6 s or so for
        // the LFA's V10.
        for (spec, expected) in [(&FA24, 0.8..1.4), (&ONE_LR_GUE, 0.4..0.9)] {
            let mut engine = Engine::new(spec);
            let mut elapsed = 0.0;
            while engine.rpm < spec.redline_rpm {
                engine.update(1.0, DT);
                elapsed += DT;
                assert!(elapsed < 2.0, "stuck at {:.0} rpm", engine.rpm);
            }
            assert!(expected.contains(&elapsed), "took {elapsed:.2} s");

            // Held flat, the limiter keeps it bouncing just below the cut.
            let mut peak: f32 = 0.0;
            for _ in 0..2000 {
                engine.update(1.0, DT);
                peak = peak.max(engine.rpm);
            }
            assert!(peak < spec.limiter_rpm.0 + 100.0, "peak {peak:.0} rpm");
        }
    }

    #[test]
    fn lifting_off_falls_back_to_idle_under_engine_braking() {
        for spec in ENGINES {
            let mut engine = Engine {
                rpm: spec.redline_rpm - 400.0,
                ..Engine::new(spec)
            };
            run(&mut engine, 0.0, 0.1);
            assert_eq!(engine.load, 0.0, "fuel should be cut on overrun");
            run(&mut engine, 0.0, 3.0);
            assert!(
                engine.rpm < spec.idle_rpm + 100.0,
                "still at {:.0} rpm",
                engine.rpm
            );
        }
    }

    #[test]
    fn part_throttle_holds_a_part_way_speed() {
        for spec in ENGINES {
            let speed_at = |pedal: f32| {
                let mut engine = Engine::new(spec);
                run(&mut engine, pedal, 5.0);
                engine.rpm
            };
            let light = speed_at(0.2);
            let medium = speed_at(0.35);
            assert!(
                spec.idle_rpm + 200.0 < light && light < medium && medium < spec.redline_rpm,
                "{light:.0} / {medium:.0} rpm"
            );
        }
    }
}
