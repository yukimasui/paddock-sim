//! An engine modelled on the FA24 2.4 L naturally aspirated flat-four of
//! the Subaru BRZ / Toyota GR86.
//!
//! The engine turns against a load torque from whatever it drives (the
//! transmission), or freely as in neutral. Torque comes from a full-throttle
//! torque curve scaled by how well
//! the throttle lets the cylinders fill with air, which falls as engine speed
//! rises. The response lags the pedal twice: the drive-by-wire throttle is
//! smoothed by the ECU, and the intake manifold takes time to fill or empty.
//! Friction and pumping losses, an idle speed controller and a fuel-cut rev
//! limiter complete the model.

/// Full-throttle torque curve (rpm, N m), roughly the FA24's: 250 N m at
/// 3700 rpm and 173 kW at 7000 rpm.
const TORQUE_CURVE: [(f32, f32); 9] = [
    (500.0, 130.0),
    (1000.0, 180.0),
    (2000.0, 205.0),
    (3000.0, 235.0),
    (3700.0, 250.0),
    (5000.0, 248.0),
    (6000.0, 245.0),
    (7000.0, 236.0),
    (7600.0, 215.0),
];

/// Rotating inertia of crank, flywheel and torque converter impeller, kg m^2.
pub const INERTIA: f32 = 0.2;
/// Time constant with which the ECU lets the throttle follow the pedal, s.
const THROTTLE_LAG: f32 = 0.12;
/// Intake manifold (plenum and runners) volume over engine displacement.
/// The manifold empties or fills over the time the engine takes to swallow
/// this many displacements.
const MANIFOLD_VOLUME_RATIO: f32 = 1.25;
/// Friction torque, N m: constant part and part per rpm.
const FRICTION_BASE: f32 = 12.0;
const FRICTION_PER_RPM: f32 = 0.004;
/// Extra pumping loss with the throttle shut, N m at 7000 rpm.
const PUMPING_LOSS: f32 = 60.0;
/// Engine speed up to which the wide-open throttle still fills the cylinders fully.
const FULL_FLOW_RPM: f32 = 8000.0;
/// Air that leaks past the closed throttle, as a fraction of full flow.
const CLOSED_THROTTLE_FLOW: f32 = 0.005;

pub const IDLE_RPM: f32 = 750.0;
/// Extra air the idle controller can add, as a fraction of full flow.
const IDLE_CONTROL_FLOW: f32 = 0.04;
/// Below idle by this many rpm the idle controller is fully open.
const IDLE_CONTROL_RANGE: f32 = 250.0;
const MIN_RPM: f32 = 400.0;

pub const REDLINE_RPM: f32 = 7400.0;
/// Fuel is cut above this speed and restored below the lower one.
const LIMITER_CUT_RPM: f32 = 7450.0;
const LIMITER_RESUME_RPM: f32 = 7300.0;

#[derive(Clone, Debug)]
pub struct Engine {
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
}

impl Default for Engine {
    fn default() -> Self {
        Self {
            rpm: IDLE_RPM,
            throttle: 0.0,
            fill: 0.0,
            load: 0.0,
            limiter: false,
        }
    }
}

impl Engine {
    /// Advances a free-revving engine (as in neutral) by `dt` seconds with
    /// the pedal at `pedal` (0..=1).
    #[cfg(test)]
    pub fn update(&mut self, pedal: f32, dt: f32) {
        self.update_with_load(pedal, 0.0, dt);
    }

    /// Advances the engine by `dt` seconds with the pedal at `pedal` (0..=1)
    /// while whatever it drives resists with `load_torque` (N m).
    pub fn update_with_load(&mut self, pedal: f32, load_torque: f32, dt: f32) {
        // Drive-by-wire maps the pedal progressively, so the first part of
        // its travel opens the throttle only a little, and smooths it.
        let requested = pedal.clamp(0.0, 1.0).powi(2);
        self.throttle += (requested - self.throttle) * (1.0 - (-dt / THROTTLE_LAG).exp());

        // The idle controller opens a bypass as the engine drops towards idle.
        let idle_air =
            ((IDLE_RPM - self.rpm) / IDLE_CONTROL_RANGE).clamp(0.0, 1.0) * IDLE_CONTROL_FLOW;
        let flow = (CLOSED_THROTTLE_FLOW + idle_air + self.throttle).min(1.0);
        // The same airflow spread over more intake strokes fills each less.
        let steady_fill = (flow * FULL_FLOW_RPM / self.rpm).min(1.0);
        // A four-stroke swallows one displacement every two revolutions.
        let manifold_time = MANIFOLD_VOLUME_RATIO * 120.0 / self.rpm;
        self.fill += (steady_fill - self.fill) * (1.0 - (-dt / manifold_time).exp());
        let fill = self.fill;

        if self.rpm > LIMITER_CUT_RPM {
            self.limiter = true;
        } else if self.rpm < LIMITER_RESUME_RPM {
            self.limiter = false;
        }
        // Overrun: with the pedal up and the engine well above idle, the ECU
        // cuts fuel.
        let overrun = requested == 0.0 && self.rpm > IDLE_RPM + 400.0;
        self.load = if self.limiter || overrun { 0.0 } else { fill };

        let combustion = full_throttle_torque(self.rpm) * self.load;
        let losses = FRICTION_BASE
            + FRICTION_PER_RPM * self.rpm
            + PUMPING_LOSS * (1.0 - fill) * self.rpm / 7000.0;
        let angular_acceleration = (combustion - losses - load_torque) / INERTIA;
        let rpm_per_rad_s = 60.0 / std::f32::consts::TAU;
        self.rpm = (self.rpm + angular_acceleration * rpm_per_rad_s * dt).max(MIN_RPM);
    }

    /// Crankshaft speed, rad/s.
    pub fn angular_velocity(&self) -> f32 {
        self.rpm * std::f32::consts::TAU / 60.0
    }
}

/// Wide-open-throttle torque at `rpm`, N m, interpolated from the curve.
fn full_throttle_torque(rpm: f32) -> f32 {
    let (first, last) = (TORQUE_CURVE[0], TORQUE_CURVE[TORQUE_CURVE.len() - 1]);
    if rpm <= first.0 {
        return first.1;
    }
    if rpm >= last.0 {
        return last.1;
    }
    let i = TORQUE_CURVE.partition_point(|&(r, _)| r <= rpm);
    let (r0, t0) = TORQUE_CURVE[i - 1];
    let (r1, t1) = TORQUE_CURVE[i];
    t0 + (t1 - t0) * (rpm - r0) / (r1 - r0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    fn run(engine: &mut Engine, pedal: f32, seconds: f32) {
        for _ in 0..(seconds / DT) as usize {
            engine.update(pedal, DT);
        }
    }

    #[test]
    fn settles_at_idle_with_the_pedal_up() {
        let mut engine = Engine {
            rpm: 2000.0,
            ..Engine::default()
        };
        run(&mut engine, 0.0, 5.0);
        assert!(
            (engine.rpm - IDLE_RPM).abs() < 60.0,
            "idle at {:.0} rpm",
            engine.rpm
        );
    }

    #[test]
    fn full_throttle_revs_to_the_limiter_quickly() {
        let mut engine = Engine::default();
        let mut elapsed = 0.0;
        while engine.rpm < REDLINE_RPM {
            engine.update(1.0, DT);
            elapsed += DT;
            assert!(elapsed < 2.0, "stuck at {:.0} rpm", engine.rpm);
        }
        // Throttle smoothing, manifold filling and inertia make it take about a second.
        assert!((0.8..1.4).contains(&elapsed), "took {elapsed:.2} s");

        // Held flat, the limiter keeps it bouncing just below the cut.
        let mut peak: f32 = 0.0;
        for _ in 0..2000 {
            engine.update(1.0, DT);
            peak = peak.max(engine.rpm);
        }
        assert!(peak < LIMITER_CUT_RPM + 100.0, "peak {peak:.0} rpm");
    }

    #[test]
    fn lifting_off_falls_back_to_idle_under_engine_braking() {
        let mut engine = Engine {
            rpm: 7000.0,
            ..Engine::default()
        };
        run(&mut engine, 0.0, 0.1);
        assert_eq!(engine.load, 0.0, "fuel should be cut on overrun");
        run(&mut engine, 0.0, 3.0);
        assert!(
            engine.rpm < IDLE_RPM + 100.0,
            "still at {:.0} rpm",
            engine.rpm
        );
    }

    #[test]
    fn part_throttle_holds_a_part_way_speed() {
        let speed_at = |pedal: f32| {
            let mut engine = Engine::default();
            run(&mut engine, pedal, 5.0);
            engine.rpm
        };
        let light = speed_at(0.2);
        let medium = speed_at(0.35);
        assert!(IDLE_RPM + 200.0 < light && light < medium && medium < REDLINE_RPM);
    }
}
