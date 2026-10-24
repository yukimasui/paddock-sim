//! Steady-state tire model for a single tire.
//!
//! Lateral force uses a simplified Pacejka Magic Formula with load-dependent
//! peak friction and cornering stiffness. Self-aligning torque is lateral force
//! times a pneumatic trail that shrinks with slip angle and changes sign past
//! the grip limit. Longitudinal force is not computed from slip ratio (there
//! are no wheel spin dynamics yet): the requested force is clipped to the
//! friction limit and reduces the available lateral force via a friction
//! circle.
//!
//! The slip angle fed to the model is a transient one: it lags the slip
//! angle implied by the contact patch velocity over the tire's relaxation
//! length, the distance the carcass needs to roll before its deflection
//! builds up. This keeps the model well-defined down to standstill.
//!
//! Sign convention follows ISO 8855: a positive slip angle produces a positive
//! (leftward) lateral force and a negative aligning torque.

use std::f32::consts::FRAC_PI_2;

#[derive(Clone, Copy, Debug)]
pub struct TireParams {
    /// Load at which the coefficients below are specified, N.
    pub nominal_load: f32,
    /// Peak friction coefficient at nominal load.
    pub peak_friction: f32,
    /// Fractional drop in peak friction per nominal load of extra load.
    pub load_sensitivity: f32,
    /// Cornering stiffness divided by load, at nominal load, 1/rad.
    pub cornering_stiffness_ratio: f32,
    /// Magic Formula shape factor C.
    pub shape: f32,
    /// Magic Formula curvature factor E (must be <= 1).
    pub curvature: f32,
    /// Pneumatic trail at zero slip and nominal load, m.
    pub pneumatic_trail: f32,
    /// Slip angle at which the pneumatic trail (and aligning torque) reaches zero, rad.
    pub trail_zero_slip_angle: f32,
    /// Shape factor of the trail's cosine falloff; above 1 lets the trail turn negative.
    pub trail_shape: f32,
    /// Distance the tire rolls while its lateral slip catches up with a change, m.
    pub relaxation_length: f32,
    /// Lateral damping of the carcass, N s/m. Only matters near standstill,
    /// where rolling no longer relaxes the deflection and the tire would
    /// otherwise act as an undamped spring.
    pub low_speed_damping: f32,
}

impl Default for TireParams {
    /// A performance road tire, roughly 225/45R17.
    fn default() -> Self {
        Self {
            nominal_load: 3000.0,
            peak_friction: 1.05,
            load_sensitivity: 0.15,
            cornering_stiffness_ratio: 18.0,
            shape: 1.3,
            curvature: -1.0,
            pneumatic_trail: 0.035,
            trail_zero_slip_angle: 0.15,
            trail_shape: 1.4,
            relaxation_length: 0.4,
            low_speed_damping: 6500.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TireForces {
    /// Longitudinal force actually transmitted, N.
    pub longitudinal: f32,
    /// Lateral force, N.
    pub lateral: f32,
    /// Self-aligning torque about the vertical axis, N m.
    pub aligning_torque: f32,
}

impl TireParams {
    /// The same tire on a surface whose grip is `grip` times that of asphalt.
    pub fn with_grip(&self, grip: f32) -> Self {
        Self {
            peak_friction: self.peak_friction * grip,
            ..*self
        }
    }

    /// Peak friction coefficient at `load`; heavier loads grip proportionally less.
    pub fn friction(&self, load: f32) -> f32 {
        let load_change = (load - self.nominal_load) / self.nominal_load;
        // The linear load sensitivity would go negative at absurd loads.
        (self.peak_friction * (1.0 - self.load_sensitivity * load_change)).max(0.0)
    }

    /// Cornering stiffness (N/rad) at `load`. Grows sub-linearly with load and
    /// peaks at twice the nominal load, as in Pacejka's model.
    pub fn cornering_stiffness(&self, load: f32) -> f32 {
        let shape = |l: f32| (2.0 * (l / (2.0 * self.nominal_load)).atan()).sin();
        self.cornering_stiffness_ratio * self.nominal_load * shape(load) / shape(self.nominal_load)
    }

    /// Forces for a tire at `slip_angle` (rad) carrying `load` (N) while
    /// `requested_longitudinal` (N) of drive or brake force is asked of it.
    pub fn forces(&self, slip_angle: f32, load: f32, requested_longitudinal: f32) -> TireForces {
        if load <= 0.0 {
            return TireForces::default();
        }

        let max_force = self.friction(load) * load;
        if max_force <= 0.0 {
            return TireForces::default();
        }
        let longitudinal = requested_longitudinal.clamp(-max_force, max_force);
        // Friction circle: longitudinal force uses up part of the grip.
        let lateral_share = self.lateral_capacity(load, longitudinal) / max_force;

        let c = self.shape;
        let e = self.curvature;
        let b = self.cornering_stiffness(load) / (c * max_force);
        let x = b * slip_angle;
        let pure_lateral = max_force * (c * (x - e * (x - x.atan())).atan()).sin();
        let lateral = pure_lateral * lateral_share;

        let trail_b = (FRAC_PI_2 / self.trail_shape).tan() / self.trail_zero_slip_angle;
        let trail = self.pneumatic_trail
            * (load / self.nominal_load)
            * (self.trail_shape * (trail_b * slip_angle).atan()).cos();

        TireForces {
            longitudinal,
            lateral,
            aligning_torque: -trail * lateral,
        }
    }

    /// Lateral force (N) still available at `load` while transmitting
    /// `longitudinal` force, by the friction circle.
    pub fn lateral_capacity(&self, load: f32, longitudinal: f32) -> f32 {
        let max_force = self.friction(load) * load;
        (max_force * max_force - longitudinal * longitudinal)
            .max(0.0)
            .sqrt()
    }

    /// Time derivative of the transient lateral slip (tangent of the slip
    /// angle) for a contact patch moving at `rolling_speed` along the wheel
    /// and `lateral_velocity` across it, m/s. The slip relaxes towards
    /// `-lateral_velocity / |rolling_speed|` over the relaxation length.
    pub fn lateral_slip_rate(
        &self,
        lateral_slip: f32,
        rolling_speed: f32,
        lateral_velocity: f32,
    ) -> f32 {
        (-lateral_velocity - rolling_speed.abs() * lateral_slip) / self.relaxation_length
    }

    /// Largest lateral slip the carcass deflection can hold at `load` once the
    /// contact patch stops sliding. Bounds the slip left over when the tire
    /// comes to rest. Roughly where the whole contact patch slides.
    pub fn stored_slip_limit(&self, load: f32) -> f32 {
        if load <= 0.0 {
            return 0.0;
        }
        2.0 * self.friction(load) * load / self.cornering_stiffness(load)
    }

    /// Carcass damping force (N) across the wheel, fading out by walking pace
    /// where the slip dynamics themselves damp lateral motion.
    pub fn low_speed_damping_force(&self, rolling_speed: f32, lateral_velocity: f32) -> f32 {
        const FADE_OUT_SPEED: f32 = 3.0; // m/s
        let fade = (1.0 - rolling_speed.abs() / FADE_OUT_SPEED).max(0.0);
        -self.low_speed_damping * lateral_velocity * fade
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEG: f32 = std::f32::consts::PI / 180.0;

    fn tire() -> TireParams {
        TireParams::default()
    }

    #[test]
    fn small_slip_slope_equals_cornering_stiffness() {
        let tire = tire();
        let load = 3500.0;
        let slip = 0.001;
        let slope = tire.forces(slip, load, 0.0).lateral / slip;
        let expected = tire.cornering_stiffness(load);
        assert!(
            (slope - expected).abs() / expected < 0.01,
            "slope {slope}, expected {expected}"
        );
    }

    #[test]
    fn lateral_force_peaks_at_road_tire_slip_angle_then_falls_off() {
        let tire = tire();
        let load = tire.nominal_load;
        let (peak_angle, peak_force) = (1..300)
            .map(|i| i as f32 * 0.1 * DEG)
            .map(|a| (a, tire.forces(a, load, 0.0).lateral))
            .fold((0.0, 0.0), |best, x| if x.1 > best.1 { x } else { best });

        assert!(
            (6.0..11.0).contains(&(peak_angle / DEG)),
            "peak at {:.1} deg",
            peak_angle / DEG
        );
        let expected_peak = tire.friction(load) * load;
        assert!((peak_force - expected_peak).abs() / expected_peak < 0.001);
        let sliding = tire.forces(25.0 * DEG, load, 0.0).lateral;
        assert!(sliding < peak_force * 0.97 && sliding > peak_force * 0.8);
    }

    #[test]
    fn heavier_load_grips_proportionally_less() {
        let tire = tire();
        let load = tire.nominal_load;
        let peak = |l: f32| tire.forces(10.0 * DEG, l, 0.0).lateral / l;
        assert!(peak(2.0 * load) < peak(load));
        assert!(peak(0.5 * load) > peak(load));
    }

    #[test]
    fn aligning_torque_opposes_slip_and_fades_before_the_grip_limit() {
        let tire = tire();
        let load = tire.nominal_load;
        let torque = |deg: f32| tire.forces(deg * DEG, load, 0.0).aligning_torque;

        assert!(torque(2.0) < 0.0 && torque(-2.0) > 0.0);
        // The torque peaks well before the lateral force does, which is what
        // makes the steering go light as the front tires approach the limit.
        let (peak_angle, _) = (1..150)
            .map(|i| i as f32 * 0.1)
            .map(|deg| (deg, -torque(deg)))
            .fold((0.0, 0.0), |best, x| if x.1 > best.1 { x } else { best });
        assert!(
            (2.0..6.0).contains(&peak_angle),
            "Mz peak at {peak_angle:.1} deg"
        );
        assert!(torque(tire.trail_zero_slip_angle / DEG).abs() < 1.0);
        assert!(torque(12.0) > -0.2 * torque(-peak_angle).abs());
    }

    #[test]
    fn longitudinal_force_uses_up_lateral_grip() {
        let tire = tire();
        let load = tire.nominal_load;
        let max = tire.friction(load) * load;
        let free = tire.forces(5.0 * DEG, load, 0.0);
        let half = tire.forces(5.0 * DEG, load, 0.5 * max);
        let full = tire.forces(5.0 * DEG, load, 2.0 * max);

        assert_eq!(full.longitudinal, max);
        assert_eq!(full.lateral, 0.0);
        assert!((half.lateral / free.lateral - 0.75_f32.sqrt()).abs() < 1e-4);
    }

    #[test]
    fn unloaded_tire_produces_nothing() {
        assert_eq!(tire().forces(0.1, 0.0, 1000.0), TireForces::default());
    }
}
