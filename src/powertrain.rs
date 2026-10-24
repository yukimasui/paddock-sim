//! Engine, torque converter and six-speed gearbox driving the rear wheels,
//! parameterised per car by a [`GearboxSpec`]. The model is a torque
//! converter automatic like the Aisin 6AT of the Subaru BRZ / Toyota GR86;
//! other cars reuse it with their own ratios and shift points.
//!
//! The torque converter lets the engine run faster than the gearbox input
//! (turbine) and multiplies torque while it slips, so the car can pull away
//! and creep. From second gear upwards a lockup clutch couples engine and
//! turbine directly once moving. Gears are chosen from turbine speed and a
//! smoothed pedal position, with kickdown skipping gears when needed. Like a
//! sporty automatic, it only shifts up while the driver is on the throttle,
//! holds the gear on lift-off, and shifts down earlier under braking,
//! blipping the throttle to match revs.
//!
//! In manual mode the driver shifts with the paddles. The gearbox still
//! blips on downshifts, refuses downshifts that would over-rev the engine,
//! and shifts down by itself only when the revs drop too low, as the BRZ / GR86's
//! automatic does in its manual mode.

use std::f32::consts::TAU;

use crate::engine::{Engine, EngineSpec, FA24};

/// What distinguishes one gearbox and its calibration from another. Speeds
/// are in rpm.
#[derive(Debug)]
pub struct GearboxSpec {
    /// Gear ratios, first to sixth.
    pub ratios: [f32; 6],
    pub final_drive: f32,
    /// Speed at which the stalled converter absorbs the engine's peak torque.
    pub stall_rpm: f32,
    /// Lockup clutch torque capacity, N m.
    pub lockup_capacity: f32,
    /// Lowest turbine speed at which lockup engages.
    pub lockup_min_rpm: f32,
    /// Automatic shift points (turbine speed) with the pedal barely pressed
    /// and floored. In between they follow the square of the pedal, so
    /// gentle driving shifts up early.
    pub upshift_rpm: (f32, f32),
    pub downshift_rpm: (f32, f32),
    /// The automatic shifts up regardless of the pedal above this, to
    /// protect the engine.
    pub over_rev_rpm: f32,
    /// Extra downshift speed at full brake, so the automatic works down
    /// through the gears while braking for a corner.
    pub braking_downshift_rpm: f32,
    /// The automatic never shifts down into a gear that would put the engine
    /// above this.
    pub max_downshift_rpm: f32,
    /// In manual, downshifts that would take the engine above this are refused.
    pub manual_downshift_limit_rpm: f32,
    /// In manual, shifts down by itself below this turbine speed.
    pub manual_min_rpm: f32,
}

/// Aisin six-speed automatic of the Subaru BRZ / Toyota GR86.
pub static AISIN_6AT: GearboxSpec = GearboxSpec {
    ratios: [3.538, 2.060, 1.404, 1.000, 0.713, 0.582],
    final_drive: 4.100,
    stall_rpm: 2500.0,
    lockup_capacity: 450.0,
    lockup_min_rpm: 1300.0,
    upshift_rpm: (1600.0, 6900.0),
    downshift_rpm: (1100.0, 4200.0),
    over_rev_rpm: 7100.0,
    braking_downshift_rpm: 2200.0,
    max_downshift_rpm: 6300.0,
    manual_downshift_limit_rpm: 7200.0,
    manual_min_rpm: 1000.0,
};

/// Lexus LFA six-speed sequential, with approximate published ratios (not
/// verified). The real car uses a single-clutch automated manual; here it
/// is driven through the same converter model, recalibrated for the V10.
pub static LFA_6_SPEED: GearboxSpec = GearboxSpec {
    ratios: [3.231, 2.188, 1.609, 1.233, 0.970, 0.795],
    final_drive: 4.083,
    stall_rpm: 3200.0,
    lockup_capacity: 750.0,
    lockup_min_rpm: 1600.0,
    upshift_rpm: (2000.0, 8800.0),
    downshift_rpm: (1400.0, 5500.0),
    over_rev_rpm: 9200.0,
    braking_downshift_rpm: 3000.0,
    max_downshift_rpm: 8000.0,
    manual_downshift_limit_rpm: 9200.0,
    manual_min_rpm: 1300.0,
};

/// Rolling radius of the rear tires (215/40R18), m. The chassis is shared
/// by every car for now.
pub const WHEEL_RADIUS: f32 = 0.31;
const DRIVELINE_EFFICIENCY: f32 = 0.9;

/// Torque multiplication with the turbine stalled.
const STALL_TORQUE_RATIO: f32 = 2.0;
/// Turbine/impeller speed ratio above which there is no torque multiplication.
const COUPLING_SPEED_RATIO: f32 = 0.85;
/// How sharply the converter stops transmitting torque as the speeds converge.
const CAPACITY_EXPONENT: f32 = 6.0;
/// The converter transmits less when the wheels drive the engine.
const OVERRUN_CAPACITY_SCALE: f32 = 0.5;

/// Lockup clutch torque per rad/s of slip.
const LOCKUP_STIFFNESS: f32 = 150.0;
/// Time for the lockup clutch to reach full capacity, s.
const LOCKUP_ENGAGE_TIME: f32 = 0.3;
/// Lowest gear (index) in which lockup engages.
const LOCKUP_MIN_GEAR: usize = 1;

/// Pedal position below which the driver counts as off the throttle, when
/// the automatic holds its gear instead of shifting up.
const UPSHIFT_MIN_PEDAL: f32 = 0.02;
/// After a shift: lockup stays open this long, s.
const SHIFT_TIME: f32 = 0.35;
/// Time over which a shift moves the ratio from the old gear to the new one,
/// as the shift clutches slip and swap over, s. Changing the ratio instantly
/// would slam the engine's inertia into the converter and the rear tires.
const SHIFT_BLEND: f32 = 0.25;
/// Minimum time between shifts, s.
const SHIFT_INTERVAL: f32 = 0.8;
/// Throttle the ECU blips to match revs on a downshift.
const REV_MATCH_PEDAL: f32 = 0.5;
/// Smoothing of the pedal for shift decisions, so a quick stab doesn't hunt gears, s.
const DEMAND_LAG: f32 = 0.3;

/// Minimum time between paddle shifts, s.
const MANUAL_SHIFT_INTERVAL: f32 = 0.25;
/// Paddle pulls queued while a shift is still under way.
const MAX_QUEUED_SHIFTS: i32 = 3;

const RAD_S_TO_RPM: f32 = 60.0 / TAU;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftMode {
    Automatic,
    Manual,
}

#[derive(Clone, Debug)]
pub struct Powertrain {
    pub engine: Engine,
    pub gearbox: &'static GearboxSpec,
    /// Index into the gear ratios (0 is first gear).
    pub gear: usize,
    /// Lockup clutch engagement, 0 (open) ..= 1 (fully engaged).
    pub lockup: f32,
    pub mode: ShiftMode,
    /// Paddle shifts waiting to happen: positive up, negative down.
    queued_shifts: i32,
    /// Smoothed pedal position used by the shift schedule.
    demand: f32,
    since_shift: f32,
    /// Overall ratio when the current shift began.
    shift_from_ratio: f32,
}

impl Default for Powertrain {
    fn default() -> Self {
        Self::new(&FA24, &AISIN_6AT)
    }
}

impl Powertrain {
    pub fn new(engine: &'static EngineSpec, gearbox: &'static GearboxSpec) -> Self {
        Self {
            engine: Engine::new(engine),
            gearbox,
            gear: 0,
            lockup: 0.0,
            mode: ShiftMode::Manual,
            queued_shifts: 0,
            demand: 0.0,
            since_shift: SHIFT_INTERVAL,
            shift_from_ratio: gearbox.ratios[0] * gearbox.final_drive,
        }
    }

    /// A powertrain already cruising with the driven wheels at `wheel_speed`
    /// (rad/s): in the gear the schedule picks at light throttle, locked up.
    #[cfg(test)]
    pub fn cruising(wheel_speed: f32) -> Self {
        let mut powertrain = Self {
            mode: ShiftMode::Automatic,
            ..Self::default()
        };
        powertrain.demand = 0.2;
        powertrain.gear = (0..powertrain.gearbox.ratios.len())
            .rev()
            .find(|&gear| {
                powertrain.turbine_rpm(wheel_speed, gear) >= powertrain.downshift_rpm(0.0)
            })
            .unwrap_or(0);
        let rpm = powertrain.turbine_rpm(wheel_speed, powertrain.gear);
        if powertrain.gear >= LOCKUP_MIN_GEAR && rpm > powertrain.gearbox.lockup_min_rpm {
            powertrain.lockup = 1.0;
            powertrain.engine.rpm = rpm;
        }
        powertrain
    }

    /// Asks for a paddle shift: positive steps up, negative down. Switches
    /// to manual mode.
    pub fn request_shift(&mut self, steps: i32) {
        self.mode = ShiftMode::Manual;
        self.queued_shifts =
            (self.queued_shifts + steps).clamp(-MAX_QUEUED_SHIFTS, MAX_QUEUED_SHIFTS);
    }

    pub fn set_automatic(&mut self) {
        self.mode = ShiftMode::Automatic;
        self.queued_shifts = 0;
    }

    /// Overall ratio from the gearbox input to the wheels: the current
    /// gear's, or part way there while a shift is under way.
    pub fn ratio(&self) -> f32 {
        let target = self.gearbox.ratios[self.gear] * self.gearbox.final_drive;
        let progress = (self.since_shift / SHIFT_BLEND).clamp(0.0, 1.0);
        let eased = progress * progress * (3.0 - 2.0 * progress);
        self.shift_from_ratio + (target - self.shift_from_ratio) * eased
    }

    /// Starts a shift into `gear`.
    fn engage(&mut self, gear: usize) {
        self.shift_from_ratio = self.ratio();
        self.gear = gear;
        self.since_shift = 0.0;
    }

    /// Gearbox input speed in `gear` with the wheels at `wheel_speed` rad/s, rpm.
    fn turbine_rpm(&self, wheel_speed: f32, gear: usize) -> f32 {
        wheel_speed * self.gearbox.ratios[gear] * self.gearbox.final_drive * RAD_S_TO_RPM
    }

    /// Advances by `dt` seconds with the throttle and brake pedals at
    /// `pedal` and `brake` (0..=1) and the driven wheels turning at
    /// `wheel_speed` rad/s. Returns the drive torque at the wheels, N m, for
    /// both driven wheels together.
    pub fn update(&mut self, pedal: f32, brake: f32, wheel_speed: f32, dt: f32) -> f32 {
        self.demand += (pedal - self.demand) * (1.0 - (-dt / DEMAND_LAG).exp());
        self.since_shift += dt;
        self.choose_gear(pedal, brake, wheel_speed);

        let ratio = self.ratio();
        let turbine = wheel_speed * ratio;
        let engine = self.engine.angular_velocity();
        let capacity = self.engine.peak_torque() / self.gearbox.stall_rpm.powi(2);
        let (impeller_torque, turbine_torque) = converter(engine, turbine, capacity);

        let lockup_wanted = self.gear >= LOCKUP_MIN_GEAR
            && turbine * RAD_S_TO_RPM > self.gearbox.lockup_min_rpm
            && self.since_shift > SHIFT_TIME;
        self.lockup = if lockup_wanted {
            (self.lockup + dt / LOCKUP_ENGAGE_TIME).min(1.0)
        } else {
            0.0
        };
        let capacity = self.gearbox.lockup_capacity * self.lockup;
        let lockup_torque = (LOCKUP_STIFFNESS * (engine - turbine)).clamp(-capacity, capacity);

        // On a downshift the ECU blips the throttle to bring the engine up
        // to the new gear's speed instead of dragging it up through the
        // converter, which would jerk the rear wheels.
        let rev_matching = self.since_shift < SHIFT_TIME && turbine > engine;
        let engine_pedal = if rev_matching {
            pedal.max(REV_MATCH_PEDAL)
        } else {
            pedal
        };
        self.engine
            .update_with_load(engine_pedal, impeller_torque + lockup_torque, dt);
        (turbine_torque + lockup_torque) * ratio * DRIVELINE_EFFICIENCY
    }

    fn upshift_rpm(&self) -> f32 {
        let (light, floored) = self.gearbox.upshift_rpm;
        light + (floored - light) * self.demand.clamp(0.0, 1.0).powi(2)
    }

    fn downshift_rpm(&self, brake: f32) -> f32 {
        let (light, floored) = self.gearbox.downshift_rpm;
        light
            + (floored - light) * self.demand.clamp(0.0, 1.0).powi(2)
            + self.gearbox.braking_downshift_rpm * brake.clamp(0.0, 1.0)
    }

    fn choose_gear(&mut self, pedal: f32, brake: f32, wheel_speed: f32) {
        match self.mode {
            ShiftMode::Automatic => self.choose_gear_automatically(pedal, brake, wheel_speed),
            ShiftMode::Manual => self.shift_manually(wheel_speed),
        }
    }

    fn shift_manually(&mut self, wheel_speed: f32) {
        if self.queued_shifts != 0 {
            if self.since_shift < MANUAL_SHIFT_INTERVAL {
                return;
            }
            let step = self.queued_shifts.signum();
            self.queued_shifts -= step;
            let Some(gear) = self.gear.checked_add_signed(step as isize) else {
                return;
            };
            if gear >= self.gearbox.ratios.len() {
                return;
            }
            let over_revs = step < 0
                && self.turbine_rpm(wheel_speed, gear) > self.gearbox.manual_downshift_limit_rpm;
            if !over_revs {
                self.engage(gear);
            }
            return;
        }
        // Don't lug the engine or stall the pull-away: drop a gear when the
        // revs fall too low.
        if self.gear > 0
            && self.since_shift >= SHIFT_INTERVAL
            && self.turbine_rpm(wheel_speed, self.gear) < self.gearbox.manual_min_rpm
        {
            self.engage(self.gear - 1);
        }
    }

    fn choose_gear_automatically(&mut self, pedal: f32, brake: f32, wheel_speed: f32) {
        if self.since_shift < SHIFT_INTERVAL {
            return;
        }
        let rpm = self.turbine_rpm(wheel_speed, self.gear);
        let top = self.gearbox.ratios.len() - 1;

        let wants_upshift = (pedal > UPSHIFT_MIN_PEDAL && rpm > self.upshift_rpm())
            || rpm > self.gearbox.over_rev_rpm;
        let target = if wants_upshift && self.gear < top {
            Some(self.gear + 1)
        } else if rpm < self.downshift_rpm(brake) && self.gear > 0 {
            // Straight to the highest gear that gives enough revs (kickdown
            // can skip several), as long as the engine can take it.
            let down = self.downshift_rpm(brake);
            let fits =
                |gear: usize| self.turbine_rpm(wheel_speed, gear) <= self.gearbox.max_downshift_rpm;
            (0..self.gear)
                .rev()
                .find(|&gear| fits(gear) && self.turbine_rpm(wheel_speed, gear) >= down)
                .or_else(|| fits(self.gear - 1).then_some(self.gear - 1))
        } else {
            None
        };
        if let Some(gear) = target {
            self.engage(gear);
        }
    }
}

/// Torque converter impeller and turbine torques (N m) for impeller (engine)
/// and turbine speeds in rad/s, with `capacity` the impeller torque per rpm^2
/// when stalled. The impeller torque loads the engine; the turbine torque
/// drives the gearbox.
fn converter(impeller: f32, turbine: f32, capacity: f32) -> (f32, f32) {
    if turbine <= impeller {
        // The engine drives. A turbine turning backwards counts as stalled.
        if impeller <= 0.0 {
            return (0.0, 0.0);
        }
        let speed_ratio = (turbine / impeller).max(0.0);
        let rpm = impeller * RAD_S_TO_RPM;
        let torque = capacity * (1.0 - speed_ratio.powf(CAPACITY_EXPONENT)) * rpm * rpm;
        let multiplication = if speed_ratio < COUPLING_SPEED_RATIO {
            STALL_TORQUE_RATIO - (STALL_TORQUE_RATIO - 1.0) * speed_ratio / COUPLING_SPEED_RATIO
        } else {
            1.0
        };
        (torque, torque * multiplication)
    } else {
        // The wheels drive the engine (engine braking through the fluid).
        let speed_ratio = (impeller / turbine).max(0.0);
        let rpm = turbine * RAD_S_TO_RPM;
        let torque = -OVERRUN_CAPACITY_SCALE
            * capacity
            * (1.0 - speed_ratio.powf(CAPACITY_EXPONENT))
            * rpm
            * rpm;
        (torque, torque)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    #[test]
    fn stalled_converter_absorbs_full_torque_at_stall_speed() {
        let stall = 2500.0 / RAD_S_TO_RPM;
        let (impeller, turbine) = converter(stall, 0.0, 250.0 / 2500.0_f32.powi(2));
        assert!((impeller - 250.0).abs() < 1.0);
        assert!(
            (turbine - 500.0).abs() < 2.0,
            "torque multiplied to {turbine}"
        );
    }

    #[test]
    fn converter_transmits_nothing_at_equal_speeds() {
        let (impeller, turbine) = converter(300.0, 300.0, 250.0 / 2500.0_f32.powi(2));
        assert!(impeller.abs() < 1e-3 && turbine.abs() < 1e-3);
    }

    #[test]
    fn idling_in_drive_creeps_against_a_stationary_car() {
        let mut powertrain = Powertrain::default();
        let mut torque = 0.0;
        for _ in 0..2000 {
            torque = powertrain.update(0.0, 0.0, 0.0, DT);
        }
        assert_eq!(powertrain.gear, 0);
        assert!(
            (powertrain.engine.rpm - FA24.idle_rpm).abs() < 150.0,
            "idle {:.0}",
            powertrain.engine.rpm
        );
        assert!(torque > 200.0, "creep torque {torque:.0} N m");
    }

    #[test]
    fn engine_flares_to_near_stall_speed_when_floored_from_rest() {
        let mut powertrain = Powertrain::default();
        for _ in 0..1000 {
            powertrain.update(1.0, 0.0, 0.0, DT);
        }
        // The engine settles where its net torque (less than the 250 N m
        // peak at these revs) matches what the stalled converter absorbs.
        let rpm = powertrain.engine.rpm;
        assert!((2000.0..2600.0).contains(&rpm), "flare to {rpm:.0} rpm");
    }

    #[test]
    fn cruising_picks_a_tall_gear_and_locks_up() {
        let wheel_speed = (100.0 / 3.6) / WHEEL_RADIUS;
        let powertrain = Powertrain::cruising(wheel_speed);
        assert_eq!(powertrain.gear, 5);
        assert_eq!(powertrain.lockup, 1.0);
        assert!((1900.0..2200.0).contains(&powertrain.engine.rpm));
    }

    #[test]
    fn kickdown_skips_straight_to_a_lower_gear() {
        let wheel_speed = (100.0 / 3.6) / WHEEL_RADIUS;
        let mut powertrain = Powertrain::cruising(wheel_speed);
        for _ in 0..1500 {
            powertrain.update(1.0, 0.0, wheel_speed, DT);
        }
        assert_eq!(powertrain.gear, 2, "should be in third");
    }

    #[test]
    fn lifting_off_holds_the_gear() {
        let wheel_speed = (110.0 / 3.6) / WHEEL_RADIUS;
        let mut powertrain = Powertrain::cruising(wheel_speed);
        for _ in 0..1500 {
            powertrain.update(1.0, 0.0, wheel_speed, DT);
        }
        let gear = powertrain.gear;
        for _ in 0..3000 {
            powertrain.update(0.0, 0.0, wheel_speed, DT);
        }
        assert_eq!(powertrain.gear, gear, "shifted on lift-off");
    }

    #[test]
    fn braking_shifts_down_with_a_throttle_blip() {
        // Braking from 100 km/h in sixth: each downshift should raise the
        // engine speed, and it should end up in a low gear at a lively rpm.
        let mut powertrain = Powertrain::cruising((100.0 / 3.6) / WHEEL_RADIUS);
        let mut speed = 100.0 / 3.6;
        let mut downshifts = 0;
        let mut gear = powertrain.gear;
        let mut rpm_at_shift = None;
        for _ in 0..6000 {
            speed = (speed - 6.0 * DT).max(5.0); // about 0.6 g
            powertrain.update(0.0, 0.6, speed / WHEEL_RADIUS, DT);
            if powertrain.gear < gear {
                downshifts += 1;
                rpm_at_shift = Some(powertrain.engine.rpm);
            }
            if let Some(before) = rpm_at_shift
                && powertrain.since_shift > 0.3
            {
                assert!(
                    powertrain.engine.rpm > before + 300.0,
                    "no blip on downshift"
                );
                rpm_at_shift = None;
            }
            gear = powertrain.gear;
        }
        assert!(downshifts >= 3, "only {downshifts} downshifts");
        assert!(
            powertrain.gear <= 1,
            "ended in gear {}",
            powertrain.gear + 1
        );
    }

    #[test]
    fn paddles_shift_one_gear_at_a_time_and_queue() {
        let wheel_speed = (60.0 / 3.6) / WHEEL_RADIUS;
        let mut powertrain = Powertrain::default();
        assert_eq!(powertrain.mode, ShiftMode::Manual);
        powertrain.request_shift(2);
        for _ in 0..100 {
            powertrain.update(0.3, 0.0, wheel_speed, DT);
        }
        assert_eq!(
            powertrain.gear, 1,
            "first shift should wait out the interval"
        );
        for _ in 0..300 {
            powertrain.update(0.3, 0.0, wheel_speed, DT);
        }
        assert_eq!(powertrain.gear, 2);
    }

    #[test]
    fn manual_holds_the_gear_at_the_limiter() {
        let mut powertrain = Powertrain::default();
        // First gear at about 7000 rpm, foot flat: no automatic upshift.
        let wheel_speed = 7000.0 / RAD_S_TO_RPM / (AISIN_6AT.ratios[0] * AISIN_6AT.final_drive);
        for _ in 0..3000 {
            powertrain.update(1.0, 0.0, wheel_speed, DT);
        }
        assert_eq!(powertrain.gear, 0);
    }

    #[test]
    fn manual_refuses_downshifts_that_would_over_rev() {
        // Third at 5500 rpm; second would be about 8070 rpm.
        let wheel_speed = 5500.0 / RAD_S_TO_RPM / (AISIN_6AT.ratios[2] * AISIN_6AT.final_drive);
        let mut powertrain = Powertrain {
            gear: 2,
            ..Powertrain::default()
        };
        powertrain.request_shift(-1);
        for _ in 0..500 {
            powertrain.update(0.0, 0.0, wheel_speed, DT);
        }
        assert_eq!(powertrain.gear, 2);
    }
}
