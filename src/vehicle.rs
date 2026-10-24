//! Planar vehicle dynamics using a dynamic bicycle model with per-wheel loads.
//!
//! Both wheels of an axle share one slip angle, as in a bicycle model, but
//! each wheel has its own vertical load so that longitudinal and lateral load
//! transfer change the grip of each tire. Slip angles are transient (see
//! `tire`), which keeps the model valid from standstill to high speed without
//! switching to a separate low-speed model. Load transfer follows the body
//! accelerations through a first-order lag that stands in for suspension
//! dynamics. The state is integrated at a fixed 1 kHz step.
//!
//! Internally the vehicle frame follows ISO 8855: x forward, y left, yaw
//! positive counter-clockwise (a left turn).

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;

use crate::menu::GameState;
#[cfg(test)]
use crate::powertrain::ShiftMode;
use crate::powertrain::{Powertrain, WHEEL_RADIUS};
use crate::surface::Surface;
use crate::tire::{TireForces, TireParams};
use crate::wheel::{Paddles, Pedals, Steering};

const PHYSICS_HZ: f64 = 1000.0;
const GRAVITY: f32 = 9.81;
const AIR_DENSITY: f32 = 1.225;

/// Lower bound on the rolling speed used to compute steady-state slip.
const MIN_ROLLING_SPEED: f32 = 0.01;

/// Traction control keeps the driven tires' combined use of their grip
/// below this share, leaving about half of the sideways grip for cornering.
const TRACTION_TARGET: f32 = 0.85;
/// Stability control steps in once the rear tires slip this much more than
/// the fronts (rad), the start of power oversteer.
const OVERSTEER_THRESHOLD: f32 = 0.01;
/// How hard it cuts torque per radian of excess oversteer, per second.
const OVERSTEER_CUT_RATE: f32 = 400.0;
/// Time for traction control to hand back full engine torque, s.
const TRACTION_RECOVERY: f32 = 0.3;

/// Indices into per-wheel arrays.
pub const FRONT_LEFT: usize = 0;
pub const FRONT_RIGHT: usize = 1;
pub const REAR_LEFT: usize = 2;
pub const REAR_RIGHT: usize = 3;

pub struct VehiclePlugin;

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Time::<Fixed>::from_hz(PHYSICS_HZ))
            .init_resource::<VehicleParams>()
            .init_resource::<Vehicle>()
            .init_resource::<Ground>()
            .add_systems(
                FixedUpdate,
                step_vehicle.run_if(in_state(GameState::Driving)),
            )
            .add_systems(Update, (shift_on_request, toggle_traction_control));
    }
}

/// Physical parameters, roughly a front-engine rear-drive compact sports car.
#[derive(Resource, Debug)]
pub struct VehicleParams {
    pub mass: f32,             // kg
    pub yaw_inertia: f32,      // kg m^2
    pub cg_to_front_axle: f32, // m
    pub cg_to_rear_axle: f32,  // m
    pub cg_height: f32,        // m
    pub front_track: f32,      // m
    pub rear_track: f32,       // m
    /// Share of lateral load transfer taken by the front axle (roll stiffness
    /// distribution). Higher values make the car understeer more at the limit.
    pub front_roll_stiffness_share: f32,
    /// Time constant of the lag between acceleration and load transfer, s.
    pub load_transfer_time_constant: f32,
    pub front_tire: TireParams,
    pub rear_tire: TireParams,
    pub max_brake_force: f32, // N, all wheels
    pub front_brake_bias: f32,
    pub drag_area: f32, // Cd * frontal area, m^2
    /// Steering wheel angle divided by road wheel angle.
    pub steering_ratio: f32,
    /// Distance the front contact patch trails behind the steering axis
    /// (caster trail), m. Lateral force acting there self-centers the steering.
    pub mechanical_trail: f32,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            mass: 1270.0,
            yaw_inertia: 2000.0,
            cg_to_front_axle: 1.21,
            cg_to_rear_axle: 1.365,
            cg_height: 0.46,
            front_track: 1.52,
            rear_track: 1.55,
            front_roll_stiffness_share: 0.65,
            load_transfer_time_constant: 0.08,
            front_tire: TireParams::default(),
            // Wider rear tires: stiffer in cornering and a little more grip.
            rear_tire: TireParams {
                cornering_stiffness_ratio: 20.0,
                peak_friction: 1.07,
                ..default()
            },
            max_brake_force: 12_500.0,
            front_brake_bias: 0.72,
            drag_area: 0.6,
            steering_ratio: 15.0,
            mechanical_trail: 0.025,
        }
    }
}

impl VehicleParams {
    /// The Lexus LFA's chassis: heavier, with a front-mid V10 and a rear
    /// transaxle for a 48:52 weight split, wide tires (265 front, 305
    /// rear) and carbon-ceramic brakes. The body drawn is still the shared one.
    pub fn lfa() -> Self {
        let defaults = Self::default();
        Self {
            mass: 1480.0,
            yaw_inertia: 2500.0,
            cg_to_front_axle: 1.355,
            cg_to_rear_axle: 1.25,
            cg_height: 0.45,
            front_track: 1.58,
            rear_track: 1.57,
            front_roll_stiffness_share: 0.6,
            front_tire: TireParams {
                nominal_load: 3500.0,
                peak_friction: 1.1,
                load_sensitivity: 0.12,
                cornering_stiffness_ratio: 19.0,
                ..defaults.front_tire
            },
            rear_tire: TireParams {
                nominal_load: 4000.0,
                peak_friction: 1.12,
                load_sensitivity: 0.12,
                cornering_stiffness_ratio: 22.0,
                ..defaults.rear_tire
            },
            max_brake_force: 17_000.0,
            front_brake_bias: 0.65,
            drag_area: 0.62,
            ..defaults
        }
    }

    pub fn wheelbase(&self) -> f32 {
        self.cg_to_front_axle + self.cg_to_rear_axle
    }

    /// Static vertical loads (front axle, rear axle) in N.
    pub fn static_axle_loads(&self) -> (f32, f32) {
        let weight = self.mass * GRAVITY;
        let wheelbase = self.wheelbase();
        (
            weight * self.cg_to_rear_axle / wheelbase,
            weight * self.cg_to_front_axle / wheelbase,
        )
    }

    /// Vertical load on each wheel (N) for body accelerations in the vehicle
    /// frame (x forward, y left, m/s^2). A wheel can lift but never pull down.
    pub fn wheel_loads(&self, acceleration: Vec2) -> [f32; 4] {
        let (static_front, static_rear) = self.static_axle_loads();
        let longitudinal_transfer = self.mass * acceleration.x * self.cg_height / self.wheelbase();
        let front = static_front - longitudinal_transfer;
        let rear = static_rear + longitudinal_transfer;

        // Cornering to the left (positive y) loads the right-hand wheels.
        let lateral_moment = self.mass * acceleration.y * self.cg_height;
        let front_transfer = self.front_roll_stiffness_share * lateral_moment / self.front_track;
        let rear_transfer =
            (1.0 - self.front_roll_stiffness_share) * lateral_moment / self.rear_track;

        let mut loads = [0.0; 4];
        loads[FRONT_LEFT] = front * 0.5 - front_transfer;
        loads[FRONT_RIGHT] = front * 0.5 + front_transfer;
        loads[REAR_LEFT] = rear * 0.5 - rear_transfer;
        loads[REAR_RIGHT] = rear * 0.5 + rear_transfer;
        loads.map(|load| load.max(0.0))
    }
}

/// Vehicle state. Vehicle-frame quantities follow ISO 8855 (y left, yaw CCW).
#[derive(Resource, Debug)]
pub struct Vehicle {
    /// Center of gravity in world space, m.
    pub position: Vec2,
    /// Heading in world space, rad, counter-clockwise from +X.
    pub heading: f32,
    /// Velocity in the vehicle frame (x forward, y left), m/s.
    pub velocity: Vec2,
    /// Yaw rate, rad/s, positive counter-clockwise.
    pub yaw_rate: f32,
    /// Road wheel angle, rad; positive means steered to the right
    /// (same convention as the steering wheel input).
    pub road_wheel_angle: f32,
    /// Acceleration felt at the CG in the vehicle frame (x forward, y left), m/s^2.
    pub acceleration: Vec2,
    /// `acceleration` delayed by the suspension lag; drives load transfer.
    pub load_transfer_acceleration: Vec2,
    /// Vertical load per wheel, N, indexed by `FRONT_LEFT` etc.
    pub wheel_loads: [f32; 4],
    /// Tire forces per wheel in each wheel's own frame.
    pub tire_forces: [TireForces; 4],
    pub front_slip_angle: f32, // rad
    pub rear_slip_angle: f32,  // rad
    /// Transient lateral slip state per axle (tangent of the slip angle).
    pub front_lateral_slip: f32,
    pub rear_lateral_slip: f32,
    /// Engine and automatic gearbox driving the rear wheels.
    pub powertrain: Powertrain,
    pub traction_control: TractionControl,
}

/// Traction and stability control: trims engine torque when the driven
/// tires near the end of their grip, or when the rear starts to slide wider
/// than the front, as both cars' systems do.
#[derive(Clone, Copy, Debug)]
pub struct TractionControl {
    pub enabled: bool,
    /// Share of engine torque currently allowed, 0..=1.
    pub torque_limit: f32,
}

impl Default for TractionControl {
    fn default() -> Self {
        Self {
            enabled: true,
            torque_limit: 1.0,
        }
    }
}

impl TractionControl {
    /// Adjusts the torque allowed next step from the `drive` force the rear
    /// tires are being asked for, the `lateral` force they already carry and
    /// their total `grip` (N).
    /// `oversteer` is how much more the rear tires slip than the fronts, rad.
    fn update(&mut self, drive: f32, lateral: f32, grip: f32, oversteer: f32, dt: f32) {
        if !self.enabled {
            self.torque_limit = 1.0;
            return;
        }
        if drive <= 0.0 || grip <= 0.0 {
            self.torque_limit = (self.torque_limit + dt / TRACTION_RECOVERY).min(1.0);
            return;
        }
        let mut limit = if drive.hypot(lateral) > TRACTION_TARGET * grip {
            // Allow only the drive that keeps the combined use on target.
            let allowed = ((TRACTION_TARGET * grip).powi(2) - lateral * lateral)
                .max(0.0)
                .sqrt();
            self.torque_limit * allowed / drive
        } else {
            self.torque_limit + dt / TRACTION_RECOVERY
        };
        if oversteer > OVERSTEER_THRESHOLD {
            limit -= OVERSTEER_CUT_RATE * (oversteer - OVERSTEER_THRESHOLD) * dt;
        }
        self.torque_limit = limit.clamp(0.0, 1.0);
    }

    /// True while it is holding torque back.
    pub fn intervening(&self) -> bool {
        self.torque_limit < 0.99
    }
}

impl Default for Vehicle {
    fn default() -> Self {
        Self {
            position: Vec2::ZERO,
            heading: FRAC_PI_2,
            velocity: Vec2::ZERO,
            yaw_rate: 0.0,
            road_wheel_angle: 0.0,
            acceleration: Vec2::ZERO,
            load_transfer_acceleration: Vec2::ZERO,
            wheel_loads: VehicleParams::default().wheel_loads(Vec2::ZERO),
            tire_forces: [TireForces::default(); 4],
            front_slip_angle: 0.0,
            rear_slip_angle: 0.0,
            front_lateral_slip: 0.0,
            rear_lateral_slip: 0.0,
            powertrain: Powertrain::default(),
            traction_control: TractionControl::default(),
        }
    }
}

impl Vehicle {
    /// Forward speed, m/s.
    pub fn speed(&self) -> f32 {
        self.velocity.x
    }

    pub fn speed_kmh(&self) -> f32 {
        self.speed() * 3.6
    }

    /// Unit vector pointing forward in world space.
    pub fn forward(&self) -> Vec2 {
        Vec2::from_angle(self.heading)
    }

    /// Contact patch positions in world space, indexed by `FRONT_LEFT` etc.
    pub fn wheel_positions(&self, params: &VehicleParams) -> [Vec2; 4] {
        let forward = self.forward();
        let left = forward.perp();
        let front = self.position + forward * params.cg_to_front_axle;
        let rear = self.position - forward * params.cg_to_rear_axle;
        let mut positions = [Vec2::ZERO; 4];
        positions[FRONT_LEFT] = front + left * params.front_track * 0.5;
        positions[FRONT_RIGHT] = front - left * params.front_track * 0.5;
        positions[REAR_LEFT] = rear + left * params.rear_track * 0.5;
        positions[REAR_RIGHT] = rear - left * params.rear_track * 0.5;
        positions
    }

    /// Sum of tire forces on the front axle, in the front wheels' frame.
    pub fn front_axle_forces(&self) -> TireForces {
        sum_forces(self.tire_forces[FRONT_LEFT], self.tire_forces[FRONT_RIGHT])
    }

    /// Torque the front tires apply to the steering wheel through the
    /// steering gear, N m; positive turns the steering wheel clockwise.
    /// As in a car without power steering, friction or column inertia.
    pub fn steering_column_torque(&self, params: &VehicleParams) -> f32 {
        let front = self.front_axle_forces();
        // Moment about the steering axes (ISO, positive steers left): the
        // tires' own aligning torque plus the lateral force pushing on the
        // contact patches, which sit the mechanical trail behind the axes.
        let steering_axis_moment = front.aligning_torque - params.mechanical_trail * front.lateral;
        // Steering the road wheels left turns the steering wheel counter-clockwise.
        -steering_axis_moment / params.steering_ratio
    }

    /// Sum of tire forces on the rear axle.
    pub fn rear_axle_forces(&self) -> TireForces {
        sum_forces(self.tire_forces[REAR_LEFT], self.tire_forces[REAR_RIGHT])
    }
}

fn sum_forces(a: TireForces, b: TireForces) -> TireForces {
    TireForces {
        longitudinal: a.longitudinal + b.longitudinal,
        lateral: a.lateral + b.lateral,
        aligning_torque: a.aligning_torque + b.aligning_torque,
    }
}

/// The ground under the car, kept up to date by whatever owns it (the track).
#[derive(Resource, Clone, Copy, Debug)]
pub struct Ground {
    /// Surface under each wheel, indexed by `FRONT_LEFT` etc.
    pub surfaces: [Surface; 4],
    /// Slope as a gradient on the ground plane: how much the ground rises
    /// per metre east and north.
    pub slope: Vec2,
}

impl Default for Ground {
    fn default() -> Self {
        Self::on(Surface::ASPHALT)
    }
}

impl Ground {
    /// Level ground with every wheel on `surface`.
    pub fn on(surface: Surface) -> Self {
        Self {
            surfaces: [surface; 4],
            slope: Vec2::ZERO,
        }
    }
}

/// Paddles (or E and Q) shift up and down in manual mode; A returns to
/// automatic.
fn shift_on_request(
    paddles: Res<Paddles>,
    keys: Res<ButtonInput<KeyCode>>,
    mut seen: Local<Option<Paddles>>,
    mut vehicle: ResMut<Vehicle>,
) {
    let previous = seen.unwrap_or(*paddles);
    *seen = Some(*paddles);
    let mut steps = paddles.upshifts.wrapping_sub(previous.upshifts) as i32
        - paddles.downshifts.wrapping_sub(previous.downshifts) as i32;
    if keys.just_pressed(KeyCode::KeyE) {
        steps += 1;
    }
    if keys.just_pressed(KeyCode::KeyQ) {
        steps -= 1;
    }
    if steps != 0 {
        vehicle.powertrain.request_shift(steps);
    }
    if keys.just_pressed(KeyCode::KeyA) {
        vehicle.powertrain.set_automatic();
    }
}

/// V switches traction control on and off.
fn toggle_traction_control(keys: Res<ButtonInput<KeyCode>>, mut vehicle: ResMut<Vehicle>) {
    if keys.just_pressed(KeyCode::KeyV) {
        let control = &mut vehicle.traction_control;
        control.enabled = !control.enabled;
    }
}

/// Driver inputs for one physics step.
#[derive(Clone, Copy, Debug, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    /// Road wheel angle, rad; positive means steered to the right.
    pub road_wheel_angle: f32,
}

pub fn step_vehicle(
    time: Res<Time>,
    params: Res<VehicleParams>,
    pedals: Res<Pedals>,
    steering: Res<Steering>,
    ground: Res<Ground>,
    mut vehicle: ResMut<Vehicle>,
) {
    let controls = Controls {
        throttle: pedals.throttle,
        brake: pedals.brake,
        road_wheel_angle: steering.angle_deg().to_radians() / params.steering_ratio,
    };
    vehicle.integrate(&params, controls, *ground, time.delta_secs());
}

impl Vehicle {
    /// Advances the state by `dt` seconds on `ground`.
    pub fn integrate(&mut self, p: &VehicleParams, controls: Controls, ground: Ground, dt: f32) {
        let surfaces = ground.surfaces;
        let a = p.cg_to_front_axle;
        let b = p.cg_to_rear_axle;

        self.road_wheel_angle = controls.road_wheel_angle;
        // ISO convention: positive steer turns left.
        let delta = -controls.road_wheel_angle;
        let (sin_delta, cos_delta) = delta.sin_cos();

        let Vec2 { x: vx, y: vy } = self.velocity;
        let r = self.yaw_rate;

        // Contact patch velocities, each in its own wheel's frame.
        let front_axle_lateral = vy + a * r;
        let front_rolling = vx * cos_delta + front_axle_lateral * sin_delta;
        let front_sliding = -vx * sin_delta + front_axle_lateral * cos_delta;
        let rear_rolling = vx;
        let rear_sliding = vy - b * r;

        // The rear wheels roll with the car (there is no wheel spin model),
        // so they turn at the rear axle's rolling speed.
        self.powertrain.engine.torque_limit = self.traction_control.torque_limit;
        let mut drive = self.powertrain.update(
            controls.throttle,
            controls.brake,
            rear_rolling / WHEEL_RADIUS,
            dt,
        ) / WHEEL_RADIUS;
        // Gravity pulls the car down the slope: for road grades the
        // downhill acceleration is g times the grade.
        let forward = self.forward();
        let downhill = -GRAVITY * ground.slope;
        let mut gravity_x = p.mass * downhill.dot(forward);
        let gravity_y = p.mass * downhill.dot(forward.perp());
        // Brakes and rolling resistance oppose the direction of travel. At a
        // standstill the brakes hold the car against drive torque (such as
        // the torque converter's creep) and the slope, up to their capacity.
        let travel_direction = if vx != 0.0 { vx.signum() } else { 0.0 };
        if travel_direction == 0.0
            && (drive + gravity_x).abs() <= controls.brake * p.max_brake_force
        {
            drive = 0.0;
            gravity_x = 0.0;
        }
        let brake = travel_direction * controls.brake * p.max_brake_force;
        let front_request = -brake * p.front_brake_bias * 0.5;
        let rear_request = (drive - brake * (1.0 - p.front_brake_bias)) * 0.5;

        let loads = p.wheel_loads(self.load_transfer_acceleration);
        let front_slip = self.front_lateral_slip.atan();
        let rear_slip = self.rear_lateral_slip.atan();
        let tire_on = |wheel: usize| {
            let tire = if wheel == FRONT_LEFT || wheel == FRONT_RIGHT {
                &p.front_tire
            } else {
                &p.rear_tire
            };
            tire.with_grip(surfaces[wheel].grip)
        };
        let front_wheel = |wheel: usize| {
            wheel_forces(
                &tire_on(wheel),
                front_slip,
                loads[wheel],
                front_request,
                front_rolling,
                front_sliding,
            )
        };
        let rear_wheel = |wheel: usize| {
            wheel_forces(
                &tire_on(wheel),
                rear_slip,
                loads[wheel],
                rear_request,
                rear_rolling,
                rear_sliding,
            )
        };
        let mut tires = [TireForces::default(); 4];
        tires[FRONT_LEFT] = front_wheel(FRONT_LEFT);
        tires[FRONT_RIGHT] = front_wheel(FRONT_RIGHT);
        tires[REAR_LEFT] = rear_wheel(REAR_LEFT);
        tires[REAR_RIGHT] = rear_wheel(REAR_RIGHT);
        let front = sum_forces(tires[FRONT_LEFT], tires[FRONT_RIGHT]);
        let rear = sum_forces(tires[REAR_LEFT], tires[REAR_RIGHT]);

        let rear_grip: f32 = [REAR_LEFT, REAR_RIGHT]
            .into_iter()
            .map(|wheel| tire_on(wheel).friction(loads[wheel]) * loads[wheel])
            .sum();
        // Oversteer: the rear sliding wider than the front, in the same direction.
        let oversteer = if front_slip * rear_slip >= 0.0 {
            rear_slip.abs() - front_slip.abs()
        } else {
            rear_slip.abs()
        };
        self.traction_control
            .update(drive, rear.lateral, rear_grip, oversteer, dt);

        let rolling_resistance: f32 = (0..4)
            .map(|wheel| loads[wheel] * surfaces[wheel].rolling_resistance)
            .sum();
        let resistance =
            travel_direction * (rolling_resistance + 0.5 * AIR_DENSITY * p.drag_area * vx * vx);

        // Body-frame equations of motion. Uneven left/right longitudinal
        // forces (e.g. a lightly loaded inner wheel reaching its grip limit)
        // and the tires' aligning torques also turn the car.
        let force_x = rear.longitudinal + front.longitudinal * cos_delta
            - front.lateral * sin_delta
            - resistance
            + gravity_x;
        let force_y =
            rear.lateral + front.lateral * cos_delta + front.longitudinal * sin_delta + gravity_y;
        let longitudinal_imbalance = (tires[FRONT_RIGHT].longitudinal
            - tires[FRONT_LEFT].longitudinal)
            * p.front_track
            * 0.5
            + (tires[REAR_RIGHT].longitudinal - tires[REAR_LEFT].longitudinal) * p.rear_track * 0.5;
        let yaw_moment = a * (front.lateral * cos_delta + front.longitudinal * sin_delta)
            - b * rear.lateral
            + longitudinal_imbalance
            + front.aligning_torque
            + rear.aligning_torque;

        let mut new_vx = vx + (force_x / p.mass + vy * r) * dt;
        // Brakes and rolling resistance can stop the car but never push it
        // backwards. Rotation (the vy * r term) can still carry the car
        // through zero, as in a spin.
        if travel_direction != 0.0 && new_vx.signum() != travel_direction {
            let carried_by_rotation = vx + (vy * r) * dt;
            if carried_by_rotation.signum() == travel_direction || carried_by_rotation == 0.0 {
                new_vx = 0.0;
            }
        }
        let new_vy = vy + (force_y / p.mass - vx * r) * dt;
        let new_r = r + yaw_moment / p.yaw_inertia * dt;

        // Acceleration felt by the body (specific force in the vehicle
        // frame), which leaves out gravity; on a slope, this is what shifts
        // weight towards the downhill axle.
        self.acceleration = Vec2::new(
            (new_vx - vx) / dt - vy * r - gravity_x / p.mass,
            (force_y - gravity_y) / p.mass,
        );
        let lag = dt / (p.load_transfer_time_constant + dt);
        self.load_transfer_acceleration +=
            (self.acceleration - self.load_transfer_acceleration) * lag;
        self.wheel_loads = loads;

        let axle_grip =
            |left: usize, right: usize| (surfaces[left].grip + surfaces[right].grip) * 0.5;
        self.front_lateral_slip = advance_slip(
            &p.front_tire.with_grip(axle_grip(FRONT_LEFT, FRONT_RIGHT)),
            self.front_lateral_slip,
            (loads[FRONT_LEFT] + loads[FRONT_RIGHT]) * 0.5,
            front_rolling,
            front_sliding,
            dt,
        );
        self.rear_lateral_slip = advance_slip(
            &p.rear_tire.with_grip(axle_grip(REAR_LEFT, REAR_RIGHT)),
            self.rear_lateral_slip,
            (loads[REAR_LEFT] + loads[REAR_RIGHT]) * 0.5,
            rear_rolling,
            rear_sliding,
            dt,
        );

        self.velocity = Vec2::new(new_vx, new_vy);
        self.yaw_rate = new_r;
        self.front_slip_angle = front_slip;
        self.rear_slip_angle = rear_slip;
        self.tire_forces = tires;

        self.heading += new_r * dt;
        let forward = self.forward();
        let left = forward.perp();
        self.position += (forward * new_vx + left * new_vy) * dt;
    }
}

/// Forces of one wheel, adding carcass damping near standstill. Velocities
/// are of the contact patch along (`rolling`) and across (`sliding`) the wheel.
fn wheel_forces(
    tire: &TireParams,
    slip_angle: f32,
    load: f32,
    requested_longitudinal: f32,
    rolling: f32,
    sliding: f32,
) -> TireForces {
    let mut forces = tire.forces(slip_angle, load, requested_longitudinal);
    let capacity = tire.lateral_capacity(load, forces.longitudinal);
    forces.lateral = (forces.lateral + tire.low_speed_damping_force(rolling, sliding))
        .clamp(-capacity, capacity);
    // The pneumatic trail sits behind the contact patch center relative to
    // the rolling direction, so it swaps sides when rolling backwards.
    if rolling < 0.0 {
        forces.aligning_torque = -forces.aligning_torque;
    }
    forces
}

/// Integrates an axle's transient lateral slip by one step.
fn advance_slip(
    tire: &TireParams,
    lateral_slip: f32,
    load: f32,
    rolling: f32,
    sliding: f32,
    dt: f32,
) -> f32 {
    let next = lateral_slip + tire.lateral_slip_rate(lateral_slip, rolling, sliding) * dt;
    // A fully sliding contact patch cannot store more deflection than it is
    // currently being dragged through, so once the tire slows down the
    // leftover slip is bounded by what the carcass can hold.
    let steady_state = sliding.abs() / rolling.abs().max(MIN_ROLLING_SPEED);
    let limit = steady_state.max(tire.stored_slip_limit(load));
    next.clamp(-limit, limit)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / PHYSICS_HZ as f32;

    fn simulate(vehicle: &mut Vehicle, seconds: f32, controls: impl FnMut(&Vehicle) -> Controls) {
        simulate_with(&VehicleParams::default(), vehicle, seconds, controls);
    }

    fn simulate_with(
        params: &VehicleParams,
        vehicle: &mut Vehicle,
        seconds: f32,
        mut controls: impl FnMut(&Vehicle) -> Controls,
    ) {
        for _ in 0..(seconds / DT) as usize {
            let c = controls(vehicle);
            vehicle.integrate(params, c, Ground::default(), DT);
        }
    }

    /// Pedal that gently holds `target` m/s, like a driver keeping a steady
    /// speed rather than flooring it at the first sign of slowing down.
    fn hold_speed(vehicle: &Vehicle, target: f32) -> f32 {
        ((target - vehicle.speed()) * 0.5).clamp(0.0, 0.4)
    }

    /// A car at rest with the gearbox in automatic mode.
    fn automatic() -> Vehicle {
        let mut vehicle = Vehicle::default();
        vehicle.powertrain.mode = ShiftMode::Automatic;
        vehicle
    }

    /// A car cruising straight at `speed`, in a matching gear. Driver aids
    /// are off, so tests of the chassis see its own behaviour.
    fn moving_at(speed: f32) -> Vehicle {
        Vehicle {
            velocity: Vec2::new(speed, 0.0),
            powertrain: Powertrain::cruising(speed / WHEEL_RADIUS),
            traction_control: TractionControl {
                enabled: false,
                ..default()
            },
            ..default()
        }
    }

    /// Settles into a steady turn at `speed` and returns the final state.
    fn steady_turn(speed: f32, road_wheel_angle: f32) -> Vehicle {
        steady_turn_with(&VehicleParams::default(), speed, road_wheel_angle)
    }

    fn steady_turn_with(params: &VehicleParams, speed: f32, road_wheel_angle: f32) -> Vehicle {
        let mut vehicle = moving_at(speed);
        simulate_with(params, &mut vehicle, 6.0, |v| Controls {
            throttle: hold_speed(v, speed),
            road_wheel_angle,
            ..default()
        });
        vehicle
    }

    #[test]
    fn static_wheel_loads_add_up_to_weight() {
        let params = VehicleParams::default();
        let total: f32 = params.wheel_loads(Vec2::ZERO).iter().sum();
        assert!((total - params.mass * GRAVITY).abs() < 0.1);
    }

    #[test]
    fn steering_at_standstill_does_not_move_the_car() {
        // Foot on the brake, as the automatic would otherwise creep.
        let mut vehicle = Vehicle::default();
        simulate(&mut vehicle, 2.0, |_| Controls {
            road_wheel_angle: 0.4,
            brake: 0.3,
            ..default()
        });
        assert_eq!(vehicle.position, Vec2::ZERO);
        assert_eq!(vehicle.velocity, Vec2::ZERO);
        assert_eq!(vehicle.yaw_rate, 0.0);
        assert_eq!(vehicle.front_axle_forces().lateral, 0.0);
    }

    #[test]
    fn full_throttle_reaches_100_kmh_in_plausible_time() {
        let mut vehicle = automatic();
        let mut elapsed = 0.0;
        while vehicle.speed_kmh() < 100.0 {
            simulate(&mut vehicle, DT, |_| Controls {
                throttle: 1.0,
                ..default()
            });
            elapsed += DT;
            assert!(elapsed < 20.0, "never reached 100 km/h");
        }
        assert!(
            (4.0..9.0).contains(&elapsed),
            "0-100 km/h took {elapsed:.2} s"
        );
    }

    #[test]
    fn full_braking_from_100_kmh_stops_in_plausible_distance() {
        let mut vehicle = moving_at(100.0 / 3.6);
        simulate(&mut vehicle, 10.0, |_| Controls {
            brake: 1.0,
            ..default()
        });
        let distance = vehicle.position.length();
        assert_eq!(vehicle.speed(), 0.0);
        assert!(
            (30.0..45.0).contains(&distance),
            "stopped in {distance:.1} m"
        );
    }

    #[test]
    fn braking_shifts_load_to_the_front() {
        let params = VehicleParams::default();
        let (static_front, _) = params.static_axle_loads();
        let mut vehicle = moving_at(100.0 / 3.6);
        simulate(&mut vehicle, 0.5, |_| Controls {
            brake: 1.0,
            ..default()
        });
        let front = vehicle.wheel_loads[FRONT_LEFT] + vehicle.wheel_loads[FRONT_RIGHT];
        assert!(front > static_front * 1.25, "front load {front:.0} N");
    }

    #[test]
    fn right_turn_loads_the_left_wheels() {
        let vehicle = steady_turn(20.0, 0.03);
        assert!(vehicle.wheel_loads[FRONT_LEFT] > vehicle.wheel_loads[FRONT_RIGHT]);
        assert!(vehicle.wheel_loads[REAR_LEFT] > vehicle.wheel_loads[REAR_RIGHT]);
    }

    /// Parameters with no pneumatic trail, so tires produce no aligning
    /// torque and the textbook bicycle model applies.
    fn params_without_trail() -> VehicleParams {
        let defaults = VehicleParams::default();
        VehicleParams {
            front_tire: TireParams {
                pneumatic_trail: 0.0,
                ..defaults.front_tire
            },
            rear_tire: TireParams {
                pneumatic_trail: 0.0,
                ..defaults.rear_tire
            },
            ..defaults
        }
    }

    #[test]
    fn steady_cornering_matches_linear_bicycle_theory() {
        let params = params_without_trail();
        let speed = 20.0;
        let road_wheel_angle = 0.01; // to the right
        let vehicle = steady_turn_with(&params, speed, road_wheel_angle);

        // Steady-state yaw rate with understeer gradient K, using the axle
        // cornering stiffness at static load. Small lateral load transfer
        // makes the real value differ slightly.
        let (front_load, rear_load) = params.static_axle_loads();
        let front_stiffness = 2.0 * params.front_tire.cornering_stiffness(front_load * 0.5);
        let rear_stiffness = 2.0 * params.rear_tire.cornering_stiffness(rear_load * 0.5);
        let wheelbase = params.wheelbase();
        let understeer_gradient = params.mass / wheelbase
            * (params.cg_to_rear_axle / front_stiffness - params.cg_to_front_axle / rear_stiffness);
        assert!(understeer_gradient > 0.0, "car should understeer");
        let vx = vehicle.speed();
        let expected = -road_wheel_angle * vx / (wheelbase + understeer_gradient * vx * vx);

        let error = (vehicle.yaw_rate - expected).abs() / expected.abs();
        assert!(
            error < 0.02,
            "yaw rate {:.4} rad/s, expected {expected:.4} rad/s",
            vehicle.yaw_rate
        );
        // A right turn is a negative yaw rate in the ISO frame.
        assert!(vehicle.yaw_rate < 0.0);
    }

    #[test]
    fn aligning_torque_adds_understeer() {
        // The tires' aligning torques act on the body against the turn.
        let with_trail = steady_turn(20.0, 0.01).yaw_rate.abs();
        let without_trail = steady_turn_with(&params_without_trail(), 20.0, 0.01)
            .yaw_rate
            .abs();
        assert!(with_trail < without_trail);
    }

    #[test]
    fn lateral_acceleration_is_bounded_by_friction() {
        let params = VehicleParams::default();
        let mut vehicle = moving_at(25.0);
        let mut peak: f32 = 0.0;
        simulate(&mut vehicle, 3.0, |v| {
            peak = peak.max(v.acceleration.y.abs());
            Controls {
                throttle: hold_speed(v, 25.0),
                road_wheel_angle: 0.2,
                ..default()
            }
        });
        let limit = params.front_tire.peak_friction * GRAVITY;
        assert!(
            peak < limit * 1.05,
            "peak lateral accel {:.2} g",
            peak / GRAVITY
        );
    }

    #[test]
    fn front_aligning_torque_fades_as_front_grip_runs_out() {
        // More steering at constant speed: the aligning torque first grows,
        // then drops as the front tires approach their limit.
        let torques: Vec<f32> = [0.01, 0.03, 0.05, 0.08, 0.12, 0.16]
            .into_iter()
            .map(|angle| {
                steady_turn(15.0, angle)
                    .front_axle_forces()
                    .aligning_torque
                    .abs()
            })
            .collect();
        let peak = torques.iter().copied().fold(0.0, f32::max);
        let peak_index = torques.iter().position(|&t| t == peak).unwrap();
        assert!(
            peak_index > 0 && peak_index < torques.len() - 1,
            "torques {torques:?}"
        );
        assert!(*torques.last().unwrap() < peak * 0.7, "torques {torques:?}");
    }

    #[test]
    fn coasting_past_the_limit_understeers_instead_of_spinning() {
        // Far more steering than the grip allows, off the throttle: the front
        // should slide more than the rear, and grip returns as speed drops.
        let mut vehicle = moving_at(20.0);
        for _ in 0..8 {
            simulate(&mut vehicle, 0.5, |_| Controls {
                road_wheel_angle: 0.14,
                ..default()
            });
            assert!(
                vehicle.front_slip_angle.abs() > vehicle.rear_slip_angle.abs(),
                "front {:.3} rad, rear {:.3} rad",
                vehicle.front_slip_angle,
                vehicle.rear_slip_angle
            );
        }
        assert!(vehicle.rear_slip_angle.abs() < 6.0_f32.to_radians());
    }

    /// Upper bound on yaw acceleration the tires can produce: every axle at
    /// its full grip (with generous allowance for load transfer), pushing the
    /// same way about the CG.
    fn max_yaw_acceleration(params: &VehicleParams) -> f32 {
        let (front, rear) = params.static_axle_loads();
        let grip = params
            .front_tire
            .peak_friction
            .max(params.rear_tire.peak_friction)
            * 1.3;
        (params.cg_to_front_axle * front + params.cg_to_rear_axle * rear) * grip
            / params.yaw_inertia
    }

    #[test]
    fn spinning_out_never_changes_yaw_faster_than_tires_allow() {
        // Power oversteer into a spin, then lift and brake until stopped.
        // Nothing but tire forces may change the rotation, however slow the
        // car gets.
        let params = VehicleParams::default();
        let limit = max_yaw_acceleration(&params);
        let mut vehicle = moving_at(12.0);
        let mut elapsed = 0.0;
        let mut worst: f32 = 0.0;
        for _ in 0..(8.0 / DT) as usize {
            let before = vehicle.yaw_rate;
            let controls = Controls {
                throttle: if elapsed < 2.0 { 1.0 } else { 0.0 },
                brake: if elapsed > 4.0 { 1.0 } else { 0.0 },
                road_wheel_angle: 0.12,
            };
            vehicle.integrate(&params, controls, Ground::default(), DT);
            elapsed += DT;
            worst = worst.max((vehicle.yaw_rate - before).abs() / DT);
        }
        assert!(
            worst < limit,
            "yaw accel {worst:.1} rad/s^2, limit {limit:.1}"
        );
        assert!(
            vehicle.velocity.length() < 0.05,
            "still moving: {:?}",
            vehicle.velocity
        );
        assert!(
            vehicle.yaw_rate.abs() < 0.01,
            "still turning: {}",
            vehicle.yaw_rate
        );
    }

    #[test]
    fn tire_forces_build_up_over_the_relaxation_length() {
        // A sudden steering input must not change the lateral force instantly.
        let params = VehicleParams::default();
        let mut vehicle = moving_at(20.0);
        let mut worst: f32 = 0.0;
        for _ in 0..1000 {
            let before = vehicle.acceleration.y;
            let controls = Controls {
                road_wheel_angle: 0.05,
                throttle: hold_speed(&vehicle, 20.0),
                ..default()
            };
            vehicle.integrate(&params, controls, Ground::default(), DT);
            worst = worst.max((vehicle.acceleration.y - before).abs() / DT);
        }
        assert!(worst < 1000.0, "lateral jerk {worst:.0} m/s^3");
    }

    #[test]
    fn slow_driving_follows_the_steering_geometry() {
        // At walking pace the tires barely slip, so the car turns on the
        // radius set by wheelbase and steering angle.
        let params = VehicleParams::default();
        let road_wheel_angle = 0.3;
        let vehicle = steady_turn(2.0, road_wheel_angle);
        let expected = -vehicle.speed() * road_wheel_angle.tan() / params.wheelbase();
        let error = (vehicle.yaw_rate - expected).abs() / expected.abs();
        assert!(
            error < 0.05,
            "yaw rate {:.3} rad/s, expected {expected:.3}",
            vehicle.yaw_rate
        );
    }

    #[test]
    fn spinning_car_can_slide_backwards() {
        // Momentum, not a speed clamp, decides where a spinning car goes.
        let mut vehicle = Vehicle {
            velocity: Vec2::new(20.0, 0.0),
            yaw_rate: 6.0,
            ..default()
        };
        let mut went_backwards = false;
        simulate(&mut vehicle, 1.5, |v| {
            went_backwards |= v.velocity.x < -0.5;
            Controls::default()
        });
        assert!(went_backwards);
    }

    #[test]
    fn steering_self_centers_and_goes_light_past_the_limit() {
        let params = VehicleParams::default();
        // Steered right, so the steering should be pushed back to the left.
        let torques: Vec<f32> = [0.01, 0.03, 0.05, 0.07, 0.10, 0.14]
            .into_iter()
            .map(|angle| steady_turn(20.0, angle).steering_column_torque(&params))
            .collect();
        assert!(torques.iter().all(|&t| t < 0.0), "torques {torques:?}");

        let magnitudes: Vec<f32> = torques.iter().map(|t| t.abs()).collect();
        let peak = magnitudes.iter().copied().fold(0.0, f32::max);
        let peak_index = magnitudes.iter().position(|&t| t == peak).unwrap();
        assert!(
            peak_index > 0 && peak_index < magnitudes.len() - 1,
            "torques {torques:?}"
        );
        assert!(
            *magnitudes.last().unwrap() < peak * 0.8,
            "torques {torques:?}"
        );
    }

    #[test]
    fn grass_offers_much_less_grip_than_asphalt() {
        let params = VehicleParams::default();
        let peak_lateral = |surface: Surface| {
            let mut vehicle = moving_at(20.0);
            let mut peak: f32 = 0.0;
            for _ in 0..3000 {
                let controls = Controls {
                    throttle: hold_speed(&vehicle, 20.0),
                    road_wheel_angle: 0.1,
                    ..default()
                };
                vehicle.integrate(&params, controls, Ground::on(surface), DT);
                peak = peak.max(vehicle.acceleration.y.abs());
            }
            peak / GRAVITY
        };
        let asphalt = peak_lateral(Surface::ASPHALT);
        let grass = peak_lateral(Surface::GRASS);
        assert!(
            grass < 0.65 && asphalt > 0.9,
            "asphalt {asphalt:.2} g, grass {grass:.2} g"
        );
    }

    #[test]
    fn automatic_creeps_off_the_brake() {
        let mut vehicle = Vehicle::default();
        simulate(&mut vehicle, 5.0, |_| Controls::default());
        let speed = vehicle.speed_kmh();
        assert!((2.0..15.0).contains(&speed), "creeping at {speed:.1} km/h");
    }

    #[test]
    fn full_throttle_works_up_through_the_gears() {
        let mut vehicle = automatic();
        let mut gears = vec![vehicle.powertrain.gear];
        simulate(&mut vehicle, 15.0, |v| {
            if gears.last() != Some(&v.powertrain.gear) {
                gears.push(v.powertrain.gear);
            }
            Controls {
                throttle: 1.0,
                ..default()
            }
        });
        assert_eq!(gears, [0, 1, 2, 3], "gear sequence {gears:?}");
        assert!(vehicle.powertrain.engine.rpm < 7500.0);
    }

    #[test]
    fn lifting_off_slows_the_car_with_engine_braking() {
        let params = VehicleParams::default();
        let start = 25.0;
        let seconds = 3.0;
        let mut vehicle = moving_at(start);
        simulate(&mut vehicle, seconds, |_| Controls::default());

        // The same coast with only air drag and rolling resistance.
        let mut drag_only = start;
        for _ in 0..(seconds / DT) as usize {
            let resistance = 0.5 * AIR_DENSITY * params.drag_area * drag_only * drag_only
                + Surface::ASPHALT.rolling_resistance * params.mass * GRAVITY;
            drag_only -= resistance / params.mass * DT;
        }
        assert!(
            vehicle.speed() < drag_only - 0.3,
            "in gear {:.2} m/s, drag only {drag_only:.2} m/s",
            vehicle.speed()
        );
    }

    /// The LFA at rest, in automatic, with traction control on or off.
    fn lfa(traction_control: bool) -> (VehicleParams, Vehicle) {
        use crate::engine::ONE_LR_GUE;
        use crate::powertrain::LFA_6_SPEED;
        let params = VehicleParams::lfa();
        let mut vehicle = Vehicle {
            powertrain: Powertrain::new(&ONE_LR_GUE, &LFA_6_SPEED),
            wheel_loads: params.wheel_loads(Vec2::ZERO),
            ..default()
        };
        vehicle.powertrain.mode = ShiftMode::Automatic;
        vehicle.traction_control.enabled = traction_control;
        (params, vehicle)
    }

    /// Floors it from rest with a touch of steering; returns the peak yaw
    /// rate (rad/s) and the time to 100 km/h, if reached.
    fn floor_it(params: &VehicleParams, vehicle: &mut Vehicle) -> (f32, Option<f32>) {
        let mut peak_yaw: f32 = 0.0;
        let mut to_100 = None;
        for step in 0..(8.0 / DT) as usize {
            let controls = Controls {
                throttle: 1.0,
                road_wheel_angle: 0.005,
                ..default()
            };
            vehicle.integrate(params, controls, Ground::default(), DT);
            peak_yaw = peak_yaw.max(vehicle.yaw_rate.abs());
            if to_100.is_none() && vehicle.speed_kmh() >= 100.0 {
                to_100 = Some(step as f32 * DT);
            }
        }
        (peak_yaw, to_100)
    }

    #[test]
    fn traction_control_keeps_a_full_throttle_launch_in_line() {
        let (params, mut vehicle) = lfa(true);
        let (peak_yaw, to_100) = floor_it(&params, &mut vehicle);
        // The touch of steering alone turns it at under 0.1 rad/s; a spin
        // would be well over 1.
        assert!(peak_yaw < 0.3, "yawed at {peak_yaw:.2} rad/s");
        let to_100 = to_100.expect("never reached 100 km/h");
        // The real LFA does 0-100 km/h in about 3.7 s.
        assert!((3.0..5.5).contains(&to_100), "0-100 km/h in {to_100:.2} s");
    }

    #[test]
    fn without_traction_control_the_lfa_spins_its_rear_tires_loose() {
        let (params, mut vehicle) = lfa(false);
        let (peak_yaw, _) = floor_it(&params, &mut vehicle);
        assert!(peak_yaw > 0.5, "only yawed at {peak_yaw:.2} rad/s");
    }

    /// Ground rising `grade` (rise per metre) straight ahead of a car at
    /// its default heading.
    fn hill(grade: f32) -> Ground {
        Ground {
            slope: Vehicle::default().forward() * grade,
            ..default()
        }
    }

    #[test]
    fn brakes_hold_the_car_on_a_hill() {
        let params = VehicleParams::default();
        let mut vehicle = Vehicle::default();
        for _ in 0..3000 {
            let controls = Controls {
                brake: 0.6,
                ..default()
            };
            vehicle.integrate(&params, controls, hill(0.15), DT);
        }
        assert_eq!(vehicle.position, Vec2::ZERO);
    }

    #[test]
    fn off_the_brakes_a_steep_hill_rolls_the_car_back() {
        // Too steep for the automatic's creep to hold.
        let params = VehicleParams::default();
        let mut vehicle = Vehicle::default();
        for _ in 0..3000 {
            vehicle.integrate(&params, Controls::default(), hill(0.25), DT);
        }
        assert!(vehicle.speed() < -0.5, "speed {:.2} m/s", vehicle.speed());
    }

    #[test]
    fn climbing_a_hill_slows_the_acceleration() {
        let params = VehicleParams::default();
        let speed_after = |ground: Ground| {
            let mut vehicle = automatic();
            for _ in 0..5000 {
                let controls = Controls {
                    throttle: 1.0,
                    ..default()
                };
                vehicle.integrate(&params, controls, ground, DT);
            }
            vehicle.speed()
        };
        let flat = speed_after(Ground::default());
        let uphill = speed_after(hill(0.1));
        // A 10 % grade takes about 1 m/s^2 off the acceleration.
        assert!(
            (3.0..7.0).contains(&(flat - uphill)),
            "flat {flat:.1} m/s, uphill {uphill:.1} m/s"
        );
    }
}
