//! 3D world view: lighting, the chase camera, the car model and its recent path.
//!
//! Physics runs in a 2D ground plane (x east, y north). The 3D world is
//! Y-up, with the ground plane mapped to x and -z so that headings and
//! rotations keep their sense when seen from above. The track itself is drawn
//! by meshes spawned in `track`.

use std::collections::VecDeque;
use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;

use crate::vehicle::{
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, Vehicle, VehicleParams, WheelSurfaces,
};

const FIELD_OF_VIEW_DEG: f32 = 60.0;
/// Chase camera offset behind and above the car at zoom 1, m.
const CHASE_DISTANCE: f32 = 7.5;
const CHASE_HEIGHT: f32 = 2.8;
/// The chase camera aims at a point this far ahead of the car, m.
const CHASE_AIM_AHEAD: f32 = 6.0;
const TOP_DOWN_HEIGHT: f32 = 60.0; // m at zoom 1
const ZOOM_STEP: f32 = 1.2;
const ZOOM_LIMITS: (f32, f32) = (0.4, 4.0);
/// Time constant with which the camera swings round behind the car, s.
const CAMERA_HEADING_LAG: f32 = 0.25;

/// A narrow box body, so the wheels stick out at the sides and stay visible.
const BODY_SIZE: Vec3 = Vec3::new(3.9, 0.35, 1.15); // length, height, width
const BODY_RIDE_HEIGHT: f32 = 0.3; // m, underside above the ground
const CABIN_SIZE: Vec3 = Vec3::new(1.5, 0.4, 0.95);
/// Cabin center relative to the body center, m (rearward of the middle).
const CABIN_OFFSET: Vec3 = Vec3::new(-0.4, 0.37, 0.0);
const WHEEL_RADIUS: f32 = 0.31; // m
const WHEEL_WIDTH: f32 = 0.22; // m
/// Body roll and pitch per g of load-transfer acceleration, rad. Purely
/// visual, roughly what a sports car's suspension allows.
const ROLL_PER_G: f32 = 0.045;
const PITCH_PER_G: f32 = 0.025;
const GRAVITY: f32 = 9.81;

/// The velocity arrow shows where the car will be after this long, s.
const VELOCITY_ARROW_TIME: f32 = 0.5;
const TRAIL_INTERVAL: f32 = 0.05; // s
const TRAIL_LENGTH: usize = 400;
/// Height above the ground at which lines are drawn, m.
const LINE_HEIGHT: f32 = 0.08;

pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Trail>()
            .init_resource::<CameraSettings>()
            .insert_resource(GlobalAmbientLight {
                brightness: 400.0,
                ..default()
            })
            .add_systems(Startup, (setup_scene, spawn_car))
            .add_systems(
                Update,
                (
                    adjust_camera,
                    place_car,
                    turn_wheels,
                    color_tires,
                    follow_vehicle,
                    record_trail,
                    draw_trail,
                    draw_velocity,
                )
                    .chain(),
            );
    }
}

/// Converts a point on the physics ground plane (m) at `height` (m) to 3D world space.
pub fn to_world(ground: Vec2, height: f32) -> Vec3 {
    Vec3::new(ground.x, height, -ground.y)
}

/// Rotation in 3D world space for a ground-plane heading (rad, CCW from +X).
/// The model's forward axis is +X.
fn heading_rotation(heading: f32) -> Quat {
    Quat::from_rotation_y(heading)
}

#[derive(Component)]
struct WorldCamera;

#[derive(Resource)]
struct CameraSettings {
    /// Chase camera behind the car; otherwise looking straight down.
    chase: bool,
    zoom: f32,
    /// Smoothed heading the camera looks along, rad.
    heading: Option<f32>,
}

impl Default for CameraSettings {
    fn default() -> Self {
        Self {
            chase: true,
            zoom: 1.0,
            heading: None,
        }
    }
}

/// Recent CG positions in meters, oldest first.
#[derive(Resource, Default)]
struct Trail {
    points: VecDeque<Vec2>,
    since_last: f32,
}

/// Root of the car model, placed at the CG on the ground.
#[derive(Component)]
struct CarModel;

/// The sprung body, which rolls and pitches.
#[derive(Component)]
struct CarBody;

/// Steering pivot of one wheel.
#[derive(Component)]
struct WheelPivot(usize);

/// The spinning part of a wheel, with its accumulated rotation, rad.
#[derive(Component)]
struct WheelSpin(f32);

/// Per-wheel tire materials, recolored by grip usage.
#[derive(Resource)]
struct TireMaterials([Handle<StandardMaterial>; 4]);

fn setup_scene(mut commands: Commands, mut gizmo_config: ResMut<GizmoConfigStore>) {
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: FIELD_OF_VIEW_DEG.to_radians(),
            ..default()
        }),
        WorldCamera,
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 6000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 60.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let (config, _) = gizmo_config.config_mut::<DefaultGizmoConfigGroup>();
    config.line.width = 2.0;
}

fn spawn_car(
    mut commands: Commands,
    params: Res<VehicleParams>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let body_material = materials.add(Color::srgb(0.85, 0.2, 0.15));
    let cabin_material = materials.add(Color::srgb(0.15, 0.17, 0.2));
    let hub_material = materials.add(Color::srgb(0.85, 0.85, 0.85));
    let tire_materials = [(); 4].map(|_| materials.add(Color::srgb(0.1, 0.1, 0.1)));
    let wheel_mesh = meshes.add(Cylinder::new(WHEEL_RADIUS, WHEEL_WIDTH));
    // A bar across the wheel face makes its rotation visible.
    let spoke_mesh = meshes.add(Cuboid::new(WHEEL_RADIUS * 1.6, WHEEL_WIDTH * 1.05, 0.08));

    // The model's local frame: +X forward, +Y up, -Z to the car's left.
    let wheel_offsets = wheel_offsets(&params);
    commands
        .spawn((CarModel, Transform::default(), Visibility::default()))
        .with_children(|car| {
            car.spawn((CarBody, Transform::default(), Visibility::default()))
                .with_children(|body| {
                    let body_center = Vec3::new(
                        axle_midpoint(&params),
                        BODY_RIDE_HEIGHT + BODY_SIZE.y * 0.5,
                        0.0,
                    );
                    body.spawn((
                        Mesh3d(meshes.add(Cuboid::from_size(BODY_SIZE))),
                        MeshMaterial3d(body_material),
                        Transform::from_translation(body_center),
                    ));
                    body.spawn((
                        Mesh3d(meshes.add(Cuboid::from_size(CABIN_SIZE))),
                        MeshMaterial3d(cabin_material),
                        Transform::from_translation(body_center + CABIN_OFFSET),
                    ));
                });

            for wheel in [FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT] {
                car.spawn((
                    WheelPivot(wheel),
                    Transform::from_translation(wheel_offsets[wheel]),
                    Visibility::default(),
                ))
                .with_children(|pivot| {
                    pivot
                        .spawn((WheelSpin(0.0), Transform::default(), Visibility::default()))
                        .with_children(|spin| {
                            // Cylinders are built along Y; turn the axle to point sideways (Z).
                            spin.spawn((
                                Mesh3d(wheel_mesh.clone()),
                                MeshMaterial3d(tire_materials[wheel].clone()),
                                Transform::from_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                            ));
                            spin.spawn((
                                Mesh3d(spoke_mesh.clone()),
                                MeshMaterial3d(hub_material.clone()),
                                Transform::from_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                            ));
                        });
                });
            }
        });
    commands.insert_resource(TireMaterials(tire_materials));
}

/// Distance from the CG forward to the middle of the wheelbase, m.
fn axle_midpoint(params: &VehicleParams) -> f32 {
    (params.cg_to_front_axle - params.cg_to_rear_axle) * 0.5
}

/// Wheel centers in the model's local frame (+X forward, -Z left).
fn wheel_offsets(params: &VehicleParams) -> [Vec3; 4] {
    let mut offsets = [Vec3::ZERO; 4];
    let front = params.cg_to_front_axle;
    let rear = -params.cg_to_rear_axle;
    offsets[FRONT_LEFT] = Vec3::new(front, WHEEL_RADIUS, -params.front_track * 0.5);
    offsets[FRONT_RIGHT] = Vec3::new(front, WHEEL_RADIUS, params.front_track * 0.5);
    offsets[REAR_LEFT] = Vec3::new(rear, WHEEL_RADIUS, -params.rear_track * 0.5);
    offsets[REAR_RIGHT] = Vec3::new(rear, WHEEL_RADIUS, params.rear_track * 0.5);
    offsets
}

fn place_car(
    vehicle: Res<Vehicle>,
    mut car: Single<&mut Transform, With<CarModel>>,
    mut body: Single<&mut Transform, (With<CarBody>, Without<CarModel>)>,
) {
    car.translation = to_world(vehicle.position, 0.0);
    car.rotation = heading_rotation(vehicle.heading);

    // Lean with the (lagged) accelerations that drive load transfer: roll
    // towards the outside of a turn, nose down under braking.
    let acceleration = vehicle.load_transfer_acceleration / GRAVITY;
    body.rotation = Quat::from_rotation_x(acceleration.y * ROLL_PER_G)
        * Quat::from_rotation_z(acceleration.x * PITCH_PER_G);
}

fn turn_wheels(
    time: Res<Time>,
    vehicle: Res<Vehicle>,
    mut pivots: Query<(&WheelPivot, &mut Transform)>,
    mut spins: Query<(&mut WheelSpin, &mut Transform), Without<WheelPivot>>,
) {
    // Front wheels steer; positive road wheel angle is to the right, which
    // is a negative (clockwise) rotation about Y.
    for (pivot, mut transform) in &mut pivots {
        let steer = if pivot.0 == FRONT_LEFT || pivot.0 == FRONT_RIGHT {
            -vehicle.road_wheel_angle
        } else {
            0.0
        };
        transform.rotation = Quat::from_rotation_y(steer);
    }

    // Rolling forward turns the top of the wheel towards +X, a negative
    // rotation about the axle (Z).
    let rolled = vehicle.velocity.x * time.delta_secs() / WHEEL_RADIUS;
    for (mut spin, mut transform) in &mut spins {
        spin.0 = (spin.0 - rolled).rem_euclid(TAU);
        transform.rotation = Quat::from_rotation_z(spin.0);
    }
}

fn color_tires(
    vehicle: Res<Vehicle>,
    params: Res<VehicleParams>,
    surfaces: Res<WheelSurfaces>,
    tire_materials: Res<TireMaterials>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for wheel in [FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT] {
        let tire = if wheel == FRONT_LEFT || wheel == FRONT_RIGHT {
            &params.front_tire
        } else {
            &params.rear_tire
        };
        let tire = tire.with_grip(surfaces.0[wheel].grip);
        let load = vehicle.wheel_loads[wheel];
        let force = vehicle.tire_forces[wheel];
        let usage = if load > 0.0 {
            Vec2::new(force.longitudinal, force.lateral).length() / (tire.friction(load) * load)
        } else {
            1.0
        };
        if let Some(mut material) = materials.get_mut(&tire_materials.0[wheel]) {
            material.base_color = grip_color(usage);
        }
    }
}

/// C toggles chase and top-down views; minus and equals zoom out and in.
fn adjust_camera(keys: Res<ButtonInput<KeyCode>>, mut settings: ResMut<CameraSettings>) {
    if keys.just_pressed(KeyCode::KeyC) {
        settings.chase = !settings.chase;
    }
    if keys.just_pressed(KeyCode::Minus) {
        settings.zoom *= ZOOM_STEP;
    }
    if keys.just_pressed(KeyCode::Equal) {
        settings.zoom /= ZOOM_STEP;
    }
    settings.zoom = settings.zoom.clamp(ZOOM_LIMITS.0, ZOOM_LIMITS.1);
}

fn follow_vehicle(
    time: Res<Time>,
    vehicle: Res<Vehicle>,
    mut settings: ResMut<CameraSettings>,
    mut camera: Single<&mut Transform, With<WorldCamera>>,
) {
    // Swing round behind the car with a lag so a spin doesn't whip the
    // camera around.
    let blend = 1.0 - (-time.delta_secs() / CAMERA_HEADING_LAG).exp();
    let heading = match settings.heading {
        Some(heading) => heading + wrap_angle(vehicle.heading - heading) * blend,
        None => vehicle.heading,
    };
    settings.heading = Some(heading);

    let forward = to_world(Vec2::from_angle(heading), 0.0);
    let car = to_world(vehicle.position, 0.0);
    **camera = if settings.chase {
        let eye =
            car - forward * CHASE_DISTANCE * settings.zoom + Vec3::Y * CHASE_HEIGHT * settings.zoom;
        let target = car + forward * CHASE_AIM_AHEAD + Vec3::Y;
        Transform::from_translation(eye).looking_at(target, Vec3::Y)
    } else {
        // Straight down with the car pointing up the screen.
        Transform::from_translation(car + Vec3::Y * TOP_DOWN_HEIGHT * settings.zoom)
            .looking_at(car, forward)
    };
}

/// Wraps an angle to [-PI, PI).
fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

fn record_trail(time: Res<Time>, vehicle: Res<Vehicle>, mut trail: ResMut<Trail>) {
    // A reset teleports the car; don't connect the old path to the new spot.
    if trail
        .points
        .back()
        .is_some_and(|last| last.distance(vehicle.position) > 20.0)
    {
        trail.points.clear();
    }

    trail.since_last += time.delta_secs();
    if trail.since_last < TRAIL_INTERVAL {
        return;
    }
    trail.since_last = 0.0;
    trail.points.push_back(vehicle.position);
    if trail.points.len() > TRAIL_LENGTH {
        trail.points.pop_front();
    }
}

fn draw_trail(trail: Res<Trail>, mut gizmos: Gizmos) {
    gizmos.linestrip(
        trail.points.iter().map(|&p| to_world(p, LINE_HEIGHT)),
        Color::srgb(0.3, 0.5, 0.8),
    );
}

/// Velocity of the CG on the ground; its angle to the body is the vehicle slip angle.
fn draw_velocity(vehicle: Res<Vehicle>, mut gizmos: Gizmos) {
    let forward = vehicle.forward();
    let world_velocity = forward * vehicle.velocity.x + forward.perp() * vehicle.velocity.y;
    gizmos.arrow(
        to_world(vehicle.position, LINE_HEIGHT),
        to_world(
            vehicle.position + world_velocity * VELOCITY_ARROW_TIME,
            LINE_HEIGHT,
        ),
        Color::srgb(0.3, 0.85, 0.4),
    );
}

/// Green while the tire has grip to spare, through yellow, to red at the limit.
fn grip_color(usage: f32) -> Color {
    let usage = usage.clamp(0.0, 1.0);
    let green = Color::srgb(0.3, 0.85, 0.4);
    let yellow = Color::srgb(0.95, 0.85, 0.3);
    let red = Color::srgb(0.95, 0.25, 0.2);
    if usage < 0.7 {
        green.mix(&yellow, usage / 0.7)
    } else {
        yellow.mix(&red, (usage - 0.7) / 0.3)
    }
}
