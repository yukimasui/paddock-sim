//! Screen-space overlay: steering wheel, pedals, FFB torque and telemetry text.
//!
//! Drawn by a dedicated camera on its own render layer so it stays fixed on
//! screen while the world camera follows the car.

use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;
use bevy::text::FontSize;

use crate::audio::EngineAudio;
use crate::engine::REDLINE_RPM;
use crate::ffb::ForceFeedback;
use crate::powertrain::ShiftMode;
use crate::surface::Surface;
use crate::track::LapTimer;
use crate::vehicle::{FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, Vehicle, WheelSurfaces};
use crate::wheel::{Pedals, Steering};

const HUD_LAYER: usize = 1;

const RIM_RADIUS: f32 = 100.0;
const HUB_RADIUS: f32 = 22.0;
/// Offset of the wheel center from the bottom-right corner of the window.
const WHEEL_CORNER_OFFSET: Vec2 = Vec2::new(-230.0, 150.0);
const PEDAL_BAR_HEIGHT: f32 = 200.0;
const BRAKE_BAR_OFFSET_X: f32 = 140.0;
const THROTTLE_BAR_OFFSET_X: f32 = 175.0;
const TORQUE_BAR_HALF_WIDTH: f32 = 130.0;
const TORQUE_BAR_OFFSET_Y: f32 = -125.0;

const ROTATION_RANGE_STEP_DEG: f32 = 90.0;
const ROTATION_RANGE_LIMITS_DEG: (f32, f32) = (90.0, 2700.0);

const TRACK_COLOR: Color = Color::srgb(0.25, 0.27, 0.3);

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<HudGizmos>()
            .add_systems(Startup, setup_hud)
            .add_systems(
                Update,
                (
                    adjust_rotation_range,
                    draw_wheel_and_pedals,
                    update_info_text,
                    update_lap_text,
                ),
            );
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
struct HudGizmos;

#[derive(Component)]
struct InfoText;

#[derive(Component)]
struct LapText;

fn setup_hud(mut commands: Commands, mut gizmo_config: ResMut<GizmoConfigStore>) {
    commands.spawn((
        Camera2d,
        Camera {
            order: 1,
            clear_color: ClearColorConfig::None,
            ..default()
        },
        RenderLayers::layer(HUD_LAYER),
        IsDefaultUiCamera,
    ));

    let (config, _) = gizmo_config.config_mut::<HudGizmos>();
    config.line.width = 6.0;
    config.render_layers = RenderLayers::layer(HUD_LAYER);

    commands.spawn((
        InfoText,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(18.0),
            ..default()
        },
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));

    commands.spawn((
        LapText,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(26.0),
            ..default()
        },
        TextLayout::justify(Justify::Right),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            right: Val::Px(16.0),
            ..default()
        },
    ));
}

/// Up/Down arrows change the assumed lock-to-lock range to match Pit House.
fn adjust_rotation_range(keys: Res<ButtonInput<KeyCode>>, mut steering: ResMut<Steering>) {
    let mut range = steering.rotation_range_deg;
    if keys.just_pressed(KeyCode::ArrowUp) {
        range += ROTATION_RANGE_STEP_DEG;
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        range -= ROTATION_RANGE_STEP_DEG;
    }
    steering.rotation_range_deg =
        range.clamp(ROTATION_RANGE_LIMITS_DEG.0, ROTATION_RANGE_LIMITS_DEG.1);
}

fn draw_wheel_and_pedals(
    window: Single<&Window>,
    steering: Res<Steering>,
    pedals: Res<Pedals>,
    ffb: Res<ForceFeedback>,
    mut gizmos: Gizmos<HudGizmos>,
) {
    let bottom_right = Vec2::new(window.width(), -window.height()) * 0.5;
    let center = bottom_right + WHEEL_CORNER_OFFSET;

    // Steering wheel. Bevy rotations are counter-clockwise, while a positive
    // steering angle is a right turn.
    let rotation = Rot2::degrees(-steering.angle_deg());
    let rim_color = if steering.connected {
        Color::srgb(0.85, 0.87, 0.9)
    } else {
        Color::srgb(0.4, 0.4, 0.45)
    };
    gizmos
        .circle_2d(center, RIM_RADIUS, rim_color)
        .resolution(64);
    gizmos.circle_2d(center, HUB_RADIUS, rim_color);
    for dir in [Vec2::X, Vec2::NEG_Y, Vec2::NEG_X] {
        let dir = rotation * dir;
        gizmos.line_2d(
            center + dir * HUB_RADIUS,
            center + dir * RIM_RADIUS,
            rim_color,
        );
    }
    let up = rotation * Vec2::Y;
    gizmos.line_2d(
        center + up * (RIM_RADIUS - 12.0),
        center + up * (RIM_RADIUS + 12.0),
        Color::srgb(0.95, 0.25, 0.2),
    );

    // Pedals, brake left of throttle as in the car.
    let bars = [
        (
            BRAKE_BAR_OFFSET_X,
            pedals.brake,
            Color::srgb(0.95, 0.3, 0.25),
        ),
        (
            THROTTLE_BAR_OFFSET_X,
            pedals.throttle,
            Color::srgb(0.3, 0.85, 0.4),
        ),
    ];
    for (offset_x, value, fill) in bars {
        let bottom = center + Vec2::new(offset_x, -RIM_RADIUS);
        gizmos.line_2d(bottom, bottom + Vec2::Y * PEDAL_BAR_HEIGHT, TRACK_COLOR);
        if value > 0.0 {
            gizmos.line_2d(bottom, bottom + Vec2::Y * PEDAL_BAR_HEIGHT * value, fill);
        }
    }

    // FFB torque; full bar width is full device torque.
    let torque_center = center + Vec2::new(0.0, TORQUE_BAR_OFFSET_Y);
    gizmos.line_2d(
        torque_center - Vec2::X * TORQUE_BAR_HALF_WIDTH,
        torque_center + Vec2::X * TORQUE_BAR_HALF_WIDTH,
        TRACK_COLOR,
    );
    gizmos.line_2d(
        torque_center,
        torque_center + Vec2::X * ffb.torque * TORQUE_BAR_HALF_WIDTH,
        Color::srgb(1.0, 0.65, 0.2),
    );
}

fn update_info_text(
    steering: Res<Steering>,
    pedals: Res<Pedals>,
    vehicle: Res<Vehicle>,
    ffb: Res<ForceFeedback>,
    mut text: Single<&mut Text, With<InfoText>>,
) {
    let device = if steering.connected {
        steering.device_name.as_deref().unwrap_or("unknown")
    } else {
        "searching for MOZA wheel..."
    };
    let ffb_state = if ffb.direction_test {
        "DIRECTION TEST"
    } else if ffb.enabled {
        "ON"
    } else {
        "OFF"
    };

    let front = vehicle.front_axle_forces();
    let rear = vehicle.rear_axle_forces();
    let loads = vehicle.wheel_loads;

    // Lateral quantities are shown with right = positive, matching the
    // steering angle, while the vehicle model uses left = positive.
    text.0 = format!(
        "Device: {device}\n\
         Steering: {:+6.1} deg (range {:.0} deg, Up/Down)\n\
         Throttle: {:5.1}%   Brake: {:5.1}%\n\
         \n\
         Speed: {:5.1} km/h\n\
         Accel long/lat: {:+5.2} / {:+5.2} g   Yaw rate: {:+6.1} deg/s\n\
         Road wheel angle: {:+5.2} deg\n\
         Slip angle F/R: {:+5.2} / {:+5.2} deg\n\
         Lateral force F/R: {:+5.2} / {:+5.2} kN\n\
         Front aligning torque: {:+6.1} N m\n\
         Wheel load kN: FL {:.2}  FR {:.2}  RL {:.2}  RR {:.2}\n\
         \n\
         FFB: {ffb_state}  strength {:.0}% (F: toggle, Left/Right, hold T: test)\n\
         Column torque: {:+5.1} N m   Output: {:+5.1}%{}   Friction: {:4.1}%\n\
         R: reset to start   C: camera   -/=: zoom\n\
         Paddles or E/Q: shift   A: automatic   M: mute",
        steering.angle_deg(),
        steering.rotation_range_deg,
        pedals.throttle * 100.0,
        pedals.brake * 100.0,
        vehicle.speed_kmh(),
        vehicle.acceleration.x / 9.81,
        -vehicle.acceleration.y / 9.81,
        -vehicle.yaw_rate.to_degrees(),
        vehicle.road_wheel_angle.to_degrees(),
        -vehicle.front_slip_angle.to_degrees(),
        -vehicle.rear_slip_angle.to_degrees(),
        -front.lateral / 1000.0,
        -rear.lateral / 1000.0,
        // Positive = pulls the steering to the right.
        -front.aligning_torque,
        loads[FRONT_LEFT] / 1000.0,
        loads[FRONT_RIGHT] / 1000.0,
        loads[REAR_LEFT] / 1000.0,
        loads[REAR_RIGHT] / 1000.0,
        ffb.strength * 100.0,
        ffb.column_torque,
        ffb.torque * 100.0,
        if ffb.clipping { " CLIP" } else { "" },
        ffb.friction * 100.0,
    );
}

fn update_lap_text(
    timer: Res<LapTimer>,
    vehicle: Res<Vehicle>,
    audio: Res<EngineAudio>,
    surfaces: Res<WheelSurfaces>,
    mut text: Single<&mut Text, With<LapText>>,
) {
    let lap = match timer.current {
        Some(time) => format!("Lap {}  {}", timer.laps + 1, format_lap_time(Some(time))),
        None => "Out lap".to_owned(),
    };
    let off_track = surfaces.0.iter().filter(|&&s| s == Surface::GRASS).count();
    let warning = if off_track > 0 {
        format!("\n{off_track} wheel(s) off track")
    } else {
        String::new()
    };
    let powertrain = &vehicle.powertrain;
    let rpm = powertrain.engine.rpm;
    let redline = if rpm >= REDLINE_RPM { " REDLINE" } else { "" };
    let lockup = if powertrain.lockup > 0.0 { " L" } else { "" };
    let mute = if audio.muted() { " (muted, M)" } else { "" };
    let mode = match powertrain.mode {
        ShiftMode::Manual => "M",
        ShiftMode::Automatic => "D",
    };
    let gear = powertrain.gear + 1;
    text.0 = format!(
        "{lap}\nLast  {}\nBest  {}\n{mode}{gear}{lockup}  {rpm:.0} rpm{redline}{mute}{warning}",
        format_lap_time(timer.last),
        format_lap_time(timer.best),
    );
}

/// Formats seconds as m:ss.sss, or dashes when there is no time.
fn format_lap_time(seconds: Option<f32>) -> String {
    match seconds {
        Some(seconds) => {
            let minutes = (seconds / 60.0).floor();
            format!("{:.0}:{:06.3}", minutes, seconds - minutes * 60.0)
        }
        None => "-:--.---".to_owned(),
    }
}
