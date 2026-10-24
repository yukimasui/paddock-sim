mod audio;
mod car;
mod course;
mod courses;
mod engine;
mod engine_sound;
mod ffb;
mod hud;
mod menu;
mod powertrain;
mod shift_lights;
mod surface;
mod tire;
mod track;
mod vehicle;
mod view;
mod wheel;

use bevy::prelude::*;

use audio::EngineAudioPlugin;
use ffb::FfbPlugin;
use hud::HudPlugin;
use menu::MenuPlugin;
use shift_lights::ShiftLightsPlugin;
use track::TrackPlugin;
use vehicle::VehiclePlugin;
use view::ViewPlugin;
use wheel::WheelPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Paddock".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            MenuPlugin,
            WheelPlugin,
            VehiclePlugin,
            TrackPlugin,
            FfbPlugin,
            ViewPlugin,
            HudPlugin,
            EngineAudioPlugin,
            ShiftLightsPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.55, 0.7, 0.88)))
        .run();
}
