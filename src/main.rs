mod audio;
mod engine;
mod engine_sound;
mod ffb;
mod hud;
mod powertrain;
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
            WheelPlugin,
            VehiclePlugin,
            TrackPlugin,
            FfbPlugin,
            ViewPlugin,
            HudPlugin,
            EngineAudioPlugin,
        ))
        .insert_resource(ClearColor(Color::srgb(0.55, 0.7, 0.88)))
        .run();
}
