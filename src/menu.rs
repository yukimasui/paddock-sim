//! The menus before driving: choose a car, then a course.

use bevy::picking::Pickable;
use bevy::picking::events::{PointerClick, PointerOut, PointerOver};
use bevy::prelude::*;
use bevy::text::FontSize;

use crate::car::{Car, SelectedCar};
use crate::course::{Course, SelectedCourse};

#[derive(States, Default, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GameState {
    #[default]
    ChoosingCar,
    ChoosingCourse,
    Driving,
}

pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<GameState>()
            .init_resource::<SelectedCar>()
            .init_resource::<SelectedCourse>()
            .add_systems(OnEnter(GameState::ChoosingCar), spawn_car_menu)
            .add_systems(OnEnter(GameState::ChoosingCourse), spawn_course_menu)
            .add_systems(OnExit(GameState::ChoosingCar), despawn_menu)
            .add_systems(OnExit(GameState::ChoosingCourse), despawn_menu)
            .add_systems(
                Update,
                (
                    pick_car_with_keys.run_if(in_state(GameState::ChoosingCar)),
                    pick_course_with_keys.run_if(in_state(GameState::ChoosingCourse)),
                    back_to_car_menu.run_if(not(in_state(GameState::ChoosingCar))),
                ),
            );
    }
}

#[derive(Component)]
struct Menu;

const BUTTON_COLOR: Color = Color::srgba(0.12, 0.13, 0.16, 0.92);
const BUTTON_HOVER_COLOR: Color = Color::srgba(0.2, 0.32, 0.5, 0.95);
/// Number keys that pick the first, second, ... option.
const OPTION_KEYS: [KeyCode; 4] = [
    KeyCode::Digit1,
    KeyCode::Digit2,
    KeyCode::Digit3,
    KeyCode::Digit4,
];

fn pick_car(car: Car, commands: &mut Commands) {
    commands.insert_resource(SelectedCar(car));
    commands.set_state(GameState::ChoosingCourse);
}

fn pick_course(course: Course, commands: &mut Commands) {
    commands.insert_resource(SelectedCourse(course));
    commands.set_state(GameState::Driving);
}

fn spawn_car_menu(mut commands: Commands) {
    let options = Car::ALL.map(|car| (car, car.spec().name, car.spec().summary));
    spawn_menu(&mut commands, "Choose your car", &options, pick_car);
}

fn spawn_course_menu(mut commands: Commands) {
    let options = Course::ALL.map(|course| (course, course.spec().name, course.spec().summary));
    spawn_menu(&mut commands, "Choose a course", &options, pick_course);
}

/// A full-screen menu of `options` (value, name, one-line summary); clicking
/// one calls `pick` with its value.
fn spawn_menu<T: Copy + Send + Sync + 'static>(
    commands: &mut Commands,
    title: &str,
    options: &[(T, &'static str, &'static str)],
    pick: fn(T, &mut Commands),
) {
    commands
        .spawn((
            Menu,
            Node {
                position_type: PositionType::Absolute,
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                row_gap: Val::Px(24.0),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.03, 0.05, 0.75)),
            // Above the telemetry text.
            GlobalZIndex(10),
        ))
        .with_children(|menu| {
            menu.spawn((
                Text::new(title),
                TextFont {
                    font_size: FontSize::Px(40.0),
                    ..default()
                },
            ));
            menu.spawn(Node {
                flex_direction: FlexDirection::Row,
                column_gap: Val::Px(24.0),
                ..default()
            })
            .with_children(|row| {
                for (index, &(value, name, summary)) in options.iter().enumerate() {
                    spawn_option(row, index + 1, name, summary).observe(
                        move |_: On<PointerClick>, mut commands: Commands| {
                            pick(value, &mut commands)
                        },
                    );
                }
            });
            menu.spawn((
                Text::new("Click an option or press its number. Esc returns to the car menu."),
                TextFont {
                    font_size: FontSize::Px(16.0),
                    ..default()
                },
                TextColor(Color::srgb(0.7, 0.72, 0.76)),
            ));
        });
}

fn spawn_option<'a>(
    row: &'a mut ChildSpawnerCommands,
    number: usize,
    name: &str,
    summary: &str,
) -> EntityCommands<'a> {
    // Picking reports clicks and hovers on the option node itself; its
    // text ignores the pointer so it can't steal them.
    let mut option = row.spawn((
        Node {
            width: Val::Px(380.0),
            padding: UiRect::all(Val::Px(20.0)),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(8.0),
            border: UiRect::all(Val::Px(2.0)),
            ..default()
        },
        BorderColor::all(Color::srgb(0.35, 0.4, 0.5)),
        BackgroundColor(BUTTON_COLOR),
    ));
    option
        .with_children(|option| {
            option.spawn((
                Text::new(format!("{number}. {name}")),
                TextFont {
                    font_size: FontSize::Px(28.0),
                    ..default()
                },
                Pickable::IGNORE,
            ));
            option.spawn((
                Text::new(summary),
                TextFont {
                    font_size: FontSize::Px(15.0),
                    ..default()
                },
                TextColor(Color::srgb(0.75, 0.78, 0.82)),
                Pickable::IGNORE,
            ));
        })
        .observe(
            |over: On<PointerOver>, mut colors: Query<&mut BackgroundColor>| {
                if let Ok(mut color) = colors.get_mut(over.entity) {
                    color.0 = BUTTON_HOVER_COLOR;
                }
            },
        )
        .observe(
            |out: On<PointerOut>, mut colors: Query<&mut BackgroundColor>| {
                if let Ok(mut color) = colors.get_mut(out.entity) {
                    color.0 = BUTTON_COLOR;
                }
            },
        );
    option
}

fn despawn_menu(mut commands: Commands, menus: Query<Entity, With<Menu>>) {
    for menu in &menus {
        commands.entity(menu).despawn();
    }
}

fn pick_car_with_keys(keys: Res<ButtonInput<KeyCode>>, mut commands: Commands) {
    for (key, car) in OPTION_KEYS.into_iter().zip(Car::ALL) {
        if keys.just_pressed(key) {
            pick_car(car, &mut commands);
        }
    }
}

fn pick_course_with_keys(keys: Res<ButtonInput<KeyCode>>, mut commands: Commands) {
    for (key, course) in OPTION_KEYS.into_iter().zip(Course::ALL) {
        if keys.just_pressed(key) {
            pick_course(course, &mut commands);
        }
    }
}

/// Esc goes back to choosing a car.
fn back_to_car_menu(keys: Res<ButtonInput<KeyCode>>, mut next: ResMut<NextState<GameState>>) {
    if keys.just_pressed(KeyCode::Escape) {
        next.set(GameState::ChoosingCar);
    }
}
