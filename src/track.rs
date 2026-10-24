//! The course being driven: centerline geometry, elevation, surface lookup,
//! rendering and timing.
//!
//! A course is a polyline of evenly spaced centerline samples, closed for a
//! circuit or open for a point-to-point road. Distance along the centerline
//! (progress) and signed distance from it (offset, positive to the left)
//! decide which surface a wheel is on and when a lap or run is completed.
//! Each sample also carries the road's elevation, which tilts the car and
//! pulls it along slopes.

use std::sync::atomic::{AtomicUsize, Ordering};

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::car::SelectedCar;
use crate::course::{CourseSpec, Layout, SelectedCourse};
use crate::menu::GameState;
use crate::powertrain::Powertrain;
use crate::surface::Surface;
use crate::vehicle::{Ground, Vehicle, VehicleParams, step_vehicle};
use crate::view::to_world;

const SAMPLE_SPACING: f32 = 2.0; // m
const SPLINE_STEPS_PER_SEGMENT: usize = 200;
const CURB_WIDTH: f32 = 1.2; // m, outside the track edge
/// Corners tighter than this radius (m) get curbs on both sides.
const CURB_MAX_RADIUS: f32 = 90.0;
/// Curbs extend this many samples before and after a tight section.
const CURB_EXTENSION: usize = 4;
const EDGE_LINE_WIDTH: f32 = 0.2; // m
const EDGE_LINE_INSET: f32 = 0.3; // m, from the track edge
const CENTER_LINE_WIDTH: f32 = 0.15; // m
/// The car starts this far before the start line, m.
const START_DISTANCE: f32 = 15.0;
/// On a point-to-point road, the start and finish lines sit this far in
/// from each end, m.
const ROAD_END_MARGIN: f32 = 30.0;
/// Heights of the ground layers above the road surface data, m, stacked to
/// avoid z-fighting.
const ASPHALT_HEIGHT: f32 = 0.02;
const MARKINGS_HEIGHT: f32 = 0.04;
/// Flat courses: mown-grass checkerboard around the track.
const GRASS_TILE: f32 = 20.0; // m
const GRASS_MARGIN: f32 = 300.0; // m around the track
/// Hilly courses: terrain grid following the road's elevation.
const TERRAIN_CELL: f32 = 8.0; // m
const TERRAIN_REACH: f32 = 90.0; // m from the road
/// The terrain sits this far below the road, so the road always shows.
const TERRAIN_DROP: f32 = 0.4; // m
/// Terrain within this distance of the road edge keeps the road's height;
/// further out it is smoothed, so where the road doubles back above itself
/// the ground slopes between the levels instead of forming spikes.
const TERRAIN_ANCHOR: f32 = 6.0; // m
const TERRAIN_SMOOTHING_PASSES: usize = 8;
/// Marker posts along both sides of the road give a sense of speed.
const POST_SPACING: f32 = 25.0; // m
const POST_OFFSET: f32 = 2.5; // m beyond the edge (and curbs)
const POST_SIZE: Vec3 = Vec3::new(0.15, 1.0, 0.15);
/// Nearest-segment searches look this many segments either side of the
/// last answer before falling back to the whole course.
const SEARCH_WINDOW: usize = 64;
/// A local answer further than this from the centerline (m) triggers a full
/// search, in case the point jumped (a reset) or the road doubles back.
const SEARCH_TRUST: f32 = 15.0;

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Track::new(SelectedCourse::default().0.spec()))
            .init_resource::<LapTimer>()
            .add_systems(Startup, (spawn_track_meshes, reset_to_start).chain())
            .add_systems(
                OnEnter(GameState::Driving),
                (load_course, reset_to_start).chain(),
            )
            .add_systems(
                FixedUpdate,
                (
                    update_ground.before(step_vehicle),
                    time_laps.after(step_vehicle),
                ),
            )
            .add_systems(Update, reset_on_key);
    }
}

#[derive(Resource, Debug)]
pub struct Track {
    /// Centerline samples, evenly spaced.
    points: Vec<Vec2>,
    /// Road elevation at each sample, m.
    elevations: Vec<f32>,
    /// Distance along the centerline to each sample, m.
    distances: Vec<f32>,
    /// Whether each sample has curbs.
    curbs: Vec<bool>,
    closed: bool,
    length: f32,
    half_width: f32,
    center_line: bool,
    /// Segment found by the last search, where the next one starts.
    search_hint: AtomicUsize,
}

/// Where a point is relative to the centerline.
#[derive(Clone, Copy, Debug)]
pub struct TrackLocation {
    /// Distance along the centerline from its start, m.
    pub progress: f32,
    /// Signed distance from the centerline, m; positive to the left.
    pub offset: f32,
    /// Index of the centerline segment the point is beside, and how far
    /// along it (0..=1).
    segment: usize,
    along: f32,
}

impl Track {
    pub fn new(spec: &CourseSpec) -> Self {
        let (points, elevations, closed) = match spec.layout {
            Layout::Circuit(controls) => {
                let points = resample_closed(&closed_catmull_rom(controls), SAMPLE_SPACING);
                let elevations = vec![0.0; points.len()];
                (points, elevations, true)
            }
            Layout::Road(data) => {
                let (points, elevations) = resample_open(data, SAMPLE_SPACING);
                (points, elevations, false)
            }
        };
        let n = points.len();
        let segments = if closed { n } else { n - 1 };
        let mut distances = Vec::with_capacity(n);
        let mut length = 0.0;
        for i in 0..n {
            distances.push(length);
            if i < segments {
                length += points[i].distance(points[(i + 1) % n]);
            }
        }

        let mut track = Self {
            points,
            elevations,
            distances,
            curbs: vec![false; n],
            closed,
            length,
            half_width: spec.half_width,
            center_line: spec.center_line,
            search_hint: AtomicUsize::new(0),
        };
        if spec.curbs {
            let tight: Vec<bool> = (0..n)
                .map(|i| track.radius_at(i) < CURB_MAX_RADIUS)
                .collect();
            // Extend curbs a few samples either side of each tight section,
            // wrapping round a circuit but not past the ends of a road.
            track.curbs = (0..n)
                .map(|i| {
                    (0..=2 * CURB_EXTENSION).any(|k| {
                        let within =
                            track.closed || (CURB_EXTENSION..n + CURB_EXTENSION).contains(&(i + k));
                        within && tight[(i + n + k - CURB_EXTENSION) % n]
                    })
                })
                .collect();
        }
        track
    }

    pub fn length(&self) -> f32 {
        self.length
    }

    pub fn closed(&self) -> bool {
        self.closed
    }

    fn segment_count(&self) -> usize {
        if self.closed {
            self.points.len()
        } else {
            self.points.len() - 1
        }
    }

    /// Sample after `i`, wrapping on a closed course.
    fn next(&self, i: usize) -> usize {
        (i + 1) % self.points.len()
    }

    /// Unit direction of travel at sample `i`.
    fn tangent(&self, i: usize) -> Vec2 {
        let n = self.points.len();
        let (before, after) = if self.closed {
            ((i + n - 1) % n, (i + 1) % n)
        } else {
            (i.saturating_sub(1), (i + 1).min(n - 1))
        };
        (self.points[after] - self.points[before]).normalize()
    }

    /// Radius of the centerline's curve at sample `i`, m.
    fn radius_at(&self, i: usize) -> f32 {
        let n = self.points.len();
        if !self.closed && (i == 0 || i == n - 1) {
            return f32::INFINITY;
        }
        circumradius(
            self.points[(i + n - 1) % n],
            self.points[i],
            self.points[(i + 1) % n],
        )
    }

    /// Nearest point on segment `i` to `point`.
    fn locate_on(&self, point: Vec2, i: usize) -> TrackLocation {
        let start = self.points[i];
        let segment = self.points[self.next(i)] - start;
        let along = ((point - start).dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
        let to_point = point - (start + segment * along);
        TrackLocation {
            progress: self.distances[i] + segment.length() * along,
            offset: to_point.length() * segment.perp_dot(to_point).signum(),
            segment: i,
            along,
        }
    }

    fn nearest(&self, point: Vec2, segments: impl Iterator<Item = usize>) -> TrackLocation {
        segments
            .map(|i| self.locate_on(point, i))
            .min_by(|a, b| a.offset.abs().total_cmp(&b.offset.abs()))
            .expect("a course has segments")
    }

    pub fn locate(&self, point: Vec2) -> TrackLocation {
        let count = self.segment_count();
        let hint = self.search_hint.load(Ordering::Relaxed).min(count - 1);
        let window = if self.closed {
            let start = hint + count - SEARCH_WINDOW.min(count / 2);
            (0..2 * SEARCH_WINDOW.min(count / 2))
                .map(|k| (start + k) % count)
                .collect::<Vec<_>>()
        } else {
            (hint.saturating_sub(SEARCH_WINDOW)..(hint + SEARCH_WINDOW).min(count)).collect()
        };
        let mut best = self.nearest(point, window.into_iter());
        if best.offset.abs() > SEARCH_TRUST {
            best = self.nearest(point, 0..count);
        }
        self.search_hint.store(best.segment, Ordering::Relaxed);
        best
    }

    pub fn surface_at(&self, location: TrackLocation) -> Surface {
        let distance = location.offset.abs();
        if distance <= self.half_width {
            Surface::ASPHALT
        } else if distance <= self.half_width + CURB_WIDTH && self.curbs[location.segment] {
            Surface::CURB
        } else {
            Surface::GRASS
        }
    }

    /// Road elevation at `location`, m.
    pub fn elevation_at(&self, location: TrackLocation) -> f32 {
        let i = location.segment;
        let (from, to) = (self.elevations[i], self.elevations[self.next(i)]);
        from + (to - from) * location.along
    }

    /// Ground slope at `location` as a gradient on the ground plane: how
    /// much the road rises per metre east and north. Only the slope along
    /// the road is modelled, not its camber.
    pub fn slope_at(&self, location: TrackLocation) -> Vec2 {
        let i = location.segment;
        let j = self.next(i);
        let run = self.points[j] - self.points[i];
        let rise = self.elevations[j] - self.elevations[i];
        run.normalize_or_zero() * rise / run.length().max(1e-3)
    }

    /// Position and heading (rad, CCW from +X) on the centerline at
    /// `progress`, wrapping on a circuit and clamped on a road.
    pub fn pose_at(&self, progress: f32) -> (Vec2, f32) {
        let progress = if self.closed {
            progress.rem_euclid(self.length)
        } else {
            progress.clamp(0.0, self.length)
        };
        let i =
            (self.distances.partition_point(|&d| d <= progress) - 1).min(self.segment_count() - 1);
        let start = self.points[i];
        let segment = self.points[self.next(i)] - start;
        let position = start + segment.normalize() * (progress - self.distances[i]);
        (position, segment.to_angle())
    }

    /// Progress (m) of the start line, where timing begins.
    fn start_line(&self) -> f32 {
        if self.closed { 0.0 } else { ROAD_END_MARGIN }
    }

    /// Progress (m) of the finish line on a point-to-point road.
    fn finish_line(&self) -> f32 {
        self.length - ROAD_END_MARGIN
    }
}

/// Lap counting and timing based on progress along the centerline. On a
/// point-to-point road a "lap" is one run from the start line to the finish.
#[derive(Resource, Default, Debug)]
pub struct LapTimer {
    /// Completed laps or runs.
    pub laps: u32,
    /// Time in the lap or run under way; `None` until the car crosses the
    /// start line.
    pub current: Option<f32>,
    pub last: Option<f32>,
    pub best: Option<f32>,
    /// A run has been completed since the last reset (roads only).
    pub finished: bool,
    last_progress: Option<f32>,
    /// A lap only counts if the car went around, not back and forth over the line.
    passed_halfway: bool,
}

impl LapTimer {
    /// Times laps of a circuit `length` metres round.
    fn advance(&mut self, progress: f32, length: f32, dt: f32) {
        if let Some(time) = &mut self.current {
            *time += dt;
        }
        if let Some(previous) = self.last_progress {
            let crossed_forward = previous > 0.75 * length && progress < 0.25 * length;
            let crossed_backward = previous < 0.25 * length && progress > 0.75 * length;
            if crossed_forward {
                if let (Some(time), true) = (self.current, self.passed_halfway) {
                    self.complete(time);
                }
                self.current = Some(0.0);
                self.passed_halfway = false;
            } else if crossed_backward {
                self.passed_halfway = false;
            }
        }
        if (0.4 * length..0.6 * length).contains(&progress) {
            self.passed_halfway = true;
        }
        self.last_progress = Some(progress);
    }

    /// Times one run from the `start` line to the `finish` line (progress, m).
    fn advance_run(&mut self, progress: f32, start: f32, finish: f32, dt: f32) {
        if let Some(time) = &mut self.current {
            *time += dt;
            if progress >= finish {
                let time = *time;
                self.complete(time);
                self.current = None;
                self.finished = true;
            }
        } else if !self.finished
            && self
                .last_progress
                .is_some_and(|previous| previous < start && progress >= start)
        {
            self.current = Some(0.0);
        }
        self.last_progress = Some(progress);
    }

    fn complete(&mut self, time: f32) {
        self.laps += 1;
        self.last = Some(time);
        self.best = Some(self.best.map_or(time, |best| best.min(time)));
    }

    /// Abandons the lap or run under way, keeping completed results.
    fn restart(&mut self) {
        self.current = None;
        self.last_progress = None;
        self.passed_halfway = false;
        self.finished = false;
    }
}

/// Finds the surface under each wheel and the slope under the car.
fn update_ground(
    track: Res<Track>,
    params: Res<VehicleParams>,
    vehicle: Res<Vehicle>,
    mut ground: ResMut<Ground>,
) {
    ground.surfaces = vehicle
        .wheel_positions(&params)
        .map(|position| track.surface_at(track.locate(position)));
    ground.slope = track.slope_at(track.locate(vehicle.position));
}

fn time_laps(
    time: Res<Time>,
    track: Res<Track>,
    vehicle: Res<Vehicle>,
    mut timer: ResMut<LapTimer>,
) {
    let progress = track.locate(vehicle.position).progress;
    if track.closed {
        timer.advance(progress, track.length(), time.delta_secs());
    } else {
        timer.advance_run(
            progress,
            track.start_line(),
            track.finish_line(),
            time.delta_secs(),
        );
    }
}

/// Puts the selected car at rest just before the start line.
fn reset_to_start(
    track: Res<Track>,
    selected: Res<SelectedCar>,
    mut params: ResMut<VehicleParams>,
    mut vehicle: ResMut<Vehicle>,
    mut timer: ResMut<LapTimer>,
) {
    let (position, heading) = track.pose_at(track.start_line() - START_DISTANCE);
    let car = selected.0.spec();
    *params = (car.chassis)();
    // Keep the driver's choice of shifting mode and traction control.
    let mode = vehicle.powertrain.mode;
    let traction_control = vehicle.traction_control.enabled;
    *vehicle = Vehicle {
        position,
        heading,
        powertrain: Powertrain::new(car.engine, car.gearbox),
        wheel_loads: params.wheel_loads(Vec2::ZERO),
        ..default()
    };
    vehicle.powertrain.mode = mode;
    vehicle.traction_control.enabled = traction_control;
    timer.restart();
}

fn reset_on_key(
    keys: Res<ButtonInput<KeyCode>>,
    track: Res<Track>,
    selected: Res<SelectedCar>,
    params: ResMut<VehicleParams>,
    vehicle: ResMut<Vehicle>,
    timer: ResMut<LapTimer>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        reset_to_start(track, selected, params, vehicle, timer);
    }
}

/// Marks every entity drawn for the course, so it can be replaced.
#[derive(Component)]
struct CourseScenery;

/// Rebuilds the course if a different one was chosen.
#[allow(clippy::too_many_arguments)]
fn load_course(
    mut commands: Commands,
    selected: Res<SelectedCourse>,
    mut loaded: Local<Option<crate::course::Course>>,
    scenery: Query<Entity, With<CourseScenery>>,
    meshes: ResMut<Assets<Mesh>>,
    materials: ResMut<Assets<StandardMaterial>>,
    mut timer: ResMut<LapTimer>,
) {
    let initial = *loaded.get_or_insert(SelectedCourse::default().0);
    if initial == selected.0 {
        return;
    }
    *loaded = Some(selected.0);
    for entity in &scenery {
        commands.entity(entity).despawn();
    }
    let track = Track::new(selected.0.spec());
    spawn_scenery(&mut commands, &track, meshes, materials);
    commands.insert_resource(track);
    *timer = LapTimer::default();
}

fn spawn_track_meshes(
    mut commands: Commands,
    track: Res<Track>,
    meshes: ResMut<Assets<Mesh>>,
    materials: ResMut<Assets<StandardMaterial>>,
) {
    spawn_scenery(&mut commands, &track, meshes, materials);
}

fn spawn_scenery(
    commands: &mut Commands,
    track: &Track,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Colors come from the vertices; the material just passes them through.
    let ground = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.9,
        cull_mode: None,
        ..default()
    });
    let hilly = track.elevations.iter().any(|&z| z.abs() > 0.01);
    let mut mesh = if hilly {
        terrain_mesh(track)
    } else {
        grass_mesh(track)
    };
    asphalt(&mut mesh, track);
    markings(&mut mesh, track);
    commands.spawn((
        CourseScenery,
        Mesh3d(meshes.add(mesh.build())),
        MeshMaterial3d(ground),
    ));

    let post = meshes.add(Cuboid::from_size(POST_SIZE));
    let post_material = materials.add(Color::srgb(0.95, 0.95, 0.95));
    let reach = track.half_width
        + if track.curbs.iter().any(|&c| c) {
            CURB_WIDTH
        } else {
            0.0
        };
    let posts = (track.length() / POST_SPACING) as usize;
    for k in 0..=posts {
        let (center, heading) = track.pose_at(k as f32 * POST_SPACING);
        let elevation = track.elevation_at(track.locate(center));
        let left = Vec2::from_angle(heading).perp();
        for side in [-1.0, 1.0] {
            let position = center + left * side * (reach + POST_OFFSET);
            commands.spawn((
                CourseScenery,
                Mesh3d(post.clone()),
                MeshMaterial3d(post_material.clone()),
                Transform::from_translation(to_world(position, elevation + POST_SIZE.y * 0.5)),
            ));
        }
    }
}

/// Flat-shaded quads with per-quad colors, built up and then turned into one
/// mesh. Normals point straight up, which is close enough on road grades.
#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    /// Adds a quad from corners (m, ground plane) given in order around its
    /// edge, each at its own height (m).
    fn quad(&mut self, corners: [Vec2; 4], heights: [f32; 4], color: Color) {
        let base = self.positions.len() as u32;
        let color = LinearRgba::from(color).to_f32_array();
        for (corner, height) in corners.into_iter().zip(heights) {
            self.positions.push(to_world(corner, height).to_array());
            self.colors.push(color);
        }
        self.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    fn build(self) -> Mesh {
        let normals = vec![[0.0, 1.0, 0.0]; self.positions.len()];
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, self.colors)
        .with_inserted_indices(Indices::U32(self.indices))
    }
}

const GRASS_LIGHT: Color = Color::srgb(0.22, 0.38, 0.2);
const GRASS_DARK: Color = Color::srgb(0.2, 0.35, 0.18);

fn track_bounds(track: &Track) -> (Vec2, Vec2) {
    track
        .points
        .iter()
        .fold((Vec2::MAX, Vec2::MIN), |(min, max), &p| {
            (min.min(p), max.max(p))
        })
}

/// Flat courses: a mown-grass checkerboard covering the track and its
/// surroundings.
fn grass_mesh(track: &Track) -> QuadMesh {
    let (min, max) = track_bounds(track);
    let first = ((min - GRASS_MARGIN) / GRASS_TILE).floor().as_ivec2();
    let last = ((max + GRASS_MARGIN) / GRASS_TILE).ceil().as_ivec2();

    let mut mesh = QuadMesh::default();
    for x in first.x..last.x {
        for y in first.y..last.y {
            let corner = Vec2::new(x as f32, y as f32) * GRASS_TILE;
            let color = if (x + y) % 2 == 0 {
                GRASS_LIGHT
            } else {
                GRASS_DARK
            };
            mesh.quad(
                [
                    corner,
                    corner + Vec2::X * GRASS_TILE,
                    corner + Vec2::splat(GRASS_TILE),
                    corner + Vec2::Y * GRASS_TILE,
                ],
                [0.0; 4],
                color,
            );
        }
    }
    mesh
}

/// Hilly courses: a grid of ground near the road, each vertex at the height
/// of the nearest bit of road (a little below it).
fn terrain_mesh(track: &Track) -> QuadMesh {
    let (min, max) = track_bounds(track);
    let first = ((min - TERRAIN_REACH) / TERRAIN_CELL).floor().as_ivec2();
    let last = ((max + TERRAIN_REACH) / TERRAIN_CELL).ceil().as_ivec2();
    let columns = (last.x - first.x + 1) as usize;
    let vertex = |x: i32, y: i32| Vec2::new(x as f32, y as f32) * TERRAIN_CELL;

    // Height of every grid vertex from the nearest road, or None if it is
    // far from the road, and whether it is close enough to stay put.
    let count = track.segment_count();
    let (mut heights, anchored): (Vec<Option<f32>>, Vec<bool>) = (first.y..=last.y)
        .flat_map(|y| (first.x..=last.x).map(move |x| (x, y)))
        .map(|(x, y)| {
            let location = track.nearest(vertex(x, y), 0..count);
            let distance = location.offset.abs();
            (
                (distance <= TERRAIN_REACH).then(|| track.elevation_at(location) - TERRAIN_DROP),
                distance <= track.half_width + TERRAIN_ANCHOR,
            )
        })
        .unzip();
    let rows = (last.y - first.y + 1) as usize;
    for _ in 0..TERRAIN_SMOOTHING_PASSES {
        let previous = heights.clone();
        for row in 0..rows {
            for column in 0..columns {
                let index = row * columns + column;
                let Some(own) = previous[index] else { continue };
                if anchored[index] {
                    continue;
                }
                let neighbours = [
                    (row > 0).then(|| index - columns),
                    (row + 1 < rows).then(|| index + columns),
                    (column > 0).then(|| index - 1),
                    (column + 1 < columns).then(|| index + 1),
                ];
                let (sum, n) = neighbours
                    .into_iter()
                    .flatten()
                    .filter_map(|i| previous[i])
                    .fold((own, 1.0), |(sum, n), h| (sum + h, n + 1.0));
                heights[index] = Some(sum / n);
            }
        }
    }
    let height =
        |x: i32, y: i32| heights[(y - first.y) as usize * columns + (x - first.x) as usize];

    let mut mesh = QuadMesh::default();
    for y in first.y..last.y {
        for x in first.x..last.x {
            let corners = [(x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1)];
            let Some(heights) = corners
                .iter()
                .map(|&(cx, cy)| height(cx, cy))
                .collect::<Option<Vec<f32>>>()
            else {
                continue;
            };
            let color = if (x / 2 + y / 2) % 2 == 0 {
                GRASS_LIGHT
            } else {
                GRASS_DARK
            };
            mesh.quad(
                corners.map(|(cx, cy)| vertex(cx, cy)),
                [heights[0], heights[1], heights[2], heights[3]],
                color,
            );
        }
    }
    mesh
}

/// Strip of road surface between `inner` and `outer` offsets (m, positive
/// left) alongside each centerline segment, `lift` (m) above the road,
/// colored by `color` (`None` skips it).
fn strip(
    mesh: &mut QuadMesh,
    track: &Track,
    inner: f32,
    outer: f32,
    lift: f32,
    mut color: impl FnMut(usize) -> Option<Color>,
) {
    for i in 0..track.segment_count() {
        let Some(color) = color(i) else { continue };
        let j = track.next(i);
        let (normal_i, normal_j) = (track.tangent(i).perp(), track.tangent(j).perp());
        let (p, q) = (track.points[i], track.points[j]);
        let (zi, zj) = (track.elevations[i] + lift, track.elevations[j] + lift);
        mesh.quad(
            [
                p + normal_i * inner,
                q + normal_j * inner,
                q + normal_j * outer,
                p + normal_i * outer,
            ],
            [zi, zj, zj, zi],
            color,
        );
    }
}

fn asphalt(mesh: &mut QuadMesh, track: &Track) {
    let asphalt = Color::srgb(0.27, 0.27, 0.29);
    let half = track.half_width;
    strip(mesh, track, -half, half, ASPHALT_HEIGHT, |_| Some(asphalt));
}

fn markings(mesh: &mut QuadMesh, track: &Track) {
    let white = Color::srgb(0.92, 0.92, 0.92);
    let red = Color::srgb(0.8, 0.12, 0.1);
    let yellow = Color::srgb(0.95, 0.75, 0.1);
    let half = track.half_width;

    // Edge lines just inside the edges.
    for side in [-1.0, 1.0] {
        let outer = side * (half - EDGE_LINE_INSET);
        let inner = side * (half - EDGE_LINE_INSET - EDGE_LINE_WIDTH);
        strip(mesh, track, inner, outer, MARKINGS_HEIGHT, |_| Some(white));
    }

    if track.center_line {
        let line = CENTER_LINE_WIDTH * 0.5;
        strip(mesh, track, -line, line, MARKINGS_HEIGHT, |_| Some(yellow));
    }

    // Red and white curbs outside the edges of tight sections, one color per sample.
    for side in [-1.0, 1.0] {
        strip(
            mesh,
            track,
            side * half,
            side * (half + CURB_WIDTH),
            MARKINGS_HEIGHT,
            |i| track.curbs[i].then_some(if i % 2 == 0 { red } else { white }),
        );
    }

    let mut lines = vec![track.start_line()];
    if !track.closed {
        lines.push(track.finish_line());
    }
    for progress in lines {
        chequered_line(mesh, track, progress);
    }
}

/// Chequered line across the road at `progress`, two rows of squares.
fn chequered_line(mesh: &mut QuadMesh, track: &Track, progress: f32) {
    let white = Color::srgb(0.92, 0.92, 0.92);
    let black = Color::srgb(0.05, 0.05, 0.05);
    let (origin, heading) = track.pose_at(progress);
    let height = track.elevation_at(track.locate(origin)) + MARKINGS_HEIGHT + 0.01;
    let forward = Vec2::from_angle(heading);
    let left = forward.perp();
    let half = track.half_width;
    let squares = (2.0 * half).round() as i32;
    let size = 2.0 * half / squares as f32;
    for row in 0..2 {
        for column in 0..squares {
            let corner =
                origin + forward * (row as f32 - 1.0) * size + left * (column as f32 * size - half);
            let color = if (row + column) % 2 == 0 {
                white
            } else {
                black
            };
            mesh.quad(
                [
                    corner,
                    corner + forward * size,
                    corner + (forward + left) * size,
                    corner + left * size,
                ],
                [height; 4],
                color,
            );
        }
    }
}

/// Dense points on a closed centripetal Catmull-Rom spline through `controls`.
fn closed_catmull_rom(controls: &[Vec2]) -> Vec<Vec2> {
    let n = controls.len();
    let mut points = Vec::with_capacity(n * SPLINE_STEPS_PER_SEGMENT);
    for i in 0..n {
        let p = [
            controls[(i + n - 1) % n],
            controls[i],
            controls[(i + 1) % n],
            controls[(i + 2) % n],
        ];
        // Centripetal parameterization avoids cusps and self-intersections.
        let mut t = [0.0; 4];
        for k in 1..4 {
            t[k] = t[k - 1] + p[k - 1].distance(p[k]).sqrt();
        }
        let lerp =
            |a: Vec2, b: Vec2, ta: f32, tb: f32, at: f32| a + (b - a) * ((at - ta) / (tb - ta));
        for step in 0..SPLINE_STEPS_PER_SEGMENT {
            let at = t[1] + (t[2] - t[1]) * step as f32 / SPLINE_STEPS_PER_SEGMENT as f32;
            let a1 = lerp(p[0], p[1], t[0], t[1], at);
            let a2 = lerp(p[1], p[2], t[1], t[2], at);
            let a3 = lerp(p[2], p[3], t[2], t[3], at);
            let b1 = lerp(a1, a2, t[0], t[2], at);
            let b2 = lerp(a2, a3, t[1], t[3], at);
            points.push(lerp(b1, b2, t[1], t[2], at));
        }
    }
    points
}

/// Points along the closed polyline `dense`, about `spacing` apart.
fn resample_closed(dense: &[Vec2], spacing: f32) -> Vec<Vec2> {
    let mut points = vec![dense[0]];
    let mut travelled = 0.0;
    for i in 1..=dense.len() {
        let point = dense[i % dense.len()];
        travelled += dense[i - 1].distance(point);
        if travelled >= spacing {
            points.push(point);
            travelled = 0.0;
        }
    }
    // The loop may end with a point almost on top of the first one.
    if points.last().unwrap().distance(points[0]) < spacing * 0.5 {
        points.pop();
    }
    points
}

/// Points and elevations exactly `spacing` apart along the open polyline
/// `data` (x, y, elevation), ending at its last point.
fn resample_open(data: &[[f32; 3]], spacing: f32) -> (Vec<Vec2>, Vec<f32>) {
    let mut points = vec![Vec2::new(data[0][0], data[0][1])];
    let mut elevations = vec![data[0][2]];
    let mut next_at = spacing;
    let mut travelled = 0.0;
    for pair in data.windows(2) {
        let (a, b) = (
            Vec2::new(pair[0][0], pair[0][1]),
            Vec2::new(pair[1][0], pair[1][1]),
        );
        let length = a.distance(b);
        while travelled + length >= next_at {
            let t = (next_at - travelled) / length;
            points.push(a.lerp(b, t));
            elevations.push(pair[0][2] + (pair[1][2] - pair[0][2]) * t);
            next_at += spacing;
        }
        travelled += length;
    }
    let last = data[data.len() - 1];
    if points.last().unwrap().distance(Vec2::new(last[0], last[1])) > spacing * 0.25 {
        points.push(Vec2::new(last[0], last[1]));
        elevations.push(last[2]);
    }
    (points, elevations)
}

/// Radius of the circle through three points.
fn circumradius(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    let twice_area = (b - a).perp_dot(c - a).abs();
    if twice_area < 1e-6 {
        return f32::INFINITY;
    }
    a.distance(b) * b.distance(c) * c.distance(a) / (2.0 * twice_area)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::course::Course;

    fn circuit() -> Track {
        Track::new(Course::Circuit.spec())
    }

    fn nihondaira() -> Track {
        Track::new(Course::Nihondaira.spec())
    }

    #[test]
    fn courses_have_their_expected_lengths() {
        let circuit = circuit().length();
        assert!(
            (1900.0..2100.0).contains(&circuit),
            "circuit {circuit:.0} m"
        );
        let road = nihondaira().length();
        assert!((6300.0..6700.0).contains(&road), "Nihondaira {road:.0} m");
    }

    #[test]
    fn corners_are_wide_enough_for_the_road_edges() {
        // The inner edge (including curbs) folds over itself if a corner is
        // tighter than the half width.
        for (track, reach) in [(circuit(), 6.0 + CURB_WIDTH), (nihondaira(), 3.5)] {
            let tightest = (0..track.points.len())
                .map(|i| track.radius_at(i))
                .fold(f32::INFINITY, f32::min);
            assert!(tightest > 2.0 * reach, "tightest radius {tightest:.1} m");
        }
    }

    #[test]
    fn circuit_never_runs_into_itself() {
        let track = circuit();
        let n = track.points.len();
        let clearance = 2.0 * (6.0 + CURB_WIDTH) + 10.0;
        for i in 0..n {
            for j in i + 1..n {
                let along = (track.distances[j] - track.distances[i])
                    .min(track.length - (track.distances[j] - track.distances[i]));
                if along > 60.0 {
                    let gap = track.points[i].distance(track.points[j]);
                    assert!(gap > clearance, "samples {i} and {j} only {gap:.1} m apart");
                }
            }
        }
    }

    #[test]
    fn locate_finds_progress_and_side_near_and_far_from_the_last_answer() {
        for track in [circuit(), nihondaira()] {
            for i in [100, track.points.len() / 2, 40] {
                let left = track.tangent(i).perp();
                let location = track.locate(track.points[i] + left * 3.0);
                assert!((location.progress - track.distances[i]).abs() < 0.5);
                assert!(
                    (location.offset - 3.0).abs() < 0.1,
                    "offset {}",
                    location.offset
                );
            }
        }
    }

    #[test]
    fn surfaces_change_from_asphalt_to_grass() {
        let track = circuit();
        let i = 50;
        let left = track.tangent(i).perp();
        let at = |point| track.surface_at(track.locate(point));
        assert_eq!(at(track.points[i]), Surface::ASPHALT);
        assert_eq!(at(track.points[i] + left * 30.0), Surface::GRASS);
    }

    #[test]
    fn start_position_is_on_the_track_just_before_the_line() {
        for track in [circuit(), nihondaira()] {
            let (position, _) = track.pose_at(track.start_line() - START_DISTANCE);
            let location = track.locate(position);
            let expected = (track.start_line() - START_DISTANCE).rem_euclid(track.length());
            assert!(location.offset.abs() < 0.01);
            assert!((location.progress - expected).abs() < 0.5);
        }
    }

    #[test]
    fn nihondaira_climbs_about_two_hundred_metres_at_road_grades() {
        let track = nihondaira();
        let climb = track.elevations.last().unwrap() - track.elevations[0];
        assert!((180.0..230.0).contains(&climb), "climb {climb:.0} m");
        // Grades on a public mountain road stay within about 12 %.
        let steepest = (0..track.segment_count())
            .map(|i| track.slope_at(track.locate_on(track.points[i], i)).length())
            .fold(0.0, f32::max);
        assert!(steepest < 0.15, "steepest grade {:.0} %", steepest * 100.0);
    }

    #[test]
    fn lap_timer_counts_full_laps_only() {
        let length = 1000.0;
        let dt = 0.01;
        let mut timer = LapTimer::default();
        // Out lap from just before the line, then two laps at 100 m/s.
        let mut progress = length - 50.0;
        for _ in 0..2100 {
            timer.advance(progress, length, dt);
            progress = (progress + 100.0 * dt) % length;
        }
        assert_eq!(timer.laps, 2);
        let last = timer.last.unwrap();
        assert!((last - 10.0).abs() < 0.02, "lap time {last}");

        // Reversing back over the line and forward again is not a lap.
        let mut timer = LapTimer::default();
        for progress in [990.0, 5.0, 995.0, 10.0] {
            timer.advance(progress, length, dt);
        }
        assert_eq!(timer.laps, 0);
    }

    #[test]
    fn run_timer_times_start_to_finish_once() {
        let dt = 0.01;
        let mut timer = LapTimer::default();
        let mut progress = 10.0;
        while progress < 1000.0 {
            timer.advance_run(progress, 30.0, 970.0, dt);
            progress += 50.0 * dt;
        }
        assert!(timer.finished);
        let last = timer.last.unwrap();
        assert!((last - 18.8).abs() < 0.05, "run time {last}");
        assert!(timer.current.is_none(), "no second run without a reset");
    }
}
