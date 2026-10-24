//! The circuit: centerline geometry, surface lookup, rendering and lap timing.
//!
//! The track is a closed centripetal Catmull-Rom spline through hand-placed
//! control points, resampled at even spacing. Distance along the centerline
//! (progress) and signed distance from it (offset, positive to the left)
//! decide which surface a wheel is on and when a lap is completed.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::surface::Surface;
use crate::vehicle::{Vehicle, VehicleParams, WheelSurfaces, step_vehicle};
use crate::view::to_world;

/// Centerline control points in meters, in driving order. The start/finish
/// line is at the first point, on the main straight.
const CONTROL_POINTS: [Vec2; 28] = [
    Vec2::new(-150.0, 0.0),
    Vec2::new(50.0, 0.0),
    Vec2::new(250.0, 0.0),
    Vec2::new(320.0, 20.0),
    Vec2::new(350.0, 80.0),
    Vec2::new(330.0, 150.0),
    Vec2::new(270.0, 190.0),
    Vec2::new(200.0, 190.0),
    Vec2::new(165.0, 155.0),
    Vec2::new(135.0, 115.0),
    Vec2::new(95.0, 105.0),
    Vec2::new(60.0, 125.0),
    Vec2::new(40.0, 170.0),
    Vec2::new(40.0, 230.0),
    Vec2::new(55.0, 290.0),
    Vec2::new(50.0, 345.0),
    Vec2::new(25.0, 375.0),
    Vec2::new(-5.0, 380.0),
    Vec2::new(-28.0, 360.0),
    Vec2::new(-35.0, 320.0),
    Vec2::new(-60.0, 285.0),
    Vec2::new(-120.0, 270.0),
    Vec2::new(-170.0, 285.0),
    Vec2::new(-230.0, 300.0),
    Vec2::new(-290.0, 270.0),
    Vec2::new(-320.0, 200.0),
    Vec2::new(-310.0, 120.0),
    Vec2::new(-260.0, 40.0),
];

const HALF_WIDTH: f32 = 6.0; // m
const SAMPLE_SPACING: f32 = 2.0; // m
const SPLINE_STEPS_PER_SEGMENT: usize = 200;
const CURB_WIDTH: f32 = 1.2; // m, outside the track edge
/// Corners tighter than this radius (m) get curbs on both sides.
const CURB_MAX_RADIUS: f32 = 90.0;
/// Curbs extend this many samples before and after a tight section.
const CURB_EXTENSION: usize = 4;
const EDGE_LINE_WIDTH: f32 = 0.25; // m
const EDGE_LINE_INSET: f32 = 0.3; // m, from the track edge
/// The car starts this far before the start/finish line, m.
const START_DISTANCE: f32 = 15.0;
const GRASS_TILE: f32 = 20.0; // m
const GRASS_MARGIN: f32 = 300.0; // m around the track
/// Heights of the ground layers, m, stacked to avoid z-fighting.
const ASPHALT_HEIGHT: f32 = 0.02;
const MARKINGS_HEIGHT: f32 = 0.04;
/// Marker posts along both sides of the track give a sense of speed.
const POST_SPACING: f32 = 25.0; // m
const POST_OFFSET: f32 = 2.5; // m beyond the curbs
const POST_SIZE: Vec3 = Vec3::new(0.15, 1.0, 0.15);

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Track::new())
            .init_resource::<LapTimer>()
            .add_systems(Startup, (spawn_track_meshes, reset_to_start))
            .add_systems(
                FixedUpdate,
                (
                    update_wheel_surfaces.before(step_vehicle),
                    time_laps.after(step_vehicle),
                ),
            )
            .add_systems(Update, reset_on_key);
    }
}

#[derive(Resource, Debug)]
pub struct Track {
    /// Centerline samples, evenly spaced, forming a closed loop.
    points: Vec<Vec2>,
    /// Distance along the centerline to each sample, m.
    distances: Vec<f32>,
    /// Whether each sample has curbs.
    curbs: Vec<bool>,
    length: f32,
}

/// Where a point is relative to the centerline.
#[derive(Clone, Copy, Debug)]
pub struct TrackLocation {
    /// Distance along the centerline from the start/finish line, m.
    pub progress: f32,
    /// Signed distance from the centerline, m; positive to the left.
    pub offset: f32,
    /// Index of the centerline segment the point is beside.
    segment: usize,
}

impl Track {
    pub fn new() -> Self {
        let points = resample(&closed_catmull_rom(&CONTROL_POINTS), SAMPLE_SPACING);
        let n = points.len();
        let mut distances = Vec::with_capacity(n);
        let mut length = 0.0;
        for i in 0..n {
            distances.push(length);
            length += points[i].distance(points[(i + 1) % n]);
        }

        let tight: Vec<bool> = (0..n)
            .map(|i| {
                circumradius(points[(i + n - 1) % n], points[i], points[(i + 1) % n])
                    < CURB_MAX_RADIUS
            })
            .collect();
        let curbs = (0..n)
            .map(|i| (0..=2 * CURB_EXTENSION).any(|k| tight[(i + n + k - CURB_EXTENSION) % n]))
            .collect();

        Self {
            points,
            distances,
            curbs,
            length,
        }
    }

    pub fn length(&self) -> f32 {
        self.length
    }

    fn next(&self, i: usize) -> usize {
        (i + 1) % self.points.len()
    }

    /// Unit direction of travel at sample `i`.
    fn tangent(&self, i: usize) -> Vec2 {
        let n = self.points.len();
        (self.points[(i + 1) % n] - self.points[(i + n - 1) % n]).normalize()
    }

    pub fn locate(&self, point: Vec2) -> TrackLocation {
        let mut best = TrackLocation {
            progress: 0.0,
            offset: f32::INFINITY,
            segment: 0,
        };
        for i in 0..self.points.len() {
            let start = self.points[i];
            let segment = self.points[self.next(i)] - start;
            let along = ((point - start).dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
            let to_point = point - (start + segment * along);
            if to_point.length() < best.offset.abs() {
                let side = segment.perp_dot(to_point).signum();
                best = TrackLocation {
                    progress: self.distances[i] + segment.length() * along,
                    offset: to_point.length() * side,
                    segment: i,
                };
            }
        }
        best
    }

    pub fn surface_at(&self, point: Vec2) -> Surface {
        let location = self.locate(point);
        let distance = location.offset.abs();
        if distance <= HALF_WIDTH {
            Surface::ASPHALT
        } else if distance <= HALF_WIDTH + CURB_WIDTH && self.curbs[location.segment] {
            Surface::CURB
        } else {
            Surface::GRASS
        }
    }

    /// Position and heading (rad, CCW from +X) on the centerline at `progress`.
    pub fn pose_at(&self, progress: f32) -> (Vec2, f32) {
        let progress = progress.rem_euclid(self.length);
        let i = self.distances.partition_point(|&d| d <= progress) - 1;
        let start = self.points[i];
        let segment = self.points[self.next(i)] - start;
        let position = start + segment.normalize() * (progress - self.distances[i]);
        (position, segment.to_angle())
    }
}

/// Lap counting and timing based on progress along the centerline.
#[derive(Resource, Default, Debug)]
pub struct LapTimer {
    /// Completed laps.
    pub laps: u32,
    /// Time in the lap under way; `None` until the car first crosses the line.
    pub current: Option<f32>,
    pub last: Option<f32>,
    pub best: Option<f32>,
    last_progress: Option<f32>,
    /// A lap only counts if the car went around, not back and forth over the line.
    passed_halfway: bool,
}

impl LapTimer {
    fn advance(&mut self, progress: f32, length: f32, dt: f32) {
        if let Some(time) = &mut self.current {
            *time += dt;
        }
        if let Some(previous) = self.last_progress {
            let crossed_forward = previous > 0.75 * length && progress < 0.25 * length;
            let crossed_backward = previous < 0.25 * length && progress > 0.75 * length;
            if crossed_forward {
                if let (Some(time), true) = (self.current, self.passed_halfway) {
                    self.laps += 1;
                    self.last = Some(time);
                    self.best = Some(self.best.map_or(time, |best| best.min(time)));
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

    /// Abandons the lap under way, keeping the results of completed laps.
    fn restart(&mut self) {
        self.current = None;
        self.last_progress = None;
        self.passed_halfway = false;
    }
}

fn update_wheel_surfaces(
    track: Res<Track>,
    params: Res<VehicleParams>,
    vehicle: Res<Vehicle>,
    mut surfaces: ResMut<WheelSurfaces>,
) {
    surfaces.0 = vehicle
        .wheel_positions(&params)
        .map(|position| track.surface_at(position));
}

fn time_laps(
    time: Res<Time>,
    track: Res<Track>,
    vehicle: Res<Vehicle>,
    mut timer: ResMut<LapTimer>,
) {
    let location = track.locate(vehicle.position);
    timer.advance(location.progress, track.length(), time.delta_secs());
}

/// Puts the car at rest on the grid, just before the start/finish line.
fn reset_to_start(track: Res<Track>, mut vehicle: ResMut<Vehicle>, mut timer: ResMut<LapTimer>) {
    let (position, heading) = track.pose_at(-START_DISTANCE);
    // Keep the driver's choice of manual or automatic shifting.
    let mode = vehicle.powertrain.mode;
    *vehicle = Vehicle {
        position,
        heading,
        ..default()
    };
    vehicle.powertrain.mode = mode;
    timer.restart();
}

fn reset_on_key(
    keys: Res<ButtonInput<KeyCode>>,
    track: Res<Track>,
    vehicle: ResMut<Vehicle>,
    timer: ResMut<LapTimer>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        reset_to_start(track, vehicle, timer);
    }
}

fn spawn_track_meshes(
    mut commands: Commands,
    track: Res<Track>,
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
    let mut mesh = grass_mesh(&track);
    asphalt(&mut mesh, &track);
    markings(&mut mesh, &track);
    commands.spawn((Mesh3d(meshes.add(mesh.build())), MeshMaterial3d(ground)));

    let post = meshes.add(Cuboid::from_size(POST_SIZE));
    let post_material = materials.add(Color::srgb(0.95, 0.95, 0.95));
    let posts = (track.length() / POST_SPACING) as usize;
    for k in 0..posts {
        let (center, heading) = track.pose_at(k as f32 * POST_SPACING);
        let left = Vec2::from_angle(heading).perp();
        for side in [-1.0, 1.0] {
            let position = center + left * side * (HALF_WIDTH + CURB_WIDTH + POST_OFFSET);
            commands.spawn((
                Mesh3d(post.clone()),
                MeshMaterial3d(post_material.clone()),
                Transform::from_translation(to_world(position, POST_SIZE.y * 0.5)),
            ));
        }
    }
}

/// Flat quads with per-quad colors on the ground, built up and then turned
/// into one mesh.
#[derive(Default)]
struct QuadMesh {
    positions: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
}

impl QuadMesh {
    /// Adds a horizontal quad at `height` (m) from corners (m, track plane)
    /// given in order around its edge.
    fn quad(&mut self, corners: [Vec2; 4], height: f32, color: Color) {
        let base = self.positions.len() as u32;
        let color = LinearRgba::from(color).to_f32_array();
        for corner in corners {
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

/// Mown-grass checkerboard covering the track and its surroundings.
fn grass_mesh(track: &Track) -> QuadMesh {
    let light = Color::srgb(0.22, 0.38, 0.2);
    let dark = Color::srgb(0.2, 0.35, 0.18);
    let (min, max) = track
        .points
        .iter()
        .fold((Vec2::MAX, Vec2::MIN), |(min, max), &p| {
            (min.min(p), max.max(p))
        });
    let first = ((min - GRASS_MARGIN) / GRASS_TILE).floor().as_ivec2();
    let last = ((max + GRASS_MARGIN) / GRASS_TILE).ceil().as_ivec2();

    let mut mesh = QuadMesh::default();
    for x in first.x..last.x {
        for y in first.y..last.y {
            let corner = Vec2::new(x as f32, y as f32) * GRASS_TILE;
            let color = if (x + y) % 2 == 0 { light } else { dark };
            mesh.quad(
                [
                    corner,
                    corner + Vec2::X * GRASS_TILE,
                    corner + Vec2::splat(GRASS_TILE),
                    corner + Vec2::Y * GRASS_TILE,
                ],
                0.0,
                color,
            );
        }
    }
    mesh
}

/// Strip of road surface between `inner` and `outer` offsets (m, positive
/// left) alongside each centerline segment, colored by `color` (`None` skips it).
fn strip(
    mesh: &mut QuadMesh,
    track: &Track,
    inner: f32,
    outer: f32,
    height: f32,
    mut color: impl FnMut(usize) -> Option<Color>,
) {
    for i in 0..track.points.len() {
        let Some(color) = color(i) else { continue };
        let j = track.next(i);
        let (normal_i, normal_j) = (track.tangent(i).perp(), track.tangent(j).perp());
        let (p, q) = (track.points[i], track.points[j]);
        mesh.quad(
            [
                p + normal_i * inner,
                q + normal_j * inner,
                q + normal_j * outer,
                p + normal_i * outer,
            ],
            height,
            color,
        );
    }
}

fn asphalt(mesh: &mut QuadMesh, track: &Track) {
    let asphalt = Color::srgb(0.27, 0.27, 0.29);
    strip(mesh, track, -HALF_WIDTH, HALF_WIDTH, ASPHALT_HEIGHT, |_| {
        Some(asphalt)
    });
}

fn markings(mesh: &mut QuadMesh, track: &Track) {
    let white = Color::srgb(0.92, 0.92, 0.92);
    let red = Color::srgb(0.8, 0.12, 0.1);

    // Edge lines just inside the track edges.
    for side in [-1.0, 1.0] {
        let outer = side * (HALF_WIDTH - EDGE_LINE_INSET);
        let inner = side * (HALF_WIDTH - EDGE_LINE_INSET - EDGE_LINE_WIDTH);
        strip(mesh, track, inner, outer, MARKINGS_HEIGHT, |_| Some(white));
    }

    // Red and white curbs outside the edges of tight sections, one color per sample.
    for side in [-1.0, 1.0] {
        strip(
            mesh,
            track,
            side * HALF_WIDTH,
            side * (HALF_WIDTH + CURB_WIDTH),
            MARKINGS_HEIGHT,
            |i| track.curbs[i].then_some(if i % 2 == 0 { red } else { white }),
        );
    }

    // Chequered start/finish line, two rows of 1 m squares.
    let (origin, heading) = track.pose_at(0.0);
    let forward = Vec2::from_angle(heading);
    let left = forward.perp();
    let black = Color::srgb(0.05, 0.05, 0.05);
    let squares = (2.0 * HALF_WIDTH) as i32;
    for row in 0..2 {
        for column in 0..squares {
            let corner =
                origin + forward * (row as f32 - 1.0) + left * (column as f32 - HALF_WIDTH);
            let color = if (row + column) % 2 == 0 {
                white
            } else {
                black
            };
            mesh.quad(
                [
                    corner,
                    corner + forward,
                    corner + forward + left,
                    corner + left,
                ],
                MARKINGS_HEIGHT,
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
fn resample(dense: &[Vec2], spacing: f32) -> Vec<Vec2> {
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

    #[test]
    fn track_is_about_two_kilometres() {
        let length = Track::new().length();
        assert!((1900.0..2100.0).contains(&length), "length {length:.0} m");
    }

    #[test]
    fn corners_are_wide_enough_for_the_road_edges() {
        // The inner edge (including curbs) folds over itself if a corner is
        // tighter than the half width.
        let track = Track::new();
        let n = track.points.len();
        let tightest = (0..n)
            .map(|i| {
                circumradius(
                    track.points[(i + n - 1) % n],
                    track.points[i],
                    track.points[(i + 1) % n],
                )
            })
            .fold(f32::INFINITY, f32::min);
        assert!(
            tightest > 2.0 * (HALF_WIDTH + CURB_WIDTH),
            "tightest radius {tightest:.1} m"
        );
    }

    #[test]
    fn track_never_runs_into_itself() {
        let track = Track::new();
        let n = track.points.len();
        let clearance = 2.0 * (HALF_WIDTH + CURB_WIDTH) + 10.0;
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
    fn locate_finds_progress_and_side() {
        let track = Track::new();
        let i = 100;
        let left = track.tangent(i).perp();
        let location = track.locate(track.points[i] + left * 3.0);
        assert!((location.progress - track.distances[i]).abs() < 0.5);
        assert!(
            (location.offset - 3.0).abs() < 0.1,
            "offset {}",
            location.offset
        );
    }

    #[test]
    fn surfaces_change_from_asphalt_to_grass() {
        let track = Track::new();
        let i = 50;
        let left = track.tangent(i).perp();
        assert_eq!(track.surface_at(track.points[i]), Surface::ASPHALT);
        assert_eq!(
            track.surface_at(track.points[i] + left * 30.0),
            Surface::GRASS
        );
    }

    #[test]
    fn start_position_is_on_the_track_just_before_the_line() {
        let track = Track::new();
        let (position, _) = track.pose_at(-START_DISTANCE);
        let location = track.locate(position);
        assert!(location.offset.abs() < 0.01);
        assert!((location.progress - (track.length() - START_DISTANCE)).abs() < 0.5);
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
}
