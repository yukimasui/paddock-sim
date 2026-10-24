//! The courses that can be driven.

use bevy::prelude::*;

use crate::courses;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Course {
    #[default]
    Circuit,
    Nihondaira,
}

/// How a course's centerline is defined.
pub enum Layout {
    /// A closed loop through hand-placed control points (m), smoothed with
    /// a spline. Flat.
    Circuit(&'static [Vec2]),
    /// An open road from imported data: x east, y north and elevation (m),
    /// already smooth and densely sampled.
    Road(&'static [[f32; 3]]),
}

pub struct CourseSpec {
    pub name: &'static str,
    /// One line about the course, for the menu.
    pub summary: &'static str,
    pub layout: Layout,
    /// Half the width of the road surface, m.
    pub half_width: f32,
    /// Red and white curbs on tight corners (a circuit, not a public road).
    pub curbs: bool,
    /// A yellow no-overtaking line down the middle, as on a Japanese
    /// two-lane road.
    pub center_line: bool,
}

/// Centerline control points of the circuit in metres, in driving order.
/// The start/finish line is at the first point, on the main straight.
const CIRCUIT_POINTS: [Vec2; 28] = [
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

static CIRCUIT: CourseSpec = CourseSpec {
    name: "Test circuit",
    summary: "2.0 km  |  flat, fast and slow corners",
    layout: Layout::Circuit(&CIRCUIT_POINTS),
    half_width: 6.0,
    curbs: true,
    center_line: false,
};

static NIHONDAIRA: CourseSpec = CourseSpec {
    name: "Nihondaira Parkway",
    summary: "6.5 km hill climb  |  Shizuoka  |  +202 m",
    layout: Layout::Road(courses::nihondaira::POINTS),
    half_width: 3.5,
    curbs: false,
    center_line: true,
};

impl Course {
    pub const ALL: [Course; 2] = [Course::Circuit, Course::Nihondaira];

    pub fn spec(self) -> &'static CourseSpec {
        match self {
            Course::Circuit => &CIRCUIT,
            Course::Nihondaira => &NIHONDAIRA,
        }
    }
}

/// The course being driven.
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct SelectedCourse(pub Course);
