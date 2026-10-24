//! The cars that can be driven.
//!
//! Every car is drawn with the same body for now; what differs is the
//! chassis (mass, weight split, tires, brakes), the engine, the gearbox and
//! the engine sound.

use bevy::prelude::*;

use crate::engine::{EngineSpec, FA24, ONE_LR_GUE};
use crate::engine_sound::{FA24_SOUND, ONE_LR_GUE_SOUND, SoundSpec};
use crate::powertrain::{AISIN_6AT, GearboxSpec, LFA_6_SPEED};
use crate::vehicle::VehicleParams;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Car {
    #[default]
    Brz,
    Lfa,
}

pub struct CarSpec {
    pub name: &'static str,
    /// One line about the engine and gearbox, for the menu.
    pub summary: &'static str,
    pub chassis: fn() -> VehicleParams,
    pub engine: &'static EngineSpec,
    pub gearbox: &'static GearboxSpec,
    pub sound: &'static SoundSpec,
}

static BRZ: CarSpec = CarSpec {
    name: "Subaru BRZ",
    summary: "FA24 2.4 L flat-four  |  6-speed automatic",
    chassis: VehicleParams::default,
    engine: &FA24,
    gearbox: &AISIN_6AT,
    sound: &FA24_SOUND,
};

static LFA: CarSpec = CarSpec {
    name: "Lexus LFA",
    summary: "1LR-GUE 4.8 L V10  |  6-speed sequential",
    chassis: VehicleParams::lfa,
    engine: &ONE_LR_GUE,
    gearbox: &LFA_6_SPEED,
    sound: &ONE_LR_GUE_SOUND,
};

impl Car {
    pub const ALL: [Car; 2] = [Car::Brz, Car::Lfa];

    pub fn spec(self) -> &'static CarSpec {
        match self {
            Car::Brz => &BRZ,
            Car::Lfa => &LFA,
        }
    }
}

/// The car being driven.
#[derive(Resource, Default, Clone, Copy, Debug)]
pub struct SelectedCar(pub Car);
