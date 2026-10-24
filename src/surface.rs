//! Ground surfaces a tire can be on.

/// Properties of the ground under one tire.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Surface {
    /// Multiplier on the tire's friction coefficient.
    pub grip: f32,
    /// Rolling resistance coefficient (force / load).
    pub rolling_resistance: f32,
}

impl Surface {
    pub const ASPHALT: Self = Self {
        grip: 1.0,
        rolling_resistance: 0.012,
    };
    /// Painted concrete curbs: a little less grip than asphalt.
    pub const CURB: Self = Self {
        grip: 0.9,
        rolling_resistance: 0.015,
    };
    /// Dry grass: low grip and the tires sink in.
    pub const GRASS: Self = Self {
        grip: 0.55,
        rolling_resistance: 0.08,
    };
}
