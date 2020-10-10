//! Commmon type definitions for 2d and 3d physics simulation.

use std::num::FpCategory;

/// The global friction that affects every `RigidBody`, both 2d and 3d.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalFriction(pub f32);

impl Default for GlobalFriction {
    fn default() -> Self {
        Self(0.95)
    }
}

/// The mass of the object or an infinite mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mass {
    /// Infinite mass (useful for immobile static objects).
    Infinite,
    /// A real mass (useful for semikinematic and mobile static objects).
    Real(f32),
}

impl Mass {
    /// Get the inverse mass.
    pub fn inverse(self) -> f32 {
        match self {
            Self::Infinite => 0.0,
            Self::Real(mass) => mass.recip(),
        }
    }

    /// Get the scalar mass.
    pub fn scalar(self) -> f32 {
        match self {
            Self::Infinite => 0.0,
            Self::Real(mass) => mass,
        }
    }
}

impl From<f32> for Mass {
    fn from(mass: f32) -> Self {
        match mass.classify() {
            FpCategory::Normal | FpCategory::Subnormal => Mass::Real(mass),
            _ => Mass::Infinite,
        }
    }
}

/// The status of a `RigidBody`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Static bodies do not check for collisions.
    ///
    /// They are normally unaffected by forces.
    /// They CAN be mobile, however only if the user sets its force/velocity manually.
    Static,
    /// Semikinematic bodies check for collisions.
    ///
    /// They are affected by forces, like gravity.
    /// Semikinematic bodies behave like kinematic bodies in some cases,
    /// and like dynamic bodies in others.  They have full collision detection with
    /// static bodies and with semikinematic bodies.  Two semikinematic bodies can
    /// push on each other based on their velocity and their mass.  A semikinematic
    /// body may stand on top of another.  Semikinematic bodies can step onto small
    /// static coliders (depending on the global step value).
    Semikinematic,
}

impl Default for Status {
    fn default() -> Self {
        Status::Semikinematic
    }
}
