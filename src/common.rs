use std::num::FpCategory;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalFriction(pub f32);

impl Default for GlobalFriction {
    fn default() -> Self {
        Self(0.95)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mass {
    Infinite,
    Real(f32),
}

impl Mass {
    pub fn inverse(self) -> f32 {
        match self {
            Self::Infinite => 0.0,
            Self::Real(mass) => mass.recip(),
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Dynamic,
    Static,
    Semikinematic,
}

impl Default for Status {
    fn default() -> Self {
        Status::Dynamic
    }
}
