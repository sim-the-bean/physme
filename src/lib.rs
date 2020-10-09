pub mod broad;
pub mod common;
pub mod dim2;
pub mod dim3;

pub mod prelude2d {
    pub use crate::common::{GlobalFriction, Mass, Status};
    pub use crate::dim2::{
        BroadPhase, DebugBody, DebugRenderPlugin, GlobalGravity, GlobalStep, Joint, Manifold,
        Physics2dPlugin, RigidBody, RotationMode, Shape, TranslationMode,
    };
}

pub mod prelude3d {
    pub use crate::common::{GlobalFriction, Mass, Status};
    pub use crate::dim3::{
        BroadPhase, GlobalGravity, GlobalStep, Joint, Manifold, Physics3dPlugin, RigidBody, Shape,
        Size3,
    };
}
