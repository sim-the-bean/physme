//! physme is a simple 2d and 3d physics library that doesn't aim for physical
//! accuracy, but instead for a good feel and ergonomics.  The engine provides
//! primitives and systems for all the physics goodies you'd expect:
//!
//! - Broad phase
//! - Narrow phase
//! - Manifold solving
//! - Physics step
//! - Draw transform synchronization
//!
//! Creating your first game with physme is as simple as adding one of the
//! physics plugin
//!
//! ```
//! # use bevy::prelude::*;
//! use physme::prelude2d::*;
//! let mut builder = App::build();
//! builder
//!     .add_plugin(Physics2dPlugin);
//! ```
//!
//! optionally setting some parameters
//!
//! ```
//! # use bevy::prelude::*;
//! # use physme::prelude2d::*;
//! # let mut builder = App::build();
//! # builder
//! #     .add_plugin(Physics2dPlugin)
//!     .add_resource(GlobalGravity(Vec2::new(0.0, -500.0)))
//!     .add_resource(GlobalFriction(0.90))
//!     .add_resource(GlobalStep(15.0));
//! ```
//!
//! and then, in your `setup` function, adding a `RigidBody` component to your
//! entities
//!
//! ```
//! # use bevy::prelude::*;
//! # use physme::prelude2d::*;
//! fn setup(
//!     mut commands: Commands,
//!     asset_server: Res<AssetServer>,
//!     mut materials: ResMut<Assets<ColorMaterial>>,
//! ) {
//!     let icon = asset_server.load("assets/icon.png").unwrap();
//!     commands
//!         .spawn(SpriteComponents {
//!             material: materials.add(icon.into()),
//!             ..Default::default()
//!         })
//!         .with(
//!             RigidBody::new(Mass::Real(1.0))
//!                 .with_status(Status::Semikinematic)
//!                 .with_position(Vec2::new(0.0, 0.0))
//!                 .with_terminal(Vec2::new(500.0, 1000.0)),
//!         );
//! }
//! ```
//!
//! as well as some children with an arbitrary amount of  `Shape` components.
//!
//! ```
//! # use bevy::prelude::*;
//! # use physme::prelude2d::*;
//! # fn setup(
//! #     mut commands: Commands,
//! #     asset_server: Res<AssetServer>,
//! #     mut materials: ResMut<Assets<ColorMaterial>>,
//! # ) {
//! #     let icon = asset_server.load("assets/icon.png").unwrap();
//! #     commands
//! #         .spawn(SpriteComponents {
//! #             material: materials.add(icon.into()),
//! #             ..Default::default()
//! #         })
//! #         .with(
//! #             RigidBody::new(Mass::Real(1.0))
//! #                 .with_status(Status::Semikinematic)
//! #                 .with_position(Vec2::new(0.0, 0.0))
//! #                 .with_terminal(Vec2::new(500.0, 1000.0)),
//! #         )
//!         .with_children(|parent| {
//!             parent.spawn((Shape::from(Size::new(28.0, 28.0)),));
//!         });
//! # }
//! ```
//!
//! And there you go! This will perform all the physics updates on every
//! frame of the game.

pub mod broad;
pub mod common;
pub mod dim2;
pub mod dim3;

pub mod prelude2d {
    //! This module re-exports all the things you might need for 2d physics
    //! simulation.
    pub use crate::common::{GlobalFriction, Mass, Status};
    pub use crate::dim2::{
        AngularTolerance, BroadPhase, GlobalGravity, GlobalStep, GlobalUp, Joint, Manifold,
        Physics2dPlugin, RigidBody, RotationMode, Shape, TranslationMode,
    };
}

pub mod prelude3d {
    //! This module re-exports all the things you might need for 3d physics
    //! simulation.
    pub use crate::common::{GlobalFriction, Mass, Status, Vec3Ext};
    pub use crate::dim3::{
        AngularTolerance, BroadPhase, GlobalGravity, GlobalStep, GlobalUp, Joint, Manifold,
        Physics3dPlugin, RigidBody, Shape, Size3, Up, UpRotation,
    };
}
