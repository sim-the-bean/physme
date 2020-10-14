//! This module provides the primitives and systems for 3d physics simulation.
//!
//! For examples, see the root of the crate.

use std::cmp::Ordering;
use std::mem;

use bevy::math::*;
use bevy::prelude::*;
use hashbrown::HashSet;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use crate::broad::{self, BoundingBox, Collider};
use crate::common::*;

mod collision;

/// This is what you want to add to your `App` if you want to run 3d physics simulation.
pub struct Physics3dPlugin;

pub mod stage {
    #[doc(hidden)]
    pub use bevy::prelude::stage::*;

    pub const COLLIDING_JOINT: &str = "colliding_joint_3d";
    pub const PHYSICS_STEP: &str = "physics_step_3d";
    pub const BROAD_PHASE: &str = "broad_phase_3d";
    pub const NARROW_PHASE: &str = "narrow_phase_3d";
    pub const PHYSICS_SOLVE: &str = "physics_solve_3d";
    pub const RIGID_JOINT: &str = "rigid_join_3d";
    pub const SYNC_TRANSFORM: &str = "sync_transform_3d";
}

impl Plugin for Physics3dPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(GlobalFriction::default())
            .add_resource(GlobalGravity::default())
            .add_resource(GlobalUp::default())
            .add_resource(GlobalStep::default())
            .add_resource(AngularTolerance::default())
            .add_event::<Manifold>()
            .add_stage_before(stage::UPDATE, stage::PHYSICS_STEP)
            .add_stage_before(stage::PHYSICS_STEP, stage::COLLIDING_JOINT)
            .add_stage_after(stage::PHYSICS_STEP, stage::BROAD_PHASE)
            .add_stage_after(stage::BROAD_PHASE, stage::NARROW_PHASE)
            .add_stage_after(stage::NARROW_PHASE, stage::PHYSICS_SOLVE)
            .add_stage_after(stage::PHYSICS_SOLVE, stage::RIGID_JOINT)
            .add_stage_after(stage::RIGID_JOINT, stage::SYNC_TRANSFORM);
        let physics_step = PhysicsStep::default().system(app.resources_mut());
        app.add_system_to_stage(stage::PHYSICS_STEP, physics_step)
            .add_system_to_stage(stage::BROAD_PHASE, broad_phase_system.system())
            .add_system_to_stage(stage::NARROW_PHASE, narrow_phase_system.system());
        let solver = Solver::default().system(app.resources_mut());
        app.add_system_to_stage(stage::PHYSICS_SOLVE, solver)
            .add_system_to_stage(stage::SYNC_TRANSFORM, sync_transform_system.system())
            .add_system_to_stage(
                FixedJointBehaviour::STAGE,
                joint_system::<FixedJointBehaviour>.system(),
            )
            .add_system_to_stage(
                MechanicalJointBehaviour::STAGE,
                joint_system::<MechanicalJointBehaviour>.system(),
            )
            .add_system_to_stage(
                SpringJointBehaviour::STAGE,
                joint_system::<SpringJointBehaviour>.system(),
            );
    }
}

pub type BroadPhase = broad::BroadPhase<Obb>;

/// The global gravity that affects every `RigidBody` with the `Semikinematic` status.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalGravity(pub Vec3);

impl Default for GlobalGravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.8, 0.0))
    }
}

/// The global step value, affects all semikinematic bodies.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalStep(pub f32);

/// The global up vector, affects all semikinematic bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlobalUp(pub Vec3);

impl Default for GlobalUp {
    fn default() -> Self {
        Self(Vec3::new(0.0, 1.0, 0.0))
    }
}

/// The global angular tolerance in radians, affects all semikinematic bodies.
///
/// This is used for step calculation and for push dynamics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularTolerance(pub f32);

impl Default for AngularTolerance {
    fn default() -> Self {
        Self(30.0_f32.to_radians())
    }
}

/// The local up vector, affects a single semikinematic body.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Properties)]
pub struct Up(pub Vec3);

impl Default for Up {
    fn default() -> Self {
        Self(Vec3::new(0.0, 1.0, 0.0))
    }
}

/// The rotation, relative to the up vector.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Properties)]
pub struct UpRotation(pub f32);

impl Default for UpRotation {
    fn default() -> Self {
        Self(0.0)
    }
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obb {
    status: Status,
    body: Entity,
    local: Transform,
    transform: Transform,
    extent: Vec3,
}

impl Obb {
    fn new(
        status: Status,
        body: Entity,
        local: Transform,
        transform: Transform,
        extent: Vec3,
    ) -> Self {
        Self {
            status,
            body,
            local,
            transform,
            extent,
        }
    }

    pub fn v0(&self) -> Vec3 {
        let v = Vec3::new(-self.extent.x(), -self.extent.y(), -self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v1(&self) -> Vec3 {
        let v = Vec3::new(self.extent.x(), -self.extent.y(), -self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v2(&self) -> Vec3 {
        let v = Vec3::new(self.extent.x(), self.extent.y(), -self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v3(&self) -> Vec3 {
        let v = Vec3::new(-self.extent.x(), self.extent.y(), -self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v4(&self) -> Vec3 {
        let v = Vec3::new(-self.extent.x(), -self.extent.y(), self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v5(&self) -> Vec3 {
        let v = Vec3::new(self.extent.x(), -self.extent.y(), self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v6(&self) -> Vec3 {
        let v = Vec3::new(self.extent.x(), self.extent.y(), self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn v7(&self) -> Vec3 {
        let v = Vec3::new(-self.extent.x(), self.extent.y(), self.extent.z());
        self.transform
            .value()
            .transform_point3(self.local.value().transform_point3(v))
    }

    pub fn min(&self) -> Vec3 {
        let min_x = self
            .v0()
            .x()
            .min(self.v1().x())
            .min(self.v2().x())
            .min(self.v3().x())
            .min(self.v4().x())
            .min(self.v5().x())
            .min(self.v6().x())
            .min(self.v7().x());
        let min_y = self
            .v0()
            .y()
            .min(self.v1().y())
            .min(self.v2().y())
            .min(self.v3().y())
            .min(self.v4().y())
            .min(self.v5().y())
            .min(self.v6().y())
            .min(self.v7().y());
        let min_z = self
            .v0()
            .z()
            .min(self.v1().z())
            .min(self.v2().z())
            .min(self.v3().z())
            .min(self.v4().z())
            .min(self.v5().z())
            .min(self.v6().z())
            .min(self.v7().z());
        Vec3::new(min_x, min_y, min_z)
    }

    pub fn max(&self) -> Vec3 {
        let max_x = self
            .v0()
            .x()
            .max(self.v1().x())
            .max(self.v2().x())
            .max(self.v3().x())
            .max(self.v4().x())
            .max(self.v5().x())
            .max(self.v6().x())
            .max(self.v7().x());
        let max_y = self
            .v0()
            .y()
            .max(self.v1().y())
            .max(self.v2().y())
            .max(self.v3().y())
            .max(self.v4().y())
            .max(self.v5().y())
            .max(self.v6().y())
            .max(self.v7().y());
        let max_z = self
            .v0()
            .z()
            .max(self.v1().z())
            .max(self.v2().z())
            .max(self.v3().z())
            .max(self.v4().z())
            .max(self.v5().z())
            .max(self.v6().z())
            .max(self.v7().z());
        Vec3::new(max_x, max_y, max_z)
    }
}

impl Collider for Obb {
    type Point = Vec3;

    fn bounding_box(&self) -> BoundingBox<Self::Point> {
        BoundingBox::new(self.min(), self.max())
    }

    fn status(&self) -> Status {
        self.status
    }
}

/// The three dimensional size of a `Shape`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Property)]
pub struct Size3 {
    pub width: f32,
    pub height: f32,
    pub depth: f32,
}

impl Size3 {
    /// Returns a new 3d size.
    pub fn new(width: f32, height: f32, depth: f32) -> Self {
        Self {
            width,
            height,
            depth,
        }
    }
}

/// The shape of a rigid body.
///
/// Contains a rotation/translation offset and a size.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Properties)]
pub struct Shape {
    local: Mat4,
    size: Size3,
}

impl Shape {
    /// Return a new `Shape` with a zero offset and a size.
    pub fn new(size: Size3) -> Self {
        let local = Mat4::identity();
        Self { local, size }
    }

    /// Return a new `Shape` with an offset and a size.
    pub fn with_local(mut self, local: Transform) -> Self {
        self.local = *local.value();
        self
    }

    pub fn extent(&self) -> Vec3 {
        Vec3::new(
            self.size.width * 0.5,
            self.size.height * 0.5,
            self.size.depth * 0.5,
        )
    }
}

impl From<Size3> for Shape {
    fn from(size: Size3) -> Self {
        Self::new(size)
    }
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnerJoint {
    body1: Entity,
    body2: Entity,
    offset: Vec3,
    angle: Quat,
}

impl InnerJoint {
    pub fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            offset: Vec3::zero(),
            angle: Quat::identity(),
        }
    }

    pub fn with_offset(mut self, offset: Vec3) -> Self {
        self.offset = offset;
        self
    }

    pub fn with_angle(mut self, angle: Quat) -> Self {
        self.angle = angle;
        self
    }
}

/// Defines a set of behaviours on how joints should move the anchored body relative to the anchor.
pub trait JointBehaviour: Send + Sync + 'static {
    const STAGE: &'static str = stage::COLLIDING_JOINT;

    /// Returns a new position for target based on `self` and `anchor`.
    fn position(
        &mut self,
        _offset: Vec3,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec3> {
        None
    }

    /// Returns a new rotation for target based on `self` and `anchor`.
    fn rotation(&mut self, _angle: Quat, _anchor: &RigidBody, _target: &RigidBody) -> Option<Quat> {
        None
    }

    /// Returns a new linear velocity for target based on `self` and `anchor`.
    fn linear_velocity(
        &mut self,
        _offset: Vec3,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec3> {
        None
    }

    /// Returns a new angular velocity for target based on `self` and `anchor`.
    fn angular_velocity(
        &mut self,
        _angle: Quat,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Quat> {
        None
    }

    /// Returns a linear impulse to apply to target based on `self` and `anchor`.
    fn linear_impulse(
        &mut self,
        _offset: Vec3,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec3> {
        None
    }

    /// Returns an angular impulse to apply to target based on `self` and `anchor`.
    fn angular_impulse(
        &mut self,
        _angle: Quat,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Quat> {
        None
    }
}

/// A joint behaviour that causes the anchored body to be rigidly fixed at an offset and an angle.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct FixedJointBehaviour;

impl JointBehaviour for FixedJointBehaviour {
    const STAGE: &'static str = stage::RIGID_JOINT;

    fn position(&mut self, offset: Vec3, anchor: &RigidBody, _target: &RigidBody) -> Option<Vec3> {
        Some(anchor.position + offset)
    }

    fn rotation(&mut self, angle: Quat, anchor: &RigidBody, _target: &RigidBody) -> Option<Quat> {
        Some(anchor.rotation * angle)
    }
}

/// A joint behaviour that causes the anchored body to be accurately positioned with an offset and an angle.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct MechanicalJointBehaviour;

impl JointBehaviour for MechanicalJointBehaviour {
    const STAGE: &'static str = stage::COLLIDING_JOINT;

    fn position(&mut self, offset: Vec3, anchor: &RigidBody, _target: &RigidBody) -> Option<Vec3> {
        Some(anchor.position + offset)
    }

    fn rotation(&mut self, angle: Quat, anchor: &RigidBody, _target: &RigidBody) -> Option<Quat> {
        Some(anchor.rotation * angle)
    }

    fn linear_velocity(
        &mut self,
        _offset: Vec3,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec3> {
        Some(Vec3::zero())
    }

    fn angular_velocity(
        &mut self,
        _angle: Quat,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Quat> {
        Some(Quat::identity())
    }
}

/// A joint behaviour that will move the anchored body into a position and angle relative to the anchor over time.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct SpringJointBehaviour {
    rigidness: f32,
}

impl SpringJointBehaviour {
    /// Create a new SpringJointBehaviour with an exact rigidness value.
    ///
    /// Rigidness describes how "snappy" the spring joint is. When it's at 0.0,
    /// the anchored body will "jump" into position softly over one second.
    /// When it's at 1.0, the anchored body will "jump" into position almost instantaenously.
    pub fn new(rigidness: f32) -> Option<SpringJointBehaviour> {
        if rigidness < 0.0 || rigidness >= 1.0 {
            None
        } else {
            Some(Self { rigidness })
        }
    }

    pub fn new_lossy(rigidness: f32) -> SpringJointBehaviour {
        Self {
            rigidness: rigidness.max(0.0).min(1.0),
        }
    }
}

impl JointBehaviour for SpringJointBehaviour {
    const STAGE: &'static str = stage::COLLIDING_JOINT;

    fn linear_velocity(
        &mut self,
        _offset: Vec3,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec3> {
        Some(Vec3::zero())
    }

    fn angular_velocity(
        &mut self,
        _angle: Quat,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Quat> {
        Some(Quat::identity())
    }

    fn linear_impulse(
        &mut self,
        offset: Vec3,
        anchor: &RigidBody,
        target: &RigidBody,
    ) -> Option<Vec3> {
        // the minimum time to "jump" into position
        const EPSILON: f32 = 0.1;
        // the maximum time to "jump" into position
        const T: f32 = 1.0;
        let springiness = 1.0 - self.rigidness;
        let position = anchor.position + offset;
        let d = position - target.position;
        let scale = (T - EPSILON) * springiness + EPSILON;
        let impulse = d * target.mass / scale;
        Some(impulse)
    }

    fn angular_impulse(
        &mut self,
        angle: Quat,
        anchor: &RigidBody,
        target: &RigidBody,
    ) -> Option<Quat> {
        // the minimum time to "jump" into position
        const EPSILON: f32 = 0.1;
        // the maximum time to "jump" into position
        const T: f32 = 1.0;
        let springiness = 1.0 - self.rigidness;
        let rotation = anchor.rotation * angle;
        let d = rotation * target.rotation.conjugate();
        let scale = (T - EPSILON) * springiness + EPSILON;
        let (axis, mut angle) = d.to_axis_angle();
        angle *= target.mass / scale;
        let impulse = Quat::from_axis_angle(axis, angle).normalize();
        Some(impulse)
    }
}

/// Allows one `RigidBody` to be anchored at another one
/// in a fixed way, along with a local offset and angle.
pub type FixedJoint = Joint<FixedJointBehaviour>;

/// Allows one `RigidBody` to be anchored at another one
/// in a physically accurate way, along with a local offset and angle.
pub type MechanicalJoint = Joint<MechanicalJointBehaviour>;

/// Allows one `RigidBody` to be anchored at another one
/// in a spring-y way, along with a local offset and angle.
pub type SpringJoint = Joint<SpringJointBehaviour>;

impl SpringJoint {
    /// Add a rigidness value to an owned `Joint`.
    pub fn with_rigidness(mut self, rigidness: f32) -> Self {
        self.behaviour.rigidness = rigidness.max(0.0).min(1.0);
        self
    }
}

/// Allows one `RigidBody` to be anchored at another one
/// in a pre-defined way, along with a local offset and angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Joint<B: JointBehaviour> {
    inner: InnerJoint,
    behaviour: B,
}

impl<B: JointBehaviour + Default> Joint<B> {
    /// Create a new joint, where the second body shall be anchored at the first body.
    pub fn new(body1: Entity, body2: Entity) -> Self {
        Self::with_behaviour(body1, body2, B::default())
    }
}

impl<B: JointBehaviour> Joint<B> {
    /// Create a new joint, where the second body shall be anchored at the first body.
    pub fn with_behaviour(body1: Entity, body2: Entity, behaviour: B) -> Self {
        Self {
            inner: InnerJoint::new(body1, body2),
            behaviour,
        }
    }

    /// Add an offset to an owned `Joint`.
    pub fn with_offset(self, offset: Vec3) -> Self {
        Self {
            inner: self.inner.with_offset(offset),
            behaviour: self.behaviour,
        }
    }

    /// Add an angle to an owned `Joint`.
    pub fn with_angle(self, angle: Quat) -> Self {
        Self {
            inner: self.inner.with_angle(angle),
            behaviour: self.behaviour,
        }
    }
}

/// The rigid body.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Properties)]
pub struct RigidBody {
    /// Current position of this rigid body.
    pub position: Vec3,
    lowest_position: Vec3,
    /// Current rotation of this rigid body.
    ///
    /// NOTE: collisions checks may or may not be broken if this is not a multiple of 90 degrees.
    pub rotation: Quat,
    /// Current linear velocity of this rigid body.
    pub linvel: Vec3,
    prev_linvel: Vec3,
    /// The terminal linear velocity of a semikinematic body.
    ///
    /// Defaults to `f32::INFINITY`.
    pub terminal: Vec3,
    accumulator: Vec3,
    dynamic_acc: Vec3,
    /// Current angular velocity of this rigid body.
    pub angvel: Quat,
    prev_angvel: Quat,
    /// The terminal angular velocity of a semikinematic body.
    ///
    /// Defaults to `f32::INFINITY`.
    pub ang_term: f32,
    /// The status, i.e. static or semikinematic.
    ///
    /// Affects how forces and collisions affect this rigid body.
    pub status: Status,
    mass: f32,
    inv_mass: f32,
    active: bool,
    sensor: bool,
}

impl RigidBody {
    /// Returns a new `RigidBody` with just a mass and all other components set to their defaults.
    pub fn new(mass: Mass) -> Self {
        Self {
            position: Vec3::zero(),
            lowest_position: Vec3::zero(),
            rotation: Quat::identity(),
            linvel: Vec3::zero(),
            prev_linvel: Vec3::zero(),
            terminal: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            accumulator: Vec3::zero(),
            dynamic_acc: Vec3::zero(),
            angvel: Quat::identity(),
            prev_angvel: Quat::identity(),
            ang_term: f32::INFINITY,
            status: Status::Semikinematic,
            mass: mass.scalar(),
            inv_mass: mass.inverse(),
            active: true,
            sensor: false,
        }
    }

    /// Returns a `RigidBody` identical to this one, but with the position set to a new one.
    pub fn with_position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the rotation set to a new one.
    pub fn with_rotation(mut self, rotation: Quat) -> Self {
        self.rotation = rotation;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the linear velocity set to a new one.
    pub fn with_linear_velocity(mut self, linvel: Vec3) -> Self {
        self.linvel = linvel;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the linear velocity set to a new one.
    pub fn with_angular_velocity(mut self, angvel: Quat) -> Self {
        self.angvel = angvel;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the terminal linear velocity set to a new one.
    pub fn with_terminal(mut self, terminal: Vec3) -> Self {
        self.terminal = terminal;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the terminal linear velocity set to a new one.
    pub fn with_angular_terminal(mut self, terminal: f32) -> Self {
        self.ang_term = terminal;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the acceleration set to a new one.
    pub fn with_acceleration(mut self, acceleration: Vec3) -> Self {
        self.accumulator = acceleration;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the status set to a new one.
    pub fn with_status(mut self, status: Status) -> Self {
        self.status = status;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the active flag set to a new one.
    pub fn with_active(mut self, active: bool) -> Self {
        self.active = active;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the sensor flag set to a new one.
    pub fn with_sensor(mut self, sensor: bool) -> Self {
        self.sensor = sensor;
        self
    }

    /// Applies an impulse to the `RigidBody`s linear velocity.
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {
        self.linvel += impulse * self.inv_mass;
    }

    /// Applies an impulse to the `RigidBody`s linear velocity.
    pub fn apply_angular_impulse(&mut self, impulse: Quat) {
        let (axis, mut angle) = impulse.to_axis_angle();
        angle *= self.inv_mass;
        self.angvel *= Quat::from_axis_angle(axis, angle);
    }

    /// Applies a force to the `RigidBody`s acceleration accumulator.
    pub fn apply_force(&mut self, force: Vec3) {
        self.accumulator += force * self.inv_mass;
    }

    /// Gets the active flag.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Gets the sensor flag.
    pub fn is_sensor(&self) -> bool {
        self.sensor
    }

    /// Gets the mass
    pub fn mass(&self) -> f32 {
        self.mass
    }

    /// Gets the mass
    pub fn inverse_mass(&self) -> f32 {
        self.inv_mass
    }

    /// Sets the active flag.
    pub fn set_active(&mut self, active: bool) {
        self.active = active;
    }

    /// Sets the sensor flag.
    pub fn set_sensor(&mut self, sensor: bool) {
        self.sensor = sensor;
    }

    /// Sets the mass.
    pub fn set_mass(&mut self, mass: Mass) {
        self.mass = mass.scalar();
        self.inv_mass = mass.inverse();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub position: Vec3,
    pub penetration: f32,
}

/// The manifold, representing detailed data on a collision between two `RigidBody`s.
///
/// Usable as an event.
#[derive(Debug, Clone)]
pub struct Manifold {
    /// The first entity.
    pub body1: Entity,
    /// The second entity.
    pub body2: Entity,
    /// The normals, relative to the second entity.
    pub normal: Vec3,
    pub penetration: f32,
    pub contacts: SmallVec<[Contact; 8]>,
}

pub fn broad_phase_system(
    mut commands: Commands,
    mut query: Query<(Entity, &RigidBody, &Children)>,
    query2: Query<&Shape>,
) {
    let mut colliders = Vec::new();
    for (entity, body, children) in &mut query.iter() {
        for &e in children.iter() {
            if let Ok(shape) = query2.get::<Shape>(e) {
                let collider = Obb::new(
                    body.status,
                    entity,
                    Transform::new(shape.local),
                    Transform::from_translation_rotation(body.position, body.rotation),
                    shape.extent(),
                );
                colliders.push(collider);
            }
        }
    }
    let broad = BroadPhase::with_colliders(colliders);
    commands.insert_resource(broad);
}

#[derive(Default)]
pub struct NarrowPhase {
    set: HashSet<[Entity; 2]>,
}

impl NarrowPhase {
    pub fn system(self, res: &mut Resources) -> Box<dyn System> {
        let system = narrow_phase_system.system();
        res.insert_local(system.id(), self);
        system
    }
}

fn narrow_phase_system(
    mut state: Local<NarrowPhase>,
    mut manifolds: ResMut<Events<Manifold>>,
    broad: Res<BroadPhase>,
) {
    state.set.clear();
    let narrow = broad.iter();
    for (collider1, collider2) in narrow {
        if collider1.body == collider2.body {
            continue;
        }

        if state.set.contains(&[collider2.body, collider1.body]) {
            continue;
        }
        state.set.insert([collider1.body, collider2.body]);

        if let Some(manifold) = collision::box_to_box(&collider1, &collider2) {
            manifolds.send(manifold);
        }
    }
}

#[derive(Default)]
pub struct Solver {
    reader: EventReader<Manifold>,
}

impl Solver {
    pub fn system(self, res: &mut Resources) -> Box<dyn System> {
        let system = solve_system.system();
        res.insert_local(system.id(), self);
        system
    }
}

fn solve_system(
    mut solver: Local<Solver>,
    time: Res<Time>,
    manifolds: Res<Events<Manifold>>,
    up: Res<GlobalUp>,
    step: Res<GlobalStep>,
    ang_tol: Res<AngularTolerance>,
    query: Query<(Mut<RigidBody>, Option<Mut<Up>>)>,
) {
    let delta_time = time.delta.as_secs_f32();

    for manifold in solver.reader.iter(&manifolds) {
        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        if a.sensor || b.sensor {
            continue;
        }

        let dynamics = if a.status == Status::Semikinematic && b.status == Status::Semikinematic {
            let push_angle = up.0.abs().dot(manifold.normal.abs()).acos();
            if push_angle > ang_tol.0 {
                let sum_recip = (a.mass + b.mass).recip();
                let br = b.linvel * b.mass;
                let ar = a.linvel * a.mass;
                let rv = br * sum_recip - ar * sum_recip;

                let impulse = -rv * manifold.normal.abs();

                let a = a.linvel - impulse;
                let b = b.linvel + impulse;
                Some((a, b))
            } else {
                None
            }
        } else {
            None
        };
        mem::drop(a);
        mem::drop(b);

        let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
        match a.status {
            Status::Static => {}
            Status::Semikinematic => {
                if let Ok(mut local_up) = query.get_mut::<Up>(manifold.body1) {
                    let angle = up.0.dot(manifold.normal).acos();
                    if angle >= 0.0 && angle < ang_tol.0 {
                        local_up.0 = manifold.normal;
                    }
                }

                if let Some((impulse, _)) = dynamics {
                    a.dynamic_acc += impulse;

                    let d = manifold.normal * manifold.penetration;
                    let v = a.linvel * delta_time;
                    if v.sign() == d.sign() {
                        // nothing
                    } else {
                        let c = a.linvel.cross(manifold.normal);
                        let c = if c.length_squared() <= f32::EPSILON {
                            Vec3::new(1.0, 1.0, 1.0) - up.0
                        } else {
                            let z = if c.x().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            let y = if c.y().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            let x = if c.z().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            Vec3::new(x, y, z)
                        };
                        a.linvel *= c;
                        a.position += d;
                    }
                } else {
                    let mut solve = true;
                    let step_angle = up.0.abs().dot(manifold.normal.abs()).acos();
                    if step_angle > ang_tol.0 {
                        let up_vector = up.0;
                        if up_vector.length_squared() != 0.0 {
                            for &point in &manifold.contacts {
                                let d = point.position - a.lowest_position;
                                let s = d.dot(up_vector);
                                if s < step.0 {
                                    let diff = a.position - a.lowest_position;
                                    a.lowest_position += up_vector * s;
                                    a.position = a.lowest_position + diff;
                                    solve = false;
                                }
                            }
                        }
                    }

                    if solve {
                        let d = manifold.normal * manifold.penetration;
                        let v = a.linvel * delta_time;
                        if v.sign() == d.sign() {
                            // nothing
                        } else {
                            let c = a.linvel.cross(manifold.normal);
                            let c = if c.length_squared() <= f32::EPSILON {
                                Vec3::new(1.0, 1.0, 1.0) - up.0
                            } else {
                                let z = if c.x().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                let y = if c.y().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                let x = if c.z().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                Vec3::new(x, y, z)
                            };
                            a.linvel *= c;
                            a.position += d;
                        }
                    }
                }
            }
        }
        mem::drop(a);

        let mut b = query.get_mut::<RigidBody>(manifold.body2).unwrap();
        match b.status {
            Status::Static => {}
            Status::Semikinematic => {
                if let Ok(mut local_up) = query.get_mut::<Up>(manifold.body2) {
                    let angle = up.0.dot(manifold.normal).acos();
                    if angle >= 0.0 && angle < ang_tol.0 {
                        local_up.0 = manifold.normal;
                    }
                }

                if let Some((_, impulse)) = dynamics {
                    b.dynamic_acc += impulse;

                    let d = -manifold.normal * manifold.penetration;
                    let v = b.linvel * delta_time;
                    if v.sign() == d.sign() {
                        // nothing
                    } else {
                        let c = b.linvel.cross(manifold.normal);
                        let c = if c.length_squared() <= f32::EPSILON {
                            Vec3::new(1.0, 1.0, 1.0) - up.0
                        } else {
                            let z = if c.x().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            let y = if c.y().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            let x = if c.z().abs() <= f32::EPSILON {
                                0.0
                            } else {
                                1.0
                            };
                            Vec3::new(x, y, z)
                        };
                        b.linvel *= c;
                        b.position += d;
                    }
                } else {
                    let mut solve = true;
                    let step_angle = up.0.abs().dot(manifold.normal.abs()).acos();
                    if step_angle > ang_tol.0 {
                        let up_vector = up.0;
                        if up_vector.length_squared() != 0.0 {
                            for &point in &manifold.contacts {
                                let d = point.position - b.lowest_position;
                                let s = d.dot(up_vector);
                                if s < step.0 {
                                    let diff = b.position - b.lowest_position;
                                    b.lowest_position += up_vector * s;
                                    b.position = b.lowest_position + diff;
                                    solve = false;
                                }
                            }
                        }
                    }

                    if solve {
                        let d = -manifold.normal * manifold.penetration;
                        let v = b.linvel * delta_time;
                        if v.sign() == d.sign() {
                            // nothing
                        } else {
                            let c = b.linvel.cross(manifold.normal);
                            let c = if c.length_squared() <= f32::EPSILON {
                                Vec3::new(1.0, 1.0, 1.0) - up.0
                            } else {
                                let z = if c.x().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                let y = if c.y().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                let x = if c.z().abs() <= f32::EPSILON {
                                    0.0
                                } else {
                                    1.0
                                };
                                Vec3::new(x, y, z)
                            };
                            b.linvel *= c;
                            b.position += d;
                        }
                    }
                }
            }
        }
        mem::drop(b);
    }
}

pub struct PhysicsStep {
    skip: usize,
}

impl PhysicsStep {
    pub fn system(self, res: &mut Resources) -> Box<dyn System> {
        let system = physics_step_system.system();
        res.insert_local(system.id(), self);
        system
    }
}

impl Default for PhysicsStep {
    fn default() -> Self {
        PhysicsStep { skip: 3 }
    }
}

fn physics_step_system(
    mut state: Local<PhysicsStep>,
    time: Res<Time>,
    friction: Res<GlobalFriction>,
    gravity: Res<GlobalGravity>,
    global_up: Res<GlobalUp>,
    mut query: Query<(Mut<RigidBody>, Option<(&Up, &UpRotation)>)>,
) {
    if state.skip > 0 {
        state.skip -= 1;
        return;
    }

    let delta_time = time.delta.as_secs_f32();

    for (mut body, local_up) in &mut query.iter() {
        if !body.active {
            continue;
        }

        if !matches!(body.status, Status::Static) {
            body.accumulator += gravity.0;
        }

        let linvel = body.linvel + body.accumulator * delta_time;
        let linvel = linvel + body.dynamic_acc;
        body.linvel = linvel;
        body.accumulator = Vec3::zero();
        body.dynamic_acc = Vec3::zero();

        if matches!(body.status, Status::Semikinematic) {
            let vel = body.linvel;
            let limit = body.terminal;
            match vel.x().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.linvel.x_mut() = vel.x().max(-limit.x()),
                Some(Ordering::Greater) => *body.linvel.x_mut() = vel.x().min(limit.x()),
                Some(Ordering::Equal) => {}
                None => *body.linvel.x_mut() = 0.0,
            }
            match vel.y().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.linvel.y_mut() = vel.y().max(-limit.y()),
                Some(Ordering::Greater) => *body.linvel.y_mut() = vel.y().min(limit.y()),
                Some(Ordering::Equal) => {}
                None => *body.linvel.y_mut() = 0.0,
            }
            match vel.z().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.linvel.z_mut() = vel.z().max(-limit.z()),
                Some(Ordering::Greater) => *body.linvel.z_mut() = vel.z().min(limit.z()),
                Some(Ordering::Equal) => {}
                None => *body.linvel.z_mut() = 0.0,
            }
            let vel = body.angvel;
            let limit = body.ang_term;
            let (axis, mut angle) = vel.to_axis_angle();
            match vel.w().partial_cmp(&0.0) {
                Some(Ordering::Less) => {
                    angle = angle.max(-limit);
                    body.angvel = Quat::from_axis_angle(axis, angle);
                }
                Some(Ordering::Greater) => {
                    angle = angle.min(limit);
                    body.angvel = Quat::from_axis_angle(axis, angle);
                }
                Some(Ordering::Equal) => {}
                None => body.angvel = Quat::identity(),
            }
        }

        let position = body.position + body.linvel * delta_time;
        body.position = position;

        let (axis, mut angle) = body.angvel.to_axis_angle();
        angle *= delta_time;
        if let Some((local_up, up_rotation)) = local_up {
            let mut axis = local_up.0 * axis;
            if axis.length_squared() <= f32::EPSILON {
                axis = Vec3::new(0.0, 1.0, 0.0);
            }
            let rotation = Quat::from_axis_angle(axis, angle + up_rotation.0);
            let angle = local_up.0.quat_between(global_up.0);
            body.rotation = (rotation * angle).normalize();
        } else {
            let rotation = body.rotation * Quat::from_axis_angle(axis, angle);
            body.rotation = rotation.normalize();
        }

        match body.status {
            Status::Semikinematic => {
                if body.linvel.x().abs() <= body.prev_linvel.x().abs() {
                    *body.linvel.x_mut() *= friction.0;
                }
                if body.linvel.y().abs() <= body.prev_linvel.y().abs() {
                    *body.linvel.y_mut() *= friction.0;
                }
                if body.linvel.z().abs() <= body.prev_linvel.z().abs() {
                    *body.linvel.z_mut() *= friction.0;
                }
                if body.angvel.w().abs() <= body.prev_angvel.w().abs() {
                    let (axis, mut angle) = body.angvel.to_axis_angle();
                    angle *= friction.0;
                    body.angvel = Quat::from_axis_angle(axis, angle);
                }
            }
            Status::Static => {}
        }
        body.prev_linvel = body.linvel;
        body.prev_angvel = body.angvel;
    }
}

pub fn joint_system<B: JointBehaviour>(
    mut commands: Commands,
    mut query: Query<(Entity, Mut<Joint<B>>)>,
    bodies: Query<Mut<RigidBody>>,
) {
    for (e, mut joint) in &mut query.iter() {
        let anchor = if let Ok(anchor) = bodies.get::<RigidBody>(joint.inner.body1) {
            anchor
        } else {
            commands.despawn_recursive(e);
            continue;
        };
        let target = if let Ok(target) = bodies.get::<RigidBody>(joint.inner.body2) {
            target
        } else {
            commands.despawn_recursive(e);
            continue;
        };
        let offset = joint.inner.offset;
        let angle = joint.inner.angle;
        let position = joint.behaviour.position(offset, &anchor, &target);
        let rotation = joint.behaviour.rotation(angle, &anchor, &target);
        let linvel = joint.behaviour.linear_velocity(offset, &anchor, &target);
        let angvel = joint.behaviour.angular_velocity(angle, &anchor, &target);
        let linimp = joint.behaviour.linear_impulse(offset, &anchor, &target);
        let angimp = joint.behaviour.angular_impulse(angle, &anchor, &target);

        mem::drop(anchor);
        mem::drop(target);

        let mut target = bodies.get_mut::<RigidBody>(joint.inner.body2).unwrap();

        if let Some(position) = position {
            target.position = position;
        }

        if let Some(rotation) = rotation {
            target.rotation = rotation;
        }

        if let Some(linvel) = linvel {
            target.linvel = linvel;
        }

        if let Some(angvel) = angvel {
            target.angvel = angvel;
        }

        if let Some(linimp) = linimp {
            target.apply_linear_impulse(linimp);
        }

        if let Some(angimp) = angimp {
            target.apply_angular_impulse(angimp);
        }
    }
}

pub fn sync_transform_system(mut query: Query<(&RigidBody, Mut<Transform>)>) {
    for (body, mut transform) in &mut query.iter() {
        transform.set_translation(body.position);
        transform.set_rotation(body.rotation);
    }
}
