//! This module provides the primitives and systems for 2d physics simulation.
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

/// This is what you want to add to your `App` if you want to run 2d physics simulation.
pub struct Physics2dPlugin;

pub mod stage {
    #[doc(hidden)]
    pub use bevy::prelude::stage::*;

    pub const COLLIDING_JOINT: &str = "colliding_joint";
    pub const PHYSICS_STEP: &str = "physics_step";
    pub const BROAD_PHASE: &str = "broad_phase";
    pub const NARROW_PHASE: &str = "narrow_phase";
    pub const PHYSICS_SOLVE: &str = "physics_solve";
    pub const RIGID_JOINT: &str = "rigid_joint";
    pub const SYNC_TRANSFORM: &str = "sync_transform";
}

impl Plugin for Physics2dPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(GlobalFriction::default())
            .add_resource(GlobalGravity::default())
            .add_resource(TranslationMode::default())
            .add_resource(RotationMode::default())
            .add_resource(GlobalStep::default())
            .add_resource(GlobalUp::default())
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
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalGravity(pub Vec2);

/// The global step value, affects all semikinematic bodies.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalStep(pub f32);

/// The global up vector, affects all semikinematic bodies.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalUp(pub Vec2);

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

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obb {
    status: Status,
    body: Entity,
    position: Vec2,
    rotation: Mat2,
    vertices: [Vec2; 4],
    normals: [Vec2; 4],
}

impl Obb {
    fn new(
        status: Status,
        body: Entity,
        rotation: Mat2,
        position: Vec2,
        v0: Vec2,
        v1: Vec2,
        v2: Vec2,
        v3: Vec2,
        n0: Vec2,
        n1: Vec2,
    ) -> Self {
        Self {
            status,
            body,
            rotation,
            position,
            vertices: [v0, v1, v2, v3],
            normals: [-n1, n0, n1, -n0],
        }
    }

    pub fn v0(&self) -> Vec2 {
        self.rotation * self.vertices[0] + self.position
    }

    pub fn v1(&self) -> Vec2 {
        self.rotation * self.vertices[1] + self.position
    }

    pub fn v2(&self) -> Vec2 {
        self.rotation * self.vertices[2] + self.position
    }

    pub fn v3(&self) -> Vec2 {
        self.rotation * self.vertices[3] + self.position
    }

    pub fn min(&self) -> Vec2 {
        let min_x = self
            .v0()
            .x()
            .min(self.v1().x())
            .min(self.v2().x())
            .min(self.v3().x());
        let min_y = self
            .v0()
            .y()
            .min(self.v1().y())
            .min(self.v2().y())
            .min(self.v3().y());
        Vec2::new(min_x, min_y)
    }

    pub fn max(&self) -> Vec2 {
        let max_x = self
            .v0()
            .x()
            .max(self.v1().x())
            .max(self.v2().x())
            .max(self.v3().x());
        let max_y = self
            .v0()
            .y()
            .max(self.v1().y())
            .max(self.v2().y())
            .max(self.v3().y());
        Vec2::new(max_x, max_y)
    }

    pub fn get_support(&self, dir: Vec2) -> Vec2 {
        let mut best_projection = f32::MIN;
        let mut best_vertex = Vec2::zero();

        for i in 0..4 {
            let v = self.vertices[i];
            let proj = v.dot(dir);

            if proj > best_projection {
                best_vertex = v;
                best_projection = proj;
            }
        }

        best_vertex
    }
}

impl Collider for Obb {
    type Point = Vec2;

    fn bounding_box(&self) -> BoundingBox<Self::Point> {
        BoundingBox::new(self.min(), self.max())
    }

    fn status(&self) -> Status {
        self.status
    }
}

/// The two dimensional size of a `Shape`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Property)]
pub struct Size2 {
    pub width: f32,
    pub height: f32,
}

impl Size2 {
    /// Returns a new 2d size.
    pub fn new(width: f32, height: f32) -> Self {
        Self { width, height }
    }
}

/// The shape of a rigid body.
///
/// Contains a rotation/translation offset and a size.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Properties)]
pub struct Shape {
    offset: Vec2,
    size: Size2,
}

impl Shape {
    /// Return a new `Shape` with a zero offset and a size.
    pub fn new(size: Size2) -> Self {
        let offset = Vec2::zero();
        Self { offset, size }
    }

    /// Return a new `Shape` with an offset and a size.
    pub fn with_offset(mut self, offset: Vec2) -> Self {
        self.offset = offset;
        self
    }
}

impl From<Size2> for Shape {
    fn from(size: Size2) -> Self {
        let x = size.width * 0.5;
        let y = size.height * 0.5;
        let offset = Vec2::new(-x, -y);
        Self { offset, size }
    }
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnerJoint {
    body1: Entity,
    body2: Entity,
    offset: Vec2,
    angle: f32,
}

impl InnerJoint {
    pub fn new(body1: Entity, body2: Entity) -> Self {
        Self {
            body1,
            body2,
            offset: Vec2::zero(),
            angle: 0.0,
        }
    }

    pub fn with_offset(mut self, offset: Vec2) -> Self {
        self.offset = offset;
        self
    }

    pub fn with_angle(mut self, angle: f32) -> Self {
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
        _offset: Vec2,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec2> {
        None
    }

    /// Returns a new rotation for target based on `self` and `anchor`.
    fn rotation(&mut self, _angle: f32, _anchor: &RigidBody, _target: &RigidBody) -> Option<f32> {
        None
    }

    /// Returns a new linear velocity for target based on `self` and `anchor`.
    fn linear_velocity(
        &mut self,
        _offset: Vec2,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec2> {
        None
    }

    /// Returns a new angular velocity for target based on `self` and `anchor`.
    fn angular_velocity(
        &mut self,
        _angle: f32,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<f32> {
        None
    }

    /// Returns a linear impulse to apply to target based on `self` and `anchor`.
    fn linear_impulse(
        &mut self,
        _offset: Vec2,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec2> {
        None
    }

    /// Returns an angular impulse to apply to target based on `self` and `anchor`.
    fn angular_impulse(
        &mut self,
        _angle: f32,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<f32> {
        None
    }
}

/// A joint behaviour that causes the anchored body to be rigidly fixed at an offset and an angle.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct FixedJointBehaviour;

impl JointBehaviour for FixedJointBehaviour {
    const STAGE: &'static str = stage::RIGID_JOINT;

    fn position(&mut self, offset: Vec2, anchor: &RigidBody, _target: &RigidBody) -> Option<Vec2> {
        Some(anchor.position + offset)
    }

    fn rotation(&mut self, angle: f32, anchor: &RigidBody, _target: &RigidBody) -> Option<f32> {
        Some(anchor.rotation + angle)
    }
}

/// A joint behaviour that causes the anchored body to be accurately positioned with an offset and an angle.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct MechanicalJointBehaviour;

impl JointBehaviour for MechanicalJointBehaviour {
    const STAGE: &'static str = stage::COLLIDING_JOINT;

    fn position(&mut self, offset: Vec2, anchor: &RigidBody, _target: &RigidBody) -> Option<Vec2> {
        Some(anchor.position + offset)
    }

    fn rotation(&mut self, angle: f32, anchor: &RigidBody, _target: &RigidBody) -> Option<f32> {
        Some(anchor.rotation + angle)
    }

    fn linear_velocity(
        &mut self,
        _offset: Vec2,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec2> {
        Some(Vec2::zero())
    }

    fn angular_velocity(
        &mut self,
        _angle: f32,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<f32> {
        Some(0.0)
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
        _offset: Vec2,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<Vec2> {
        Some(Vec2::zero())
    }

    fn angular_velocity(
        &mut self,
        _angle: f32,
        _anchor: &RigidBody,
        _target: &RigidBody,
    ) -> Option<f32> {
        Some(0.0)
    }

    fn linear_impulse(
        &mut self,
        offset: Vec2,
        anchor: &RigidBody,
        target: &RigidBody,
    ) -> Option<Vec2> {
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
        angle: f32,
        anchor: &RigidBody,
        target: &RigidBody,
    ) -> Option<f32> {
        // the minimum time to "jump" into position
        const EPSILON: f32 = 0.1;
        // the maximum time to "jump" into position
        const T: f32 = 1.0;
        let springiness = 1.0 - self.rigidness;
        let rotation = anchor.rotation + angle;
        let d = rotation - target.rotation;
        let scale = (T - EPSILON) * springiness + EPSILON;
        let impulse = d * target.mass / scale;
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
        self.behaviour.rigidness = rigidness;
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
    pub fn with_offset(self, offset: Vec2) -> Self {
        Self {
            inner: self.inner.with_offset(offset),
            behaviour: self.behaviour,
        }
    }

    /// Add an angle to an owned `Joint`.
    pub fn with_angle(self, angle: f32) -> Self {
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
    pub position: Vec2,
    lowest_position: Vec2,
    /// Current rotation of this rigid body.
    ///
    /// NOTE: collisions checks may or may not be broken if this is not a multiple of 90 degrees.
    pub rotation: f32,
    /// Current linear velocity of this rigid body.
    pub linvel: Vec2,
    prev_linvel: Vec2,
    /// The terminal linear velocity of a semikinematic body.
    ///
    /// Defaults to `f32::INFINITY`.
    pub terminal: Vec2,
    accumulator: Vec2,
    dynamic_acc: Vec2,
    /// Current angular velocity of this rigid body.
    pub angvel: f32,
    prev_angvel: f32,
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
            position: Vec2::zero(),
            lowest_position: Vec2::zero(),
            rotation: 0.0,
            linvel: Vec2::zero(),
            prev_linvel: Vec2::zero(),
            terminal: Vec2::new(f32::INFINITY, f32::INFINITY),
            accumulator: Vec2::zero(),
            dynamic_acc: Vec2::zero(),
            angvel: 0.0,
            prev_angvel: 0.0,
            ang_term: f32::INFINITY,
            status: Status::Semikinematic,
            mass: mass.scalar(),
            inv_mass: mass.inverse(),
            active: true,
            sensor: false,
        }
    }

    /// Returns a `RigidBody` identical to this one, but with the position set to a new one.
    pub fn with_position(mut self, position: Vec2) -> Self {
        self.position = position;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the rotation set to a new one.
    pub fn with_rotation(mut self, rotation: f32) -> Self {
        self.rotation = rotation;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the linear velocity set to a new one.
    pub fn with_linear_velocity(mut self, linvel: Vec2) -> Self {
        self.linvel = linvel;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the linear velocity set to a new one.
    pub fn with_angular_velocity(mut self, angvel: f32) -> Self {
        self.angvel = angvel;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the terminal linear velocity set to a new one.
    pub fn with_terminal(mut self, terminal: Vec2) -> Self {
        self.terminal = terminal;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the terminal linear velocity set to a new one.
    pub fn with_angular_terminal(mut self, terminal: f32) -> Self {
        self.ang_term = terminal;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the acceleration set to a new one.
    pub fn with_acceleration(mut self, acceleration: Vec2) -> Self {
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
    pub fn apply_linear_impulse(&mut self, impulse: Vec2) {
        self.linvel += impulse * self.inv_mass;
    }

    /// Applies an impulse to the `RigidBody`s linear velocity.
    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        self.angvel += impulse * self.inv_mass;
    }

    /// Applies a force to the `RigidBody`s acceleration accumulator.
    pub fn apply_force(&mut self, force: Vec2) {
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

    /// Returns the difference between the last known linear velocity and the current linear velocity.
    pub fn linear_deceleration(&self) -> Vec2 {
        self.prev_linvel.abs() - self.linvel.abs()
    }

    /// Returns the difference between the last known angular velocity and the current angular velocity.
    pub fn angular_deceleration(&self) -> f32 {
        self.prev_angvel.abs() - self.angvel.abs()
    }
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
    /// The penetration, relative to the second entity.
    pub penetration: f32,
    /// The normal, relative to the second entity.
    pub normal: Vec2,
    /// The contact points of this manifold.
    pub contacts: SmallVec<[Vec2; 4]>,
}

pub fn broad_phase_system(
    commands: &mut Commands,
    query: Query<(Entity, &RigidBody, &Children)>,
    query2: Query<&Shape>,
) {
    let mut colliders = Vec::new();
    for (entity, body, children) in &mut query.iter() {
        for &e in children.iter() {
            if let Ok(shape) = query2.get_component::<Shape>(e) {
                let v0 = shape.offset;
                let v1 = shape.offset + Vec2::new(shape.size.width, 0.0);
                let v2 = shape.offset + Vec2::new(shape.size.width, shape.size.height);
                let v3 = shape.offset + Vec2::new(0.0, shape.size.height);
                let rotation = Mat2::from_angle(body.rotation);
                let position = body.position;
                let n0 = Vec2::new(1.0, 0.0);
                let n1 = Vec2::new(0.0, 1.0);
                let collider = Obb::new(
                    body.status,
                    entity,
                    rotation,
                    position,
                    v0,
                    v1,
                    v2,
                    v3,
                    n0,
                    n1,
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

fn bias_greater_than(a: f32, b: f32) -> bool {
    const BIAS_RELATIVE: f32 = 0.95;
    const BIAS_ABSOLUTE: f32 = 0.01;
    a >= b * BIAS_RELATIVE + a * BIAS_ABSOLUTE
}

fn find_axis_of_least_penetration(a: &Obb, b: &Obb) -> (f32, usize) {
    let mut best_distance = f32::MIN;
    let mut best_index = 0;

    for i in 0..4 {
        let n = a.normals[i];
        let nw = a.rotation * n;

        let brt = b.rotation.transpose();
        let n = brt * nw;

        let s = b.get_support(-n);

        let mut v = a.vertices[i];
        v = a.rotation * v + a.position;
        v -= b.position;
        v = brt * v;

        let d = n.dot(s - v);

        if d > best_distance {
            best_distance = d;
            best_index = i;
        }
    }

    (best_distance, best_index)
}

fn find_incident_face(ref_poly: &Obb, inc_poly: &Obb, idx: usize) -> [Vec2; 2] {
    let mut ref_normal = ref_poly.normals[idx];

    ref_normal = ref_poly.rotation * ref_normal;
    ref_normal = inc_poly.rotation.transpose() * ref_normal;

    let mut incident_face = 0;
    let mut min_dot = f32::MAX;

    for i in 0..4 {
        let dot = ref_normal.dot(inc_poly.normals[i]);
        if dot < min_dot {
            min_dot = dot;
            incident_face = i;
        }
    }

    let v0 = inc_poly.rotation * inc_poly.vertices[incident_face] + inc_poly.position;
    incident_face = (incident_face + 1) & 0x3;
    let v1 = inc_poly.rotation * inc_poly.vertices[incident_face] + inc_poly.position;
    [v0, v1]
}

fn clip(n: Vec2, c: f32, face: &mut [Vec2]) -> usize {
    let mut sp = 0;
    let mut out = [face[0], face[1]];

    let d1 = n.dot(face[0]) - c;
    let d2 = n.dot(face[1]) - c;

    if d1 <= 0.0 {
        out[sp] = face[0];
        sp += 1;
    }

    if d2 <= 0.0 {
        out[sp] = face[1];
        sp += 1;
    }

    if d1 * d2 < 0.0 {
        let alpha = d1 / (d1 - d2);
        out[sp] = face[0] + alpha * (face[1] - face[0]);
        sp += 1;
    }

    face[0] = out[0];
    face[1] = out[1];

    debug_assert_ne!(sp, 3);

    sp
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

        let (penetration_a, face_a) = find_axis_of_least_penetration(&collider1, &collider2);
        if penetration_a >= 0.0 {
            continue;
        }

        let (penetration_b, face_b) = find_axis_of_least_penetration(&collider2, &collider1);
        if penetration_b >= 0.0 {
            continue;
        }

        let mut ref_index;
        let flip;
        let ref_poly;
        let inc_poly;

        if bias_greater_than(penetration_a, penetration_b) {
            ref_poly = &collider1;
            inc_poly = &collider2;
            ref_index = face_a;
            flip = false;
        } else {
            ref_poly = &collider2;
            inc_poly = &collider1;
            ref_index = face_b;
            flip = true;
        }

        let mut incident_face = find_incident_face(ref_poly, inc_poly, ref_index);

        let mut v1 = ref_poly.vertices[ref_index];
        ref_index = (ref_index + 1) & 0x3;
        let mut v2 = ref_poly.vertices[ref_index];

        v1 = ref_poly.rotation * v1 + ref_poly.position;
        v2 = ref_poly.rotation * v2 + ref_poly.position;

        let side_plane_normal = (v2 - v1).normalize();

        let ref_face_normal = Vec2::new(side_plane_normal.y(), -side_plane_normal.x());

        let refc = ref_face_normal.dot(v1);
        let negside = -side_plane_normal.dot(v1);
        let posside = side_plane_normal.dot(v2);

        if clip(-side_plane_normal, negside, &mut incident_face) < 2 {
            continue;
        }

        if clip(side_plane_normal, posside, &mut incident_face) < 2 {
            continue;
        }

        let normal = if flip {
            -ref_face_normal
        } else {
            ref_face_normal
        };
        let mut penetration = 0.0;

        let mut contacts = SmallVec::new();

        let mut cp = 0;
        let sep = ref_face_normal.dot(incident_face[0]) - refc;
        if sep <= 0.0 {
            contacts.push(incident_face[0]);
            penetration = -sep;
            cp += 1;
        }

        let sep = ref_face_normal.dot(incident_face[1]) - refc;
        if sep <= 0.0 {
            contacts.push(incident_face[1]);
            penetration += -sep;
            cp += 1;
            penetration /= cp as f32;
        }

        let manifold = Manifold {
            body1: collider1.body,
            body2: collider2.body,
            penetration,
            normal,
            contacts,
        };
        manifolds.send(manifold);
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
    step: Res<GlobalStep>,
    up: Res<GlobalUp>,
    ang_tol: Res<AngularTolerance>,
    mut query: Query<Mut<RigidBody>>,
) {
    let delta_time = time.delta.as_secs_f32();

    for manifold in solver.reader.iter(&manifolds) {
        let a = query.get_component::<RigidBody>(manifold.body1).unwrap();
        let b = query.get_component::<RigidBody>(manifold.body2).unwrap();

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

        let mut a = query
            .get_component_mut::<RigidBody>(manifold.body1)
            .unwrap();
        match a.status {
            Status::Static => {}
            Status::Semikinematic => {
                if let Some((impulse, _)) = dynamics {
                    a.dynamic_acc += impulse;

                    let d = -manifold.normal * manifold.penetration;
                    let v = a.linvel * delta_time;
                    if v.signum() == d.signum() {
                        // nothing
                    } else {
                        a.linvel *= Vec2::new(manifold.normal.y().abs(), manifold.normal.x().abs());
                        a.position += d;
                    }
                } else {
                    let mut solve = true;
                    let step_angle = up.0.abs().dot(manifold.normal.abs()).acos();
                    if step_angle > ang_tol.0 {
                        let up_vector = up.0;
                        if up_vector.length_squared() != 0.0 {
                            for &point in &manifold.contacts {
                                let d = point - a.lowest_position;
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
                        let d = -manifold.normal * manifold.penetration;
                        let v = a.linvel * delta_time;
                        if v.signum() == d.signum() {
                            // nothing
                        } else {
                            a.linvel *=
                                Vec2::new(manifold.normal.y().abs(), manifold.normal.x().abs());
                            a.position += d;
                        }
                    }
                }
            }
        }
        mem::drop(a);

        let mut b = query
            .get_component_mut::<RigidBody>(manifold.body2)
            .unwrap();
        match b.status {
            Status::Static => {}
            Status::Semikinematic => {
                if let Some((_, impulse)) = dynamics {
                    b.dynamic_acc += impulse;

                    let d = manifold.normal * manifold.penetration;
                    let v = b.linvel * delta_time;
                    if v.signum() == d.signum() {
                        // nothing
                    } else {
                        b.linvel *= Vec2::new(manifold.normal.y().abs(), manifold.normal.x().abs());
                        b.position += d;
                    }
                } else {
                    let mut solve = true;
                    let step_angle = up.0.abs().dot(manifold.normal.abs()).acos();
                    if step_angle > ang_tol.0 {
                        let up_vector = up.0;
                        if up_vector.length_squared() != 0.0 {
                            for &point in &manifold.contacts {
                                let d = point - b.lowest_position;
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
                        let d = manifold.normal * manifold.penetration;
                        let v = b.linvel * delta_time;
                        if v.signum() == d.signum() {
                            // nothing
                        } else {
                            b.linvel *=
                                Vec2::new(manifold.normal.y().abs(), manifold.normal.x().abs());
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
    up: Res<GlobalUp>,
    mut query: Query<(Mut<RigidBody>, &Children)>,
    shapes: Query<&Shape>,
) {
    if state.skip > 0 {
        state.skip -= 1;
        return;
    }

    let delta_time = time.delta.as_secs_f32();

    for (mut body, children) in query.iter_mut() {
        if !body.active {
            continue;
        }

        if !matches!(body.status, Status::Static) {
            body.accumulator += gravity.0;
        }

        let linvel = body.linvel + body.accumulator * delta_time;
        let linvel = linvel + body.dynamic_acc;
        body.linvel = linvel;
        body.accumulator = Vec2::zero();
        body.dynamic_acc = Vec2::zero();

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
            let vel = body.angvel;
            let limit = body.ang_term;
            match vel.partial_cmp(&0.0) {
                Some(Ordering::Less) => body.angvel = vel.max(-limit),
                Some(Ordering::Greater) => body.angvel = vel.min(limit),
                Some(Ordering::Equal) => {}
                None => body.angvel = 0.0,
            }
        }

        let position = body.position + body.linvel * delta_time;
        body.position = position;

        let rotation = body.rotation + body.angvel * delta_time;
        body.rotation = rotation;

        match body.status {
            Status::Semikinematic => {
                if body.linvel.x().abs() <= body.prev_linvel.x().abs() {
                    *body.linvel.x_mut() *= friction.0
                }
                if body.linvel.y().abs() <= body.prev_linvel.y().abs() {
                    *body.linvel.y_mut() *= friction.0
                }
                if body.angvel.abs() <= body.prev_angvel.abs() {
                    body.angvel *= friction.0
                }
            }
            Status::Static => {}
        }
        body.prev_linvel = body.linvel;
        body.prev_angvel = body.angvel;

        for &child in children.iter() {
            if let Ok(shape) = shapes.get_component::<Shape>(child) {
                let v0 = shape.offset;
                let v1 = shape.offset + Vec2::new(shape.size.width, 0.0);
                let v2 = shape.offset + Vec2::new(shape.size.width, shape.size.height);
                let v3 = shape.offset + Vec2::new(0.0, shape.size.height);
                let rotation = Mat2::from_angle(body.rotation);
                let position = body.position;
                let v0 = rotation * v0;
                let v1 = rotation * v1;
                let v2 = rotation * v2;
                let v3 = rotation * v3;
                let s0 = v0.dot(up.0);
                let s1 = v1.dot(up.0);
                let s2 = v2.dot(up.0);
                let s3 = v3.dot(up.0);
                let v = [v0, v1, v2, v3];
                let s = [s0, s1, s2, s3];
                let min = s0.min(s1).min(s2).min(s3);
                let mut lowest_point = Vec2::zero();
                let mut count = 0;
                for (&v, &s) in v.iter().zip(&s) {
                    // clippy "gently recommends" doing this
                    if (s - min).abs() < f32::EPSILON {
                        lowest_point += v;
                        count += 1;
                    }
                }
                body.lowest_position = position + lowest_point / count as f32;
            }
        }
    }
}

pub fn joint_system<B: JointBehaviour>(
    commands: &mut Commands,
    mut query: Query<(Entity, Mut<Joint<B>>)>,
    mut bodies: Query<Mut<RigidBody>>,
) {
    for (e, mut joint) in query.iter_mut() {
        let anchor = if let Ok(anchor) = bodies.get_component::<RigidBody>(joint.inner.body1) {
            anchor
        } else {
            commands.despawn_recursive(e);
            continue;
        };
        let target = if let Ok(target) = bodies.get_component::<RigidBody>(joint.inner.body2) {
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

        let mut target = bodies
            .get_component_mut::<RigidBody>(joint.inner.body2)
            .unwrap();

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

/// The plane on which to translate the 2d position into 3d coordinates.
#[derive(Debug, Clone, Copy)]
pub enum TranslationMode {
    AxesXY,
    AxesXZ,
    AxesYZ,
}

impl Default for TranslationMode {
    fn default() -> Self {
        Self::AxesXY
    }
}

/// The axis on which to rotate the 2d rotation into a 3d quaternion.
#[derive(Debug, Clone, Copy)]
pub enum RotationMode {
    AxisX,
    AxisY,
    AxisZ,
}

impl Default for RotationMode {
    fn default() -> Self {
        Self::AxisZ
    }
}

pub fn sync_transform_system(
    translation_mode: Res<TranslationMode>,
    rotation_mode: Res<RotationMode>,
    mut query: Query<(&RigidBody, Mut<Transform>)>,
) {
    for (body, mut transform) in query.iter_mut() {
        match *translation_mode {
            TranslationMode::AxesXY => {
                let x = body.position.x();
                let y = body.position.y();
                let z = 0.0;
                transform.translation = Vec3::new(x, y, z);
            }
            TranslationMode::AxesXZ => {
                let x = body.position.x();
                let y = 0.0;
                let z = body.position.y();
                transform.translation = Vec3::new(x, y, z);
            }
            TranslationMode::AxesYZ => {
                let x = 0.0;
                let y = body.position.x();
                let z = body.position.y();
                transform.translation = Vec3::new(x, y, z);
            }
        }
        match *rotation_mode {
            RotationMode::AxisX => {
                transform.rotation = Quat::from_rotation_x(body.rotation);
            }
            RotationMode::AxisY => {
                transform.rotation = Quat::from_rotation_y(body.rotation);
            }
            RotationMode::AxisZ => {
                transform.rotation = Quat::from_rotation_z(body.rotation);
            }
        }
    }
}
