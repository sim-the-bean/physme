//! This module provides the primitives and systems for 3d physics simulation.
//!
//! For examples, see the root of the crate.

use std::cmp::Ordering;
use std::mem;

use bevy::math::*;
use bevy::prelude::*;
use hashbrown::{HashMap, HashSet};
use smallvec::SmallVec;

use crate::broad::{self, BoundingBox, Collider};
use crate::common::*;

/// This is what you want to add to your `App` if you want to run 3d physics simulation.
pub struct Physics3dPlugin;

pub mod stage {
    #[doc(hidden)]
    pub use bevy::prelude::stage::*;

    pub const PHYSICS_STEP: &str = "physics_step_3d";
    pub const BROAD_PHASE: &str = "broad_phase_3d";
    pub const NARROW_PHASE: &str = "narrow_phase_3d";
    pub const PHYSICS_SOLVE: &str = "physics_solve_3d";
    pub const SYNC_TRANSFORM: &str = "sync_transform_3d";
}

impl Plugin for Physics3dPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(GlobalFriction::default())
            .add_resource(GlobalGravity::default())
            .add_event::<Manifold>()
            .add_stage_before(stage::UPDATE, stage::PHYSICS_STEP)
            .add_stage_after(stage::PHYSICS_STEP, stage::BROAD_PHASE)
            .add_stage_after(stage::BROAD_PHASE, stage::NARROW_PHASE)
            .add_stage_after(stage::NARROW_PHASE, stage::PHYSICS_SOLVE)
            .add_stage_after(stage::PHYSICS_SOLVE, stage::SYNC_TRANSFORM);
        let physics_step = PhysicsStep::default().system(app.resources_mut());
        app.add_system_to_stage(stage::PHYSICS_STEP, physics_step)
            .add_system_to_stage(stage::BROAD_PHASE, broad_phase_system.system())
            .add_system_to_stage(stage::NARROW_PHASE, narrow_phase_system.system());
        let solver = Solver::default().system(app.resources_mut());
        app.add_system_to_stage(stage::PHYSICS_SOLVE, solver)
            .add_system_to_stage(stage::SYNC_TRANSFORM, sync_transform_system.system());
    }
}

pub type BroadPhase = broad::BroadPhase<Aabb>;

/// The global gravity that affects every `RigidBody` with the `Semikinematic` status.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalGravity(pub Vec3);

/// The global step value, affects all semikinematic bodies.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalStep(pub f32);

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    status: Status,
    body: Entity,
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn new(status: Status, body: Entity, min: Vec3, max: Vec3) -> Self {
        Self {
            status,
            body,
            min,
            max,
        }
    }
}

impl Collider for Aabb {
    type Point = Vec3;

    fn bounding_box(&self) -> BoundingBox<Self::Point> {
        BoundingBox::new(self.min, self.max)
    }

    fn status(&self) -> Status {
        self.status
    }
}

/// The three dimensional size of a `Shape`
#[derive(Debug, Clone, Copy, PartialEq)]
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shape {
    offset: Vec3,
    size: Size3,
}

impl Shape {
    /// Return a new `Shape` with a zero offset and a size.
    pub fn new(size: Size3) -> Self {
        let offset = Vec3::zero();
        Self { offset, size }
    }

    /// Return a new `Shape` with an offset and a size.
    pub fn with_offset(mut self, offset: Vec3) -> Self {
        self.offset = offset;
        self
    }
}

impl From<Size3> for Shape {
    fn from(size: Size3) -> Self {
        let x = size.width * 0.5;
        let y = size.height * 0.5;
        let z = size.depth * 0.5;
        let offset = Vec3::new(-x, -y, -z);
        Self { offset, size }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Joint {
    body1: Entity,
    body2: Entity,
    offset: Vec3,
    angle: Quat,
}

impl Joint {
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

#[doc(hidden)]
#[derive(Default, Debug, Clone, Copy)]
pub struct Solved {
    x: bool,
    y: bool,
    z: bool,
}

/// The rigid body.
#[derive(Debug, Clone, Copy)]
pub struct RigidBody {
    /// Current position of this rigid body.
    pub position: Vec3,
    /// Current rotation of this rigid body.
    ///
    /// NOTE: collisions checks may or may not be broken if this is not a multiple of 90 degrees.
    pub rotation: Quat,
    /// Current linear velocity of this rigid body.
    pub velocity: Vec3,
    prev_velocity: Vec3,
    /// The terminal velocity of a semikinematic body.
    ///
    /// Defaults to `f32::INFINITY`.
    pub terminal: Vec3,
    accumulator: Vec3,
    dynamic_acc: Vec3,
    /// The status, i.e. static or semikinematic.
    ///
    /// Affects how forces and collisions affect this rigid body.
    pub status: Status,
    mass: f32,
    inv_mass: f32,
    active: bool,
    sensor: bool,
    solved: Solved,
}

impl RigidBody {
    /// Returns a new `RigidBody` with just a mass and all other components set to their defaults.
    pub fn new(mass: Mass) -> Self {
        Self {
            position: Vec3::zero(),
            rotation: Quat::identity(),
            velocity: Vec3::zero(),
            prev_velocity: Vec3::zero(),
            terminal: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            accumulator: Vec3::zero(),
            dynamic_acc: Vec3::zero(),
            status: Status::Semikinematic,
            mass: mass.scalar(),
            inv_mass: mass.inverse(),
            active: true,
            sensor: false,
            solved: Default::default(),
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

    /// Returns a `RigidBody` identical to this one, but with the velocity set to a new one.
    pub fn with_velocity(mut self, velocity: Vec3) -> Self {
        self.velocity = velocity;
        self
    }

    /// Returns a `RigidBody` identical to this one, but with the terminal velocity set to a new one.
    pub fn with_terminal(mut self, terminal: Vec3) -> Self {
        self.terminal = terminal;
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

    /// Applies an impulse to the `RigidBody`s velocity.
    pub fn apply_impulse(&mut self, impulse: Vec3) {
        self.velocity += impulse * self.inv_mass;
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

/// Represents a single active collision between this and another `RigidBody`.
#[derive(Debug, Clone, Copy)]
pub struct CollisionInfo {
    /// The other entity.
    pub other: Entity,
    /// The penetration, relative to the other entity.
    pub penetration: Vec3,
    /// The normals, relative to the other entity.
    ///
    /// NOTE: This is not a normalized vector, all three its components are always set to either -1 or 1.
    pub normals: Vec3,
}

/// Represents all active collisions of this `RigidBody`.
#[derive(Default, Debug, Clone)]
pub struct Collisions {
    /// Active collisions on the X axis.
    pub x: SmallVec<[CollisionInfo; 16]>,
    /// Active collisions on the Y axis.
    pub y: SmallVec<[CollisionInfo; 16]>,
    /// Active collisions on the Z axis.
    pub z: SmallVec<[CollisionInfo; 16]>,
}

/// Checks whether a `Shape` is contained within another, per axis.
#[derive(Debug, Clone, Copy)]
pub struct Contained {
    /// Is the `Shape` contained within the other along the X axis?
    pub x: bool,
    /// Is the `Shape` contained within the other along the Y axis?
    pub y: bool,
    /// Is the `Shape` contained within the other along the Z axis?
    pub z: bool,
}

/// The manifold, representing detailed data on a collision between two `RigidBody`s.
///
/// Usable as an event.
#[derive(Debug, Clone, Copy)]
pub struct Manifold {
    /// The first entity.
    pub body1: Entity,
    /// The second entity.
    pub body2: Entity,
    /// The penetration, relative to the second entity.
    pub penetration: Vec3,
    /// The normals, relative to the second entity.
    ///
    /// NOTE: This is not a normalized vector, both its components are always set to either -1 or 1.
    pub normals: Vec3,
    /// The normal along which pushing should be performed.
    ///
    /// NOTE: This is always axis aligned.
    pub pnormal: Vec3,
    /// Checks whether the entities are contained within the other, per axis.
    pub contained: [Contained; 2],
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
                let mut min = shape.offset;
                let mut max =
                    shape.offset + Vec3::new(shape.size.width, shape.size.height, shape.size.depth);
                let rotation = body.rotation;
                min = body.position + rotation * min;
                max = body.position + rotation * max;
                let min_x = min.x().min(max.x());
                let min_y = min.y().min(max.y());
                let min_z = min.z().min(max.z());
                let max_x = min.x().max(max.x());
                let max_y = min.y().max(max.y());
                let max_z = min.z().max(max.z());
                let min = Vec3::new(min_x, min_y, min_z);
                let max = Vec3::new(max_x, max_y, max_z);
                let collider = Aabb::new(body.status, entity, min, max);
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

        let b_pos = (collider2.min + collider2.max) * 0.5;
        let a_pos = (collider1.min + collider1.max) * 0.5;
        let d = b_pos - a_pos;

        let a_extent = (collider1.max.x() - collider1.min.x()) * 0.5;
        let b_extent = (collider2.max.x() - collider2.min.x()) * 0.5;
        let x_overlap = a_extent + b_extent - d.x().abs();

        if x_overlap > 0.0 {
            let a_x_contained =
                collider1.min.x() >= collider2.min.x() && collider1.max.x() <= collider2.max.x();
            let b_x_contained =
                collider2.min.x() >= collider1.min.x() && collider2.max.x() <= collider1.max.x();

            let a_extent = (collider1.max.y() - collider1.min.y()) * 0.5;
            let b_extent = (collider2.max.y() - collider2.min.y()) * 0.5;
            let y_overlap = a_extent + b_extent - d.y().abs();

            if y_overlap > 0.0 {
                let a_y_contained = collider1.min.y() >= collider2.min.y()
                    && collider1.max.y() <= collider2.max.y();
                let b_y_contained = collider2.min.y() >= collider1.min.y()
                    && collider2.max.y() <= collider1.max.y();

                let a_extent = (collider1.max.z() - collider1.min.z()) * 0.5;
                let b_extent = (collider2.max.z() - collider2.min.z()) * 0.5;
                let z_overlap = a_extent + b_extent - d.z().abs();

                if z_overlap > 0.0 {
                    let a_z_contained = collider1.min.z() >= collider2.min.z()
                        && collider1.max.z() <= collider2.max.z();
                    let b_z_contained = collider2.min.z() >= collider1.min.z()
                        && collider2.max.z() <= collider1.max.z();

                    let pn_x1 = (collider2.max.x() - collider1.min.x()).abs();
                    let pn_x2 = (collider2.min.x() - collider1.max.x()).abs();
                    let pn_y1 = (collider2.max.y() - collider1.min.y()).abs();
                    let pn_y2 = (collider2.min.y() - collider1.max.y()).abs();
                    let pn_z1 = (collider2.max.z() - collider1.min.z()).abs();
                    let pn_z2 = (collider2.min.z() - collider1.max.z()).abs();
                    let min = pn_x1.min(pn_x2).min(pn_y1).min(pn_y2).min(pn_z1).min(pn_z2);
                    let pnormal = if min == pn_x1 {
                        Vec3::new(1.0, 0.0, 0.0)
                    } else if min == pn_x2 {
                        Vec3::new(-1.0, 0.0, 0.0)
                    } else if min == pn_y1 {
                        Vec3::new(0.0, 1.0, 0.0)
                    } else if min == pn_y2 {
                        Vec3::new(0.0, -1.0, 0.0)
                    } else if min == pn_z1 {
                        Vec3::new(0.0, 0.0, 1.0)
                    } else {
                        Vec3::new(0.0, 0.0, -1.0)
                    };

                    let n_x = if d.x() < 0.0 { -1.0 } else { 1.0 };
                    let n_y = if d.y() < 0.0 { -1.0 } else { 1.0 };
                    let n_z = if d.z() < 0.0 { -1.0 } else { 1.0 };
                    let manifold = Manifold {
                        body1: collider1.body,
                        body2: collider2.body,
                        penetration: Vec3::new(x_overlap, y_overlap, z_overlap),
                        normals: Vec3::new(n_x, n_y, n_z),
                        pnormal,
                        contained: [
                            Contained {
                                x: a_x_contained,
                                y: a_y_contained,
                                z: a_z_contained,
                            },
                            Contained {
                                x: b_x_contained,
                                y: b_y_contained,
                                z: b_z_contained,
                            },
                        ],
                    };
                    manifolds.send(manifold);
                }
            }
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
    mut commands: Commands,
    mut solver: Local<Solver>,
    time: Res<Time>,
    manifolds: Res<Events<Manifold>>,
    step: Res<GlobalStep>,
    mut query: Query<(Mut<RigidBody>, Option<Mut<Collisions>>)>,
) {
    let delta_time = time.delta.as_secs_f32();

    let mut transaction: HashMap<Entity, Collisions> = HashMap::new();

    for manifold in solver.reader.iter(&manifolds) {
        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        if a.sensor || b.sensor {
            continue;
        }

        let mut a_maybe_step = None;
        let mut b_maybe_step = None;
        let dont_x = a.status == Status::Semikinematic
            && b.status == Status::Semikinematic
            && manifold.pnormal.x() == 0.0;
        let mut pushed_x = false;

        if !dont_x {
            let dynamics = if a.status == Status::Semikinematic
                && b.status == Status::Semikinematic
                && manifold.pnormal.x() != 0.0
            {
                let sum_recip = (a.mass + b.mass).recip();
                let br = b.velocity.x() * b.mass;
                let ar = a.velocity.x() * a.mass;
                let rv = br * sum_recip - ar * sum_recip;

                let impulse = -rv;

                let a = a.velocity.x() - impulse;
                let b = b.velocity.x() + impulse;
                Some((a, b))
            } else {
                None
            };
            mem::drop(a);
            mem::drop(b);

            pushed_x = dynamics.is_some();

            let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
            match a.status {
                Status::Static => {}
                Status::Semikinematic => {
                    if let Some((impulse, _)) = dynamics {
                        *a.dynamic_acc.x_mut() += impulse;

                        if !manifold.contained[0].x {
                            let d = -manifold.normals.x() * manifold.penetration.x();
                            let v = a.velocity.x() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                a.solved.x = true;
                                *a.velocity.x_mut() = 0.0;
                                *a.position.x_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body1) {
                            for info in &collisions.x {
                                if info.other == manifold.body2 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[0].x {
                                let d = -manifold.normals.x() * manifold.penetration.x();
                                let v = a.velocity.x() * delta_time;
                                if v.signum() == d.signum() {
                                    // nothing
                                } else {
                                    a_maybe_step = Some((
                                        Vec3::new(-d, 0.0, 0.0),
                                        Vec3::new(a.velocity.x(), 0.0, 0.0),
                                    ));
                                    a.solved.x = true;
                                    *a.velocity.x_mut() = 0.0;
                                    *a.position.x_mut() += d;
                                }
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
                    if let Some((_, impulse)) = dynamics {
                        *b.dynamic_acc.x_mut() += impulse;

                        if !manifold.contained[1].x {
                            let d = manifold.normals.x() * manifold.penetration.x();
                            let v = b.velocity.x() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                b.solved.x = true;
                                *b.velocity.x_mut() = 0.0;
                                *b.position.x_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body2) {
                            for info in &collisions.x {
                                if info.other == manifold.body1 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[1].x {
                                let d = manifold.normals.x() * manifold.penetration.x();
                                let v = b.velocity.x() * delta_time;
                                if v.signum() == d.signum() {
                                    // nothing
                                } else {
                                    b_maybe_step = Some((
                                        Vec3::new(-d, 0.0, 0.0),
                                        Vec3::new(b.velocity.x(), 0.0, 0.0),
                                    ));
                                    b.solved.x = true;
                                    *b.velocity.x_mut() = 0.0;
                                    *b.position.x_mut() += d;
                                }
                            }
                        }
                    }
                }
            }
            mem::drop(b);
        } else {
            mem::drop(a);
            mem::drop(b);
        }

        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        let dont_z = a.status == Status::Semikinematic
            && b.status == Status::Semikinematic
            && manifold.pnormal.z() == 0.0;
        let mut pushed_z = false;

        if !dont_z {
            let dynamics = if a.status == Status::Semikinematic
                && b.status == Status::Semikinematic
                && manifold.pnormal.z() != 0.0
            {
                let sum_recip = (a.mass + b.mass).recip();
                let br = b.velocity.z() * b.mass;
                let ar = a.velocity.z() * a.mass;
                let rv = br * sum_recip - ar * sum_recip;

                let impulse = -rv;

                let a = a.velocity.z() - impulse;
                let b = b.velocity.z() + impulse;
                Some((a, b))
            } else {
                None
            };
            mem::drop(a);
            mem::drop(b);

            pushed_z = dynamics.is_some();

            let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
            match a.status {
                Status::Static => {}
                Status::Semikinematic => {
                    if let Some((impulse, _)) = dynamics {
                        *a.dynamic_acc.z_mut() += impulse;

                        if !manifold.contained[0].z {
                            let d = -manifold.normals.z() * manifold.penetration.z();
                            let v = a.velocity.z() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                a.solved.z = true;
                                *a.velocity.z_mut() = 0.0;
                                *a.position.z_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body1) {
                            for info in &collisions.z {
                                if info.other == manifold.body2 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[0].z {
                                let d = -manifold.normals.z() * manifold.penetration.z();
                                let v = a.velocity.z() * delta_time;
                                if v.signum() == d.signum() {
                                    // nothing
                                } else {
                                    a_maybe_step = Some((
                                        Vec3::new(0.0, 0.0, -d),
                                        Vec3::new(0.0, 0.0, a.velocity.z()),
                                    ));
                                    a.solved.z = true;
                                    *a.velocity.z_mut() = 0.0;
                                    *a.position.z_mut() += d;
                                }
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
                    if let Some((_, impulse)) = dynamics {
                        *b.dynamic_acc.z_mut() += impulse;

                        if !manifold.contained[1].z {
                            let d = manifold.normals.z() * manifold.penetration.z();
                            let v = b.velocity.z() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                b.solved.z = true;
                                *b.velocity.z_mut() = 0.0;
                                *b.position.z_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body2) {
                            for info in &collisions.z {
                                if info.other == manifold.body1 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[1].z {
                                let d = manifold.normals.z() * manifold.penetration.z();
                                let v = b.velocity.z() * delta_time;
                                if v.signum() == d.signum() {
                                    // nothing
                                } else {
                                    b_maybe_step = Some((
                                        Vec3::new(0.0, 0.0, -d),
                                        Vec3::new(0.0, 0.0, b.velocity.z()),
                                    ));
                                    b.solved.z = true;
                                    *b.velocity.z_mut() = 0.0;
                                    *b.position.z_mut() += d;
                                }
                            }
                        }
                    }
                }
            }
            mem::drop(b);
        } else {
            mem::drop(a);
            mem::drop(b);
        }

        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        if !pushed_x && !pushed_z {
            let dynamics = a.status == Status::Semikinematic
                && b.status == Status::Semikinematic
                && manifold.pnormal.y() != 0.0;
            mem::drop(a);
            mem::drop(b);

            let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
            match a.status {
                Status::Static => {}
                Status::Semikinematic => {
                    if dynamics {
                        if !manifold.contained[0].y {
                            let d = -manifold.normals.y() * manifold.penetration.y();
                            let v = a.velocity.y();
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                a.solved.y = true;
                                *a.velocity.y_mut() = 0.0;
                                *a.position.y_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body1) {
                            for info in &collisions.y {
                                if info.other == manifold.body2 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[0].y {
                                let d = -manifold.normals.y() * manifold.penetration.y();
                                if a_maybe_step.is_some() && d > 0.0 && d <= step.0 {
                                    let (dp, dv) = a_maybe_step.unwrap();
                                    *a.position.y_mut() += d;
                                    a.position += dp;
                                    a.velocity = dv;
                                    a.solved.x = false;
                                    a.solved.y = true;
                                    a.solved.z = false;
                                } else {
                                    let v = a.velocity.y();
                                    if v.signum() == d.signum() {
                                        // nothing
                                    } else {
                                        a.solved.y = true;
                                        *a.velocity.y_mut() = 0.0;
                                        *a.position.y_mut() += d;
                                    }
                                }
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
                    if dynamics {
                        if !manifold.contained[1].y {
                            let d = manifold.normals.y() * manifold.penetration.y();
                            let v = b.velocity.y();
                            if v.signum() == d.signum() {
                                // nothing
                            } else {
                                b.solved.y = true;
                                *b.velocity.y_mut() = 0.0;
                                *b.position.y_mut() += d;
                            }
                        }
                    } else {
                        let mut solve = true;
                        if let Ok(collisions) = query.get::<Collisions>(manifold.body2) {
                            for info in &collisions.y {
                                if info.other == manifold.body1 {
                                    solve = false;
                                    break;
                                }
                            }
                        }
                        if solve {
                            if !manifold.contained[1].y {
                                let d = manifold.normals.y() * manifold.penetration.y();
                                if b_maybe_step.is_some() && d > 0.0 && d <= step.0 {
                                    let (dp, dv) = b_maybe_step.unwrap();
                                    *b.position.y_mut() += d;
                                    b.position += dp;
                                    b.velocity = dv;
                                    b.solved.x = false;
                                    b.solved.y = true;
                                    b.solved.z = false;
                                } else {
                                    let v = b.velocity.y();
                                    if v.signum() == d.signum() {
                                        // nothing
                                    } else {
                                        b.solved.y = true;
                                        *b.velocity.y_mut() = 0.0;
                                        *b.position.y_mut() += d;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            mem::drop(b);
        }

        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        if !a.solved.x {
            transaction
                .entry(manifold.body1)
                .or_default()
                .x
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: manifold.penetration,
                    normals: manifold.normals,
                });
        }
        if !a.solved.y {
            transaction
                .entry(manifold.body1)
                .or_default()
                .y
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: manifold.penetration,
                    normals: manifold.normals,
                });
        }
        if !a.solved.z {
            transaction
                .entry(manifold.body1)
                .or_default()
                .z
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: manifold.penetration,
                    normals: manifold.normals,
                });
        }

        if !b.solved.x {
            transaction
                .entry(manifold.body2)
                .or_default()
                .x
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: -manifold.penetration,
                    normals: -manifold.normals,
                });
        }

        if !b.solved.y {
            transaction
                .entry(manifold.body2)
                .or_default()
                .y
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: -manifold.penetration,
                    normals: -manifold.normals,
                });
        }

        if !b.solved.z {
            transaction
                .entry(manifold.body2)
                .or_default()
                .z
                .push(CollisionInfo {
                    other: manifold.body1,
                    penetration: -manifold.penetration,
                    normals: -manifold.normals,
                });
        }
    }

    for (mut body, collisions) in &mut query.iter() {
        body.solved.x = false;
        body.solved.y = false;
        body.solved.z = false;
        if let Some(mut collisions) = collisions {
            collisions.x.clear();
            collisions.y.clear();
            collisions.z.clear();
        }
    }

    for (e, info) in transaction {
        commands.insert_one(e, info);
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
    mut query: Query<Mut<RigidBody>>,
) {
    if state.skip > 0 {
        state.skip -= 1;
        return;
    }

    let delta_time = time.delta.as_secs_f32();

    for mut body in &mut query.iter() {
        if !body.active {
            continue;
        }

        if !matches!(body.status, Status::Static) {
            body.accumulator += gravity.0;
        }

        let velocity = body.velocity + body.accumulator * delta_time;
        let velocity = velocity + body.dynamic_acc;
        body.velocity = velocity;
        body.accumulator = Vec3::zero();
        body.dynamic_acc = Vec3::zero();

        if matches!(body.status, Status::Semikinematic) {
            let vel = body.velocity;
            let limit = body.terminal;
            match vel.x().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.velocity.x_mut() = vel.x().max(-limit.x()),
                Some(Ordering::Greater) => *body.velocity.x_mut() = vel.x().min(limit.x()),
                Some(Ordering::Equal) => {}
                None => *body.velocity.x_mut() = 0.0,
            }
            match vel.y().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.velocity.y_mut() = vel.y().max(-limit.y()),
                Some(Ordering::Greater) => *body.velocity.y_mut() = vel.y().min(limit.y()),
                Some(Ordering::Equal) => {}
                None => *body.velocity.y_mut() = 0.0,
            }
            match vel.z().partial_cmp(&0.0) {
                Some(Ordering::Less) => *body.velocity.z_mut() = vel.z().max(-limit.z()),
                Some(Ordering::Greater) => *body.velocity.z_mut() = vel.z().min(limit.z()),
                Some(Ordering::Equal) => {}
                None => *body.velocity.z_mut() = 0.0,
            }
        }

        let position = body.position + body.velocity * delta_time;
        body.position = position;

        match body.status {
            Status::Semikinematic => {
                if body.velocity.x().abs() <= body.prev_velocity.x().abs() {
                    *body.velocity.x_mut() *= friction.0
                }
                if body.velocity.y().abs() <= body.prev_velocity.y().abs() {
                    *body.velocity.y_mut() *= friction.0
                }
                if body.velocity.z().abs() <= body.prev_velocity.z().abs() {
                    *body.velocity.z_mut() *= friction.0
                }
            }
            Status::Static => {}
        }
        body.prev_velocity = body.velocity;
    }
}

pub fn sync_transform_system(mut query: Query<(&RigidBody, Mut<Transform>)>) {
    for (body, mut transform) in &mut query.iter() {
        transform.set_translation(body.position);
        transform.set_rotation(body.rotation);
    }
}
