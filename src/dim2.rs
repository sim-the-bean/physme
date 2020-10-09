use std::cmp::Ordering;
use std::mem;

use bevy::math::*;
use bevy::prelude::*;
use bevy::render::{
    mesh::*,
    pipeline::{
        BlendDescriptor, BlendFactor, BlendOperation, ColorStateDescriptor, ColorWrite,
        CompareFunction, CullMode, DepthStencilStateDescriptor, DynamicBinding, FrontFace,
        PipelineDescriptor, PipelineSpecialization, PrimitiveTopology,
        RasterizationStateDescriptor, RenderPipeline, RenderPipelines, StencilStateDescriptor,
        StencilStateFaceDescriptor,
    },
    render_graph::{
        base::{self, MainPass},
        AssetRenderResourcesNode, RenderGraph, RenderResourcesNode,
    },
    renderer::RenderResources,
    shader::{asset_shader_defs_system, Shader, ShaderDefs, ShaderStage, ShaderStages},
    texture::TextureFormat,
};
use hashbrown::{HashMap, HashSet};
use smallvec::SmallVec;

use crate::broad::{self, BoundingBox, Collider};
use crate::common::*;

pub struct Physics2dPlugin;

pub mod stage {
    pub use bevy::prelude::stage::*;

    pub const PHYSICS_STEP: &str = "physics_step";
    pub const BROAD_PHASE: &str = "broad_phase";
    pub const NARROW_PHASE: &str = "narrow_phase";
    pub const PHYSICS_SOLVE: &str = "physics_solve";
    pub const SYNC_TRANSFORM: &str = "sync_transform";
}

impl Plugin for Physics2dPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_resource(GlobalFriction::default())
            .add_resource(GlobalGravity::default())
            .add_resource(TranslationMode::default())
            .add_resource(RotationMode::default())
            .add_resource(StepAxis::default())
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

pub struct DebugRenderPlugin;

impl Plugin for DebugRenderPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app.add_asset::<DebugMaterial>()
            .add_startup_system(init_debug_system.system())
            .add_system(debug_render_system.system())
            .add_system_to_stage(
                stage::POST_UPDATE,
                asset_shader_defs_system::<DebugMaterial>.system(),
            );

        let resources = app.resources();
        let mut render_graph = resources.get_mut::<RenderGraph>().unwrap();
        render_graph.add_debug_graph(resources);
    }
}

pub type BroadPhase = broad::BroadPhase<Aabb>;

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalGravity(pub Vec2);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepAxis {
    X,
    Y,
}

impl Default for StepAxis {
    fn default() -> Self {
        Self::Y
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GlobalStep {
    pub axis: StepAxis,
    pub step: f32,
}

impl GlobalStep {
    pub fn y(step: f32) -> Self {
        Self {
            axis: StepAxis::Y,
            step,
        }
    }
}

impl GlobalStep {
    pub fn x(step: f32) -> Self {
        Self {
            axis: StepAxis::X,
            step,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    body: Entity,
    min: Vec2,
    max: Vec2,
}

impl Aabb {
    fn new(body: Entity, min: Vec2, max: Vec2) -> Self {
        Self { body, min, max }
    }
}

impl Collider for Aabb {
    type Point = Vec2;

    fn bounding_box(&self) -> BoundingBox<Self::Point> {
        BoundingBox::new(self.min, self.max)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shape {
    offset: Vec2,
    size: Size,
}

impl Shape {
    pub fn new(size: Size) -> Self {
        let offset = Vec2::zero();
        Self { offset, size }
    }

    pub fn with_offset(mut self, offset: Vec2) -> Self {
        self.offset = offset;
        self
    }
}

impl From<Size> for Shape {
    fn from(size: Size) -> Self {
        let x = size.width * 0.5;
        let y = size.height * 0.5;
        let offset = Vec2::new(-x, -y);
        Self { offset, size }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Joint {
    body1: Entity,
    body2: Entity,
    offset: Vec2,
    angle: f32,
}

impl Joint {
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

#[derive(Default, Debug, Clone, Copy)]
pub struct Solved {
    x: bool,
    y: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct RigidBody {
    pub position: Vec2,
    pub rotation: f32,
    pub velocity: Vec2,
    prev_velocity: Vec2,
    pub terminal: Vec2,
    accumulator: Vec2,
    pub status: Status,
    inv_mass: f32,
    pub restitution: f32,
    active: bool,
    sensor: bool,
    solved: Solved,
}

impl RigidBody {
    pub fn new(mass: Mass) -> Self {
        Self {
            position: Vec2::zero(),
            rotation: 0.0,
            velocity: Vec2::zero(),
            prev_velocity: Vec2::zero(),
            terminal: Vec2::new(f32::INFINITY, f32::INFINITY),
            accumulator: Vec2::zero(),
            status: Status::Dynamic,
            inv_mass: mass.inverse(),
            restitution: 0.5,
            active: true,
            sensor: false,
            solved: Default::default(),
        }
    }

    pub fn with_position(mut self, position: Vec2) -> Self {
        self.position = position;
        self
    }

    pub fn with_rotation(mut self, rotation: f32) -> Self {
        self.rotation = rotation;
        self
    }

    pub fn with_velocity(mut self, velocity: Vec2) -> Self {
        self.velocity = velocity;
        self
    }

    pub fn with_terminal(mut self, terminal: Vec2) -> Self {
        self.terminal = terminal;
        self
    }

    pub fn with_acceleration(mut self, acceleration: Vec2) -> Self {
        self.accumulator = acceleration;
        self
    }

    pub fn with_status(mut self, status: Status) -> Self {
        self.status = status;
        self
    }

    pub fn with_restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution;
        self
    }

    pub fn with_active(mut self, active: bool) -> Self {
        self.active = active;
        self
    }

    pub fn with_sensor(mut self, sensor: bool) -> Self {
        self.sensor = sensor;
        self
    }

    pub fn apply_impulse(&mut self, impulse: Vec2) {
        self.velocity += impulse * self.inv_mass;
    }

    pub fn apply_force(&mut self, force: Vec2) {
        self.accumulator += force * self.inv_mass;
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CollisionInfo {
    pub other: Entity,
    pub penetration: Vec2,
    pub normals: Vec2,
}

#[derive(Default, Debug, Clone)]
pub struct Collisions {
    pub x: SmallVec<[CollisionInfo; 16]>,
    pub y: SmallVec<[CollisionInfo; 16]>,
}

#[derive(Debug, Clone, Copy)]
pub struct Contained {
    pub x: bool,
    pub y: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct Manifold {
    pub body1: Entity,
    pub body2: Entity,
    pub penetration: Vec2,
    pub normals: Vec2,
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
                let mut max = shape.offset + Vec2::new(shape.size.width, shape.size.height);
                let rotation = Mat2::from_angle(body.rotation);
                min = body.position + rotation * min;
                max = body.position + rotation * max;
                let min_x = min.x().min(max.x());
                let min_y = min.y().min(max.y());
                let max_x = min.x().max(max.x());
                let max_y = min.y().max(max.y());
                let min = Vec2::new(min_x, min_y);
                let max = Vec2::new(max_x, max_y);
                let collider = Aabb::new(entity, min, max);
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
                collider1.min.x() > collider2.min.x() && collider1.max.x() < collider2.max.x();
            let b_x_contained =
                collider2.min.x() > collider1.min.x() && collider2.max.x() < collider1.max.x();

            let a_extent = (collider1.max.y() - collider1.min.y()) * 0.5;
            let b_extent = (collider2.max.y() - collider2.min.y()) * 0.5;
            let y_overlap = a_extent + b_extent - d.y().abs();

            if y_overlap > 0.0 {
                let a_y_contained =
                    collider1.min.y() > collider2.min.y() && collider1.max.y() < collider2.max.y();
                let b_y_contained =
                    collider2.min.y() > collider1.min.y() && collider2.max.y() < collider1.max.y();

                let n_x = if d.x() < 0.0 { -1.0 } else { 1.0 };
                let n_y = if d.y() < 0.0 { -1.0 } else { 1.0 };
                let manifold = Manifold {
                    body1: collider1.body,
                    body2: collider2.body,
                    penetration: Vec2::new(x_overlap, y_overlap),
                    normals: Vec2::new(n_x, n_y),
                    contained: [
                        Contained {
                            x: a_x_contained,
                            y: a_y_contained,
                        },
                        Contained {
                            x: b_x_contained,
                            y: b_y_contained,
                        },
                    ],
                };
                manifolds.send(manifold);
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
    mut query: Query<(Mut<RigidBody>, Option<&Collisions>)>,
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

        {
            let rv = b.velocity.x() - a.velocity.x();

            let restitution = a.restitution.min(b.restitution);

            let mut j = -(1.0 + restitution) * rv;
            j /= a.inv_mass + b.inv_mass;

            let impulse = j * manifold.normals.x();
            let percent = 0.2;
            let slop = 0.01;
            let correction = manifold.normals.x() * (manifold.penetration.x() - slop).max(0.0)
                / (a.inv_mass + b.inv_mass)
                * percent;
            mem::drop(a);
            mem::drop(b);

            let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
            match a.status {
                Status::Static => {}
                Status::Dynamic => {
                    let inv_mass = a.inv_mass;
                    if !manifold.contained[0].x {
                        *a.velocity.x_mut() -= impulse * inv_mass;
                        *a.position.x_mut() -= inv_mass * correction;
                    }
                }
                Status::Semikinematic => {
                    let mut solve = true;
                    if let Ok(collisions) = query.get::<Collisions>(manifold.body1) {
                        for info in &collisions.x {
                            if info.other == manifold.body2 {
                                solve = false;
                                break;
                            }
                        }
                    }
                    if solve && !a.solved.x {
                        if !manifold.contained[0].x {
                            let d = -manifold.normals.x() * manifold.penetration.x();
                            let v = a.velocity.x() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else if d.abs() < v.abs() {
                                if step.axis == StepAxis::Y {
                                    a_maybe_step = Some((-d, a.velocity.x()));
                                }
                                a.solved.x = true;
                                *a.velocity.x_mut() = 0.0;
                                *a.position.x_mut() += d;
                            } else {
                                if step.axis == StepAxis::Y {
                                    a_maybe_step = Some((v, a.velocity.x()));
                                }
                                a.solved.x = true;
                                *a.velocity.x_mut() = 0.0;
                                *a.position.x_mut() -= v;
                            }
                        }
                    }
                }
            }
            mem::drop(a);

            let mut b = query.get_mut::<RigidBody>(manifold.body2).unwrap();
            match b.status {
                Status::Static => {}
                Status::Dynamic => {
                    let inv_mass = b.inv_mass;
                    if !manifold.contained[1].x {
                        *b.velocity.x_mut() += impulse * inv_mass;
                        *b.position.x_mut() += inv_mass * correction;
                    }
                }
                Status::Semikinematic => {
                    let mut solve = true;
                    if let Ok(collisions) = query.get::<Collisions>(manifold.body2) {
                        for info in &collisions.x {
                            if info.other == manifold.body1 {
                                solve = false;
                                break;
                            }
                        }
                    }
                    if solve && !b.solved.x {
                        if !manifold.contained[1].x {
                            let d = manifold.normals.x() * manifold.penetration.x();
                            let v = b.velocity.x() * delta_time;
                            if v.signum() == d.signum() {
                                // nothing
                            } else if d.abs() < v.abs() {
                                if step.axis == StepAxis::Y {
                                    b_maybe_step = Some((-d, b.velocity.x()));
                                }
                                b.solved.x = true;
                                *b.velocity.x_mut() = 0.0;
                                *b.position.x_mut() += d;
                            } else {
                                if step.axis == StepAxis::Y {
                                    b_maybe_step = Some((v, b.velocity.x()));
                                }
                                b.solved.x = true;
                                *b.velocity.x_mut() = 0.0;
                                *b.position.x_mut() -= v;
                            }
                        }
                    }
                }
            }
            mem::drop(b);
        }

        let a = query.get::<RigidBody>(manifold.body1).unwrap();
        let b = query.get::<RigidBody>(manifold.body2).unwrap();

        {
            let rv = b.velocity.y() - a.velocity.y();

            let restitution = a.restitution.min(b.restitution);

            let mut j = -(1.0 + restitution) * rv;
            j /= a.inv_mass + b.inv_mass;

            let impulse = j * manifold.normals.y();
            let percent = 0.2;
            let slop = 0.01;
            let correction = manifold.normals.y() * (manifold.penetration.y() - slop).max(0.0)
                / (a.inv_mass + b.inv_mass)
                * percent;
            mem::drop(a);
            mem::drop(b);

            let mut a = query.get_mut::<RigidBody>(manifold.body1).unwrap();
            match a.status {
                Status::Static => {}
                Status::Dynamic => {
                    let inv_mass = a.inv_mass;
                    if !manifold.contained[0].y {
                        *a.velocity.y_mut() -= impulse * inv_mass;
                        *a.position.y_mut() -= inv_mass * correction;
                    }
                }
                Status::Semikinematic => {
                    let mut solve = true;
                    if let Ok(collisions) = query.get::<Collisions>(manifold.body1) {
                        for info in &collisions.y {
                            if info.other == manifold.body2 {
                                solve = false;
                                break;
                            }
                        }
                    }
                    if solve && !a.solved.y {
                        if !manifold.contained[0].y {
                            let d = -manifold.normals.y() * manifold.penetration.y();
                            if a_maybe_step.is_some() && step.axis == StepAxis::Y && d <= step.step
                            {
                                let (dx, vx) = a_maybe_step.unwrap();
                                *a.position.y_mut() += d;
                                *a.position.x_mut() += dx;
                                *a.velocity.x_mut() = vx;
                                a.solved.x = false;
                                a.solved.y = true;
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
            mem::drop(a);

            let mut b = query.get_mut::<RigidBody>(manifold.body2).unwrap();
            match b.status {
                Status::Static => {}
                Status::Dynamic => {
                    let inv_mass = b.inv_mass;
                    if !manifold.contained[1].y {
                        *b.velocity.y_mut() += impulse * inv_mass;
                        *b.position.y_mut() += inv_mass * correction;
                    }
                }
                Status::Semikinematic => {
                    let mut solve = true;
                    if let Ok(collisions) = query.get::<Collisions>(manifold.body2) {
                        for info in &collisions.y {
                            if info.other == manifold.body1 {
                                solve = false;
                                break;
                            }
                        }
                    }
                    if solve && !b.solved.y {
                        if !manifold.contained[1].y {
                            let d = manifold.normals.y() * manifold.penetration.y();
                            if b_maybe_step.is_some() && step.axis == StepAxis::Y && d <= step.step
                            {
                                let (dx, vx) = b_maybe_step.unwrap();
                                *b.position.y_mut() += d;
                                *b.position.x_mut() += dx;
                                *b.velocity.x_mut() = vx;
                                b.solved.x = false;
                                b.solved.y = true;
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
    }

    for (e, info) in transaction {
        commands.insert_one(e, info);
    }

    for (mut body, _) in &mut query.iter() {
        body.solved.x = false;
        body.solved.y = false;
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
        if !body.active || matches!(body.status, Status::Static) {
            continue;
        }

        body.accumulator += gravity.0;

        let velocity = body.velocity + body.accumulator * delta_time;
        body.velocity = velocity;
        body.accumulator = Vec2::zero();

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
            }
            Status::Dynamic => body.velocity *= friction.0,
            Status::Static => {}
        }
        body.prev_velocity = body.velocity;
    }
}

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
    for (body, mut transform) in &mut query.iter() {
        match *translation_mode {
            TranslationMode::AxesXY => {
                let x = body.position.x();
                let y = body.position.y();
                let z = 0.0;
                transform.set_translation(Vec3::new(x, y, z));
            }
            TranslationMode::AxesXZ => {
                let x = body.position.x();
                let y = 0.0;
                let z = body.position.y();
                transform.set_translation(Vec3::new(x, y, z));
            }
            TranslationMode::AxesYZ => {
                let x = 0.0;
                let y = body.position.x();
                let z = body.position.y();
                transform.set_translation(Vec3::new(x, y, z));
            }
        }
        match *rotation_mode {
            RotationMode::AxisX => {
                transform.set_rotation(Quat::from_rotation_x(body.rotation));
            }
            RotationMode::AxisY => {
                transform.set_rotation(Quat::from_rotation_y(body.rotation));
            }
            RotationMode::AxisZ => {
                transform.set_rotation(Quat::from_rotation_z(body.rotation));
            }
        }
    }
}

pub const DEBUG_MESH: Handle<Mesh> = Handle::from_bytes([
    226, 81, 40, 16, 70, 208, 159, 71, 87, 31, 13, 163, 88, 87, 21, 114,
]);

pub fn init_debug_system(translation_mode: Res<TranslationMode>, mut meshes: ResMut<Assets<Mesh>>) {
    let positions = match *translation_mode {
        TranslationMode::AxesXY => vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        TranslationMode::AxesXZ => vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ],
        TranslationMode::AxesYZ => vec![
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 1.0],
            [0.0, 0.0, 1.0],
        ],
    };
    let normals = match *translation_mode {
        TranslationMode::AxesXY => vec![[0.0, 0.0, 1.0]; 4],
        TranslationMode::AxesXZ => vec![[0.0, 1.0, 0.0]; 4],
        TranslationMode::AxesYZ => vec![[1.0, 0.0, 0.0]; 4],
    };
    let uvs = vec![[0.0, 0.0]; 4];
    let indices = vec![0, 1, 1, 2, 2, 3, 3, 0];
    let attributes = vec![
        VertexAttribute::position(positions),
        VertexAttribute::normal(normals),
        VertexAttribute::uv(uvs),
    ];
    let mesh = Mesh {
        primitive_topology: PrimitiveTopology::LineList,
        attributes,
        indices: Some(indices),
    };
    meshes.set(DEBUG_MESH, mesh);
}

#[derive(Debug, Clone, Copy, RenderResources, ShaderDefs)]
pub struct DebugMaterial {
    color: Color,
}

impl From<Color> for DebugMaterial {
    fn from(color: Color) -> Self {
        Self { color }
    }
}

#[derive(Debug)]
pub struct DebugBody;

#[derive(Debug)]
pub struct DebugShape;

#[derive(Debug)]
pub struct DebugRender;

#[derive(Bundle)]
pub struct DebugRenderComponents {
    pub debug: DebugRender,
    pub mesh: Handle<Mesh>,
    pub material: Handle<DebugMaterial>,
    pub main_pass: MainPass,
    pub draw: Draw,
    pub render_pipelines: RenderPipelines,
    pub transform: Transform,
    pub global_transform: GlobalTransform,
}

impl Default for DebugRenderComponents {
    fn default() -> Self {
        Self {
            debug: DebugRender,
            mesh: DEBUG_MESH,
            material: Default::default(),
            main_pass: Default::default(),
            draw: Default::default(),
            render_pipelines: RenderPipelines::from_pipelines(vec![RenderPipeline::specialized(
                DEBUG_PIPELINE_HANDLE,
                PipelineSpecialization {
                    dynamic_bindings: vec![DynamicBinding {
                        bind_group: 2,
                        binding: 0,
                    }],
                    ..Default::default()
                },
            )]),
            transform: Default::default(),
            global_transform: Default::default(),
        }
    }
}

pub fn debug_render_system(
    mut commands: Commands,
    mut materials: ResMut<Assets<DebugMaterial>>,
    mut query: Query<With<DebugBody, (Entity, &RigidBody, &Children)>>,
    shapes: Query<Without<DebugShape, &Shape>>,
    mut debug: Query<With<DebugRender, (&Parent, Mut<Draw>)>>,
) {
    for (entity, body, children) in &mut query.iter() {
        let color = if body.sensor {
            Color::rgb(0.0, 1.0, 1.0)
        } else {
            match body.status {
                Status::Static => Color::rgb(0.0, 1.0, 0.0),
                Status::Dynamic => Color::rgb(1.0, 0.0, 0.0),
                Status::Semikinematic => Color::rgb(0.0, 0.0, 1.0),
            }
        };
        let mut material = None;
        let mut new_children = SmallVec::<[Entity; 8]>::new();
        for &shape in children.iter() {
            if let Ok(shape_ref) = shapes.get::<Shape>(shape) {
                if material.is_none() {
                    material = Some(materials.add(color.into()));
                }
                let material = material.unwrap();
                let offset = shape_ref.offset;
                let size = shape_ref.size;
                commands
                    .insert_one(shape, DebugShape)
                    .spawn(DebugRenderComponents {
                        material,
                        transform: Transform::from_translation(Vec3::new(
                            offset.x(),
                            offset.y(),
                            0.0,
                        ))
                        .with_non_uniform_scale(Vec3::new(
                            size.width,
                            size.height,
                            1.0,
                        )),
                        ..Default::default()
                    })
                    .with(Parent(entity))
                    .for_current_entity(|e| {
                        new_children.push(e);
                    });
            }
        }
        commands.push_children(entity, &new_children);
    }

    for (parent, mut draw) in &mut debug.iter() {
        let body = query.get::<RigidBody>(**parent).unwrap();
        draw.is_visible = body.active;
    }
}

pub const DEBUG_PIPELINE_HANDLE: Handle<PipelineDescriptor> = Handle::from_bytes([
    94, 231, 249, 140, 146, 108, 29, 243, 61, 57, 59, 120, 40, 178, 172, 61,
]);

pub fn build_debug_pipeline(shaders: &mut Assets<Shader>) -> PipelineDescriptor {
    PipelineDescriptor {
        rasterization_state: Some(RasterizationStateDescriptor {
            front_face: FrontFace::Ccw,
            cull_mode: CullMode::None,
            depth_bias: 0,
            depth_bias_slope_scale: 0.0,
            depth_bias_clamp: 0.0,
            clamp_depth: false,
        }),
        depth_stencil_state: Some(DepthStencilStateDescriptor {
            format: TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: CompareFunction::LessEqual,
            stencil: StencilStateDescriptor {
                front: StencilStateFaceDescriptor::IGNORE,
                back: StencilStateFaceDescriptor::IGNORE,
                read_mask: 0,
                write_mask: 0,
            },
        }),
        color_states: vec![ColorStateDescriptor {
            format: TextureFormat::Bgra8UnormSrgb,
            color_blend: BlendDescriptor {
                src_factor: BlendFactor::SrcAlpha,
                dst_factor: BlendFactor::OneMinusSrcAlpha,
                operation: BlendOperation::Add,
            },
            alpha_blend: BlendDescriptor {
                src_factor: BlendFactor::One,
                dst_factor: BlendFactor::One,
                operation: BlendOperation::Add,
            },
            write_mask: ColorWrite::ALL,
        }],
        ..PipelineDescriptor::new(ShaderStages {
            vertex: shaders.add(Shader::from_glsl(
                ShaderStage::Vertex,
                include_str!("debug_shader.vert"),
            )),
            fragment: Some(shaders.add(Shader::from_glsl(
                ShaderStage::Fragment,
                include_str!("debug_shader.frag"),
            ))),
        })
    }
}

pub mod node {
    pub const TRANSFORM: &str = "transform";
    pub const DEBUG_MATERIAL: &str = "debug_material";
}

pub trait DebugRenderGraphBuilder {
    fn add_debug_graph(&mut self, resources: &Resources) -> &mut Self;
}

impl DebugRenderGraphBuilder for RenderGraph {
    fn add_debug_graph(&mut self, resources: &Resources) -> &mut Self {
        self.add_system_node(
            node::TRANSFORM,
            RenderResourcesNode::<GlobalTransform>::new(true),
        );
        self.add_system_node(
            node::DEBUG_MATERIAL,
            AssetRenderResourcesNode::<ColorMaterial>::new(false),
        );

        self.add_node_edge(node::TRANSFORM, base::node::MAIN_PASS)
            .unwrap();
        self.add_node_edge(node::DEBUG_MATERIAL, base::node::MAIN_PASS)
            .unwrap();

        let mut pipelines = resources.get_mut::<Assets<PipelineDescriptor>>().unwrap();
        let mut shaders = resources.get_mut::<Assets<Shader>>().unwrap();
        pipelines.set(DEBUG_PIPELINE_HANDLE, build_debug_pipeline(&mut shaders));
        self
    }
}
