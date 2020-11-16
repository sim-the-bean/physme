use bevy::prelude::*;
use physme::prelude3d::*;

pub struct CharacterController {
    on_ground: bool,
    jump: bool,
    camera: Entity,
}

impl CharacterController {
    pub fn new(camera: Entity) -> Self {
        Self {
            on_ground: false,
            jump: false,
            camera,
        }
    }
}

fn main() {
    let mut builder = App::build();
    builder
        .add_plugins(DefaultPlugins)
        .add_plugin(Physics3dPlugin)
        .add_resource(GlobalFriction(0.90))
        .add_resource(GlobalStep(0.5))
        .add_startup_system(setup.system());
    let character_system = CharacterControllerSystem::default().system(builder.resources_mut());
    builder.add_system(character_system);
    builder.run();
}

fn setup(
    commands: &mut Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube = meshes.add(shape::Cube { size: 0.5 }.into());
    let bigcube = meshes.add(shape::Cube { size: 8.0 }.into());
    let smallcube = meshes.add(shape::Cube { size: 0.2 }.into());
    let mut camera = None;
    let mut anchor = None;
    let mut target = None;
    commands
        .spawn(LightComponents {
            transform: Transform::from_translation(Vec3::new(0.0, 5.0, 5.0)),
            ..Default::default()
        })
        .spawn((Transform::identity(), GlobalTransform::identity()))
        .with_children(|parent| {
            let mut transform = Transform::from_translation(Vec3::new(0.0, 8.0, 8.0));
            transform.rotation = Quat::from_rotation_x(-45.0_f32.to_radians());
            parent.spawn(Camera3dComponents {
                transform,
                ..Default::default()
            });
        })
        .for_current_entity(|e| camera = Some(e))
        .spawn(PbrComponents {
            mesh: cube.clone_weak(),
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec3::new(0.0, 2.0, 0.0)),
        )
        .with(Up::default())
        .with(UpRotation::default())
        .with(CharacterController::new(camera.unwrap()))
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(1.0, 1.0, 1.0)),));
        })
        .for_current_entity(|e| anchor = Some(e))
        .spawn(PbrComponents {
            mesh: smallcube.clone_weak(),
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(0.5))
                .with_status(Status::Semikinematic)
                .with_position(Vec3::new(10.0, 10.0, 10.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(0.2, 0.2, 0.2)),));
        })
        .for_current_entity(|e| target = Some(e))
        .spawn((SpringJoint::new(anchor.unwrap(), target.unwrap())
            .with_rigidness(0.5)
            .with_offset(Vec3::new(2.0, 2.0, 0.0)),))
        .spawn(PbrComponents {
            mesh: cube,
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec3::new(5.0, 5.0, 5.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(1.0, 1.0, 1.0)),));
        })
        .spawn(PbrComponents {
            mesh: bigcube.clone_weak(),
            material: materials.add(Color::rgb(0.2, 0.8, 0.2).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec3::new(0.0, -8.0, 0.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(16.0, 16.0, 16.0)),));
        })
        .spawn(PbrComponents {
            mesh: bigcube,
            material: materials.add(Color::rgb(0.5, 0.5, 0.5).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec3::new(0.0, -7.0, -16.0))
                .with_rotation(Quat::from_rotation_x(10.0_f32.to_radians())),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(16.0, 16.0, 16.0)),));
        })
        .spawn(PbrComponents {
            mesh: smallcube,
            material: materials.add(Color::rgb(0.2, 0.8, 0.2).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec3::new(-3.0, 0.0, -3.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(0.4, 0.4, 0.4)),));
        });
}

#[derive(Default)]
pub struct CharacterControllerSystem {
    reader: EventReader<Manifold>,
}

impl CharacterControllerSystem {
    pub fn system(self, res: &mut Resources) -> Box<dyn System> {
        let system = character_system.system();
        res.insert_local(system.id(), self);
        system
    }
}

fn character_system(
    mut state: Local<CharacterControllerSystem>,
    time: Res<Time>,
    input: Res<Input<KeyCode>>,
    manifolds: Res<Events<Manifold>>,
    up: Res<GlobalUp>,
    ang_tol: Res<AngularTolerance>,
    mut query: Query<(Mut<CharacterController>, Mut<RigidBody>, Mut<UpRotation>)>,
    mut camera: Query<Mut<Transform>>,
) {
    let delta_time = time.delta.as_secs_f32();
    for manifold in state.reader.iter(&manifolds) {
        let dot = up.0.dot(manifold.normal);
        let angle = (-dot).acos();
        let angle2 = dot.acos();
        if angle >= 0.0 && angle < ang_tol.0 {
            if let Ok(mut controller) =
                query.get_component_mut::<CharacterController>(manifold.body1)
            {
                controller.on_ground = true;
            }
        } else if angle2 >= 0.0 && angle2 < ang_tol.0 {
            if let Ok(mut controller) =
                query.get_component_mut::<CharacterController>(manifold.body2)
            {
                controller.on_ground = true;
            }
        }
    }

    for (mut controller, mut body, mut rotation) in query.iter_mut() {
        if input.just_pressed(KeyCode::Space) {
            controller.jump = true;
        }
        if controller.on_ground {
            if controller.jump {
                body.apply_force(Vec3::new(0.0, 500.0, 0.0));
                controller.jump = false;
            }
        }
        if input.pressed(KeyCode::Q) {
            rotation.0 += 1.0 * delta_time;
        }
        if input.pressed(KeyCode::E) {
            rotation.0 -= 1.0 * delta_time;
        }
        if input.pressed(KeyCode::W) {
            let impulse = body.rotation * Vec3::new(0.0, 0.0, -0.5);
            body.apply_linear_impulse(impulse);
        }
        if input.pressed(KeyCode::S) {
            let impulse = body.rotation * Vec3::new(0.0, 0.0, 0.5);
            body.apply_linear_impulse(impulse);
        }
        if input.pressed(KeyCode::A) {
            let impulse = body.rotation * Vec3::new(-0.5, 0.0, 0.0);
            body.apply_linear_impulse(impulse);
        }
        if input.pressed(KeyCode::D) {
            let impulse = body.rotation * Vec3::new(0.5, 0.0, 0.0);
            body.apply_linear_impulse(impulse);
        }
        controller.on_ground = false;

        let pitch = rotation.0;
        if let Ok(mut transform) = camera.get_component_mut::<Transform>(controller.camera) {
            transform.translation = body.position;
            transform.rotation = Quat::from_rotation_y(pitch);
        }
    }
}
