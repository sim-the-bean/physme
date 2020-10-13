use bevy::prelude::*;
use physme::prelude3d::*;

pub trait Vec3Ext {
    fn quat_between(&self, other: Vec3) -> Quat;
}

impl Vec3Ext for Vec3 {
    fn quat_between(&self, other: Vec3) -> Quat {
        let dot = self.dot(other);
        if dot > 0.995 || dot < -0.995 {
            return Quat::identity();
        }

        let axis = self.cross(other);
        let angle = (self.length_squared() * other.length_squared()).sqrt() + dot;
        Quat::from_axis_angle(axis, angle)
    }
}

trait QuatExt {
    fn to_rotation_ypr(&self) -> (f32, f32, f32);
}

impl QuatExt for Quat {
    fn to_rotation_ypr(&self) -> (f32, f32, f32) {
        let sinr_cosp = 2.0 * (self.w() * self.x() + self.y() * self.z());
        let cosr_cosp = 1.0 - 2.0 * (self.x() * self.x() + self.y() * self.y());

        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = 2.0 * (self.w() * self.y() - self.z() * self.x());
        let pitch = if sinp.abs() >= 1.0 {
            std::f32::consts::PI * sinp.signum()
        } else {
            sinp.asin()
        };

        let siny_cosp = 2.0 * (self.w() * self.z() + self.x() * self.y());
        let cosy_cosp = 1.0 - 2.0 * (self.y() * self.y() + self.z() * self.z());
        let yaw = siny_cosp.atan2(cosy_cosp);
        (roll, pitch, yaw)
    }
}

pub struct CharacterController {
    on_ground: bool,
    jump: bool,
    camera: Entity,
    up: Vec3,
}

impl CharacterController {
    pub fn new(camera: Entity) -> Self {
        Self {
            on_ground: false,
            jump: false,
            camera,
            up: Vec3::new(0.0, 1.0, 0.0),
        }
    }
}

fn main() {
    let mut builder = App::build();
    builder
        .add_default_plugins()
        .add_plugin(Physics3dPlugin)
        .add_resource(GlobalFriction(0.90))
        .add_resource(GlobalStep(0.5))
        .add_startup_system(setup.system());
    let character_system = CharacterControllerSystem::default().system(builder.resources_mut());
    builder.add_system(character_system);
    builder.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube = meshes.add(shape::Cube { size: 0.5 }.into());
    let bigcube = meshes.add(shape::Cube { size: 8.0 }.into());
    let smallcube = meshes.add(shape::Cube { size: 0.2 }.into());
    let mut camera = None;
    commands
        .spawn(LightComponents {
            transform: Transform::from_translation(Vec3::new(0.0, 5.0, 5.0)),
            ..Default::default()
        })
        .spawn((Transform::identity(), GlobalTransform::identity()))
        .with_children(|parent| {
            parent.spawn(Camera3dComponents {
                transform: Transform::from_translation_rotation(
                    Vec3::new(0.0, 8.0, 8.0),
                    Quat::from_rotation_x(-45.0_f32.to_radians()),
                ),
                ..Default::default()
            });
        })
        .for_current_entity(|e| camera = Some(e))
        .spawn(PbrComponents {
            mesh: cube,
            material: materials.add(Color::rgb(1.0, 1.0, 1.0).into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec3::new(0.0, 2.0, 0.0)),
        )
        .with(CharacterController::new(camera.unwrap()))
        .with_children(|parent| {
            parent.spawn((Shape::from(Size3::new(1.0, 1.0, 1.0)),));
        })
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
            mesh: bigcube,
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
    input: Res<Input<KeyCode>>,
    manifolds: Res<Events<Manifold>>,
    up: Res<GlobalUp>,
    ang_tol: Res<AngularTolerance>,
    mut query: Query<(Mut<CharacterController>, Mut<RigidBody>)>,
    camera: Query<Mut<Transform>>,
) {
    for manifold in state.reader.iter(&manifolds) {
        let dot = up.0.dot(manifold.normal);
        let angle = (-dot).acos();
        let angle2 = dot.acos();
        if angle >= 0.0 && angle < ang_tol.0 {
            if let Ok(mut controller) = query.get_mut::<CharacterController>(manifold.body1) {
                controller.on_ground = true;

                // let mut body = query.get_mut::<RigidBody>(manifold.body1).unwrap();
                // let angle = controller.up.quat_between(manifold.normal);
                // let (axis, angle) = angle.to_axis_angle();
                // let angle = Quat::from_axis_angle(-axis, angle * 0.5).normalize();
                // body.rotation *= angle;

                // controller.up = manifold.normal;
            }
        } else if angle2 >= 0.0 && angle2 < ang_tol.0 {
            if let Ok(mut controller) = query.get_mut::<CharacterController>(manifold.body2) {
                controller.on_ground = true;

                // let mut body = query.get_mut::<RigidBody>(manifold.body2).unwrap();
                // let angle = controller.up.quat_between(manifold.normal);
                // let (axis, angle) = angle.to_axis_angle();
                // let angle = Quat::from_axis_angle(-axis, angle * 0.5).normalize();
                // body.rotation *= angle;

                // controller.up = -manifold.normal;
            }
        }
    }

    for (mut controller, mut body) in &mut query.iter() {
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
            body.apply_angular_impulse(Quat::from_rotation_y(0.2));
        }
        if input.pressed(KeyCode::E) {
            body.apply_angular_impulse(Quat::from_rotation_y(-0.2));
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

        let (_, pitch, _) = body.rotation.to_rotation_ypr();
        if let Ok(mut transform) = camera.get_mut::<Transform>(controller.camera) {
            transform.set_translation(body.position);
            transform.set_rotation(Quat::from_rotation_y(pitch));
        }
    }
}
