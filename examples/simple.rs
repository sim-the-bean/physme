use bevy::prelude::*;
use physme::prelude2d::*;

#[derive(Default)]
pub struct CharacterController {
    on_ground: bool,
    jump: bool,
}

fn main() {
    let mut builder = App::build();
    builder
        .add_plugins(DefaultPlugins)
        .add_plugin(Physics2dPlugin)
        .add_resource(GlobalGravity(Vec2::new(0.0, -500.0)))
        .add_resource(GlobalFriction(0.90))
        .add_resource(GlobalStep(15.0))
        .add_resource(GlobalUp(Vec2::new(0.0, 1.0)))
        .add_startup_system(setup.system());
    let character_system = CharacterControllerSystem::default().system(builder.resources_mut());
    builder.add_system(character_system);
    builder.run();
}

fn setup(
    commands: &mut Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let icon = asset_server.load("icon.png");
    let plat = asset_server.load("platform.png");
    let square = asset_server.load("square.png");
    let mut anchor = None;
    let mut target = None;
    commands
        .spawn(Camera2dComponents::default())
        .spawn(SpriteComponents {
            material: materials.add(icon.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(0.0, 0.0))
                .with_terminal(Vec2::new(500.0, 1000.0))
                .with_angular_terminal(7.8),
        )
        .with(CharacterController::default())
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(28.0, 28.0)),));
        })
        .for_current_entity(|e| anchor = Some(e))
        .spawn(SpriteComponents {
            material: materials.add(plat.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(0.0, -100.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(120.0, -90.0))
                .with_rotation(10.0_f32.to_radians()),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(-120.0, -90.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(360.0, -50.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(120.0, -10.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(-120.0, 20.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(square.clone_weak().into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(30.0, 60.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(20.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(square.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(100.0, 100.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size2::new(20.0, 20.0)),));
        })
        .for_current_entity(|e| target = Some(e))
        .spawn((
            SpringJoint::new(anchor.unwrap(), target.unwrap()).with_offset(Vec2::new(30.0, 30.0)),
        ));
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
    mut query: Query<(Mut<CharacterController>, Mut<RigidBody>)>,
) {
    for manifold in state.reader.iter(&manifolds) {
        if manifold.normal.y() < 0.0 {
            if let Ok(mut controller) =
                query.get_component_mut::<CharacterController>(manifold.body1)
            {
                controller.on_ground = true;
            }
        } else if manifold.normal.y() > 0.0 {
            if let Ok(mut controller) =
                query.get_component_mut::<CharacterController>(manifold.body2)
            {
                controller.on_ground = true;
            }
        }
    }

    for (mut controller, mut body) in query.iter_mut() {
        if input.just_pressed(KeyCode::Space) || input.just_pressed(KeyCode::W) {
            controller.jump = true;
        }
        if controller.on_ground {
            if controller.jump {
                body.apply_force(Vec2::new(0.0, 12000.0));
                controller.jump = false;
            }
        }
        if input.pressed(KeyCode::A) {
            body.apply_linear_impulse(Vec2::new(-5.0, 0.0));
            body.apply_angular_impulse(1.0);
        }
        if input.pressed(KeyCode::D) {
            body.apply_linear_impulse(Vec2::new(5.0, 0.0));
            body.apply_angular_impulse(-1.0);
        }
        controller.on_ground = false;
    }
}
