use bevy::prelude::*;
use physme::prelude2d::*;

#[derive(Default)]
pub struct CharacterController {
    on_ground: bool,
}

fn main() {
    let mut builder = App::build();
    builder
        .add_default_plugins()
        .add_plugin(Physics2dPlugin)
        .add_resource(GlobalGravity(Vec2::new(0.0, -500.0)))
        .add_resource(GlobalFriction(0.98))
        .add_startup_system(setup.system());
    let character_system = CharacterControllerSystem::default().system(builder.resources_mut());
    builder.add_system(character_system);
    builder.run();
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let icon = asset_server.load("assets/icon.png").unwrap();
    let plat = asset_server.load("assets/platform.png").unwrap();
    commands
        .spawn(Camera2dComponents::default())
        .spawn(SpriteComponents {
            material: materials.add(icon.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(0.0, 0.0)),
        )
        .with(CharacterController::default())
        .with_children(|parent| {
            parent.spawn((Shape::from(Size::new(28.0, 28.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Static)
                .with_position(Vec2::new(0.0, -100.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size::new(120.0, 20.0)),));
        })
        .spawn(SpriteComponents {
            material: materials.add(plat.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Static)
                .with_position(Vec2::new(120.0, -80.0)),
        )
        .with_children(|parent| {
            parent.spawn((Shape::from(Size::new(120.0, 20.0)),));
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
    mut query: Query<(Mut<CharacterController>, Mut<RigidBody>)>,
) {
    for manifold in state.reader.iter(&manifolds) {
        if manifold.normals.y() < 0.0 {
            if let Ok(mut controller) = query.get_mut::<CharacterController>(manifold.body1) {
                controller.on_ground = true;
            }
        } else if manifold.normals.y() > 0.0 {
            if let Ok(mut controller) = query.get_mut::<CharacterController>(manifold.body2) {
                controller.on_ground = true;
            }
        }
    }

    for (mut controller, mut body) in &mut query.iter() {
        if controller.on_ground {
            if input.just_pressed(KeyCode::Space) || input.just_pressed(KeyCode::W) {
                body.apply_force(Vec2::new(0.0, 12000.0));
            }
        }
        if input.pressed(KeyCode::A) {
            body.apply_impulse(Vec2::new(-5.0, 0.0));
        }
        if input.pressed(KeyCode::D) {
            body.apply_impulse(Vec2::new(5.0, 0.0));
        }
        controller.on_ground = false;
    }
}
