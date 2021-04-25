use bevy::prelude::*;
use physme::prelude2d::*;

#[derive(Default)]
pub struct CharacterController {

}

fn main() {
    let mut builder = App::build();
    builder
        .add_plugins(DefaultPlugins)
        .add_plugin(Physics2dPlugin)
        .insert_resource(GlobalGravity(Vec2::new(0.0, -540.0)))
        .insert_resource(GlobalFriction(0.90))
        .insert_resource(GlobalStep(0.0))
        .insert_resource(GlobalUp(Vec2::new(0.0, 1.0)))
        .add_startup_system(setup.system())
        .add_system(bevy::input::system::exit_on_esc_system.system());
    // let character_system = CharacterControllerSystem::default().system(builder.resources_mut());
    builder.add_system(character_system.system());
    builder.run();
}

fn setup(
    mut commands: Commands,
    // asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // let icon = asset_server.load("icon.png");
    // let plat = asset_server.load("platform.png");
    // let square = asset_server.load("square.png");

    let blue = materials.add(Color::ALICE_BLUE.into());
    let black = materials.add(Color::BLACK.into());
    let another_color = materials.add(Color::GOLD.into());

    // Spawn the damn camera
    commands
        .spawn_bundle(OrthographicCameraBundle::new_2d());


    // Spawn character
    let _player_id = commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(28.0,28.0)),
            material: blue.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(0.0, 0.0))
                .with_terminal(Vec2::new(500.0, 1000.0))
                .with_angular_terminal(7.8),
        )
        .insert(CharacterController::default())
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(28.0, 28.0)),));
        }).id();
    
    // center floor
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(600.0,20.0)),
            material: black.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(150.0, -200.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(600.0, 20.0)),));
        });

    // wall
    commands
    .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(20.0,500.0)),
            material: black.clone(),
            ..Default::default()
    })
    .insert(
        RigidBody::new(Mass::Infinite)
        .with_status(Status::Static)
        .with_position(Vec2::new(450.0, 0.0))
    )
    .with_children(|parent| {
        parent.spawn_bundle((Shape::from(Size2::new(20.0, 500.0)),));
    });
    // spawn another floor
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(300.0,20.0)),
            material: black.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(-300.0, -190.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(300.0, 20.0)),));
        });

    // yet another floor
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(120.0,20.0)),
            material: black.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(360.0, -50.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(120.0, 20.0)),));
        });

    // is this the last floor?
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(120.0,20.0)),
            material: black.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(120.0, -10.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(120.0, 20.0)),));
        });

    // dude i think there is enough floors already
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::new(120.0,20.0)),
            material: black.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Infinite)
                .with_status(Status::Static)
                .with_position(Vec2::new(-120.0, 20.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(120.0, 20.0)),));
        });

    // Spawn the cube near us
    commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::splat(20.0)),
            material: another_color.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Real(2.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(30.0, 60.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(20.0, 20.0)),));
        });

    // spawn the cube connected to us
    let _target_id = commands
        .spawn_bundle(SpriteBundle {
            sprite : Sprite::new(Vec2::splat(20.0)),
            material: another_color.clone(),
            ..Default::default()
        })
        .insert(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(100.0, 100.0)),
        )
        .with_children(|parent| {
            parent.spawn_bundle((Shape::from(Size2::new(20.0, 20.0)),));
        }).id();

    // spawn the joint between the player and the target cube
    // commands
    //     .spawn()
    //     .insert(
    //         SpringJoint::new(player_id, target_id).with_offset(Vec2::new(30.0,30.0))
    //     );
        // .for_current_entity(|e| target = Some(e))
        // .spawn((
        //     SpringJoint::new(anchor.unwrap(), target.unwrap()).with_offset(Vec2::new(30.0, 30.0)),
        // ));
}

#[derive(Default)]
pub struct CharacterControllerSystem;
// {
//     reader: EventReader<Manifold>,
// }

// impl CharacterControllerSystem {
//     pub fn system(self, res: &mut Resources) -> Box<dyn System> {
//         let system = character_system.system();
//         res.insert_local(system.id(), self);
//         system
//     }
// }

fn character_system(
    input: Res<Input<KeyCode>>,
    gravity : Res<GlobalGravity>,
    // manifolds: Res<Events<Manifold>>,
    mut query: Query<(&CharacterController, &mut RigidBody)>,
) {
    for (_, mut body) in query.iter_mut() {
        if let Some(normal) = body.on_wall() {
            body.linvel -= normal * 0.1;

            if body.linvel.y < -1.0 {
                body.linvel.y = -1.0;
            }
        }

        if body.on_floor().is_some() || body.on_wall().is_some() {
            if input.just_pressed(KeyCode::Space) || input.just_pressed(KeyCode::W) {
                // This is just a weird way to do jump, using the gravity direction and size(tho you dont need the size)
                // it works by sliding on the gravity direction(so nothing in the direction of gravity)
                // then adding the jump force(here its gravity * 0.5) to the velocity
                body.linvel = body.linvel.slide(gravity.0.normalize()) - gravity.0 * 0.8;
                let wall = body.on_wall().unwrap_or(Vec2::ZERO) * 250.0;
                body.linvel += wall;
            }
        }
        if input.pressed(KeyCode::A) {
            body.apply_linear_impulse(Vec2::new(-5.0, 0.0));
            // body.apply_angular_impulse(1.0);
        }
        if input.pressed(KeyCode::D) {
            body.apply_linear_impulse(Vec2::new(5.0, 0.0));
            // body.apply_angular_impulse(-1.0);
        }
    }
}
