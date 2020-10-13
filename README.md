# physme

physme is a simple physics engine for 2d and 3d video games that 
doesn't produce correct results according to Newtonian mechanics,
it instead aims to provide satisfying results.  The engine only
supports static and semikinematic bodies.  Static bodies don't
move and only affect semikinematic bodies.  Semikinematic bodies
do move, but in ways that are physically inaccurate, but they
look good on the screen.  Semikinematic bodies are called so,
because they combine characteristics of kinematic and dynamic
bodies known from other physics engines.  They are affected by
forces, but also have their own made up physics.

This engine might be sufficient for your next jam game, or your
hobby project.  You should probably not use it for an AAA game.

By the way, it only works with Bevy.

## Quick Start

Creating your first game with physme is as simple as adding one of the
physics plugin

```rust
use physme::prelude2d::*;

let mut builder = App::build();
builder
    .add_plugin(Physics2dPlugin);
```

optionally setting some parameters

```rust
    .add_resource(GlobalGravity(Vec2::new(0.0, -500.0)))
    .add_resource(GlobalFriction(0.90))
    .add_resource(GlobalStep(15.0));
```

and then, in your `setup` function, adding a `RigidBody` component to your
entities

```rust
fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let icon = asset_server.load("assets/icon.png").unwrap();
    commands
        .spawn(SpriteComponents {
            material: materials.add(icon.into()),
            ..Default::default()
        })
        .with(
            RigidBody::new(Mass::Real(1.0))
                .with_status(Status::Semikinematic)
                .with_position(Vec2::new(0.0, 0.0))
                .with_terminal(Vec2::new(500.0, 1000.0)),
        );
}
```

as well as some children with an arbitrary amount of  `Shape` components.

```rust
        .with_children(|parent| {
            parent.spawn((Shape::from(Size::new(28.0, 28.0)),));
        });
```

And there you go! This will perform all the physics updates on every
frame of the game.
