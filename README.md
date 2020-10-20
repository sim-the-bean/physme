# physme

physme is a simple physics engine for 2d and 3d video games that 
doesn't produce correct results according to Newtonian mechanics,
it instead aims to provide satisfying results. One big goal of
the engine is to provide a very simple and ergonomic API that will
allow anyone to get a quick start into physics in their game. Part
of this goal is to make 2D and 3D physics have the same API and
the same effects on the world, but in different spaces.

The engine only
supports static and semikinematic bodies.  Static bodies don't
move and only affect semikinematic bodies.  Semikinematic bodies
do move, but in ways that are physically inaccurate, but they
look good on the screen.  Semikinematic bodies are called so,
because they combine characteristics of kinematic and dynamic
bodies known from other physics engines.  They are affected by
forces, but also have their own made up physics.

Three types of joints are also included.
- `FixedJoint` is the simplest joint. It simply locks the target body
   at the anchor with an offset and an angle without considering other
   geometry (ignoring collisions).
- `MechanicalJoint` is like a `FixedJoint`, but it does consider other
   geometry (allows the target body to be displaced by collisions).
   The `MechanicalJoint` also appears to provide some minor "cushioning",
   or a spring effect. This is an artifact of how joints are calculated.
   If you want real spring joints, see the next point.
- `SpringJoint` will "jump" the target body to its target position
   over time. This time can be controlled by the `rigidness` value.
   A rigidness of 0.0 means high springiness, where the time it
   takes for the target body to go into position is some high
   unspecified number. A rigidness of 1.0 means low springiness,
   where  the time it the target body to go into position is some low
   number,  but bigger than `0.0` or even `f32::EPSILON` (it may or
   may not be 100ms).

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
    let icon = asset_server.load("icon.png");
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

If you want to add one of the three joint types to your bodies, you
would first have to get the `Entity` ids while spawning an entity

```rust
let mut anchor = None;
let mut target = None;
    // (...)
    .spawn(...)
    .for_current_entity(|e| anchor = Some(e))
    // (...)
    .spawn(...)
    .for_current_entity(|e| target = Some(e))
```

then spawn a separate entity with one of the joint components.

```rust
    // (...)
    .spawn((SpringJoint::new(anchor.unwrap(), target.unwrap())
        .with_offset(Vec2::new(50.0, 50.0)),))
```

The joints will then be calculated in one of the three provided
`joint_system`s. It is also possible to create custom joints using
the provided `JointBehaviour` trait in either `physme::dim2` or
`physme::dim3` and later adding the `joint_system::<B>`, where `B`
is your own behaviour. Joints are able to manipulate the position,
rotation, linear and angular velocity and apply a linear or angular
impulse.
