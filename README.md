# physme.rs

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

By the way, it only works with Bevy.  Also, there are only AABBs
(for now).
