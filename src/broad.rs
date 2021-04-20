//! This module provides the broad phase using an R*-tree.  You shouldn't
//! have to use it directly.

use std::ops::{Deref, DerefMut};

use bevy::math::*;
use rstar::{Point, RTree, RTreeObject, AABB};

use crate::common::Status;
use crate::dim2;
use crate::dim3;

mod private {
    use bevy::math::{Vec2, Vec3};

    pub trait Seal {}

    impl Seal for Vec2 {}
    impl Seal for Vec3 {}
}

#[doc(hidden)]
pub trait PhysPoint: private::Seal {}

#[doc(hidden)]
impl PhysPoint for Vec2 {}

#[doc(hidden)]
impl PhysPoint for Vec3 {}

#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct NPoint<T: PhysPoint>(T);

#[doc(hidden)]
impl<T: PhysPoint> NPoint<T> {
    pub fn into_inner(self) -> T {
        self.0
    }
}

#[doc(hidden)]
impl<T: PhysPoint> From<T> for NPoint<T> {
    fn from(p: T) -> Self {
        Self(p)
    }
}

#[doc(hidden)]
impl<T: PhysPoint> Deref for NPoint<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[doc(hidden)]
impl<T: PhysPoint> DerefMut for NPoint<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[doc(hidden)]
impl Point for NPoint<Vec2> {
    type Scalar = f32;
    const DIMENSIONS: usize = 2;

    fn generate(generator: impl Fn(usize) -> Self::Scalar) -> Self {
        Self::from(Vec2::new(generator(0), generator(1)))
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        match index {
            0 => self.0.x,
            1 => self.0.y,
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        match index {
            0 => &mut self.0.x,
            1 => &mut self.0.y,
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }
}

#[doc(hidden)]
impl Point for NPoint<Vec3> {
    type Scalar = f32;
    const DIMENSIONS: usize = 3;

    fn generate(generator: impl Fn(usize) -> Self::Scalar) -> Self {
        Self::from(Vec3::new(generator(0), generator(1), generator(2)))
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        match index {
            0 => self.0.x,
            1 => self.0.y,
            2 => self.0.z,
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        match index {
            0 => &mut self.0.x,
            1 => &mut self.0.y,
            2 => &mut self.0.z,
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }
}

/// A simple wrapper around `rstar`s AABB.  Generic over the dimesions of the point.
pub struct BoundingBox<P: PhysPoint>
where
    NPoint<P>: Point,
{
    aabb: AABB<NPoint<P>>,
}

impl<P: PhysPoint> BoundingBox<P>
where
    NPoint<P>: Point,
{
    /// Create a new `BoundingBox` with the minimum and maximum points.
    pub fn new(min: P, max: P) -> Self {
        Self {
            aabb: AABB::from_corners(NPoint::from(min), NPoint::from(max)),
        }
    }
}

/// A generic collider trait for the shapes in `dim2` and `dim3`.
pub trait Collider
where
    NPoint<Self::Point>: Point,
{
    /// The n-dimensional point over which this collider is generic.
    type Point: PhysPoint;

    /// Get the `BoundingBox` of this collider.
    fn bounding_box(&self) -> BoundingBox<Self::Point>;
    /// Get the `Status` of this collider (Static or Semikinematic).
    fn status(&self) -> Status;
}

impl RTreeObject for dim2::Obb {
    type Envelope = AABB<NPoint<Vec2>>;

    fn envelope(&self) -> Self::Envelope {
        self.bounding_box().aabb
    }
}

impl RTreeObject for dim3::Obb {
    type Envelope = AABB<NPoint<Vec3>>;

    fn envelope(&self) -> Self::Envelope {
        self.bounding_box().aabb
    }
}

/// The broad phase, using an R*-tree.
#[derive(Default, Debug, Clone)]
pub struct BroadPhase<T: RTreeObject> {
    rstar: RTree<T>,
}

impl<T: RTreeObject + Collider> BroadPhase<T>
where
    NPoint<T::Point>: Point,
{
    /// Create a new `BroadPhase` with some colliders.
    pub fn with_colliders(colliders: Vec<T>) -> Self {
        Self {
            rstar: RTree::bulk_load(colliders),
        }
    }

    /// Iterate through all pairs of shapes that are potentially colliding.
    ///
    /// Collisions with static objects are iterated over before semikinematic.
    pub fn iter(&self) -> impl Iterator<Item = (&T, &T)> + '_ {
        let statics = self
            .rstar
            .iter()
            .filter(|collider| collider.status() == Status::Static);
        let semiks = self
            .rstar
            .iter()
            .filter(|collider| collider.status() == Status::Semikinematic);
        statics
            .flat_map(move |collider1| {
                self.rstar
                    .locate_in_envelope_intersecting(&collider1.envelope())
                    .map(move |collider2| (collider1, collider2))
            })
            .chain(semiks.flat_map(move |collider1| {
                self.rstar
                    .locate_in_envelope_intersecting(&collider1.envelope())
                    .map(move |collider2| (collider1, collider2))
            }))
    }
}
