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
            0 => self.0.x(),
            1 => self.0.y(),
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        match index {
            0 => self.0.x_mut(),
            1 => self.0.y_mut(),
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
            0 => self.0.x(),
            1 => self.0.y(),
            2 => self.0.z(),
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        match index {
            0 => self.0.x_mut(),
            1 => self.0.y_mut(),
            2 => self.0.z_mut(),
            // unreachable according to the rstart 0.8 docs
            _ => unreachable!(),
        }
    }
}

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
    pub fn new(min: P, max: P) -> Self {
        Self {
            aabb: AABB::from_corners(NPoint::from(min), NPoint::from(max)),
        }
    }
}

pub trait Collider
where
    NPoint<Self::Point>: Point,
{
    type Point: PhysPoint;

    fn bounding_box(&self) -> BoundingBox<Self::Point>;
    fn status(&self) -> Status;
}

impl RTreeObject for dim2::Aabb {
    type Envelope = AABB<NPoint<Vec2>>;

    fn envelope(&self) -> Self::Envelope {
        self.bounding_box().aabb
    }
}

impl RTreeObject for dim3::Aabb {
    type Envelope = AABB<NPoint<Vec3>>;

    fn envelope(&self) -> Self::Envelope {
        self.bounding_box().aabb
    }
}

#[derive(Default, Debug, Clone)]
pub struct BroadPhase<T: RTreeObject> {
    rstar: RTree<T>,
}

impl<T: RTreeObject + Collider> BroadPhase<T>
where
    NPoint<T::Point>: Point,
{
    pub fn new() -> Self {
        Self {
            rstar: RTree::new(),
        }
    }

    pub fn with_colliders(colliders: Vec<T>) -> Self {
        Self {
            rstar: RTree::bulk_load(colliders),
        }
    }

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
