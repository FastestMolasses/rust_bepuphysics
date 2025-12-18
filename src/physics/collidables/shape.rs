use glam::{Quat, Vec3};

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;

use super::ray::RayData;

/// Trait for ray hit handling during shape ray tests.
pub trait IShapeRayHitHandler {
    /// Returns true if the child at the given index should be tested.
    fn allow_test(&self, child_index: i32) -> bool;
    /// Called when a ray hits a shape.
    fn on_ray_hit(&mut self, ray: &RayData, maximum_t: &mut f32, t: f32, normal: Vec3, child_index: i32);
}

/// Defines a type usable as a shape by collidables.
pub trait IShape {
    /// Unique type id for this shape type.
    fn type_id() -> i32;
}

/// Defines functions available on all convex shapes.
/// Convex shapes have no hollowed out regions; any line passing through a convex shape
/// will never enter and exit more than once.
pub trait IConvexShape: IShape {
    /// Computes the bounding box of a shape given an orientation.
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3);

    /// Computes information about how the bounding box should be expanded in response to angular velocity.
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    );

    /// Computes the inertia for a body given a mass.
    fn compute_inertia(&self, mass: f32) -> BodyInertia;

    /// Tests a ray against the shape.
    fn ray_test(
        &self,
        pose: &RigidPose,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool;
}

/// Defines a shape that has internal resources that need explicit disposal.
pub trait IDisposableShape: IShape {
    /// Returns all resources used by the shape instance to the given pool.
    fn dispose(&mut self, pool: &mut BufferPool);
}

/// Widely vectorized bundle representation of a shape.
pub trait IShapeWide<TShape: IShape> {
    /// Gets whether this type supports accessing its memory by lane offsets.
    fn allow_offset_memory_access(&self) -> bool;

    /// Gets the number of bytes required for allocations within the wide shape.
    fn internal_allocation_size(&self) -> usize;

    /// Provides memory for internal allocations.
    fn initialize(&mut self, memory: &Buffer<u8>);

    /// Places the specified AOS-formatted shape into the first lane of the wide 'this' reference.
    fn write_first(&mut self, source: &TShape);

    /// Places the specified AOS-formatted shape into the selected slot.
    fn write_slot(&mut self, index: usize, source: &TShape);

    /// Broadcasts a scalar shape into a bundle containing the same shape in every lane.
    fn broadcast(&mut self, shape: &TShape);

    /// Computes the bounds of all shapes in the bundle.
    fn get_bounds(
        &self,
        orientations: &mut QuaternionWide,
        count_in_bundle: i32,
        maximum_radius: &mut Vector<f32>,
        maximum_angular_expansion: &mut Vector<f32>,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    );

    /// Gets the lower bound on the number of rays to execute in a wide fashion.
    fn minimum_wide_ray_count() -> i32;

    /// Tests a ray against the shape bundle.
    fn ray_test(
        &self,
        poses: &mut RigidPoseWide,
        ray_wide: &mut super::ray::RayWide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    );
}

/// Defines a support finder for GJK/MPR-based collision detection.
pub trait ISupportFinder<TShape: IConvexShape, TShapeWide: IShapeWide<TShape>> {
    /// Gets whether this shape type has a margin (e.g. spheres and capsules).
    fn has_margin(&self) -> bool;

    /// Gets the margin for the shape.
    fn get_margin(&self, shape: &TShapeWide, margin: &mut Vector<f32>);

    /// Computes the support point in world space.
    fn compute_support(
        &self,
        shape: &TShapeWide,
        orientation: &Matrix3x3Wide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    );

    /// Computes the support point in local space.
    fn compute_local_support(
        &self,
        shape: &TShapeWide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    );
}
