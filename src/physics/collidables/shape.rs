use glam::{Quat, Vec3};

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;

// Re-export from canonical location.
pub use crate::physics::collision_detection::ray_batchers::{IShapeRayHitHandler, RayData};

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

/// Internal allocation interface for shape wide types that may need dynamic memory.
/// Separated from `IShapeWide<TShape>` so it can be constrained without knowing TShape.
pub trait IShapeWideAllocation {
    /// Gets the number of bytes required for internal allocations. Returns 0 for most shapes.
    fn internal_allocation_size_of(&self) -> usize { 0 }
    /// Provides memory for internal allocations. No-op for shapes with zero allocation size.
    fn initialize_allocation(&mut self, _memory: &Buffer<u8>) {}

    /// Writes a scalar shape from a raw pointer into the specified SIMD lane.
    ///
    /// The default implementation assumes the wide type is composed entirely of `Vector<f32>`
    /// fields (valid for Sphere, Capsule, Box, Triangle, Cylinder). Types with non-uniform
    /// layouts (e.g. ConvexHullWide) must override.
    ///
    /// # Safety
    /// `source` must point to a valid instance of the corresponding scalar shape type.
    unsafe fn write_slot_raw(&mut self, index: usize, source: *const u8) where Self: Sized {
        let vector_width = crate::utilities::vector::VECTOR_WIDTH;
        let scalar_float_count = std::mem::size_of::<Self>() / (vector_width * std::mem::size_of::<f32>());
        let src = source as *const f32;
        let dst = self as *mut Self as *mut f32;
        for i in 0..scalar_float_count {
            *dst.add(i * vector_width + index) = *src.add(i);
        }
    }
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

// Forward declarations for compound shape traits.
use crate::physics::collidables::compound::CompoundChild;

/// Defines a compound shape type that has children of potentially different types.
pub trait ICompoundShape: IDisposableShape {
    /// Gets the number of children in the compound shape.
    fn child_count(&self) -> i32;

    /// Gets a child from the compound by index.
    fn get_child(&self, child_index: i32) -> &CompoundChild;

    /// Computes the bounding box of the compound shape.
    fn compute_bounds(&self, orientation: glam::Quat, min: &mut glam::Vec3, max: &mut glam::Vec3);

    /// Finds overlapping children within a local bounding box.
    fn find_local_overlaps<TOverlaps: super::compound::IOverlapCollector>(
        &self,
        local_min: &glam::Vec3,
        local_max: &glam::Vec3,
        overlaps: &mut TOverlaps,
    );
}

/// Defines a compound shape type that has children of only one type.
pub trait IHomogeneousCompoundShape<TChildShape: IConvexShape, TChildShapeWide: IShapeWide<TChildShape>>:
    IDisposableShape
{
    /// Gets the number of children in the compound shape.
    fn child_count(&self) -> i32;

    /// Gets a child shape as it appears in the compound's local space.
    fn get_local_child(&self, child_index: i32, child_data: &mut TChildShape);

    /// Gets a local child and writes it into the first slot of a wide instance.
    /// Used with `GatherScatter::get_offset_instance_mut` to fill specific SIMD lanes.
    fn get_local_child_wide(&self, child_index: i32, target: &mut TChildShapeWide);

    /// Gets a child shape and its pose in the compound's local space.
    fn get_posed_local_child(&self, child_index: i32, child_data: &mut TChildShape, child_pose: &mut RigidPose);

    /// Computes the bounding box of the compound shape.
    fn compute_bounds(&self, orientation: glam::Quat, min: &mut glam::Vec3, max: &mut glam::Vec3);

    /// Finds overlapping children within a local bounding box.
    fn find_local_overlaps<TOverlaps: super::compound::IOverlapCollector>(
        &self,
        local_min: &glam::Vec3,
        local_max: &glam::Vec3,
        overlaps: &mut TOverlaps,
    );
}
