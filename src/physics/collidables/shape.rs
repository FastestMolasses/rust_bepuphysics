use glam::{Quat, Vec3};

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

// Re-export from canonical location.
pub use crate::physics::collision_detection::ray_batchers::{
    IShapeRayHitHandler, ShapeRayHitHandlerRef,
};
use crate::physics::trees::ray_batcher::RaySource;

/// Defines a type usable as a shape by collidables.
pub trait IShape {
    /// Unique type id for this shape type.
    fn type_id() -> i32;
}

/// Defines functions available on all convex shapes.
/// Convex shapes have no hollowed out regions; any line passing through a convex shape
/// will never enter and exit more than once.
pub trait IConvexShape: IShape + Sized {
    /// The wide (SIMD-bundled) representation of this shape type.
    type Wide: IShapeWide<Self> + IShapeWideAllocation + Default;

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
    /// Number of bytes this wide type requires for its internal allocation. Zero for most shapes.
    const INTERNAL_ALLOCATION_SIZE: usize = 0;

    /// Alignment the internal allocation must satisfy.
    const INTERNAL_ALLOCATION_ALIGN: usize = 1;

    /// Gets the number of bytes required for internal allocations. Returns 0 for most shapes.
    fn internal_allocation_size_of(&self) -> usize {
        Self::INTERNAL_ALLOCATION_SIZE
    }
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
    unsafe fn write_slot_raw(&mut self, index: usize, source: *const u8)
    where
        Self: Sized,
    {
        let vector_width = crate::utilities::vector::VECTOR_WIDTH;
        let scalar_float_count =
            std::mem::size_of::<Self>() / (vector_width * std::mem::size_of::<f32>());
        let src = source as *const f32;
        let dst = self as *mut Self as *mut f32;
        for i in 0..scalar_float_count {
            *dst.add(i * vector_width + index) = *src.add(i);
        }
    }
}

/// Alignment `WideAllocationScratch` guarantees for the memory it hands out.
pub const WIDE_ALLOCATION_SCRATCH_ALIGN: usize = 64;

/// Byte capacity of `WideAllocationScratch`, sized for the largest internal allocation any wide
/// shape type currently asks for.
pub const WIDE_ALLOCATION_SCRATCH_SIZE: usize = crate::utilities::vector::VECTOR_WIDTH
    * std::mem::size_of::<crate::physics::collidables::convex_hull::ConvexHull>();

/// Inline stack scratch backing `IShapeWideAllocation` internal allocations. Always held in a
/// `MaybeUninit`; the wide shape writes every lane it later reads.
#[repr(C, align(64))]
pub struct WideAllocationScratch([std::mem::MaybeUninit<u8>; WIDE_ALLOCATION_SCRATCH_SIZE]);

const _: () = assert!(
    std::mem::align_of::<WideAllocationScratch>() == WIDE_ALLOCATION_SCRATCH_ALIGN,
    "WIDE_ALLOCATION_SCRATCH_ALIGN must track the repr(align) on WideAllocationScratch"
);

/// Returns a spilled internal allocation to the pool it came from on every exit path, including
/// an unwind out of the loop that uses it.
pub struct SpilledAllocation {
    pub buffer: Buffer<u8>,
    pub pool: *mut BufferPool,
}

impl Drop for SpilledAllocation {
    #[inline(always)]
    fn drop(&mut self) {
        if self.buffer.allocated() {
            unsafe { (*self.pool).return_buffer(&mut self.buffer) };
        }
    }
}

/// Monomorphization-time proof that `WideAllocationScratch` can serve `TShapeWide`.
struct ScratchFits<TShapeWide: IShapeWideAllocation>(std::marker::PhantomData<TShapeWide>);

impl<TShapeWide: IShapeWideAllocation> ScratchFits<TShapeWide> {
    const CHECK: () = {
        assert!(
            TShapeWide::INTERNAL_ALLOCATION_SIZE <= WIDE_ALLOCATION_SCRATCH_SIZE,
            "wide shape internal allocation exceeds WIDE_ALLOCATION_SCRATCH_SIZE"
        );
        assert!(
            TShapeWide::INTERNAL_ALLOCATION_ALIGN <= WIDE_ALLOCATION_SCRATCH_ALIGN,
            "wide shape internal allocation alignment exceeds WIDE_ALLOCATION_SCRATCH_ALIGN"
        );
    };
}

/// Hands `wide` the internal allocation it asks for, taken from `scratch` when it fits and
/// otherwise from `pool`. `scratch` and the returned guard must outlive every use of `wide`.
///
/// A wide type whose declared requirement exceeds the scratch is a compile error rather than a
/// pool allocation on the hot path; the runtime spill only covers a `internal_allocation_size_of`
/// override that reports more than `INTERNAL_ALLOCATION_SIZE`.
///
/// # Safety
/// `pool` must stay valid for the lifetime of the returned guard.
#[inline(always)]
pub unsafe fn initialize_internal_allocation<TShapeWide: IShapeWideAllocation>(
    wide: &mut TShapeWide,
    scratch: &mut std::mem::MaybeUninit<WideAllocationScratch>,
    pool: *mut BufferPool,
) -> SpilledAllocation {
    let () = ScratchFits::<TShapeWide>::CHECK;
    let mut spilled = SpilledAllocation {
        buffer: Buffer::default(),
        pool,
    };
    let size = wide.internal_allocation_size_of();
    if size > 0 {
        let memory = if size <= WIDE_ALLOCATION_SCRATCH_SIZE {
            Buffer::new(scratch.as_mut_ptr() as *mut u8, size as i32, -1)
        } else {
            spilled.buffer = (*pool).take_at_least::<u8>(size as i32);
            Buffer::new(spilled.buffer.as_mut_ptr(), size as i32, spilled.buffer.id())
        };
        wide.initialize_allocation(&memory);
    }
    spilled
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

/// Trait for non-convex shapes that can compute their own bounding box from an orientation
/// and support ray testing. Both compound and homogeneous compound shapes implement this,
/// but the method signature does not require child type parameters.
pub trait INonConvexBounds {
    fn compute_bounds_by_orientation(
        &self,
        orientation: glam::Quat,
        min: &mut glam::Vec3,
        max: &mut glam::Vec3,
    );

    /// Identifies this shape for `Shapes::ray_test`'s monomorphized dispatch.
    fn shape_ray_ref(&self) -> super::shapes::ShapeRayRef;

    /// Tests a ray against this shape with an erased hit handler.
    ///
    /// # Safety
    /// Caller must ensure shape data and pose are valid.
    unsafe fn ray_test_shape(
        &self,
        pose: &RigidPose,
        ray: &crate::physics::collision_detection::ray_batchers::RayData,
        maximum_t: &mut f32,
        pool: &mut BufferPool,
        hit_handler: &mut ShapeRayHitHandlerRef<'_>,
    );

    /// Tests a batch of rays against this shape with an erased hit handler.
    ///
    /// # Safety
    /// Caller must ensure shape data, rays and pool are valid.
    unsafe fn ray_test_shape_batched(
        &self,
        pose: &RigidPose,
        rays: &mut RaySource,
        pool: &mut BufferPool,
        hit_handler: &mut ShapeRayHitHandlerRef<'_>,
    );
}

/// Defines a compound shape type that has children of potentially different types.
pub trait ICompoundShape: IDisposableShape {
    /// Gets the number of children in the compound shape.
    fn child_count(&self) -> i32;

    /// Gets a child from the compound by index.
    fn get_child(&self, child_index: i32) -> &CompoundChild;

    /// Computes the bounding box of the compound shape.
    fn compute_bounds(
        &self,
        orientation: glam::Quat,
        shape_batches: &super::shapes::Shapes,
        min: &mut glam::Vec3,
        max: &mut glam::Vec3,
    );

    /// Finds overlapping children within a local bounding box.
    fn find_local_overlaps<TOverlaps: super::compound::IOverlapCollector>(
        &self,
        local_min: &glam::Vec3,
        local_max: &glam::Vec3,
        pool: &mut BufferPool,
        shapes: &super::shapes::Shapes,
        overlaps: &mut TOverlaps,
    );

    /// Adds child bounding boxes to the batcher for compound shape bounds computation.
    fn add_child_bounds_to_batcher(
        &self,
        batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher,
        pose: &crate::physics::body_properties::RigidPose,
        velocity: &crate::physics::body_properties::BodyVelocity,
        body_index: i32,
    );

    /// Identifies this shape for `Shapes::ray_test`'s monomorphized dispatch.
    fn shape_ray_ref(&self) -> super::shapes::ShapeRayRef;

    /// Tests a ray against this compound shape with an erased hit handler.
    ///
    /// # Safety
    /// Caller must ensure shape data and pose are valid.
    unsafe fn ray_test_shape(
        &self,
        pose: &RigidPose,
        ray: &crate::physics::collision_detection::ray_batchers::RayData,
        maximum_t: &mut f32,
        shape_batches: &super::shapes::Shapes,
        pool: &mut BufferPool,
        hit_handler: &mut ShapeRayHitHandlerRef<'_>,
    );

    /// Tests a batch of rays against this compound shape with an erased hit handler.
    ///
    /// # Safety
    /// Caller must ensure shape data, rays and pool are valid.
    unsafe fn ray_test_shape_batched(
        &self,
        pose: &RigidPose,
        rays: &mut RaySource,
        shape_batches: &super::shapes::Shapes,
        pool: &mut BufferPool,
        hit_handler: &mut ShapeRayHitHandlerRef<'_>,
    );
}

/// Defines a compound shape type that has children of only one type.
pub trait IHomogeneousCompoundShape<
    TChildShape: IConvexShape,
    TChildShapeWide: IShapeWide<TChildShape>,
>: IDisposableShape
{
    /// Gets the number of children in the compound shape.
    fn child_count(&self) -> i32;

    /// Gets a child shape as it appears in the compound's local space.
    fn get_local_child(&self, child_index: i32, child_data: &mut TChildShape);

    /// Gets a local child and writes it into the first slot of a wide instance.
    /// Used with `GatherScatter::get_offset_instance_mut` to fill specific SIMD lanes.
    fn get_local_child_wide(&self, child_index: i32, target: &mut TChildShapeWide);

    /// Gets a child shape and its pose in the compound's local space.
    fn get_posed_local_child(
        &self,
        child_index: i32,
        child_data: &mut TChildShape,
        child_pose: &mut RigidPose,
    );

    /// Computes the bounding box of the compound shape.
    fn compute_bounds(&self, orientation: glam::Quat, min: &mut glam::Vec3, max: &mut glam::Vec3);

    /// Finds overlapping children within a local bounding box.
    fn find_local_overlaps<TOverlaps: super::compound::IOverlapCollector>(
        &self,
        local_min: &glam::Vec3,
        local_max: &glam::Vec3,
        pool: &mut BufferPool,
        overlaps: &mut TOverlaps,
    );
}
