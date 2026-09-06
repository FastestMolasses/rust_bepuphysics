// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundOverlapFinder.cs

use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shape::{
    initialize_internal_allocation, IConvexShape, IShapeWide, IShapeWideAllocation,
    WideAllocationScratch,
};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

use super::convex_compound_collision_task::IConvexCompoundOverlapFinder;
use super::convex_compound_task_overlaps::ConvexCompoundTaskOverlaps;

/// Trait for shapes that support bounding box overlap queries against their children.
/// Covers both compound shapes (with child poses) and mesh shapes (with triangles).
pub trait IBoundsQueryableCompound {
    /// Gets the number of children in this compound.
    fn child_count(&self) -> i32;

    /// Finds which children overlap the given query bounds and stores their indices.
    fn find_local_overlaps<TOverlaps, TSubpairOverlaps>(
        &self,
        query_bounds: &Buffer<super::compound_pair_overlaps::OverlapQueryForPair>,
        pool: &mut BufferPool,
        shapes: &Shapes,
        overlaps: &mut TOverlaps,
    ) where
        TSubpairOverlaps: super::compound_pair_overlaps::ICollisionTaskSubpairOverlaps,
        TOverlaps: super::compound_pair_overlaps::ICollisionTaskOverlaps<TSubpairOverlaps>;

    /// Finds which children overlap the given sweep bounds (sweep-specific overload).
    ///
    /// # Safety
    /// `overlaps` must point to a valid overlaps collection of the appropriate type.
    unsafe fn find_local_overlaps_sweep(
        &self,
        min: Vec3,
        max: Vec3,
        sweep: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        overlaps: *mut u8,
    );
}

/// Finds overlapping children between a convex shape and a compound/mesh container.
pub struct ConvexCompoundOverlapFinder<TConvex, TConvexWide, TCompound>
where
    TConvex: IConvexShape,
    TConvexWide: IShapeWide<TConvex> + IShapeWideAllocation + Default,
    TCompound: IBoundsQueryableCompound,
{
    _marker: PhantomData<(TConvex, TConvexWide, TCompound)>,
}

impl<TConvex, TConvexWide, TCompound> IConvexCompoundOverlapFinder
    for ConvexCompoundOverlapFinder<TConvex, TConvexWide, TCompound>
where
    TConvex: IConvexShape,
    TConvexWide: IShapeWide<TConvex> + IShapeWideAllocation + Default,
    TCompound: IBoundsQueryableCompound,
{
    /// Finds all overlapping children for the given set of bounds-tested pairs.
    ///
    /// # Safety
    /// Each `pair.a` must point to a valid `TConvex` and each `pair.b` to a valid `TCompound`.
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut ConvexCompoundTaskOverlaps,
    ) {
        let lanes = Vector::<f32>::LEN as i32;

        let mut convex_wide = TConvexWide::default();
        // The allocation storage must outlive convex_wide.
        let mut alloc_scratch = std::mem::MaybeUninit::<WideAllocationScratch>::uninit();
        let _spilled =
            initialize_internal_allocation(&mut convex_wide, &mut alloc_scratch, pool as *mut _);

        let mut offset_b = Vector3Wide::default();
        let mut orientation_a = QuaternionWide::default();
        let mut orientation_b = QuaternionWide::default();
        let mut relative_linear_velocity_a = Vector3Wide::default();
        let mut angular_velocity_a = Vector3Wide::default();
        let mut angular_velocity_b = Vector3Wide::default();
        let mut maximum_allowed_expansion = Vector::<f32>::default();

        let mut i = 0i32;
        while i < pair_count {
            let mut count = pair_count - i;
            if count > lanes {
                count = lanes;
            }

            // Compute the local bounding boxes using wide operations for the expansion work.
            for j in 0..count as usize {
                let pair_index = i + j as i32;
                let pair = &pairs[pair_index];
                overlaps.subpair_queries[pair_index].container = pair.b;

                Vector3Wide::write_slot(pair.offset_b, j, &mut offset_b);
                QuaternionWide::write_slot(pair.orientation_a, j, &mut orientation_a);
                QuaternionWide::write_slot(pair.orientation_b, j, &mut orientation_b);
                Vector3Wide::write_slot(
                    pair.relative_linear_velocity_a,
                    j,
                    &mut relative_linear_velocity_a,
                );
                Vector3Wide::write_slot(pair.angular_velocity_a, j, &mut angular_velocity_a);
                Vector3Wide::write_slot(pair.angular_velocity_b, j, &mut angular_velocity_b);
                *(&mut maximum_allowed_expansion as *mut Vector<f32> as *mut f32).add(j) =
                    pair.maximum_expansion;

                let convex_shape = &*(pair.a as *const TConvex);
                convex_wide.write_slot(j, convex_shape);
            }

            let to_local_b = QuaternionWide::conjugate(&orientation_b);
            let mut local_offset_b = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(&offset_b, &to_local_b, &mut local_offset_b);
            let mut local_orientation_a = QuaternionWide::default();
            QuaternionWide::concatenate_without_overlap(
                &orientation_a,
                &to_local_b,
                &mut local_orientation_a,
            );
            let mut local_relative_linear_velocity_a = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(
                &relative_linear_velocity_a,
                &to_local_b,
                &mut local_relative_linear_velocity_a,
            );

            let mut maximum_radius = Vector::<f32>::default();
            let mut maximum_angular_expansion = Vector::<f32>::default();
            let mut min = Vector3Wide::default();
            let mut max = Vector3Wide::default();
            convex_wide.get_bounds(
                &mut local_orientation_a,
                count,
                &mut maximum_radius,
                &mut maximum_angular_expansion,
                &mut min,
                &mut max,
            );

            let mut local_position_a = Vector3Wide::default();
            Vector3Wide::negate(&local_offset_b, &mut local_position_a);
            BoundingBoxHelpers::expand_local_bounding_boxes(
                &mut min,
                &mut max,
                Vector::<f32>::splat(0.0),
                &local_position_a,
                &local_relative_linear_velocity_a,
                &angular_velocity_a,
                &angular_velocity_b,
                dt,
                maximum_radius,
                maximum_angular_expansion,
                maximum_allowed_expansion,
            );

            for j in 0..count as usize {
                let pair_to_test = &mut overlaps.subpair_queries[i + j as i32];
                Vector3Wide::read_slot(&min, j, &mut pair_to_test.min);
                Vector3Wide::read_slot(&max, j, &mut pair_to_test.max);
            }

            i += lanes;
        }

        // Use the compound's acceleration structure to find overlapping children.
        // The choice of instance here is irrelevant — all compounds of the same type
        // have the same tree structure query method.
        let compound_ptr: *const TCompound =
            overlaps.subpair_queries[0i32].container as *const TCompound;
        let compound = &*compound_ptr;
        // Copy the subpair_queries buffer to avoid aliasing with the mutable overlaps ref.
        let subpair_queries = overlaps.subpair_queries;
        compound.find_local_overlaps(&subpair_queries, pool, shapes, overlaps);
    }
}
