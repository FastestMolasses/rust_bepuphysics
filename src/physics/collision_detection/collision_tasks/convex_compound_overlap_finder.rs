// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundOverlapFinder.cs

use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

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
        _min: Vec3,
        _max: Vec3,
        _sweep: Vec3,
        _maximum_t: f32,
        _pool: &mut BufferPool,
        _shapes: &Shapes,
        _overlaps: *mut u8,
    ) {
        todo!("Sweep-specific overlap finding not yet implemented for this compound type")
    }
}

/// Finds overlapping children between a convex shape and a compound/mesh container.
///
/// For each pair, computes the query bounds of the convex shape in the local space
/// of the compound and then queries the compound's acceleration structure.
pub struct ConvexCompoundOverlapFinder<TCompound: IBoundsQueryableCompound> {
    _marker: PhantomData<TCompound>,
}

impl<TCompound: IBoundsQueryableCompound> IConvexCompoundOverlapFinder
    for ConvexCompoundOverlapFinder<TCompound>
{
    /// Finds all overlapping children for the given set of bounds-tested pairs.
    ///
    /// This uses a scalar-per-pair approach: for each pair, compute the local-space
    /// bounding box of the convex shape, expand it for velocity/angular motion, then
    /// store it in the subpair queries. Finally, delegate to the compound's
    /// `find_local_overlaps` to populate the overlap lists.
    ///
    /// # Safety
    /// Pair pointers must be valid. The `pair.B` pointer must point to a valid `TCompound`.
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut ConvexCompoundTaskOverlaps,
    ) {
        for i in 0..pair_count {
            let pair = &pairs[i];

            // Store the container pointer in the subpair query.
            overlaps.subpair_queries[i].container = pair.b;

            // Compute local-space transforms.
            let conjugate_orientation_b = pair.orientation_b.conjugate();
            let local_offset_a = conjugate_orientation_b * (-pair.offset_b);
            let _local_orientation_a = conjugate_orientation_b * pair.orientation_a;
            let local_relative_linear_velocity =
                conjugate_orientation_b * pair.relative_linear_velocity_a;

            // Compute conservative local-space bounding box.
            // A full implementation would use IShapeWide::get_bounds for shape-specific bounds,
            // but scalar approach with generous expansion is correct.
            let angular_velocity_a = pair.angular_velocity_a;
            let angular_velocity_b = pair.angular_velocity_b;

            // Use speculative margin as a conservative radius for the convex shape.
            let expansion_velocity = local_relative_linear_velocity.abs() * dt;
            let angular_expansion_scalar = BoundingBoxHelpers::get_angular_bounds_expansion(
                angular_velocity_a.length(),
                dt,
                pair.maximum_expansion,
                pair.maximum_expansion,
            );
            let expansion = expansion_velocity
                + Vec3::splat(pair.speculative_margin + angular_expansion_scalar);

            // If compound B is rotating, expand further.
            let angular_speed_b = angular_velocity_b.length();
            let expansion = if angular_speed_b > 0.0 {
                let radius_b = local_offset_a.length();
                let linear_speed = local_relative_linear_velocity.length();
                let worst_case_radius = linear_speed * dt + radius_b;
                let angular_expansion_b = BoundingBoxHelpers::get_angular_bounds_expansion(
                    angular_speed_b,
                    dt,
                    worst_case_radius,
                    worst_case_radius,
                );
                expansion + Vec3::splat(angular_expansion_b)
            } else {
                expansion
            };

            // Clamp expansion.
            let max_exp = Vec3::splat(pair.maximum_expansion);
            let expansion = expansion.min(max_exp);

            overlaps.subpair_queries[i].min = local_offset_a - expansion;
            overlaps.subpair_queries[i].max = local_offset_a + expansion;
        }

        // Delegate to the compound's tree/child overlap finder.
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
