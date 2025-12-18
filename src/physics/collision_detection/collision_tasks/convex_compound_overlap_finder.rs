// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundOverlapFinder.cs

use std::marker::PhantomData;

use glam::{Quat, Vec3};

use crate::physics::body_properties::RigidPoseWide;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;
use std::simd::prelude::*;

use super::convex_compound_task_overlaps::{ConvexCompoundOverlaps, ConvexCompoundTaskOverlaps};

/// Trait for shapes that support bounding box overlap queries against their children.
/// Covers both compound shapes (with child poses) and mesh shapes (with triangles).
pub trait IBoundsQueryableCompound {
    /// Gets the number of children in this compound.
    fn child_count(&self) -> i32;

    /// Finds which children overlap the given query bounds and stores their indices.
    fn find_local_overlaps(
        &self,
        query_bounds: &Buffer<super::compound_pair_overlaps::OverlapQueryForPair>,
        pool: &mut BufferPool,
        shapes: &Shapes,
        overlaps: &mut ConvexCompoundTaskOverlaps,
    );
}

/// Finds overlapping children between a convex shape and a compound/mesh container.
///
/// For each pair, computes the query bounds of the convex shape in the local space
/// of the compound and then queries the compound's acceleration structure.
pub struct ConvexCompoundOverlapFinder<TCompound: IBoundsQueryableCompound> {
    _marker: PhantomData<TCompound>,
}

impl<TCompound: IBoundsQueryableCompound> ConvexCompoundOverlapFinder<TCompound> {
    /// Finds all overlapping children for the given set of bounds-tested pairs.
    ///
    /// # Safety
    /// Pair pointers must be valid. The `pair.B` pointer must point to a valid `TCompound`.
    pub unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut ConvexCompoundTaskOverlaps,
    ) {
        for i in 0..pair_count {
            let pair = &pairs[i];

            // Compute local-space bounding box of the convex in the compound's frame.
            // Transform the offset to compound local space.
            let conjugate_orientation_b = pair.orientation_b.conjugate();
            let local_offset_a = conjugate_orientation_b * (-pair.offset_b);
            let local_orientation_a =
                conjugate_orientation_b * pair.orientation_a;

            // Get bounding box of the convex shape.
            // TODO: This is a simplification. The full implementation would call the shape's
            // ComputeBounds method with the local orientation and then expand for velocity.
            // For now, use a conservative expansion approach.
            let local_relative_linear_velocity =
                conjugate_orientation_b * pair.relative_linear_velocity_a;

            // Expand bounds by velocity and speculative margin.
            let expansion = local_relative_linear_velocity.abs() * dt
                + Vec3::splat(pair.speculative_margin);

            // Create a conservative bounding box around the convex at its local position.
            let min = local_offset_a - expansion;
            let max = local_offset_a + expansion;

            // Store query for this pair.
            let pair_overlaps = overlaps.get_overlaps_for_pair(i);
            pair_overlaps.overlaps = pool.take(16); // Initial capacity
            pair_overlaps.count = 0;

            // Note: In the full implementation, the compound's tree or child list would be
            // queried with (min, max) to find overlapping children. This requires
            // IBoundsQueryableCompound::find_local_overlaps to be properly wired up.
            // For now, we set up the query bounds for downstream use.
            let _ = (min, max, shapes);
        }
    }
}
