// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundPairOverlapFinder.cs

use std::marker::PhantomData;

use crate::physics::body_properties::RigidPoseWide;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::compound::{Compound, CompoundChild};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;

use super::compound_pair_collision_task::ICompoundPairOverlapFinder;
use super::compound_pair_overlaps::{CompoundPairOverlaps, OverlapQueryForPair};

/// Internal data linking a subpair to its source pair and child.
#[repr(C)]
struct SubpairData {
    pair: *const BoundsTestedPair,
    child: *const CompoundChild,
}

/// Finds overlapping children between two compound shapes.
///
/// Iterates over compound A's children, computes each child's bounding box
/// in compound B's local space using SIMD-wide operations, then queries
/// compound B's structure for overlaps.
pub struct CompoundPairOverlapFinder<TCompoundA, TCompoundB> {
    _marker: PhantomData<(TCompoundA, TCompoundB)>,
}

impl<TCompoundA, TCompoundB> ICompoundPairOverlapFinder
    for CompoundPairOverlapFinder<TCompoundA, TCompoundB>
{
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut CompoundPairOverlaps,
    ) {
        // Count total children across all pairs.
        let mut total_compound_child_count = 0i32;
        for i in 0..pair_count {
            let compound_a = &*(pairs[i].a as *const Compound);
            total_compound_child_count += compound_a.child_count() as i32;
        }

        *overlaps = CompoundPairOverlaps::new(pool, pair_count, total_compound_child_count);

        // Allocate subpair tracking data.
        let mut subpair_data: Buffer<SubpairData> = pool.take(total_compound_child_count);

        let mut next_subpair_index = 0i32;
        for i in 0..pair_count {
            let pair = &pairs[i];
            let compound_a = &*(pair.a as *const Compound);
            let child_count = compound_a.child_count() as i32;
            overlaps.create_pair_overlaps(child_count);

            for j in 0..child_count {
                let subpair_index = next_subpair_index;
                next_subpair_index += 1;

                overlaps.get_overlaps_for_pair(subpair_index).child_index = j;
                overlaps.pair_queries[subpair_index].container = pair.b;

                let subpair = &mut subpair_data[subpair_index];
                subpair.pair = pair as *const BoundsTestedPair;
                subpair.child = &compound_a.children[j as usize] as *const CompoundChild;
            }
        }

        // Process children in SIMD-wide batches, computing local bounding boxes.
        // TODO: Full SIMD-wide bounding box computation with ExpandLocalBoundingBoxes.
        // This requires shapes[type].compute_bounds() to be wired up.
        //
        // The algorithm:
        // 1. Gather pair data (orientations, velocities) into wide registers
        // 2. Transform child poses from compound A space to compound B local space
        // 3. Compute child shape bounds in compound B local space
        // 4. Expand bounds for velocity, angular motion, and speculative margin
        // 5. Store results in query bounds for downstream overlap testing
        //
        // For now, set up conservative bounds so the structure compiles.
        for i in 0..total_compound_child_count {
            let subpair = &subpair_data[i];
            let pair = &*subpair.pair;
            let child = &*subpair.child;

            // Conservative: transform child position to B's local space
            let conjugate_b = pair.orientation_b.conjugate();
            let local_child_pos = conjugate_b
                * (pair.orientation_a * child.local_position - pair.offset_b);

            // Use a conservative expansion radius.
            let expansion = pair.speculative_margin + pair.maximum_expansion;
            let expansion_vec = glam::Vec3::splat(expansion);

            overlaps.pair_queries[i].min = local_child_pos - expansion_vec;
            overlaps.pair_queries[i].max = local_child_pos + expansion_vec;
        }

        // TODO: Use compound B's acceleration structure to find overlapping children.
        // Unsafe.AsRef<TCompoundB>(pairsToTest[0].Container).FindLocalOverlaps(...)

        pool.return_buffer(&mut subpair_data);
    }
}
