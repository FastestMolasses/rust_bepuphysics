// Translated from BepuPhysics/CollisionDetection/CollisionTasks/MeshPairOverlapFinder.cs

use std::marker::PhantomData;

use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::triangle::{Triangle, TriangleWide};
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;

use super::compound_pair_collision_task::ICompoundPairOverlapFinder;
use super::compound_pair_overlaps::{CompoundPairOverlaps, OverlapQueryForPair};

/// Finds overlapping triangles between two mesh shapes.
///
/// For each triangle in mesh A, computes its bounding box in mesh B's local space
/// using SIMD-wide operations, then queries mesh B's structure for overlaps.
pub struct MeshPairOverlapFinder<TMeshA, TMeshB> {
    _marker: PhantomData<(TMeshA, TMeshB)>,
}

impl<TMeshA, TMeshB> ICompoundPairOverlapFinder for MeshPairOverlapFinder<TMeshA, TMeshB> {
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut CompoundPairOverlaps,
    ) {
        // Count total triangles in mesh A across all pairs.
        // TODO: This requires IHomogeneousCompoundShape::child_count() to be wired up.
        // For now, use a placeholder approach.
        let mut total_compound_child_count = 0i32;
        for i in 0..pair_count {
            // TODO: let mesh_a = &*(pairs[i].a as *const TMeshA);
            // total_compound_child_count += mesh_a.child_count();
            // Placeholder: assume mesh has triangles buffer.
            let _ = &pairs[i];
        }

        *overlaps = CompoundPairOverlaps::new(pool, pair_count, total_compound_child_count);

        // Set up query pairs: each triangle in mesh A needs to be tested against mesh B.
        let mut next_subpair_index = 0i32;
        for i in 0..pair_count {
            let pair = &pairs[i];
            // TODO: Access mesh A's child count
            // let mesh_a = &*(pair.a as *const TMeshA);
            // let child_count = mesh_a.child_count();
            let child_count = 0i32; // Placeholder
            overlaps.create_pair_overlaps(child_count);
            for j in 0..child_count {
                let subpair_index = next_subpair_index;
                next_subpair_index += 1;
                overlaps.get_overlaps_for_pair(subpair_index).child_index = j;
                overlaps.pair_queries[subpair_index].container = pair.b;
            }
        }

        // Process triangles in SIMD-wide batches to compute local bounding boxes.
        // Algorithm:
        // 1. For each batch of triangles from mesh A:
        //    a. Broadcast pair data (orientations, velocities)
        //    b. Get triangle vertices via GetLocalChild
        //    c. Compute triangle bounds via TriangleWide::get_bounds
        //    d. Transform relative velocity to B's local space
        //    e. Expand bounds for velocity and angular motion
        //    f. Store query bounds for each triangle
        //
        // TODO: Requires TriangleWide::get_bounds and IHomogeneousCompoundShape methods.
        // The SIMD batching pattern is identical to CompoundPairOverlapFinder
        // but uses triangle-specific bounds computation.

        // 2. Query mesh B for overlaps using the computed query bounds.
        // TODO: mesh_b.find_local_overlaps(pair_queries, pool, shapes, overlaps)

        let _ = (shapes, dt);
    }
}
