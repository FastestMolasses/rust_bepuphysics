// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexMeshContinuations.cs

use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::triangle::Triangle;
use crate::physics::collision_detection::collision_batcher::{BoundsTestedPair, ICollisionCallbacks};
use crate::physics::collision_detection::collision_batcher_continuations::CollisionContinuationType;
use crate::physics::collision_detection::mesh_reduction::MeshReduction;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::compound_pair_overlaps::OverlapQueryForPair;
use super::convex_compound_collision_task::IConvexCompoundContinuationHandler;

/// Continuation handler for convex-mesh collisions using MeshReduction.
///
/// Creates MeshReduction continuations that handle mesh boundary smoothing
/// by storing triangle data and query bounds for post-processing.
pub struct ConvexMeshContinuations<TMesh> {
    _marker: PhantomData<TMesh>,
}

impl<TMesh> Default for ConvexMeshContinuations<TMesh> {
    fn default() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<TMesh> IConvexCompoundContinuationHandler<MeshReduction>
    for ConvexMeshContinuations<TMesh>
{
    fn collision_continuation_type(&self) -> CollisionContinuationType {
        CollisionContinuationType::MeshReduction
    }

    unsafe fn create_continuation<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        child_count: i32,
        pair: &BoundsTestedPair,
        query_for_pair: &OverlapQueryForPair,
        continuation_index: &mut i32,
    ) -> *mut MeshReduction {
        // TODO: Once CollisionBatcher has MeshReductions field:
        // let continuation = (*batcher).mesh_reductions.create_continuation(
        //     child_count, &mut *(*batcher).pool, continuation_index
        // );
        // Pass ownership of the triangles buffer to the continuation.
        // let pool = &mut *(*batcher).pool;
        // (*continuation).triangles = pool.take(child_count);
        // (*continuation).mesh_orientation = pair.orientation_b;
        // // A flip is required whenever contacts are generated as if the triangle is in slot B,
        // // which is whenever this pair has *not* been flipped.
        // (*continuation).requires_flip = pair.flip_mask == 0;
        // (*continuation).query_bounds.min = query_for_pair.min;
        // (*continuation).query_bounds.max = query_for_pair.max;
        // // TODO: Not flexible with respect to different mesh types.
        // (*continuation).mesh = query_for_pair.container as *mut u8;
        // continuation
        let _ = (batcher, child_count, pair, query_for_pair, continuation_index);
        std::ptr::null_mut()
    }

    unsafe fn configure_continuation_child<TCallbacks: ICollisionCallbacks>(
        &self,
        _batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation: *mut MeshReduction,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        _shape_type_a: i32,
        child_index: i32,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    ) {
        // The triangles list persists until the continuation completes,
        // so we can pass a pointer to avoid additional shape copying.
        let cont = &mut *continuation;
        let triangle_ptr = &mut cont.triangles[continuation_child_index] as *mut crate::physics::collision_detection::mesh_reduction::Triangle;
        *child_shape_data_b = triangle_ptr as *const u8;
        *child_type_b = Triangle::ID;

        // TODO: Get the local child from the mesh:
        // let mesh = &*(pair.b as *const TMesh);
        // mesh.get_local_child(child_index, &mut (*continuation).triangles[continuation_child_index]);

        // Triangles already have their local pose baked into their vertices,
        // so we just need the orientation.
        *child_pose_b = RigidPose::new(Vec3::ZERO, pair.orientation_b);

        let continuation_child =
            &mut cont.inner.children[continuation_child_index];
        continuation_child.offset_a = Vec3::ZERO;
        continuation_child.offset_b = Vec3::ZERO;
        if pair.flip_mask < 0 {
            continuation_child.child_index_a = child_index;
            continuation_child.child_index_b = 0;
        } else {
            continuation_child.child_index_a = 0;
            continuation_child.child_index_b = child_index;
        }
    }
}
