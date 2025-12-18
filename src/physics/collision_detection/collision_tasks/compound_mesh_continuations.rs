// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundMeshContinuations.cs

use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::compound::Compound;
use crate::physics::collidables::triangle::Triangle;
use crate::physics::collision_detection::collision_batcher::{BoundsTestedPair, ICollisionCallbacks};
use crate::physics::collision_detection::collision_batcher_continuations::CollisionContinuationType;
use crate::physics::collision_detection::compound_mesh_reduction::CompoundMeshReduction;
use crate::physics::collision_detection::mesh_reduction::BoundingBox;
use crate::utilities::memory::buffer::Buffer;

use super::compound_pair_collision_task::ICompoundPairContinuationHandler;
use super::compound_pair_overlaps::{ChildOverlapsCollection, OverlapQueryForPair};

/// Continuation handler for compound-mesh collisions using CompoundMeshReduction.
///
/// Creates CompoundMeshReduction continuations that handle mesh boundary smoothing
/// across multiple children of the compound shape, each of which may have overlaps
/// with triangles of the mesh.
pub struct CompoundMeshContinuations<TCompound, TMesh> {
    _marker: PhantomData<(TCompound, TMesh)>,
}

impl<TCompound, TMesh> Default for CompoundMeshContinuations<TCompound, TMesh> {
    fn default() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<TCompound, TMesh> ICompoundPairContinuationHandler<CompoundMeshReduction>
    for CompoundMeshContinuations<TCompound, TMesh>
{
    fn collision_continuation_type(&self) -> CollisionContinuationType {
        CollisionContinuationType::CompoundMeshReduction
    }

    unsafe fn create_continuation<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        total_child_count: i32,
        pair_overlaps: &mut Buffer<ChildOverlapsCollection>,
        pair_queries: &mut Buffer<OverlapQueryForPair>,
        pair: &BoundsTestedPair,
        continuation_index: &mut i32,
    ) -> *mut CompoundMeshReduction {
        // TODO: Once CollisionBatcher has CompoundMeshReductions field:
        // let continuation = (*batcher).compound_mesh_reductions.create_continuation(
        //     total_child_count, &mut *(*batcher).pool, continuation_index
        // );
        // let pool = &mut *(*batcher).pool;
        //
        // // Pass ownership of triangle and region buffers to the continuation.
        // (*continuation).triangles = pool.take(total_child_count);
        // (*continuation).child_manifold_regions = pool.take(pair_overlaps.len());
        // (*continuation).query_bounds = pool.take(pair_overlaps.len());
        // (*continuation).region_count = pair_overlaps.len();
        // (*continuation).mesh_orientation = pair.orientation_b;
        // (*continuation).mesh = pair.b as *mut u8;
        // (*continuation).requires_flip = pair.flip_mask == 0;
        //
        // // All regions must be assigned ahead of time. Some trailing regions may be empty,
        // // so the dispatch may occur before all children are visited in the later loop.
        // let mut next_child_index = 0i32;
        // debug_assert!(pair_overlaps.len() == pair_queries.len());
        // for j in 0..pair_overlaps.len() {
        //     let child_overlaps = &pair_overlaps[j];
        //     (*continuation).child_manifold_regions[j] = (next_child_index, child_overlaps.count);
        //     next_child_index += child_overlaps.count;
        //     (*continuation).query_bounds[j] = BoundingBox {
        //         min: pair_queries[j].min,
        //         max: pair_queries[j].max,
        //         ..Default::default()
        //     };
        // }
        // continuation

        let _ = (
            batcher,
            total_child_count,
            pair_overlaps,
            pair_queries,
            pair,
            continuation_index,
        );
        std::ptr::null_mut()
    }

    unsafe fn get_child_a_data<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        _continuation: *mut CompoundMeshReduction,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        child_pose_a: &mut RigidPose,
        child_type_a: &mut i32,
        child_shape_data_a: &mut *const u8,
    ) {
        let compound = &*(pair.a as *const Compound);
        let compound_child = &compound.children[child_index_a as usize];

        Compound::get_rotated_child_pose(
            compound_child.local_position,
            compound_child.local_orientation,
            pair.orientation_a,
            &mut child_pose_a.position,
            &mut child_pose_a.orientation,
        );

        *child_type_a = compound_child.shape_index.type_id();
        // TODO: Access shape data through shapes batch registry once wired up.
        *child_shape_data_a = std::ptr::null();
    }

    unsafe fn configure_continuation_child<TCallbacks: ICollisionCallbacks>(
        &self,
        _batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation: *mut CompoundMeshReduction,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        _child_type_a: i32,
        child_index_b: i32,
        child_pose_a: &RigidPose,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    ) {
        // The triangles list persists until the continuation completes,
        // so we can pass a pointer to avoid additional shape copying.
        let cont = &mut *continuation;
        let triangle_ptr = &mut cont.triangles[continuation_child_index]
            as *mut crate::physics::collision_detection::mesh_reduction::Triangle;
        *child_shape_data_b = triangle_ptr as *const u8;
        *child_type_b = Triangle::ID;

        // TODO: Get the local child triangle from the mesh:
        // let mesh = &*(pair.b as *const TMesh);
        // mesh.get_local_child(child_index_b, &mut (*continuation).triangles[continuation_child_index]);

        // In meshes, the triangle's vertices already contain the offset,
        // so there is no additional offset — just the orientation.
        *child_pose_b = RigidPose::new(Vec3::ZERO, pair.orientation_b);

        let continuation_child =
            &mut cont.inner.children[continuation_child_index];
        if pair.flip_mask < 0 {
            continuation_child.child_index_a = child_index_b;
            continuation_child.child_index_b = child_index_a;
            continuation_child.offset_a = child_pose_b.position;
            continuation_child.offset_b = child_pose_a.position;
        } else {
            continuation_child.child_index_a = child_index_a;
            continuation_child.child_index_b = child_index_b;
            continuation_child.offset_a = child_pose_a.position;
            continuation_child.offset_b = child_pose_b.position;
        }
    }
}
