// Translated from BepuPhysics/CollisionDetection/CollisionTasks/MeshPairContinuations.cs

use std::cell::Cell;
use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::shape::IHomogeneousCompoundShape;
use crate::physics::collidables::triangle::{Triangle, TriangleWide};
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::physics::collision_detection::collision_batcher_continuations::CollisionContinuationType;
use crate::physics::collision_detection::collision_task_registry::BatcherVtable;
use crate::physics::collision_detection::compound_mesh_reduction::CompoundMeshReduction;
use crate::physics::collision_detection::mesh_reduction::BoundingBox;
use crate::utilities::memory::buffer::Buffer;

use super::compound_pair_collision_task::ICompoundPairContinuationHandler;
use super::compound_pair_overlaps::{ChildOverlapsCollection, OverlapQueryForPair};

/// Continuation handler for mesh-mesh collisions using CompoundMeshReduction.
///
/// Creates CompoundMeshReduction continuations where both shape A and shape B
/// are meshes composed of triangles. Triangles from mesh A are stored in a
/// surplus area of the continuation's triangle buffer.
pub struct MeshPairContinuations<TMeshA, TMeshB> {
    /// Start index in the triangles buffer where mesh A's triangles are stored.
    /// Uses Cell for interior mutability since get_child_a_data takes &self but needs to mutate.
    triangle_a_start_index: Cell<i32>,
    _marker: PhantomData<(TMeshA, TMeshB)>,
}

impl<TMeshA, TMeshB> Default for MeshPairContinuations<TMeshA, TMeshB> {
    fn default() -> Self {
        Self {
            triangle_a_start_index: Cell::new(0),
            _marker: PhantomData,
        }
    }
}

impl<
        TMeshA: IHomogeneousCompoundShape<Triangle, TriangleWide>,
        TMeshB: IHomogeneousCompoundShape<Triangle, TriangleWide>,
    > ICompoundPairContinuationHandler<CompoundMeshReduction>
    for MeshPairContinuations<TMeshA, TMeshB>
{
    fn collision_continuation_type(&self) -> CollisionContinuationType {
        CollisionContinuationType::CompoundMeshReduction
    }

    unsafe fn create_continuation(
        &self,
        vtable: &BatcherVtable,
        total_child_count: i32,
        pair_overlaps: &mut Buffer<ChildOverlapsCollection>,
        pair_queries: &mut Buffer<OverlapQueryForPair>,
        pair: &BoundsTestedPair,
        continuation_index: &mut i32,
    ) -> *mut CompoundMeshReduction {
        let pool = &mut *vtable.pool;
        let (index, continuation) = (&mut *vtable.compound_mesh_reductions).create_continuation(total_child_count, pool);
        *continuation_index = index;

        // Store mesh A's triangles in surplus space beyond total_child_count.
        // This is a bit of a hack, but simple and cheap.
        self.triangle_a_start_index.set(total_child_count);
        continuation.triangles = pool.take(total_child_count + pair_overlaps.len() as i32);
        continuation.child_manifold_regions = pool.take(pair_overlaps.len());
        continuation.query_bounds = pool.take(pair_overlaps.len());
        continuation.region_count = pair_overlaps.len() as i32;
        continuation.mesh_orientation = pair.orientation_b;
        continuation.requires_flip = pair.flip_mask == 0;
        continuation.mesh = pair.b as *mut u8;

        // All regions must be assigned ahead of time.
        let mut next_child_index = 0i32;
        debug_assert!(pair_overlaps.len() == pair_queries.len());
        for j in 0..pair_overlaps.len() as i32 {
            let child_overlaps = &pair_overlaps[j];
            continuation.child_manifold_regions[j] = (next_child_index, child_overlaps.count);
            next_child_index += child_overlaps.count;
            continuation.query_bounds[j] = BoundingBox {
                min: pair_queries[j].min,
                max: pair_queries[j].max,
                ..Default::default()
            };
        }
        continuation as *mut CompoundMeshReduction
    }

    unsafe fn get_child_a_data(
        &self,
        _vtable: &BatcherVtable,
        continuation: *mut CompoundMeshReduction,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        child_pose_a: &mut RigidPose,
        child_type_a: &mut i32,
        child_shape_data_a: &mut *const u8,
    ) {
        let cont = &mut *continuation;
        let triangle_index = self.triangle_a_start_index.get();
        self.triangle_a_start_index.set(triangle_index + 1);
        let triangle = &mut cont.triangles[triangle_index];
        *child_shape_data_a = triangle as *const _ as *const u8;
        *child_type_a = Triangle::ID;
        let mesh_a = &*(pair.a as *const TMeshA);
        mesh_a.get_local_child(child_index_a, triangle);
        *child_pose_a = RigidPose::new(Vec3::ZERO, pair.orientation_a);
    }

    unsafe fn configure_continuation_child(
        &self,
        _vtable: &BatcherVtable,
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
        let cont = &mut *continuation;
        let triangle_ptr = &mut cont.triangles[continuation_child_index]
            as *mut crate::physics::collision_detection::mesh_reduction::Triangle;
        *child_shape_data_b = triangle_ptr as *const u8;
        *child_type_b = Triangle::ID;

        let mesh_b = &*(pair.b as *const TMeshB);
        mesh_b.get_local_child(child_index_b, &mut cont.triangles[continuation_child_index]);

        // In meshes, the triangle's vertices already contain the offset.
        *child_pose_b = RigidPose::new(Vec3::ZERO, pair.orientation_b);

        let continuation_child =
            &mut cont.inner.children[continuation_child_index];
        // Note: child pose offsets are default for mesh-mesh (vertices contain positions).
        continuation_child.offset_a = Vec3::ZERO;
        continuation_child.offset_b = Vec3::ZERO;
        if pair.flip_mask < 0 {
            continuation_child.child_index_a = child_index_b;
            continuation_child.child_index_b = child_index_a;
        } else {
            continuation_child.child_index_a = child_index_a;
            continuation_child.child_index_b = child_index_b;
        }
    }
}
