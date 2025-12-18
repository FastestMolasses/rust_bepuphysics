// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundPairContinuations.cs

use std::marker::PhantomData;

use glam::Vec3;

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::compound::Compound;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::physics::collision_detection::collision_batcher_continuations::CollisionContinuationType;
use crate::physics::collision_detection::collision_task_registry::BatcherVtable;
use crate::physics::collision_detection::nonconvex_reduction::NonconvexReduction;
use crate::utilities::memory::buffer::Buffer;

use super::compound_pair_collision_task::ICompoundPairContinuationHandler;
use super::compound_pair_overlaps::{ChildOverlapsCollection, OverlapQueryForPair};

/// Continuation handler for compound-compound collisions using NonconvexReduction.
///
/// Creates NonconvexReduction continuations that accumulate child manifolds
/// from both compound shapes and reduce them into a single nonconvex manifold.
pub struct CompoundPairContinuations<TCompoundA, TCompoundB> {
    _marker: PhantomData<(TCompoundA, TCompoundB)>,
}

impl<TCompoundA, TCompoundB> Default for CompoundPairContinuations<TCompoundA, TCompoundB> {
    fn default() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<TCompoundA, TCompoundB> ICompoundPairContinuationHandler<NonconvexReduction>
    for CompoundPairContinuations<TCompoundA, TCompoundB>
{
    fn collision_continuation_type(&self) -> CollisionContinuationType {
        CollisionContinuationType::NonconvexReduction
    }

    unsafe fn create_continuation(
        &self,
        vtable: &BatcherVtable,
        total_child_count: i32,
        _pair_overlaps: &mut Buffer<ChildOverlapsCollection>,
        _pair_queries: &mut Buffer<OverlapQueryForPair>,
        _pair: &BoundsTestedPair,
        continuation_index: &mut i32,
    ) -> *mut NonconvexReduction {
        let pool = &mut *vtable.pool;
        let (index, continuation) = (&mut *vtable.nonconvex_reductions).create_continuation(total_child_count, pool);
        *continuation_index = index;
        continuation as *mut NonconvexReduction
    }

    unsafe fn get_child_a_data(
        &self,
        vtable: &BatcherVtable,
        _continuation: *mut NonconvexReduction,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        child_pose_a: &mut RigidPose,
        child_type_a: &mut i32,
        child_shape_data_a: &mut *const u8,
    ) {
        let compound_a = &*(pair.a as *const Compound);
        let compound_child_a = &compound_a.children[child_index_a as usize];

        Compound::get_rotated_child_pose(
            compound_child_a.local_position,
            compound_child_a.local_orientation,
            pair.orientation_a,
            &mut child_pose_a.position,
            &mut child_pose_a.orientation,
        );

        *child_type_a = compound_child_a.shape_index.type_id();
        let shapes = &*vtable.shapes;
        let batch = shapes.get_batch(*child_type_a as usize).expect("Shape batch must exist");
        let (shape_data, _) = batch.get_shape_data(compound_child_a.shape_index.index() as usize);
        *child_shape_data_a = shape_data;
    }

    unsafe fn configure_continuation_child(
        &self,
        vtable: &BatcherVtable,
        continuation: *mut NonconvexReduction,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        _child_index_a: i32,
        _child_type_a: i32,
        child_index_b: i32,
        child_pose_a: &RigidPose,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    ) {
        let cont = &mut *continuation;
        let continuation_child =
            &mut cont.children[continuation_child_index];

        let compound_b = &*(pair.b as *const Compound);
        let compound_child_b = &compound_b.children[child_index_b as usize];

        *child_type_b = compound_child_b.shape_index.type_id();
        let shapes = &*vtable.shapes;
        let batch = shapes.get_batch(*child_type_b as usize).expect("Shape batch must exist");
        let (shape_data, _) = batch.get_shape_data(compound_child_b.shape_index.index() as usize);
        *child_shape_data_b = shape_data;

        Compound::get_rotated_child_pose(
            compound_child_b.local_position,
            compound_child_b.local_orientation,
            pair.orientation_b,
            &mut child_pose_b.position,
            &mut child_pose_b.orientation,
        );

        if pair.flip_mask < 0 {
            continuation_child.child_index_a = child_index_b;
            continuation_child.child_index_b = _child_index_a;
            continuation_child.offset_a = child_pose_b.position;
            continuation_child.offset_b = child_pose_a.position;
        } else {
            continuation_child.child_index_a = _child_index_a;
            continuation_child.child_index_b = child_index_b;
            continuation_child.offset_a = child_pose_a.position;
            continuation_child.offset_b = child_pose_b.position;
        }
    }
}
