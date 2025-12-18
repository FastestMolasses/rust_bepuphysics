// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundContinuations.cs

use std::marker::PhantomData;

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::compound::Compound;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::physics::collision_detection::collision_batcher_continuations::CollisionContinuationType;
use crate::physics::collision_detection::collision_task_registry::BatcherVtable;
use crate::physics::collision_detection::nonconvex_reduction::NonconvexReduction;

use super::compound_pair_overlaps::OverlapQueryForPair;
use super::convex_compound_collision_task::IConvexCompoundContinuationHandler;

/// Continuation handler for convex-compound collisions using NonconvexReduction.
///
/// Creates NonconvexReduction continuations to accumulate child manifolds
/// from a compound shape and reduce them into a single nonconvex manifold.
pub struct ConvexCompoundContinuations<TCompound> {
    _marker: PhantomData<TCompound>,
}

impl<TCompound> Default for ConvexCompoundContinuations<TCompound> {
    fn default() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<TCompound> IConvexCompoundContinuationHandler<NonconvexReduction>
    for ConvexCompoundContinuations<TCompound>
{
    fn collision_continuation_type(&self) -> CollisionContinuationType {
        CollisionContinuationType::NonconvexReduction
    }

    unsafe fn create_continuation(
        &self,
        vtable: &BatcherVtable,
        child_count: i32,
        _pair: &BoundsTestedPair,
        _query_for_pair: &OverlapQueryForPair,
        continuation_index: &mut i32,
    ) -> *mut NonconvexReduction {
        let pool = &mut *vtable.pool;
        let (index, continuation) = (&mut *vtable.nonconvex_reductions).create_continuation(child_count, pool);
        *continuation_index = index;
        continuation as *mut NonconvexReduction
    }

    unsafe fn configure_continuation_child(
        &self,
        vtable: &BatcherVtable,
        continuation: *mut NonconvexReduction,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        _shape_type_a: i32,
        child_index: i32,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    ) {
        // Get the compound child and compute its world-space pose.
        let compound = &*(pair.b as *const Compound);
        let compound_child = &compound.children[child_index as usize];

        // Compute the rotated child pose in world space.
        Compound::get_rotated_child_pose(
            compound_child.local_position,
            compound_child.local_orientation,
            pair.orientation_b,
            &mut child_pose_b.position,
            &mut child_pose_b.orientation,
        );

        // Get the shape data for the child.
        *child_type_b = compound_child.shape_index.type_id();
        let shapes = &*vtable.shapes;
        let batch = shapes.get_batch(*child_type_b as usize).expect("Shape batch must exist");
        let (shape_data, _) = batch.get_shape_data(compound_child.shape_index.index() as usize);
        *child_shape_data_b = shape_data;

        // Configure the continuation child with offset and index data.
        let cont = &mut *continuation;
        let continuation_child = &mut cont.children[continuation_child_index];
        if pair.flip_mask < 0 {
            continuation_child.child_index_a = child_index;
            continuation_child.child_index_b = 0;
            continuation_child.offset_a = child_pose_b.position;
            continuation_child.offset_b = glam::Vec3::ZERO;
        } else {
            continuation_child.child_index_a = 0;
            continuation_child.child_index_b = child_index;
            continuation_child.offset_a = glam::Vec3::ZERO;
            continuation_child.offset_b = child_pose_b.position;
        }
    }
}
