// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundPairCollisionTask.cs

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::{BoundsTestedPair, ICollisionCallbacks};
use crate::physics::collision_detection::collision_batcher_continuations::{
    CollisionContinuationType, ICollisionTestContinuation, PairContinuation,
};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::compound_pair_overlaps::{
    ChildOverlapsCollection, CompoundPairOverlaps, OverlapQueryForPair,
};

/// Trait for finding overlaps between two compound shapes.
pub trait ICompoundPairOverlapFinder {
    /// Finds all overlapping child pairs between compound shapes.
    ///
    /// # Safety
    /// Pair buffer and shape pointers must be valid.
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut CompoundPairOverlaps,
    );
}

/// Trait for handling continuations in compound-compound collision tasks.
///
/// Implementations create and configure continuation data for compound-compound child collisions,
/// handling both sides of the pair.
pub trait ICompoundPairContinuationHandler<TContinuation: ICollisionTestContinuation> {
    /// The type of collision continuation this handler produces.
    fn collision_continuation_type(&self) -> CollisionContinuationType;

    /// Creates a continuation for processing child manifolds from a compound pair.
    ///
    /// # Safety
    /// Caller must ensure batcher and buffer references are valid.
    unsafe fn create_continuation<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        total_child_count: i32,
        pair_overlaps: &mut Buffer<ChildOverlapsCollection>,
        pair_queries: &mut Buffer<OverlapQueryForPair>,
        pair: &BoundsTestedPair,
        continuation_index: &mut i32,
    ) -> *mut TContinuation;

    /// Gets shape data and pose for a child of compound A.
    ///
    /// # Safety
    /// Caller must ensure all pointers are valid.
    unsafe fn get_child_a_data<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation: *mut TContinuation,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        child_pose_a: &mut RigidPose,
        child_type_a: &mut i32,
        child_shape_data_a: &mut *const u8,
    );

    /// Configures a child of the continuation with shape data from compound B.
    ///
    /// # Safety
    /// Caller must ensure all pointers are valid.
    unsafe fn configure_continuation_child<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation: *mut TContinuation,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        child_index_a: i32,
        child_type_a: i32,
        child_index_b: i32,
        child_pose_a: &RigidPose,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    );
}

/// Collision task for compound-compound (or compound-mesh) shape pairs.
///
/// This task:
/// 1. Finds which children of both compounds overlap (via TOverlapFinder)
/// 2. Creates a continuation to accumulate child manifold results (via TContinuationHandler)
/// 3. Spawns individual child-child pair tests through the collision batcher
pub struct CompoundPairCollisionTask<
    TOverlapFinder: ICompoundPairOverlapFinder,
    TContinuationHandler: ICompoundPairContinuationHandler<TContinuation>,
    TContinuation: ICollisionTestContinuation,
> {
    pub batch_size: i32,
    pub shape_type_index_a: i32,
    pub shape_type_index_b: i32,
    pub subtask_generator: bool,
    _marker: std::marker::PhantomData<(TOverlapFinder, TContinuationHandler, TContinuation)>,
}

impl<
        TOverlapFinder: ICompoundPairOverlapFinder,
        TContinuationHandler: ICompoundPairContinuationHandler<TContinuation>,
        TContinuation: ICollisionTestContinuation,
    > CompoundPairCollisionTask<TOverlapFinder, TContinuationHandler, TContinuation>
{
    pub fn new(shape_type_a: i32, shape_type_b: i32) -> Self {
        Self {
            batch_size: 16,
            shape_type_index_a: shape_type_a,
            shape_type_index_b: shape_type_b,
            subtask_generator: true,
            _marker: std::marker::PhantomData,
        }
    }

    /// Executes a batch of compound-compound collision tests.
    ///
    /// For each pair:
    /// 1. Finds overlapping children from both compounds
    /// 2. Creates a continuation for collecting child-child results
    /// 3. Spawns child pair collision tests through the batcher
    ///
    /// # Safety
    /// All pointers in pairs and the batcher must be valid.
    pub unsafe fn execute_batch<TCallbacks: ICollisionCallbacks>(
        &self,
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation_handler: &TContinuationHandler,
    ) {
        let batcher_ref = &mut *batcher;
        let pool = &mut *batcher_ref.pool;
        let shapes = &*batcher_ref.shapes;

        // Find all overlaps up front.
        let mut overlaps = CompoundPairOverlaps::new(pool, 0, 0);
        TOverlapFinder::find_local_overlaps(
            pairs,
            pair_count,
            pool,
            shapes,
            batcher_ref.dt,
            &mut overlaps,
        );

        for pair_index in 0..pair_count {
            let (region_start, region_count) = overlaps.get_pair_region(pair_index);
            let pair = &pairs[pair_index];

            // Count total overlaps across all children of this pair.
            let mut total_overlap_count = 0;
            for j in 0..region_count {
                total_overlap_count += overlaps.get_overlaps_for_pair(region_start + j).count;
            }

            if total_overlap_count > 0 {
                debug_assert!(
                    total_overlap_count < PairContinuation::EXCLUSIVE_MAXIMUM_CHILD_INDEX as i32,
                    "Are there REALLY supposed to be that many overlaps?"
                );

                let mut continuation_index = 0i32;
                let continuation = continuation_handler.create_continuation(
                    batcher,
                    total_overlap_count,
                    &mut overlaps.child_overlaps,
                    &mut overlaps.pair_queries,
                    pair,
                    &mut continuation_index,
                );

                let mut next_continuation_child_index = 0;
                for j in 0..region_count {
                    let child_overlaps = overlaps.get_overlaps_for_pair(region_start + j);
                    if child_overlaps.count == 0 {
                        continue;
                    }

                    let mut child_pose_a = RigidPose::IDENTITY;
                    let mut child_type_a = 0i32;
                    let mut child_shape_data_a: *const u8 = std::ptr::null();
                    continuation_handler.get_child_a_data(
                        batcher,
                        continuation,
                        pair,
                        child_overlaps.child_index,
                        &mut child_pose_a,
                        &mut child_type_a,
                        &mut child_shape_data_a,
                    );

                    for k in 0..child_overlaps.count {
                        let original_child_index_b = child_overlaps.overlaps[k];

                        // Account for pair flipping.
                        let (child_a, child_b) = if pair.flip_mask < 0 {
                            (original_child_index_b, child_overlaps.child_index)
                        } else {
                            (child_overlaps.child_index, original_child_index_b)
                        };

                        let continuation_child_index = next_continuation_child_index;
                        next_continuation_child_index += 1;

                        if continuation_child_index
                            >= PairContinuation::EXCLUSIVE_MAXIMUM_CHILD_INDEX as i32
                        {
                            break;
                        }

                        let _subpair_continuation = PairContinuation::new(
                            pair.continuation.pair_id,
                            child_a,
                            child_b,
                            continuation_handler.collision_continuation_type(),
                            continuation_index,
                            continuation_child_index,
                        );

                        // TODO: Check AllowCollisionTesting callback, then configure and
                        // add the child pair to the batcher via AddDirectly.
                        let mut child_pose_b = RigidPose::IDENTITY;
                        let mut child_type_b = 0i32;
                        let mut child_shape_data_b: *const u8 = std::ptr::null();
                        continuation_handler.configure_continuation_child(
                            batcher,
                            continuation,
                            continuation_child_index,
                            pair,
                            child_overlaps.child_index,
                            child_type_a,
                            original_child_index_b,
                            &child_pose_a,
                            &mut child_pose_b,
                            &mut child_type_b,
                            &mut child_shape_data_b,
                        );

                        // TODO: Dispatch child pair via batcher.add_directly(...)
                    }
                }
            } else {
                // TODO: batcher.process_empty_result(pair.continuation)
            }
        }

        overlaps.dispose(pool);
        // Note: triangle lists are not disposed here — they're handed off to continuations.
    }
}
