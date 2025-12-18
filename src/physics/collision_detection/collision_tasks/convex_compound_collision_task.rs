// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundCollisionTask.cs

use crate::physics::body_properties::RigidPose;
use crate::physics::collision_detection::collision_batcher::{BoundsTestedPair, ICollisionCallbacks};
use crate::physics::collision_detection::collision_batcher_continuations::{
    CollisionContinuationType, ICollisionTestContinuation, PairContinuation,
};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::compound_pair_overlaps::OverlapQueryForPair;
use super::convex_compound_task_overlaps::ConvexCompoundTaskOverlaps;

/// Trait for handling continuations in convex-compound collision tasks.
///
/// Implementations create and configure continuation data for compound child collisions.
pub trait IConvexCompoundContinuationHandler<TContinuation: ICollisionTestContinuation> {
    /// The type of collision continuation this handler produces.
    fn collision_continuation_type(&self) -> CollisionContinuationType;

    /// Creates a continuation for processing child manifolds.
    ///
    /// # Safety
    /// Caller must ensure batcher and pair references are valid.
    unsafe fn create_continuation<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        child_count: i32,
        pair: &BoundsTestedPair,
        query_for_pair: &OverlapQueryForPair,
        continuation_index: &mut i32,
    ) -> *mut TContinuation;

    /// Configures a child of the continuation with the compound child's shape data and pose.
    ///
    /// # Safety
    /// Caller must ensure all pointers are valid.
    unsafe fn configure_continuation_child<TCallbacks: ICollisionCallbacks>(
        &self,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        continuation: *mut TContinuation,
        continuation_child_index: i32,
        pair: &BoundsTestedPair,
        shape_type_a: i32,
        child_index: i32,
        child_pose_b: &mut RigidPose,
        child_type_b: &mut i32,
        child_shape_data_b: &mut *const u8,
    );
}

/// Trait for finding overlaps between a convex shape and a compound shape.
pub trait IConvexCompoundOverlapFinder {
    /// Finds local overlaps for all pairs in the batch.
    ///
    /// # Safety
    /// Pair buffer pointers must be valid.
    unsafe fn find_local_overlaps(
        &self,
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: *mut crate::physics::collidables::shapes::Shapes,
        dt: f32,
        overlaps: &mut ConvexCompoundTaskOverlaps,
    );
}

/// Collision task for convex vs compound/mesh shape pairs.
///
/// This task:
/// 1. Finds which children of the compound overlap the convex shape (via TOverlapFinder)
/// 2. Creates a continuation to accumulate child manifold results (via TContinuationHandler)
/// 3. Spawns individual convex-child pair tests through the collision batcher
///
/// # Type Parameters
/// - `TConvex`: The convex shape type
/// - `TCompound`: The compound/mesh shape type
/// - `TOverlapFinder`: Finds which compound children overlap the convex
/// - `TContinuationHandler`: Creates and manages the continuation
/// - `TContinuation`: The continuation type that accumulates results
pub struct ConvexCompoundCollisionTask<
    TOverlapFinder: IConvexCompoundOverlapFinder,
    TContinuationHandler: IConvexCompoundContinuationHandler<TContinuation>,
    TContinuation: ICollisionTestContinuation,
> {
    pub batch_size: i32,
    pub shape_type_index_a: i32,
    pub shape_type_index_b: i32,
    pub subtask_generator: bool,
    _marker: std::marker::PhantomData<(TOverlapFinder, TContinuationHandler, TContinuation)>,
}

impl<
        TOverlapFinder: IConvexCompoundOverlapFinder,
        TContinuationHandler: IConvexCompoundContinuationHandler<TContinuation>,
        TContinuation: ICollisionTestContinuation,
    > ConvexCompoundCollisionTask<TOverlapFinder, TContinuationHandler, TContinuation>
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

    /// Executes a batch of convex-compound collision tests.
    ///
    /// For each pair:
    /// 1. Finds overlapping compound children
    /// 2. Creates a continuation for collecting child results
    /// 3. Spawns child collision tests through the batcher
    ///
    /// # Safety
    /// All pointers in pairs and the batcher must be valid.
    pub unsafe fn execute_batch<TCallbacks: ICollisionCallbacks>(
        &self,
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        batcher: *mut crate::physics::collision_detection::collision_batcher::CollisionBatcher<TCallbacks>,
        overlap_finder: &TOverlapFinder,
        continuation_handler: &TContinuationHandler,
    ) {
        let batcher_ref = &mut *batcher;
        let pool = &mut *batcher_ref.pool;

        // Find all overlaps up front to improve cache behavior.
        let mut overlaps = ConvexCompoundTaskOverlaps::new(pool, pair_count);
        overlap_finder.find_local_overlaps(
            pairs,
            pair_count,
            pool,
            batcher_ref.shapes,
            batcher_ref.dt,
            &mut overlaps,
        );

        for i in 0..pair_count {
            let pair_overlaps = overlaps.get_overlaps_for_pair(i);
            let pair = &pairs[i];

            if pair_overlaps.count > 0 {
                let mut overlap_count = pair_overlaps.count;
                // Clamp to maximum representable child index.
                debug_assert!(
                    overlap_count < PairContinuation::EXCLUSIVE_MAXIMUM_CHILD_INDEX as i32,
                    "Are there REALLY supposed to be that many overlaps?"
                );
                if overlap_count >= PairContinuation::EXCLUSIVE_MAXIMUM_CHILD_INDEX as i32 {
                    overlap_count = PairContinuation::EXCLUSIVE_MAXIMUM_CHILD_INDEX as i32 - 1;
                }

                let mut continuation_index = 0i32;
                let continuation = continuation_handler.create_continuation(
                    batcher,
                    overlap_count,
                    pair,
                    &OverlapQueryForPair::default(),
                    &mut continuation_index,
                );

                let mut next_continuation_child_index = 0;
                for j in 0..overlap_count {
                    let child_index = pair_overlaps.overlaps[j];

                    // Account for pair flipping.
                    let (child_a, child_b) = if pair.flip_mask < 0 {
                        (child_index, 0)
                    } else {
                        (0, child_index)
                    };

                    let continuation_child_index = next_continuation_child_index;
                    next_continuation_child_index += 1;
                    let _subpair_continuation = PairContinuation::new(
                        pair.continuation.pair_id,
                        child_a,
                        child_b,
                        continuation_handler.collision_continuation_type(),
                        continuation_index,
                        continuation_child_index,
                    );

                    // TODO: Check AllowCollisionTesting callback, then configure and
                    // add the child pair to the batcher. Requires full batcher AddDirectly
                    // method implementation.
                    let mut child_pose_b = RigidPose::IDENTITY;
                    let mut child_type_b = 0i32;
                    let mut child_shape_data_b: *const u8 = std::ptr::null();
                    continuation_handler.configure_continuation_child(
                        batcher,
                        continuation,
                        continuation_child_index,
                        pair,
                        self.shape_type_index_a,
                        child_index,
                        &mut child_pose_b,
                        &mut child_type_b,
                        &mut child_shape_data_b,
                    );

                    // TODO: Dispatch child pair via batcher.add_directly(...)
                }
            } else {
                // TODO: batcher.process_empty_result(pair.continuation)
            }
        }

        overlaps.dispose(pool);
    }
}
