// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundCollisionTask.cs

use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::physics::collision_detection::collision_batcher_continuations::{
    CollisionContinuationType, ICollisionTestContinuation, PairContinuation,
};
use crate::physics::collision_detection::collision_task_registry::{
    BatcherVtable, CollisionTask, CollisionTaskPairType,
};
use crate::physics::collision_detection::untyped_list::UntypedList;
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
    /// Caller must ensure vtable pointers are valid.
    unsafe fn create_continuation(
        &self,
        vtable: &BatcherVtable,
        child_count: i32,
        pair: &BoundsTestedPair,
        query_for_pair: &OverlapQueryForPair,
        continuation_index: &mut i32,
    ) -> *mut TContinuation;

    /// Configures a child of the continuation with the compound child's shape data and pose.
    ///
    /// # Safety
    /// Caller must ensure all pointers are valid.
    unsafe fn configure_continuation_child(
        &self,
        vtable: &BatcherVtable,
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
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &crate::physics::collidables::shapes::Shapes,
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
    /// All pointers in pairs and the vtable must be valid.
    pub unsafe fn execute_batch(
        &self,
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        vtable: &BatcherVtable,
        continuation_handler: &TContinuationHandler,
    ) {
        let pool = &mut *vtable.pool;

        // Find all overlaps up front to improve cache behavior.
        let mut overlaps = ConvexCompoundTaskOverlaps::new(pool, pair_count);
        TOverlapFinder::find_local_overlaps(
            pairs,
            pair_count,
            pool,
            &*vtable.shapes,
            vtable.dt,
            &mut overlaps,
        );

        for i in 0..pair_count {
            // Use raw pointer access to avoid double mutable borrow — accessing different fields.
            let pair_overlaps = &mut *overlaps.pair_overlaps.get_mut(i);
            let pair_query = &*overlaps.subpair_queries.get(i);
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
                    vtable,
                    overlap_count,
                    pair,
                    pair_query,
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
                    let subpair_continuation = PairContinuation::new(
                        pair.continuation.pair_id,
                        child_a,
                        child_b,
                        continuation_handler.collision_continuation_type(),
                        continuation_index,
                        continuation_child_index,
                    );

                    if (vtable.allow_collision_testing)(
                        vtable.batcher,
                        pair.continuation.pair_id,
                        child_a,
                        child_b,
                    ) {
                        let mut child_pose_b = RigidPose::IDENTITY;
                        let mut child_type_b = 0i32;
                        let mut child_shape_data_b: *const u8 = std::ptr::null();
                        continuation_handler.configure_continuation_child(
                            vtable,
                            continuation,
                            continuation_child_index,
                            pair,
                            self.shape_type_index_a,
                            child_index,
                            &mut child_pose_b,
                            &mut child_type_b,
                            &mut child_shape_data_b,
                        );

                        let convex_to_child = child_pose_b.position + pair.offset_b;
                        if pair.flip_mask < 0 {
                            // By reversing the order of the parameters, the manifold orientation is flipped.
                            // This compensates for the flip induced by order requirements on this task.
                            (vtable.add_directly)(
                                vtable.batcher,
                                child_type_b,
                                self.shape_type_index_a,
                                child_shape_data_b,
                                pair.a,
                                -convex_to_child,
                                child_pose_b.orientation,
                                pair.orientation_a,
                                &BodyVelocity::default(),
                                &BodyVelocity::default(),
                                pair.speculative_margin,
                                0.0,
                                &subpair_continuation,
                            );
                        } else {
                            (vtable.add_directly)(
                                vtable.batcher,
                                self.shape_type_index_a,
                                child_type_b,
                                pair.a,
                                child_shape_data_b,
                                convex_to_child,
                                pair.orientation_a,
                                child_pose_b.orientation,
                                &BodyVelocity::default(),
                                &BodyVelocity::default(),
                                pair.speculative_margin,
                                0.0,
                                &subpair_continuation,
                            );
                        }
                    } else {
                        (vtable.process_untested_subpair_convex_result)(
                            vtable.batcher,
                            &subpair_continuation,
                        );
                    }
                }
            } else {
                (vtable.process_empty_result)(vtable.batcher, &pair.continuation);
            }
        }

        overlaps.dispose(pool);
    }
}

impl<
        TOverlapFinder: IConvexCompoundOverlapFinder + 'static,
        TContinuationHandler: IConvexCompoundContinuationHandler<TContinuation> + Default + 'static,
        TContinuation: ICollisionTestContinuation + 'static,
    > CollisionTask
    for ConvexCompoundCollisionTask<TOverlapFinder, TContinuationHandler, TContinuation>
{
    fn batch_size(&self) -> i32 {
        self.batch_size
    }
    fn shape_type_index_a(&self) -> i32 {
        self.shape_type_index_a
    }
    fn shape_type_index_b(&self) -> i32 {
        self.shape_type_index_b
    }
    fn subtask_generator(&self) -> bool {
        true
    }
    fn pair_type(&self) -> CollisionTaskPairType {
        CollisionTaskPairType::BoundsTestedPair
    }
    fn execute_batch(&self, batch: &mut UntypedList, vtable: &BatcherVtable) {
        unsafe {
            let pairs = batch.buffer.cast::<BoundsTestedPair>();
            let handler = TContinuationHandler::default();
            self.execute_batch(&pairs, batch.count, vtable, &handler);
        }
    }
}
