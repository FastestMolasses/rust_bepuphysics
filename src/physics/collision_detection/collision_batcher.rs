// Translated from BepuPhysics/CollisionDetection/CollisionBatcher.cs

use crate::physics::body_properties::BodyVelocity;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::collision_detection::collision_batcher_continuations::{
    BatcherContinuations, CollisionContinuationType, ICollisionTestContinuation, PairContinuation,
};
use crate::physics::collision_detection::collision_task_registry::{
    BatcherVtable, CollisionTaskPairType, CollisionTaskRegistry,
};
use crate::physics::collision_detection::compound_mesh_reduction::CompoundMeshReduction;
use crate::physics::collision_detection::contact_manifold::{
    ConvexContactManifold, IContactManifold,
};
use crate::physics::collision_detection::mesh_reduction::MeshReduction;
use crate::physics::collision_detection::nonconvex_reduction::{FlushResult, NonconvexReduction};
use crate::physics::collision_detection::untyped_list::UntypedList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};

/// A raw byte blob that doesn't auto-resize. Used for caching shape data.
pub(crate) struct UntypedBlob {
    pub buffer: Buffer<u8>,
    pub byte_count: i32,
}

impl UntypedBlob {
    #[inline(always)]
    pub unsafe fn allocate(&mut self, allocation_size_in_bytes: i32) -> *mut u8 {
        let new_byte_count = self.byte_count + allocation_size_in_bytes;
        debug_assert!(
            new_byte_count <= self.buffer.len(),
            "This collection doesn't auto-resize! You forgot to initialize this, or initialized it to an insufficient capacity."
        );
        let to_return = self.buffer.as_mut_ptr().add(self.byte_count as usize);
        self.byte_count = new_byte_count;
        to_return
    }
}

impl Default for UntypedBlob {
    fn default() -> Self {
        Self {
            buffer: Buffer::default(),
            byte_count: 0,
        }
    }
}

/// A pending batch of collision pairs plus cached shape data.
pub(crate) struct CollisionBatch {
    pub pairs: UntypedList,
    pub shapes: UntypedBlob,
}

impl Default for CollisionBatch {
    fn default() -> Self {
        Self {
            pairs: UntypedList::default(),
            shapes: UntypedBlob::default(),
        }
    }
}

/// Callbacks interface for collision batcher results.
pub trait ICollisionCallbacks {
    /// Called when a pair submitted to a collision batcher has finished collision detection.
    fn on_pair_completed<TManifold: IContactManifold>(
        &mut self,
        pair_id: i32,
        manifold: &mut TManifold,
    );

    /// Called when a child pair of a compound/mesh collision completes.
    fn on_child_pair_completed(
        &mut self,
        pair_id: i32,
        child_a: i32,
        child_b: i32,
        manifold: &mut ConvexContactManifold,
    );

    /// Checks whether further collision testing should be performed for a given subtask.
    fn allow_collision_testing(&self, pair_id: i32, child_a: i32, child_b: i32) -> bool;
}

/// Collision pair fed to collision tasks for standard pairs.
#[repr(C)]
pub struct CollisionPair {
    pub a: *const u8,
    pub b: *const u8,
    pub flip_mask: i32,
    pub offset_b: Vec3,
    pub orientation_a: Quat,
    pub orientation_b: Quat,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

/// Collision pair variation that omits the flip mask.
#[repr(C)]
pub struct FliplessPair {
    pub a: *const u8,
    pub b: *const u8,
    pub offset_b: Vec3,
    pub orientation_a: Quat,
    pub orientation_b: Quat,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

/// Collision pair with bounds testing data (velocity, expansion).
#[repr(C)]
pub struct BoundsTestedPair {
    pub a: *const u8,
    pub b: *const u8,
    pub flip_mask: i32,
    pub offset_b: Vec3,
    pub orientation_a: Quat,
    pub orientation_b: Quat,
    pub relative_linear_velocity_a: Vec3,
    pub angular_velocity_a: Vec3,
    pub angular_velocity_b: Vec3,
    pub maximum_expansion: f32,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

/// Batches collision tests and manages post-processing continuations.
pub struct CollisionBatcher<TCallbacks: ICollisionCallbacks> {
    pub pool: *mut BufferPool,
    pub shapes: *mut crate::physics::collidables::shapes::Shapes,
    type_matrix: *mut CollisionTaskRegistry,
    pub callbacks: TCallbacks,
    /// Timestep duration used by pairs which rely on velocity to compute local bounding boxes.
    pub dt: f32,

    minimum_batch_index: i32,
    maximum_batch_index: i32,
    pub(crate) batches: Buffer<CollisionBatch>,
    pub nonconvex_reductions: BatcherContinuations<NonconvexReduction>,
    pub mesh_reductions: BatcherContinuations<MeshReduction>,
    pub compound_mesh_reductions: BatcherContinuations<CompoundMeshReduction>,
}

impl<TCallbacks: ICollisionCallbacks> CollisionBatcher<TCallbacks> {
    pub fn new(
        pool: *mut BufferPool,
        shapes: *mut crate::physics::collidables::shapes::Shapes,
        collision_type_matrix: *mut CollisionTaskRegistry,
        dt: f32,
        callbacks: TCallbacks,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let type_matrix = unsafe { &*collision_type_matrix };
        let task_count = type_matrix.tasks.len() as i32;
        let mut batches: Buffer<CollisionBatch> = pool_ref.take_at_least(task_count);
        // Clear to ensure we know when a batch needs to be created
        for i in 0..task_count {
            unsafe {
                *batches.get_mut(i) = CollisionBatch::default();
            }
        }

        Self {
            pool,
            shapes,
            type_matrix: collision_type_matrix,
            callbacks,
            dt,
            minimum_batch_index: task_count,
            maximum_batch_index: -1,
            batches,
            nonconvex_reductions: BatcherContinuations::new(),
            mesh_reductions: BatcherContinuations::new(),
            compound_mesh_reductions: BatcherContinuations::new(),
        }
    }

    /// Adds a collision pair using shape type indices, looking up shape data from the Shapes table.
    #[inline(always)]
    pub unsafe fn add(
        &mut self,
        shape_index_a: TypedIndex,
        shape_index_b: TypedIndex,
        offset_b: Vec3,
        orientation_a: Quat,
        orientation_b: Quat,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
        speculative_margin: f32,
        maximum_expansion: f32,
        continuation: PairContinuation,
    ) {
        let shapes = &*self.shapes;
        let shape_type_a = shape_index_a.type_id();
        let shape_type_b = shape_index_b.type_id();
        let batch_a = shapes.get_batch(shape_type_a as usize).expect("shape batch A must exist");
        let batch_b = shapes.get_batch(shape_type_b as usize).expect("shape batch B must exist");
        let (shape_a, _size_a) = batch_a.get_shape_data(shape_index_a.index() as usize);
        let (shape_b, _size_b) = batch_b.get_shape_data(shape_index_b.index() as usize);
        self.add_directly(
            shape_type_a, shape_type_b,
            shape_a, shape_b,
            offset_b, orientation_a, orientation_b,
            velocity_a, velocity_b,
            speculative_margin, maximum_expansion,
            &continuation,
        );
    }

    /// Adds a collision pair directly with pointers to shape data.
    #[inline(always)]
    pub unsafe fn add_directly(
        &mut self,
        shape_type_a: i32,
        shape_type_b: i32,
        shape_a: *const u8,
        shape_b: *const u8,
        offset_b: Vec3,
        orientation_a: Quat,
        orientation_b: Quat,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
        speculative_margin: f32,
        maximum_expansion: f32,
        pair_continuation: &PairContinuation,
    ) {
        let type_matrix = &*self.type_matrix;
        let reference = type_matrix.get_task_reference(shape_type_a, shape_type_b);

        if reference.task_index < 0 {
            // No task for this shape type pair. Immediately respond with an empty manifold.
            let mut manifold = ConvexContactManifold::default();
            self.callbacks
                .on_pair_completed(pair_continuation.pair_id, &mut manifold);
            return;
        }

        if shape_type_a != reference.expected_first_type_id {
            debug_assert!(shape_type_b == reference.expected_first_type_id);
            self.add_internal(
                &reference,
                -1,
                shape_type_b,
                shape_type_a,
                shape_b,
                shape_a,
                -offset_b,
                orientation_b,
                orientation_a,
                velocity_b,
                velocity_a,
                speculative_margin,
                maximum_expansion,
                pair_continuation,
            );
        } else {
            self.add_internal(
                &reference,
                0,
                shape_type_a,
                shape_type_b,
                shape_a,
                shape_b,
                offset_b,
                orientation_a,
                orientation_b,
                velocity_a,
                velocity_b,
                speculative_margin,
                maximum_expansion,
                pair_continuation,
            );
        }
    }

    unsafe fn add_internal(
        &mut self,
        reference: &crate::physics::collision_detection::collision_task_registry::CollisionTaskReference,
        flip_mask: i32,
        _shape_type_a: i32,
        _shape_type_b: i32,
        shape_a: *const u8,
        shape_b: *const u8,
        offset_b: Vec3,
        orientation_a: Quat,
        orientation_b: Quat,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
        speculative_margin: f32,
        maximum_expansion: f32,
        continuation: &PairContinuation,
    ) {
        let batch = &mut *self.batches.get_mut(reference.task_index);

        if !batch.pairs.buffer.allocated() {
            let element_size = match reference.pair_type {
                CollisionTaskPairType::StandardPair => std::mem::size_of::<CollisionPair>() as i32,
                CollisionTaskPairType::FliplessPair => std::mem::size_of::<FliplessPair>() as i32,
                CollisionTaskPairType::BoundsTestedPair => {
                    std::mem::size_of::<BoundsTestedPair>() as i32
                }
                _ => std::mem::size_of::<CollisionPair>() as i32,
            };
            let pool = &mut *self.pool;
            batch.pairs = UntypedList::new(element_size, reference.batch_size, pool);
            if self.minimum_batch_index > reference.task_index {
                self.minimum_batch_index = reference.task_index;
            }
            if self.maximum_batch_index < reference.task_index {
                self.maximum_batch_index = reference.task_index;
            }
        }

        match reference.pair_type {
            CollisionTaskPairType::StandardPair => {
                let pair_ptr = batch.pairs.allocate_unsafely::<CollisionPair>();
                let pair = &mut *pair_ptr;
                pair.a = shape_a;
                pair.b = shape_b;
                pair.flip_mask = flip_mask;
                pair.offset_b = offset_b;
                pair.orientation_a = orientation_a;
                pair.orientation_b = orientation_b;
                pair.speculative_margin = speculative_margin;
                pair.continuation = *continuation;
            }
            CollisionTaskPairType::FliplessPair => {
                let pair_ptr = batch.pairs.allocate_unsafely::<FliplessPair>();
                let pair = &mut *pair_ptr;
                pair.a = shape_a;
                pair.b = shape_b;
                pair.offset_b = offset_b;
                pair.orientation_a = orientation_a;
                pair.orientation_b = orientation_b;
                pair.speculative_margin = speculative_margin;
                pair.continuation = *continuation;
            }
            CollisionTaskPairType::BoundsTestedPair => {
                let pair_ptr = batch.pairs.allocate_unsafely::<BoundsTestedPair>();
                let pair = &mut *pair_ptr;
                pair.a = shape_a;
                pair.b = shape_b;
                pair.flip_mask = flip_mask;
                pair.offset_b = offset_b;
                pair.orientation_a = orientation_a;
                pair.orientation_b = orientation_b;
                pair.relative_linear_velocity_a = velocity_a.linear - velocity_b.linear;
                pair.angular_velocity_a = velocity_a.angular;
                pair.angular_velocity_b = velocity_b.angular;
                pair.maximum_expansion = maximum_expansion;
                pair.speculative_margin = speculative_margin;
                pair.continuation = *continuation;
            }
            _ => {}
        }

        // If the batch is full, execute it
        if batch.pairs.count == reference.batch_size {
            let type_matrix = &*self.type_matrix;
            // Use raw pointer to avoid double-mutable-borrow since batch borrows from self.batches
            let batch_ptr = batch as *mut CollisionBatch;
            let vtable = self.make_vtable();
            type_matrix.tasks[reference.task_index as usize].execute_batch(
                &mut (*batch_ptr).pairs,
                &vtable,
            );
            (*batch_ptr).pairs.count = 0;
            (*batch_ptr).pairs.byte_count = 0;
            (*batch_ptr).shapes.byte_count = 0;
        }
    }

    /// Constructs a type-erased vtable for batcher operations.
    ///
    /// This enables compound collision tasks (which generate subtasks back into the batcher)
    /// to call batcher methods without knowing the concrete `TCallbacks` type.
    unsafe fn make_vtable(&mut self) -> BatcherVtable {
        BatcherVtable {
            batcher: self as *mut Self as *mut u8,
            pool: self.pool,
            shapes: self.shapes,
            dt: self.dt,
            nonconvex_reductions: &mut self.nonconvex_reductions,
            mesh_reductions: &mut self.mesh_reductions,
            compound_mesh_reductions: &mut self.compound_mesh_reductions,
            process_convex_result: Self::process_convex_result_trampoline,
            add_directly: Self::add_directly_trampoline,
            process_empty_result: Self::process_empty_result_trampoline,
            process_untested_subpair_convex_result: Self::process_untested_subpair_trampoline,
            allow_collision_testing: Self::allow_collision_testing_trampoline,
        }
    }

    /// Type-erased trampoline for calling `process_convex_result` from collision tasks.
    /// This allows collision tasks to dispatch results without knowing `TCallbacks`.
    unsafe fn process_convex_result_trampoline(
        batcher: *mut u8,
        manifold: &mut ConvexContactManifold,
        continuation: &PairContinuation,
    ) {
        let batcher = &mut *(batcher as *mut CollisionBatcher<TCallbacks>);
        batcher.process_convex_result(manifold, continuation);
    }

    /// Type-erased trampoline for `add_directly`.
    unsafe fn add_directly_trampoline(
        batcher: *mut u8,
        shape_type_a: i32,
        shape_type_b: i32,
        shape_a: *const u8,
        shape_b: *const u8,
        offset_b: Vec3,
        orientation_a: Quat,
        orientation_b: Quat,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
        speculative_margin: f32,
        maximum_expansion: f32,
        pair_continuation: &PairContinuation,
    ) {
        let batcher = &mut *(batcher as *mut CollisionBatcher<TCallbacks>);
        batcher.add_directly(
            shape_type_a, shape_type_b, shape_a, shape_b,
            offset_b, orientation_a, orientation_b,
            velocity_a, velocity_b, speculative_margin, maximum_expansion,
            pair_continuation,
        );
    }

    /// Type-erased trampoline for `process_empty_result`.
    unsafe fn process_empty_result_trampoline(
        batcher: *mut u8,
        continuation: &PairContinuation,
    ) {
        let batcher = &mut *(batcher as *mut CollisionBatcher<TCallbacks>);
        batcher.process_empty_result(continuation);
    }

    /// Type-erased trampoline for `process_untested_subpair_convex_result`.
    unsafe fn process_untested_subpair_trampoline(
        batcher: *mut u8,
        continuation: &PairContinuation,
    ) {
        let batcher = &mut *(batcher as *mut CollisionBatcher<TCallbacks>);
        batcher.process_untested_subpair_convex_result(continuation);
    }

    /// Type-erased trampoline for `allow_collision_testing`.
    unsafe fn allow_collision_testing_trampoline(
        batcher: *mut u8,
        pair_id: i32,
        child_a: i32,
        child_b: i32,
    ) -> bool {
        let batcher = &mut *(batcher as *mut CollisionBatcher<TCallbacks>);
        batcher.callbacks.allow_collision_testing(pair_id, child_a, child_b)
    }

    /// Reports the result of a convex collision test to the callbacks and, if necessary, to any continuations for postprocessing.
    pub fn process_convex_result(
        &mut self,
        manifold: &mut ConvexContactManifold,
        continuation: &PairContinuation,
    ) {
        if continuation.continuation_type() == CollisionContinuationType::Direct {
            // This result concerns a pair which had no higher level owner. Directly report the manifold result.
            self.callbacks
                .on_pair_completed(continuation.pair_id, manifold);
        } else {
            // This result is associated with another pair and requires additional processing.
            // Before we move to the next stage, notify the submitter that the subpair has completed.
            self.callbacks.on_child_pair_completed(
                continuation.pair_id,
                continuation.child_a,
                continuation.child_b,
                manifold,
            );
            unsafe {
                match continuation.continuation_type() {
                    CollisionContinuationType::NonconvexReduction => {
                        let slot = &mut *self.nonconvex_reductions.continuations.get_mut(continuation.index());
                        slot.on_child_completed(continuation, manifold);
                        if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                            self.report_flush_result(continuation.pair_id, flush_result);
                            self.nonconvex_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                        }
                    }
                    CollisionContinuationType::MeshReduction => {
                        let slot = &mut *self.mesh_reductions.continuations.get_mut(continuation.index());
                        slot.on_child_completed(continuation, manifold);
                        if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                            self.report_flush_result(continuation.pair_id, flush_result);
                            self.mesh_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                        }
                    }
                    CollisionContinuationType::CompoundMeshReduction => {
                        let slot = &mut *self.compound_mesh_reductions.continuations.get_mut(continuation.index());
                        slot.on_child_completed(continuation, manifold);
                        if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                            self.report_flush_result(continuation.pair_id, flush_result);
                            self.compound_mesh_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    /// Reports the flush result of a continuation to the appropriate callbacks.
    fn report_flush_result(&mut self, pair_id: i32, flush_result: FlushResult) {
        match flush_result {
            FlushResult::Nonconvex(mut manifold) => {
                self.callbacks.on_pair_completed(pair_id, &mut manifold);
            }
            FlushResult::Convex(mut manifold) => {
                self.callbacks.on_pair_completed(pair_id, &mut manifold);
            }
        }
    }

    /// Reports an empty collision result.
    pub fn process_empty_result(&mut self, continuation: &PairContinuation) {
        let mut manifold = ConvexContactManifold::default();
        self.process_convex_result(&mut manifold, continuation);
    }

    /// Submits a subpair whose testing was blocked by user callback as complete to any relevant continuations.
    /// Note: does NOT call on_child_pair_completed. A callback is only invoked if a child is actually tested.
    pub fn process_untested_subpair_convex_result(&mut self, continuation: &PairContinuation) {
        unsafe {
            match continuation.continuation_type() {
                CollisionContinuationType::NonconvexReduction => {
                    let slot = &mut *self.nonconvex_reductions.continuations.get_mut(continuation.index());
                    slot.on_untested_child_completed(continuation);
                    if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                        self.report_flush_result(continuation.pair_id, flush_result);
                        self.nonconvex_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                    }
                }
                CollisionContinuationType::MeshReduction => {
                    let slot = &mut *self.mesh_reductions.continuations.get_mut(continuation.index());
                    slot.on_untested_child_completed(continuation);
                    if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                        self.report_flush_result(continuation.pair_id, flush_result);
                        self.mesh_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                    }
                }
                CollisionContinuationType::CompoundMeshReduction => {
                    let slot = &mut *self.compound_mesh_reductions.continuations.get_mut(continuation.index());
                    slot.on_untested_child_completed(continuation);
                    if let Some(flush_result) = slot.try_flush(continuation.pair_id, &mut *self.pool) {
                        self.report_flush_result(continuation.pair_id, flush_result);
                        self.compound_mesh_reductions.id_pool.return_id(continuation.index(), &mut *self.pool);
                    }
                }
                _ => {}
            }
        }
    }

    /// Forces any remaining partial batches to execute and disposes the batcher.
    pub fn flush(&mut self) {
        unsafe {
            let type_matrix = &*self.type_matrix;
            let vtable = self.make_vtable();
            // Execute remaining partial batches
            for i in self.minimum_batch_index..=self.maximum_batch_index {
                let batch_ptr = self.batches.get_mut_ptr(i) as *mut CollisionBatch;
                if (*batch_ptr).pairs.count > 0 {
                    type_matrix.tasks[i as usize].execute_batch(
                        &mut (*batch_ptr).pairs,
                        &vtable,
                    );
                }
            }
            // Dispose batch resources
            let pool = &mut *self.pool;
            for i in self.minimum_batch_index..=self.maximum_batch_index {
                let batch = &mut *self.batches.get_mut(i);
                if batch.pairs.buffer.allocated() {
                    pool.return_buffer(&mut batch.pairs.buffer);
                }
                if batch.shapes.buffer.allocated() {
                    pool.return_buffer(&mut batch.shapes.buffer);
                }
            }
            pool.return_buffer(&mut self.batches);
            // Dispose continuation resources
            self.nonconvex_reductions.dispose(pool);
            self.mesh_reductions.dispose(pool);
            self.compound_mesh_reductions.dispose(pool);
        }
    }
}
