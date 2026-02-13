// Translated from BepuPhysics/CollisionDetection/ConstraintRemover.cs

use crate::physics::bodies::Bodies;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::handy_enumerators::PassthroughReferenceCollector;
use crate::physics::solver::Solver;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::sync::Mutex;

#[derive(Clone, Copy, Default)]
#[repr(C)]
pub(crate) struct TypeBatchIndex {
    pub type_batch: i16,
    pub batch: i16,
}

impl TypeBatchIndex {
    #[inline(always)]
    fn as_i32(&self) -> i32 {
        unsafe { std::mem::transmute::<TypeBatchIndex, i32>(*self) }
    }
}

/// Per-body removal target, identifying the constraint to remove and the body involved.
#[derive(Clone, Copy)]
pub(crate) struct PerBodyRemovalTarget {
    pub encoded_body_index: i32,
    pub constraint_handle: ConstraintHandle,
    pub batch_index: i32,
    pub body_handle: BodyHandle,
}

impl Default for PerBodyRemovalTarget {
    fn default() -> Self {
        Self {
            encoded_body_index: 0,
            constraint_handle: ConstraintHandle(0),
            batch_index: 0,
            body_handle: BodyHandle(0),
        }
    }
}

#[derive(Clone, Copy)]
struct RemovalsForTypeBatch {
    constraint_handles_to_remove: QuickList<ConstraintHandle>,
    per_body_removal_targets: QuickList<PerBodyRemovalTarget>,
}

struct RemovalCache {
    batch_count: i32,
    type_batches: Buffer<TypeBatchIndex>,
    removals_for_type_batches: Buffer<RemovalsForTypeBatch>,
    minimum_capacity_per_batch: i32,
}

impl RemovalCache {
    fn new(pool: &mut BufferPool, batch_capacity: i32, minimum_capacity_per_batch: i32) -> Self {
        let type_batches = pool.take_at_least(batch_capacity);
        let removals = pool.take_at_least(batch_capacity);
        Self {
            batch_count: 0,
            type_batches,
            removals_for_type_batches: removals,
            minimum_capacity_per_batch,
        }
    }

    #[inline(always)]
    fn index_of(&self, type_batch_index: TypeBatchIndex) -> i32 {
        let index_as_int = type_batch_index.as_i32();
        for i in 0..self.batch_count {
            unsafe {
                if index_as_int == (*self.type_batches.get(i)).as_i32() {
                    return i;
                }
            }
        }
        -1
    }

    fn allocate_space_for_targets(
        &mut self,
        type_batch_index: TypeBatchIndex,
        constraint_handle_count: i32,
        per_body_removal_count: i32,
        pool: &mut BufferPool,
    ) -> i32 {
        let index = self.index_of(type_batch_index);
        if index >= 0 {
            unsafe {
                let slot = &mut *self.removals_for_type_batches.get_mut(index);
                slot.per_body_removal_targets.ensure_capacity(
                    slot.per_body_removal_targets.count + per_body_removal_count,
                    pool,
                );
                slot.constraint_handles_to_remove.ensure_capacity(
                    slot.constraint_handles_to_remove.count + constraint_handle_count,
                    pool,
                );
            }
            return index;
        }
        let index = self.batch_count;
        self.batch_count += 1;

        if self.type_batches.len() <= index {
            pool.resize_to_at_least(&mut self.type_batches, self.batch_count, index);
        }
        if self.removals_for_type_batches.len() <= index {
            pool.resize_to_at_least(&mut self.removals_for_type_batches, self.batch_count, index);
        }
        unsafe {
            *self.type_batches.get_mut(index) = type_batch_index;
            *self.removals_for_type_batches.get_mut(index) = RemovalsForTypeBatch {
                constraint_handles_to_remove: QuickList::with_capacity(
                    i32::max(constraint_handle_count, self.minimum_capacity_per_batch),
                    pool,
                ),
                per_body_removal_targets: QuickList::with_capacity(
                    i32::max(per_body_removal_count, self.minimum_capacity_per_batch),
                    pool,
                ),
            };
        }
        index
    }

    fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.type_batches);
        for i in 0..self.batch_count {
            unsafe {
                let removal = &mut *self.removals_for_type_batches.get_mut(i);
                removal.per_body_removal_targets.dispose(pool);
                removal.constraint_handles_to_remove.dispose(pool);
            }
        }
        pool.return_buffer(&mut self.removals_for_type_batches);
        self.batch_count = 0;
    }
}

struct WorkerCache {
    pool: *mut BufferPool,
    removals: RemovalCache,
}

impl WorkerCache {
    fn new(pool: *mut BufferPool, batch_capacity: i32, minimum_capacity_per_batch: i32) -> Self {
        debug_assert!(minimum_capacity_per_batch > 0);
        let pool_ref = unsafe { &mut *pool };
        Self {
            pool,
            removals: RemovalCache::new(pool_ref, batch_capacity, minimum_capacity_per_batch),
        }
    }

    #[inline(always)]
    unsafe fn enqueue_for_removal(
        &mut self,
        constraint_handle: ConstraintHandle,
        solver: &Solver,
        bodies: &Bodies,
    ) {
        let constraint = &*solver.handle_to_constraint.get(constraint_handle.0);
        debug_assert!(
            constraint.set_index == 0,
            "The constraint remover requires that the target constraint is active."
        );

        let mut type_batch_index = TypeBatchIndex::default();
        type_batch_index.batch = constraint.batch_index as i16;
        let constraint_batch = solver.active_set().batches.get(constraint.batch_index);
        type_batch_index.type_batch = *constraint_batch
            .type_index_to_type_batch_index
            .get(constraint.type_id) as i16;

        let type_processor = solver.type_processors[constraint.type_id as usize]
            .as_ref()
            .unwrap();
        let bodies_per_constraint = type_processor.bodies_per_constraint as i32;

        let pool = &mut *self.pool;
        let linear_type_batch_index = self.removals.allocate_space_for_targets(
            type_batch_index,
            1,
            bodies_per_constraint,
            pool,
        );

        let type_batch_removals = &mut *self
            .removals
            .removals_for_type_batches
            .get_mut(linear_type_batch_index);
        type_batch_removals
            .constraint_handles_to_remove
            .add_unsafely(constraint_handle);

        // Enumerate connected body references and push PerBodyRemovalTargets.
        // We do this here rather than during flush because removals from type batches
        // make enumerating connected body indices a race condition there.
        let type_batch = constraint_batch
            .type_batches
            .get(type_batch_index.type_batch as i32);
        let mut encoded_body_indices = [0i32; 8]; // max bodies per constraint
        debug_assert!(
            bodies_per_constraint <= 8,
            "Bodies per constraint exceeds stack buffer size"
        );
        let mut enumerator = PassthroughReferenceCollector::new(encoded_body_indices.as_mut_ptr());
        solver.enumerate_connected_raw_body_references_from_type_batch(
            type_batch,
            constraint.index_in_type_batch,
            &mut enumerator,
        );

        for i in 0..bodies_per_constraint {
            let encoded = encoded_body_indices[i as usize];
            type_batch_removals
                .per_body_removal_targets
                .add_unsafely(PerBodyRemovalTarget {
                    encoded_body_index: encoded,
                    constraint_handle,
                    batch_index: type_batch_index.batch as i32,
                    body_handle: BodyHandle(
                        bodies
                            .active_set()
                            .index_to_handle
                            .get(encoded & Bodies::BODY_REFERENCE_MASK)
                            .0,
                    ),
                });
        }
    }

    fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        self.removals.dispose(pool);
    }
}

/// Accumulates constraints to remove from multiple threads, and efficiently removes them all as a batch.
pub struct ConstraintRemover {
    pub(crate) solver: *mut Solver,
    pub(crate) bodies: *mut Bodies,
    pool: *mut BufferPool,

    previous_capacity_per_batch: i32,
    previous_batch_capacity: i32,
    previous_capacity_multiplier: f32,
    minimum_constraint_capacity: i32,
    minimum_type_capacity: i32,
    worker_caches: Vec<WorkerCache>,
    thread_count: i32,

    batches: Option<RemovalCache>,
    removed_type_batches: Vec<TypeBatchIndex>,
    allocation_ids_to_free: QuickList<i32>,
    batch_removal_locker: Mutex<()>,
}

impl ConstraintRemover {
    pub fn new(
        pool: *mut BufferPool,
        bodies: *mut Bodies,
        solver: *mut Solver,
        minimum_type_capacity: i32,
        minimum_removal_capacity: i32,
        previous_capacity_multiplier: f32,
    ) -> Self {
        Self {
            pool,
            bodies,
            solver,
            minimum_constraint_capacity: minimum_removal_capacity,
            minimum_type_capacity,
            previous_capacity_multiplier,
            previous_capacity_per_batch: 0,
            previous_batch_capacity: 0,
            worker_caches: Vec::new(),
            thread_count: 0,
            batches: None,
            removed_type_batches: Vec::new(),
            allocation_ids_to_free: QuickList::default(),
            batch_removal_locker: Mutex::new(()),
        }
    }

    pub fn with_defaults(pool: *mut BufferPool, bodies: *mut Bodies, solver: *mut Solver) -> Self {
        Self::new(pool, bodies, solver, 4, 128, 1.25)
    }

    /// Prepares per-worker caches for constraint removal.
    pub fn prepare(&mut self, dispatcher: Option<&dyn IThreadDispatcher>) {
        self.thread_count = match dispatcher {
            Some(d) => d.thread_count(),
            None => 1,
        };

        let batch_capacity = i32::max(
            self.minimum_type_capacity,
            (self.previous_batch_capacity as f32 * self.previous_capacity_multiplier) as i32,
        );
        let capacity_per_batch = i32::max(
            self.minimum_constraint_capacity,
            (self.previous_capacity_per_batch as f32 * self.previous_capacity_multiplier) as i32,
        );

        self.worker_caches.clear();
        if let Some(d) = dispatcher {
            for i in 0..self.thread_count {
                // Use per-thread worker pools to avoid contention.
                self.worker_caches.push(WorkerCache::new(
                    d.worker_pool_ptr(i),
                    batch_capacity,
                    capacity_per_batch,
                ));
            }
        } else {
            self.worker_caches.push(WorkerCache::new(
                self.pool,
                batch_capacity,
                capacity_per_batch,
            ));
        }

        // Allocate the fallback deallocation list if needed.
        unsafe {
            let solver = &*self.solver;
            if solver.active_set().batches.count > solver.fallback_batch_threshold() {
                let pool = &mut *self.pool;
                self.allocation_ids_to_free = QuickList::with_capacity(3, pool);
            }
        }
    }

    /// Creates flush jobs after accumulating removals. Returns the number of type-batch removal jobs.
    pub fn create_flush_jobs(&mut self, deterministic: bool) -> i32 {
        let pool = unsafe { &mut *self.pool };
        let mut batches = RemovalCache::new(pool, 32, 8);
        let mut removed_constraint_count = 0i32;

        for i in 0..self.thread_count as usize {
            let cache = &mut self.worker_caches[i];
            for j in 0..cache.removals.batch_count {
                unsafe {
                    let type_batch_index = *cache.removals.type_batches.get(j);
                    let worker_removals = &*cache.removals.removals_for_type_batches.get(j);
                    removed_constraint_count += worker_removals.constraint_handles_to_remove.count;
                    let batch_index = batches.allocate_space_for_targets(
                        type_batch_index,
                        worker_removals.constraint_handles_to_remove.count,
                        worker_removals.per_body_removal_targets.count,
                        pool,
                    );

                    let combined_removals =
                        &mut *batches.removals_for_type_batches.get_mut(batch_index);
                    combined_removals
                        .constraint_handles_to_remove
                        .add_range_unsafely_from_slice(
                            worker_removals.constraint_handles_to_remove.as_slice(),
                            0,
                            worker_removals.constraint_handles_to_remove.count as usize,
                        );
                    combined_removals
                        .per_body_removal_targets
                        .add_range_unsafely_from_slice(
                            worker_removals.per_body_removal_targets.as_slice(),
                            0,
                            worker_removals.per_body_removal_targets.count as usize,
                        );
                }
            }
        }

        // Deterministic sorting: sort within each type batch and between batches.
        if deterministic {
            // Sort within each type batch by constraint handle (and for removal targets, by constraint + body handle).
            for i in 0..batches.batch_count {
                unsafe {
                    let removals = &mut *batches.removals_for_type_batches.get_mut(i);
                    removals
                        .constraint_handles_to_remove
                        .as_mut_slice()
                        .sort_unstable_by(|a, b| a.0.cmp(&b.0));
                    removals
                        .per_body_removal_targets
                        .as_mut_slice()
                        .sort_unstable_by(|a, b| {
                            let a_key =
                                ((a.constraint_handle.0 as i64) << 32) | (a.body_handle.0 as i64);
                            let b_key =
                                ((b.constraint_handle.0 as i64) << 32) | (b.body_handle.0 as i64);
                            a_key.cmp(&b_key)
                        });
                }
            }
            // Sort the batches by type batch index (deterministic ordering).
            unsafe {
                let count = batches.batch_count as usize;
                let mut paired: Vec<(TypeBatchIndex, usize)> = (0..count)
                    .map(|i| (*batches.type_batches.get(i as i32), i))
                    .collect();
                paired.sort_unstable_by(|a, b| a.0.as_i32().cmp(&b.0.as_i32()));

                // Apply the permutation to both TypeBatches and RemovalsForTypeBatches.
                // RemovalsForTypeBatch is Copy, so we can just read and write.
                let new_type_batches: Vec<TypeBatchIndex> =
                    paired.iter().map(|(tb, _)| *tb).collect();
                let new_removals: Vec<RemovalsForTypeBatch> = paired
                    .iter()
                    .map(|(_, old_idx)| *batches.removals_for_type_batches.get(*old_idx as i32))
                    .collect();
                for i in 0..count {
                    *batches.type_batches.get_mut(i as i32) = new_type_batches[i];
                    *batches.removals_for_type_batches.get_mut(i as i32) = new_removals[i];
                }
            }
        }

        // Ensure that the solver's id pool is large enough to hold all constraint handles being removed.
        unsafe {
            let solver = &mut *self.solver;
            solver.handle_pool.ensure_capacity(
                solver.handle_pool.available_id_count() + removed_constraint_count,
                &mut *self.pool,
            );

            // Ensure that the removal list is large enough to hold every single type batch in the worst case.
            let mut type_batch_count = 0;
            let active_set = solver.active_set();
            for i in 0..active_set.batches.count {
                type_batch_count += active_set.batches.get(i).type_batches.count;
            }
            self.removed_type_batches = Vec::with_capacity(type_batch_count as usize);
        }

        let batch_count = batches.batch_count;
        self.batches = Some(batches);
        batch_count
    }

    /// Returns the handles associated with all removed constraints to the solver's handle pool.
    pub fn return_constraint_handles(&mut self) {
        if let Some(ref batches) = self.batches {
            for i in 0..batches.batch_count {
                unsafe {
                    let batch_handles =
                        &(*batches.removals_for_type_batches.get(i)).constraint_handles_to_remove;
                    let solver = &mut *self.solver;
                    for j in 0..batch_handles.count {
                        solver.handle_pool.return_unsafely(batch_handles.get(j).0);
                    }
                }
            }
        }
    }

    /// Removes constraints from body lists.
    pub fn remove_constraints_from_body_lists(&mut self) {
        if let Some(ref batches) = self.batches {
            for i in 0..batches.batch_count {
                unsafe {
                    let removals =
                        &(*batches.removals_for_type_batches.get(i)).per_body_removal_targets;
                    let bodies = &mut *self.bodies;
                    let solver = &mut *self.solver;
                    for j in 0..removals.count {
                        let target = removals.get(j);
                        let was_last = bodies.remove_constraint_reference(
                            target.encoded_body_index & Bodies::BODY_REFERENCE_MASK,
                            target.constraint_handle,
                        );
                        if was_last
                            && (target.encoded_body_index & Bodies::KINEMATIC_MASK as i32) != 0
                        {
                            // This is a kinematic with no remaining constraints. Remove from constrained kinematic set.
                            let _pool = &mut *self.pool;
                            solver
                                .constrained_kinematic_handles
                                .fast_remove(&target.body_handle.0);
                        }
                    }
                }
            }
        }
    }

    /// Removes constraints from the specified type batch.
    pub fn remove_constraints_from_type_batch(&mut self, index: i32) {
        unsafe {
            let batches = self.batches.as_ref().unwrap();
            let batch = *batches.type_batches.get(index);
            let solver_ptr = self.solver;
            let constraint_batch = (*solver_ptr)
                .active_set_mut()
                .batches
                .get_mut(batch.batch as i32);
            let type_batch = constraint_batch
                .type_batches
                .get_mut(batch.type_batch as i32);
            let type_processor = (&(*solver_ptr).type_processors)[type_batch.type_id as usize]
                .as_ref()
                .unwrap();
            let removals = &(*batches.removals_for_type_batches.get(index));
            let fallback_threshold = (*solver_ptr).fallback_batch_threshold();

            for i in 0..removals.constraint_handles_to_remove.count {
                let handle = *removals.constraint_handles_to_remove.get(i);
                // Look up index dynamically since removals change indices.
                let location = *(*solver_ptr).handle_to_constraint.get(handle.0);
                type_processor.inner().remove(
                    type_batch,
                    location.index_in_type_batch,
                    &mut (*solver_ptr).handle_to_constraint,
                    location.batch_index == fallback_threshold,
                );
                if type_batch.constraint_count == 0 {
                    // This batch-typebatch needs to be removed.
                    let _lock = self.batch_removal_locker.lock().unwrap();
                    self.removed_type_batches.push(batch);
                }
            }
        }
    }

    /// Marks affected constraints as removed from the solver.
    pub fn mark_affected_constraints_as_removed_from_solver(&mut self) {
        if let Some(ref batches) = self.batches {
            for i in 0..batches.batch_count {
                unsafe {
                    let batch_handles =
                        &(*batches.removals_for_type_batches.get(i)).constraint_handles_to_remove;
                    let solver = &mut *self.solver;
                    for j in 0..batch_handles.count {
                        (*solver.handle_to_constraint.get_mut(batch_handles.get(j).0)).set_index =
                            -1;
                    }
                }
            }
        }
    }

    /// Removes a body from constrained kinematics and removes all fallback batch constraints for it.
    /// Called during sleep transitions. Equivalent to C# ConstraintRemover.TryRemoveBodyFromConstrainedKinematicsAndRemoveAllConstraintsForBodyFromFallbackBatch.
    pub unsafe fn try_remove_body_from_constrained_kinematics_and_remove_all_constraints_for_body_from_fallback_batch(
        &mut self,
        body_handle: BodyHandle,
        body_index: i32,
    ) {
        let solver = &mut *self.solver;
        let _pool = &mut *self.pool;
        solver.try_remove_dynamic_body_from_fallback(
            body_handle,
            body_index,
            &mut self.allocation_ids_to_free,
        );
        // Note that we don't check kinematicity here. If it's dynamic, that's fine, this won't do anything.
        solver
            .constrained_kinematic_handles
            .fast_remove(&body_handle.0);
    }

    /// Removes constraints from batch referenced handles (non-fallback batches).
    pub fn remove_constraints_from_batch_referenced_handles(&mut self) {
        if let Some(ref batches) = self.batches {
            unsafe {
                let solver = &mut *self.solver;
                let fallback_threshold = solver.fallback_batch_threshold();
                for i in 0..batches.batch_count {
                    let type_batch_idx = *batches.type_batches.get(i);
                    if type_batch_idx.batch as i32 == fallback_threshold {
                        // Fallback batch is handled separately.
                        continue;
                    }
                    let removals =
                        &(*batches.removals_for_type_batches.get(i)).per_body_removal_targets;
                    for j in 0..removals.count {
                        let target = removals.get(j);
                        solver
                            .batch_referenced_handles
                            .get_mut(target.batch_index)
                            .unset(target.body_handle.0);
                    }
                }
            }
        }
    }

    /// Removes constraints from fallback batch referenced handles.
    pub fn remove_constraints_from_fallback_batch_referenced_handles(&mut self) {
        if let Some(ref batches) = self.batches {
            unsafe {
                let solver = &mut *self.solver;
                let fallback_threshold = solver.fallback_batch_threshold();
                debug_assert!((*solver).active_set().batches.count > fallback_threshold);
                for i in 0..batches.batch_count {
                    let type_batch_idx = *batches.type_batches.get(i);
                    if type_batch_idx.batch as i32 != fallback_threshold {
                        continue;
                    }
                    let removals =
                        &(*batches.removals_for_type_batches.get(i)).per_body_removal_targets;
                    for j in 0..removals.count {
                        let target = removals.get(j);
                        let encoded = target.encoded_body_index & Bodies::BODY_REFERENCE_MASK;
                        if solver
                            .active_set_mut()
                            .sequential_fallback
                            .remove_one_body_reference_from_dynamics_set(
                                encoded,
                                &mut self.allocation_ids_to_free,
                            )
                        {
                            solver
                                .batch_referenced_handles
                                .get_mut(target.batch_index)
                                .unset(target.body_handle.0);
                        }
                    }
                }
            }
        }
    }

    /// Post-flush cleanup: removes empty type batches and returns worker cache allocations.
    pub fn postflush(&mut self) {
        unsafe {
            if !self.removed_type_batches.is_empty() {
                // Sort removed batches from highest to lowest so higher index batches get removed first.
                self.removed_type_batches
                    .sort_unstable_by(|a, b| b.as_i32().cmp(&a.as_i32()));
                let solver_ptr = self.solver;
                for i in 0..self.removed_type_batches.len() {
                    let batch_indices = self.removed_type_batches[i];
                    let active_set = (*solver_ptr).active_set_mut();
                    let batch = active_set.batches.get_mut(batch_indices.batch as i32);
                    let pool = &mut *(*solver_ptr).pool;
                    batch.remove_type_batch_if_empty(batch_indices.type_batch as i32, pool);
                    (*solver_ptr).remove_batch_if_empty(batch_indices.batch as i32);
                }
            }
            self.removed_type_batches.clear();

            // Return any fallback allocation ids.
            let pool = &mut *self.pool;
            if self.allocation_ids_to_free.span.allocated() {
                for i in 0..self.allocation_ids_to_free.count {
                    pool.return_unsafely(*self.allocation_ids_to_free.get(i));
                }
                self.allocation_ids_to_free.dispose(pool);
            }
        }

        if let Some(mut batches) = self.batches.take() {
            let pool = unsafe { &mut *self.pool };
            batches.dispose(pool);
        }

        // Get rid of worker cache allocations and store capacities for next frame
        self.previous_capacity_per_batch = 0;
        for i in 0..self.thread_count as usize {
            let worker_cache = &mut self.worker_caches[i];
            for j in 0..worker_cache.removals.batch_count {
                unsafe {
                    let removals = &*worker_cache.removals.removals_for_type_batches.get(j);
                    let count = removals.constraint_handles_to_remove.count;
                    if self.previous_capacity_per_batch < count {
                        self.previous_capacity_per_batch = count;
                    }
                }
            }
            if self.previous_batch_capacity < worker_cache.removals.batch_count {
                self.previous_batch_capacity = worker_cache.removals.batch_count;
            }
            worker_cache.dispose();
        }
    }

    /// Enqueues a constraint for removal from a specific worker thread.
    #[inline(always)]
    pub fn enqueue_removal(&mut self, worker_index: i32, constraint_handle: ConstraintHandle) {
        unsafe {
            let solver = &*self.solver;
            let bodies = &*self.bodies;
            self.worker_caches[worker_index as usize].enqueue_for_removal(
                constraint_handle,
                solver,
                bodies,
            );
        }
    }
}
