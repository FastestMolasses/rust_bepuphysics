// Translated from BepuPhysics/IslandAwakener.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_set::BodySet;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::solver::{PrimitiveComparer, Solver};
use crate::physics::statics::Statics;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quick_dictionary::QuickDictionary;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use crate::utilities::vector::Vector;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

/// Provides functionality for efficiently waking up sleeping bodies.
pub struct IslandAwakener {
    solver: *mut Solver,
    statics: *mut Statics,
    bodies: *mut Bodies,
    broad_phase: *mut BroadPhase,
    sleeper: *mut IslandSleeper,
    pool: *mut BufferPool,

    pub(crate) pair_cache: *mut PairCache,

    // Worker dispatch state
    job_index: UnsafeCell<i32>,
    job_count: i32,

    reset_activity_states: bool,
    unique_set_indices: QuickList<i32>,
    phase_one_jobs: QuickList<PhaseOneJob>,
    phase_two_jobs: QuickList<PhaseTwoJob>,
}

// Use real types from their modules.
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::collision_detection::pair_cache::PairCache;
use crate::physics::island_sleeper::IslandSleeper;

// --- Job types ---

#[derive(Clone, Copy)]
#[repr(i32)]
enum PhaseOneJobType {
    PairCache = 0,
    MoveFallbackBatchBodies,
    UpdateBatchReferencedHandles,
    CopyBodyRegion,
}

#[derive(Clone, Copy)]
struct PhaseOneJob {
    job_type: PhaseOneJobType,
    batch_index: i32,
    // Only used by body region copy
    source_set: i32,
    source_start: i32,
    target_start: i32,
    count: i32,
}

#[derive(Clone, Copy)]
#[repr(i32)]
enum PhaseTwoJobType {
    BroadPhase = 0,
    CopyConstraintRegion,
    AddFallbackTypeBatchConstraints,
}

#[derive(Clone, Copy)]
struct CopyConstraintRegionJob {
    source_start: i32,
    target_start: i32,
    count: i32,
    source_set: i32,
    type_id: i32,
    batch: i32,
    source_type_batch: i32,
    target_type_batch: i32,
}

#[derive(Clone, Copy)]
struct FallbackAddSource {
    source_set: i32,
    source_type_batch_index: i32,
}

struct AddFallbackTypeBatchConstraintsJob {
    sources: Buffer<FallbackAddSource>,
    type_id: i32,
    target_type_batch: i32,
}

#[derive(Clone, Copy)]
#[repr(C)]
union PhaseTwoJobData {
    copy_constraint_region: CopyConstraintRegionJob,
    // Fallback constraints handled via separate storage
}

#[derive(Clone, Copy)]
struct PhaseTwoJob {
    job_type: PhaseTwoJobType,
    copy_constraint_region: CopyConstraintRegionJob,
    // For AddFallbackTypeBatchConstraints we store the data alongside
    fallback_sources: Buffer<FallbackAddSource>,
    fallback_type_id: i32,
    fallback_target_type_batch: i32,
}

impl Default for PhaseTwoJob {
    fn default() -> Self {
        Self {
            job_type: PhaseTwoJobType::BroadPhase,
            copy_constraint_region: CopyConstraintRegionJob {
                source_start: 0,
                target_start: 0,
                count: 0,
                source_set: 0,
                type_id: 0,
                batch: 0,
                source_type_batch: 0,
                target_type_batch: 0,
            },
            fallback_sources: Buffer::default(),
            fallback_type_id: 0,
            fallback_target_type_batch: 0,
        }
    }
}

impl IslandAwakener {
    pub fn new(
        bodies: *mut Bodies,
        statics: *mut Statics,
        solver: *mut Solver,
        broad_phase: *mut BroadPhase,
        sleeper: *mut IslandSleeper,
        pair_cache: *mut PairCache,
        pool: *mut BufferPool,
    ) -> Self {
        Self {
            bodies,
            statics,
            solver,
            broad_phase,
            sleeper,
            pair_cache,
            pool,
            job_index: UnsafeCell::new(-1),
            job_count: 0,
            reset_activity_states: false,
            unique_set_indices: QuickList::default(),
            phase_one_jobs: QuickList::default(),
            phase_two_jobs: QuickList::default(),
        }
    }

    fn solver(&self) -> &Solver {
        unsafe { &*self.solver }
    }

    fn solver_mut(&self) -> &mut Solver {
        unsafe { &mut *self.solver }
    }

    fn bodies(&self) -> &Bodies {
        unsafe { &*self.bodies }
    }

    fn pool(&self) -> &mut BufferPool {
        unsafe { &mut *self.pool }
    }

    /// Wakes up a body if it is sleeping. All bodies that can be found by traversing the
    /// constraint graph from the body will also be awakened. If already awake, does nothing.
    pub fn awaken_body(&mut self, body_handle: BodyHandle) {
        let bodies = self.bodies();
        bodies.validate_existing_handle(body_handle);
        let set_index = unsafe { bodies.handle_to_location.get(body_handle.0) }.set_index;
        self.awaken_set(set_index);
    }

    /// Wakes up any sleeping bodies associated with a constraint.
    pub fn awaken_constraint(&mut self, constraint_handle: ConstraintHandle) {
        let set_index =
            unsafe { self.solver().handle_to_constraint.get(constraint_handle.0) }.set_index;
        self.awaken_set(set_index);
    }

    /// Wakes up all bodies and constraints within a set.
    /// Doesn't do anything if the set is awake (index zero).
    pub fn awaken_set(&mut self, set_index: i32) {
        if set_index > 0 {
            let pool = self.pool();
            let mut list = QuickList::with_capacity(1, pool);
            list.add_unsafely(set_index);
            unsafe {
                self.awaken_sets(&mut list, None);
            }
            list.dispose(self.pool());
        }
    }

    /// Awakens a list of set indices.
    pub unsafe fn awaken_sets(
        &mut self,
        set_indices: &mut QuickList<i32>,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        let pool = self.pool();
        let bodies = self.bodies();
        let mut unique_set_indices = QuickList::with_capacity(set_indices.count, pool);
        let mut unique_set = IndexSet::new(pool, bodies.sets.len());
        self.accumulate_unique_indices(set_indices, &mut unique_set, &mut unique_set_indices);
        unique_set.dispose(pool);

        let thread_count = match thread_dispatcher {
            Some(d) => d.thread_count(),
            None => 1,
        };

        let (phase_one_job_count, phase_two_job_count) =
            self.prepare_jobs(&mut unique_set_indices, true, thread_count);

        // Use raw pointer to self to avoid borrow conflicts between execute/dispose
        let self_ptr = self as *mut Self;

        if thread_count > 1 {
            let dispatcher = thread_dispatcher.unwrap();
            // Phase one: MT dispatch
            *(*self_ptr).job_index.get() = -1;
            (*self_ptr).job_count = phase_one_job_count;
            dispatcher.dispatch_workers(
                Self::phase_one_worker_fn,
                phase_one_job_count,
                self_ptr as *mut (),
                None,
            );

            // Phase two: MT dispatch
            *(*self_ptr).job_index.get() = -1;
            (*self_ptr).job_count = phase_two_job_count;
            dispatcher.dispatch_workers(
                Self::phase_two_worker_fn,
                phase_two_job_count,
                self_ptr as *mut (),
                None,
            );
        } else {
            // Execute phase one jobs
            for i in 0..phase_one_job_count {
                (*self_ptr).execute_phase_one_job(i);
            }

            // Execute phase two jobs
            for i in 0..phase_two_job_count {
                (*self_ptr).execute_phase_two_job(i);
            }
        }

        (*self_ptr).dispose_for_completed_awakenings(&mut unique_set_indices);
        let pool = (*self_ptr).pool();
        unique_set_indices.dispose(pool);
    }

    pub(crate) fn accumulate_unique_indices(
        &self,
        candidate_set_indices: &QuickList<i32>,
        unique_set: &mut IndexSet,
        unique_set_indices: &mut QuickList<i32>,
    ) {
        let pool = self.pool();
        for i in 0..candidate_set_indices.count {
            let candidate = *candidate_set_indices.get(i);
            if !unique_set.contains(candidate) {
                unique_set.add_unsafely(candidate);
                *unique_set_indices.allocate_unsafely() = candidate;
            }
        }
        if unique_set_indices.count > 0 {
            // Sort to ensure deterministic order (Rust sort_unstable maps to C# QuickSort.Sort)
            let slice = unsafe {
                std::slice::from_raw_parts_mut(
                    unique_set_indices.span.get_ptr(0) as *mut i32,
                    unique_set_indices.count as usize,
                )
            };
            slice.sort_unstable();
        }
    }

    /// Worker function for MT phase one dispatch.
    fn phase_one_worker_fn(_worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let awakener = &mut *(dispatcher.unmanaged_context() as *mut IslandAwakener);
            loop {
                let index = AtomicI32::from_ptr(awakener.job_index.get())
                    .fetch_add(1, Ordering::AcqRel)
                    + 1;
                if index >= awakener.job_count {
                    break;
                }
                awakener.execute_phase_one_job(index);
            }
        }
    }

    /// Worker function for MT phase two dispatch.
    fn phase_two_worker_fn(_worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let awakener = &mut *(dispatcher.unmanaged_context() as *mut IslandAwakener);
            loop {
                let index = AtomicI32::from_ptr(awakener.job_index.get())
                    .fetch_add(1, Ordering::AcqRel)
                    + 1;
                if index >= awakener.job_count {
                    break;
                }
                awakener.execute_phase_two_job(index);
            }
        }
    }

    pub(crate) unsafe fn execute_phase_one_job(&self, index: i32) {
        let job = *self.phase_one_jobs.get(index);
        match job.job_type {
            PhaseOneJobType::PairCache => {
                // Update pair cache for awakened sets
                let pair_cache = &mut *self.pair_cache;
                for i in 0..self.unique_set_indices.count {
                    let set_index = *self.unique_set_indices.get(i);
                    pair_cache.awaken_set(set_index);
                }
            }
            PhaseOneJobType::UpdateBatchReferencedHandles => {
                let solver = self.solver_mut();
                let batch_index = job.batch_index;
                let target = solver.batch_referenced_handles.get_mut(batch_index);
                for i in 0..self.unique_set_indices.count {
                    let set_index = *self.unique_set_indices.get(i);
                    let source_set = solver.sets.get(set_index);
                    if source_set.batches.count > batch_index {
                        let batch = source_set.batches.get(batch_index);
                        for type_batch_index in 0..batch.type_batches.count {
                            let type_batch = batch.type_batches.get(type_batch_index);
                            let tp = solver.type_processors[type_batch.type_id as usize]
                                .as_ref()
                                .unwrap();
                            tp.inner()
                                .add_waking_body_handles_to_batch_references(type_batch, target);
                        }
                    }
                }
            }
            PhaseOneJobType::MoveFallbackBatchBodies => {
                // Move fallback batch body tracking from sleeping sets to active set.
                // Inactive sets store body handles; active set stores body indices. Make the transition.
                let solver = self.solver_mut();
                let solver_ptr = solver as *mut Solver;
                let bodies = &*self.bodies;
                for i in 0..self.unique_set_indices.count {
                    let set_index = *self.unique_set_indices.get(i);
                    debug_assert!(set_index > 0);
                    let source_fallback = &(*solver_ptr).sets.get(set_index).sequential_fallback;
                    if source_fallback.dynamic_body_constraint_counts.count > 0 {
                        let source_count = source_fallback.dynamic_body_constraint_counts.count;
                        // Collect source data first to avoid borrow conflicts
                        for j in 0..source_count {
                            // Inactive set keys are body handles; translate to body indices via HandleToLocation.
                            // HandleToLocation was updated during job setup, so indices point to the active set.
                            let body_handle = *(*solver_ptr)
                                .sets
                                .get(set_index)
                                .sequential_fallback
                                .dynamic_body_constraint_counts
                                .key_at(j);
                            let value = *(*solver_ptr)
                                .sets
                                .get(set_index)
                                .sequential_fallback
                                .dynamic_body_constraint_counts
                                .value_at(j);
                            let body_location = bodies.handle_to_location.get(body_handle);
                            debug_assert!(body_location.set_index == 0,
                                "Any batch moved into active set should deal with bodies already in the active set.");
                            (*solver_ptr)
                                .active_set_mut()
                                .sequential_fallback
                                .dynamic_body_constraint_counts
                                .add_unsafely(body_location.index, value);
                        }
                    }
                }
            }
            PhaseOneJobType::CopyBodyRegion => {
                let bodies = &mut *self.bodies;
                let source_set_ptr = bodies.sets.get(job.source_set) as *const BodySet;
                let target_set = bodies.active_set_mut();

                // Copy body data from sleeping set to active set
                (*source_set_ptr).collidables.copy_to(
                    job.source_start,
                    &mut target_set.collidables,
                    job.target_start,
                    job.count,
                );
                (*source_set_ptr).constraints.copy_to(
                    job.source_start,
                    &mut target_set.constraints,
                    job.target_start,
                    job.count,
                );
                (*source_set_ptr).dynamics_state.copy_to(
                    job.source_start,
                    &mut target_set.dynamics_state,
                    job.target_start,
                    job.count,
                );

                // This rescans the memory, but it should be still floating in cache ready to access.
                let source_set = &*source_set_ptr;
                let solver = &mut *self.solver;
                for i in 0..job.count {
                    let source_body_index = i + job.source_start;
                    if Bodies::is_kinematic_unsafe_gc_hole(
                        &source_set
                            .dynamics_state
                            .get(source_body_index)
                            .inertia
                            .local,
                    ) && source_set.constraints.get(source_body_index).count > 0
                    {
                        // SpinLock: acquire
                        while solver
                            .constrained_kinematic_lock
                            .compare_exchange_weak(
                                false,
                                true,
                                std::sync::atomic::Ordering::Acquire,
                                std::sync::atomic::Ordering::Relaxed,
                            )
                            .is_err()
                        {
                            std::hint::spin_loop();
                        }
                        solver
                            .constrained_kinematic_handles
                            .add_unsafely(&source_set.index_to_handle.get(source_body_index).0);
                        // SpinLock: release
                        solver
                            .constrained_kinematic_lock
                            .store(false, std::sync::atomic::Ordering::Release);
                    }
                }

                (*source_set_ptr).activity.copy_to(
                    job.source_start,
                    &mut target_set.activity,
                    job.target_start,
                    job.count,
                );
                (*source_set_ptr).index_to_handle.copy_to(
                    job.source_start,
                    &mut target_set.index_to_handle,
                    job.target_start,
                    job.count,
                );

                if self.reset_activity_states {
                    // Reset activity states for woken bodies
                    for target_index in job.target_start..(job.target_start + job.count) {
                        let activity = target_set.activity.get_mut(target_index);
                        activity.timesteps_under_threshold_count = 0;
                        activity.sleep_candidate = false;
                    }
                }
            }
        }
    }

    pub(crate) unsafe fn execute_phase_two_job(&self, index: i32) {
        let job = &*self.phase_two_jobs.get(index);
        match job.job_type {
            PhaseTwoJobType::BroadPhase => {
                // Move collidables from static tree back to active tree.
                // The broad phase add/remove has a dependency on the body copies (hence phase two).
                let bodies = &mut *self.bodies;
                let broad_phase = &mut *self.broad_phase;
                let statics = &mut *self.statics;

                for i in 0..self.unique_set_indices.count {
                    let set_index = *self.unique_set_indices.get(i);
                    // Use raw pointer to sleeping set to avoid borrow conflict with active set.
                    let sleeping_set_ptr = bodies.sets.get(set_index) as *const BodySet;
                    let sleeping_count = (*sleeping_set_ptr).count;
                    for j in 0..sleeping_count {
                        let handle_value = (*sleeping_set_ptr).index_to_handle.get(j).0;
                        let body_location = *bodies.handle_to_location.get(handle_value);
                        debug_assert!(body_location.set_index == 0);
                        // The broad phase index value is currently the static index.
                        let collidable = bodies
                            .active_set_mut()
                            .collidables
                            .get_mut(body_location.index);
                        let broad_phase_index = collidable.broad_phase_index;
                        if broad_phase_index >= 0 {
                            // Get bounds from static tree.
                            let (min_ptr, max_ptr) =
                                broad_phase.get_static_bounds_pointers(broad_phase_index);
                            let min = *min_ptr;
                            let max = *max_ptr;
                            let leaf = *broad_phase.static_leaves.get(broad_phase_index);
                            // Add to active tree.
                            collidable.broad_phase_index = broad_phase.add_active(leaf, &min, &max);

                            // Remove from static tree.
                            let static_index_to_remove = broad_phase_index;
                            let mut moved_leaf = CollidableReference::default();
                            if broad_phase.remove_static_at(static_index_to_remove, &mut moved_leaf)
                            {
                                if moved_leaf.mobility() == CollidableMobility::Static {
                                    statics
                                        .get_direct_reference_mut(moved_leaf.static_handle())
                                        .broad_phase_index = static_index_to_remove;
                                } else {
                                    bodies.update_collidable_broad_phase_index(
                                        moved_leaf.body_handle(),
                                        static_index_to_remove,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            PhaseTwoJobType::CopyConstraintRegion => {
                let j = &job.copy_constraint_region;
                let solver = self.solver_mut();
                let type_proc = solver.type_processors[j.type_id as usize].as_ref().unwrap();
                type_proc.inner().copy_sleeping_to_active(
                    j.source_set,
                    j.batch,
                    j.source_type_batch,
                    j.target_type_batch,
                    j.source_start,
                    j.target_start,
                    j.count,
                    &*self.bodies,
                    solver,
                );
            }
            PhaseTwoJobType::AddFallbackTypeBatchConstraints => {
                let solver = self.solver_mut();
                let bodies = &*self.bodies;
                for i in 0..job.fallback_sources.len() {
                    let source = *job.fallback_sources.get(i);
                    let type_proc = solver.type_processors[job.fallback_type_id as usize]
                        .as_ref()
                        .unwrap();
                    type_proc.inner().add_sleeping_to_active_for_fallback(
                        source.source_set,
                        source.source_type_batch_index,
                        job.fallback_target_type_batch,
                        bodies,
                        solver,
                    );
                }
            }
        }
    }

    pub(crate) unsafe fn prepare_jobs(
        &mut self,
        set_indices: &mut QuickList<i32>,
        reset_activity_states: bool,
        _thread_count: i32,
    ) -> (i32, i32) {
        if set_indices.count == 0 {
            return (0, 0);
        }
        self.unique_set_indices = set_indices.clone();
        self.reset_activity_states = reset_activity_states;

        let pool = unsafe { &mut *self.pool };
        let solver = unsafe { &mut *self.solver };
        let bodies = unsafe { &*self.bodies };

        // Count new bodies and determine batch requirements
        let mut new_body_count = 0i32;
        let mut highest_new_batch_count = 0i32;
        let mut highest_required_type_capacity = 0i32;
        let mut additional_required_fallback_capacity = 0i32;
        for i in 0..set_indices.count {
            let set_index = *set_indices.get(i);
            new_body_count += bodies.sets.get(set_index).count;
            let set_batch_count = solver.sets.get(set_index).batches.count;
            if highest_new_batch_count < set_batch_count {
                highest_new_batch_count = set_batch_count;
            }
            let constraint_set = solver.sets.get(set_index);
            additional_required_fallback_capacity +=
                constraint_set.sequential_fallback.body_count();
            for batch_index in 0..constraint_set.batches.count {
                let batch = constraint_set.batches.get(batch_index);
                for type_batch_index in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(type_batch_index);
                    if highest_required_type_capacity < type_batch.type_id {
                        highest_required_type_capacity = type_batch.type_id;
                    }
                }
            }
        }
        // We accumulated type IDs above; add one to get the capacity requirement.
        highest_required_type_capacity += 1;

        // Build per-batch per-type constraint counts.
        // TypeAllocationSizes: Buffer<i32> of per-type counts + highest occupied type index.
        struct TypeAllocationSizes {
            type_counts: Buffer<i32>,
            highest_occupied_type_index: i32,
        }
        impl TypeAllocationSizes {
            unsafe fn new(pool: &mut BufferPool, max_type_count: i32) -> Self {
                let mut type_counts: Buffer<i32> = pool.take_at_least(max_type_count);
                type_counts.clear(0, max_type_count);
                Self {
                    type_counts,
                    highest_occupied_type_index: 0,
                }
            }
            fn add(&mut self, type_id: i32, count: i32) {
                *self.type_counts.get_mut(type_id) += count;
                if type_id > self.highest_occupied_type_index {
                    self.highest_occupied_type_index = type_id;
                }
            }
            fn dispose(&mut self, pool: &mut BufferPool) {
                pool.return_buffer(&mut self.type_counts);
            }
        }

        let mut constraint_count_per_type_per_batch: Buffer<TypeAllocationSizes> =
            pool.take_at_least(highest_new_batch_count);
        for batch_index in 0..highest_new_batch_count {
            *constraint_count_per_type_per_batch.get_mut(batch_index) =
                TypeAllocationSizes::new(pool, highest_required_type_capacity);
        }

        let pair_cache = &*self.pair_cache;
        let mut new_pair_count = 0i32;
        for i in 0..set_indices.count {
            let set_index = *set_indices.get(i);
            let constraint_set = solver.sets.get(set_index);
            for batch_index in 0..constraint_set.batches.count {
                let constraint_count_per_type =
                    constraint_count_per_type_per_batch.get_mut(batch_index);
                let batch = constraint_set.batches.get(batch_index);
                for type_batch_index in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(type_batch_index);
                    constraint_count_per_type.add(type_batch.type_id, type_batch.constraint_count);
                }
            }
            let source_set = pair_cache.sleeping_sets.get(set_index);
            new_pair_count += source_set.pairs.count;
        }

        // Ensure capacities on all systems.
        let bodies_mut = &mut *self.bodies;
        bodies_mut.ensure_capacity(bodies_mut.active_set().count + new_body_count);
        solver.constrained_kinematic_handles.ensure_capacity(
            solver.constrained_kinematic_handles.count + new_body_count,
            pool,
        );
        let broad_phase = &mut *self.broad_phase;
        broad_phase.ensure_capacity(
            broad_phase.active_tree.leaf_count + new_body_count,
            broad_phase.static_tree.leaf_count,
        );
        solver
            .active_set_mut()
            .batches
            .ensure_capacity(highest_new_batch_count, pool);
        if additional_required_fallback_capacity > 0 {
            let current_body_count = solver.active_set().sequential_fallback.body_count();
            solver.active_set_mut().sequential_fallback.ensure_capacity(
                current_body_count + additional_required_fallback_capacity,
                pool,
            );
        }
        debug_assert!(
            highest_new_batch_count <= solver.fallback_batch_threshold() + 1,
            "Shouldn't have any batches beyond the fallback batch."
        );
        solver
            .batch_referenced_handles
            .ensure_capacity(highest_new_batch_count, pool);
        // Create new batches if needed.
        let solver_ptr = solver as *mut Solver;
        for batch_index in (*solver_ptr).active_set().batches.count..highest_new_batch_count {
            *(*solver_ptr).active_set_mut().batches.allocate_unsafely() =
                ConstraintBatch::new(pool, 16);
            *(*solver_ptr).batch_referenced_handles.allocate_unsafely() = IndexSet::new(
                pool,
                bodies_mut.handle_pool.highest_possibly_claimed_id() + 1,
            );
        }
        let fallback_threshold = (*solver_ptr).fallback_batch_threshold();
        for batch_index in 0..highest_new_batch_count {
            let constraint_count_per_type =
                constraint_count_per_type_per_batch.get_mut(batch_index);
            let batch = (*solver_ptr).active_set_mut().batches.get_mut(batch_index);
            batch.ensure_type_map_size(pool, constraint_count_per_type.highest_occupied_type_index);
            (*solver_ptr)
                .batch_referenced_handles
                .get_mut(batch_index)
                .ensure_capacity(
                    bodies_mut.handle_pool.highest_possibly_claimed_id() + 1,
                    pool,
                );
            for type_id in 0..=constraint_count_per_type.highest_occupied_type_index {
                let mut count_for_type = *constraint_count_per_type.type_counts.get(type_id);
                // The fallback batch must allocate worst case: every new constraint needs its own bundle.
                if batch_index == fallback_threshold {
                    count_for_type *= crate::utilities::vector::VECTOR_WIDTH as i32;
                }
                if count_for_type > 0 {
                    // Avoid implicit autoref on raw pointer dereference.
                    let tp_vec = std::ptr::addr_of!((*solver_ptr).type_processors);
                    let tp_slice = std::slice::from_raw_parts((*tp_vec).as_ptr(), (*tp_vec).len());
                    let type_processor = tp_slice[type_id as usize].as_ref().unwrap();
                    let type_batch = batch.get_or_create_type_batch(
                        type_id,
                        type_processor.inner(),
                        count_for_type,
                        pool,
                    );
                    let target_capacity = count_for_type + (*type_batch).constraint_count;
                    if target_capacity > (*type_batch).index_to_handle.len() {
                        type_processor
                            .inner()
                            .resize(&mut *type_batch, target_capacity, pool);
                    }
                }
            }
            constraint_count_per_type.dispose(pool);
        }
        pool.return_buffer(&mut constraint_count_per_type_per_batch);
        // Ensure pair cache mapping capacity.
        let pair_cache_mut = &mut *self.pair_cache;
        pair_cache_mut
            .mapping
            .ensure_capacity(pair_cache_mut.mapping.count + new_pair_count, pool);

        // Create phase one jobs
        self.phase_one_jobs = QuickList::with_capacity(32.max(highest_new_batch_count + 2), pool);

        // Pair cache job
        {
            let job = self.phase_one_jobs.allocate_unsafely();
            job.job_type = PhaseOneJobType::PairCache;
            job.batch_index = 0;
            job.source_set = 0;
            job.source_start = 0;
            job.target_start = 0;
            job.count = 0;
        }

        // MoveFallbackBatchBodies job
        {
            let job = self.phase_one_jobs.allocate_unsafely();
            job.job_type = PhaseOneJobType::MoveFallbackBatchBodies;
            job.batch_index = 0;
            job.source_set = 0;
            job.source_start = 0;
            job.target_start = 0;
            job.count = 0;
        }

        // UpdateBatchReferencedHandles jobs (one per batch)
        for batch_index in 0..highest_new_batch_count {
            let job = self.phase_one_jobs.allocate_unsafely();
            job.job_type = PhaseOneJobType::UpdateBatchReferencedHandles;
            job.batch_index = batch_index;
            job.source_set = 0;
            job.source_start = 0;
            job.target_start = 0;
            job.count = 0;
        }

        // Create phase two jobs
        self.phase_two_jobs = QuickList::with_capacity(32, pool);

        // BroadPhase job
        {
            let job = self.phase_two_jobs.allocate_unsafely();
            job.job_type = PhaseTwoJobType::BroadPhase;
        }

        // Multiple source sets can contribute to the same target type batch in the fallback.
        // Track those as we enumerate sets so we can create a single job for each target after the loop.
        let mut target_fallback_type_batches_to_sources: QuickDictionary<
            i32,
            QuickList<FallbackAddSource>,
            PrimitiveComparer<i32>,
        > = if highest_new_batch_count > solver.fallback_batch_threshold() {
            QuickDictionary::with_capacity(8, 3, pool, PrimitiveComparer::<i32>::default())
        } else {
            QuickDictionary::default()
        };

        // CopyBodyRegion jobs + CopyConstraintRegion jobs
        let bodies_mut = &mut *self.bodies;
        let active_set_ptr = bodies_mut.active_set_mut() as *mut BodySet;
        for i in 0..set_indices.count {
            let source_set_index = *set_indices.get(i);

            // Body copy jobs
            {
                const BODY_JOB_SIZE: i32 = 64;
                let source_set = bodies_mut.sets.get(source_set_index);
                let source_count = source_set.count;
                let set_job_count = 1.max(source_count / BODY_JOB_SIZE);
                let base_per_job = source_count / set_job_count;
                let remainder = source_count - base_per_job * set_job_count;
                let mut previous_source_end = 0;

                let phase_one_jobs =
                    &mut *(&mut self.phase_one_jobs as *mut QuickList<PhaseOneJob>);
                phase_one_jobs.ensure_capacity(phase_one_jobs.count + set_job_count, pool);
                for job_index in 0..set_job_count {
                    let job = phase_one_jobs.allocate_unsafely();
                    job.job_type = PhaseOneJobType::CopyBodyRegion;
                    job.source_set = source_set_index;
                    job.source_start = previous_source_end;
                    job.target_start = (*active_set_ptr).count;
                    job.count = if job_index < remainder {
                        base_per_job + 1
                    } else {
                        base_per_job
                    };
                    previous_source_end += job.count;
                    (*active_set_ptr).count += job.count;

                    // Update HandleToLocation up front — MoveFallbackBatchBodies and the narrow phase
                    // flush depend on it being current before phase one executes.
                    let source_set = bodies_mut.sets.get(source_set_index);
                    for j in 0..job.count {
                        let source_index = job.source_start + j;
                        let target_index = job.target_start + j;
                        let handle_value = source_set.index_to_handle.get(source_index).0;
                        let body_location = bodies_mut.handle_to_location.get_mut(handle_value);
                        body_location.set_index = 0;
                        body_location.index = target_index;
                    }
                }
            }

            // Constraint copy jobs
            {
                const CONSTRAINT_JOB_SIZE: i32 = 32;
                let solver_ptr = solver as *mut Solver;
                let source_constraint_set =
                    (*solver_ptr).sets.get(source_set_index) as *const ConstraintSet;
                let fallback_index = (*solver_ptr).fallback_batch_threshold();
                let active_solver_set = (*solver_ptr).active_set_mut() as *mut ConstraintSet;

                let sync_batch_count = if (*source_constraint_set).batches.count > fallback_index {
                    fallback_index
                } else {
                    (*source_constraint_set).batches.count
                };

                for batch_index in 0..sync_batch_count {
                    let source_batch = (*source_constraint_set).batches.get(batch_index);
                    let target_batch = (*active_solver_set).batches.get_mut(batch_index);
                    for source_type_batch_index in 0..source_batch.type_batches.count {
                        let source_type_batch =
                            source_batch.type_batches.get(source_type_batch_index);
                        let target_type_batch_index = *target_batch
                            .type_index_to_type_batch_index
                            .get(source_type_batch.type_id);
                        let target_type_batch =
                            target_batch.type_batches.get_mut(target_type_batch_index);
                        let job_count =
                            1.max(source_type_batch.constraint_count / CONSTRAINT_JOB_SIZE);
                        let base = source_type_batch.constraint_count / job_count;
                        let rem = source_type_batch.constraint_count - base * job_count;
                        let mut prev_end = 0;

                        let phase_two_jobs =
                            &mut *(&mut self.phase_two_jobs as *mut QuickList<PhaseTwoJob>);
                        phase_two_jobs.ensure_capacity(phase_two_jobs.count + job_count, pool);
                        for j in 0..job_count {
                            let count = if j < rem { base + 1 } else { base };
                            let job = phase_two_jobs.allocate_unsafely();
                            job.job_type = PhaseTwoJobType::CopyConstraintRegion;
                            job.copy_constraint_region = CopyConstraintRegionJob {
                                type_id: source_type_batch.type_id,
                                batch: batch_index,
                                source_set: source_set_index,
                                source_type_batch: source_type_batch_index,
                                target_type_batch: target_type_batch_index,
                                count,
                                source_start: prev_end,
                                target_start: target_type_batch.constraint_count,
                            };
                            prev_end += count;
                            let old_bundle_count = target_type_batch.bundle_count();
                            target_type_batch.constraint_count += count;
                            if target_type_batch.bundle_count() != old_bundle_count {
                                // A new bundle was created; guarantee trailing slots are -1.
                                let bodies_per_constraint = (&(*solver_ptr).type_processors)
                                    [source_type_batch.type_id as usize]
                                    .as_ref()
                                    .unwrap()
                                    .bodies_per_constraint;
                                let vector_size = std::mem::size_of::<Vector<i32>>();
                                let bundle_start = target_type_batch.body_references.as_ptr().add(
                                    (target_type_batch.bundle_count() - 1)
                                        * bodies_per_constraint as usize
                                        * vector_size,
                                )
                                    as *mut Vector<i32>;
                                let neg_one = Vector::<i32>::splat(-1);
                                for vi in 0..bodies_per_constraint as usize {
                                    *bundle_start.add(vi) = neg_one;
                                }
                            }
                        }
                    }
                }

                // Fallback batch handling
                if (*source_constraint_set).batches.count > fallback_index {
                    let source_batch = (*source_constraint_set).batches.get(fallback_index);
                    let target_batch = (*active_solver_set).batches.get_mut(fallback_index);
                    for source_type_batch_index in 0..source_batch.type_batches.count {
                        let source_type_batch =
                            source_batch.type_batches.get(source_type_batch_index);
                        let target_type_batch_index = *target_batch
                            .type_index_to_type_batch_index
                            .get(source_type_batch.type_id);
                        let mut slot_index = 0i32;
                        target_fallback_type_batches_to_sources.ensure_capacity(
                            target_fallback_type_batches_to_sources.count + 1,
                            pool,
                        );
                        if !target_fallback_type_batches_to_sources.find_or_allocate_slot_unsafely(
                            &target_type_batch_index,
                            &mut slot_index,
                        ) {
                            *target_fallback_type_batches_to_sources.value_at_mut(slot_index) =
                                QuickList::with_capacity(8, pool);
                        }
                        let sources_list =
                            target_fallback_type_batches_to_sources.value_at_mut(slot_index);
                        sources_list.ensure_capacity(sources_list.count + 1, pool);
                        *sources_list.allocate_unsafely() = FallbackAddSource {
                            source_set: source_set_index,
                            source_type_batch_index,
                        };
                    }
                }
            }
        }

        // Create fallback jobs from accumulated dictionary
        if target_fallback_type_batches_to_sources.keys.allocated() {
            let phase_two_jobs = &mut *(&mut self.phase_two_jobs as *mut QuickList<PhaseTwoJob>);
            phase_two_jobs.ensure_capacity(
                phase_two_jobs.count + target_fallback_type_batches_to_sources.count,
                pool,
            );
            for i in 0..target_fallback_type_batches_to_sources.count {
                let job = phase_two_jobs.allocate_unsafely();
                job.job_type = PhaseTwoJobType::AddFallbackTypeBatchConstraints;
                let target_tb_index = *target_fallback_type_batches_to_sources.key_at(i);
                let list = target_fallback_type_batches_to_sources.value_at(i);
                // Slice the span to create a buffer of just the used portion
                job.fallback_sources = list.span.slice_count(list.count);
                job.fallback_target_type_batch = target_tb_index;
                let active_solver_set = solver.active_set();
                let fallback_batch = active_solver_set
                    .batches
                    .get(solver.fallback_batch_threshold());
                job.fallback_type_id = fallback_batch.type_batches.get(target_tb_index).type_id;
            }
            // Dispose the dictionary container, but NOT the per-target lists —
            // the spans are referenced by the phase two jobs and disposed in dispose_for_completed_awakenings.
            target_fallback_type_batches_to_sources.dispose(pool);
        }

        (self.phase_one_jobs.count, self.phase_two_jobs.count)
    }

    pub(crate) unsafe fn dispose_for_completed_awakenings(
        &mut self,
        set_indices: &mut QuickList<i32>,
    ) {
        let pool = &mut *self.pool;
        let bodies = &mut *self.bodies;
        let solver = &mut *self.solver;

        for i in 0..set_indices.count {
            let set_index = *set_indices.get(i);
            // Dispose body set
            bodies.sets.get_mut(set_index).dispose_buffers(pool);
            // Dispose constraint set if allocated
            if solver.sets.get(set_index).allocated() {
                solver.sets.get_mut(set_index).dispose(pool);
            }
            // Dispose pair cache sleeping set if allocated
            let pair_cache = &mut *self.pair_cache;
            let sleeping_set = pair_cache.sleeping_sets.get_mut(set_index);
            if sleeping_set.allocated() {
                sleeping_set.dispose(pool);
            }
            // Return the set id to the sleeper
            (&mut *self.sleeper).return_set_id(set_index);
        }

        if self.phase_one_jobs.allocated() {
            let phase_one = &mut *(&mut self.phase_one_jobs as *mut QuickList<PhaseOneJob>);
            phase_one.dispose(pool);
            // Dispose fallback sources in phase two jobs
            let phase_two = &mut *(&mut self.phase_two_jobs as *mut QuickList<PhaseTwoJob>);
            for i in 0..phase_two.count {
                let job = phase_two.get_mut(i);
                if matches!(
                    job.job_type,
                    PhaseTwoJobType::AddFallbackTypeBatchConstraints
                ) {
                    if job.fallback_sources.allocated() {
                        pool.return_buffer(&mut job.fallback_sources);
                    }
                }
            }
            phase_two.dispose(pool);
        }
    }
}

unsafe impl Send for IslandAwakener {}
unsafe impl Sync for IslandAwakener {}
