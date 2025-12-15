// Translated from BepuPhysics/IslandAwakener.cs

use crate::physics::bodies::Bodies;
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::island_scaffold::IslandScaffold;
use crate::physics::solver::{PrimitiveComparer, Solver};
use crate::physics::statics::Statics;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::sync::atomic::{AtomicI32, Ordering};

/// Provides functionality for efficiently waking up sleeping bodies.
pub struct IslandAwakener {
    solver: *mut Solver,
    statics: *mut Statics,
    bodies: *mut Bodies,
    broad_phase: *mut BroadPhase,
    sleeper: *mut IslandSleeper,
    pool: *mut BufferPool,

    // TODO: pub(crate) pair_cache: *mut PairCache,

    // Worker dispatch state
    job_index: AtomicI32,
    job_count: i32,

    reset_activity_states: bool,
    unique_set_indices: QuickList<i32>,
    phase_one_jobs: QuickList<PhaseOneJob>,
    phase_two_jobs: QuickList<PhaseTwoJob>,
}

// Forward declarations for types not yet fully translated
pub struct BroadPhase {
    _opaque: [u8; 0],
}

pub struct IslandSleeper {
    _opaque: [u8; 0],
}

impl IslandSleeper {
    pub unsafe fn return_set_id(&self, _id: i32) {
        // TODO: implement
    }
}

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
        pool: *mut BufferPool,
    ) -> Self {
        Self {
            bodies,
            statics,
            solver,
            broad_phase,
            sleeper,
            pool,
            job_index: AtomicI32::new(-1),
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
    pub fn awaken_body(&self, body_handle: BodyHandle) {
        let bodies = self.bodies();
        bodies.validate_existing_handle(body_handle);
        let set_index = unsafe { bodies.handle_to_location.get(body_handle.0) }.set_index;
        self.awaken_set(set_index);
    }

    /// Wakes up any sleeping bodies associated with a constraint.
    pub fn awaken_constraint(&self, constraint_handle: ConstraintHandle) {
        let set_index = unsafe { self.solver().handle_to_constraint.get(constraint_handle.0) }.set_index;
        self.awaken_set(set_index);
    }

    /// Wakes up all bodies and constraints within a set.
    /// Doesn't do anything if the set is awake (index zero).
    pub fn awaken_set(&self, set_index: i32) {
        if set_index > 0 {
            let pool = self.pool();
            let mut list = QuickList::with_capacity(1, pool);
            list.add_unsafely(set_index);
            // TODO: self.awaken_sets(&mut list, None);
            list.dispose(pool);
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

        // Execute phase one jobs
        for i in 0..phase_one_job_count {
            (*self_ptr).execute_phase_one_job(i);
        }

        // Execute phase two jobs
        for i in 0..phase_two_job_count {
            (*self_ptr).execute_phase_two_job(i);
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
            // Sort to ensure deterministic order
            // TODO: QuickSort.sort with PrimitiveComparer<i32>
            let slice = unsafe {
                std::slice::from_raw_parts_mut(
                    unique_set_indices.span.get_ptr(0) as *mut i32,
                    unique_set_indices.count as usize,
                )
            };
            slice.sort_unstable();
        }
    }

    pub(crate) unsafe fn execute_phase_one_job(&self, index: i32) {
        let job = *self.phase_one_jobs.get(index);
        match job.job_type {
            PhaseOneJobType::PairCache => {
                // TODO: Update pair cache for awakened sets
                // for i in 0..self.unique_set_indices.count {
                //     self.pair_cache.awaken_set(self.unique_set_indices[i]);
                // }
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
                // TODO: Move fallback batch bodies from sleeping to active
            }
            PhaseOneJobType::CopyBodyRegion => {
                let bodies = &mut *self.bodies;
                let source_set = bodies.sets.get(job.source_set);
                let target_set = bodies.active_set_mut();

                // Copy body data from sleeping set to active set
                // source_set.collidables.copy_to(job.source_start, target_set.collidables, job.target_start, job.count);
                // source_set.constraints.copy_to(...);
                // source_set.dynamics_state.copy_to(...);
                // source_set.activity.copy_to(...);
                // source_set.index_to_handle.copy_to(...);

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
                // TODO: Move collidables from static tree back to active tree
            }
            PhaseTwoJobType::CopyConstraintRegion => {
                let j = &job.copy_constraint_region;
                let solver = self.solver_mut();
                let type_proc = solver.type_processors[j.type_id as usize]
                    .as_ref()
                    .unwrap();
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
                // TODO: Add sleeping fallback constraints to active set
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
        for i in 0..set_indices.count {
            let set_index = *set_indices.get(i);
            new_body_count += bodies.sets.get(set_index).count;
            let set_batch_count = solver.sets.get(set_index).batches.count;
            if highest_new_batch_count < set_batch_count {
                highest_new_batch_count = set_batch_count;
            }
        }

        // Ensure capacities
        // bodies.ensure_capacity(bodies.active_set().count + new_body_count);

        // Create phase one jobs
        self.phase_one_jobs = QuickList::with_capacity(
            32.max(highest_new_batch_count + 2),
            pool,
        );

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

        // CopyBodyRegion jobs + CopyConstraintRegion jobs
        let active_body_count = bodies.active_set().count;
        for i in 0..set_indices.count {
            let source_set_index = *set_indices.get(i);

            // Body copy jobs
            const BODY_JOB_SIZE: i32 = 64;
            let source_count = bodies.sets.get(source_set_index).count;
            let set_job_count = 1.max(source_count / BODY_JOB_SIZE);
            let base_per_job = source_count / set_job_count;
            let remainder = source_count - base_per_job * set_job_count;
            let mut previous_source_end = 0;

            let phase_one_jobs = &mut *(&mut self.phase_one_jobs as *mut QuickList<PhaseOneJob>);
            phase_one_jobs.ensure_capacity(
                phase_one_jobs.count + set_job_count,
                pool,
            );
            for job_index in 0..set_job_count {
                let job = phase_one_jobs.allocate_unsafely();
                job.job_type = PhaseOneJobType::CopyBodyRegion;
                job.source_set = source_set_index;
                job.source_start = previous_source_end;
                job.target_start = active_body_count;
                job.count = if job_index < remainder {
                    base_per_job + 1
                } else {
                    base_per_job
                };
                previous_source_end += job.count;
                // Note: in full implementation, active_set.count is incremented here
            }

            // Constraint copy jobs
            const CONSTRAINT_JOB_SIZE: i32 = 32;
            let source_constraint_set = solver.sets.get(source_set_index);
            let fallback_index = solver.fallback_batch_threshold();

            let sync_batch_count = if source_constraint_set.batches.count > fallback_index {
                fallback_index
            } else {
                source_constraint_set.batches.count
            };

            for batch_index in 0..sync_batch_count {
                let source_batch = source_constraint_set.batches.get(batch_index);
                for source_type_batch_index in 0..source_batch.type_batches.count {
                    let source_type_batch = source_batch.type_batches.get(source_type_batch_index);
                    let job_count = 1.max(source_type_batch.constraint_count / CONSTRAINT_JOB_SIZE);
                    let base = source_type_batch.constraint_count / job_count;
                    let rem = source_type_batch.constraint_count - base * job_count;
                    let mut prev_end = 0;

                    let phase_two_jobs = &mut *(&mut self.phase_two_jobs as *mut QuickList<PhaseTwoJob>);
                    phase_two_jobs
                        .ensure_capacity(phase_two_jobs.count + job_count, pool);
                    for j in 0..job_count {
                        let count = if j < rem { base + 1 } else { base };
                        let job = phase_two_jobs.allocate_unsafely();
                        job.job_type = PhaseTwoJobType::CopyConstraintRegion;
                        job.copy_constraint_region = CopyConstraintRegionJob {
                            type_id: source_type_batch.type_id,
                            batch: batch_index,
                            source_set: source_set_index,
                            source_type_batch: source_type_batch_index,
                            target_type_batch: 0, // TODO: look up target
                            count,
                            source_start: prev_end,
                            target_start: 0, // TODO: look up target
                        };
                        prev_end += count;
                    }
                }
            }
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
            // TODO: dispose pair cache set if allocated
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
                if matches!(job.job_type, PhaseTwoJobType::AddFallbackTypeBatchConstraints) {
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
