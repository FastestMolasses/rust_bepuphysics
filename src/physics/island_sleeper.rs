// Translated from BepuPhysics/IslandSleeper.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_set::BodySet;
use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::handles::ConstraintHandle;
use crate::physics::island_scaffold::{IslandScaffold, IslandScaffoldTypeBatch};
use crate::physics::sequential_fallback_batch::SequentialFallbackBatch;
use crate::physics::solver::Solver;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use glam::Vec3;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

// Use real BroadPhase, PairCache, and ConstraintRemover from their modules.
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::collision_detection::constraint_remover::ConstraintRemover;
use crate::physics::collision_detection::inactive_set_builder::SleepingSetBuilder;
use crate::physics::collision_detection::pair_cache::PairCache;

/// Provides functionality for putting bodies to sleep when they become inactive.
pub struct IslandSleeper {
    set_id_pool: IdPool,
    bodies: *mut Bodies,
    solver: *mut Solver,
    broad_phase: *mut BroadPhase,
    pub(crate) pair_cache: *mut PairCache,
    pub(crate) constraint_remover: *mut ConstraintRemover,
    pool: *mut BufferPool,

    /// Initial capacity for island body lists during traversal.
    pub initial_island_body_capacity: i32,
    /// Initial capacity for island constraint lists during traversal.
    pub initial_island_constraint_capacity: i32,

    /// Multiplier applied to the active body count to calculate the number of sleep traversals per timestep.
    pub tested_fraction_per_frame: f32,
    /// Fraction of the active set to target as the number of bodies slept in a given frame.
    pub target_slept_fraction: f32,
    /// Fraction of the active set to target as the number of bodies traversed for sleeping in a given frame.
    pub target_traversed_fraction: f32,

    // Worker dispatch state
    schedule_offset: i32,
    job_index: UnsafeCell<i32>,

    target_traversed_body_count_per_thread: i32,
    target_slept_body_count_per_thread: i32,
    traversal_start_body_indices: QuickList<i32>,
    force_sleep: bool,

    // MT state — only valid during sleep phases.
    worker_traversal_results: Buffer<WorkerTraversalResults>,
    gathering_jobs: QuickList<GatheringJob>,
    new_inactive_sets: QuickList<InactiveSetReference>,
    removal_jobs: QuickList<RemovalJob>,
    type_batch_constraint_removal_job_count: i32,
}

struct WorkerTraversalResults {
    traversed_bodies: IndexSet,
    islands: QuickList<IslandScaffold>,
}

impl WorkerTraversalResults {
    fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.islands.count {
            unsafe {
                self.islands.get_mut(i).dispose(pool);
            }
        }
        self.islands.dispose(pool);
        self.traversed_bodies.dispose(pool);
    }
}

/// Describes cached broad phase data for a sleeping body's collidable.
#[derive(Clone, Copy)]
struct CachedBroadPhaseData {
    reference: CollidableReference,
    bounds_min: Vec3,
    bounds_max: Vec3,
}

#[derive(Clone, Copy)]
struct InactiveSetReference {
    index: i32,
    broad_phase_data: Buffer<CachedBroadPhaseData>,
}

#[derive(Clone, Copy)]
struct GatheringJob {
    target_set_index: i32,
    source_indices: QuickList<i32>,
    start_index: i32,
    end_index: i32,
    /// If true, this job relates to a subset of body indices. If false, constraint handles.
    is_body_job: bool,
    target_set_index_in_reference_list: i32,
    target_batch_index: i32,
    target_type_batch_index: i32,
}

#[derive(Clone, Copy)]
#[repr(i32)]
enum RemovalJobType {
    RemoveFromBatchReferencedHandles = 0,
    NotifyNarrowPhasePairCache = 1,
    AddCollidablesToStaticTree = 2,
    RemoveBodiesFromActiveSet = 3,
}

#[derive(Clone, Copy)]
struct RemovalJob {
    job_type: RemovalJobType,
}

/// Wraps a predicate with previously-traversed body tracking for duplicate avoidance.
struct TraversalTest<'a, P: IPredicate<i32>> {
    previously_traversed_bodies: std::cell::UnsafeCell<IndexSet>,
    predicate: &'a P,
}

impl<'a, P: IPredicate<i32>> IPredicate<i32> for TraversalTest<'a, P> {
    #[inline(always)]
    fn matches(&self, body_index: &i32) -> bool {
        unsafe {
            let ptb = &mut *self.previously_traversed_bodies.get();
            if ptb.contains(*body_index) {
                return false;
            }
            ptb.add_unsafely(*body_index);
        }
        self.predicate.matches(body_index)
    }
}

/// Predicate that checks if a body is a sleep candidate.
pub trait IPredicate<T> {
    fn matches(&self, item: &T) -> bool;
}

struct ForcedSleepPredicate;

impl IPredicate<i32> for ForcedSleepPredicate {
    #[inline(always)]
    fn matches(&self, _body_index: &i32) -> bool {
        true
    }
}

struct SleepPredicate {
    bodies: *const Bodies,
}

impl IPredicate<i32> for SleepPredicate {
    #[inline(always)]
    fn matches(&self, body_index: &i32) -> bool {
        unsafe {
            let bodies = &*self.bodies;
            bodies
                .active_set()
                .activity
                .get(*body_index)
                .sleep_candidate
        }
    }
}

/// Body enumerator that collects body indices discovered during constraint graph traversal.
struct ConstraintBodyEnumerator {
    constraint_body_indices: QuickList<i32>,
    pool: *mut BufferPool,
    source_index: i32,
}

impl IForEach<i32> for ConstraintBodyEnumerator {
    #[inline(always)]
    fn loop_body(&mut self, body_index: i32) {
        if body_index != self.source_index {
            unsafe {
                self.constraint_body_indices
                    .add(body_index, &mut *self.pool);
            }
        }
    }
}

impl IslandSleeper {
    pub fn new(
        bodies: *mut Bodies,
        solver: *mut Solver,
        broad_phase: *mut BroadPhase,
        constraint_remover: *mut ConstraintRemover,
        pool: *mut BufferPool,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let mut set_id_pool = IdPool::new(16, pool_ref);
        // Reserve index 0 for the active set.
        set_id_pool.take();

        Self {
            set_id_pool,
            bodies,
            solver,
            broad_phase,
            pair_cache: std::ptr::null_mut(),
            constraint_remover,
            pool,
            initial_island_body_capacity: 1024,
            initial_island_constraint_capacity: 1024,
            tested_fraction_per_frame: 0.01,
            target_slept_fraction: 0.005,
            target_traversed_fraction: 0.01,
            schedule_offset: 0,
            job_index: UnsafeCell::new(-1),
            target_traversed_body_count_per_thread: 0,
            target_slept_body_count_per_thread: 0,
            traversal_start_body_indices: QuickList::default(),
            force_sleep: false,
            worker_traversal_results: Buffer::default(),
            gathering_jobs: QuickList::default(),
            new_inactive_sets: QuickList::default(),
            removal_jobs: QuickList::default(),
            type_batch_constraint_removal_job_count: 0,
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

    pub(crate) fn return_set_id(&mut self, id: i32) {
        let pool = unsafe { &mut *self.pool };
        self.set_id_pool.return_id(id, pool);
    }

    /// Traverses the active constraint graph collecting bodies that match a predicate.
    /// If any body visited during the traversal fails to match the predicate, the traversal terminates.
    pub unsafe fn collect_island<P: IPredicate<i32>>(
        &self,
        pool: &mut BufferPool,
        starting_active_body_index: i32,
        predicate: &mut P,
        body_indices: &mut QuickList<i32>,
        constraint_handles: &mut QuickList<ConstraintHandle>,
    ) -> bool {
        let bodies = self.bodies();
        let solver = self.solver();

        let initial_body_capacity = self
            .initial_island_body_capacity
            .min(bodies.active_set().count);

        let mut considered_bodies = IndexSet::new(pool, bodies.active_set().count);
        let mut considered_constraints =
            IndexSet::new(pool, solver.handle_pool.highest_possibly_claimed_id() + 1);
        let mut visitation_stack = QuickList::with_capacity(initial_body_capacity, pool);

        // Start traversal by pushing the initial body
        if !Self::push_body(
            starting_active_body_index,
            &mut considered_bodies,
            body_indices,
            &mut visitation_stack,
            pool,
            predicate,
        ) {
            considered_bodies.dispose(pool);
            considered_constraints.dispose(pool);
            visitation_stack.dispose(pool);
            return false;
        }

        let mut enumerator = ConstraintBodyEnumerator {
            constraint_body_indices: QuickList::with_capacity(4, pool),
            pool: pool as *mut BufferPool,
            source_index: 0,
        };

        let mut disqualified = false;
        while let Some(next_index) = visitation_stack.try_pop() {
            if !self.enqueue_unvisited_neighbors(
                next_index,
                body_indices,
                constraint_handles,
                &mut considered_bodies,
                &mut considered_constraints,
                &mut visitation_stack,
                &mut enumerator,
                pool,
                predicate,
            ) {
                disqualified = true;
                break;
            }
        }

        enumerator.constraint_body_indices.dispose(pool);
        considered_bodies.dispose(pool);
        considered_constraints.dispose(pool);
        visitation_stack.dispose(pool);

        !disqualified
    }

    #[inline(always)]
    fn push_body<P: IPredicate<i32>>(
        body_index: i32,
        considered_bodies: &mut IndexSet,
        body_indices: &mut QuickList<i32>,
        visitation_stack: &mut QuickList<i32>,
        pool: &mut BufferPool,
        predicate: &P,
    ) -> bool {
        if !considered_bodies.contains(body_index) {
            if !predicate.matches(&body_index) {
                return false;
            }
            body_indices.add(body_index, pool);
            considered_bodies.add_unsafely(body_index);
            visitation_stack.add(body_index, pool);
        }
        true
    }

    #[inline(always)]
    unsafe fn enqueue_unvisited_neighbors<P: IPredicate<i32>>(
        &self,
        body_index: i32,
        body_indices: &mut QuickList<i32>,
        constraint_handles: &mut QuickList<ConstraintHandle>,
        considered_bodies: &mut IndexSet,
        considered_constraints: &mut IndexSet,
        visitation_stack: &mut QuickList<i32>,
        enumerator: &mut ConstraintBodyEnumerator,
        pool: &mut BufferPool,
        predicate: &P,
    ) -> bool {
        let bodies = self.bodies();
        let solver = self.solver();
        enumerator.source_index = body_index;

        let list = bodies.active_set().constraints.get(body_index);
        for i in 0..list.count {
            let entry = list.get(i);
            let handle_value = entry.connecting_constraint_handle.0;
            if !considered_constraints.contains(handle_value) {
                constraint_handles.add(entry.connecting_constraint_handle, pool);
                considered_constraints.add_unsafely(handle_value);
                enumerator.constraint_body_indices.count = 0;
                solver.enumerate_connected_raw_body_references(
                    entry.connecting_constraint_handle,
                    enumerator,
                );
                for j in 0..enumerator.constraint_body_indices.count {
                    let connected = *enumerator.constraint_body_indices.get(j);
                    if !Self::push_body(
                        connected,
                        considered_bodies,
                        body_indices,
                        visitation_stack,
                        pool,
                        predicate,
                    ) {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Updates the sleeper, performing incremental island detection and sleeping.
    pub unsafe fn update(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        let bodies = self.bodies();
        let active_count = bodies.active_set().count;
        if active_count == 0 {
            return;
        }

        let candidate_count =
            (1.0f32).max(active_count as f32 * self.tested_fraction_per_frame) as i32;
        let pool = unsafe { &mut *self.pool };
        let mut traversal_start_body_indices = QuickList::with_capacity(candidate_count, pool);

        let spacing = active_count / candidate_count;

        if self.schedule_offset > active_count {
            self.schedule_offset = 0;
        }

        let mut index = self.schedule_offset;
        for _ in 0..candidate_count {
            if index >= active_count {
                index -= active_count;
            }
            *traversal_start_body_indices.allocate_unsafely() = index;
            index += spacing;
        }
        self.schedule_offset += 1;

        let target_slept = (active_count as f32 * self.target_slept_fraction).ceil() as i32;
        let target_traversed = (active_count as f32 * self.target_traversed_fraction).ceil() as i32;

        // If the simulation is too small to generate parallel work, don't bother using threading.
        // (Passing None forces the single-threaded codepath.)
        let thread_dispatcher = if active_count < (2.0 / self.tested_fraction_per_frame) as i32 {
            None
        } else {
            thread_dispatcher
        };

        // Use raw pointer to avoid borrow conflict between pool and self
        let self_ptr = self as *mut Self;

        (*self_ptr).sleep(
            &mut traversal_start_body_indices,
            thread_dispatcher,
            deterministic,
            target_slept,
            target_traversed,
            false,
        );

        traversal_start_body_indices.dispose(pool);
    }

    // ── Worker functions for MT dispatch ──

    /// Per-worker island finding, generic over sleep predicate.
    unsafe fn find_islands_on_worker<P: IPredicate<i32>>(
        &self,
        worker_index: i32,
        thread_pool: &mut BufferPool,
        predicate: &P,
    ) {
        let bodies = &*self.bodies;
        let solver = &*self.solver;
        let results = &mut *(self
            .worker_traversal_results
            .as_ptr()
            .add(worker_index as usize)
            as *mut WorkerTraversalResults);
        results.islands = QuickList::with_capacity(64, thread_pool);
        let mut body_indices = QuickList::with_capacity(
            self.initial_island_body_capacity
                .min(bodies.active_set().count),
            thread_pool,
        );
        let mut constraint_handles = QuickList::with_capacity(
            8.max(
                self.initial_island_constraint_capacity
                    .min(solver.handle_pool.highest_possibly_claimed_id() + 1),
            ),
            thread_pool,
        );

        let mut traversal_test = TraversalTest {
            previously_traversed_bodies: std::cell::UnsafeCell::new(IndexSet::new(
                thread_pool,
                bodies.active_set().count,
            )),
            predicate,
        };

        let mut traversed_bodies = 0;
        let mut slept_bodies = 0;

        while traversed_bodies < self.target_traversed_body_count_per_thread
            && slept_bodies < self.target_slept_body_count_per_thread
        {
            let target_index =
                AtomicI32::from_ptr(self.job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
            if target_index >= self.traversal_start_body_indices.count {
                break;
            }
            let body_index = *self.traversal_start_body_indices.get(target_index);
            if self.collect_island(
                thread_pool,
                body_index,
                &mut traversal_test,
                &mut body_indices,
                &mut constraint_handles,
            ) {
                slept_bodies += body_indices.count;
                let island = IslandScaffold::new(
                    &mut body_indices,
                    &mut constraint_handles,
                    solver,
                    thread_pool,
                );
                results.islands.add(island, thread_pool);
            }
            traversed_bodies += body_indices.count;
            body_indices.count = 0;
            constraint_handles.count = 0;
        }
        body_indices.dispose(thread_pool);
        constraint_handles.dispose(thread_pool);
        results.traversed_bodies = traversal_test.previously_traversed_bodies.into_inner();
    }

    fn find_islands_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let sleeper = &*(dispatcher.unmanaged_context() as *const IslandSleeper);
            let thread_pool = &mut *dispatcher.worker_pool_ptr(worker_index);
            if sleeper.force_sleep {
                let pred = ForcedSleepPredicate;
                sleeper.find_islands_on_worker(worker_index, thread_pool, &pred);
            } else {
                let pred = SleepPredicate {
                    bodies: sleeper.bodies,
                };
                sleeper.find_islands_on_worker(worker_index, thread_pool, &pred);
            }
        }
    }

    fn gather_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let sleeper = &*(dispatcher.unmanaged_context() as *const IslandSleeper);
            let bodies_ptr = sleeper.bodies;
            let solver_ptr = sleeper.solver;
            let broad_phase = &*sleeper.broad_phase;
            loop {
                let index =
                    AtomicI32::from_ptr(sleeper.job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
                if index >= sleeper.gathering_jobs.count {
                    break;
                }
                let job = sleeper.gathering_jobs.get(index);
                if job.is_body_job {
                    let source_set = (*bodies_ptr).sets.get(0);
                    let set_ref = sleeper
                        .new_inactive_sets
                        .get(job.target_set_index_in_reference_list);
                    for target_index in job.start_index..job.end_index {
                        let source_index = *job.source_indices.get(target_index);
                        let target_set = (*bodies_ptr).sets.get_mut(job.target_set_index);
                        *target_set.index_to_handle.get_mut(target_index) =
                            *source_set.index_to_handle.get(source_index);
                        *target_set.activity.get_mut(target_index) =
                            *source_set.activity.get(source_index);
                        let source_collidable = *source_set.collidables.get(source_index);
                        *target_set.collidables.get_mut(target_index) = source_collidable;
                        *target_set.constraints.get_mut(target_index) =
                            *source_set.constraints.get(source_index);
                        *target_set.dynamics_state.get_mut(target_index) =
                            *source_set.dynamics_state.get(source_index);

                        if source_collidable.shape.exists() {
                            let bp_data =
                                &mut *(set_ref.broad_phase_data.as_ptr().add(target_index as usize)
                                    as *mut CachedBroadPhaseData);
                            bp_data.reference = *broad_phase
                                .active_leaves
                                .get(source_collidable.broad_phase_index);
                            let (min_ptr, max_ptr) = broad_phase
                                .get_active_bounds_pointers(source_collidable.broad_phase_index);
                            bp_data.bounds_min = *min_ptr;
                            bp_data.bounds_max = *max_ptr;
                        }
                    }
                } else {
                    let target_type_batch = (*solver_ptr)
                        .sets
                        .get_mut(job.target_set_index)
                        .batches
                        .get_mut(job.target_batch_index)
                        .type_batches
                        .get_mut(job.target_type_batch_index);
                    let tp = (&(*solver_ptr).type_processors)[target_type_batch.type_id as usize]
                        .as_ref()
                        .unwrap();
                    // Create a temporary scaffold referencing the job's constraint handles.
                    let source_scaffold = IslandScaffoldTypeBatch {
                        type_id: target_type_batch.type_id,
                        handles: job.source_indices,
                    };
                    tp.inner().gather_active_constraints(
                        &*bodies_ptr,
                        &*solver_ptr,
                        &source_scaffold,
                        job.start_index,
                        job.end_index,
                        &mut *target_type_batch,
                    );
                    let constraint_remover = &mut *sleeper.constraint_remover;
                    for index_in_type_batch in job.start_index..job.end_index {
                        let handle = *target_type_batch.index_to_handle.get(index_in_type_batch);
                        constraint_remover.enqueue_removal(worker_index, handle);
                    }
                }
            }
        }
    }

    fn execute_removal_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let sleeper = &mut *(dispatcher.unmanaged_context() as *mut IslandSleeper);
            loop {
                let index =
                    AtomicI32::from_ptr(sleeper.job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
                if index >= sleeper.removal_jobs.count {
                    break;
                }
                sleeper.execute_removal(*sleeper.removal_jobs.get(index));
            }
            let _ = worker_index;
        }
    }

    fn type_batch_constraint_removal_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let sleeper = &*(dispatcher.unmanaged_context() as *const IslandSleeper);
            loop {
                let index =
                    AtomicI32::from_ptr(sleeper.job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
                if index >= sleeper.type_batch_constraint_removal_job_count {
                    break;
                }
                (&mut *sleeper.constraint_remover).remove_constraints_from_type_batch(index);
            }
            let _ = worker_index;
        }
    }

    unsafe fn execute_removal(&mut self, job: RemovalJob) {
        let pool = &mut *self.pool;
        match job.job_type {
            RemovalJobType::RemoveFromBatchReferencedHandles => {
                (&mut *self.constraint_remover).remove_constraints_from_batch_referenced_handles();
            }
            RemovalJobType::RemoveBodiesFromActiveSet => {
                let bodies_ptr = self.bodies;
                let solver_ptr = self.solver;
                for set_ref_index in 0..self.new_inactive_sets.count {
                    let set_index = self.new_inactive_sets.get(set_ref_index).index;
                    let inactive_body_count = (*bodies_ptr).sets.get(set_index).count;
                    for body_index_in_inactive_set in 0..inactive_body_count {
                        let body_handle = *(*bodies_ptr)
                            .sets
                            .get(set_index)
                            .index_to_handle
                            .get(body_index_in_inactive_set);
                        let location = (*bodies_ptr).handle_to_location.get(body_handle.0);
                        debug_assert!(
                            location.set_index == 0,
                            "Set should still be 0 at this point."
                        );
                        (&mut *self.constraint_remover).try_remove_body_from_constrained_kinematics_and_remove_all_constraints_for_body_from_fallback_batch(body_handle, location.index);
                        (*bodies_ptr).remove_from_active_set(location.index);
                        let location = (*bodies_ptr).handle_to_location.get_mut(body_handle.0);
                        location.set_index = set_index;
                        location.index = body_index_in_inactive_set;
                    }
                }
            }
            RemovalJobType::AddCollidablesToStaticTree => {
                let bodies_ptr = self.bodies;
                let broad_phase = &mut *self.broad_phase;
                for set_ref_index in 0..self.new_inactive_sets.count {
                    let set_ref = self.new_inactive_sets.get(set_ref_index);
                    let set = (*bodies_ptr).sets.get_mut(set_ref.index);
                    for body_index in 0..set.count {
                        let collidable = set.collidables.get_mut(body_index);
                        if collidable.shape.exists() {
                            let data = set_ref.broad_phase_data.get(body_index);
                            collidable.broad_phase_index = broad_phase.add_static(
                                data.reference,
                                &data.bounds_min,
                                &data.bounds_max,
                            );
                        } else {
                            collidable.broad_phase_index = -1;
                        }
                    }
                }
            }
            RemovalJobType::NotifyNarrowPhasePairCache => {
                if !self.pair_cache.is_null() {
                    let pair_cache = &mut *self.pair_cache;
                    let solver_ptr = self.solver;
                    let bodies_ptr = self.bodies;
                    let mut largest_body_count = 0;
                    for i in 0..self.new_inactive_sets.count {
                        let set_count = (*bodies_ptr)
                            .sets
                            .get(self.new_inactive_sets.get(i).index)
                            .count;
                        if set_count > largest_body_count {
                            largest_body_count = set_count;
                        }
                    }
                    let mut set_builder = SleepingSetBuilder::new(pool, largest_body_count * 4);
                    for i in 0..self.new_inactive_sets.count {
                        pair_cache.sleep_type_batch_pairs(
                            &mut set_builder,
                            self.new_inactive_sets.get(i).index,
                            &*solver_ptr,
                        );
                    }
                    set_builder.dispose(pool);
                }
            }
        }
    }

    // ── Main sleep method with MT support ──

    unsafe fn sleep(
        &mut self,
        traversal_start_body_indices: &mut QuickList<i32>,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
        target_slept_body_count: i32,
        target_traversed_body_count: i32,
        force_sleep: bool,
    ) {
        let bodies = &*self.bodies;
        if bodies.active_set().count == 0 || traversal_start_body_indices.count == 0 {
            return;
        }

        let pool = &mut *self.pool;
        let solver = &*self.solver;
        let thread_count = thread_dispatcher.map_or(1, |td| td.thread_count());
        let worker_traversal_thread_count = if deterministic { 1 } else { thread_count };

        self.target_slept_body_count_per_thread =
            1.max(target_slept_body_count / worker_traversal_thread_count);
        self.target_traversed_body_count_per_thread =
            1.max(target_traversed_body_count / worker_traversal_thread_count);

        // ═══ PHASE 1: TRAVERSAL ═══
        self.traversal_start_body_indices = *traversal_start_body_indices;
        self.worker_traversal_results =
            pool.take::<WorkerTraversalResults>(worker_traversal_thread_count);
        *self.job_index.get() = -1;
        self.force_sleep = force_sleep;

        if worker_traversal_thread_count > 1 {
            let td = thread_dispatcher.unwrap();
            let self_ptr = self as *mut IslandSleeper as *mut ();
            td.dispatch_workers(
                Self::find_islands_worker,
                worker_traversal_thread_count,
                self_ptr,
                None,
            );
        } else {
            // Single-threaded: run directly with the appropriate predicate.
            if force_sleep {
                let pred = ForcedSleepPredicate;
                self.find_islands_on_worker(0, pool, &pred);
            } else {
                let pred = SleepPredicate {
                    bodies: self.bodies,
                };
                self.find_islands_on_worker(0, pool, &pred);
            }
        }

        // Check for any islands found.
        let mut total_island_count = 0;
        for i in 0..worker_traversal_thread_count {
            total_island_count += self.worker_traversal_results.get(i).islands.count;
        }

        let dispose_worker_traversal_results =
            |sleeper: &mut IslandSleeper,
             pool: &mut BufferPool,
             td: Option<&dyn IThreadDispatcher>| {
                if worker_traversal_thread_count > 1 {
                    let td = td.unwrap();
                    for worker_index in 0..worker_traversal_thread_count {
                        let wpool = &mut *td.worker_pool_ptr(worker_index);
                        sleeper
                            .worker_traversal_results
                            .get_mut(worker_index)
                            .dispose(wpool);
                    }
                } else {
                    sleeper.worker_traversal_results.get_mut(0).dispose(pool);
                }
                pool.return_buffer(&mut sleeper.worker_traversal_results);
            };

        if total_island_count == 0 {
            dispose_worker_traversal_results(self, pool, thread_dispatcher);
            return;
        }

        // ═══ PHASE 2: GATHERING ═══
        let objects_per_gather_job = 64;
        self.gathering_jobs = QuickList::with_capacity(512, pool);
        self.new_inactive_sets = QuickList::with_capacity(32, pool);
        let mut slept_body_count = 0;
        let self_ptr = self as *mut IslandSleeper;
        let bodies_ptr = self.bodies;
        let solver_ptr = self.solver;

        for worker_index in 0..worker_traversal_thread_count {
            let worker_islands = &(*self_ptr)
                .worker_traversal_results
                .get(worker_index)
                .islands;
            for j in 0..worker_islands.count {
                let island = worker_islands.get(j);

                // Check for duplicate islands from other workers.
                let mut skip = false;
                for prev_worker in 0..worker_index {
                    debug_assert!(island.body_indices.count > 0);
                    if (*self_ptr)
                        .worker_traversal_results
                        .get(prev_worker)
                        .traversed_bodies
                        .contains(*island.body_indices.get(0))
                    {
                        skip = true;
                        break;
                    }
                }
                if skip {
                    continue;
                }

                // Allocate new inactive set.
                let set_index = (*self_ptr).set_id_pool.take();
                (*self_ptr)
                    .new_inactive_sets
                    .ensure_capacity((*self_ptr).new_inactive_sets.count + 1, pool);
                let reference_index = (*self_ptr).new_inactive_sets.count;
                let new_set_ref = (*self_ptr).new_inactive_sets.allocate_unsafely();
                new_set_ref.index = set_index;
                new_set_ref.broad_phase_data =
                    pool.take::<CachedBroadPhaseData>(island.body_indices.count);
                (*self_ptr).ensure_sets_capacity(set_index + 1);

                let body_count = island.body_indices.count;
                *(*bodies_ptr).sets.get_mut(set_index) = BodySet::new(body_count, pool);
                (*bodies_ptr).sets.get_mut(set_index).count = body_count;
                slept_body_count += body_count;

                // Create inactive constraint set from the scaffold.
                if island.protobatches.count > 0 {
                    *(*solver_ptr).sets.get_mut(set_index) =
                        ConstraintSet::new(pool, island.protobatches.count);
                    let constraint_set = (*solver_ptr).sets.get_mut(set_index);
                    for batch_index in 0..island.protobatches.count {
                        let source_batch = island.protobatches.get(batch_index);
                        let target_batch = constraint_set.batches.allocate_unsafely();
                        *target_batch =
                            ConstraintBatch::new(pool, source_batch.type_id_to_index.len());
                        for type_batch_index in 0..source_batch.type_batches.count {
                            let source_type_batch = source_batch.type_batches.get(type_batch_index);
                            let tp = (&(*solver_ptr).type_processors)
                                [source_type_batch.type_id as usize]
                                .as_ref()
                                .unwrap();
                            let target_type_batch = (*target_batch).create_new_type_batch(
                                source_type_batch.type_id,
                                tp.inner(),
                                source_type_batch.handles.count,
                                pool,
                            );
                            (*target_type_batch).constraint_count = source_type_batch.handles.count;
                        }
                    }
                }

                // Create body gathering jobs.
                {
                    let job_count = 1.max(body_count / objects_per_gather_job);
                    let bodies_per_job = body_count / job_count;
                    let remainder = body_count - bodies_per_job * job_count;
                    let mut previous_end = 0;
                    (*self_ptr)
                        .gathering_jobs
                        .ensure_capacity((*self_ptr).gathering_jobs.count + job_count, pool);
                    for i in 0..job_count {
                        let bodies_in_job = if i < remainder {
                            bodies_per_job + 1
                        } else {
                            bodies_per_job
                        };
                        let job = (*self_ptr).gathering_jobs.allocate_unsafely();
                        job.is_body_job = true;
                        job.source_indices = island.body_indices;
                        job.start_index = previous_end;
                        previous_end += bodies_in_job;
                        job.end_index = previous_end;
                        job.target_set_index = set_index;
                        job.target_set_index_in_reference_list = reference_index;
                        job.target_batch_index = 0;
                        job.target_type_batch_index = 0;
                    }
                }

                // Create constraint gathering jobs per type batch.
                for batch_index in 0..island.protobatches.count {
                    let source_batch = island.protobatches.get(batch_index);
                    for type_batch_index in 0..source_batch.type_batches.count {
                        let source_type_batch = source_batch.type_batches.get(type_batch_index);
                        let handle_count = source_type_batch.handles.count;
                        let job_count = 1.max(handle_count / objects_per_gather_job);
                        let constraints_per_job = handle_count / job_count;
                        let remainder = handle_count - constraints_per_job * job_count;
                        let mut previous_end = 0;
                        (*self_ptr)
                            .gathering_jobs
                            .ensure_capacity((*self_ptr).gathering_jobs.count + job_count, pool);
                        for i in 0..job_count {
                            let constraints_in_job = if i < remainder {
                                constraints_per_job + 1
                            } else {
                                constraints_per_job
                            };
                            let job = (*self_ptr).gathering_jobs.allocate_unsafely();
                            job.is_body_job = false;
                            job.source_indices = source_type_batch.handles;
                            job.start_index = previous_end;
                            previous_end += constraints_in_job;
                            job.end_index = previous_end;
                            job.target_set_index = set_index;
                            job.target_batch_index = batch_index;
                            job.target_type_batch_index = type_batch_index;
                            job.target_set_index_in_reference_list = 0;
                        }
                    }
                }

                // Copy fallback batch if applicable.
                if island.protobatches.count > (*solver_ptr).fallback_batch_threshold() {
                    let constraint_set = (*solver_ptr).sets.get_mut(set_index);
                    constraint_set.sequential_fallback =
                        SequentialFallbackBatch::create_from(&island.fallback_batch, pool);
                }
            }
        }

        (&mut *(*self_ptr).constraint_remover).prepare(thread_dispatcher);

        *(*self_ptr).job_index.get() = -1;
        if thread_count > 1 {
            let td = thread_dispatcher.unwrap();
            let ctx = self_ptr as *mut ();
            td.dispatch_workers(
                Self::gather_worker,
                (*self_ptr).gathering_jobs.count,
                ctx,
                None,
            );
        } else {
            Self::gather_worker_inline(self_ptr, pool);
        }

        dispose_worker_traversal_results(&mut *self_ptr, pool, thread_dispatcher);
        (*self_ptr).gathering_jobs.dispose(pool);

        // ═══ PHASE 3: REMOVAL BOOKKEEPING ═══
        let broad_phase = &mut *(*self_ptr).broad_phase;
        broad_phase.ensure_capacity(
            broad_phase.active_tree.leaf_count,
            broad_phase.static_tree.leaf_count + slept_body_count,
        );

        (*self_ptr).type_batch_constraint_removal_job_count =
            (&mut *(*self_ptr).constraint_remover).create_flush_jobs(deterministic);

        (*self_ptr).removal_jobs = QuickList::with_capacity(4, pool);
        // Schedule heavier jobs first for better load balancing.
        *(*self_ptr).removal_jobs.allocate_unsafely() = RemovalJob {
            job_type: RemovalJobType::NotifyNarrowPhasePairCache,
        };
        *(*self_ptr).removal_jobs.allocate_unsafely() = RemovalJob {
            job_type: RemovalJobType::RemoveBodiesFromActiveSet,
        };
        *(*self_ptr).removal_jobs.allocate_unsafely() = RemovalJob {
            job_type: RemovalJobType::AddCollidablesToStaticTree,
        };
        *(*self_ptr).removal_jobs.allocate_unsafely() = RemovalJob {
            job_type: RemovalJobType::RemoveFromBatchReferencedHandles,
        };

        *(*self_ptr).job_index.get() = -1;
        if thread_count > 1 {
            let td = thread_dispatcher.unwrap();
            let ctx = self_ptr as *mut ();
            td.dispatch_workers(
                Self::execute_removal_worker,
                (*self_ptr).removal_jobs.count,
                ctx,
                None,
            );
        } else {
            for i in 0..(*self_ptr).removal_jobs.count {
                let job = *(*self_ptr).removal_jobs.get(i);
                (*self_ptr).execute_removal(job);
            }
        }
        (*self_ptr).removal_jobs.dispose(pool);

        // ═══ PHASE 4: CONSTRAINT REMOVAL FROM TYPE BATCHES ═══
        *(*self_ptr).job_index.get() = -1;
        if thread_count > 1 {
            let td = thread_dispatcher.unwrap();
            let ctx = self_ptr as *mut ();
            td.dispatch_workers(
                Self::type_batch_constraint_removal_worker,
                (*self_ptr).type_batch_constraint_removal_job_count,
                ctx,
                None,
            );
        } else {
            for i in 0..(*self_ptr).type_batch_constraint_removal_job_count {
                (&mut *(*self_ptr).constraint_remover).remove_constraints_from_type_batch(i);
            }
        }

        // Sequential: update handle→constraint mappings for all sleeping constraints.
        for i in 0..(*self_ptr).new_inactive_sets.count {
            let set_index = (*self_ptr).new_inactive_sets.get(i).index;
            let set = (*solver_ptr).sets.get(set_index);
            for batch_index in 0..set.batches.count {
                let batch = set.batches.get(batch_index);
                for type_batch_index in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(type_batch_index);
                    for index_in_type_batch in 0..type_batch.constraint_count {
                        let handle = *type_batch.index_to_handle.get(index_in_type_batch);
                        let constraint_location =
                            (*solver_ptr).handle_to_constraint.get_mut(handle.0);
                        constraint_location.set_index = set_index;
                        constraint_location.batch_index = batch_index;
                        constraint_location.index_in_type_batch = index_in_type_batch;
                    }
                }
            }
        }

        // Clean up.
        for i in 0..(*self_ptr).new_inactive_sets.count {
            pool.return_buffer(&mut (*self_ptr).new_inactive_sets.get_mut(i).broad_phase_data);
        }
        (*self_ptr).new_inactive_sets.dispose(pool);
        (&mut *(*self_ptr).constraint_remover).postflush();
    }

    /// Single-threaded inline gather (avoids dispatch overhead).
    unsafe fn gather_worker_inline(sleeper: *mut IslandSleeper, pool: &mut BufferPool) {
        let bodies_ptr = (*sleeper).bodies;
        let solver_ptr = (*sleeper).solver;
        let broad_phase = &*(*sleeper).broad_phase;
        for index in 0..(*sleeper).gathering_jobs.count {
            let job = (*sleeper).gathering_jobs.get(index);
            if job.is_body_job {
                let source_set = (*bodies_ptr).sets.get(0);
                let set_ref = (*sleeper)
                    .new_inactive_sets
                    .get(job.target_set_index_in_reference_list);
                for target_index in job.start_index..job.end_index {
                    let source_index = *job.source_indices.get(target_index);
                    let target_set = (*bodies_ptr).sets.get_mut(job.target_set_index);
                    *target_set.index_to_handle.get_mut(target_index) =
                        *source_set.index_to_handle.get(source_index);
                    *target_set.activity.get_mut(target_index) =
                        *source_set.activity.get(source_index);
                    let source_collidable = *source_set.collidables.get(source_index);
                    *target_set.collidables.get_mut(target_index) = source_collidable;
                    *target_set.constraints.get_mut(target_index) =
                        *source_set.constraints.get(source_index);
                    *target_set.dynamics_state.get_mut(target_index) =
                        *source_set.dynamics_state.get(source_index);

                    if source_collidable.shape.exists() {
                        let bp_data =
                            &mut *(set_ref.broad_phase_data.as_ptr().add(target_index as usize)
                                as *mut CachedBroadPhaseData);
                        bp_data.reference = *broad_phase
                            .active_leaves
                            .get(source_collidable.broad_phase_index);
                        let (min_ptr, max_ptr) = broad_phase
                            .get_active_bounds_pointers(source_collidable.broad_phase_index);
                        bp_data.bounds_min = *min_ptr;
                        bp_data.bounds_max = *max_ptr;
                    }
                }
            } else {
                let target_type_batch = (*solver_ptr)
                    .sets
                    .get_mut(job.target_set_index)
                    .batches
                    .get_mut(job.target_batch_index)
                    .type_batches
                    .get_mut(job.target_type_batch_index);
                let tp = (&(*solver_ptr).type_processors)[target_type_batch.type_id as usize]
                    .as_ref()
                    .unwrap();
                // The source_indices for constraint jobs contains ConstraintHandles stored as i32.
                // Create a temporary scaffold referencing the job's constraint handles.
                let source_scaffold = IslandScaffoldTypeBatch {
                    type_id: target_type_batch.type_id,
                    handles: job.source_indices,
                };
                tp.inner().gather_active_constraints(
                    &*bodies_ptr,
                    &*solver_ptr,
                    &source_scaffold,
                    job.start_index,
                    job.end_index,
                    &mut *target_type_batch,
                );
                let constraint_remover = &mut *(*sleeper).constraint_remover;
                for index_in_type_batch in job.start_index..job.end_index {
                    let handle = *target_type_batch.index_to_handle.get(index_in_type_batch);
                    constraint_remover.enqueue_removal(0, handle);
                }
            }
        }
    }

    /// Forcefully sleeps a list of bodies and all bodies reachable through the constraint graph.
    pub unsafe fn sleep_bodies(
        &mut self,
        body_indices: &mut QuickList<i32>,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        self.sleep(
            body_indices,
            thread_dispatcher,
            deterministic,
            i32::MAX,
            i32::MAX,
            true,
        );
    }

    /// Forces a single body (and its island) to go to sleep.
    pub unsafe fn sleep_body(&mut self, body_index: i32) {
        let pool = &mut *self.pool;
        let mut list = QuickList::with_capacity(1, pool);
        *list.allocate_unsafely() = body_index;
        let self_ptr = self as *mut Self;
        (*self_ptr).sleep_bodies(&mut list, None, false);
        list.dispose(pool);
    }

    /// Ensures that Bodies, Solver, and NarrowPhase can hold at least the given number of sets.
    pub fn ensure_sets_capacity(&mut self, sets_capacity: i32) {
        let bodies = unsafe { &mut *self.bodies };
        let solver = unsafe { &mut *self.solver };
        let min_allocated =
            unsafe { (*(&self.set_id_pool as *const IdPool)).highest_possibly_claimed_id() } + 1;
        let potentially_allocated = min_allocated.min(bodies.sets.len()).min(solver.sets.len());

        if sets_capacity > bodies.sets.len() {
            bodies.resize_sets_capacity(sets_capacity, potentially_allocated);
        }
        if sets_capacity > solver.sets.len() {
            solver.resize_sets_capacity(sets_capacity, potentially_allocated);
        }
        if !self.pair_cache.is_null() {
            let pair_cache = unsafe { &mut *self.pair_cache };
            if sets_capacity > pair_cache.sleeping_sets.len() {
                pair_cache.resize_sets_capacity(sets_capacity, potentially_allocated);
            }
        }
    }

    /// Resizes the sets capacity for Bodies and Solver.
    pub fn resize_sets_capacity(&mut self, sets_capacity: i32) {
        let bodies = unsafe { &mut *self.bodies };
        let solver = unsafe { &mut *self.solver };
        let min_allocated =
            unsafe { (*(&self.set_id_pool as *const IdPool)).highest_possibly_claimed_id() } + 1;
        let potentially_allocated = min_allocated.min(bodies.sets.len()).min(solver.sets.len());
        let target = potentially_allocated.max(sets_capacity);

        bodies.resize_sets_capacity(target, potentially_allocated);
        solver.resize_sets_capacity(target, potentially_allocated);
        if !self.pair_cache.is_null() {
            let pair_cache = unsafe { &mut *self.pair_cache };
            pair_cache.resize_sets_capacity(target, potentially_allocated);
        }
    }

    pub fn clear(&mut self) {
        self.set_id_pool.clear();
        // Slot 0 is reserved for the active set.
        self.set_id_pool.take();
    }

    pub fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        self.set_id_pool.dispose(pool);
    }
}

unsafe impl Send for IslandSleeper {}
unsafe impl Sync for IslandSleeper {}
