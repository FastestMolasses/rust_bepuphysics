// Translated from BepuPhysics/IslandSleeper.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_set::BodySet;
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::island_scaffold::IslandScaffold;
use crate::physics::sequential_fallback_batch::SequentialFallbackBatch;
use crate::physics::solver::Solver;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
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
    constraint_remover: *mut ConstraintRemover,
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
            bodies.active_set().activity.get(*body_index).sleep_candidate
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
                self.constraint_body_indices.add(body_index, &mut *self.pool);
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

        let initial_body_capacity = self.initial_island_body_capacity.min(bodies.active_set().count);

        let mut considered_bodies = IndexSet::new(pool, bodies.active_set().count);
        let mut considered_constraints = IndexSet::new(pool, solver.handle_pool.highest_possibly_claimed_id() + 1);
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
                solver.enumerate_connected_raw_body_references(entry.connecting_constraint_handle, enumerator);
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

        let candidate_count = (1.0f32).max(active_count as f32 * self.tested_fraction_per_frame) as i32;
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

        // Use raw pointer to avoid borrow conflict between pool and self
        let self_ptr = self as *mut Self;

        // For now, run single-threaded
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

    unsafe fn sleep(
        &mut self,
        traversal_start_body_indices: &mut QuickList<i32>,
        _thread_dispatcher: Option<&dyn IThreadDispatcher>,
        _deterministic: bool,
        target_slept_body_count: i32,
        target_traversed_body_count: i32,
        force_sleep: bool,
    ) {
        let bodies = &*self.bodies;
        if bodies.active_set().count == 0 || traversal_start_body_indices.count == 0 {
            return;
        }

        // Phase 1: TRAVERSAL - Find islands
        let pool = &mut *self.pool;
        let solver = &*self.solver;

        let mut body_indices = QuickList::with_capacity(
            self.initial_island_body_capacity.min(bodies.active_set().count),
            pool,
        );
        let mut constraint_handles = QuickList::with_capacity(
            8.max(self.initial_island_constraint_capacity.min(solver.handle_pool.highest_possibly_claimed_id() + 1)),
            pool,
        );

        let mut previously_traversed = IndexSet::new(pool, bodies.active_set().count);
        let mut islands: Vec<IslandScaffold> = Vec::new();
        let mut traversed_bodies = 0;
        let mut slept_bodies = 0;

        let mut target_index = 0;
        while traversed_bodies < target_traversed_body_count
            && slept_bodies < target_slept_body_count
            && target_index < traversal_start_body_indices.count
        {
            let body_index = *traversal_start_body_indices.get(target_index);
            target_index += 1;

            // Skip already-traversed bodies
            if previously_traversed.contains(body_index) {
                continue;
            }

            if force_sleep {
                let mut pred = ForcedSleepPredicate;
                if self.collect_island(pool, body_index, &mut pred, &mut body_indices, &mut constraint_handles) {
                    slept_bodies += body_indices.count;
                    let island = IslandScaffold::new(
                        &mut body_indices,
                        &mut constraint_handles,
                        solver,
                        pool,
                    );
                    islands.push(island);
                }
            } else {
                let mut pred = SleepPredicate { bodies: self.bodies };
                if self.collect_island(pool, body_index, &mut pred, &mut body_indices, &mut constraint_handles) {
                    slept_bodies += body_indices.count;
                    let island = IslandScaffold::new(
                        &mut body_indices,
                        &mut constraint_handles,
                        solver,
                        pool,
                    );
                    islands.push(island);
                }
            }

            // Mark bodies as traversed
            for i in 0..body_indices.count {
                previously_traversed.add_unsafely(*body_indices.get(i));
            }

            traversed_bodies += body_indices.count;
            body_indices.count = 0;
            constraint_handles.count = 0;
        }

        body_indices.dispose(pool);
        constraint_handles.dispose(pool);
        previously_traversed.dispose(pool);

        if islands.is_empty() {
            return;
        }

        // Phase 2: GATHERING — allocate inactive sets and copy body/constraint data.
        let bodies_ptr = self.bodies;
        let solver_ptr = self.solver;
        let self_ptr = self as *mut IslandSleeper;

        struct InactiveSetReference {
            index: i32,
        }
        let mut new_inactive_sets: Vec<InactiveSetReference> = Vec::new();

        for island in &mut islands {
            // Allocate a new inactive set index.
            let set_index = (*self_ptr).set_id_pool.take();
            new_inactive_sets.push(InactiveSetReference { index: set_index });
            (*self_ptr).ensure_sets_capacity(set_index + 1);

            // Create the inactive body set.
            let body_count = island.body_indices.count;
            *(*bodies_ptr).sets.get_mut(set_index) = BodySet::new(body_count, pool);
            (*bodies_ptr).sets.get_mut(set_index).count = body_count;

            // Copy body data from active set to inactive set.
            let source_set_ptr = (*bodies_ptr).sets.get(0) as *const crate::physics::body_set::BodySet;
            let target_set_ptr = (*bodies_ptr).sets.get_mut(set_index) as *mut crate::physics::body_set::BodySet;
            for target_index in 0..body_count {
                let source_index = *island.body_indices.get(target_index);
                let source_set = &*source_set_ptr;
                let target_set = &mut *target_set_ptr;
                *target_set.index_to_handle.get_mut(target_index) = *source_set.index_to_handle.get(source_index);
                *target_set.activity.get_mut(target_index) = *source_set.activity.get(source_index);
                *target_set.collidables.get_mut(target_index) = *source_set.collidables.get(source_index);
                *target_set.constraints.get_mut(target_index) = *source_set.constraints.get(source_index);
                *target_set.dynamics_state.get_mut(target_index) = *source_set.dynamics_state.get(source_index);
            }

            // Create the inactive constraint set from the scaffold.
            if island.protobatches.count > 0 {
                *(*solver_ptr).sets.get_mut(set_index) = ConstraintSet::new(pool, island.protobatches.count);
                let constraint_set = &mut *(*solver_ptr).sets.get_mut(set_index);
                for batch_index in 0..island.protobatches.count {
                    let source_batch = &*island.protobatches.get(batch_index);
                    let target_batch = constraint_set.batches.allocate_unsafely();
                    *target_batch = ConstraintBatch::new(pool, source_batch.type_id_to_index.len());
                    for type_batch_index in 0..source_batch.type_batches.count {
                        let source_type_batch = &*source_batch.type_batches.get(type_batch_index);
                        let tp = (&(*solver_ptr).type_processors)[source_type_batch.type_id as usize]
                            .as_ref()
                            .unwrap();
                        let target_type_batch = (*target_batch).create_new_type_batch(
                            source_type_batch.type_id,
                            tp.inner(),
                            source_type_batch.handles.count,
                            pool,
                        );
                        (*target_type_batch).constraint_count = source_type_batch.handles.count;

                        // Copy constraint data from active to inactive type batch.
                        tp.inner().gather_active_constraints(
                            &*bodies_ptr,
                            &*solver_ptr,
                            source_type_batch,
                            0,
                            source_type_batch.handles.count,
                            &mut *target_type_batch,
                        );
                    }
                }

                // Copy fallback batch if applicable.
                if island.protobatches.count > (*solver_ptr).fallback_batch_threshold() {
                    constraint_set.sequential_fallback = SequentialFallbackBatch::create_from(
                        &island.fallback_batch,
                        pool,
                    );
                }
            }

            // Enqueue constraints for removal.
            let constraint_remover = &mut *(*self_ptr).constraint_remover;
            constraint_remover.prepare(None);
            if (*solver_ptr).sets.get(set_index).allocated() {
                let cset = (*solver_ptr).sets.get(set_index);
                for bi in 0..cset.batches.count {
                    let batch = cset.batches.get(bi);
                    for tbi in 0..batch.type_batches.count {
                        let tb = batch.type_batches.get(tbi);
                        for ci in 0..tb.constraint_count {
                            let handle = *tb.index_to_handle.get(ci);
                            constraint_remover.enqueue_removal(0, handle);
                        }
                    }
                }
            }
        }

        // Phase 3: REMOVAL — remove bodies from active set, notify pair cache.
        let constraint_remover = &mut *(*self_ptr).constraint_remover;
        let type_batch_removal_job_count = constraint_remover.create_flush_jobs(_deterministic);

        // 3a: Notify pair cache — move overlap pairs to sleeping sets.
        if !(*self_ptr).pair_cache.is_null() {
            let pair_cache = &mut *(*self_ptr).pair_cache;
            let mut largest_body_count = 0;
            for set_ref in &new_inactive_sets {
                let set_count = (*bodies_ptr).sets.get(set_ref.index).count;
                if set_count > largest_body_count {
                    largest_body_count = set_count;
                }
            }
            let mut set_builder = SleepingSetBuilder::new(pool, largest_body_count * 4);
            for set_ref in &new_inactive_sets {
                pair_cache.sleep_type_batch_pairs(
                    &mut set_builder,
                    set_ref.index,
                    &*solver_ptr,
                );
            }
            set_builder.dispose(pool);
        }

        // 3b: Remove bodies from active set and update handle→location mappings.
        let bodies = &mut *bodies_ptr;
        for set_ref in &new_inactive_sets {
            let inactive_body_count = (*bodies_ptr).sets.get(set_ref.index).count;
            for body_index_in_inactive_set in 0..inactive_body_count {
                let body_handle = *(*bodies_ptr).sets.get(set_ref.index).index_to_handle.get(body_index_in_inactive_set);
                let active_index = (*bodies_ptr).handle_to_location.get(body_handle.0).index;
                bodies.remove_from_active_set(active_index);
                // Update the handle→body mapping to point at the inactive set.
                let location = &mut *(*bodies_ptr).handle_to_location.get_mut(body_handle.0);
                location.set_index = set_ref.index;
                location.index = body_index_in_inactive_set;
            }
        }

        // 3c: Remove constraints from batch referenced handles.
        let constraint_remover = &mut *(*self_ptr).constraint_remover;
        constraint_remover.remove_constraints_from_batch_referenced_handles();

        // Phase 4: CONSTRAINT REMOVAL FROM TYPE BATCHES.
        for i in 0..type_batch_removal_job_count {
            (&mut *(*self_ptr).constraint_remover).remove_constraints_from_type_batch(i);
        }

        // Sequential: update handle→constraint mappings for all sleeping constraints.
        for set_ref in &new_inactive_sets {
            let set = (*solver_ptr).sets.get(set_ref.index);
            for batch_index in 0..set.batches.count {
                let batch = set.batches.get(batch_index);
                for type_batch_index in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(type_batch_index);
                    for index_in_type_batch in 0..type_batch.constraint_count {
                        let handle = *type_batch.index_to_handle.get(index_in_type_batch);
                        let constraint_location = &mut *(*solver_ptr).handle_to_constraint.get_mut(handle.0);
                        constraint_location.set_index = set_ref.index;
                        constraint_location.batch_index = batch_index;
                        constraint_location.index_in_type_batch = index_in_type_batch;
                    }
                }
            }
        }

        (&mut *(*self_ptr).constraint_remover).postflush();

        // Dispose the islands.
        for island in &mut islands {
            island.dispose(pool);
        }
    }

    /// Forcefully sleeps a list of bodies and all bodies reachable through the constraint graph.
    pub unsafe fn sleep_bodies(
        &mut self,
        body_indices: &mut QuickList<i32>,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        self.sleep(body_indices, thread_dispatcher, deterministic, i32::MAX, i32::MAX, true);
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
        let min_allocated = unsafe { (*(&self.set_id_pool as *const IdPool)).highest_possibly_claimed_id() } + 1;
        let potentially_allocated = min_allocated
            .min(bodies.sets.len())
            .min(solver.sets.len());

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
        let min_allocated = unsafe { (*(&self.set_id_pool as *const IdPool)).highest_possibly_claimed_id() } + 1;
        let potentially_allocated = min_allocated
            .min(bodies.sets.len())
            .min(solver.sets.len());
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
