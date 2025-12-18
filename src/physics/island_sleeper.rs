// Translated from BepuPhysics/IslandSleeper.cs

use crate::physics::bodies::Bodies;
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
use std::sync::atomic::{AtomicI32, Ordering};

// Use real BroadPhase from its module. ConstraintRemover and PairCache remain opaque until translated.
use crate::physics::collision_detection::broad_phase::BroadPhase;

pub struct ConstraintRemover {
    _opaque: [u8; 0],
}

pub struct PairCache {
    _opaque: [u8; 0],
}

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
    job_index: AtomicI32,

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
            job_index: AtomicI32::new(-1),
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
        let bodies = self.bodies();
        if bodies.active_set().count == 0 || traversal_start_body_indices.count == 0 {
            return;
        }

        // Phase 1: TRAVERSAL - Find islands
        let pool = self.pool();
        let solver = self.solver();

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

        // Phase 2-4: GATHERING, REMOVAL, CONSTRAINT REMOVAL
        // TODO: Full implementation of gathering data into inactive sets,
        // removing from active set, and cleaning up constraint type batches.
        // This requires ConstraintRemover, PairCache, BroadPhase integration.

        // For now, dispose the islands
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
        // TODO: if sets_capacity > pair_cache.sleeping_sets.len()
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
        // TODO: pair_cache.resize_sets_capacity(...)
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
