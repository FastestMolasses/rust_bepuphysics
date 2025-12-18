// Translated from BepuPhysics/CollisionDetection/PairCache.cs and PairCache_Activity.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::handles::ConstraintHandle;
use crate::physics::solver::Solver;
use crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer;
use crate::utilities::collections::quick_dictionary::QuickDictionary;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::fmt;

use super::inactive_set_builder::{SleepingSet, SleepingSetBuilder};
use super::worker_pair_cache::WorkerPendingPairChanges;

/// Packed pair of collidable references.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CollidablePair {
    pub a: CollidableReference,
    pub b: CollidableReference,
}

const _: () = {
    assert!(std::mem::size_of::<CollidablePair>() == 8);
};

impl CollidablePair {
    #[inline(always)]
    pub fn new(a: CollidableReference, b: CollidableReference) -> Self {
        Self { a, b }
    }
}

impl Default for CollidablePair {
    fn default() -> Self {
        Self {
            a: CollidableReference::default(),
            b: CollidableReference::default(),
        }
    }
}

impl fmt::Display for CollidablePair {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<{}, {}>", self.a, self.b)
    }
}

/// Comparer for CollidablePair. Pairs are sorted by handle, so order matters.
#[derive(Debug, Clone, Copy, Default)]
pub struct CollidablePairComparer;

impl RefEqualityComparer<CollidablePair> for CollidablePairComparer {
    #[inline(always)]
    fn equals(&self, a: &CollidablePair, b: &CollidablePair) -> bool {
        // Compare as u64 for speed
        unsafe {
            *(a as *const CollidablePair as *const u64) == *(b as *const CollidablePair as *const u64)
        }
    }

    #[inline(always)]
    fn hash(&self, item: &CollidablePair) -> i32 {
        const P1: u64 = 961748927;
        const P2: u64 = 899809343;
        let hash64 = (item.a.packed as u64).wrapping_mul(P1.wrapping_mul(P2))
            .wrapping_add((item.b.packed as u64).wrapping_mul(P2));
        (hash64 ^ (hash64 >> 32)) as i32
    }
}

/// Type alias for the overlap mapping dictionary.
pub type OverlapMapping = QuickDictionary<CollidablePair, ConstraintCache, CollidablePairComparer>;

/// Refers to a change in a PairCache.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PairCacheChangeIndex {
    /// Index of the WorkerPendingPairChanges storing the pending change, if any.
    /// If -1, then this pair cache change refers to a change directly to the mapping.
    pub worker_index: i32,
    /// Index of the change in the cache.
    pub index: i32,
}

impl PairCacheChangeIndex {
    /// Gets whether this change is pending (from a worker).
    #[inline(always)]
    pub fn is_pending(&self) -> bool {
        self.worker_index >= 0
    }
}

/// Stores information about a contact constraint from the previous timestep.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ConstraintCache {
    /// Handle of the contact constraint associated with this cache.
    pub constraint_handle: ConstraintHandle,
    /// Feature id of the first contact in the constraint.
    pub feature_id0: i32,
    /// Feature id of the second contact in the constraint.
    pub feature_id1: i32,
    /// Feature id of the third contact in the constraint.
    pub feature_id2: i32,
    /// Feature id of the fourth contact in the constraint.
    pub feature_id3: i32,
}

impl Default for ConstraintCache {
    fn default() -> Self {
        Self {
            constraint_handle: ConstraintHandle(0),
            feature_id0: 0,
            feature_id1: 0,
            feature_id2: 0,
            feature_id3: 0,
        }
    }
}

/// Location of a collision pair, used for mapping constraint handles back to pairs.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct CollisionPairLocation {
    pub pair: CollidablePair,
    /// Used only when the collision pair was moved into a sleeping set.
    pub inactive_set_index: i32,
    pub inactive_pair_index: i32,
}

impl Default for CollisionPairLocation {
    fn default() -> Self {
        Self {
            pair: CollidablePair::default(),
            inactive_set_index: 0,
            inactive_pair_index: 0,
        }
    }
}

/// Cache of collision pairs, tracking overlap mappings and sleeping sets.
pub struct PairCache {
    pub mapping: OverlapMapping,
    /// Per-pair 'freshness' flags set when a pair is added or updated by the narrow phase execution.
    /// Only initialized for the duration of the narrowphase's execution.
    pub(crate) pair_freshness: Buffer<u8>,
    pub(crate) pool: *mut BufferPool,
    minimum_pending_size: i32,
    previous_pending_size: i32,

    /// While the current worker caches are read from, changes to the cache are accumulated.
    pub(crate) worker_pending_changes: Buffer<WorkerPendingPairChanges>,
    pub(crate) cached_dispatcher: Option<*mut dyn IThreadDispatcher>,

    /// Mapping from constraint handle back to collision detection pair cache locations.
    pub(crate) constraint_handle_to_pair: Buffer<CollisionPairLocation>,

    /// Sleeping sets, filled in parallel with Bodies.Sets and Solver.Sets.
    /// Note that this does not include the active set, so index 0 is always empty.
    pub(crate) sleeping_sets: Buffer<SleepingSet>,

}

impl PairCache {
    /// Number of collision constraint types.
    pub const COLLISION_CONSTRAINT_TYPE_COUNT: i32 = 22;
    /// Number of collision types.
    pub const COLLISION_TYPE_COUNT: i32 = 16;

    /// Creates a new PairCache.
    pub fn new(
        pool: *mut BufferPool,
        initial_set_capacity: i32,
        minimum_mapping_size: i32,
        minimum_pending_size: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let mut cache = Self {
            mapping: OverlapMapping::with_capacity(
                minimum_mapping_size,
                3,
                pool_ref,
                CollidablePairComparer,
            ),
            pair_freshness: Buffer::default(),
            pool,
            minimum_pending_size,
            previous_pending_size: 0,
            worker_pending_changes: Buffer::default(),
            cached_dispatcher: None,
            constraint_handle_to_pair: Buffer::default(),
            sleeping_sets: Buffer::default(),
        };
        cache.resize_sets_capacity(initial_set_capacity, 0);
        cache
    }

    /// Prepares the pair cache for a new narrow phase execution.
    pub fn prepare(&mut self, thread_dispatcher: Option<*mut dyn IThreadDispatcher>) {
        let pool_ref = unsafe { &mut *self.pool };
        let thread_count = if let Some(dispatcher) = thread_dispatcher {
            unsafe { (&*dispatcher).thread_count() }
        } else {
            1
        };

        self.cached_dispatcher = thread_dispatcher;

        let pending_size = i32::max(self.minimum_pending_size, self.previous_pending_size);
        self.worker_pending_changes = pool_ref.take(thread_count as i32);

        if let Some(dispatcher) = thread_dispatcher {
            for i in 0..thread_count {
                unsafe {
                    let worker_pool = &mut *(&*dispatcher).worker_pool_ptr(i as i32);
                    *self.worker_pending_changes.get_mut(i as i32) =
                        WorkerPendingPairChanges::new(worker_pool, pending_size);
                }
            }
        } else {
            unsafe {
                *self.worker_pending_changes.get_mut(0) =
                    WorkerPendingPairChanges::new(pool_ref, pending_size);
            }
        }

        // Create the pair freshness array for the existing overlaps.
        self.pair_freshness = pool_ref.take_at_least(self.mapping.count);
        // This clears 1 byte per pair.
        self.pair_freshness.clear(0, self.mapping.count);
    }

    /// Ensures the constraint-to-pair mapping has sufficient capacity.
    pub(crate) fn ensure_constraint_to_pair_mapping_capacity(
        &mut self,
        solver: &Solver,
        target_capacity: i32,
    ) {
        let pool_ref = unsafe { &mut *self.pool };
        let target = i32::max(solver.handle_pool.highest_possibly_claimed_id() + 1, target_capacity);
        if self.constraint_handle_to_pair.len() < target {
            let copy_count = self.constraint_handle_to_pair.len();
            pool_ref.resize_to_at_least(
                &mut self.constraint_handle_to_pair,
                target,
                copy_count,
            );
        }
    }

    /// Resizes the constraint-to-pair mapping capacity.
    pub(crate) fn resize_constraint_to_pair_mapping_capacity(
        &mut self,
        solver: &Solver,
        target_capacity: i32,
    ) {
        let pool_ref = unsafe { &mut *self.pool };
        let target = BufferPool::get_capacity_for_count::<CollisionPairLocation>(
            i32::max(solver.handle_pool.highest_possibly_claimed_id() + 1, target_capacity),
        );
        if self.constraint_handle_to_pair.len() != target {
            let copy_count = i32::min(target, self.constraint_handle_to_pair.len());
            pool_ref.resize_to_at_least(
                &mut self.constraint_handle_to_pair,
                target,
                copy_count,
            );
        }
    }

    /// Flush all deferred changes from the last narrow phase execution.
    pub fn flush_mapping_changes(&mut self) {
        // Flush all pending adds from the new set.
        // Note that this phase accesses no shared memory.
        for i in 0..self.worker_pending_changes.len() {
            unsafe {
                let cache = &mut *self.worker_pending_changes.get_mut(i);

                // Walk backwards on the off chance that a swap can be avoided.
                for j in (0..cache.pending_removes.count).rev() {
                    let removed = self.mapping.fast_remove(cache.pending_removes.get(j));
                    debug_assert!(removed);
                }
                for j in 0..cache.pending_adds.count {
                    let pending = &*cache.pending_adds.get(j);
                    let added = self.mapping.try_add_unsafely(pending.pair, pending.cache);
                    debug_assert!(added);
                }
            }
        }
    }

    /// Post-flush bookkeeping and disposal.
    pub fn postflush(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        let mut largest_pending_size = 0i32;
        for i in 0..self.worker_pending_changes.len() {
            unsafe {
                let pending_changes = &*self.worker_pending_changes.get(i);
                if pending_changes.pending_adds.count > largest_pending_size {
                    largest_pending_size = pending_changes.pending_adds.count;
                }
                if pending_changes.pending_removes.count > largest_pending_size {
                    largest_pending_size = pending_changes.pending_removes.count;
                }
            }
        }

        if self.worker_pending_changes.len() > 1 {
            if let Some(dispatcher) = self.cached_dispatcher {
                for i in 0..self.worker_pending_changes.len() {
                    unsafe {
                        let worker_pool = &mut *(&*dispatcher).worker_pool_ptr(i);
                        (*self.worker_pending_changes.get_mut(i)).dispose(worker_pool);
                    }
                }
            }
        } else {
            unsafe {
                (*self.worker_pending_changes.get_mut(0)).dispose(pool_ref);
            }
        }
        self.previous_pending_size = largest_pending_size;

        pool_ref.return_buffer(&mut self.worker_pending_changes);
        self.cached_dispatcher = None;
    }

    /// Clears all sleeping sets.
    pub(crate) fn clear(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        for i in 1..self.sleeping_sets.len() {
            unsafe {
                let set = &mut *self.sleeping_sets.get_mut(i);
                if set.allocated() {
                    set.dispose(pool_ref);
                }
            }
        }
        debug_assert!(!self.worker_pending_changes.allocated());
    }

    /// Disposes the pair cache.
    pub fn dispose(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        debug_assert!(!self.worker_pending_changes.allocated());

        self.mapping.dispose(pool_ref);
        for i in 1..self.sleeping_sets.len() {
            unsafe {
                let set = &mut *self.sleeping_sets.get_mut(i);
                if set.allocated() {
                    set.dispose(pool_ref);
                }
            }
        }
        pool_ref.return_buffer(&mut self.sleeping_sets);
        if self.constraint_handle_to_pair.allocated() {
            pool_ref.return_buffer(&mut self.constraint_handle_to_pair);
        }
    }

    /// Gets the index of a pair in the mapping.
    #[inline(always)]
    pub fn index_of(&self, pair: &CollidablePair) -> Option<i32> {
        self.mapping.index_of(pair)
    }

    /// Gets the cache at a given index.
    #[inline(always)]
    pub fn get_cache(&self, index: i32) -> &ConstraintCache {
        unsafe { self.mapping.values.get(index) }
    }

    /// Adds a new pair via a worker. Returns the change index.
    #[inline(always)]
    pub(crate) fn add(
        &mut self,
        worker_index: i32,
        pair: CollidablePair,
        constraint_cache: &ConstraintCache,
    ) -> PairCacheChangeIndex {
        // Note that we do not have to set any freshness bytes here; using this path means there exists no previous overlap to remove anyway.
        let pool = if self.cached_dispatcher.is_none() {
            unsafe { &mut *self.pool }
        } else {
            unsafe {
                let dispatcher = self.cached_dispatcher.unwrap();
                &mut *(&*dispatcher).worker_pool_ptr(worker_index)
            }
        };
        let index = unsafe {
            (*self.worker_pending_changes.get_mut(worker_index)).add(pool, pair, constraint_cache)
        };
        PairCacheChangeIndex {
            worker_index,
            index,
        }
    }

    /// Updates an existing pair in the mapping. Returns the change index.
    #[inline(always)]
    pub(crate) fn update(
        &mut self,
        pair_index: i32,
        cache: &ConstraintCache,
    ) -> PairCacheChangeIndex {
        // We're updating an existing pair, so we should prevent this pair from being removed.
        unsafe {
            *self.pair_freshness.get_mut(pair_index) = 0xFF;
            *self.mapping.values.get_mut(pair_index) = *cache;
        }
        PairCacheChangeIndex {
            worker_index: -1,
            index: pair_index,
        }
    }

    /// Gets the old constraint handle for a pair at the given index.
    #[inline(always)]
    pub(crate) fn get_old_constraint_handle(&self, pair_index: i32) -> ConstraintHandle {
        unsafe { self.mapping.values.get(pair_index).constraint_handle }
    }

    // --- PairCache_Activity methods ---

    /// Resizes the sleeping sets capacity.
    pub(crate) fn resize_sets_capacity(&mut self, sets_capacity: i32, potentially_allocated_count: i32) {
        let pool_ref = unsafe { &mut *self.pool };
        debug_assert!(
            sets_capacity >= potentially_allocated_count
                && potentially_allocated_count <= self.sleeping_sets.len()
        );
        let capacity = BufferPool::get_capacity_for_count::<SleepingSet>(sets_capacity);
        if self.sleeping_sets.len() != capacity {
            let old_capacity = self.sleeping_sets.len();
            pool_ref.resize_to_at_least(
                &mut self.sleeping_sets,
                capacity,
                potentially_allocated_count,
            );
            if old_capacity < self.sleeping_sets.len() {
                // We rely on unused slots being default initialized.
                self.sleeping_sets.clear(old_capacity, self.sleeping_sets.len() - old_capacity);
            }
        }
    }

    /// Validates that a constraint handle appears the expected number of times in the mapping.
    #[cfg(debug_assertions)]
    pub(crate) fn validate_handle_count_in_mapping(
        &self,
        constraint_handle: ConstraintHandle,
        expected_count: i32,
    ) {
        let mut count = 0;
        for i in 0..self.mapping.count {
            unsafe {
                let existing_cache = self.mapping.values.get(i);
                if existing_cache.constraint_handle == constraint_handle {
                    count += 1;
                    debug_assert!(
                        count <= expected_count && count <= 1,
                        "Expected count violated."
                    );
                }
            }
        }
        debug_assert!(
            count == expected_count,
            "Expected count for this handle not found!"
        );
    }

    /// Moves type batch pairs into a sleeping set.
    pub(crate) fn sleep_type_batch_pairs(
        &mut self,
        builder: &mut SleepingSetBuilder,
        set_index: i32,
        solver: &Solver,
    ) {
        let pool_ref = unsafe { &mut *self.pool };
        let constraint_set = unsafe { &*solver.sets.get(set_index) };
        for batch_index in 0..constraint_set.batches.count {
            let batch = unsafe { &*constraint_set.batches.get(batch_index) };
            for type_batch_index in 0..batch.type_batches.count {
                let type_batch = unsafe { &*batch.type_batches.get(type_batch_index) };
                debug_assert!(
                    type_batch.constraint_count > 0,
                    "If a type batch exists, it should contain constraints."
                );
                if is_contact_constraint_type(type_batch.type_id) {
                    for index_in_type_batch in 0..type_batch.constraint_count {
                        let handle = unsafe { *type_batch.index_to_handle.get(index_in_type_batch) };
                        let pair_location = unsafe {
                            &mut *self.constraint_handle_to_pair.get_mut(handle.0)
                        };
                        let mut table_index = 0i32;
                        let mut element_index = 0i32;
                        self.mapping.get_table_indices(
                            &pair_location.pair,
                            &mut table_index,
                            &mut element_index,
                        );
                        let cache = unsafe { *self.mapping.values.get(element_index) };
                        pair_location.inactive_set_index = set_index;
                        pair_location.inactive_pair_index = builder.add(
                            pool_ref,
                            unsafe { *self.mapping.keys.get(element_index) },
                            &cache,
                        );

                        // Now that any existing cache data has been moved into the inactive set,
                        // we should remove the overlap from the overlap mapping.
                        self.mapping.fast_remove_at(table_index, element_index);
                    }
                }
            }
        }
        builder.finalize_set(pool_ref, unsafe {
            &mut *self.sleeping_sets.get_mut(set_index)
        });
    }

    /// Awakens a sleeping set, moving its pairs back into the active mapping.
    pub(crate) fn awaken_set(&mut self, set_index: i32) {
        let sleeping_set = unsafe { &*self.sleeping_sets.get(set_index) };
        // If there are no pairs, there is no need for an inactive set, so it's not guaranteed to be allocated.
        if sleeping_set.allocated() {
            for i in 0..sleeping_set.pairs.count {
                let pair = unsafe { &*sleeping_set.pairs.get(i) };
                self.mapping.try_add_unsafely(pair.pair, pair.cache);
            }
        }
    }

    /// Removes a reference if the type is a contact constraint.
    pub(crate) fn remove_reference_if_contact_constraint(
        &mut self,
        handle: ConstraintHandle,
        type_id: i32,
    ) {
        if is_contact_constraint_type(type_id) {
            let pair = unsafe { &(*self.constraint_handle_to_pair.get(handle.0)).pair };
            let removed = self.mapping.fast_remove(pair);
            debug_assert!(
                removed,
                "If a contact constraint is being directly removed, it must exist within the pair mapping."
            );
        }
    }
}

/// Checks whether a constraint type id represents a contact constraint.
#[inline(always)]
pub fn is_contact_constraint_type(type_id: i32) -> bool {
    // Contact constraints are the first COLLISION_CONSTRAINT_TYPE_COUNT type ids.
    type_id < PairCache::COLLISION_CONSTRAINT_TYPE_COUNT
}
