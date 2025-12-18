// Translated from BepuPhysics/CollisionDetection/InactiveSetBuilder.cs

use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;
use super::pair_cache::{CollidablePair, ConstraintCache};

/// A pair stored in a sleeping set.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SleepingPair {
    pub pair: CollidablePair,
    pub cache: ConstraintCache,
}

/// A set of sleeping pairs.
#[derive(Copy, Clone)]
pub struct SleepingSet {
    pub pairs: QuickList<SleepingPair>,
}

impl SleepingSet {
    /// Gets whether this sleeping set has allocated memory.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.pairs.span.allocated()
    }

    /// Disposes the sleeping set.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        self.pairs.dispose(pool);
    }
}

impl Default for SleepingSet {
    fn default() -> Self {
        Self {
            pairs: QuickList::default(),
        }
    }
}

/// Builder for constructing sleeping sets from active pairs.
pub struct SleepingSetBuilder {
    pub pairs: QuickList<SleepingPair>,
}

impl SleepingSetBuilder {
    /// Creates a new sleeping set builder.
    pub fn new(pool: &mut BufferPool, initial_pair_capacity: i32) -> Self {
        Self {
            pairs: QuickList::with_capacity(initial_pair_capacity, pool),
        }
    }

    /// Adds a pair to the builder. Returns the index of the pair.
    pub fn add(&mut self, pool: &mut BufferPool, pair: CollidablePair, cache: &ConstraintCache) -> i32 {
        let pair_index = self.pairs.count;
        let entry = self.pairs.allocate(pool);
        entry.pair = pair;
        entry.cache = *cache;
        pair_index
    }

    /// Finalizes the builder into a sleeping set, minimizing memory usage.
    pub fn finalize_set(&mut self, pool: &mut BufferPool, set: &mut SleepingSet) {
        // Repackage the gathered caches into a smaller format for longer term storage.
        // This adds a little extra cost, but it
        // 1) avoids the need for most incremental resizes during inactive set construction by sharing allocations and
        // 2) minimizes the memory required for the inactive set.
        if self.pairs.count > 0 {
            set.pairs = QuickList::with_capacity(self.pairs.count, pool);
            set.pairs.add_range_unsafely(&self.pairs.span, 0, self.pairs.count);
            self.pairs.count = 0;
        } else {
            // No pairs -> no set required.
            *set = SleepingSet::default();
        }
    }

    /// Disposes the builder.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        self.pairs.dispose(pool);
    }
}
