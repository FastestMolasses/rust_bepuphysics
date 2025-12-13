// Translated from BepuPhysics/ConstraintSet.cs

use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::sequential_fallback_batch::SequentialFallbackBatch;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;

pub struct ConstraintSet {
    pub batches: QuickList<ConstraintBatch>,
    pub sequential_fallback: SequentialFallbackBatch,
}

impl ConstraintSet {
    pub fn new(pool: &mut BufferPool, initial_batch_capacity: i32) -> Self {
        Self {
            batches: QuickList::with_capacity(initial_batch_capacity, pool),
            sequential_fallback: SequentialFallbackBatch::default(),
        }
    }

    /// Gets the total number of bundles across all types and batches.
    pub fn bundle_count(&self) -> i32 {
        let mut count = 0;
        for i in 0..self.batches.count {
            let batch = self.batches.get(i);
            for j in 0..batch.type_batches.count {
                count += batch.type_batches.get(j).bundle_count() as i32;
            }
        }
        count
    }

    /// Gets the total number of constraints across all types and batches.
    pub fn constraint_count(&self) -> i32 {
        let mut count = 0;
        for i in 0..self.batches.count {
            let batch = self.batches.get(i);
            for j in 0..batch.type_batches.count {
                count += batch.type_batches.get(j).constraint_count;
            }
        }
        count
    }

    /// Gets whether this constraint set is allocated.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.batches.span.allocated()
    }

    pub fn clear(&mut self, pool: &mut BufferPool) {
        for i in 0..self.batches.count {
            self.batches.get_mut(i).dispose(pool);
        }
        self.sequential_fallback.dispose(pool);
        self.batches.count = 0;
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.batches.count {
            self.batches.get_mut(i).dispose(pool);
        }
        self.sequential_fallback.dispose(pool);
        self.batches.dispose(pool);
        *self = Self::default();
    }
}

impl Default for ConstraintSet {
    fn default() -> Self {
        Self {
            batches: QuickList::default(),
            sequential_fallback: SequentialFallbackBatch::default(),
        }
    }
}
