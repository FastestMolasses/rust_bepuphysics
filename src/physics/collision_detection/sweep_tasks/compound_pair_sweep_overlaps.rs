// Translated from BepuPhysics/CollisionDetection/SweepTasks/CompoundPairSweepOverlaps.cs

use crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::ChildOverlapsCollection;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

pub struct CompoundPairSweepOverlaps {
    child_overlaps: Buffer<ChildOverlapsCollection>,
    pub child_count: i32,
}

impl CompoundPairSweepOverlaps {
    pub fn new(pool: &mut BufferPool, child_count: i32) -> Self {
        let mut child_overlaps: Buffer<ChildOverlapsCollection> = pool.take(child_count);
        // We rely on the length being zero to begin with for lazy initialization.
        unsafe {
            child_overlaps.clear(0, child_count);
        }
        Self {
            child_overlaps,
            child_count,
        }
    }

    #[inline(always)]
    pub fn get_overlaps_for_child(&mut self, pair_index: i32) -> &mut ChildOverlapsCollection {
        self.child_overlaps.get_mut(pair_index)
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.child_count {
            self.child_overlaps.get_mut(i).dispose(pool);
        }
        pool.return_buffer(&mut self.child_overlaps);
    }
}

impl Default for CompoundPairSweepOverlaps {
    fn default() -> Self {
        Self {
            child_overlaps: Buffer::default(),
            child_count: 0,
        }
    }
}
