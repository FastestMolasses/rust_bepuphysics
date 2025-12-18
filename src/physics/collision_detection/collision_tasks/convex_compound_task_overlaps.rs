// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundTaskOverlaps.cs

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Stores the overlap results for a single convex-compound pair.
/// Contains a list of child indices that overlap with the convex shape.
pub struct ConvexCompoundOverlaps {
    /// Buffer of child indices that overlap.
    pub overlaps: Buffer<i32>,
    /// Number of overlapping children.
    pub count: i32,
}

impl ConvexCompoundOverlaps {
    /// Allocates storage for overlap indices.
    #[inline(always)]
    pub fn allocate(&mut self, pool: &mut BufferPool, child_count: i32) {
        self.overlaps = pool.take(child_count);
        self.count = 0;
    }

    /// Disposes the overlap buffer back to the pool.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.overlaps);
    }
}

impl Default for ConvexCompoundOverlaps {
    fn default() -> Self {
        Self {
            overlaps: Buffer::default(),
            count: 0,
        }
    }
}

/// Stores overlap results for all pairs in a convex-compound collision task batch.
pub struct ConvexCompoundTaskOverlaps {
    /// Buffer of per-pair overlaps.
    pub pair_overlaps: Buffer<ConvexCompoundOverlaps>,
}

impl ConvexCompoundTaskOverlaps {
    /// Creates task overlaps for the given number of pairs.
    pub fn new(pool: &mut BufferPool, pair_count: i32) -> Self {
        let pair_overlaps = pool.take(pair_count);
        Self { pair_overlaps }
    }

    /// Gets a mutable reference to the overlaps for a given pair.
    #[inline(always)]
    pub fn get_overlaps_for_pair(&mut self, pair_index: i32) -> &mut ConvexCompoundOverlaps {
        &mut self.pair_overlaps[pair_index]
    }

    /// Disposes all pair overlap buffers and the container buffer.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.pair_overlaps.len() {
            self.pair_overlaps[i].dispose(pool);
        }
        pool.return_buffer(&mut self.pair_overlaps);
    }
}
