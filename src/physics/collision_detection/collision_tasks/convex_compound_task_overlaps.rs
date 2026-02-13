// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCompoundTaskOverlaps.cs

use super::compound_pair_overlaps::{
    ICollisionTaskOverlaps, ICollisionTaskSubpairOverlaps, OverlapQueryForPair,
};
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
    /// Pre-allocates storage for the given number of overlap indices.
    #[inline(always)]
    pub fn allocate_for_count(&mut self, pool: &mut BufferPool, child_count: i32) {
        self.overlaps = pool.take(child_count);
        self.count = 0;
    }

    /// Allocates and returns a mutable reference to the next overlap slot, growing if needed.
    #[inline(always)]
    pub fn allocate(&mut self, pool: &mut BufferPool) -> &mut i32 {
        if self.count == self.overlaps.len() {
            let new_size = if self.overlaps.len() == 0 {
                4
            } else {
                self.overlaps.len() * 2
            };
            pool.resize(&mut self.overlaps, new_size, self.count);
        }
        let index = self.count as usize;
        self.count += 1;
        &mut self.overlaps[index]
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

impl ICollisionTaskSubpairOverlaps for ConvexCompoundOverlaps {
    #[inline(always)]
    fn allocate(&mut self, pool: &mut BufferPool) -> &mut i32 {
        ConvexCompoundOverlaps::allocate(self, pool)
    }
}

impl ICollisionTaskOverlaps<ConvexCompoundOverlaps> for ConvexCompoundTaskOverlaps {
    #[inline(always)]
    fn get_overlaps_for_pair(&mut self, pair_index: i32) -> &mut ConvexCompoundOverlaps {
        &mut self.pair_overlaps[pair_index]
    }
}

/// Stores overlap results for all pairs in a convex-compound collision task batch.
pub struct ConvexCompoundTaskOverlaps {
    /// Buffer of per-pair overlaps.
    pub pair_overlaps: Buffer<ConvexCompoundOverlaps>,
    /// Buffer of per-pair query bounds (container pointer + min/max).
    pub subpair_queries: Buffer<OverlapQueryForPair>,
}

impl ConvexCompoundTaskOverlaps {
    /// Creates task overlaps for the given number of pairs.
    pub fn new(pool: &mut BufferPool, pair_count: i32) -> Self {
        let mut pair_overlaps: Buffer<ConvexCompoundOverlaps> = pool.take(pair_count);
        let subpair_queries: Buffer<OverlapQueryForPair> = pool.take(pair_count);
        // We rely on the length being zero to begin with for lazy initialization.
        pair_overlaps.clear(0, pair_count);
        Self {
            pair_overlaps,
            subpair_queries,
        }
    }

    /// Gets a mutable reference to the overlaps for a given pair.
    #[inline(always)]
    pub fn get_overlaps_for_pair(&mut self, pair_index: i32) -> &mut ConvexCompoundOverlaps {
        &mut self.pair_overlaps[pair_index]
    }

    /// Gets a mutable reference to the query bounds for a given pair.
    #[inline(always)]
    pub fn get_query_for_pair(&mut self, pair_index: i32) -> &mut OverlapQueryForPair {
        &mut self.subpair_queries[pair_index]
    }

    /// Disposes all pair overlap buffers and the container buffer.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.pair_overlaps.len() {
            self.pair_overlaps[i].dispose(pool);
        }
        pool.return_buffer(&mut self.subpair_queries);
        pool.return_buffer(&mut self.pair_overlaps);
    }
}
