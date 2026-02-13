// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundPairOverlaps.cs

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

/// Trait for subpair overlap storage that can allocate new overlap entries.
pub trait ICollisionTaskSubpairOverlaps {
    /// Allocates a new overlap entry and returns a mutable reference to the child index slot.
    fn allocate(&mut self, pool: &mut BufferPool) -> &mut i32;
}

/// Trait for pair-level overlap storage that can provide per-pair overlaps.
pub trait ICollisionTaskOverlaps<T: ICollisionTaskSubpairOverlaps> {
    /// Gets a mutable reference to the subpair overlaps for the given pair index.
    fn get_overlaps_for_pair(&mut self, pair_index: i32) -> &mut T;
}

/// Stores overlapping child indices for one child of a compound pair.
#[repr(C)]
pub struct ChildOverlapsCollection {
    /// Buffer of overlapping child indices.
    pub overlaps: Buffer<i32>,
    /// Number of overlaps.
    pub count: i32,
    /// Index of the child in the parent compound.
    pub child_index: i32,
}

impl ChildOverlapsCollection {
    /// Allocates a new overlap entry and returns a mutable reference to the child index slot.
    #[inline(always)]
    pub fn allocate(&mut self, pool: &mut BufferPool) -> &mut i32 {
        if self.count == self.overlaps.len() {
            let new_size = if self.count > 0 { self.count * 2 } else { 4 };
            let mut new_buffer: Buffer<i32> = pool.take_at_least(new_size);
            if self.count > 0 {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        self.overlaps.as_ptr(),
                        new_buffer.as_mut_ptr(),
                        self.count as usize,
                    );
                }
                pool.return_buffer(&mut self.overlaps);
            }
            self.overlaps = new_buffer;
        }
        let index = self.count;
        self.count += 1;
        &mut self.overlaps[index]
    }
}

impl ChildOverlapsCollection {
    /// Disposes the overlaps buffer back to the pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        if self.overlaps.allocated() {
            pool.return_buffer(&mut self.overlaps);
        }
    }
}

impl ICollisionTaskSubpairOverlaps for ChildOverlapsCollection {
    fn allocate(&mut self, pool: &mut BufferPool) -> &mut i32 {
        ChildOverlapsCollection::allocate(self, pool)
    }
}

impl Default for ChildOverlapsCollection {
    fn default() -> Self {
        Self {
            overlaps: Buffer::default(),
            count: 0,
            child_index: 0,
        }
    }
}

/// Query bounds for overlap testing between compound pairs.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct OverlapQueryForPair {
    /// Pointer to the container shape data.
    pub container: *const u8,
    /// Minimum of the query bounding box.
    pub min: Vec3,
    /// Maximum of the query bounding box.
    pub max: Vec3,
}

/// Stores overlap results for all children of all pairs in a compound-compound collision task batch.
pub struct CompoundPairOverlaps {
    /// Buffer of per-child overlap collections.
    pub child_overlaps: Buffer<ChildOverlapsCollection>,
    /// Buffer of per-pair query bounds.
    pub pair_queries: Buffer<OverlapQueryForPair>,
    /// Buffer of (start, count) regions mapping pairs to their children in child_overlaps.
    pub pair_regions: Buffer<(i32, i32)>,
    /// Number of pairs.
    pair_count: i32,
    /// Cursor for the next child index in child_overlaps.
    child_cursor: i32,
}

impl CompoundPairOverlaps {
    /// Creates a new CompoundPairOverlaps with the given capacity.
    pub fn new(pool: &mut BufferPool, pair_count: i32, total_compound_child_count: i32) -> Self {
        let child_overlaps = pool.take(total_compound_child_count);
        let pair_queries = pool.take(total_compound_child_count);
        let pair_regions = pool.take(pair_count);
        Self {
            child_overlaps,
            pair_queries,
            pair_regions,
            pair_count,
            child_cursor: 0,
        }
    }

    /// Creates a new region for a pair with the given child count.
    #[inline(always)]
    pub fn create_pair_overlaps(&mut self, child_count: i32) {
        let pair_index = self.pair_count_so_far();
        self.pair_regions[pair_index] = (self.child_cursor, child_count);
        // Initialize child overlap entries.
        for i in 0..child_count {
            self.child_overlaps[self.child_cursor + i] = ChildOverlapsCollection::default();
        }
        self.child_cursor += child_count;
    }

    fn pair_count_so_far(&self) -> i32 {
        // Count pairs that have been registered based on cursor advancement.
        // We use pair_regions to track which pairs have been set up.
        let mut count = 0;
        for i in 0..self.pair_count {
            let (start, len) = self.pair_regions[i];
            if start > 0 || len > 0 || i == 0 {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Gets a mutable reference to the overlaps for a specific subpair.
    #[inline(always)]
    pub fn get_overlaps_for_pair(&mut self, subpair_index: i32) -> &mut ChildOverlapsCollection {
        &mut self.child_overlaps[subpair_index]
    }

    /// Gets the pair region (start, count) for a given pair.
    #[inline(always)]
    pub fn get_pair_region(&self, pair_index: i32) -> (i32, i32) {
        self.pair_regions[pair_index]
    }

    /// Disposes all buffers back to the pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.child_overlaps.len() {
            if self.child_overlaps[i].overlaps.allocated() {
                pool.return_buffer(&mut self.child_overlaps[i].overlaps);
            }
        }
        pool.return_buffer(&mut self.child_overlaps);
        pool.return_buffer(&mut self.pair_queries);
        pool.return_buffer(&mut self.pair_regions);
    }
}

impl ICollisionTaskOverlaps<ChildOverlapsCollection> for CompoundPairOverlaps {
    fn get_overlaps_for_pair(&mut self, pair_index: i32) -> &mut ChildOverlapsCollection {
        CompoundPairOverlaps::get_overlaps_for_pair(self, pair_index)
    }
}
