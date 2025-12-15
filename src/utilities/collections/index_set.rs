use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;
use std::cmp::min;

/// Collection of unique indices supporting add, remove, and contains operations.
/// Uses packed bitfields where each bit represents one index's containment state.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct IndexSet {
    /// Packed bitfields representing index containment.
    pub flags: Buffer<u64>,
}

impl IndexSet {
    const SHIFT: i32 = 6;
    const MASK: i32 = 63;

    /// Gets the bundle capacity needed for the given count.
    #[inline(always)]
    pub fn get_bundle_capacity(count: i32) -> i32 {
        (count + Self::MASK) >> Self::SHIFT
    }

    /// Creates a new IndexSet with the specified initial capacity.
    #[inline(always)]
    pub fn new(pool: &mut impl UnmanagedMemoryPool, initial_capacity: i32) -> Self {
        let mut flags = Buffer::default();
        Self::internal_resize(&mut flags, pool, initial_capacity);
        IndexSet { flags }
    }

    /// Creates a default empty IndexSet.
    #[inline(always)]
    pub fn empty() -> Self {
        IndexSet {
            flags: Buffer::default(),
        }
    }

    #[inline(always)]
    fn internal_resize(flags: &mut Buffer<u64>, pool: &mut impl UnmanagedMemoryPool, capacity: i32) {
        Self::internal_resize_for_bundle_count(flags, pool, Self::get_bundle_capacity(capacity));
    }

    #[inline(always)]
    fn internal_resize_for_bundle_count(
        flags: &mut Buffer<u64>,
        pool: &mut impl UnmanagedMemoryPool,
        bundle_capacity: i32,
    ) {
        let copy_region_length = min(bundle_capacity, flags.len());
        pool.resize_to_at_least(flags, bundle_capacity, copy_region_length);
        // Since the pool's data is not guaranteed to be clean, we must clear any memory beyond the copied region.
        if flags.len() > copy_region_length {
            flags.clear(copy_region_length, flags.len() - copy_region_length);
        }
    }

    /// Checks if an index is contained in the set.
    #[inline(always)]
    pub fn contains(&self, index: i32) -> bool {
        let packed_index = index >> Self::SHIFT;
        packed_index < self.flags.len()
            && (self.flags[packed_index] & (1u64 << (index & Self::MASK))) > 0
    }

    /// Gets whether the set could hold the specified indices.
    #[inline(always)]
    pub fn can_fit(&self, index_list: &[i32]) -> bool {
        for &index in index_list {
            if self.contains(index) {
                return false;
            }
        }
        true
    }

    #[inline(always)]
    fn set_unsafely_with_bundle(&mut self, index: i32, bundle_index: i32) {
        let slot = 1u64 << (index & Self::MASK);
        self.flags[bundle_index] |= slot;
    }

    /// Sets an index in the set without checking capacity or whether it is already set.
    #[inline(always)]
    pub fn set_unsafely(&mut self, index: i32) {
        self.set_unsafely_with_bundle(index, index >> Self::SHIFT);
    }

    /// Sets an index in the set without checking whether it is already set.
    #[inline(always)]
    pub fn set(&mut self, index: i32, pool: &mut impl UnmanagedMemoryPool) {
        let bundle_index = index >> Self::SHIFT;
        if bundle_index >= self.flags.len() {
            // Round up to power of 2
            let new_capacity = ((bundle_index + 1) as u32).next_power_of_two() as i32;
            Self::internal_resize_for_bundle_count(&mut self.flags, pool, new_capacity);
        }
        self.set_unsafely_with_bundle(index, index >> Self::SHIFT);
    }

    /// Marks an index in the set as uncontained without checking whether it is already set.
    #[inline(always)]
    pub fn unset(&mut self, index: i32) {
        self.flags[index >> Self::SHIFT] &= !(1u64 << (index & Self::MASK));
    }

    /// Adds an index to the set without checking capacity.
    #[inline(always)]
    pub fn add_unsafely(&mut self, index: i32) {
        debug_assert!(
            (self.flags[index >> Self::SHIFT] & (1u64 << (index & Self::MASK))) == 0,
            "Cannot add if it's already present!"
        );
        self.set_unsafely_with_bundle(index, index >> Self::SHIFT);
    }

    /// Adds an index to the set.
    #[inline(always)]
    pub fn add(&mut self, index: i32, pool: &mut impl UnmanagedMemoryPool) {
        let bundle_index = index >> Self::SHIFT;
        if bundle_index >= self.flags.len() {
            // Round up to power of 2
            let new_capacity = ((bundle_index + 1) as u32).next_power_of_two() as i32;
            Self::internal_resize_for_bundle_count(&mut self.flags, pool, new_capacity);
        }
        debug_assert!(
            (self.flags[index >> Self::SHIFT] & (1u64 << (index & Self::MASK))) == 0,
            "Cannot add if it's already present!"
        );
        self.set_unsafely_with_bundle(index, bundle_index);
    }

    /// Removes an index from the set.
    #[inline(always)]
    pub fn remove(&mut self, index: i32) {
        debug_assert!(
            (self.flags[index >> Self::SHIFT] & (1u64 << (index & Self::MASK))) > 0,
            "If you try to remove an index, it should be present."
        );
        self.unset(index);
    }

    /// Clears all indices from the set.
    #[inline(always)]
    pub fn clear(&mut self) {
        self.flags.clear(0, self.flags.len());
    }

    /// Ensures the set has capacity for the specified number of indices.
    #[inline(always)]
    pub fn ensure_capacity(&mut self, index_capacity: i32, pool: &mut impl UnmanagedMemoryPool) {
        if (self.flags.len() << Self::SHIFT) < index_capacity {
            Self::internal_resize(&mut self.flags, pool, index_capacity);
        }
    }

    /// Compacts the internal buffer to the minimum size required.
    #[inline(always)]
    pub fn compact(&mut self, index_capacity: i32, pool: &mut impl UnmanagedMemoryPool) {
        let desired_bundle_count =
            BufferPool::get_capacity_for_count::<u64>(Self::get_bundle_capacity(index_capacity));
        if self.flags.len() > desired_bundle_count {
            Self::internal_resize_for_bundle_count(&mut self.flags, pool, desired_bundle_count);
        }
    }

    /// Resizes the internal buffer for the specified index capacity.
    #[inline(always)]
    pub fn resize(&mut self, index_capacity: i32, pool: &mut impl UnmanagedMemoryPool) {
        let desired_bundle_count =
            BufferPool::get_capacity_for_count::<u64>(Self::get_bundle_capacity(index_capacity));
        if self.flags.len() != desired_bundle_count {
            Self::internal_resize_for_bundle_count(&mut self.flags, pool, desired_bundle_count);
        }
    }

    /// Returns the resources associated with the set to the pool.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        debug_assert!(self.flags.len() > 0, "Cannot double-dispose.");
        pool.return_buffer(&mut self.flags);
        self.flags = Buffer::default();
    }
}

impl Default for IndexSet {
    fn default() -> Self {
        Self::empty()
    }
}
