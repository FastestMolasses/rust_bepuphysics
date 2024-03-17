use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::unmanaged_mempool::IUnmanagedMemoryPool;
use std::cmp::min;

struct IndexSet {
    flags: Buffer<u64>,
}

impl IndexSet {
    const SHIFT: usize = 6;
    const MASK: u64 = 63;

    #[inline(always)]
    fn get_bundle_capacity(count: usize) -> usize {
        (count + Self::MASK as usize) >> Self::SHIFT
    }

    pub fn new(pool: &mut impl IUnmanagedMemoryPool, initial_capacity: usize) -> Self {
        let mut flags = unsafe { Buffer::new() };
        Self::internal_resize(&mut flags, pool, initial_capacity);
        IndexSet { flags }
    }

    #[inline(always)]
    fn internal_resize(
        flags: &mut Buffer<u64>,
        pool: &mut impl IUnmanagedMemoryPool,
        capacity: usize,
    ) {
        Self::internal_resize_for_bundle_count(flags, pool, Self::get_bundle_capacity(capacity));
    }

    #[inline(always)]
    fn internal_resize_for_bundle_count(
        flags: &mut Buffer<u64>,
        pool: &mut impl IUnmanagedMemoryPool,
        bundle_capacity: usize,
    ) {
        let copy_region_length = min(bundle_capacity, flags.len());
        pool.resize_to_at_least(flags, bundle_capacity, copy_region_length);
        if flags.len() > copy_region_length {
            flags.clear(copy_region_length, flags.len() - copy_region_length);
        }
    }

    #[inline(always)]
    pub fn contains(&self, index: usize) -> bool {
        let packed_index = index >> Self::SHIFT;
        packed_index < self.flags.len()
            && (self.flags.get(packed_index) & (1 << (index & Self::MASK as usize))) > 0
    }

    #[inline(always)]
    pub fn can_fit(&self, index_list: &[usize]) -> bool {
        for &index in index_list.iter() {
            if self.contains(index) {
                return false;
            }
        }
        true
    }

    #[inline(always)]
    fn set_unsafely(&mut self, index: usize, bundle_index: usize) {
        let slot = 1u64 << (index & Self::MASK as usize);
        // Unsafe due to direct memory access; ensure that this operation cannot violate memory safety.
        unsafe {
            let mut bundle = self.flags.get_mut(bundle_index);
            *bundle |= slot;
        }
    }

    #[inline(always)]
    pub fn set(&mut self, index: usize, pool: &mut impl IUnmanagedMemoryPool) {
        let bundle_index = index >> Self::SHIFT;
        if bundle_index >= self.flags.len() {
            Self::internal_resize_for_bundle_count(&mut self.flags, pool, bundle_index + 1);
        }
        self.set_unsafely(index, bundle_index);
    }

    #[inline(always)]
    pub fn unset(&mut self, index: usize) {
        let bundle_index = index >> Self::SHIFT;
        // Unsafe due to direct memory access; ensure that this operation cannot violate memory safety.
        unsafe {
            let mut bundle = self.flags.get_mut(bundle_index);
            *bundle &= !(1u64 << (index & Self::MASK as usize));
        }
    }

    #[inline(always)]
    pub fn add_unsafely(&mut self, index: usize) {
        // This debug assertion checks that the bit is not already set, replicating the C# Debug.Assert.
        debug_assert_eq!(
            self.flags.get(index >> Self::SHIFT) & (1u64 << (index & Self::MASK as usize)),
            0
        );
        self.set_unsafely(index, index >> Self::SHIFT);
    }

    #[inline(always)]
    pub fn add(&mut self, index: usize, pool: &mut impl IUnmanagedMemoryPool) {
        let bundle_index = index >> Self::SHIFT;
        if bundle_index >= self.flags.length() {
            // Resize might involve re-allocating and copying old values to the new buffer.
            // Unsafe due to potential reallocation and direct memory access.
            unsafe {
                Self::internal_resize_for_bundle_count(&mut self.flags, pool, bundle_index + 1);
            }
        }
        // This debug assertion checks that the bit is not already set, replicating the C# Debug.Assert.
        debug_assert_eq!(
            self.flags.get(index >> Self::SHIFT) & (1u64 << (index & Self::MASK as usize)),
            0
        );
        self.set_unsafely(index, bundle_index);
    }

    #[inline(always)]
    pub fn remove(&mut self, index: usize) {
        // This debug assertion checks that the bit is already set, replicating the C# Debug.Assert.
        debug_assert!(
            self.flags.get(index >> Self::SHIFT) & (1u64 << (index & Self::MASK as usize)) > 0
        );
        self.unset(index);
    }

    pub fn clear(&mut self) {
        self.flags.clear(0, self.flags.len());
    }

    pub fn ensure_capacity(&mut self, index_capacity: usize, pool: &mut impl IUnmanagedMemoryPool) {
        if (self.flags.length() << Self::SHIFT) < index_capacity {
            Self::internal_resize(&mut self.flags, pool, index_capacity);
        }
    }

    pub fn compact(&mut self, index_capacity: usize, pool: &mut impl IUnmanagedMemoryPool) {
        let desired_bundle_count = Self::get_bundle_capacity(index_capacity);
        if self.flags.length() > desired_bundle_count {
            // Unsafe due to potential reallocation and direct memory access.
            unsafe {
                Self::internal_resize_for_bundle_count(&mut self.flags, pool, desired_bundle_count);
            }
        }
    }

    pub fn resize(&mut self, index_capacity: usize, pool: &mut impl IUnmanagedMemoryPool) {
        let desired_bundle_count = Self::get_bundle_capacity(index_capacity);
        if self.flags.length() != desired_bundle_count {
            // Unsafe due to potential reallocation and direct memory access.
            unsafe {
                Self::internal_resize_for_bundle_count(&mut self.flags, pool, desired_bundle_count);
            }
        }
    }

    pub fn dispose(&mut self, pool: &mut impl IUnmanagedMemoryPool) {
        debug_assert!(self.flags.length() > 0, "Cannot double-dispose.");
        pool.return_buffer(&mut self.flags);
        // After disposing, we reset the flags to a new, empty buffer to ensure the structure can be reused safely.
        self.flags = unsafe { Buffer::new() };
    }
}
