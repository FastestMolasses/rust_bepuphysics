use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;
use crate::utilities::memory::{
    managed_id_pool::ManagedIdPool, span_helper::MAXIMUM_SPAN_SIZE_POWER,
};
use std::alloc::{self, Layout};
use std::collections::HashSet;
use std::ptr::NonNull;

struct PowerPool {
    blocks: Vec<NonNull<u8>>,
    slots: ManagedIdPool,
    #[cfg(debug_assertions)]
    outstanding_ids: HashSet<usize>,
    #[cfg(all(debug_assertions, feature = "leak_debug"))]
    outstanding_allocators: HashMap<String, HashSet<usize>>,
    suballocations_per_block: usize,
    suballocations_per_block_shift: usize,
    suballocations_per_block_mask: usize,
    power: usize,
    suballocation_size: usize,
    block_size: usize,
    block_count: usize,
}

impl PowerPool {
    const ID_POWER_SHIFT: usize = 26;
    const BLOCK_ALIGNMENT: usize = 128;

    fn new(power: usize, minimum_block_size: usize, expected_pooled_count: usize) -> Self {
        let suballocation_size = 1 << power;
        let block_size = suballocation_size.max(minimum_block_size);
        let suballocations_per_block = block_size / suballocation_size;
        let suballocations_per_block_shift = suballocations_per_block
            .next_power_of_two()
            .trailing_zeros() as usize;
        let suballocations_per_block_mask = (1 << suballocations_per_block_shift) - 1;

        Self {
            blocks: Vec::new(),
            slots: ManagedIdPool::new(expected_pooled_count),
            #[cfg(debug_assertions)]
            outstanding_ids: HashSet::new(),
            #[cfg(all(debug_assertions, feature = "leak_debug"))]
            outstanding_allocators: HashMap::new(),
            suballocations_per_block,
            suballocations_per_block_shift,
            suballocations_per_block_mask,
            power,
            suballocation_size,
            block_size,
            block_count: 0,
        }
    }

    fn resize(&mut self, new_size: usize) {
        self.blocks.resize_with(new_size, || NonNull::dangling());
    }

    unsafe fn allocate_block(&mut self, block_index: usize) {
        #[cfg(debug_assertions)]
        {
            // Ensure all previous blocks are allocated before this one.
            for i in 0..block_index {
                debug_assert!(
                    !self.blocks[i].as_ptr().is_null(),
                    "All previous blocks should be allocated already."
                );
            }
        }

        debug_assert!(
            self.blocks
                .get(block_index)
                .map_or(true, |&ptr| ptr.as_ptr().is_null()),
            "Block should not be allocated already."
        );

        let layout = Layout::from_size_align(self.block_size, Self::BLOCK_ALIGNMENT)
            .expect("Layout creation failed");

        let ptr = alloc::alloc(layout);
        if ptr.is_null() {
            panic!("Memory allocation failed");
        }

        self.blocks[block_index] = NonNull::new(ptr).expect("NonNull failed after non-null check");
        self.block_count = block_index + 1;
    }

    fn ensure_capacity(&mut self, capacity: usize) {
        let needed_block_count = (capacity + self.block_size - 1) / self.block_size;
        if self.block_count < needed_block_count {
            if needed_block_count > self.blocks.len() {
                self.resize(needed_block_count);
            }
            for i in self.block_count..needed_block_count {
                unsafe { self.allocate_block(i) };
            }
        }
    }

    fn get_start_pointer_for_slot(&self, slot: usize) -> *mut u8 {
        let block_index = slot >> self.suballocations_per_block_shift;
        let index_in_block = slot & self.suballocations_per_block_mask;
        unsafe {
            self.blocks[block_index]
                .as_ptr()
                .add(index_in_block * self.suballocation_size)
        }
    }

    fn take(&mut self) -> Buffer<u8> {
        let slot = self.slots.take();
        let block_index = slot >> self.suballocations_per_block_shift;
        // Adjust resizing to ensure it accommodates the new block index adequately
        if block_index >= self.blocks.len() {
            self.resize(block_index.next_power_of_two());
        }
        // Ensure the block is allocated if not already present
        if block_index >= self.block_count {
            // Note: No longer using atomic operations
            unsafe {
                self.allocate_block(block_index);
            }
        }
        let ptr = unsafe { self.get_start_pointer_for_slot(slot) };
        unsafe {
            Buffer::new(
                ptr,
                self.suballocation_size,
                (self.power << Self::ID_POWER_SHIFT) | slot,
            )
        }
    }

    #[cfg(debug_assertions)]
    fn validate_buffer_is_contained<T>(&self, buffer: &Buffer<T>)
    where
        T: std::marker::Sized, // Ensure T has a known size at compile time
    {
        // Safety: Reinterpreting buffer memory as byte slice is unsafe
        // because it assumes T can be represented as a sequence of bytes.
        let buffer_memory_as_bytes = unsafe {
            std::slice::from_raw_parts(
                buffer.memory.as_ptr() as *const u8,
                buffer.len() * std::mem::size_of::<T>(),
            )
        };

        let slot_index = (buffer.id & ((1 << Self::ID_POWER_SHIFT) - 1)) as usize;
        let block_index = slot_index >> self.suballocations_per_block_shift;
        let index_in_allocator_block = slot_index & self.suballocations_per_block_mask;

        debug_assert!(
            buffer_memory_as_bytes.len() <= self.suballocation_size,
            "A buffer taken from a pool should have a specific size."
        );
        debug_assert!(
            block_index < self.block_count,
            "The block pointed to by a returned buffer should actually exist within the pool."
        );

        let memory_offset = unsafe {
            buffer
                .memory
                .as_ptr()
                .offset_from(self.blocks[block_index].as_ptr() as *const T) as isize
        } * std::mem::size_of::<T>() as isize;
        debug_assert!(
            memory_offset >= 0 && (memory_offset as usize) < self.block_size,
            "If a raw buffer points to a given block as its source, the address should be within the block's memory region."
        );
        debug_assert!(
            unsafe {
                self.blocks[block_index].as_ptr().add(
                    index_in_allocator_block * self.suballocation_size / std::mem::size_of::<T>(),
                ) as *const u8
            } == buffer.memory.as_ptr() as *const u8,
            "The implied address of a buffer in its block should match its actual address."
        );
        debug_assert!(
            buffer_memory_as_bytes.len() + index_in_allocator_block * self.suballocation_size
                <= self.block_size,
            "The extent of the buffer should fit within the block."
        );
    }

    fn return_slot(&mut self, slot_index: usize) {
        #[cfg(debug_assertions)]
        {
            assert!(
                self.outstanding_ids.remove(&slot_index),
                "This buffer id must have been taken from the pool previously."
            );
            #[cfg(all(debug_assertions, feature = "leak_debug"))]
            {
                let found = self.outstanding_allocators.iter_mut().any(|(_key, ids)| {
                    if ids.remove(&slot_index) {
                        if ids.is_empty() {
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                });
                assert!(found, "Allocator set must contain the buffer id.");
            }
        }
        self.slots.return_id(slot_index);
    }

    fn clear(&mut self) {
        #[cfg(debug_assertions)]
        {
            self.outstanding_ids.clear();
            #[cfg(all(debug_assertions, feature = "leak_debug"))]
            {
                self.outstanding_allocators.clear();
            }
        }
        // Properly deallocate memory for each block
        for block in &mut self.blocks {
            unsafe {
                let layout = std::alloc::Layout::from_size_align_unchecked(
                    self.block_size,
                    Self::BLOCK_ALIGNMENT,
                );
                std::alloc::dealloc(block.as_ptr(), layout);
            }
            *block = NonNull::dangling(); // Reset pointer after deallocation
        }
        self.blocks.clear();
        self.slots.clear();
        self.block_count = 0;
    }
}

pub struct BufferPool {
    pools: Vec<PowerPool>,
}

impl BufferPool {
    #[inline(always)]
    pub fn new(
        minimum_block_allocation_size: usize,
        expected_pooled_resource_count: usize,
    ) -> Self {
        #[cfg(debug_assertions)]
        debug_assert!(
            minimum_block_allocation_size.is_power_of_two(),
            "Block allocation size must be a power of 2."
        );
        let pools = (0..=MAXIMUM_SPAN_SIZE_POWER)
            .map(|power| {
                PowerPool::new(
                    power,
                    minimum_block_allocation_size,
                    expected_pooled_resource_count,
                )
            })
            .collect();
        Self { pools }
    }

    #[inline(always)]
    pub fn new_with_minimum_block_size(minimum_block_size: usize) -> Self {
        Self::new(minimum_block_size, 16)
    }

    /// Ensures the specified power pool has the capacity to handle a given number of bytes
    #[inline(always)]
    pub fn ensure_capacity_for_power(&mut self, byte_count: usize, power: usize) {
        self.pools[power].ensure_capacity(byte_count);
    }

    #[inline(always)]
    pub fn get_capacity_for_power(&self, power: usize) -> usize {
        self.pools[power].block_count * self.pools[power].block_size
    }

    #[inline(always)]
    pub fn take_for_power(&mut self, power: usize) -> Buffer<u8> {
        assert!(power <= MAXIMUM_SPAN_SIZE_POWER);
        self.pools[power].take()
    }

    /// Decomposes a buffer ID into its power index and slot index components.
    #[inline(always)]
    fn decompose_id(buffer_id: i32) -> (usize, usize) {
        let power_index = (buffer_id >> PowerPool::ID_POWER_SHIFT) as usize;
        let slot_index = (buffer_id & ((1 << PowerPool::ID_POWER_SHIFT) - 1)) as usize;
        (power_index, slot_index)
    }

    /// Returns a buffer to the pool unsafely by its ID, without performing any checks.
    #[inline(always)]
    pub fn return_unsafely(&mut self, id: i32) {
        let (power_index, slot_index) = Self::decompose_id(id);
        self.pools[power_index].return_slot(slot_index);
    }

    #[cfg(debug_assertions)]
    pub fn assert_empty(&self) {
        for (i, pool) in self.pools.iter().enumerate() {
            if !pool.outstanding_ids.is_empty() {
                eprintln!("Power pool {} contains allocations.", i);
                #[cfg(feature = "leak_debug")]
                for (allocator, ids) in &pool.outstanding_allocators {
                    eprintln!("{} ALLOCATION COUNT: {}", allocator, ids.len());
                }
                debug_assert!(pool.outstanding_ids.is_empty(), "Pool is not empty");
            }
        }
    }
}

impl UnmanagedMemoryPool for BufferPool {
    #[inline(always)]
    fn take_at_least<T>(&mut self, count: i32, buffer: &mut Buffer<T>)
    where
        T: Copy,
    {
        let count = count.max(1); // Avoid zero-length spans
        let total_size = count as usize * std::mem::size_of::<T>();
        let power = total_size.next_power_of_two().trailing_zeros() as usize;
        let mut raw_buffer = self.take_for_power(power);
        unsafe { raw_buffer.reinterpret_as::<T>() }
    }

    #[inline(always)]
    fn take<T>(&mut self, count: i32, buffer: &mut Buffer<T>)
    where
        T: Copy,
    {
        let mut buffer = self.take_at_least(count);
        buffer.length = count;
        buffer
    }

    /// Returns a buffer to the appropriate pool
    #[inline(always)]
    fn return_to_pool<T>(&mut self, buffer: &mut Buffer<T>)
    where
        T: Copy + Sized,
    {
        #[cfg(debug_assertions)]
        {
            let (power_index, slot_index) = Self::decompose_id(buffer.id);
            self.pools[power_index].validate_buffer_is_contained(buffer);
        }

        self.return_unsafely(buffer.id);
        *buffer = Buffer::default();
    }

    #[inline(always)]
    fn get_capacity_for_count<T>(count: i32) -> i32
    where
        T: Sized,
    {
        let count = count.max(1); // Ensure at least 1
        let size_in_bytes = count as usize * std::mem::size_of::<T>();
        let rounded_up = size_in_bytes.next_power_of_two();
        debug_assert!(
            rounded_up as u64 * std::mem::size_of::<T>() as u64 <= i32::MAX as u64,
            "This function assumes that counts aren't going to overflow a signed 32 bit integer."
        );
        rounded_up / std::mem::size_of::<T>()
    }

    #[inline(always)]
    fn return_unsafely(&mut self, id: i32) {
        let (power_index, slot_index) = BufferPool::decompose_id(id);
        self.pools[power_index].return_slot(slot_index);
    }

    #[inline(always)]
    fn resize_to_at_least<T>(
        &mut self,
        buffer: &mut Buffer<T>,
        target_size: i32,
        copy_count: i32,
    ) where
        T: Copy,
    {
        debug_assert!(
            copy_count <= target_size && copy_count <= buffer.length(),
            "Can't copy more elements than exist in the source or target buffers."
        );
        let target_capacity = self.get_capacity_for_count::<T>(target_size);

        if !buffer.allocated() {
            debug_assert!(buffer.length() == 0, "If a buffer is pointing at null, then it should be default initialized and have a length of zero too.");
            *buffer = self.take_at_least::<T>(target_capacity);
        } else {
            let original_allocated_size_in_bytes = 1 << (buffer.id >> PowerPool::ID_POWER_SHIFT);
            let original_allocated_size =
                original_allocated_size_in_bytes / std::mem::size_of::<T>();
            debug_assert!(original_allocated_size >= buffer.length(), "The original allocated capacity must be sufficient for the buffer's observed length.");

            if target_capacity > original_allocated_size {
                let mut new_buffer = self.take_at_least::<T>(target_capacity);
                buffer.copy_to(0, &mut new_buffer, 0, copy_count);
                self.return_unsafely(buffer.id);
                *buffer = new_buffer;
            } else {
                // Original allocation is large enough; just update the length.
                buffer.set_length(original_allocated_size);
            }
        }
    }

    #[inline(always)]
    fn resize<T>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32)
    where
        T: Copy,
    {
        self.resize_to_at_least(buffer, target_size, copy_count);
        buffer.set_length(target_size);
    }

    /// Clears all pools, deallocating all memory
    #[inline(always)]
    fn clear(&mut self) {
        for pool in &mut self.pools {
            pool.clear();
        }
    }

    #[inline(always)]
    fn get_total_allocated_byte_count(&self) -> u64 {
        self.pools
            .iter()
            .map(|pool| pool.block_count as u64 * pool.block_size as u64)
            .sum()
    }
}

impl Default for BufferPool {
    fn default() -> Self {
        Self::new(131072, 16)
    }
}

impl Drop for BufferPool {
    #[inline(always)]
    fn drop(&mut self) {
        self.clear();
        #[cfg(debug_assertions)]
        {
            let total_block_count: usize = self.pools.iter().map(|p| p.block_count).sum();
            debug_assert!(
                total_block_count == 0,
                "Memory leak warning! Don't let a buffer pool die without clearing it!"
            );
        }
    }
}
