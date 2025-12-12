//! Unmanaged memory pool that suballocates from memory blocks pulled from native heap.
//!
//! This is a high-performance memory allocator designed for physics simulations where
//! cache locality and minimal allocation overhead are critical.

use super::buffer::Buffer;
use super::managed_id_pool::ManagedIdPool;
use super::span_helper;
use std::alloc::{self, Layout};
use std::mem::size_of;
use std::ptr;

/// Byte alignment to enforce for all block allocations within the buffer pool.
/// Since this only applies at the level of blocks, we can use a large value without much concern.
pub const BLOCK_ALIGNMENT: usize = 128;

/// Shift value used to encode the power index in the buffer ID.
const ID_POWER_SHIFT: i32 = 26;

/// Internal pool for a specific power-of-2 size.
struct PowerPool {
    /// Array of pointers to allocated blocks.
    blocks: Vec<*mut u8>,
    /// Pool of slots available to this power level.
    slots: ManagedIdPool,
    /// Number of suballocations that fit in each block.
    suballocations_per_block: i32,
    /// Log2 of suballocations_per_block for fast division.
    suballocations_per_block_shift: i32,
    /// Mask for fast modulo of suballocations_per_block.
    suballocations_per_block_mask: i32,
    /// The power (log2) of the suballocation size.
    power: i32,
    /// Size of each suballocation in bytes (2^power).
    suballocation_size: i32,
    /// Size of each block in bytes.
    block_size: i32,
    /// Number of allocated blocks.
    block_count: i32,

    #[cfg(debug_assertions)]
    outstanding_ids: std::collections::HashSet<i32>,
}

impl PowerPool {
    fn new(power: i32, minimum_block_size: i32, expected_pooled_count: i32) -> Self {
        let suballocation_size = 1 << power;
        let block_size = suballocation_size.max(minimum_block_size);
        let suballocations_per_block = block_size / suballocation_size;
        let suballocations_per_block_shift =
            span_helper::get_containing_power_of_2(suballocations_per_block);
        let suballocations_per_block_mask = (1 << suballocations_per_block_shift) - 1;

        Self {
            blocks: Vec::with_capacity(1),
            slots: ManagedIdPool::new(expected_pooled_count),
            suballocations_per_block,
            suballocations_per_block_shift,
            suballocations_per_block_mask,
            power,
            suballocation_size,
            block_size,
            block_count: 0,

            #[cfg(debug_assertions)]
            outstanding_ids: std::collections::HashSet::new(),
        }
    }

    fn resize(&mut self, new_size: usize) {
        self.blocks.resize(new_size, ptr::null_mut());
    }

    unsafe fn allocate_block(&mut self, block_index: i32) {
        #[cfg(debug_assertions)]
        {
            for i in 0..block_index {
                debug_assert!(
                    !self.blocks[i as usize].is_null(),
                    "If we are allocating a block, all previous blocks should be allocated already."
                );
            }
        }
        debug_assert!(
            self.blocks[block_index as usize].is_null(),
            "Block should not already be allocated"
        );

        // Allocate aligned memory for the block
        let layout =
            Layout::from_size_align(self.block_size as usize, BLOCK_ALIGNMENT).expect("Invalid layout");
        let ptr = alloc::alloc(layout);
        if ptr.is_null() {
            alloc::handle_alloc_error(layout);
        }

        self.blocks[block_index as usize] = ptr;
        self.block_count = block_index + 1;
    }

    fn ensure_capacity(&mut self, capacity: i32) {
        let needed_block_count =
            (capacity as f64 / self.block_size as f64).ceil() as i32;
        if self.block_count < needed_block_count {
            if needed_block_count as usize > self.blocks.len() {
                self.resize(needed_block_count as usize);
            }
            for i in self.block_count..needed_block_count {
                unsafe {
                    self.allocate_block(i);
                }
            }
            self.block_count = needed_block_count;
        }
    }

    #[inline(always)]
    fn get_start_pointer_for_slot(&self, slot: i32) -> *mut u8 {
        let block_index = slot >> self.suballocations_per_block_shift;
        let index_in_block = slot & self.suballocations_per_block_mask;
        unsafe {
            self.blocks[block_index as usize]
                .add((index_in_block * self.suballocation_size) as usize)
        }
    }

    fn take(&mut self) -> Buffer<u8> {
        let slot = self.slots.take();
        let block_index = slot >> self.suballocations_per_block_shift;

        if block_index as usize >= self.blocks.len() {
            let new_size = ((block_index + 1) as u32).next_power_of_two() as usize;
            self.resize(new_size);
        }
        if block_index >= self.block_count {
            unsafe {
                self.allocate_block(block_index);
            }
        }

        let index_in_block = slot & self.suballocations_per_block_mask;
        let ptr = unsafe {
            self.blocks[block_index as usize]
                .add((index_in_block * self.suballocation_size) as usize)
        };
        let id = (self.power << ID_POWER_SHIFT) | slot;

        debug_assert!(id >= 0 && self.power >= 0 && self.power < 32);

        #[cfg(debug_assertions)]
        {
            const MAXIMUM_OUTSTANDING_COUNT: usize = 1 << 26;
            debug_assert!(
                self.outstanding_ids.len() < MAXIMUM_OUTSTANDING_COUNT,
                "Do you actually need {} allocations, or is this a memory leak?",
                MAXIMUM_OUTSTANDING_COUNT
            );
            debug_assert!(
                self.outstanding_ids.insert(slot),
                "Should not be able to request the same slot twice."
            );
        }

        Buffer::new(ptr, self.suballocation_size, id)
    }

    #[cfg(debug_assertions)]
    fn validate_buffer_is_contained<T>(&self, buffer: &Buffer<T>) {
        let byte_buffer: Buffer<u8> = buffer.cast();
        let slot_index = byte_buffer.id() & ((1 << ID_POWER_SHIFT) - 1);
        let block_index = slot_index >> self.suballocations_per_block_shift;
        let index_in_block = slot_index & self.suballocations_per_block_mask;

        debug_assert!(
            byte_buffer.len() <= self.suballocation_size,
            "A buffer taken from a pool should have a specific size."
        );
        debug_assert!(
            block_index >= 0 && block_index < self.block_count,
            "The block pointed to by a returned buffer should actually exist within the pool."
        );

        let memory_offset =
            byte_buffer.as_ptr() as isize - self.blocks[block_index as usize] as isize;
        debug_assert!(
            memory_offset >= 0 && memory_offset < self.block_size as isize,
            "If a raw buffer points to a given block as its source, the address should be within the block's memory region."
        );
        debug_assert!(
            unsafe {
                self.blocks[block_index as usize]
                    .add((index_in_block * self.suballocation_size) as usize)
            } == byte_buffer.as_ptr() as *mut u8,
            "The implied address of a buffer in its block should match its actual address."
        );
        debug_assert!(
            byte_buffer.len() + index_in_block * self.suballocation_size <= self.block_size,
            "The extent of the buffer should fit within the block."
        );
    }

    fn return_slot(&mut self, slot_index: i32) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                self.outstanding_ids.remove(&slot_index),
                "This buffer id must have been taken from the pool previously."
            );
        }
        self.slots.return_id(slot_index);
    }

    fn clear(&mut self) {
        #[cfg(debug_assertions)]
        {
            self.outstanding_ids.clear();
        }

        for i in 0..self.block_count as usize {
            if !self.blocks[i].is_null() {
                unsafe {
                    let layout = Layout::from_size_align(self.block_size as usize, BLOCK_ALIGNMENT)
                        .expect("Invalid layout");
                    alloc::dealloc(self.blocks[i], layout);
                    self.blocks[i] = ptr::null_mut();
                }
            }
        }
        self.slots.clear();
        self.block_count = 0;
    }
}

impl Drop for PowerPool {
    fn drop(&mut self) {
        self.clear();
    }
}

/// Unmanaged memory pool that suballocates from memory blocks pulled from native heap.
///
/// Memory is organized into "power pools" where each pool handles allocations of a specific
/// power-of-2 size. This allows O(1) allocation and deallocation with minimal fragmentation.
pub struct BufferPool {
    pools: Vec<PowerPool>,
    minimum_block_size: i32,
}

impl BufferPool {
    /// Creates a new buffer pool.
    ///
    /// # Arguments
    /// * `minimum_block_allocation_size` - Minimum size of individual block allocations. Must be a power of 2.
    ///   Pools with single allocations larger than the minimum will use the minimum value necessary to hold one element.
    ///   Buffers will be suballocated from blocks.
    /// * `expected_pooled_resource_count` - Number of suballocations to preallocate reference space for.
    ///   This does not preallocate actual blocks, just the space to hold references that are waiting in the pool.
    pub fn new(minimum_block_allocation_size: i32, expected_pooled_resource_count: i32) -> Self {
        debug_assert!(
            (minimum_block_allocation_size & (minimum_block_allocation_size - 1)) == 0,
            "Block allocation size must be a power of 2."
        );

        let mut pools = Vec::with_capacity(span_helper::MAXIMUM_SPAN_SIZE_POWER as usize + 1);
        for power in 0..=span_helper::MAXIMUM_SPAN_SIZE_POWER {
            pools.push(PowerPool::new(
                power,
                minimum_block_allocation_size,
                expected_pooled_resource_count,
            ));
        }

        Self {
            pools,
            minimum_block_size: minimum_block_allocation_size,
        }
    }

    /// Creates a new buffer pool with default settings.
    pub fn new_default() -> Self {
        Self::new(131072, 16)
    }

    /// Creates a new buffer pool with a custom minimum block size.
    pub fn new_with_minimum_block_size(minimum_block_size: i32) -> Self {
        Self::new(minimum_block_size, 16)
    }

    /// Ensures that the pool associated with a given power has at least a certain amount of capacity.
    pub fn ensure_capacity_for_power(&mut self, byte_count: i32, power: i32) {
        span_helper::validate_power(power);
        self.pools[power as usize].ensure_capacity(byte_count);
    }

    /// Gets the capacity allocated for a power.
    pub fn get_capacity_for_power(&self, power: i32) -> i32 {
        span_helper::validate_power(power);
        let pool = &self.pools[power as usize];
        pool.block_count * pool.block_size
    }

    /// Computes the total number of bytes allocated from native memory in this buffer pool.
    pub fn get_total_allocated_byte_count(&self) -> u64 {
        let mut sum: u64 = 0;
        for pool in &self.pools {
            sum += (pool.block_count as u64) * (pool.block_size as u64);
        }
        sum
    }

    /// Takes a buffer large enough to contain a number of elements of a given type.
    /// Capacity may be larger than requested.
    #[inline(always)]
    pub fn take_at_least<T>(&mut self, count: i32) -> Buffer<T> {
        // Avoid returning a zero length span
        let count = if count == 0 { 1 } else { count };
        let power = span_helper::get_containing_power_of_2(count * size_of::<T>() as i32);
        let raw_buffer = self.take_for_power(power);
        raw_buffer.cast()
    }

    /// Takes a typed buffer of the requested size from the pool.
    #[inline(always)]
    pub fn take<T>(&mut self, count: i32) -> Buffer<T> {
        let mut buffer = self.take_at_least(count);
        buffer.set_length(count);
        buffer
    }

    /// Takes a buffer large enough to contain a number of bytes given by a power.
    #[inline(always)]
    pub fn take_for_power(&mut self, power: i32) -> Buffer<u8> {
        debug_assert!(power >= 0 && power <= span_helper::MAXIMUM_SPAN_SIZE_POWER);
        self.pools[power as usize].take()
    }

    /// Decomposes a buffer ID into power index and slot index.
    #[inline(always)]
    pub fn decompose_id(buffer_id: i32) -> (i32, i32) {
        let power_index = buffer_id >> ID_POWER_SHIFT;
        let slot_index = buffer_id & ((1 << ID_POWER_SHIFT) - 1);
        (power_index, slot_index)
    }

    /// Returns a buffer to the pool by id without clearing the buffer reference.
    #[inline(always)]
    pub fn return_unsafely(&mut self, id: i32) {
        let (power_index, slot_index) = Self::decompose_id(id);
        self.pools[power_index as usize].return_slot(slot_index);
    }

    /// Returns a buffer to the pool.
    #[inline(always)]
    pub fn return_buffer<T>(&mut self, buffer: &mut Buffer<T>) {
        #[cfg(debug_assertions)]
        {
            let (power_index, _) = Self::decompose_id(buffer.id());
            self.pools[power_index as usize].validate_buffer_is_contained(buffer);
        }
        self.return_unsafely(buffer.id());
        *buffer = Buffer::default();
    }

    /// Resizes a typed buffer to the smallest size available in the pool which contains the target size.
    pub fn resize_to_at_least<T: Copy>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32) {
        debug_assert!(
            copy_count <= target_size && copy_count <= buffer.len(),
            "Can't copy more elements than exist in the source or target buffers."
        );

        let target_size = Self::get_capacity_for_count::<T>(target_size);

        if !buffer.allocated() {
            debug_assert!(
                buffer.len() == 0,
                "If a buffer is pointing at null, then it should be default initialized and have a length of zero too."
            );
            *buffer = self.take_at_least(target_size);
        } else {
            let original_allocated_size_in_bytes = 1 << (buffer.id() >> ID_POWER_SHIFT);
            let original_allocated_size = original_allocated_size_in_bytes / size_of::<T>() as i32;

            debug_assert!(
                original_allocated_size >= buffer.len(),
                "The original allocated capacity must be sufficient for the buffer's observed length."
            );

            if target_size > original_allocated_size {
                // The original allocation isn't big enough; allocate a new buffer
                let mut new_buffer: Buffer<T> = self.take_at_least(target_size);
                buffer.copy_to(0, &mut new_buffer, 0, copy_count);
                self.return_unsafely(buffer.id());
                *buffer = new_buffer;
            } else {
                // Original allocation is large enough; just expose the full original size
                buffer.set_length(original_allocated_size);
            }
        }
    }

    /// Resizes a buffer to the target size.
    pub fn resize<T: Copy>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32) {
        self.resize_to_at_least(buffer, target_size, copy_count);
        buffer.set_length(target_size);
    }

    /// Gets the capacity of a buffer that would be returned by the pool for a given element count.
    #[inline(always)]
    pub fn get_capacity_for_count<T>(count: i32) -> i32 {
        let count = if count == 0 { 1 } else { count };
        let byte_count = count as u32 * size_of::<T>() as u32;
        let rounded = byte_count.next_power_of_two();
        (rounded / size_of::<T>() as u32) as i32
    }

    /// Issues debug assertions that all pools are empty.
    #[cfg(debug_assertions)]
    pub fn assert_empty(&self) {
        for (i, pool) in self.pools.iter().enumerate() {
            if !pool.outstanding_ids.is_empty() {
                eprintln!("Power pool {} contains allocations.", i);
                debug_assert!(pool.outstanding_ids.is_empty());
            }
        }
    }

    /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated silently.
    /// The pool will remain in a usable state after clearing.
    pub fn clear(&mut self) {
        for pool in &mut self.pools {
            pool.clear();
        }
    }
}

impl Drop for BufferPool {
    fn drop(&mut self) {
        #[cfg(debug_assertions)]
        {
            let total_block_count: i32 = self.pools.iter().map(|p| p.block_count).sum();
            debug_assert!(
                total_block_count == 0,
                "Memory leak warning! Don't let a buffer pool die without clearing it!"
            );
        }
    }
}

// Implement Send and Sync - BufferPool is safe to send between threads,
// though not safe to share without synchronization
unsafe impl Send for BufferPool {}

impl super::unmanaged_mempool::UnmanagedMemoryPool for BufferPool {
    #[inline(always)]
    fn take_at_least<T>(&mut self, count: i32) -> Buffer<T> {
        BufferPool::take_at_least(self, count)
    }

    #[inline(always)]
    fn take<T>(&mut self, count: i32) -> Buffer<T> {
        BufferPool::take(self, count)
    }

    #[inline(always)]
    fn return_buffer<T>(&mut self, buffer: &mut Buffer<T>) {
        BufferPool::return_buffer(self, buffer)
    }

    #[inline(always)]
    fn get_capacity_for_count<T>(count: i32) -> i32 {
        BufferPool::get_capacity_for_count::<T>(count)
    }

    #[inline(always)]
    fn return_unsafely(&mut self, id: i32) {
        BufferPool::return_unsafely(self, id)
    }

    #[inline(always)]
    fn resize_to_at_least<T: Copy>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32) {
        BufferPool::resize_to_at_least(self, buffer, target_size, copy_count)
    }

    #[inline(always)]
    fn resize<T: Copy>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32) {
        BufferPool::resize(self, buffer, target_size, copy_count)
    }

    #[inline(always)]
    fn clear(&mut self) {
        BufferPool::clear(self)
    }

    #[inline(always)]
    fn get_total_allocated_byte_count(&self) -> u64 {
        BufferPool::get_total_allocated_byte_count(self)
    }
}
