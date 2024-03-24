use crate::utilities::memory::buffer::Buffer;
use std::alloc::{self, Layout};
use std::collections::HashSet;
use std::ptr::NonNull;

use crate::utilities::memory::{
    managed_id_pool::ManagedIdPool, span_helper::MAXIMUM_SPAN_SIZE_POWER,
};

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
        Buffer::new(
            ptr,
            self.suballocation_size,
            (self.power << Self::ID_POWER_SHIFT) | slot,
        )
    }

    #[cfg(debug_assertions)]
    fn validate_buffer_is_contained<T>(&self, buffer: &Buffer<T>)
    where
        T: std::marker::Copy, // Ensure T can be safely reinterpreted as bytes
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
            buffer.memory.as_ptr().offset_from(self.blocks[block_index].as_ptr() as *const T) as isize
        } * std::mem::size_of::<T>() as isize;
        debug_assert!(
            memory_offset >= 0 && (memory_offset as usize) < self.block_size,
            "If a raw buffer points to a given block as its source, the address should be within the block's memory region."
        );
        debug_assert!(
            unsafe {
                self.blocks[block_index].as_ptr().add(index_in_allocator_block * self.suballocation_size / std::mem::size_of::<T>()) as *const u8
            } == buffer.memory.as_ptr() as *const u8,
            "The implied address of a buffer in its block should match its actual address."
        );
        debug_assert!(
            buffer_memory_as_bytes.len() + index_in_allocator_block * self.suballocation_size <= self.block_size,
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

    /// Ensures the specified power pool has the capacity to handle a given number of bytes
    pub fn ensure_capacity_for_power(&mut self, byte_count: usize, power: usize) {
        self.pools[power].ensure_capacity(byte_count);
    }

    pub fn get_capacity_for_power(&self, power: usize) -> usize {
        self.pools[power].block_count * self.pools[power].block_size
    }

    pub fn get_total_allocated_byte_count(&self) -> u64 {
        self.pools.iter()
            .map(|pool| pool.block_count as u64 * pool.block_size as u64)
            .sum()
    }

    #[inline]
    pub fn take_at_least<T>(&mut self, count: usize) -> Buffer<T> 
    where
        T: Copy,
    {
        let count = count.max(1); // Avoid zero-length spans
        let total_size = count * std::mem::size_of::<T>();
        let power = total_size.next_power_of_two().trailing_zeros() as usize;
        let mut raw_buffer = self.take_for_power(power);
        unsafe {
            raw_buffer.reinterpret_as::<T>()
        }
    }

    #[inline]
    pub fn take<T>(&mut self, count: usize) -> Buffer<T> 
    where
        T: Copy,
    {
        let mut buffer = self.take_at_least(count);
        buffer.length = count;
        buffer
    }

    #[inline]
    pub fn take_for_power(&mut self, power: usize) -> Buffer<u8> {
        assert!(power <= MAXIMUM_SPAN_SIZE_POWER);
        self.pools[power].take()
    }

    /// Returns a buffer to the appropriate pool
    #[inline]
    pub fn return_buffer<T>(&mut self, buffer: *mut T, count: usize)
    where
        T: Sized,
    {
        let type_size = std::mem::size_of::<T>();
        let total_size = count * type_size;
        let power = total_size.next_power_of_two().trailing_zeros() as usize;
        let slot = buffer as usize >> self.pools[power].suballocations_per_block_shift;
        self.pools[power].return_buffer(slot);
    }

    /// Clears all pools, deallocating all memory
    pub unsafe fn clear(&mut self) {
        for pool in &mut self.pools {
            pool.clear();
        }
    }
}
