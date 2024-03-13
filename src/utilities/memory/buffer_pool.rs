use std::alloc::{self, Layout};
use std::ptr::NonNull;

use crate::utilities::memory::{
    managed_id_pool::ManagedIdPool, span_helper::MAXIMUM_SPAN_SIZE_POWER,
};

struct PowerPool {
    blocks: Vec<NonNull<u8>>,
    slots: ManagedIdPool,
    suballocations_per_block_shift: usize,
    suballocation_size: usize,
    block_size: usize,
}

impl PowerPool {
    const BLOCK_ALIGNMENT: usize = 128;

    pub fn new(power: usize, minimum_block_size: usize, expected_pooled_count: usize) -> Self {
        let suballocation_size = 1 << power;
        let block_size = suballocation_size.max(minimum_block_size);
        let suballocations_per_block = block_size / suballocation_size;
        let suballocations_per_block_shift = suballocations_per_block.trailing_zeros() as usize;

        Self {
            blocks: Vec::new(),
            slots: ManagedIdPool::new(expected_pooled_count),
            suballocations_per_block_shift,
            suballocation_size,
            block_size,
        }
    }

    unsafe fn allocate_block(&mut self) {
        let layout = Layout::from_size_align(self.block_size, Self::BLOCK_ALIGNMENT)
            .expect("Layout creation failed");
        let block_ptr = alloc::alloc(layout);
        self.blocks
            .push(NonNull::new(block_ptr).expect("Allocation failed"));
    }

    pub fn ensure_capacity(&mut self, capacity: usize) {
        let needed_blocks = (capacity + self.block_size - 1) / self.block_size;
        while self.blocks.len() < needed_blocks {
            unsafe {
                self.allocate_block();
            }
        }
    }

    pub unsafe fn take(&mut self) -> NonNull<u8> {
        let slot = self.slots.take();
        let block_index = slot >> self.suballocations_per_block_shift;
        if block_index >= self.blocks.len() {
            self.ensure_capacity(block_index + 1);
        }
        let index_in_block = slot & ((1 << self.suballocations_per_block_shift) - 1);
        let offset = index_in_block * self.suballocation_size;
        NonNull::new_unchecked(self.blocks[block_index].as_ptr().add(offset))
    }

    pub fn return_buffer(&mut self, slot_index: usize) {
        self.slots.return_id(slot_index);
    }

    pub fn clear(&mut self) {
        for block in self.blocks.drain(..) {
            unsafe {
                let layout =
                    Layout::from_size_align_unchecked(self.block_size, Self::BLOCK_ALIGNMENT);
                alloc::dealloc(block.as_ptr(), layout);
            }
        }
        self.slots.clear();
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
        let needed_capacity = (byte_count + self.pools[power].suballocation_size - 1)
            / self.pools[power].suballocation_size;
        self.pools[power].slots.ensure_capacity(needed_capacity);
    }

    // Takes a buffer of at least the specified size
    pub unsafe fn take_at_least<T>(&mut self, count: usize) -> *mut T
    where
        T: Sized,
    {
        let type_size = std::mem::size_of::<T>();
        let total_size = count * type_size;
        let power = total_size.next_power_of_two().trailing_zeros() as usize;
        self.ensure_capacity_for_power(total_size, power);
        self.pools[power].take() as *mut T
    }

    /// Returns a buffer to the appropriate pool
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
