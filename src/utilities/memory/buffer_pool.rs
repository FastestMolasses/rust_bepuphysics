use crate::utilities::memory::managed_id_pool::ManagedIdPool;
use std::alloc::{self, Layout};
use std::ptr::NonNull;

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
