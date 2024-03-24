use std::{collections::HashMap, ptr::NonNull};

use crate::utilities::memory::{
    buffer::Buffer, buffer_pool::IUnmanagedMemoryPool, managed_id_pool::ManagedIdPool,
};

/// Represents a chunk of abstract memory supporting allocations and deallocations.
/// Never moves any memory.
///
/// Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations. Not particularly efficient.
pub struct Allocator {
    pool: Box<dyn IUnmanagedMemoryPool>,
    capacity: usize,
    search_start_index: usize,
    allocations: HashMap<u64, Allocation>,
}

struct Allocation {
    start: usize,
    end: usize,
    previous: u64,
    next: u64,
}

impl Allocator {
    /// Creates a new allocator.
    ///
    /// # Arguments
    /// * `capacity`: Size of the memory handled by the allocator in elements.
    /// * `pool`: Pool to pull internal resources from.
    pub fn new(capacity: usize, pool: Box<dyn IUnmanagedMemoryPool>) -> Self {
        Self {
            pool,
            capacity,
            search_start_index: 0,
            allocations: HashMap::new(),
        }
    }

    /// Checks if the id is currently allocated.
    pub fn contains(&self, id: u64) -> bool {
        self.allocations.contains_key(&id)
    }

    /// Gets the allocation region associated with the given allocation id if it is present.
    pub fn get_allocation_region(&self, allocation_id: u64) -> Option<&Allocation> {
        self.allocations.get(&allocation_id)
    }

    /// Checks if a block of memory can fit into the current state of the allocator.
    pub fn can_fit(&self, size: usize) -> bool {
        if self.allocations.is_empty() {
            return size <= self.capacity;
        }

        let mut allocation_index = self.search_start_index;
        let initial_id = *self.allocations.keys().nth(allocation_index).unwrap();

        loop {
            let allocation = self.allocations.get(&initial_id).unwrap();

            let next_allocation_id = allocation.next;
            let next_allocation = self.allocations.get(&next_allocation_id).unwrap();

            if next_allocation.start < allocation.end {
                // Wrapped around, so the gap goes from here to the end of the memory block,
                // and from the beginning of the memory block to the next allocation.
                // But we need contiguous space so the two areas have to be tested independently.
                if self.capacity - allocation.end >= size {
                    return true;
                } else if next_allocation.start >= size {
                    return true;
                }
            } else {
                // The next allocation is in order.
                if next_allocation.start - allocation.end >= size {
                    return true;
                }
            }

            // If we get here, no open space was found.
            // Move on to the next spot.
            allocation_index = self
                .allocations
                .keys()
                .position(|&id| id == next_allocation_id)
                .unwrap();

            // Have we already wrapped around?
            if *self.allocations.keys().nth(allocation_index).unwrap() == initial_id {
                // Wrapped around without finding any space.
                return false;
            }
        }
    }

    /// Attempts to allocate a range of memory.
    pub fn allocate(&mut self, id: u64, size: usize) -> Option<usize> {
        if self.allocations.contains_key(&id) {
            return None; // Id must not already be present.
        }

        if self.allocations.is_empty() {
            // If it's the first allocation, then the next and previous pointers should circle around.
            if size <= self.capacity {
                self.allocations.insert(
                    id,
                    Allocation {
                        start: 0,
                        end: size,
                        next: id,
                        previous: id,
                    },
                );
                self.search_start_index = 0;
                return Some(0);
            }
            return None;
        }

        let mut allocation_index = self.search_start_index;
        let initial_id = *self.allocations.keys().nth(allocation_index).unwrap();

        loop {
            let allocation = self.allocations.get(&initial_id).unwrap();

            let next_allocation_id = allocation.next;
            let next_allocation = self.allocations.get(&next_allocation_id).unwrap();

            if next_allocation.start < allocation.end {
                // Wrapped around, so the gap goes from here to the end of the memory block,
                // and from the beginning of the memory block to the next allocation.
                // But we need contiguous space so the two areas have to be tested independently.
                if self.capacity - allocation.end >= size {
                    self.allocations.insert(
                        id,
                        Allocation {
                            start: allocation.end,
                            end: allocation.end + size,
                            next: allocation.next,
                            previous: initial_id,
                        },
                    );
                    self.search_start_index = self.allocations.len() - 1;
                    return Some(allocation.end);
                } else if next_allocation.start >= size {
                    self.allocations.insert(
                        id,
                        Allocation {
                            start: 0,
                            end: size,
                            next: allocation.next,
                            previous: initial_id,
                        },
                    );
                    self.search_start_index = self.allocations.len() - 1;
                    return Some(0);
                }
            } else {
                // The next allocation is in order.
                if next_allocation.start - allocation.end >= size {
                    self.allocations.insert(
                        id,
                        Allocation {
                            start: allocation.end,
                            end: allocation.end + size,
                            next: allocation.next,
                            previous: initial_id,
                        },
                    );
                    self.search_start_index = self.allocations.len() - 1;
                    return Some(allocation.end);
                }
            }

            // If we get here, no open space was found.
            // Move on to the next spot.
            allocation_index = self
                .allocations
                .keys()
                .position(|&id| id == next_allocation_id)
                .unwrap();

            // Have we already wrapped around?
            if *self.allocations.keys().nth(allocation_index).unwrap() == initial_id {
                // Wrapped around without finding any space.
                return None;
            }
        }
    }

    /// Removes the memory associated with the id from the pool.
    pub fn deallocate(&mut self, id: u64) -> bool {
        if let Some(allocation) = self.allocations.remove(&id) {
            if allocation.previous != id {
                // Make the previous allocation point to the next allocation to get rid of the current allocation.
                self.allocations.get_mut(&allocation.previous).unwrap().next = allocation.next;

                // Make the next allocation point to the previous allocation to get rid of the current allocation.
                self.allocations.get_mut(&allocation.next).unwrap().previous = allocation.previous;
            }

            // By removing this id, a promising place to look for an allocation next time is the position next to the previous allocation!
            self.search_start_index = self
                .allocations
                .keys()
                .position(|&id| id == allocation.previous)
                .unwrap_or(0);

            true
        } else {
            false
        }
    }

    /// Gets the size of the largest contiguous area and the total free space in the allocator.
    /// Not very efficient; runs in linear time for the number of allocations.
    pub fn get_largest_contiguous_size(&self) -> (usize, usize) {
        if self.allocations.is_empty() {
            return (self.capacity, self.capacity);
        }

        let mut largest_contiguous = 0;
        let mut total_free_space = 0;

        for allocation in self.allocations.values() {
            let next_allocation = self.allocations.get(&allocation.next).unwrap();

            let to_next = next_allocation.start.wrapping_sub(allocation.end);

            if to_next < 0 {
                // The next allocation requires a wrap, so the actual contiguous area is only from our end to the end of the pool,
                // and then a second region from 0 to the next allocation.
                let adjacent = self.capacity - allocation.end;
                let wrapped = next_allocation.start;

                largest_contiguous = largest_contiguous.max(adjacent).max(wrapped);
                total_free_space += adjacent + wrapped;
            } else {
                largest_contiguous = largest_contiguous.max(to_next);
                total_free_space += to_next;
            }
        }

        (largest_contiguous, total_free_space)
    }

    /// Finds the first allocation with empty space before it and pulls it forward to close the gap. Assumes the ability to perform synchronous reallocation.
    pub fn incremental_compact(&mut self) -> Option<(u64, usize, usize, usize)> {
        // Find the allocation nearest to the zero index. Identify it by checking for the previous allocation requiring a wraparound.
        // Start at the beginning of the list since it's marginally more likely to be there than at the end of the list where new allocations get appended.
        for allocation in self.allocations.values() {
            let previous_allocation = self.allocations.get(&allocation.previous).unwrap();

            if previous_allocation.end > allocation.start {
                // Found the beginning of the list! This index is the first index.
                // Now, scan forward through the allocation links looking for the first gap.
                let mut index = self
                    .allocations
                    .keys()
                    .position(|&id| id == allocation.previous)
                    .unwrap();
                let mut previous_end = 0;

                // Note that we stop before wrapping.
                for _ in 0..self.allocations.len() {
                    self.search_start_index = index; // If the traversal ends, we want to have this index cached so that the next allocation will start at the end of the contiguous block.

                    let allocation = self.allocations.values().nth(index).unwrap();

                    if allocation.start > previous_end {
                        // Found a gap.
                        let id = *self.allocations.keys().nth(index).unwrap();
                        let size = allocation.end - allocation.start;
                        let old_start = allocation.start;
                        let new_start = previous_end;

                        // Actually perform the move.
                        self.allocations.get_mut(&id).unwrap().start = new_start;
                        self.allocations.get_mut(&id).unwrap().end = new_start + size;

                        return Some((id, size, old_start, new_start));
                    }

                    // Haven't found a gap yet. Move to the next.
                    previous_end = allocation.end;
                    index = self
                        .allocations
                        .keys()
                        .position(|&id| id == allocation.next)
                        .unwrap();
                }
                break;
            }
        }

        None
    }

    /// Attempts to resize a given allocation to a new size. If the new size is smaller, the start index remains unchanged.
    pub fn resize(&mut self, id: u64, size: usize) -> Option<(usize, usize)> {
        let allocation = self.allocations.get_mut(&id)?;

        let old_start = allocation.start;
        let current_size = allocation.end - allocation.start;

        if size < current_size {
            // We can resize without worrying about redoing an allocation.
            // Note that we always shrink the interval by moving the end closer to the start, even though that might
            // increase fragmentation. However, by only moving the endpoint, we eliminate the need to move the interval.
            // Externally, this means resource uploads are avoided.
            // Conceptually, the incremental compaction algorithm already induces a bias toward 0. In other words,
            // temporarily introducing fragmentation doesn't matter because the incremental compaction algorithm ends up
            // doing the same amount of work either way. So we might as well avoid doing double-moves.
            allocation.end = allocation.start + size;
            return Some((old_start, allocation.start));
        }

        // The size is increasing. This requires a reallocation.
        let success = self.deallocate(id);
        debug_assert!(success, "Sanity check: you just looked this allocation up, yet the deallocation failed. Did you introduce a race condition?");

        if let Some(new_start) = self.allocate(id, size) {
            Some((old_start, new_start))
        } else {
            // Failed to find a location that fits the requested size. Allocate at the old size.
            let success = self.allocate(id, current_size);
            debug_assert!(success.is_some(), "You just deallocated a region of this size, so the allocation must succeed. Did you introduce a race condition?");
            None
        }
    }
}

impl Drop for Allocator {
    fn drop(&mut self) {
        for (_, allocation) in self.allocations.drain() {
            let buffer = unsafe {
                Buffer::new(
                    allocation.start as *mut u8,
                    allocation.end - allocation.start,
                    -1,
                )
            };
            self.pool.return_buffer(buffer);
        }
    }
}
