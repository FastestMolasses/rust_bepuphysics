use std::collections::BTreeMap;
use std::mem::size_of;

use crate::utilities::memory::{buffer::Buffer, buffer_pool::BufferPool};

/// Represents a chunk of abstract memory supporting allocations and deallocations.
/// Never moves any memory.
pub struct Allocator {
    pool: BufferPool,
    capacity: usize,
    search_start_index: usize,
    allocations: BTreeMap<u64, Allocation>,
}

struct Allocation {
    start: usize,
    end: usize,
}

impl Allocator {
    /// Creates a new allocator.
    pub fn new(capacity: usize, pool: BufferPool) -> Self {
        Allocator {
            pool,
            capacity,
            search_start_index: 0,
            allocations: BTreeMap::new(),
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
            let allocation = self.allocations.get_index(allocation_index).unwrap().1;
            let next_allocation_index = (allocation_index + 1) % self.allocations.len();
            let next_allocation = self.allocations.get_index(next_allocation_index).unwrap().1;

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
            allocation_index = next_allocation_index;

            // Have we already wrapped around?
            if *self.allocations.keys().nth(allocation_index).unwrap() == initial_id {
                // Wrapped around without finding any space.
                return false;
            }
        }
    }

    /// Attempts to allocate a range of memory.
    pub fn allocate(&mut self, id: u64, size: usize) -> Option<usize> {
        assert!(
            !self.allocations.contains_key(&id),
            "Id must not already be present."
        );

        if self.allocations.is_empty() {
            // If it's the first allocation, then the next and previous pointers should circle around.
            if size <= self.capacity {
                self.allocations.insert(
                    id,
                    Allocation {
                        start: 0,
                        end: size,
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
            let allocation = self.allocations.get_index(allocation_index).unwrap().1;
            let next_allocation_index = (allocation_index + 1) % self.allocations.len();
            let next_allocation = self.allocations.get_index(next_allocation_index).unwrap().1;

            if next_allocation.start < allocation.end {
                // Wrapped around
                if self.capacity - allocation.end >= size {
                    self.allocations.insert(
                        id,
                        Allocation {
                            start: allocation.end,
                            end: allocation.end + size,
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
                        },
                    );
                    self.search_start_index = self.allocations.len() - 1;
                    return Some(allocation.end);
                }
            }

            // If we get here, no open space was found.
            // Move on to the next spot.
            allocation_index = next_allocation_index;

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
            // Update search_start_index
            self.search_start_index = self
                .allocations
                .keys()
                .position(|&key| key == allocation.start)
                .unwrap_or(0);
            true
        } else {
            false
        }
    }

    /// Gets the size of the largest contiguous area and the total free space in the allocator.
    pub fn get_largest_contiguous_size(&self) -> (usize, usize) {
        if self.allocations.is_empty() {
            return (self.capacity, self.capacity);
        }

        let mut largest_contiguous = 0;
        let mut total_free_space = 0;

        for i in 0..self.allocations.len() {
            let allocation = self.allocations.get_index(i).unwrap().1;
            let next_allocation = self
                .allocations
                .get_index((i + 1) % self.allocations.len())
                .unwrap()
                .1;

            let to_next = next_allocation.start - allocation.end;
            if to_next < 0 {
                // The next allocation requires a wrap
                let adjacent = self.capacity - allocation.end;
                let wrapped = next_allocation.start;
                largest_contiguous = std::cmp::max(largest_contiguous, adjacent);
                largest_contiguous = std::cmp::max(largest_contiguous, wrapped);
                total_free_space += adjacent + wrapped;
            } else {
                largest_contiguous = std::cmp::max(largest_contiguous, to_next);
                total_free_space += to_next;
            }
        }

        (largest_contiguous, total_free_space)
    }

    /// Finds the first allocation with empty space before it and pulls it forward to close the gap.
    pub fn incremental_compact(&mut self) -> Option<(u64, usize, usize, usize)> {
        if self.allocations.is_empty() {
            return None;
        }

        // Find the allocation nearest to the zero index.
        let mut index = self
            .allocations
            .keys()
            .position(|&key| key == 0)
            .unwrap_or(0);

        let mut previous_end = 0;
        for _ in 0..self.allocations.len() {
            let (id, allocation) = self.allocations.get_index(index).unwrap();
            if allocation.start > previous_end {
                // Found a gap.
                let old_start = allocation.start;
                let new_start = previous_end;
                let size = allocation.end - allocation.start;

                // Update the allocation in the map.
                self.allocations.insert(
                    *id,
                    Allocation {
                        start: new_start,
                        end: new_start + size,
                    },
                );

                // Update search_start_index
                self.search_start_index = index;

                return Some((*id, size, old_start, new_start));
            }

            previous_end = allocation.end;
            index = (index + 1) % self.allocations.len();
        }

        None
    }
}

impl Drop for Allocator {
    fn drop(&mut self) {
        self.clear();
    }
}

impl Allocator {
    fn clear(&mut self) {
        for (_, allocation) in self.allocations.iter() {
            let size = allocation.end - allocation.start;
            let buffer = Buffer::new(
                allocation.start as *mut u8,
                size,
                self.pool.get_id_for_allocation(allocation.start),
            );
            self.pool.return_unsafely(buffer);
        }
        self.allocations.clear();
    }
}
