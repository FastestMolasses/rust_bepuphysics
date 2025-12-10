//! Represents a chunk of abstract memory supporting allocations and deallocations.
//!
//! Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations.
//! Not particularly efficient, but simple and correct.

use std::collections::HashMap;

/// Represents an allocation within the allocator.
#[derive(Clone, Copy, Debug)]
pub struct Allocation {
    pub start: i64,
    pub end: i64,
    pub previous: u64,
    pub next: u64,
}

/// Represents a chunk of abstract memory supporting allocations and deallocations.
/// Never moves any memory.
///
/// Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations.
pub struct Allocator {
    capacity: i64,
    /// Index in allocations that we should start at during the next allocation attempt.
    search_start_index: usize,
    /// Maps allocation IDs to their allocation data.
    allocations: HashMap<u64, Allocation>,
    /// Maintains iteration order for allocations (keys in insertion order).
    allocation_keys: Vec<u64>,
}

impl Allocator {
    /// Creates a new allocator.
    ///
    /// # Arguments
    /// * `capacity` - Size of the memory handled by the allocator in elements.
    /// * `initial_allocation_capacity` - Estimated number of allocations to allocate room for.
    pub fn new(capacity: i64, initial_allocation_capacity: usize) -> Self {
        debug_assert!(capacity >= 0, "Capacity must be non-negative.");
        Self {
            capacity,
            search_start_index: 0,
            allocations: HashMap::with_capacity(initial_allocation_capacity),
            allocation_keys: Vec::with_capacity(initial_allocation_capacity),
        }
    }

    /// Gets the capacity of the allocator.
    pub fn capacity(&self) -> i64 {
        self.capacity
    }

    /// Sets the capacity of the allocator.
    ///
    /// # Panics
    /// Panics if the new capacity is less than any existing allocation endpoint.
    pub fn set_capacity(&mut self, value: i64) {
        debug_assert!(value >= 0, "Capacity must be non-negative.");
        if value < self.capacity {
            for allocation in self.allocations.values() {
                assert!(
                    value >= allocation.end,
                    "Can't reduce capacity below any existing allocation endpoint."
                );
            }
        }
        self.capacity = value;
    }

    /// Checks if the id is currently allocated.
    #[inline(always)]
    pub fn contains(&self, id: u64) -> bool {
        self.allocations.contains_key(&id)
    }

    /// Gets the allocation region associated with the given allocation id if it is present.
    pub fn try_get_allocation_region(&self, allocation_id: u64) -> Option<Allocation> {
        self.allocations.get(&allocation_id).copied()
    }

    /// Gets the number of allocations.
    pub fn allocation_count(&self) -> usize {
        self.allocations.len()
    }

    /// Checks if a block of memory can fit into the current state of the allocator.
    pub fn can_fit(&self, size: i64) -> bool {
        if self.allocations.is_empty() {
            return size <= self.capacity;
        }

        let mut allocation_index = self.search_start_index;
        let initial_id = self.allocation_keys[allocation_index];

        loop {
            let allocation = self.allocations.get(&self.allocation_keys[allocation_index]).unwrap();
            let next_allocation_id = allocation.next;
            let next_allocation = self.allocations.get(&next_allocation_id).unwrap();

            if next_allocation.start < allocation.end {
                // Wrapped around
                if self.capacity - allocation.end >= size {
                    return true;
                }
                if next_allocation.start >= size {
                    return true;
                }
            } else {
                // In order
                if next_allocation.start - allocation.end >= size {
                    return true;
                }
            }

            // Move to next
            allocation_index = self.allocation_keys.iter().position(|&k| k == next_allocation_id).unwrap();

            if self.allocation_keys[allocation_index] == initial_id {
                return false;
            }
        }
    }

    /// Attempts to allocate a range of memory.
    ///
    /// # Arguments
    /// * `id` - Unique id of the memory to allocate.
    /// * `size` - Size of the memory to allocate.
    ///
    /// # Returns
    /// Starting index of the allocated memory if successful, None otherwise.
    pub fn allocate(&mut self, id: u64, size: i64) -> Option<i64> {
        debug_assert!(!self.allocations.contains_key(&id), "Id must not already be present.");

        if self.allocations.is_empty() {
            if size <= self.capacity {
                self.allocations.insert(id, Allocation {
                    start: 0,
                    end: size,
                    next: id,
                    previous: id,
                });
                self.allocation_keys.push(id);
                self.search_start_index = 0;
                return Some(0);
            }
            return None;
        }

        debug_assert!(
            self.search_start_index < self.allocation_keys.len(),
            "Search start index must be within the allocation set!"
        );

        let initial_id = self.allocation_keys[self.search_start_index];
        let mut current_id = initial_id;

        loop {
            let allocation = *self.allocations.get(&current_id).unwrap();
            let next_allocation = *self.allocations.get(&allocation.next).unwrap();

            if next_allocation.start < allocation.end {
                // Wrapped around
                if self.capacity - allocation.end >= size {
                    let output_start = allocation.end;
                    self.add_allocation(id, output_start, output_start + size, current_id, allocation.next);
                    return Some(output_start);
                }
                if next_allocation.start >= size {
                    self.add_allocation(id, 0, size, current_id, allocation.next);
                    return Some(0);
                }
            } else {
                // In order
                if next_allocation.start - allocation.end >= size {
                    let output_start = allocation.end;
                    self.add_allocation(id, output_start, output_start + size, current_id, allocation.next);
                    return Some(output_start);
                }
            }

            current_id = allocation.next;
            if current_id == initial_id {
                return None;
            }
        }
    }

    fn add_allocation(&mut self, id: u64, start: i64, end: i64, previous_id: u64, next_id: u64) {
        // Update the previous allocation to point to the new one
        self.allocations.get_mut(&previous_id).unwrap().next = id;
        // Update the next allocation to point back to the new one
        self.allocations.get_mut(&next_id).unwrap().previous = id;

        self.search_start_index = self.allocation_keys.len();
        self.allocation_keys.push(id);
        self.allocations.insert(id, Allocation {
            start,
            end,
            next: next_id,
            previous: previous_id,
        });
    }

    /// Removes the memory associated with the id from the pool.
    pub fn deallocate(&mut self, id: u64) -> bool {
        if let Some(allocation) = self.allocations.remove(&id) {
            // Remove from keys list
            if let Some(pos) = self.allocation_keys.iter().position(|&k| k == id) {
                self.allocation_keys.swap_remove(pos);
            }

            if allocation.previous != id {
                // Update pointers
                if let Some(prev) = self.allocations.get_mut(&allocation.previous) {
                    prev.next = allocation.next;
                }
                if let Some(next) = self.allocations.get_mut(&allocation.next) {
                    next.previous = allocation.previous;
                }
            }

            // Update search start index
            self.search_start_index = self.allocation_keys.iter()
                .position(|&k| k == allocation.previous)
                .unwrap_or(0);

            true
        } else {
            false
        }
    }

    /// Gets the size of the largest contiguous area and the total free space in the allocator.
    pub fn get_largest_contiguous_size(&self) -> (i64, i64) {
        if self.allocations.is_empty() {
            return (self.capacity, self.capacity);
        }

        let mut largest_contiguous: i64 = 0;
        let mut total_free_space: i64 = 0;

        for allocation in self.allocations.values() {
            let next_allocation = self.allocations.get(&allocation.next).unwrap();
            let to_next = next_allocation.start - allocation.end;

            if to_next < 0 {
                // Wrapped
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

    /// Finds the first allocation with empty space before it and pulls it forward to close the gap.
    ///
    /// # Returns
    /// If a compaction was performed: (id, size, old_start, new_start)
    pub fn incremental_compact(&mut self) -> Option<(u64, i64, i64, i64)> {
        // Find the allocation nearest to the zero index
        for &key in &self.allocation_keys {
            let allocation = *self.allocations.get(&key).unwrap();
            let previous_allocation = *self.allocations.get(&allocation.previous).unwrap();

            if previous_allocation.end > allocation.start {
                // Found the beginning of the list
                let mut current_id = key;
                let mut previous_end: i64 = 0;

                for i in 0..self.allocations.len() {
                    self.search_start_index = self.allocation_keys.iter()
                        .position(|&k| k == current_id)
                        .unwrap_or(0);

                    let allocation = *self.allocations.get(&current_id).unwrap();

                    if allocation.start > previous_end {
                        // Found a gap
                        let size = allocation.end - allocation.start;
                        let old_start = allocation.start;
                        let new_start = previous_end;

                        // Perform the move
                        let alloc = self.allocations.get_mut(&current_id).unwrap();
                        alloc.start = new_start;
                        alloc.end = new_start + size;

                        return Some((current_id, size, old_start, new_start));
                    }

                    previous_end = allocation.end;
                    current_id = allocation.next;

                    if i + 1 < self.allocations.len() && current_id == key {
                        break;
                    }
                }
                break;
            }
        }

        None
    }

    /// Attempts to resize a given allocation to a new size.
    ///
    /// # Returns
    /// If successful: (old_start, new_start)
    pub fn resize(&mut self, id: u64, size: i64) -> Option<(i64, i64)> {
        let allocation = *self.allocations.get(&id)?;
        let old_start = allocation.start;
        let current_size = allocation.end - allocation.start;

        debug_assert!(size != current_size, "Why are you calling resize if the new size is the same?");

        if size < current_size {
            // Shrinking - just move the endpoint
            self.allocations.get_mut(&id).unwrap().end = allocation.start + size;
            return Some((old_start, allocation.start));
        }

        // Growing - requires reallocation
        let success = self.deallocate(id);
        debug_assert!(success, "Deallocation of existing allocation should succeed.");

        if let Some(new_start) = self.allocate(id, size) {
            Some((old_start, new_start))
        } else {
            // Failed - restore at old size
            let restore_result = self.allocate(id, current_size);
            debug_assert!(restore_result.is_some(), "Restore should succeed.");
            None
        }
    }

    /// Returns all allocations in the allocator to a fresh state.
    pub fn clear(&mut self) {
        self.allocations.clear();
        self.allocation_keys.clear();
        self.search_start_index = 0;
    }
}

