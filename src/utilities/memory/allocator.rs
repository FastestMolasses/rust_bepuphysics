//! Represents a chunk of abstract memory supporting allocations and deallocations.
//!
//! Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations.
//! Not particularly efficient, but simple and correct.

use crate::utilities::collections::predicate::Predicate;
use crate::utilities::collections::primitive_comparer::PrimitiveComparer;
use crate::utilities::collections::quick_dictionary::QuickDictionary;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;

/// Represents an allocation within the allocator.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Allocation {
    pub start: i64,
    pub end: i64,
    pub previous: u64,
    pub next: u64,
}

/// Matches nothing; used by the predicate-free `can_fit`.
#[derive(Clone, Copy, Default)]
struct IgnoreNothing;

impl Predicate<u64> for IgnoreNothing {
    #[inline(always)]
    fn matches(&self, _item: &u64) -> bool {
        false
    }
}

/// Represents a chunk of abstract memory supporting allocations and deallocations.
/// Never moves any memory.
///
/// Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations.
pub struct Allocator {
    capacity: i64,
    /// Index in allocations that we should start at during the next allocation attempt.
    search_start_index: i32,
    allocations: QuickDictionary<u64, Allocation, PrimitiveComparer<u64>>,
}

impl Allocator {
    /// Creates a new allocator.
    ///
    /// # Arguments
    /// * `capacity` - Size of the memory handled by the allocator in elements.
    /// * `pool` - Pool to pull internal resources from.
    /// * `initial_allocation_capacity` - Estimated number of allocations to allocate room for.
    pub fn new(
        capacity: i64,
        pool: &mut impl UnmanagedMemoryPool,
        initial_allocation_capacity: i32,
    ) -> Self {
        assert!(capacity >= 0, "Capacity must be positive.");
        Self {
            capacity,
            search_start_index: 0,
            allocations: QuickDictionary::with_capacity(
                initial_allocation_capacity,
                2,
                pool,
                PrimitiveComparer::new(),
            ),
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
        assert!(value >= 0, "Capacity must be positive.");
        if value < self.capacity {
            for i in 0..self.allocations.count {
                assert!(
                    value >= self.allocations.value_at(i).end,
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
    pub fn allocation_count(&self) -> i32 {
        self.allocations.count
    }

    /// Checks if a block of memory can fit into the current state of the allocator,
    /// treating allocations matched by `ignored_ids` as nonexistent.
    pub fn can_fit_ignoring<TPredicate: Predicate<u64>>(
        &self,
        size: i64,
        ignored_ids: &TPredicate,
    ) -> bool {
        if self.allocations.count == 0 {
            return size <= self.capacity;
        }
        let mut allocation_index = self.search_start_index;
        let initial_id = *self.allocations.key_at(allocation_index);
        loop {
            let allocation = *self.allocations.value_at(allocation_index);
            let mut next_allocation_index;
            let mut next_allocation;

            // Skip any subsequent allocations that are ignored.
            let mut next_allocation_id = allocation.next;
            loop {
                next_allocation_index = self.allocations.index_of(&next_allocation_id).unwrap();
                next_allocation = *self.allocations.value_at(next_allocation_index);
                next_allocation_id = next_allocation.next;
                if !ignored_ids.matches(&next_allocation_id) {
                    break;
                }
            }

            if next_allocation.start < allocation.end {
                // Wrapped around, so the gap goes from here to the end of the memory block, and from
                // the beginning of the memory block to the next allocation. Contiguous space is
                // required, so the two areas have to be tested independently.
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
            // No open space was found; move on to the next spot.
            allocation_index = next_allocation_index;

            if *self.allocations.key_at(allocation_index) == initial_id {
                // Wrapped around without finding any space.
                return false;
            }
        }
    }

    /// Checks if a block of memory can fit into the current state of the allocator.
    pub fn can_fit(&self, size: i64) -> bool {
        self.can_fit_ignoring(size, &IgnoreNothing)
    }

    #[inline(always)]
    fn add_allocation(
        &mut self,
        id: u64,
        start: i64,
        end: i64,
        allocation_index: i32,
        next_allocation_index: i32,
        pool: &mut impl UnmanagedMemoryPool,
    ) {
        let new_allocation = Allocation {
            next: self.allocations.value_at(allocation_index).next,
            previous: self.allocations.value_at(next_allocation_index).previous,
            start,
            end,
        };
        // The pointer modifications come BEFORE the new addition; this avoids a potential
        // invalidation caused by a resize in the allocations dictionary.
        self.allocations.value_at_mut(allocation_index).next = id;
        self.allocations.value_at_mut(next_allocation_index).previous = id;
        // There was space here this time, so there is a high chance of more space next time.
        self.search_start_index = self.allocations.count;
        self.allocations.add(id, new_allocation, pool);
    }

    /// Attempts to allocate a range of memory.
    ///
    /// # Arguments
    /// * `id` - Unique id of the memory to allocate.
    /// * `size` - Size of the memory to allocate.
    ///
    /// # Returns
    /// Starting index of the allocated memory if successful, None otherwise.
    pub fn allocate(
        &mut self,
        id: u64,
        size: i64,
        pool: &mut impl UnmanagedMemoryPool,
    ) -> Option<i64> {
        debug_assert!(
            !self.allocations.contains_key(&id),
            "Id must not already be present."
        );
        if self.allocations.count == 0 {
            // If it's the first allocation, then the next and previous pointers circle around.
            if size <= self.capacity {
                self.allocations.add(
                    id,
                    Allocation {
                        start: 0,
                        end: size,
                        next: id,
                        previous: id,
                    },
                    pool,
                );
                self.search_start_index = 0;
                return Some(0);
            }
            return None;
        }
        debug_assert!(
            self.search_start_index >= 0 && self.search_start_index < self.allocations.count,
            "Search start index must be within the allocation set!"
        );
        let mut allocation_index = self.search_start_index;
        let initial_id = *self.allocations.key_at(allocation_index);
        loop {
            let allocation = *self.allocations.value_at(allocation_index);
            let next_allocation_index = self.allocations.index_of(&allocation.next).unwrap();
            let next_allocation = *self.allocations.value_at(next_allocation_index);
            if next_allocation.start < allocation.end {
                // Wrapped around, so the gap goes from here to the end of the memory block, and from
                // the beginning of the memory block to the next allocation. Contiguous space is
                // required, so the two areas have to be tested independently.
                if self.capacity - allocation.end >= size {
                    let output_start = allocation.end;
                    self.add_allocation(
                        id,
                        output_start,
                        allocation.end + size,
                        allocation_index,
                        next_allocation_index,
                        pool,
                    );
                    return Some(output_start);
                } else if next_allocation.start >= size {
                    self.add_allocation(
                        id,
                        0,
                        size,
                        allocation_index,
                        next_allocation_index,
                        pool,
                    );
                    return Some(0);
                }
            } else {
                // The next allocation is in order.
                if next_allocation.start - allocation.end >= size {
                    let output_start = allocation.end;
                    self.add_allocation(
                        id,
                        output_start,
                        allocation.end + size,
                        allocation_index,
                        next_allocation_index,
                        pool,
                    );
                    return Some(output_start);
                }
            }
            // No open space was found; move on to the next spot.
            allocation_index = next_allocation_index;

            if *self.allocations.key_at(allocation_index) == initial_id {
                // Wrapped around without finding any space.
                return None;
            }
        }
    }

    /// Removes the memory associated with the id from the pool.
    pub fn deallocate(&mut self, id: u64) -> bool {
        let mut allocation = Allocation::default();
        if self.allocations.try_get_value(&id, &mut allocation) {
            if allocation.previous != id {
                let previous_index = self.allocations.index_of(&allocation.previous).unwrap();
                debug_assert!(
                    self.allocations.value_at(previous_index).next == id,
                    "Previous and current must agree about their relationship."
                );
                // Make the previous allocation point to the next allocation.
                self.allocations.value_at_mut(previous_index).next = allocation.next;

                let next_index = self.allocations.index_of(&allocation.next).unwrap();
                debug_assert!(
                    self.allocations.value_at(next_index).previous == id,
                    "Next and current must agree about their relationship."
                );
                // Make the next allocation point to the previous allocation.
                self.allocations.value_at_mut(next_index).previous = allocation.previous;
            } else {
                debug_assert!(
                    allocation.next == id,
                    "The next index should be itself too, if previous was itself."
                );
                debug_assert!(
                    self.allocations.count == 1,
                    "The only time where the previous allocation is itself should be when there is only a single allocation."
                );
            }
            self.allocations.fast_remove(&id);
            // Removing this id makes the position next to the previous allocation a promising place
            // to look next time. The index is requested again because it may have moved during the
            // removal; an invalid index is fine, since an empty set does not use it.
            self.search_start_index = self
                .allocations
                .index_of(&allocation.previous)
                .unwrap_or(-1);
            debug_assert!(
                self.allocations.count == 0
                    || (self.search_start_index >= 0
                        && self.search_start_index < self.allocations.count),
                "Search start index must be within the allocation set!"
            );
            true
        } else {
            false
        }
    }

    /// Gets the size of the largest contiguous area and the total free space in the allocator.
    /// Not very efficient; runs in linear time for the number of allocations.
    pub fn get_largest_contiguous_size(&self) -> (i64, i64) {
        if self.allocations.count == 0 {
            return (self.capacity, self.capacity);
        }
        let mut largest_contiguous: i64 = 0;
        let mut total_free_space: i64 = 0;
        for i in 0..self.allocations.count {
            let allocation = *self.allocations.value_at(i);
            let mut next_allocation = Allocation::default();
            self.allocations
                .try_get_value(&allocation.next, &mut next_allocation);
            let to_next = next_allocation.start - allocation.end;
            if to_next < 0 {
                // The next allocation requires a wrap, so the actual contiguous area is only from
                // this end to the end of the pool, plus a second region from 0 to the next.
                let adjacent = self.capacity - allocation.end;
                let wrapped = next_allocation.start;
                if largest_contiguous < adjacent {
                    largest_contiguous = adjacent;
                }
                if largest_contiguous < wrapped {
                    largest_contiguous = wrapped;
                }
                total_free_space += adjacent + wrapped;
            } else {
                if largest_contiguous < to_next {
                    largest_contiguous = to_next;
                }
                total_free_space += to_next;
            }
        }
        (largest_contiguous, total_free_space)
    }

    /// Finds the first allocation with empty space before it and pulls it forward to close the gap.
    /// Assumes the ability to perform synchronous reallocation.
    ///
    /// # Returns
    /// If a compaction was performed: (id, size, old_start, new_start)
    pub fn incremental_compact(&mut self) -> Option<(u64, i64, i64, i64)> {
        // Find the allocation nearest to the zero index by checking for the previous allocation
        // requiring a wraparound. Start at the beginning of the list since it's marginally more
        // likely to be there than at the end where new allocations get appended.
        for i in 0..self.allocations.count {
            let allocation = *self.allocations.value_at(i);
            let mut previous_allocation = Allocation::default();
            self.allocations
                .try_get_value(&allocation.previous, &mut previous_allocation);
            if previous_allocation.end > allocation.start {
                // Found the beginning of the list. Scan forward through the allocation links
                // looking for the first gap, stopping before wrapping.
                let mut index = i;
                let mut previous_end: i64 = 0;
                for _ in 0..self.allocations.count {
                    // If the traversal ends, this index is cached so that the next allocation
                    // starts at the end of the contiguous block.
                    self.search_start_index = index;
                    debug_assert!(
                        index >= 0 && index < self.allocations.count,
                        "Search start index must be within the allocation set!"
                    );
                    let current = *self.allocations.value_at(index);
                    if current.start > previous_end {
                        let id = *self.allocations.key_at(index);
                        let size = current.end - current.start;
                        let old_start = current.start;
                        let new_start = previous_end;
                        let slot = self.allocations.value_at_mut(index);
                        slot.start = new_start;
                        slot.end = new_start + size;
                        return Some((id, size, old_start, new_start));
                    }
                    previous_end = current.end;
                    index = self.allocations.index_of(&current.next).unwrap();
                }
                break;
            }
        }
        None
    }

    /// Attempts to resize a given allocation to a new size. If the new size is smaller, the start
    /// index remains unchanged.
    ///
    /// # Returns
    /// `Ok((old_start, new_start))` if the resize succeeded. `Err((old_start, new_start))` if there
    /// was no room for the larger size; the allocation is restored at its original size, but not
    /// necessarily at its original address, so `new_start` still matters.
    pub fn resize(
        &mut self,
        id: u64,
        size: i64,
        pool: &mut impl UnmanagedMemoryPool,
    ) -> Result<(i64, i64), (i64, i64)> {
        let allocation_index = self
            .allocations
            .index_of(&id)
            .expect("The allocation must be present inside the allocator to resize it!");
        let allocation = *self.allocations.value_at(allocation_index);
        let old_start = allocation.start;
        let current_size = allocation.end - allocation.start;
        debug_assert!(
            size != current_size,
            "Why are you calling resize if the new size is the same as the old one?"
        );

        if size < current_size {
            // The interval is always shrunk by moving the end closer to the start, even though that
            // might increase fragmentation; only moving the endpoint avoids moving the interval,
            // and the incremental compaction algorithm does the same amount of work either way.
            self.allocations.value_at_mut(allocation_index).end = allocation.start + size;
            return Ok((old_start, allocation.start));
        }
        // The size is increasing, which requires a reallocation.
        let deallocated = self.deallocate(id);
        debug_assert!(
            deallocated,
            "Sanity check: you just looked this allocation up, yet the deallocation failed."
        );
        match self.allocate(id, size, pool) {
            Some(new_start) => Ok((old_start, new_start)),
            None => {
                // Failed to find a location that fits the requested size. Allocate at the old size.
                let restored = self.allocate(id, current_size, pool);
                debug_assert!(
                    restored.is_some(),
                    "You just deallocated a region of this size, so the allocation must succeed."
                );
                Err((old_start, restored.unwrap_or(0)))
            }
        }
    }

    /// Returns all allocations in the allocator to a fresh state.
    pub fn clear(&mut self) {
        self.allocations.clear();
        self.search_start_index = 0;
    }

    #[cfg(debug_assertions)]
    #[allow(dead_code)]
    fn validate_pointers(&self) {
        if self.allocations.count == 0 {
            return;
        }
        let initial_id = *self.allocations.key_at(0);
        let mut backward_id = initial_id;
        let mut forward_id = initial_id;
        for _ in 0..self.allocations.count {
            let backward_index = self.allocations.index_of(&backward_id).unwrap();
            backward_id = self.allocations.value_at(backward_index).previous;

            let forward_index = self.allocations.index_of(&forward_id).unwrap();
            forward_id = self.allocations.value_at(forward_index).next;
        }
        debug_assert!(
            initial_id == backward_id && initial_id == forward_id,
            "We should be able to walk back to the starting id in exactly allocations.Count steps in either direction."
        );
    }

    /// Returns the resources associated with the allocator to pools.
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        self.allocations.dispose(pool);
    }
}
