use crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer;
use crate::utilities::collections::quick_dictionary::HashHelper;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;

/// Container supporting set behaviors built on top of unmanaged buffers.
/// Uses hash-based indexing with linear probing for collision resolution.
///
/// Be very careful when using this type. It has sacrificed a lot upon the altar of performance.
#[repr(C)]
pub struct QuickSet<T: Copy, TEqualityComparer> {
    /// Number of elements in the set.
    pub count: i32,

    /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
    pub table_mask: i32,

    /// Desired size of the table relative to the size of the span in terms of a power of 2.
    pub table_power_offset: i32,

    /// Backing memory of the set's table. Values are distributed according to the EqualityComparer's hash function.
    /// Slots containing 0 are unused. Slots containing higher values are equal to one plus the index of an element in the Span.
    pub table: Buffer<i32>,

    /// Backing memory containing the elements of the set.
    pub span: Buffer<T>,

    /// Equality comparer used to compare and hash elements.
    pub equality_comparer: TEqualityComparer,
}

impl<T: Copy, TEqualityComparer: RefEqualityComparer<T>> QuickSet<T, TEqualityComparer> {
    /// Creates a new set.
    #[inline(always)]
    pub fn new(
        span: Buffer<T>,
        table: Buffer<i32>,
        equality_comparer: TEqualityComparer,
        table_power_offset: i32,
    ) -> Self {
        debug_assert!(
            table.len() >= span.len(),
            "The table span must be at least as large as the element span."
        );
        debug_assert!(
            (table.len() & (table.len() - 1)) == 0,
            "QuickSets depend upon power of 2 backing table span sizes for efficient modulo operations."
        );

        let table_mask = table.len() - 1;

        QuickSet {
            count: 0,
            table_mask,
            table_power_offset,
            table,
            span,
            equality_comparer,
        }
    }

    /// Creates a new set with a specified initial capacity.
    #[inline(always)]
    pub fn with_capacity(
        initial_capacity: i32,
        table_power_offset: i32,
        pool: &mut impl UnmanagedMemoryPool,
        equality_comparer: TEqualityComparer,
    ) -> Self {
        let span = pool.take_at_least::<T>(initial_capacity);
        let table_capacity = (span.len() << table_power_offset).max(1 << table_power_offset);
        let mut table = pool.take_at_least::<i32>(table_capacity);
        // Clear the table since pool memory may contain garbage
        table.clear(0, table.len());

        Self::new(span, table, equality_comparer, table_power_offset)
    }

    /// Gets the index of the element in the table.
    /// Returns true if the element is present, false otherwise.
    /// On return, `table_index` contains the table slot, and `element_index` contains the
    /// element's index in the span (-1 if not found).
    #[inline(always)]
    pub fn get_table_indices(
        &self,
        element: &T,
        table_index: &mut i32,
        element_index: &mut i32,
    ) -> bool {
        // The table lengths are guaranteed to be a power of 2, so the modulo is a simple binary operation.
        *table_index = HashHelper::rehash(self.equality_comparer.hash(element)) & self.table_mask;

        // 0 in the table means 'not taken'; all other values are offset by 1 upward.
        loop {
            *element_index = self.table[*table_index];
            if *element_index <= 0 {
                *element_index = -1;
                return false;
            }
            // This table index is taken. Is this the specified element?
            // Remember to decode the object index.
            *element_index -= 1;
            if self.equality_comparer.equals(&self.span[*element_index], element) {
                return true;
            }
            *table_index = (*table_index + 1) & self.table_mask;
        }
    }

    /// Gets the index of the element in the set if it exists.
    #[inline(always)]
    pub fn index_of(&self, element: &T) -> i32 {
        let mut table_index = 0;
        let mut element_index = 0;
        self.get_table_indices(element, &mut table_index, &mut element_index);
        element_index
    }

    /// Checks if a given element already belongs to the set.
    #[inline(always)]
    pub fn contains(&self, element: &T) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        self.get_table_indices(element, &mut table_index, &mut element_index)
    }

    /// Adds an element to the set if it is not already present.
    /// Does not resize in the event that capacity is exceeded.
    /// Returns true if the element was added, false if it was already present.
    #[inline(always)]
    pub fn add_unsafely(&mut self, element: &T) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(element, &mut table_index, &mut element_index) {
            // Already present!
            return false;
        }

        // It wasn't in the set. Add it!
        debug_assert!(
            self.count < self.span.len(),
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.span[self.count] = *element;
        // Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
        self.count += 1;
        self.table[table_index] = self.count;
        true
    }

    /// Adds an element to the set. If a version of the element is already present, it is replaced.
    /// Does not resize in the event that capacity is exceeded.
    /// Returns true if the element was added, false if it was replaced.
    #[inline(always)]
    pub fn add_and_replace_unsafely(&mut self, element: &T) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(element, &mut table_index, &mut element_index) {
            // Already present! Replace it.
            self.span[element_index] = *element;
            return false;
        }

        // It wasn't in the set. Add it!
        debug_assert!(
            self.count < self.span.len(),
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.span[self.count] = *element;
        self.count += 1;
        self.table[table_index] = self.count;
        true
    }

    /// Resizes the internal buffers.
    pub fn resize(&mut self, new_size: i32, pool: &mut impl UnmanagedMemoryPool) {
        let mut new_span = pool.take_at_least::<T>(new_size);
        let table_capacity = (new_span.len() << self.table_power_offset).max(1 << self.table_power_offset);
        let mut new_table = pool.take_at_least::<i32>(table_capacity);

        // Clear the new table
        new_table.clear(0, new_table.len());

        let mut old_span = self.span;
        let mut old_table = self.table;

        // Copy elements to new span
        old_span.copy_to(0, &mut new_span, 0, self.count);

        self.span = new_span;
        self.table = new_table;
        self.table_mask = new_table.len() - 1;

        // Rehash all elements into new table
        for i in 0..self.count {
            let element = &self.span[i];
            let mut table_index =
                HashHelper::rehash(self.equality_comparer.hash(element)) & self.table_mask;

            // Find empty slot
            while self.table[table_index] != 0 {
                table_index = (table_index + 1) & self.table_mask;
            }
            self.table[table_index] = i + 1; // +1 encoding
        }

        pool.return_buffer(&mut old_span);
        pool.return_buffer(&mut old_table);
    }

    /// Adds an element to the set if it is not already present.
    /// Resizes if necessary.
    /// Returns true if the element was added, false if it was already present.
    #[inline(always)]
    pub fn add(&mut self, element: &T, pool: &mut impl UnmanagedMemoryPool) -> bool {
        if self.count == self.span.len() {
            // There's no room left; resize.
            self.resize(self.count * 2, pool);
        }
        self.add_unsafely(element)
    }

    /// Adds an element to the set. If a version is already present, it is replaced.
    /// Resizes if necessary.
    /// Returns true if the element was added, false if it was replaced.
    #[inline(always)]
    pub fn add_and_replace(&mut self, element: &T, pool: &mut impl UnmanagedMemoryPool) -> bool {
        if self.count == self.span.len() {
            self.resize(self.count * 2, pool);
        }
        self.add_and_replace_unsafely(element)
    }

    /// Removes an element by its table and element indices.
    /// Can only be used if the indices are known to be valid.
    pub fn fast_remove_at(&mut self, table_index: i32, element_index: i32) {
        // Maintain the invariant: all items are either at their desired index
        // or in a contiguous block clockwise from the desired index.
        // Search clockwise for an item to fill the gap.
        let mut gap_index = table_index;
        let mut search_index = table_index;

        loop {
            search_index = (search_index + 1) & self.table_mask;
            let move_candidate_index = self.table[search_index];

            if move_candidate_index <= 0 {
                break;
            }

            let candidate_element_index = move_candidate_index - 1;
            let desired_index = HashHelper::rehash(
                self.equality_comparer.hash(&self.span[candidate_element_index]),
            ) & self.table_mask;

            // Would this element be closer to its actual index if moved to the gap?
            let distance_from_gap = (search_index - gap_index) & self.table_mask;
            let distance_from_ideal = (search_index - desired_index) & self.table_mask;

            if distance_from_gap <= distance_from_ideal {
                // Move to the gap
                self.table[gap_index] = self.table[search_index];
                gap_index = search_index;
            }
        }

        // Clear the gap in the table
        self.table[gap_index] = 0;

        // Swap the final element into the removed element's slot if needed
        self.count -= 1;
        if element_index < self.count {
            self.span[element_index] = self.span[self.count];

            // Update the table for the swapped element
            let mut swap_table_index = 0;
            let mut old_element_index = 0;
            self.get_table_indices(
                &self.span[element_index],
                &mut swap_table_index,
                &mut old_element_index,
            );
            self.table[swap_table_index] = element_index + 1; // +1 encoding
        }

        // Clear the final slot (matches C# `Span[Count] = default`)
        unsafe {
            std::ptr::write_bytes(self.span.as_mut_ptr().add(self.count as usize), 0, 1);
        }
    }

    /// Removes an element from the set if it exists.
    /// Does not preserve order.
    /// Returns true if the element was found and removed.
    #[inline(always)]
    pub fn fast_remove(&mut self, element: &T) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(element, &mut table_index, &mut element_index) {
            self.fast_remove_at(table_index, element_index);
            true
        } else {
            false
        }
    }

    /// Clears all elements from the set.
    pub fn clear(&mut self) {
        self.table.clear(0, self.table.len());
        self.span.clear(0, self.count);
        self.count = 0;
    }

    /// Clears the set without zeroing the element span.
    pub fn fast_clear(&mut self) {
        self.table.clear(0, self.table.len());
        self.count = 0;
    }

    /// Returns the resources associated with the set to the pool.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        pool.return_buffer(&mut self.span);
        pool.return_buffer(&mut self.table);
    }

    /// Ensures that the set has enough room to hold the specified number of elements.
    #[inline(always)]
    pub fn ensure_capacity(&mut self, count: i32, pool: &mut impl UnmanagedMemoryPool) {
        if count > self.span.len() {
            self.resize(count, pool);
        }
    }

    /// Shrinks the internal buffers to the smallest acceptable size.
    pub fn compact(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        let target_capacity = BufferPool::get_capacity_for_count::<T>(self.count);
        if target_capacity < self.span.len() {
            self.resize(self.count, pool);
        }
    }
    /// Adds an element by value. If already present, replaces it. Does not resize.
    #[inline(always)]
    pub fn add_and_replace_unsafely_val(&mut self, element: T) -> bool {
        let mut e = element;
        self.add_and_replace_unsafely(&mut e)
    }

    /// Adds an element by value if not already present. Does not resize.
    #[inline(always)]
    pub fn add_unsafely_val(&mut self, element: T) -> bool {
        let mut e = element;
        self.add_unsafely(&mut e)
    }

    /// Adds an element by value. If already present, replaces it.
    #[inline(always)]
    pub fn add_and_replace_val(&mut self, element: T, pool: &mut impl UnmanagedMemoryPool) -> bool {
        let mut e = element;
        self.add_and_replace(&mut e, pool)
    }

    /// Adds an element by value if not already present.
    #[inline(always)]
    pub fn add_val(&mut self, element: T, pool: &mut impl UnmanagedMemoryPool) -> bool {
        let mut e = element;
        self.add(&mut e, pool)
    }

    /// Removes an element by value if it belongs to the set.
    #[inline(always)]
    pub fn fast_remove_val(&mut self, element: T) -> bool {
        let mut e = element;
        self.fast_remove(&mut e)
    }
    /// Returns an iterator over the set elements.
    pub fn iter(&self) -> QuickSetIterator<'_, T> {
        QuickSetIterator {
            span: &self.span,
            count: self.count,
            index: 0,
        }
    }
}

impl<T: Copy, TEqualityComparer> std::ops::Index<i32> for QuickSet<T, TEqualityComparer> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: i32) -> &Self::Output {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds.");
        &self.span[index]
    }
}

/// Iterator over QuickSet elements.
pub struct QuickSetIterator<'a, T: Copy> {
    span: &'a Buffer<T>,
    count: i32,
    index: i32,
}

impl<'a, T: Copy> Iterator for QuickSetIterator<'a, T> {
    type Item = T;

    #[inline(always)]
    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.count {
            let value = self.span[self.index];
            self.index += 1;
            Some(value)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = (self.count - self.index) as usize;
        (remaining, Some(remaining))
    }
}

impl<'a, T: Copy> ExactSizeIterator for QuickSetIterator<'a, T> {}
