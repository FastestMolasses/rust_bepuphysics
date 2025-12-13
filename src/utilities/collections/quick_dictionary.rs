use crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;

/// Contains basic helpers for hashing.
pub struct HashHelper;

impl HashHelper {
    /// Redistributes a hash. Useful for converting unique but contiguous hashes into a semirandom distribution.
    #[inline(always)]
    pub fn rehash(hash: i32) -> i32 {
        const A: u32 = 6;
        const B: u32 = 13;
        const C: u32 = 25;

        let uhash = (hash as u32).wrapping_mul(982451653u32);
        let redongled = ((uhash << A) | (uhash >> (32 - A)))
            ^ ((uhash << B) | (uhash >> (32 - B)))
            ^ ((uhash << C) | (uhash >> (32 - C)));

        redongled as i32
    }
}

/// Container supporting constant time adds and removes of key-value pairs while preserving fast iteration times.
/// Offers very direct access to information at the cost of safety.
///
/// Be very careful when using this type. It has sacrificed a lot upon the altar of performance.
#[repr(C)]
pub struct QuickDictionary<TKey: Copy, TValue: Copy, TEqualityComparer> {
    /// Number of elements in the dictionary.
    pub count: i32,

    /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
    pub table_mask: i32,

    /// Desired size of the table relative to the size of the key/value spans in terms of a power of 2.
    pub table_power_offset: i32,

    /// Backing memory of the dictionary's table. Values are distributed according to the EqualityComparer's hash function.
    /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
    pub table: Buffer<i32>,

    /// Backing memory containing the keys of the dictionary.
    pub keys: Buffer<TKey>,

    /// Backing memory containing the values of the dictionary.
    pub values: Buffer<TValue>,

    /// Equality comparer used to compare and hash keys.
    pub equality_comparer: TEqualityComparer,
}

impl<TKey: Copy, TValue: Copy, TEqualityComparer: RefEqualityComparer<TKey>>
    QuickDictionary<TKey, TValue, TEqualityComparer>
{
    /// Creates a new dictionary.
    #[inline(always)]
    pub fn new(
        keys: Buffer<TKey>,
        values: Buffer<TValue>,
        table: Buffer<i32>,
        equality_comparer: TEqualityComparer,
        table_power_offset: i32,
    ) -> Self {
        debug_assert!(
            table.len() >= keys.len(),
            "The table span must be at least as large as the key span."
        );
        debug_assert!(
            values.len() >= keys.len(),
            "The value span must be at least as large as the key span."
        );
        debug_assert!(
            (table.len() & (table.len() - 1)) == 0,
            "Dictionaries depend upon power of 2 backing table span sizes for efficient modulo operations."
        );

        let table_mask = table.len() - 1;

        QuickDictionary {
            count: 0,
            table_mask,
            table_power_offset,
            table,
            keys,
            values,
            equality_comparer,
        }
    }

    /// Creates a new dictionary with a specified initial capacity.
    #[inline(always)]
    pub fn with_capacity(
        initial_capacity: i32,
        table_power_offset: i32,
        pool: &mut impl UnmanagedMemoryPool,
        equality_comparer: TEqualityComparer,
    ) -> Self {
        let keys: Buffer<TKey> = pool.take_at_least(initial_capacity);
        let values: Buffer<TValue> = pool.take_at_least(keys.len());
        let mut table: Buffer<i32> = pool.take_at_least(keys.len() << table_power_offset);

        // Clear the table to ensure it's clean.
        table.clear(0, table.len());

        Self::new(keys, values, table, equality_comparer, table_power_offset)
    }

    /// Gets the number of elements in the dictionary.
    #[inline(always)]
    pub fn len(&self) -> i32 {
        self.count
    }

    /// Checks if the dictionary is empty.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Gets the table indices for a given key, returns (table_index, element_index).
    #[inline(always)]
    pub fn get_table_indices(&self, key: &TKey, table_index: &mut i32, element_index: &mut i32) -> bool {
        let hash_code = self.equality_comparer.hash(key);
        *table_index = HashHelper::rehash(hash_code) & self.table_mask;

        loop {
            *element_index = self.table[*table_index] - 1;

            if *element_index < 0 {
                // Empty slot, key not found.
                return false;
            }

            // Check if the key at the element index matches.
            if self.equality_comparer.equals(&self.keys[*element_index], key) {
                return true;
            }

            // Collision, move to the next slot.
            *table_index = (*table_index + 1) & self.table_mask;
        }
    }

    /// Checks if a given key already belongs to the dictionary.
    #[inline(always)]
    pub fn contains_key(&self, key: &TKey) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        self.get_table_indices(key, &mut table_index, &mut element_index)
    }

    /// Gets the index of a key if present in the dictionary.
    #[inline(always)]
    pub fn index_of(&self, key: &TKey) -> Option<i32> {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(key, &mut table_index, &mut element_index) {
            Some(element_index)
        } else {
            None
        }
    }

    /// Tries to retrieve the value associated with a key if it exists.
    #[inline(always)]
    pub fn get(&self, key: &TKey) -> Option<&TValue> {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(key, &mut table_index, &mut element_index) {
            Some(&self.values[element_index])
        } else {
            None
        }
    }

    /// Tries to retrieve the value associated with a key mutably if it exists.
    #[inline(always)]
    pub fn get_mut(&mut self, key: &TKey) -> Option<&mut TValue> {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(key, &mut table_index, &mut element_index) {
            Some(&mut self.values[element_index])
        } else {
            None
        }
    }

    /// Adds a pair to the dictionary without checking capacity.
    #[inline(always)]
    pub fn add_unsafely(&mut self, key: TKey, value: TValue) {
        debug_assert!(
            self.count < self.keys.len(),
            "Adding would exceed capacity"
        );

        let hash_code = self.equality_comparer.hash(&key);
        let mut table_index = HashHelper::rehash(hash_code) & self.table_mask;

        // Find an empty slot
        while self.table[table_index] != 0 {
            table_index = (table_index + 1) & self.table_mask;
        }

        self.keys[self.count] = key;
        self.values[self.count] = value;
        self.table[table_index] = self.count + 1;
        self.count += 1;
    }

    /// Adds a pair to the dictionary if it is not already present.
    #[inline(always)]
    pub fn add(&mut self, key: TKey, value: TValue, pool: &mut impl UnmanagedMemoryPool) -> bool {
        if self.contains_key(&key) {
            return false;
        }
        self.ensure_capacity(self.count + 1, pool);
        self.add_unsafely(key, value);
        true
    }

    /// Tries to add a pair to the dictionary without checking capacity.
    #[inline(always)]
    pub fn try_add_unsafely(&mut self, key: TKey, value: TValue) -> bool {
        if self.contains_key(&key) {
            return false;
        }
        self.add_unsafely(key, value);
        true
    }

    /// Adds or updates a pair in the dictionary.
    #[inline(always)]
    pub fn add_or_update(&mut self, key: TKey, value: TValue, pool: &mut impl UnmanagedMemoryPool) {
        let mut table_index = 0;
        let mut element_index = 0;
        if self.get_table_indices(&key, &mut table_index, &mut element_index) {
            self.values[element_index] = value;
        } else {
            self.ensure_capacity(self.count + 1, pool);
            self.add_unsafely(key, value);
        }
    }

    /// Removes a key from the dictionary.
    /// Does not preserve the order of elements in the dictionary.
    #[inline(always)]
    pub fn fast_remove(&mut self, key: &TKey) -> bool {
        let mut table_index = 0;
        let mut element_index = 0;
        if !self.get_table_indices(key, &mut table_index, &mut element_index) {
            return false;
        }

        // Clear the table entry
        self.table[table_index] = 0;
        self.count -= 1;

        // If this wasn't the last element, move the last element to this position
        if element_index < self.count {
            let last_key = self.keys[self.count];
            let last_value = self.values[self.count];

            // Update the table entry for the moved element
            let last_hash = self.equality_comparer.hash(&last_key);
            let mut last_table_index = HashHelper::rehash(last_hash) & self.table_mask;
            while self.table[last_table_index] != self.count + 1 {
                last_table_index = (last_table_index + 1) & self.table_mask;
            }
            self.table[last_table_index] = element_index + 1;

            // Move the element
            self.keys[element_index] = last_key;
            self.values[element_index] = last_value;
        }

        true
    }

    /// Removes a key from the dictionary using pre-computed table and element indices.
    /// Does not preserve the order of elements in the dictionary.
    #[inline(always)]
    pub fn fast_remove_at(&mut self, table_index: i32, element_index: i32) {
        // Clear the table entry
        self.table[table_index] = 0;
        self.count -= 1;

        // If this wasn't the last element, move the last element to this position
        if element_index < self.count {
            let last_key = self.keys[self.count];
            let last_value = self.values[self.count];

            // Update the table entry for the moved element
            let last_hash = self.equality_comparer.hash(&last_key);
            let mut last_table_index = HashHelper::rehash(last_hash) & self.table_mask;
            while self.table[last_table_index] != self.count + 1 {
                last_table_index = (last_table_index + 1) & self.table_mask;
            }
            self.table[last_table_index] = element_index + 1;

            // Move the element
            self.keys[element_index] = last_key;
            self.values[element_index] = last_value;
        }
    }

    /// Finds the existing slot for a key, or allocates a new one if it doesn't exist.
    /// Returns true if the key already existed, false if a new slot was allocated.
    /// In either case, slot_index is set to the element index.
    #[inline(always)]
    pub fn find_or_allocate_slot_unsafely(&mut self, key: &TKey, slot_index: &mut i32) -> bool {
        let hash_code = self.equality_comparer.hash(key);
        let mut table_index = HashHelper::rehash(hash_code) & self.table_mask;

        loop {
            let element_index = self.table[table_index] - 1;

            if element_index < 0 {
                // Empty slot — allocate new entry.
                *slot_index = self.count;
                self.keys[self.count] = *key;
                self.table[table_index] = self.count + 1;
                self.count += 1;
                return false;
            }

            if self.equality_comparer.equals(&self.keys[element_index], key) {
                *slot_index = element_index;
                return true;
            }

            table_index = (table_index + 1) & self.table_mask;
        }
    }

    /// Removes all elements from the dictionary.
    #[inline(always)]
    pub fn clear(&mut self) {
        self.table.clear(0, self.table.len());
        self.count = 0;
    }

    /// Ensures that the dictionary has enough room to hold the specified number of elements.
    #[inline(always)]
    pub fn ensure_capacity(&mut self, count: i32, pool: &mut impl UnmanagedMemoryPool) {
        if count > self.keys.len() {
            self.resize(count, pool);
        }
    }

    /// Resizes the dictionary's backing arrays for the given size.
    #[inline(always)]
    pub fn resize(&mut self, new_size: i32, pool: &mut impl UnmanagedMemoryPool) {
        let target_key_capacity = BufferPool::get_capacity_for_count::<TKey>(new_size);
        if target_key_capacity != self.keys.len() {
            let new_keys: Buffer<TKey> = pool.take_at_least(new_size);
            let new_values: Buffer<TValue> = pool.take_at_least(new_keys.len());
            let mut new_table: Buffer<i32> =
                pool.take_at_least(new_keys.len() << self.table_power_offset);

            // Clear the new table
            new_table.clear(0, new_table.len());

            // Save old data
            let old_count = self.count.min(new_keys.len());
            let mut old_keys = self.keys;
            let mut old_values = self.values;
            let mut old_table = self.table;

            // Reset state
            self.keys = new_keys;
            self.values = new_values;
            self.table = new_table;
            self.table_mask = self.table.len() - 1;
            self.count = 0;

            // Re-insert elements
            for i in 0..old_count {
                self.add_unsafely(old_keys[i], old_values[i]);
            }

            // Return old buffers
            pool.return_buffer(&mut old_keys);
            pool.return_buffer(&mut old_values);
            pool.return_buffer(&mut old_table);
        }
    }

    /// Compacts the internal buffers to the smallest acceptable size.
    #[inline(always)]
    pub fn compact(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        let target_length = BufferPool::get_capacity_for_count::<TKey>(self.count);
        if target_length < self.keys.len() {
            self.resize(self.count, pool);
        }
    }

    /// Returns the resources associated with the dictionary to pools.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        pool.return_buffer(&mut self.keys);
        pool.return_buffer(&mut self.values);
        pool.return_buffer(&mut self.table);
    }

    /// Gets a reference to the key at the given index.
    #[inline(always)]
    pub fn key_at(&self, index: i32) -> &TKey {
        debug_assert!(index >= 0 && index < self.count);
        &self.keys[index]
    }

    /// Gets a reference to the value at the given index.
    #[inline(always)]
    pub fn value_at(&self, index: i32) -> &TValue {
        debug_assert!(index >= 0 && index < self.count);
        &self.values[index]
    }

    /// Gets a mutable reference to the value at the given index.
    #[inline(always)]
    pub fn value_at_mut(&mut self, index: i32) -> &mut TValue {
        debug_assert!(index >= 0 && index < self.count);
        &mut self.values[index]
    }
}

impl<TKey: Copy, TValue: Copy, TEqualityComparer: Default> Default
    for QuickDictionary<TKey, TValue, TEqualityComparer>
{
    fn default() -> Self {
        Self {
            count: 0,
            table_mask: 0,
            table_power_offset: 0,
            table: Buffer::default(),
            keys: Buffer::default(),
            values: Buffer::default(),
            equality_comparer: TEqualityComparer::default(),
        }
    }
}
