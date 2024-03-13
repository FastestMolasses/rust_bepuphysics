use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    marker::PhantomData,
    ptr::NonNull,
};

use crate::utilities::memory::{
    buffer::Buffer, buffer_pool::IUnmanagedMemoryPool, managed_id_pool::ManagedIdPool,
};

/// Contains basic helpers for hashing.
pub struct HashHelper;

impl HashHelper {
    /// Redistributes a hash. Useful for converting unique but contiguous hashes into a semirandom distribution.
    #[inline]
    pub fn rehash(hash: u64) -> u64 {
        const A: u32 = 6;
        const B: u32 = 13;
        const C: u32 = 25;

        let uhash = hash * 982451653u64;
        let redongled = ((uhash << A) | (uhash >> (64 - A)))
            ^ ((uhash << B) | (uhash >> (64 - B)))
            ^ ((uhash << C) | (uhash >> (64 - C)));

        redongled
    }
}

/// Container supporting constant time adds and removes of key-value pairs while preserving fast iteration times.
/// Offers very direct access to information at the cost of safety.
///
/// Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
/// it is a value type and copying it around will break things without extreme care,
/// it cannot be validly default-constructed,
/// it exposes internal structures to user modification,
/// it rarely checks input for errors,
/// the enumerator doesn't check for mid-enumeration modification,
/// it allows unsafe addition that can break if the user doesn't manage the capacity,
/// it works on top of an abstracted memory blob which might internally be a pointer that could be rugpulled,
/// it does not (and is incapable of) checking that provided memory gets returned to the same pool that it came from.
///
/// Note that the implementation is extremely simple. It uses single-step linear probing under the assumption of very low collision rates.
/// A generous table capacity is recommended; this trades some memory for simplicity and runtime performance.
pub struct QuickDictionary<TKey, TValue>
where
    TKey: Eq + Hash,
{
    /// Number of elements in the dictionary.
    count: usize,

    /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
    table_mask: usize,

    /// Desired size of the table relative to the size of the key/value spans in terms of a power of 2. Table capacity target will be elementCapacityTarget * 2^TablePowerOffset.
    table_power_offset: usize,

    /// Backing memory of the dictionary's table. Values are distributed according to the key's hash function.
    /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
    table: Buffer<usize>,

    /// Backing memory containing the keys of the dictionary.
    /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
    keys: Buffer<TKey>,

    /// Backing memory containing the values of the dictionary.
    /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
    values: Buffer<TValue>,
}

impl<TKey, TValue> QuickDictionary<TKey, TValue>
where
    TKey: Eq + Hash,
{
    /// Creates a new dictionary.
    pub fn new(
        initial_key_span: Buffer<TKey>,
        initial_value_span: Buffer<TValue>,
        initial_table_span: Buffer<usize>,
        table_power_offset: usize,
    ) -> Self {
        debug_assert!(
            initial_table_span.len() >= initial_key_span.len(),
            "The table span must be at least as large as the key span."
        );
        debug_assert!(
            initial_value_span.len() >= initial_key_span.len(),
            "The value span must be at least as large as the key span."
        );
        debug_assert!(
            (initial_table_span.len() & (initial_table_span.len() - 1)) == 0,
            "Dictionaries depend upon power of 2 backing table span sizes for efficient modulo operations."
        );

        for i in 0..initial_table_span.len() {
            debug_assert!(
                initial_table_span[i] == 0,
                "The table provided to the set must be cleared."
            );
        }

        QuickDictionary {
            count: 0,
            table_mask: initial_table_span.len() - 1,
            table_power_offset,
            table: initial_table_span,
            keys: initial_key_span,
            values: initial_value_span,
        }
    }

    /// Creates a new dictionary with a specified initial capacity.
    pub fn with_capacity(
        initial_capacity: usize,
        table_power_offset: usize,
        pool: &mut impl IUnmanagedMemoryPool,
    ) -> Self {
        let initial_key_span = pool.take(initial_capacity).unwrap();
        let initial_value_span = pool.take(initial_capacity).unwrap();
        let initial_table_span = pool.take(initial_capacity << table_power_offset).unwrap();

        // Clear the table to ensure it's clean.
        initial_table_span.clear(0, initial_table_span.len());

        Self::new(
            initial_key_span,
            initial_value_span,
            initial_table_span,
            table_power_offset,
        )
    }

    /// Gets the number of elements in the dictionary.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Checks if the dictionary is empty.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Gets the index of the element in the table.
    fn get_table_indices(&self, key: &TKey) -> Option<usize> {
        let mut table_index = HashHelper::rehash(self.hash(key)) & self.table_mask;

        loop {
            let element_index = self.table[table_index];

            if element_index == 0 {
                // Empty slot, key not found.
                return None;
            }

            // Check if the key at the element index matches.
            if self.keys[element_index - 1] == *key {
                return Some(element_index - 1);
            }

            // Collision, move to the next slot.
            table_index = (table_index + 1) & self.table_mask;
        }
    }

    /// Hashes the given key using the default hasher.
    fn hash(&self, key: &TKey) -> u64 {
        let mut hasher = DefaultHasher::new();
        key.hash(&mut hasher);
        hasher.finish()
    }

    /// Checks if a given key already belongs to the dictionary.
    pub fn contains_key(&self, key: &TKey) -> bool {
        self.get_table_indices(key).is_some()
    }

    /// Tries to retrieve the value associated with a key if it exists.
    pub fn get(&self, key: &TKey) -> Option<&TValue> {
        self.get_table_indices(key)
            .map(|element_index| &self.values[element_index])
    }

    /// Tries to retrieve the value associated with a key mutably if it exists.
    pub fn get_mut(&mut self, key: &TKey) -> Option<&mut TValue> {
        self.get_table_indices(key)
            .map(move |element_index| &mut self.values[element_index])
    }

    /// Adds a pair to the dictionary if it is not already present.
    pub fn insert(&mut self, key: TKey, value: TValue, pool: &mut impl IUnmanagedMemoryPool) {
        if self.count == self.keys.len() {
            self.resize(self.count * 2, pool);
        }

        if self.get_table_indices(&key).is_none() {
            // Key not found, insert it.
            self.keys[self.count] = key;
            self.values[self.count] = value;
            let table_index = HashHelper::rehash(self.hash(&key)) & self.table_mask;
            self.table[table_index] = self.count + 1; // Encoding: offset by 1.
            self.count += 1;
        }
    }

    /// Removes a pair associated with a key from the dictionary if it belongs to the dictionary.
    /// Does not preserve the order of elements in the dictionary.
    pub fn remove(&mut self, key: &TKey) -> Option<TValue> {
        if let Some(element_index) = self.get_table_indices(key) {
            // Key found, remove it.
            let value = self.values.remove(element_index);

            // Update table and shift elements if necessary.
            let mut gap_index = HashHelper::rehash(self.hash(key)) & self.table_mask;
            let mut table_index = (gap_index + 1) & self.table_mask;

            while self.table[table_index] != 0 {
                let move_candidate_index = self.table[table_index] - 1;
                let desired_index = HashHelper::rehash(self.hash(&self.keys[move_candidate_index]))
                    & self.table_mask;

                let distance_from_gap = (table_index - gap_index) & self.table_mask;
                let distance_from_ideal = (table_index - desired_index) & self.table_mask;

                if distance_from_gap <= distance_from_ideal {
                    // Move the element to the gap.
                    self.table[gap_index] = self.table[table_index];
                    gap_index = table_index;
                }

                table_index = (table_index + 1) & self.table_mask;
            }

            // Clear the gap in the table.
            self.table[gap_index] = 0;
            self.count -= 1;

            Some(value)
        } else {
            None
        }
    }

    /// Removes all elements from the dictionary.
    pub fn clear(&mut self) {
        self.table.clear(0, self.table.len());
        self.keys.clear(0, self.count);
        self.values.clear(0, self.count);
        self.count = 0;
    }

    /// Resizes the dictionary's backing arrays for the given size.
    /// If the new size is smaller, the dictionary's count is truncated and the extra elements are dropped.
    fn resize(&mut self, new_size: usize, pool: &mut impl IUnmanagedMemoryPool) {
        let new_key_span = pool.take(new_size).unwrap();
        let new_value_span = pool.take(new_size).unwrap();
        let new_table_span = pool.take(new_size << self.table_power_offset).unwrap();

        // Clear the new table.
        new_table_span.clear(0, new_table_span.len());

        let old_count = self.count;
        let new_count = std::cmp::min(old_count, new_size);

        // Re-insert elements into the new dictionary.
        for i in 0..new_count {
            self.insert(self.keys.remove(0), self.values.remove(0), pool);
        }

        // Return old buffers to the pool.
        pool.return_buffer(self.keys);
        pool.return_buffer(self.values);
        pool.return_buffer(self.table);

        // Update internal state with new buffers.
        self.keys = new_key_span;
        self.values = new_value_span;
        self.table = new_table_span;
        self.count = new_count;
        self.table_mask = new_table_span.len() - 1;
    }
}
