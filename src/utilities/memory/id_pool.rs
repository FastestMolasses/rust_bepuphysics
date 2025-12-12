// Translated from BepuUtilities/Memory/IdPool.cs

use super::buffer::Buffer;
use super::buffer_pool::BufferPool;

/// Manages a pool of identifier values. Grabbing an id from the pool picks a number that has been
/// picked and returned before, or if none of those are available, the minimum value greater
/// than any existing id.
pub struct IdPool {
    next_index: i32,
    available_id_count: i32,
    available_ids: Buffer<i32>,
}

impl IdPool {
    /// Creates a new IdPool with the given initial capacity.
    pub fn new(initial_capacity: i32, pool: &mut BufferPool) -> Self {
        let available_ids = pool.take_at_least(initial_capacity);
        IdPool {
            next_index: 0,
            available_id_count: 0,
            available_ids,
        }
    }

    /// Gets the highest value which any index claimed thus far could possibly have.
    /// -1 if nothing has ever been claimed.
    #[inline(always)]
    pub fn highest_possibly_claimed_id(&self) -> i32 {
        self.next_index - 1
    }

    /// Gets the number of previously returned ids waiting in the pool.
    #[inline(always)]
    pub fn available_id_count(&self) -> i32 {
        self.available_id_count
    }

    /// Gets the capacity of the available ids buffer.
    #[inline(always)]
    pub fn capacity(&self) -> i32 {
        self.available_ids.len()
    }

    /// Gets whether the id pool has backing resources allocated.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.available_ids.allocated()
    }

    /// Takes an id from the pool.
    #[inline(always)]
    pub fn take(&mut self) -> i32 {
        debug_assert!(self.available_ids.allocated());
        if self.available_id_count > 0 {
            self.available_id_count -= 1;
            *self.available_ids.get(self.available_id_count)
        } else {
            let id = self.next_index;
            self.next_index += 1;
            id
        }
    }

    /// Returns an id to the pool, resizing the internal buffer if necessary.
    #[inline(always)]
    pub fn return_id(&mut self, id: i32, pool: &mut BufferPool) {
        debug_assert!(self.available_ids.allocated());
        if self.available_id_count == self.available_ids.len() {
            let mut old_available_ids = self.available_ids;
            self.available_ids = pool.take_at_least(
                (self.available_id_count * 2).max(self.available_ids.len()),
            );
            old_available_ids.copy_to(0, &mut self.available_ids, 0, self.available_id_count);
            pool.return_buffer(&mut old_available_ids);
        }
        self.return_unsafely(id);
    }

    /// Returns an id to the pool without checking if a resize is required.
    #[inline(always)]
    pub fn return_unsafely(&mut self, id: i32) {
        debug_assert!(
            self.available_ids.allocated() && self.available_ids.len() > self.available_id_count
        );
        *self.available_ids.get_mut(self.available_id_count) = id;
        self.available_id_count += 1;
    }

    /// Resets the IdPool.
    pub fn clear(&mut self) {
        self.next_index = 0;
        self.available_id_count = 0;
    }

    fn internal_resize(&mut self, new_size: i32, pool: &mut BufferPool) {
        let mut old_available_ids = self.available_ids;
        self.available_ids = pool.take_at_least(new_size);
        debug_assert!(
            old_available_ids.len() != self.available_ids.len(),
            "Did you really mean to resize this? Nothing changed!"
        );
        old_available_ids.copy_to(0, &mut self.available_ids, 0, self.available_id_count);
        pool.return_buffer(&mut old_available_ids);
    }

    /// Ensures that the underlying id queue can hold at least a certain number of ids.
    pub fn ensure_capacity(&mut self, count: i32, pool: &mut BufferPool) {
        if !self.available_ids.allocated() {
            *self = IdPool::new(count, pool);
        } else if self.available_ids.len() < count {
            self.internal_resize(count, pool);
        }
    }

    /// Shrinks the available ids queue to the smallest size that can fit the given count.
    pub fn compact(&mut self, minimum_count: i32, pool: &mut BufferPool) {
        debug_assert!(self.available_ids.allocated());
        let target_length =
            BufferPool::get_capacity_for_count::<i32>(minimum_count.max(self.available_id_count));
        if self.available_ids.len() > target_length {
            self.internal_resize(target_length, pool);
        }
    }

    /// Resizes the underlying buffer to the smallest size required to hold the given count.
    pub fn resize(&mut self, count: i32, pool: &mut BufferPool) {
        if !self.available_ids.allocated() {
            *self = IdPool::new(count, pool);
        } else {
            let target_length =
                BufferPool::get_capacity_for_count::<i32>(count.max(self.available_id_count));
            if self.available_ids.len() != target_length {
                self.internal_resize(target_length, pool);
            }
        }
    }

    /// Returns underlying memory to the pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.available_ids);
        *self = IdPool {
            next_index: 0,
            available_id_count: 0,
            available_ids: Buffer::default(),
        };
    }
}
