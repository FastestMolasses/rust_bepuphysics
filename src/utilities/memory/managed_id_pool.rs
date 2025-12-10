//! Manages a pool of identifier values.
//!
//! Grabbing an id from the pool picks a number that has been picked and returned before,
//! or if none of those are available, the minimum value greater than any existing id.

/// Manages a pool of identifier values.
///
/// This is a simple stack-based ID pool that provides O(1) allocation and deallocation.
/// IDs are recycled when returned to the pool.
pub struct ManagedIdPool {
    /// The next ID to allocate if no recycled IDs are available.
    next_index: i32,
    /// Stack of available (recycled) IDs.
    available_ids: Vec<i32>,
}

impl ManagedIdPool {
    /// Creates a new ManagedIdPool with the specified initial capacity.
    #[inline(always)]
    pub fn new(initial_capacity: i32) -> Self {
        debug_assert!(initial_capacity > 0);
        ManagedIdPool {
            next_index: 0,
            available_ids: Vec::with_capacity(initial_capacity as usize),
        }
    }

    /// Takes an ID from the pool.
    ///
    /// Returns a recycled ID if available, otherwise allocates a new one.
    #[inline(always)]
    pub fn take(&mut self) -> i32 {
        match self.available_ids.pop() {
            Some(id) => id,
            None => {
                let id = self.next_index;
                self.next_index += 1;
                id
            }
        }
    }

    /// Returns an ID to the pool for recycling.
    #[inline(always)]
    pub fn return_id(&mut self, id: i32) {
        self.available_ids.push(id);
    }

    /// Returns an ID to the pool without checking if a resize is required.
    ///
    /// # Safety
    /// The caller must ensure that the available_ids vector has sufficient capacity.
    #[inline(always)]
    pub fn return_unsafely(&mut self, id: i32) {
        debug_assert!(self.available_ids.len() < self.available_ids.capacity());
        self.available_ids.push(id);
    }

    /// Resets the IdPool, clearing all allocated and recycled IDs.
    #[inline(always)]
    pub fn clear(&mut self) {
        self.next_index = 0;
        self.available_ids.clear();
    }

    /// Ensures that the underlying id queue can hold at least a certain number of ids.
    #[inline(always)]
    pub fn ensure_capacity(&mut self, count: i32) {
        if (self.available_ids.capacity() as i32) < count {
            self.available_ids.reserve((count as usize) - self.available_ids.capacity());
        }
    }

    /// Shrinks the available ids queue to the smallest size that can fit the given count
    /// and the current available id count.
    #[inline(always)]
    pub fn compact(&mut self, minimum_count: i32) {
        let target_length = minimum_count.max(self.available_ids.len() as i32) as usize;
        if self.available_ids.capacity() > target_length {
            self.available_ids.shrink_to(target_length);
        }
    }

    /// Gets the highest value which any index claimed thus far could possibly have.
    /// This is not necessarily the current highest claimed index; this value may represent
    /// an earlier claim that has already been released.
    /// Returns -1 if nothing has ever been claimed.
    #[inline(always)]
    pub fn highest_possibly_claimed_id(&self) -> i32 {
        self.next_index - 1
    }

    /// Gets the number of previously returned ids waiting in the pool.
    #[inline(always)]
    pub fn available_id_count(&self) -> i32 {
        self.available_ids.len() as i32
    }

    /// Gets the capacity of the id pool for returned ids.
    #[inline(always)]
    pub fn capacity(&self) -> i32 {
        self.available_ids.capacity() as i32
    }
}
