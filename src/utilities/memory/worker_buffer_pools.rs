//! Collection of pools used by worker threads.

use super::buffer_pool::BufferPool;

/// Collection of pools used by worker threads.
///
/// Each worker thread gets its own pool to avoid contention during parallel operations.
pub struct WorkerBufferPools {
    pools: Vec<BufferPool>,
    default_block_capacity: i32,
}

impl WorkerBufferPools {
    /// Creates a new set of worker pools.
    ///
    /// # Arguments
    /// * `initial_worker_count` - Initial number of workers to allocate space for.
    /// * `default_block_capacity` - Default block capacity in thread pools.
    pub fn new(initial_worker_count: usize, default_block_capacity: i32) -> Self {
        let pools = (0..initial_worker_count)
            .map(|_| BufferPool::new_with_minimum_block_size(default_block_capacity))
            .collect();
        Self {
            pools,
            default_block_capacity,
        }
    }

    /// Creates a new set of worker pools with default block capacity.
    pub fn new_default(initial_worker_count: usize) -> Self {
        Self::new(initial_worker_count, 16384)
    }

    /// Gets the pool associated with this worker.
    ///
    /// # Arguments
    /// * `worker_index` - Worker index of the pool to look up.
    ///
    /// # Returns
    /// Mutable reference to the pool associated with the given worker.
    #[inline(always)]
    pub fn get_pool(&mut self, worker_index: usize) -> &mut BufferPool {
        &mut self.pools[worker_index]
    }

    /// Gets an immutable reference to the pool associated with this worker.
    #[inline(always)]
    pub fn get_pool_ref(&self, worker_index: usize) -> &BufferPool {
        &self.pools[worker_index]
    }

    /// Gets the number of worker pools.
    #[inline(always)]
    pub fn pool_count(&self) -> usize {
        self.pools.len()
    }

    /// Gets or sets the default block capacity for any newly created arena subpools.
    #[inline(always)]
    pub fn default_block_capacity(&self) -> i32 {
        self.default_block_capacity
    }

    /// Sets the default block capacity for any newly created arena subpools.
    #[inline(always)]
    pub fn set_default_block_capacity(&mut self, capacity: i32) {
        self.default_block_capacity = capacity;
    }

    /// Ensures that there are at least `worker_count` pools available.
    pub fn ensure_worker_count(&mut self, worker_count: usize) {
        while self.pools.len() < worker_count {
            self.pools.push(BufferPool::new_with_minimum_block_size(self.default_block_capacity));
        }
    }

    /// Clears all allocations from worker pools. Pools can still be used after being cleared.
    ///
    /// # Safety
    /// This does not take any locks and should not be called if any other threads
    /// may be using any of the involved pools.
    pub fn clear(&mut self) {
        for pool in &mut self.pools {
            pool.clear();
        }
    }

    /// Gets the total number of bytes allocated from native memory by all worker pools.
    /// Includes memory that is not currently in use by external allocators.
    pub fn get_total_allocated_byte_count(&self) -> u64 {
        self.pools
            .iter()
            .map(|p| p.get_total_allocated_byte_count())
            .sum()
    }
}

impl Drop for WorkerBufferPools {
    /// Disposes all worker pools. Pools cannot be used after being disposed.
    fn drop(&mut self) {
        self.clear();
    }
}

