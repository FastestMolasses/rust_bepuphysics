use crate::utilities::memory::buffer_pool::BufferPool;

/// Collection of pools used by worker threads.
pub struct WorkerBufferPools {
    pools: Vec<BufferPool>,
    default_block_capacity: usize,
}

impl WorkerBufferPools {
    /// Creates a new set of worker pools.
    ///
    /// # Arguments
    ///
    /// * `initial_worker_count` - Initial number of workers to allocate space for.
    /// * `default_block_capacity` - Default block capacity in thread pools.
    pub fn new(initial_worker_count: usize, default_block_capacity: usize) -> Self {
        let pools = (0..initial_worker_count)
            .map(|_| BufferPool::new_with_minimum_block_size(default_block_capacity))
            .collect();
        Self {
            pools,
            default_block_capacity,
        }
    }

    /// Gets the pool associated with this worker.
    ///
    /// # Arguments
    ///
    /// * `worker_index` - Worker index of the pool to look up.
    ///
    /// # Returns
    ///
    /// * Pool associated with the given worker.
    ///
    /// # Safety
    ///
    /// Calling this method is safe as long as the caller ensures no other threads
    /// are accessing this pool concurrently, which aligns with the non-thread-safe requirement.
    pub fn get_pool(&mut self, worker_index: usize) -> &mut BufferPool {
        &mut self.pools[worker_index]
    }

    /// Gets or sets the default block capacity for any newly created arena subpools.
    pub fn default_block_capacity(&self) -> usize {
        self.default_block_capacity
    }

    pub fn set_default_block_capacity(&mut self, capacity: usize) {
        self.default_block_capacity = capacity;
    }

    /// Clears all allocations from worker pools. Pools can still be used after being cleared.
    ///
    /// # Remarks
    ///
    /// This does not take any locks and should not be called if any other threads may be using any of the involved pools.
    pub fn clear(&mut self) {
        for pool in &mut self.pools {
            unsafe {
                pool.clear();
            }
        }
    }

    /// Gets the total number of bytes allocated from native memory by all worker pools. Includes memory that is not currently in use by external allocators.
    ///
    /// # Returns
    ///
    /// Total number of bytes allocated from native memory by all worker pools. Includes memory that is not currently in use by external allocators.
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
