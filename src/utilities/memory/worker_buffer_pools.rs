//! Collection of pools used by worker threads.

use std::cell::UnsafeCell;

use super::buffer_pool::BufferPool;

/// Collection of pools used by worker threads.
///
/// Each worker thread gets its own pool to avoid contention during parallel operations.
/// Uses `UnsafeCell` internally to allow obtaining `*mut BufferPool` from shared references,
/// matching C#'s implicit mutability semantics. The safety invariant is that each pool
/// is only ever accessed by its owning worker thread.
pub struct WorkerBufferPools {
    pools: Vec<UnsafeCell<BufferPool>>,
    default_block_capacity: i32,
}

// Safety: Each pool is only accessed by its owning worker thread.
// UnsafeCell prevents Send/Sync from being auto-derived, so we add it back.
unsafe impl Send for WorkerBufferPools {}
unsafe impl Sync for WorkerBufferPools {}

impl WorkerBufferPools {
    /// Creates a new set of worker pools.
    ///
    /// # Arguments
    /// * `initial_worker_count` - Initial number of workers to allocate space for.
    /// * `default_block_capacity` - Default block capacity in thread pools.
    pub fn new(initial_worker_count: usize, default_block_capacity: i32) -> Self {
        let pools = (0..initial_worker_count)
            .map(|_| {
                UnsafeCell::new(BufferPool::new_with_minimum_block_size(
                    default_block_capacity,
                ))
            })
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

    /// Gets a mutable reference to the pool associated with this worker.
    ///
    /// # Safety
    /// Caller must ensure exclusive access to this pool (i.e., only the owning worker calls this).
    #[inline(always)]
    pub unsafe fn get_pool(&self, worker_index: usize) -> &mut BufferPool {
        &mut *self.pools[worker_index].get()
    }

    /// Gets a raw mutable pointer to the pool associated with this worker.
    ///
    /// This uses `UnsafeCell::get()` which is the sanctioned way to obtain a `*mut T`
    /// from a `&self` context. The caller must ensure exclusive access.
    #[inline(always)]
    pub fn get_pool_ptr(&self, worker_index: usize) -> *mut BufferPool {
        self.pools[worker_index].get()
    }

    /// Gets an immutable reference to the pool associated with this worker.
    ///
    /// # Safety
    /// Caller must ensure no mutable access is happening concurrently.
    #[inline(always)]
    pub unsafe fn get_pool_ref(&self, worker_index: usize) -> &BufferPool {
        &*self.pools[worker_index].get()
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
            self.pools
                .push(UnsafeCell::new(BufferPool::new_with_minimum_block_size(
                    self.default_block_capacity,
                )));
        }
    }

    /// Clears all allocations from worker pools. Pools can still be used after being cleared.
    ///
    /// # Safety
    /// This does not take any locks and should not be called if any other threads
    /// may be using any of the involved pools.
    pub fn clear(&mut self) {
        for pool in &mut self.pools {
            pool.get_mut().clear();
        }
    }

    /// Gets the total number of bytes allocated from native memory by all worker pools.
    /// Includes memory that is not currently in use by external allocators.
    pub fn get_total_allocated_byte_count(&self) -> u64 {
        self.pools
            .iter()
            .map(|p| unsafe { (*p.get()).get_total_allocated_byte_count() })
            .sum()
    }
}
