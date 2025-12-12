//! Thread dispatcher for multithreaded physics simulation.

use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::worker_buffer_pools::WorkerBufferPools;

/// Function to be invoked on a worker thread. Provides the worker index and a reference to the dispatcher.
pub type WorkerBodyFn = fn(worker_index: i32, dispatcher: &dyn IThreadDispatcher);

/// Provides multithreading dispatch primitives, a thread count, and per thread resource pools for the simulation to use.
///
/// Note that the simulation does not require a true load balancing forloop implementation. All that's needed is a way to jumpstart some threads.
/// All systems which use multithreading tend to have some form of domain specific load balancing that a general purpose thread pool or parallel for loop implementation 
/// couldn't match. The simulation also tends to keep the number of dispatches as low as it can. Combined, these two things reduce the importance of a
/// very highly optimized dispatcher.
///
/// This is important when a user wants to share some other thread pool, but doesn't want to guarantee extremely high performance and high quality
/// load balancing. Instead of worrying about that, they can just wrap whatever implementation they happen to have and it'll probably work fine.
pub trait IThreadDispatcher: Send + Sync {
    /// Gets the number of workers available in the thread dispatcher.
    ///
    /// Note that some systems (like the solver) expect the ThreadCount to be backed by truly independent threads capable of progression even when one is blocked.
    /// If the ThreadCount doesn't represent independent threads, deadlocks will occur.
    fn thread_count(&self) -> i32;

    /// Gets the unmanaged context associated with the current dispatch, if any.
    /// This is intended to help pass information to workers.
    fn unmanaged_context(&self) -> *mut ();

    /// Gets the managed context associated with the current dispatch, if any.
    fn managed_context(&self) -> Option<&dyn std::any::Any>;

    /// Dispatches all the available workers.
    ///
    /// # Safety
    /// The worker_body function pointer must be valid for the duration of the dispatch.
    /// The unmanaged_context must be valid for the duration of the dispatch if provided.
    unsafe fn dispatch_workers(
        &self,
        worker_body: WorkerBodyFn,
        maximum_worker_count: i32,
        unmanaged_context: *mut (),
        managed_context: Option<&dyn std::any::Any>,
    );

    /// Gets the set of memory pools associated with thread workers.
    /// All usages of these worker pools within the simulation are guaranteed to return thread pool memory before the function returns.
    fn worker_pools(&self) -> &WorkerBufferPools;

    /// Gets a specific worker's buffer pool as an immutable reference.
    ///
    /// # Safety
    /// Caller must ensure no mutable access is happening concurrently.
    fn worker_pool(&self, worker_index: i32) -> &BufferPool {
        unsafe { self.worker_pools().get_pool_ref(worker_index as usize) }
    }

    /// Gets a raw mutable pointer to a specific worker's buffer pool.
    ///
    /// Uses `UnsafeCell::get()` internally, which is the correct Rust mechanism for
    /// obtaining interior mutability.
    ///
    /// # Safety
    /// The caller must guarantee exclusive access to this pool. By the task scheduling design,
    /// each worker thread exclusively accesses its own pool.
    fn worker_pool_ptr(&self, worker_index: i32) -> *mut BufferPool {
        self.worker_pools().get_pool_ptr(worker_index as usize)
    }
}