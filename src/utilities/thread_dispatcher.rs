//! Thread dispatcher for multithreaded physics simulation.

use crate::utilities::memory::worker_buffer_pools::WorkerBufferPools;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Function to be invoked on a worker thread. Provides the worker index and a reference to the dispatcher.
pub type WorkerBodyFn = unsafe fn(worker_index: i32, dispatcher: &dyn IThreadDispatcher);

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
    fn thread_count(&self) -> i32;

    /// Gets the unmanaged context associated with the current dispatch, if any.
    /// This is intended to help pass information to workers.
    unsafe fn unmanaged_context(&self) -> *mut ();

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

    /// Gets a specific worker's buffer pool.
    fn worker_pool(&self, worker_index: i32) -> &BufferPool {
        &self.worker_pools().pools[worker_index as usize]
    }
}
