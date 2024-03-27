use std::marker::PhantomData;
use std::ptr::null_mut;
use std::sync::Arc;

/// Function to be invoked on a worker thread in an `IThreadDispatcher`. Provides the unmanaged context of the dispatch.
///
/// # Arguments
///
/// * `worker_index` - Index of the worker in the dispatcher executing this function.
/// * `context` - Pointer to the context of this dispatch, if any.
///
pub type ThreadDispatcherWorker =
    unsafe extern "C" fn(worker_index: i32, context: *mut std::ffi::c_void);

/// Provides multithreading dispatch primitives, a thread count, and per thread resource pools for the simulation to use.
///
/// Note that the simulation does not require a true load balancing for loop implementation. All that's needed is a way to jumpstart some threads.
/// All systems which use multithreading tend to have some form of domain specific load balancing that a general purpose thread pool or parallel for loop implementation
/// couldn't match. The simulation also tends to keep the number of dispatches as low as it can. Combined, these two things reduce the importance of a
/// very highly optimized dispatcher.
///
/// This is important when a user wants to share some other thread pool, but doesn't have the time to guarantee extremely high performance and high quality
/// load balancing. Instead of worrying about that, they can just wrap whatever implementation they happen to have and it'll probably work fine.
pub trait IThreadDispatcher: Send + Sync {
    /// Gets the number of workers available in the thread dispatcher.
    ///
    /// Note that some systems (like the solver) expect the `thread_count` to be backed by truly independent threads capable of progression even when one is blocked.
    /// If the `thread_count` doesn't represent independent threads, deadlocks will occur.
    fn thread_count(&self) -> i32;

    /// Gets the unmanaged context associated with the current dispatch, if any.
    ///
    /// This is intended to help pass information to workers, especially those defined with function pointers that can't just include extra state in closures.
    fn unmanaged_context(&self) -> *mut std::ffi::c_void;

    /// Gets the managed context associated with the current dispatch, if any.
    ///
    /// This is intended to help pass information to workers, especially those defined with function pointers that can't just include extra state in closures.
    fn managed_context(&self) -> Option<Arc<dyn std::any::Any + Send + Sync>>;

    /// Dispatches all the available workers.
    ///
    /// # Arguments
    ///
    /// * `worker_body` - Function pointer to be invoked on every worker. Matches the signature of the `ThreadDispatcherWorker`.
    /// * `maximum_worker_count` - Maximum number of workers to dispatch.
    /// * `unmanaged_context` - Unmanaged context of the dispatch.
    /// * `managed_context` - Managed context of the dispatch.
    unsafe fn dispatch_workers(
        &self,
        worker_body: ThreadDispatcherWorker,
        maximum_worker_count: i32,
        unmanaged_context: *mut std::ffi::c_void,
        managed_context: Option<Arc<dyn std::any::Any + Send + Sync>>,
    );

    /// Dispatches all the available workers with a null context.
    ///
    /// # Arguments
    ///
    /// * `worker_body` - Closure to be invoked on every worker.
    /// * `maximum_worker_count` - Maximum number of workers to dispatch.
    /// * `unmanaged_context` - Unmanaged context of the dispatch.
    /// * `managed_context` - Managed context of the dispatch.
    fn dispatch_worker_closures<F>(
        &self,
        worker_body: F,
        maximum_worker_count: i32,
        unmanaged_context: *mut std::ffi::c_void,
        managed_context: Option<Arc<dyn std::any::Any + Send + Sync>>,
    ) where
        F: Fn(i32) + Send + Sync + 'static;

    /// Dispatches all available workers with a simpler Rust closure, allowing for more idiomatic Rust code.
    /// This version automatically handles the conversion to function pointers internally.
    fn dispatch_workers_rust<F: FnMut(usize)>(
        &self,
        mut worker_body: F,
        maximum_worker_count: usize,
        unmanaged_context: *mut c_void,
        managed_context: Option<Box<dyn std::any::Any>>,
    ) {
        unsafe {
            // Example of converting a Rust closure to a C function pointer.
            // Note: This simple conversion approach might not work for capturing closures.
            // You may need a more complex setup with dynamic dispatch or static variables for stateful closures.
            let wrapper = |index: c_int, context: *mut c_void| {
                worker_body(index as usize);
            };
            let worker_body_ptr: extern "C" fn(c_int, *mut c_void) = wrapper;

            self.dispatch_workers(
                worker_body_ptr,
                maximum_worker_count,
                unmanaged_context,
                managed_context,
            );
        }
    }

    /// Gets the set of memory pools associated with thread workers.
    ///
    /// All usages of these worker pools within the simulation are guaranteed to return thread pool memory before the function returns. In other words,
    /// thread memory pools are used for strictly ephemeral memory, and it will never be held by the simulation outside the scope of a function that
    /// takes the `IThreadDispatcher` as input.
    fn worker_pools(&self) -> &WorkerBufferPools;
}

struct WorkerBufferPools {
    _private: PhantomData<()>,
}
