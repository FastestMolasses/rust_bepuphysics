// Assuming WorkerBufferPools is defined elsewhere and imported here
use crate::utilities::memory::worker_buffer_pools::WorkerBufferPools;
use std::os::raw::{c_int, c_void};

// Define the worker function type in Rust. The `extern "C"` calling convention is used for FFI compatibility.
// However, in this Rust-only context, we could also use the default Rust calling convention for better performance.
extern "C" {
    fn thread_dispatcher_worker(worker_index: c_int, context: *mut c_void);
}

// Interface translation to Rust uses a trait instead.
pub trait IThreadDispatcher {
    /// Returns the number of workers available in the thread dispatcher.
    /// Must ensure thread independence to avoid deadlocks.
    fn thread_count(&self) -> usize;

    /// Returns the unmanaged context associated with the current dispatch, if any.
    /// Useful for passing information to workers, particularly with function pointers.
    unsafe fn unmanaged_context(&self) -> *mut c_void;

    /// Returns the managed context associated with the current dispatch, if any.
    /// Useful for passing information to workers, especially in more dynamic Rust scenarios.
    fn managed_context(&self) -> &dyn std::any::Any;

    /// Dispatches all the available workers with the given worker body function, maximum worker count, and optional contexts.
    /// The worker body matches the `ThreadDispatcherWorker` signature.
    /// `maximum_worker_count` defaults to `usize::MAX` which represents no practical limit under normal usage.
    unsafe fn dispatch_workers(
        &self,
        worker_body: extern "C" fn(c_int, *mut c_void),
        maximum_worker_count: usize,
        unmanaged_context: *mut c_void,
        managed_context: Option<Box<dyn std::any::Any>>,
    );

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

    /// Returns the set of memory pools associated with thread workers.
    /// All usages within the simulation are ephemeral, ensuring no memory is held outside the function scope.
    fn worker_pools(&self) -> &WorkerBufferPools;
}
