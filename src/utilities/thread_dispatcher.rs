use std::{
    marker::PhantomData,
    ptr::NonNull,
    sync::{
        atomic::{AtomicBool, AtomicI32, Ordering},
        Arc, Condvar, Mutex,
    },
    thread::{self, JoinHandle},
};

use crate::utilities::worker_buffer_pools::WorkerBufferPools;

type UnmanagedWorkerFn = unsafe fn(worker_index: i32, dispatcher: &ThreadDispatcher);
type ManagedWorkerFn = Box<dyn Fn(i32) + Send + Sync>;

#[repr(u8)]
enum WorkerType {
    Managed,
    Unmanaged,
}

/// A worker in the thread dispatcher
struct Worker {
    thread: JoinHandle<()>,
    signal: Arc<(Mutex<bool>, Condvar)>,
}

/// Provides multithreading dispatch primitives, a thread count, and per thread resource pools for the simulation to use.
///
/// Note that the simulation does not require a true load balancing forloop implementation. All that's needed is a way to jumpstart some threads.
/// All systems which use multithreading tend to have some form of domain specific load balancing that a general purpose thread pool or parallel for loop implementation 
/// couldn't match. The simulation also tends to keep the number of dispatches as low as it can. Combined, these two things reduce the importance of a
/// very highly optimized dispatcher.
///
/// This is important when a user wants to share some other thread pool, but doesn't want to guarantee extremely high performance and high quality
/// load balancing. Instead of worrying about that, they can just wrap whatever implementation they happen to have and it'll probably work fine.
pub trait ThreadDispatcher: Send + Sync {
    /// Gets the number of workers available in the thread dispatcher.
    fn thread_count(&self) -> i32;

    /// Gets the unmanaged context associated with the current dispatch, if any.
    unsafe fn unmanaged_context(&self) -> Option<NonNull<()>>;

    /// Gets the managed context associated with the current dispatch, if any.
    fn managed_context(&self) -> Option<&()>;

    /// Dispatches all the available workers.
    ///
    /// # Safety
    /// The worker_body function pointer must be valid for the duration of the dispatch.
    /// The unmanaged_context must be valid for the duration of the dispatch if provided.
    unsafe fn dispatch_workers(
        &self,
        worker_body: UnmanagedWorkerFn,
        maximum_worker_count: Option<i32>,
        unmanaged_context: Option<NonNull<()>>,
        managed_context: Option<&()>,
    );

    /// Dispatches all the available workers with a managed worker function.
    fn dispatch_workers_managed(
        &self,
        worker_body: impl Fn(i32) + Send + Sync + 'static,
        maximum_worker_count: Option<i32>,
        unmanaged_context: Option<NonNull<()>>,
        managed_context: Option<&()>,
    );

    /// Gets the set of memory pools associated with thread workers.
    fn worker_pools(&self) -> &WorkerBufferPools;
}
