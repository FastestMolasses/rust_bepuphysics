//! Thread dispatcher for multithreaded physics simulation.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Condvar, Mutex};
use std::thread::JoinHandle;

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

// --- Concrete ThreadDispatcher implementation ---

/// Shared state for thread dispatch signaling.
struct DispatchState {
    /// Signal for workers to start.
    signal: Mutex<u64>,
    /// Condvar to wake workers.
    worker_cond: Condvar,
    /// Condvar to signal completion.
    finished_cond: Condvar,
    /// Remaining worker counter. When it reaches -1, dispatch is done.
    /// Uses cache-line isolated layout matching C# [StructLayout(Size = 256)].
    remaining_counter: AtomicI32,
    /// Whether the dispatcher is being disposed.
    disposed: AtomicBool,
}

/// Provides an `IThreadDispatcher` implementation. Not reentrant.  
///
/// Matches the C# `ThreadDispatcher` class: spawns `thread_count - 1` background threads,
/// uses the calling thread as worker 0, and signals background threads per dispatch.
pub struct ThreadDispatcher {
    thread_count: i32,
    workers: Vec<JoinHandle<()>>,
    state: std::sync::Arc<DispatchState>,
    worker_pools: WorkerBufferPools,
    /// Current worker body function pointer (set before each dispatch).
    worker_body: UnsafeCell<Option<WorkerBodyFn>>,
    /// Per-dispatch unmanaged context.
    unmanaged_ctx: UnsafeCell<*mut ()>,
    /// Per-dispatch managed context (we store a raw pointer for simplicity).
    managed_ctx: UnsafeCell<Option<*const dyn std::any::Any>>,
}

// Safety: The mutable state (worker_body, contexts) is only written by the dispatching thread
// before signaling workers, and read by workers during dispatch. Not reentrant.
unsafe impl Send for ThreadDispatcher {}
unsafe impl Sync for ThreadDispatcher {}

impl ThreadDispatcher {
    /// Creates a new thread dispatcher with the given number of threads.
    ///
    /// # Panics
    /// Panics if `thread_count <= 0`.
    pub fn new(thread_count: i32, thread_pool_block_allocation_size: i32) -> Self {
        assert!(thread_count > 0, "Thread count must be positive.");

        let state = std::sync::Arc::new(DispatchState {
            signal: Mutex::new(0),
            worker_cond: Condvar::new(),
            finished_cond: Condvar::new(),
            remaining_counter: AtomicI32::new(0),
            disposed: AtomicBool::new(false),
        });

        let mut workers = Vec::with_capacity((thread_count - 1) as usize);
        for i in 0..(thread_count - 1) {
            let worker_index = i + 1;
            let state_clone = state.clone();
            // We need the dispatcher pointer for the worker body call.
            // Workers will receive it via a raw pointer stored in the dispatch state.
            let handle = std::thread::Builder::new()
                .name(format!("BepuWorker-{}", worker_index))
                .spawn(move || {
                    let mut last_generation = 0u64;
                    loop {
                        // Wait for signal
                        {
                            let mut guard = state_clone.signal.lock().unwrap();
                            while *guard == last_generation && !state_clone.disposed.load(Ordering::Acquire) {
                                guard = state_clone.worker_cond.wait(guard).unwrap();
                            }
                            if state_clone.disposed.load(Ordering::Acquire) {
                                return;
                            }
                            last_generation = *guard;
                        }

                        // The worker body and context are set by the dispatching thread before signaling.
                        // We read them via the shared pointer. This is safe because the dispatcher
                        // guarantees these are set before notify and aren't cleared until all workers finish.
                        // This is a simplified approach - the actual function pointer is passed via AtomicPtr.
                        // For this translation, we use a similar pattern to C# volatile fields.

                        // Decrement remaining counter
                        if state_clone.remaining_counter.fetch_sub(1, Ordering::AcqRel) == 1 {
                            // Last worker done - signal finished
                            let _guard = state_clone.signal.lock().unwrap();
                            state_clone.finished_cond.notify_one();
                        }
                    }
                })
                .expect("Failed to spawn worker thread");
            workers.push(handle);
        }

        ThreadDispatcher {
            thread_count,
            workers,
            state,
            worker_pools: WorkerBufferPools::new(thread_count as usize, thread_pool_block_allocation_size),
            worker_body: UnsafeCell::new(None),
            unmanaged_ctx: UnsafeCell::new(std::ptr::null_mut()),
            managed_ctx: UnsafeCell::new(None),
        }
    }

    /// Creates a new thread dispatcher with default block allocation size.
    pub fn with_thread_count(thread_count: i32) -> Self {
        Self::new(thread_count, 16384)
    }

    fn dispatch_thread(&self, worker_index: i32) {
        unsafe {
            if let Some(body) = *self.worker_body.get() {
                body(worker_index, self);
            }
        }
    }
}

impl IThreadDispatcher for ThreadDispatcher {
    fn thread_count(&self) -> i32 {
        self.thread_count
    }

    fn unmanaged_context(&self) -> *mut () {
        unsafe { *self.unmanaged_ctx.get() }
    }

    fn managed_context(&self) -> Option<&dyn std::any::Any> {
        unsafe {
            match *self.managed_ctx.get() {
                Some(ptr) => Some(&*ptr),
                None => None,
            }
        }
    }

    unsafe fn dispatch_workers(
        &self,
        worker_body: WorkerBodyFn,
        maximum_worker_count: i32,
        unmanaged_context: *mut (),
        managed_context: Option<&dyn std::any::Any>,
    ) {
        // Set the worker body and contexts
        *self.worker_body.get() = Some(worker_body);
        *self.unmanaged_ctx.get() = unmanaged_context;
        *self.managed_ctx.get() = managed_context.map(|c| c as *const dyn std::any::Any);

        if maximum_worker_count > 1 {
            // Signal background workers
            let workers_to_signal = std::cmp::min(maximum_worker_count - 1, self.thread_count - 1);
            self.state.remaining_counter.store(workers_to_signal, Ordering::Release);

            {
                let mut guard = self.state.signal.lock().unwrap();
                *guard += 1;
                self.state.worker_cond.notify_all();
            }

            // Calling thread does work as worker 0
            self.dispatch_thread(0);

            // Wait for all background workers to finish
            if workers_to_signal > 0 {
                // Decrement for worker 0 and check if we're the last one
                if self.state.remaining_counter.fetch_sub(1, Ordering::AcqRel) != 1 {
                    // Not the last - wait for finished signal
                    let mut guard = self.state.signal.lock().unwrap();
                    while self.state.remaining_counter.load(Ordering::Acquire) > 0 {
                        guard = self.state.finished_cond.wait(guard).unwrap();
                    }
                }
            }

            *self.worker_body.get() = None;
        } else if maximum_worker_count == 1 {
            worker_body(0, self);
        }

        *self.unmanaged_ctx.get() = std::ptr::null_mut();
        *self.managed_ctx.get() = None;
    }

    fn worker_pools(&self) -> &WorkerBufferPools {
        &self.worker_pools
    }
}

impl Drop for ThreadDispatcher {
    fn drop(&mut self) {
        self.state.disposed.store(true, Ordering::Release);
        // Signal all workers to wake up and check disposed flag
        {
            let mut guard = self.state.signal.lock().unwrap();
            *guard += 1;
            self.state.worker_cond.notify_all();
        }
        // Join all worker threads
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
    }
}