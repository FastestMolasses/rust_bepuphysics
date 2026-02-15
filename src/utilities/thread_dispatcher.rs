//! Thread dispatcher for multithreaded physics simulation.
//!
//! Matches C# `ThreadDispatcher` class: spawns `thread_count - 1` background threads,
//! uses the calling thread as worker 0, and per-worker `AutoResetEvent`-style signals.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Condvar, Mutex};
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

// --- Cache-line padded counter matching C# [StructLayout(LayoutKind.Explicit, Size = 256)] Counter ---
// The 128-byte padding before and after isolates the counter from other fields to prevent false sharing.

#[repr(C)]
struct PaddedCounter {
    _pad_before: [u8; 128],
    /// The actual counter value. Accessed via plain writes (single-threaded init) and
    /// `AtomicI32::from_ptr()` for `Interlocked.Decrement`.
    value: UnsafeCell<i32>,
    _pad_after: [u8; 124],
}

const _: () = {
    assert!(std::mem::size_of::<PaddedCounter>() == 256);
};

impl PaddedCounter {
    fn new() -> Self {
        PaddedCounter {
            _pad_before: [0u8; 128],
            value: UnsafeCell::new(0),
            _pad_after: [0u8; 124],
        }
    }

    /// Plain store (C# non-volatile write, single-threaded setup).
    #[inline(always)]
    unsafe fn store_plain(&self, val: i32) {
        *self.value.get() = val;
    }

    /// Atomic decrement matching C# `Interlocked.Decrement(ref Value)`.
    /// Returns the NEW value (post-decrement), matching Interlocked.Decrement semantics.
    #[inline(always)]
    unsafe fn interlocked_decrement(&self) -> i32 {
        AtomicI32::from_ptr(self.value.get()).fetch_sub(1, Ordering::AcqRel) - 1
    }
}

// --- Shared dispatch state (accessible by all workers via Arc) ---

struct DispatchState {
    /// Per-worker AutoResetEvent equivalent: `(signaled_flag, condvar)`.
    /// Index `i` corresponds to background worker `i+1` (worker 0 is the calling thread).
    worker_signals: Vec<(Mutex<bool>, Condvar)>,
    /// Finished signal for the calling thread (AutoResetEvent equivalent).
    finished: (Mutex<bool>, Condvar),
    /// Cache-line isolated remaining worker counter.
    /// Set to `workers_to_signal` before dispatch; all N workers (background + worker 0) decrement.
    /// When counter reaches -1, the last worker signals `finished`.
    remaining_counter: PaddedCounter,
    /// Current worker body function pointer. Set before signaling, read by workers.
    /// Volatile-equivalent semantics: Mutex release/acquire provides happens-before.
    worker_body: UnsafeCell<Option<WorkerBodyFn>>,
    /// Raw fat pointer to `&dyn IThreadDispatcher` (the ThreadDispatcher itself).
    /// Valid only during dispatch (calling thread blocks until completion).
    dispatcher_ptr: UnsafeCell<Option<*const dyn IThreadDispatcher>>,
    /// Per-dispatch unmanaged context.
    unmanaged_ctx: UnsafeCell<*mut ()>,
    /// Per-dispatch managed context.
    managed_ctx: UnsafeCell<Option<*const dyn std::any::Any>>,
    /// Disposed flag (C# `volatile bool disposed`).
    disposed: AtomicBool,
}

unsafe impl Send for DispatchState {}
unsafe impl Sync for DispatchState {}

impl DispatchState {
    /// Execute worker body and decrement counter. Signals finished if last.
    /// Matches C# `DispatchThread(int workerIndex)`.
    #[inline(never)]
    unsafe fn dispatch_thread(&self, worker_index: i32) {
        // Call the worker body (C#: unmanagedWorker(workerIndex, this))
        let body = (*self.worker_body.get()).unwrap();
        let dispatcher = &*(*self.dispatcher_ptr.get()).unwrap();
        body(worker_index, dispatcher);

        // Interlocked.Decrement(ref remainingWorkerCounter.Value) == -1
        if self.remaining_counter.interlocked_decrement() == -1 {
            // Last worker done — signal finished (C#: finished.Set())
            let (lock, cvar) = &self.finished;
            let mut done = lock.lock().unwrap();
            *done = true;
            cvar.notify_one();
        }
    }
}

/// Provides an `IThreadDispatcher` implementation. Not reentrant.
///
/// Matches the C# `ThreadDispatcher` class: spawns `thread_count - 1` background threads,
/// uses the calling thread as worker 0, and per-worker signals for dispatch.
pub struct ThreadDispatcher {
    thread_count: i32,
    workers: Vec<JoinHandle<()>>,
    state: Arc<DispatchState>,
    worker_pools: WorkerBufferPools,
}

// Safety: The mutable state (worker_body, contexts) in DispatchState is only written by the
// dispatching thread before signaling workers, and read by workers after being signaled.
// The Mutex/Condvar provides the necessary happens-before ordering. Not reentrant.
unsafe impl Send for ThreadDispatcher {}
unsafe impl Sync for ThreadDispatcher {}

impl ThreadDispatcher {
    /// Creates a new thread dispatcher with the given number of threads.
    ///
    /// # Panics
    /// Panics if `thread_count <= 0`.
    pub fn new(thread_count: i32, thread_pool_block_allocation_size: i32) -> Self {
        assert!(thread_count > 0, "Thread count must be positive.");

        let background_count = (thread_count - 1) as usize;
        let mut worker_signals = Vec::with_capacity(background_count);
        for _ in 0..background_count {
            worker_signals.push((Mutex::new(false), Condvar::new()));
        }

        let state = Arc::new(DispatchState {
            worker_signals,
            finished: (Mutex::new(false), Condvar::new()),
            remaining_counter: PaddedCounter::new(),
            worker_body: UnsafeCell::new(None),
            dispatcher_ptr: UnsafeCell::new(None),
            unmanaged_ctx: UnsafeCell::new(std::ptr::null_mut()),
            managed_ctx: UnsafeCell::new(None),
            disposed: AtomicBool::new(false),
        });

        let mut workers = Vec::with_capacity(background_count);
        for i in 0..background_count {
            let worker_index = (i + 1) as i32;
            let state_clone = state.clone();
            let handle = std::thread::Builder::new()
                .name(format!("BepuWorker-{}", worker_index))
                .spawn(move || {
                    // Matches C# WorkerLoop(object untypedSignal)
                    loop {
                        // AutoResetEvent.WaitOne()
                        {
                            let (lock, cvar) = &state_clone.worker_signals[i];
                            let mut signaled = lock.lock().unwrap();
                            while !*signaled {
                                signaled = cvar.wait(signaled).unwrap();
                            }
                            *signaled = false; // Auto-reset
                        }
                        if state_clone.disposed.load(Ordering::Acquire) {
                            return;
                        }
                        // Do work and decrement counter (C#: DispatchThread(workerIndex))
                        unsafe {
                            state_clone.dispatch_thread(worker_index);
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
            worker_pools: WorkerBufferPools::new(
                thread_count as usize,
                thread_pool_block_allocation_size,
            ),
        }
    }

    /// Creates a new thread dispatcher with default block allocation size.
    pub fn with_thread_count(thread_count: i32) -> Self {
        Self::new(thread_count, 16384)
    }

    /// Matches C# `SignalThreads(int maximumWorkerCount)`.
    fn signal_threads(&self, maximum_worker_count: i32) {
        // Worker 0 is not signalled; it's the executing thread.
        let maximum_workers_to_signal = maximum_worker_count - 1;
        let workers_to_signal =
            std::cmp::min(maximum_workers_to_signal, self.thread_count - 1) as usize;
        // C# plain write: remainingWorkerCounter.Value = workersToSignal
        unsafe {
            self.state
                .remaining_counter
                .store_plain(workers_to_signal as i32);
        }
        for i in 0..workers_to_signal {
            let (lock, cvar) = &self.state.worker_signals[i];
            let mut signaled = lock.lock().unwrap();
            *signaled = true;
            cvar.notify_one();
        }
    }
}

impl IThreadDispatcher for ThreadDispatcher {
    fn thread_count(&self) -> i32 {
        self.thread_count
    }

    fn unmanaged_context(&self) -> *mut () {
        unsafe { *self.state.unmanaged_ctx.get() }
    }

    fn managed_context(&self) -> Option<&dyn std::any::Any> {
        unsafe {
            match *self.state.managed_ctx.get() {
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
        // Set the worker body and contexts (C# volatile writes, before signaling)
        *self.state.worker_body.get() = Some(worker_body);
        *self.state.unmanaged_ctx.get() = unmanaged_context;
        *self.state.managed_ctx.get() =
            managed_context.map(|c| c as *const dyn std::any::Any);
        *self.state.dispatcher_ptr.get() =
            Some(self as &dyn IThreadDispatcher as *const dyn IThreadDispatcher);

        if maximum_worker_count > 1 {
            self.signal_threads(maximum_worker_count);
            // Calling thread does work as worker 0 (C#: DispatchThread(0))
            self.state.dispatch_thread(0);
            // Wait for all workers (C#: finished.WaitOne())
            {
                let (lock, cvar) = &self.state.finished;
                let mut done = lock.lock().unwrap();
                while !*done {
                    done = cvar.wait(done).unwrap();
                }
                *done = false; // Reset for next dispatch
            }
            *self.state.worker_body.get() = None;
        } else if maximum_worker_count == 1 {
            worker_body(0, self);
        }

        *self.state.unmanaged_ctx.get() = std::ptr::null_mut();
        *self.state.managed_ctx.get() = None;
        *self.state.dispatcher_ptr.get() = None;
    }

    fn worker_pools(&self) -> &WorkerBufferPools {
        &self.worker_pools
    }
}

impl Drop for ThreadDispatcher {
    fn drop(&mut self) {
        self.state.disposed.store(true, Ordering::Release);
        // Signal all workers to wake up and check disposed flag (C#: Dispose → SignalThreads)
        for i in 0..self.workers.len() {
            let (lock, cvar) = &self.state.worker_signals[i];
            let mut signaled = lock.lock().unwrap();
            *signaled = true;
            cvar.notify_one();
        }
        // Join all worker threads
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
    }
}
