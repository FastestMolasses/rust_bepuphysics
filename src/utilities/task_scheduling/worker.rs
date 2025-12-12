//! Worker for task execution in the task scheduling system.
//!
//! Workers manage job allocation and continuation tracking for individual threads.

use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

use super::continuation_block::ContinuationBlock;
use super::continuation_handle::ContinuationHandle;
use super::job::Job;
use super::task::Task;
use super::task_continuation::TaskContinuation;

/// Obtains a mutable reference to a BufferPool for the given worker from the dispatcher.
///
/// Uses `IThreadDispatcher::worker_pool_ptr` which goes through `UnsafeCell::get()`,
/// the correct Rust mechanism for interior mutability.
///
/// # Safety
/// The caller must guarantee exclusive access to this pool. In the task scheduling system,
/// this is ensured by the design constraint that each worker thread exclusively accesses
/// its own pool.
#[inline(always)]
pub(crate) unsafe fn pool_mut(
    dispatcher: &dyn IThreadDispatcher,
    worker_index: i32,
) -> &mut BufferPool {
    &mut *dispatcher.worker_pool_ptr(worker_index)
}

/// A worker responsible for executing tasks and managing their lifecycles.
///
/// Workers track job allocations and manage continuation blocks for their thread.
/// Each worker operates on its own data without synchronization (single-writer).
#[repr(C)]
pub struct Worker {
    /// Tracks allocations made over the course of the worker's lifetime for later disposal.
    /// Stores raw pointers to jobs as `usize` (matching C#'s `QuickList<nuint>`).
    allocated_jobs: QuickList<usize>,
    /// Head of the linked list of continuation blocks.
    continuation_head: *mut ContinuationBlock,
    /// Index of this worker in the thread pool.
    worker_index: i32,
}

// Safety: Worker is only accessed by its owning thread during normal operation.
// Cross-thread access only happens during setup/teardown when properly synchronized.
unsafe impl Send for Worker {}
unsafe impl Sync for Worker {}

impl Worker {
    /// Creates a new worker with a specified index.
    pub fn new(
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        initial_job_capacity: i32,
        continuation_block_capacity: i32,
    ) -> Self {
        let thread_pool = unsafe { pool_mut(dispatcher, worker_index) };
        let allocated_jobs = QuickList::with_capacity(initial_job_capacity, thread_pool);
        let continuation_head =
            unsafe { ContinuationBlock::create(continuation_block_capacity, thread_pool) };

        Self {
            allocated_jobs,
            continuation_head,
            worker_index,
        }
    }

    /// Gets the worker index.
    #[inline(always)]
    pub fn worker_index(&self) -> i32 {
        self.worker_index
    }

    /// Gets the continuation head pointer.
    #[inline(always)]
    pub fn continuation_head(&self) -> *mut ContinuationBlock {
        self.continuation_head
    }

    /// Validates the integrity of tasks within jobs for debugging purposes.
    #[cfg(debug_assertions)]
    pub fn validate_tasks(&self) {
        for i in 0..self.allocated_jobs.count {
            let job_ptr = self.allocated_jobs.span[i] as *const Job;
            unsafe {
                let job = &*job_ptr;
                for j in 0..job.tasks.len() {
                    debug_assert!(
                        job.tasks[j].function.is_some(),
                        "Task function should not be null."
                    );
                }
            }
        }
    }

    /// Disposes of the worker, cleaning up any allocated jobs and continuation blocks.
    ///
    /// # Safety
    /// The thread_pool must be the correct pool for this worker.
    pub unsafe fn dispose(&mut self, thread_pool: &mut BufferPool) {
        for i in 0..self.allocated_jobs.count {
            let job_ptr = self.allocated_jobs.span[i] as *mut Job;
            (*job_ptr).dispose(thread_pool);
        }
        self.allocated_jobs.dispose(thread_pool);
        if !self.continuation_head.is_null() {
            (*self.continuation_head).dispose(thread_pool);
        }
    }

    /// Resets the worker to its initial state, disposing of current jobs and continuation blocks.
    ///
    /// # Safety
    /// The thread_pool must be the correct pool for this worker.
    pub unsafe fn reset(&mut self, thread_pool: &mut BufferPool) {
        for i in 0..self.allocated_jobs.count {
            let job_ptr = self.allocated_jobs.span[i] as *mut Job;
            (*job_ptr).dispose(thread_pool);
        }
        self.allocated_jobs.count = 0;
        let capacity = (*self.continuation_head).continuations.len();
        (*self.continuation_head).dispose(thread_pool);
        self.continuation_head = ContinuationBlock::create(capacity, thread_pool);
    }

    /// Allocates a job consisting of a set of tasks.
    ///
    /// # Safety
    /// - If the worker associated with this stack might be active, this function
    ///   can only be called by the owning worker.
    /// - Tasks must not be empty.
    pub unsafe fn allocate_job(
        &mut self,
        tasks: &[Task],
        tag: u64,
        dispatcher: &dyn IThreadDispatcher,
    ) -> *mut Job {
        debug_assert!(
            !tasks.is_empty(),
            "Probably shouldn't be trying to push zero tasks."
        );

        let thread_pool = pool_mut(dispatcher, self.worker_index);
        // Note that we allocate jobs on the heap directly; it's safe to resize the
        // AllocatedJobs list because it's just storing pointers.
        let job = Job::create(tasks, tag, thread_pool);
        *self.allocated_jobs.allocate(thread_pool) = job as usize;
        job
    }

    /// Allocates a continuation for a set of tasks.
    ///
    /// # Returns
    /// Handle to the newly allocated continuation.
    pub unsafe fn allocate_continuation(
        &mut self,
        task_count: i32,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Task,
    ) -> ContinuationHandle {
        let mut continuation: *mut TaskContinuation = std::ptr::null_mut();

        if !(*self.continuation_head).try_allocate_continuation(&mut continuation) {
            // Couldn't allocate; need to allocate a new block.
            // (The reason for the linked list style allocation is that resizing a buffer—
            // and returning the old buffer—opens up a potential race condition.)
            let thread_pool = pool_mut(dispatcher, self.worker_index);
            let capacity = (*self.continuation_head).continuations.len();
            let new_block = ContinuationBlock::create(capacity, thread_pool);
            (*new_block).previous = self.continuation_head;
            self.continuation_head = new_block;

            let allocated = (*self.continuation_head).try_allocate_continuation(&mut continuation);
            debug_assert!(allocated, "Just created that block! Is the capacity wrong?");
        }

        (*continuation).on_completed = on_completed;
        // C# plain write: continuation->RemainingTaskCounter = taskCount;
        // Single-threaded initialization before the continuation is shared.
        *(*continuation).remaining_task_counter.get() = task_count;
        ContinuationHandle::new(continuation)
    }

    /// Gets the approximate count of continuations (for diagnostics).
    pub fn approximate_continuation_count(&self) -> i32 {
        let mut sum = 0;
        let mut block = self.continuation_head;
        while !block.is_null() {
            unsafe {
                sum += (*block).count;
                block = (*block).previous;
            }
        }
        sum
    }
}
