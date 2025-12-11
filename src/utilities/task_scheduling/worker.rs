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

/// A worker responsible for executing tasks and managing their lifecycles.
///
/// Workers track job allocations and manage continuation blocks for their thread.
/// Each worker operates on its own data without synchronization (single-writer).
#[repr(C)]
pub struct Worker {
    /// Tracks allocations made over the course of the worker's lifetime for later disposal.
    /// Stores raw pointers to jobs.
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
    ///
    /// # Arguments
    /// * `worker_index` - Index of this worker in the thread pool.
    /// * `dispatcher` - Thread dispatcher to get buffer pools from.
    /// * `initial_job_capacity` - Initial number of jobs to allocate space for.
    /// * `continuation_block_capacity` - Number of slots in each continuation block.
    pub fn new(
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        initial_job_capacity: i32,
        continuation_block_capacity: i32,
    ) -> Self {
        let thread_pool = dispatcher.worker_pool(worker_index);
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
    pub unsafe fn dispose(&mut self, thread_pool: &BufferPool) {
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
    pub unsafe fn reset(&mut self, thread_pool: &BufferPool) {
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
    /// # Arguments
    /// * `tasks` - Tasks composing the job.
    /// * `tag` - User tag associated with the job.
    /// * `dispatcher` - Dispatcher used to pull thread allocations if necessary.
    ///
    /// # Returns
    /// Pointer to the newly created job.
    ///
    /// # Safety
    /// - If the worker associated with this stack might be active, this function
    ///   can only be called by the worker.
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

        let thread_pool = dispatcher.worker_pool(self.worker_index);
        let job = Job::create(tasks, tag, thread_pool);
        *self.allocated_jobs.allocate(thread_pool) = job as usize;
        job
    }

    /// Allocates a continuation for a set of tasks.
    ///
    /// # Arguments
    /// * `task_count` - Number of tasks associated with the continuation.
    /// * `dispatcher` - Dispatcher from which to pull a buffer pool if needed.
    /// * `on_completed` - Function to execute upon completing all associated tasks.
    ///
    /// # Returns
    /// Handle to the newly allocated continuation.
    pub unsafe fn allocate_continuation(
        &mut self,
        task_count: i32,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Task,
    ) -> ContinuationHandle {
        // Try to allocate from the current head block
        if let Some(continuation) = (*self.continuation_head).try_allocate_continuation() {
            // Initialize the continuation
            (*continuation).on_completed = on_completed;
            (*continuation).remaining_task_counter = task_count;
            return ContinuationHandle::new(continuation);
        }

        // Couldn't allocate; need to allocate a new block.
        // (The linked list style allocation avoids race conditions that would occur
        // if we resized and returned the old buffer.)
        let thread_pool = dispatcher.worker_pool(self.worker_index);
        let capacity = (*self.continuation_head).continuations.len();
        let new_block = ContinuationBlock::create(capacity, thread_pool);

        // Link new block to old
        (*new_block).previous = self.continuation_head;
        self.continuation_head = new_block;

        // Allocate from the new block
        let continuation = (*self.continuation_head)
            .try_allocate_continuation()
            .expect("Just created that block! Is the capacity wrong?");

        (*continuation).on_completed = on_completed;
        (*continuation).remaining_task_counter = task_count;
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
