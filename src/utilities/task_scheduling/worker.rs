use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::continuation_block::ContinuationBlock;
use crate::utilities::task_scheduling::continuation_handle::ContinuationHandle;
use crate::utilities::task_scheduling::job::Job;
use crate::utilities::task_scheduling::task::Task;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::{cell::UnsafeCell, ptr::NonNull};

/// A worker responsible for executing tasks and managing their lifecycles.
/// It emphasizes performance through careful memory management and minimizing synchronization.
pub struct Worker {
    /// Tracks allocations made over the course of the worker's lifetime for later disposal.
    allocated_jobs: UnsafeCell<QuickList<NonNull<Job>>>,
    continuation_head: UnsafeCell<*mut ContinuationBlock>,
    worker_index: usize,
}

impl Worker {
    /// Creates a new worker with a specified index, using a dispatcher to allocate necessary resources.
    pub fn new(
        worker_index: usize,
        dispatcher: &dyn IThreadDispatcher,
        initial_job_capacity: usize,
        continuation_block_capacity: usize,
    ) -> Self {
        let thread_pool = dispatcher.worker_pools().get_pool(worker_index);
        let allocated_jobs = QuickList::with_capacity(initial_job_capacity, thread_pool);
        let continuation_head =
            unsafe { ContinuationBlock::create(continuation_block_capacity, thread_pool) };

        Self {
            allocated_jobs: UnsafeCell::new(allocated_jobs),
            continuation_head: UnsafeCell::new(continuation_head),
            worker_index,
        }
    }

    /// Validates the integrity of tasks within jobs for debugging purposes.
    #[cfg(debug_assertions)]
    pub fn validate_tasks(&self) {
        let allocated_jobs = unsafe { &*self.allocated_jobs.get() };
        for job_ptr in allocated_jobs.iter() {
            let job = unsafe { &**job_ptr };
            for task in job.tasks.iter() {
                debug_assert!(task.function.is_some(), "Task function should not be null.");
            }
        }
    }

    /// Disposes of the worker, cleaning up any allocated jobs and continuation blocks.
    pub fn dispose(&self, thread_pool: &mut BufferPool) {
        let allocated_jobs = unsafe { &mut *self.allocated_jobs.get() };
        for job_ptr in allocated_jobs.iter() {
            unsafe { (**job_ptr).dispose(thread_pool) };
        }
        allocated_jobs.dispose(thread_pool);
        unsafe {
            (**self.continuation_head.get()).dispose(thread_pool);
        }
    }

    /// Resets the worker to its initial state, disposing of current jobs and continuation blocks.
    pub fn reset(&self, thread_pool: &mut BufferPool) {
        let allocated_jobs = unsafe { &mut *self.allocated_jobs.get() };
        for job_ptr in allocated_jobs.iter() {
            unsafe { (**job_ptr).dispose(thread_pool) };
        }
        allocated_jobs.clear();
        let continuation_head = unsafe { &mut *self.continuation_head.get() };
        let capacity = continuation_head.continuations.capacity();
        continuation_head.dispose(thread_pool);
        *continuation_head = unsafe { ContinuationBlock::create(capacity, thread_pool) };
    }

    /// Allocates a job consisting of a set of tasks, tagging it for identification.
    pub fn allocate_job(
        &self,
        tasks: &[Task],
        tag: u64,
        dispatcher: &dyn IThreadDispatcher,
    ) -> *mut Job {
        debug_assert!(
            !tasks.is_empty(),
            "Attempting to push zero tasks is likely a mistake."
        );

        let thread_pool = dispatcher.worker_pools(self.worker_index);
        let job = Job::create(tasks, tag, thread_pool);

        let allocated_jobs = unsafe { &mut *self.allocated_jobs.get() };
        allocated_jobs.allocate(thread_pool).as_mut().unwrap() = NonNull::new(job).unwrap();

        job
    }

    /// Allocates a continuation for a set of tasks, setting up an on-completion action if provided.
    pub fn allocate_continuation(
        &self,
        task_count: usize,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Task,
    ) -> ContinuationHandle {
        let continuation_head_ptr = unsafe { &mut *self.continuation_head.get() };
        let continuation_head = unsafe { &mut **continuation_head_ptr };
        unsafe {
            match continuation_head.try_allocate_continuation() {
                Some(continuation) => {
                    continuation.on_completed = on_completed;
                    continuation.remaining_task_counter = task_count;
                    ContinuationHandle::new(continuation)
                }
                None => {
                    // Need to allocate a new block if we couldn't allocate a continuation
                    let new_block = ContinuationBlock::create(
                        continuation_head.continuations.capacity(),
                        dispatcher.worker_pools(self.worker_index),
                    );
                    new_block.set_previous(NonNull::new(continuation_head_ptr));
                    *continuation_head_ptr = new_block;

                    let continuation = new_block
                        .try_allocate_continuation()
                        .expect("New block should have enough capacity for allocation.");
                    continuation.on_completed = on_completed;
                    continuation.remaining_task_counter = task_count;
                    ContinuationHandle::new(continuation)
                }
            }
        }
    }
}
