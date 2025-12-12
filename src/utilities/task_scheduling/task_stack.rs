//! Lock-free task stack for parallel task execution.
//!
//! TaskStack provides a thread-safe work-stealing task queue with support for
//! continuations, job tagging/filtering, and efficient parallel for loops.

use std::cell::UnsafeCell;
use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicPtr, Ordering};

use crossbeam_utils::Backoff;

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

use super::continuation_handle::ContinuationHandle;
use super::job::Job;
use super::job_filter::{AllowAllJobs, IJobFilter};
use super::pop_task_result::PopTaskResult;
use super::task::Task;
use super::worker::Worker;

/// Padding to prevent false sharing on the stop flag.
///
/// Matches C# `[StructLayout(LayoutKind.Explicit, Size = 256 + 16)]` with
/// `[FieldOffset(128)]` for the stop flag.
#[repr(C)]
struct StopPad {
    /// Padding before the stop flag (128 bytes).
    _padding_before: [u8; 128],
    /// Stop flag — C# declares as `volatile bool`.
    /// UnsafeCell enables volatile access via `AtomicBool::from_ptr()` only where needed.
    stop: UnsafeCell<bool>,
    /// Padding after the stop flag (143 bytes, total struct = 272 bytes).
    _padding_after: [u8; 143],
}

// Verify layout matches C# at compile time.
const _: () = {
    assert!(std::mem::offset_of!(StopPad, stop) == 128);
    assert!(std::mem::size_of::<StopPad>() == 272);
};

impl Default for StopPad {
    fn default() -> Self {
        Self {
            _padding_before: [0; 128],
            stop: UnsafeCell::new(false),
            _padding_after: [0; 143],
        }
    }
}

/// Manages a linked stack of tasks for parallel execution.
///
/// TaskStack provides:
/// - Lock-free concurrent task pushing and popping
/// - Continuation support for task dependencies
/// - Job tagging and filtering for selective work stealing
/// - Parallel for loop primitives
pub struct TaskStack {
    /// Per-worker storage for job allocation and continuation management.
    workers: Buffer<Worker>,
    /// Padded stop flag to prevent false sharing.
    padded: StopPad,
    /// Head of the job stack (most recently pushed job).
    /// C# declares as `volatile nuint head`.
    /// UnsafeCell enables volatile/atomic access via `AtomicPtr::from_ptr()` only where needed.
    head: UnsafeCell<*mut Job>,
}

// Safety: TaskStack uses atomic operations for all concurrent access.
// Workers are accessed only by their owning thread.
unsafe impl Send for TaskStack {}
unsafe impl Sync for TaskStack {}

impl TaskStack {
    /// Gets a mutable reference to a worker from this TaskStack.
    ///
    /// # Safety
    /// Caller must ensure that only the owning worker thread mutates through this pointer.
    /// This is safe because each worker is only accessed by its own thread by design.
    #[inline(always)]
    unsafe fn worker_mut(&self, index: i32) -> &mut Worker {
        &mut *(self.workers.as_ptr().add(index as usize) as *mut Worker)
    }

    /// Constructs a new parallel task stack.
    ///
    /// # Arguments
    /// * `pool` - Buffer pool to allocate non-thread allocated resources from.
    /// * `dispatcher` - Thread dispatcher to pull thread pools from.
    /// * `worker_count` - Number of workers to allocate space for.
    /// * `initial_worker_job_capacity` - Initial job capacity per worker (default: 128).
    /// * `continuation_block_capacity` - Slots per continuation block (default: 128).
    pub fn new(
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        worker_count: i32,
        initial_worker_job_capacity: i32,
        continuation_block_capacity: i32,
    ) -> Self {
        let mut workers: Buffer<Worker> = pool.take(worker_count);

        for i in 0..worker_count as usize {
            unsafe {
                std::ptr::write(
                    workers.as_mut_ptr().add(i),
                    Worker::new(
                        i as i32,
                        dispatcher,
                        initial_worker_job_capacity,
                        continuation_block_capacity,
                    ),
                );
            }
        }

        let mut stack = Self {
            workers,
            padded: StopPad::default(),
            head: UnsafeCell::new(std::ptr::null_mut()),
        };

        stack.reset(dispatcher);
        stack
    }

    /// Returns the stack to a fresh state without reallocating.
    pub fn reset(&mut self, dispatcher: &dyn IThreadDispatcher) {
        for i in 0..self.workers.len() {
            unsafe {
                let worker = &mut *self.workers.as_mut_ptr().add(i as usize);
                let thread_pool = &mut *dispatcher.worker_pool_ptr(worker.worker_index());
                worker.reset(thread_pool);
            }
        }
        // C# volatile writes: padded.Stop = false; head = (nuint)null;
        unsafe {
            AtomicBool::from_ptr(self.padded.stop.get()).store(false, Ordering::Release);
            AtomicPtr::<Job>::from_ptr(self.head.get()).store(std::ptr::null_mut(), Ordering::Release);
        }
    }

    /// Returns unmanaged resources held by the TaskStack to a pool.
    pub fn dispose(&mut self, pool: &mut BufferPool, dispatcher: &dyn IThreadDispatcher) {
        for i in 0..self.workers.len() {
            unsafe {
                let worker = &mut *self.workers.as_mut_ptr().add(i as usize);
                let worker_pool = &mut *dispatcher.worker_pool_ptr(worker.worker_index());
                worker.dispose(worker_pool);
            }
        }
        pool.return_buffer(&mut self.workers);
    }

    /// Gets the approximate number of active tasks.
    /// Not guaranteed to measure the true number at any point in time.
    pub fn approximate_task_count(&self) -> i32 {
        let mut sum = 0i32;
        // C# volatile read: var job = (Job*)head;
        let mut job = unsafe { AtomicPtr::<Job>::from_ptr(self.head.get()).load(Ordering::Acquire) };
        while !job.is_null() {
            unsafe {
                // C# plain read: job->Counter (NOT volatile, NOT Interlocked)
                // Relaxed = plain load on all architectures, avoids Rust data-race UB.
                sum += AtomicI32::from_ptr((*job).counter.get()).load(Ordering::Relaxed).max(0);
                job = (*job).previous;
            }
        }
        sum
    }

    /// Gets the approximate number of active continuations.
    /// Not guaranteed to measure the true number; checks each worker sequentially.
    pub fn approximate_continuation_count(&self) -> i32 {
        let mut sum = 0;
        for i in 0..self.workers.len() {
            sum += self.workers[i].approximate_continuation_count();
        }
        sum
    }

    /// Attempts to allocate a continuation for a set of tasks.
    pub fn allocate_continuation(
        &self,
        task_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Task,
    ) -> ContinuationHandle {
        unsafe {
            self.worker_mut(worker_index)
                .allocate_continuation(task_count, dispatcher, on_completed)
        }
    }

    /// Attempts to pop a task from the stack.
    ///
    /// This implementation does not need to lock against anything. We just follow the pointer.
    ///
    /// # Arguments
    /// * `filter` - Filter to apply to jobs. Only allowed jobs can have tasks popped from them.
    /// * `task` - Output: popped task, if any.
    ///
    /// # Returns
    /// Result status of the pop attempt.
    pub fn try_pop<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
        task: &mut Task,
    ) -> PopTaskResult {
        // C# volatile read: var job = (Job*)head;
        let mut job = unsafe { AtomicPtr::<Job>::from_ptr(self.head.get()).load(Ordering::Acquire) };

        loop {
            if job.is_null() {
                // There is no job to pop from.
                *task = Task::default();
                // C# volatile read: padded.Stop
                return if unsafe { AtomicBool::from_ptr(self.padded.stop.get()).load(Ordering::Acquire) } {
                    PopTaskResult::Stop
                } else {
                    PopTaskResult::Empty
                };
            }

            unsafe {
                // Check if this job is allowed by the filter
                if !filter.allow_job((*job).tag) {
                    // This job isn't allowed for this pop; go to the next one.
                    job = (*job).previous;
                    continue;
                }

                // Try to pop a task from the current job
                if (*job).try_pop(task) {
                    debug_assert!(task.function.is_some());
                    return PopTaskResult::Success;
                } else {
                    // There was no task available in this job, which means the sampled job
                    // should be removed from the stack.
                    // Note that other threads might be doing the same thing; we must use an
                    // interlocked operation to try to swap the head.
                    // If this fails, the head has changed before we could remove it and the
                    // current empty job will persist in the stack until some other dequeue
                    // finds it. That's okay.
                    let previous = (*job).previous;
                    // C#: Interlocked.CompareExchange(ref head, (nuint)job->Previous, (nuint)job);
                    // C# ignores the return value and always re-reads head.
                    let _ = AtomicPtr::<Job>::from_ptr(self.head.get())
                        .compare_exchange(job, previous, Ordering::AcqRel, Ordering::Relaxed);
                    // C# volatile read: job = (Job*)head;
                    job = AtomicPtr::<Job>::from_ptr(self.head.get()).load(Ordering::Acquire);
                }
            }
        }
    }

    /// Attempts to pop a task without filtering.
    #[inline(always)]
    pub fn try_pop_unfiltered(&self, task: &mut Task) -> PopTaskResult {
        self.try_pop(&AllowAllJobs, task)
    }

    /// Attempts to pop a task and run it.
    #[inline(always)]
    pub fn try_pop_and_run<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) -> PopTaskResult {
        let mut task = Task::default();
        let result = self.try_pop(filter, &mut task);
        if result == PopTaskResult::Success {
            unsafe {
                task.run(worker_index, dispatcher);
            }
        }
        result
    }

    /// Attempts to pop and run without filtering.
    #[inline(always)]
    pub fn try_pop_and_run_unfiltered(
        &self,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) -> PopTaskResult {
        self.try_pop_and_run(&AllowAllJobs, worker_index, dispatcher)
    }

    /// Pushes a set of tasks onto the task stack. **Not thread safe.**
    ///
    /// # Safety
    /// Must not be used while other threads could be performing pushes or pops
    /// that could affect the specified worker.
    pub unsafe fn push_unsafely(
        &self,
        tasks: &[Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        let job = self.worker_mut(worker_index).allocate_job(tasks, tag, dispatcher);
        // C# volatile read: job->Previous = (Job*)head;
        (*job).previous = AtomicPtr::<Job>::from_ptr(self.head.get()).load(Ordering::Acquire);
        // C# volatile write: head = (nuint)job;
        AtomicPtr::<Job>::from_ptr(self.head.get()).store(job, Ordering::Release);
    }

    /// Pushes a single task onto the task stack. **Not thread safe.**
    ///
    /// # Safety
    /// Must not be used while other threads could be performing pushes or pops.
    #[inline(always)]
    pub unsafe fn push_unsafely_single(
        &self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.push_unsafely(&[task], worker_index, dispatcher, tag);
    }

    /// Pushes a set of tasks onto the task stack (thread-safe).
    pub fn push(
        &self,
        tasks: &[Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        unsafe {
            let job = self.worker_mut(worker_index).allocate_job(tasks, tag, dispatcher);

            loop {
                // C# volatile read: job->Previous = (Job*)head;
                (*job).previous = AtomicPtr::<Job>::from_ptr(self.head.get()).load(Ordering::Acquire);

                // C#: if ((nuint)job->Previous == Interlocked.CompareExchange(ref head, (nuint)job, (nuint)job->Previous))
                // CompareExchange returns the original value; if it matches Previous, the swap succeeded.
                match AtomicPtr::<Job>::from_ptr(self.head.get()).compare_exchange_weak(
                    (*job).previous,
                    job,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(_) => continue,
                }
            }
        }
    }

    /// Pushes a single task onto the task stack (thread-safe).
    #[inline(always)]
    pub fn push_single(
        &self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.push(&[task], worker_index, dispatcher, tag);
    }

    /// Pushes tasks with a created continuation.
    ///
    /// # Returns
    /// Handle of the created continuation.
    pub fn allocate_continuation_and_push(
        &self,
        tasks: &mut [Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
        on_complete: Task,
    ) -> ContinuationHandle {
        let continuation_handle =
            self.allocate_continuation(tasks.len() as i32, worker_index, dispatcher, on_complete);

        for task in tasks.iter_mut() {
            debug_assert!(
                !task.continuation.initialized(),
                "This function creates a continuation for the tasks"
            );
            task.continuation = continuation_handle;
        }

        self.push(tasks, worker_index, dispatcher, tag);
        continuation_handle
    }

    /// Waits for a continuation to complete, executing other tasks while waiting.
    ///
    /// Instead of spinning the entire time, this may pop and execute pending tasks to fill the gap.
    pub fn wait_for_completion<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
        continuation: ContinuationHandle,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let backoff = Backoff::new();
        debug_assert!(
            continuation.initialized(),
            "This codepath should only run if the continuation was allocated earlier."
        );

        while !continuation.completed() {
            let mut task = Task::default();
            let result = self.try_pop(filter, &mut task);

            match result {
                PopTaskResult::Stop => return,
                PopTaskResult::Success => {
                    unsafe {
                        task.run(worker_index, dispatcher);
                    }
                    backoff.reset();
                }
                PopTaskResult::Empty => {
                    backoff.snooze();
                }
            }
        }
    }

    /// Waits for a continuation without filtering.
    #[inline(always)]
    pub fn wait_for_completion_unfiltered(
        &self,
        continuation: ContinuationHandle,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        self.wait_for_completion(&AllowAllJobs, continuation, worker_index, dispatcher);
    }

    /// Pushes tasks and returns when all are complete.
    ///
    /// The calling thread will execute the first task directly, then help
    /// work on remaining tasks until all complete.
    ///
    /// Note that this will keep working until all tasks are run. It may execute tasks
    /// unrelated to the requested tasks while waiting on other workers to complete
    /// constituent tasks.
    pub fn run_tasks<TJobFilter: IJobFilter>(
        &self,
        tasks: &mut [Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        filter: &TJobFilter,
        tag: u64,
    ) {
        if tasks.is_empty() {
            return;
        }

        let mut continuation_handle = ContinuationHandle::default();

        if tasks.len() > 1 {
            // Note that we only submit tasks to the stack for tasks beyond the first.
            // The current thread is responsible for at least task 0.
            let task_count = (tasks.len() - 1) as i32;

            continuation_handle = unsafe {
                self.worker_mut(worker_index).allocate_continuation(
                    task_count,
                    dispatcher,
                    Task::default(),
                )
            };

            // Build tasks_to_push with the continuation set.
            let mut tasks_to_push = Vec::with_capacity(task_count as usize);
            for i in 1..tasks.len() {
                let mut task = tasks[i];
                debug_assert!(
                    !task.continuation.initialized(),
                    "None of the source tasks should have continuations when provided to run_tasks."
                );
                task.continuation = continuation_handle;
                tasks_to_push.push(task);
            }
            self.push(&tasks_to_push, worker_index, dispatcher, tag);
        }

        // Tasks [1, count) are submitted to the stack and may now be executing on other workers.
        // The thread calling the for loop should not relinquish its timeslice.
        // It should immediately begin working on task 0.
        let task0 = &tasks[0];
        debug_assert!(
            !task0.continuation.initialized(),
            "None of the source tasks should have continuations when provided to run_tasks."
        );

        unsafe {
            (task0.function.unwrap_unchecked())(task0.id, task0.context, worker_index, dispatcher);
        }

        if tasks.len() > 1 {
            // Task 0 is done; this thread should seek out other work until the job is complete.
            self.wait_for_completion(filter, continuation_handle, worker_index, dispatcher);
        }
    }

    /// Pushes tasks and returns when all are complete (no filtering).
    #[inline(always)]
    pub fn run_tasks_unfiltered(
        &self,
        tasks: &mut [Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.run_tasks(tasks, worker_index, dispatcher, &AllowAllJobs, tag);
    }

    /// Runs a single task.
    #[inline(always)]
    pub fn run_task(
        &self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.run_tasks_unfiltered(&mut [task], worker_index, dispatcher, tag);
    }

    /// Runs a single task with filtering.
    #[inline(always)]
    pub fn run_task_filtered<TJobFilter: IJobFilter>(
        &self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        filter: &TJobFilter,
        tag: u64,
    ) {
        self.run_tasks(&mut [task], worker_index, dispatcher, filter, tag);
    }

    /// Requests that all workers stop.
    /// The next time a worker runs out of tasks, if it sees a stop command, it will be reported.
    pub fn request_stop(&self) {
        // C# volatile write: padded.Stop = true;
        unsafe { AtomicBool::from_ptr(self.padded.stop.get()).store(true, Ordering::Release) };
    }

    /// Convenience function for requesting a stop.
    /// Requires the context to be a pointer to the TaskStack.
    pub unsafe fn request_stop_task_function(
        _id: i64,
        untyped_context: *mut c_void,
        _worker_index: i32,
        _dispatcher: &dyn IThreadDispatcher,
    ) {
        (*(untyped_context as *mut TaskStack)).request_stop();
    }

    /// Convenience function for getting a task representing a stop request.
    pub fn get_request_stop_task(stack: *mut TaskStack) -> Task {
        Task::new(
            Self::request_stop_task_function,
            stack as *mut c_void,
            0,
            ContinuationHandle::default(),
        )
    }

    /// Pushes a for loop onto the task stack (not thread safe).
    ///
    /// # Safety
    /// Must not be used while other threads could affect the specified worker.
    pub unsafe fn push_for_unsafely(
        &self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
        continuation: ContinuationHandle,
    ) {
        let mut tasks = Vec::with_capacity(iteration_count as usize);
        for i in 0..iteration_count {
            tasks.push(Task::new(
                function,
                context,
                (inclusive_start_index + i) as i64,
                continuation,
            ));
        }
        self.push_unsafely(&tasks, worker_index, dispatcher, tag);
    }

    /// Pushes a for loop onto the task stack (thread-safe).
    ///
    /// This function will not attempt to run any iterations of the loop itself.
    pub fn push_for(
        &self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
        continuation: ContinuationHandle,
    ) {
        let mut tasks = Vec::with_capacity(iteration_count as usize);
        for i in 0..iteration_count {
            tasks.push(Task::new(
                function,
                context,
                (inclusive_start_index + i) as i64,
                continuation,
            ));
        }
        self.push(&tasks, worker_index, dispatcher, tag);
    }

    /// Submits a for loop and returns when all iterations complete.
    pub fn for_loop<TJobFilter: IJobFilter>(
        &self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        filter: &TJobFilter,
        tag: u64,
    ) {
        if iteration_count <= 0 {
            return;
        }

        let mut tasks = Vec::with_capacity(iteration_count as usize);
        for i in 0..iteration_count {
            tasks.push(Task::with_context(
                function,
                context,
                (inclusive_start_index + i) as i64,
            ));
        }
        self.run_tasks(&mut tasks, worker_index, dispatcher, filter, tag);
    }

    /// Submits a for loop and returns when all iterations complete (no filtering).
    #[inline(always)]
    pub fn for_loop_unfiltered(
        &self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.for_loop(
            function,
            context,
            inclusive_start_index,
            iteration_count,
            worker_index,
            dispatcher,
            &AllowAllJobs,
            tag,
        );
    }

    /// Worker function that pops tasks from the stack and executes them.
    ///
    /// Uses `crossbeam_utils::Backoff` for adaptive spinning that matches C#'s `SpinWait`
    /// behavior with `-1` parameter (spin-then-yield, no sleep).
    pub fn dispatch_worker_function(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        let task_stack = unsafe { &*(dispatcher.unmanaged_context() as *const TaskStack) };
        let backoff = Backoff::new();

        loop {
            match task_stack.try_pop_and_run_unfiltered(worker_index, dispatcher) {
                PopTaskResult::Stop => {
                    // Done!
                    return;
                }
                PopTaskResult::Success => {
                    // If we ran a task, reset the backoff because more work may be
                    // immediately available.
                    backoff.reset();
                }
                PopTaskResult::Empty => {
                    // No work available, but we should keep going.
                    backoff.snooze();
                }
            }
        }
    }

    /// Dispatches workers to execute tasks from the given stack.
    pub fn dispatch_workers(
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        maximum_worker_count: i32,
    ) {
        unsafe {
            dispatcher.dispatch_workers(
                Self::dispatch_worker_function,
                maximum_worker_count,
                task_stack as *mut (),
                None,
            );
        }
    }
}
