//! Lock-free task stack for parallel task execution.
//!
//! TaskStack provides a thread-safe work-stealing task queue with support for
//! continuations, job tagging/filtering, and efficient parallel for loops.

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

use super::continuation_handle::ContinuationHandle;
use super::job::Job;
use super::job_filter::{AllowAllJobs, IJobFilter};
use super::pop_task_result::PopTaskResult;
use super::task::Task;
use super::worker::Worker;

use std::ffi::c_void;
use std::hint::spin_loop;
use std::ptr;

/// Padding to prevent false sharing on the stop flag.
/// Uses 256 + 16 bytes to match the C# implementation's cache line isolation.
#[repr(C, align(128))]
struct StopPad {
    /// Padding before the stop flag.
    _padding_before: [u8; 128],
    /// Stop flag - when true, workers should stop after exhausting work.
    stop: bool,
    /// Padding after the stop flag.
    _padding_after: [u8; 127],
}

impl Default for StopPad {
    fn default() -> Self {
        Self {
            _padding_before: [0; 128],
            stop: false,
            _padding_after: [0; 127],
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
    /// May be null if the stack is empty.
    head: *mut Job,
}

// Safety: TaskStack is designed for concurrent access.
unsafe impl Send for TaskStack {}
unsafe impl Sync for TaskStack {}

impl TaskStack {
    /// Constructs a new parallel task stack.
    ///
    /// # Arguments
    /// * `pool` - Buffer pool to allocate non-thread allocated resources from.
    /// * `dispatcher` - Thread dispatcher to pull thread pools from.
    /// * `worker_count` - Number of workers to allocate space for.
    /// * `initial_worker_job_capacity` - Initial job capacity per worker (default: 128).
    /// * `continuation_block_capacity` - Slots per continuation block (default: 128).
    pub fn new(
        pool: &BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        worker_count: i32,
        initial_worker_job_capacity: i32,
        continuation_block_capacity: i32,
    ) -> Self {
        let mut workers: Buffer<Worker> = pool.take(worker_count);

        for i in 0..worker_count {
            workers[i] = Worker::new(
                i,
                dispatcher,
                initial_worker_job_capacity,
                continuation_block_capacity,
            );
        }

        let mut stack = Self {
            workers,
            padded: StopPad::default(),
            head: ptr::null_mut(),
        };

        stack.reset(dispatcher);
        stack
    }

    /// Returns the stack to a fresh state without reallocating.
    ///
    /// # Arguments
    /// * `dispatcher` - Dispatcher whose thread pools should be used.
    pub fn reset(&mut self, dispatcher: &dyn IThreadDispatcher) {
        for i in 0..self.workers.len() {
            let pool = dispatcher.worker_pool(self.workers[i].worker_index());
            unsafe {
                self.workers[i].reset(pool);
            }
        }
        unsafe {
            ptr::write_volatile(&mut self.padded.stop, false);
            ptr::write_volatile(&mut self.head, ptr::null_mut());
        }
    }

    /// Returns unmanaged resources held by the TaskStack to a pool.
    ///
    /// # Arguments
    /// * `pool` - Buffer pool to return resources to.
    /// * `dispatcher` - Dispatcher whose thread pools should be used.
    pub fn dispose(&mut self, pool: &BufferPool, dispatcher: &dyn IThreadDispatcher) {
        for i in 0..self.workers.len() {
            let worker_pool = dispatcher.worker_pool(self.workers[i].worker_index());
            unsafe {
                self.workers[i].dispose(worker_pool);
            }
        }
        pool.return_buffer(&mut self.workers);
    }

    /// Gets the approximate number of active tasks.
    /// Not guaranteed to measure the true number at any point in time.
    pub fn approximate_task_count(&self) -> i32 {
        let mut sum = 0i32;
        let mut job = unsafe { ptr::read_volatile(&self.head) };
        while !job.is_null() {
            unsafe {
                sum += (*job).counter().max(0);
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
    ///
    /// # Arguments
    /// * `task_count` - Number of tasks associated with the continuation.
    /// * `worker_index` - Worker index to allocate the continuation on.
    /// * `dispatcher` - Dispatcher for per-thread allocations.
    /// * `on_completed` - Function to execute upon completing all tasks (optional).
    pub fn allocate_continuation(
        &mut self,
        task_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Task,
    ) -> ContinuationHandle {
        unsafe {
            self.workers[worker_index].allocate_continuation(task_count, dispatcher, on_completed)
        }
    }

    /// Attempts to pop a task from the stack.
    ///
    /// # Arguments
    /// * `filter` - Filter to apply to jobs.
    ///
    /// # Returns
    /// Tuple of (result status, optional task).
    pub fn try_pop<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
    ) -> (PopTaskResult, Option<Task>) {
        // Note: This implementation does not need locks. We just follow the pointer.
        let mut job = unsafe { ptr::read_volatile(&self.head) };

        loop {
            if job.is_null() {
                // No job to pop from
                let result = if unsafe { ptr::read_volatile(&self.padded.stop) } {
                    PopTaskResult::Stop
                } else {
                    PopTaskResult::Empty
                };
                return (result, None);
            }

            unsafe {
                // Check if this job is allowed by the filter
                if !filter.allow_job((*job).tag) {
                    // Skip to next job
                    job = (*job).previous;
                    continue;
                }

                // Try to pop a task from this job
                if let Some(task) = (*job).try_pop() {
                    debug_assert!(task.function.is_some());
                    return (PopTaskResult::Success, Some(task));
                } else {
                    // No task available, try to remove this empty job from the stack
                    // Other threads might be doing the same; use CAS
                    let previous = (*job).previous;
                    let head_ptr = &self.head as *const *mut Job as *mut *mut Job;
                    let (old, success) = core::intrinsics::atomic_cxchg_acqrel_acquire(head_ptr, job, previous);

                    // If failed, head changed; reload and try again
                    job = if success {
                        ptr::read_volatile(&self.head)
                    } else {
                        old
                    };
                }
            }
        }
    }

    /// Attempts to pop a task without filtering.
    #[inline(always)]
    pub fn try_pop_unfiltered(&self) -> (PopTaskResult, Option<Task>) {
        self.try_pop(&AllowAllJobs)
    }

    /// Attempts to pop a task and run it.
    ///
    /// # Arguments
    /// * `filter` - Filter to apply to jobs.
    /// * `worker_index` - Index of the worker to pass into the task function.
    /// * `dispatcher` - Thread dispatcher running this task stack.
    ///
    /// # Returns
    /// Result status of the pop attempt.
    #[inline(always)]
    pub fn try_pop_and_run<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) -> PopTaskResult {
        let (result, task) = self.try_pop(filter);
        if let Some(task) = task {
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

    /// Pushes a set of tasks onto the task stack. Not thread safe.
    ///
    /// # Safety
    /// Must not be used while other threads could be performing pushes or pops
    /// that could affect the specified worker.
    pub unsafe fn push_unsafely(
        &mut self,
        tasks: &[Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        let job = self.workers[worker_index].allocate_job(tasks, tag, dispatcher);
        (*job).previous = ptr::read_volatile(&self.head);
        ptr::write_volatile(&mut self.head, job);
    }

    /// Pushes a single task onto the task stack. Not thread safe.
    ///
    /// # Safety
    /// Must not be used while other threads could be performing pushes or pops.
    #[inline(always)]
    pub unsafe fn push_unsafely_single(
        &mut self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.push_unsafely(&[task], worker_index, dispatcher, tag);
    }

    /// Pushes a set of tasks onto the task stack (thread-safe).
    ///
    /// # Arguments
    /// * `tasks` - Tasks composing the job.
    /// * `worker_index` - Index of the worker stack to push onto.
    /// * `dispatcher` - Thread dispatcher for allocations.
    /// * `tag` - User-defined tag for the job.
    pub fn push(
        &mut self,
        tasks: &[Task],
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        unsafe {
            let job = self.workers[worker_index].allocate_job(tasks, tag, dispatcher);

            loop {
                // Pre-set the previous pointer so it's visible when the job is swapped in
                (*job).previous = ptr::read_volatile(&self.head);

                // Try to atomically swap in the new job
                let head_ptr = &self.head as *const *mut Job as *mut *mut Job;
                let (_old, success) = core::intrinsics::atomic_cxchgweak_acqrel_relaxed(head_ptr, (*job).previous, job);

                if success {
                    break;
                }
                // If failed, previous pointer is wrong; loop and try again
            }
        }
    }

    /// Pushes a single task onto the task stack (thread-safe).
    #[inline(always)]
    pub fn push_single(
        &mut self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
    ) {
        self.push(&[task], worker_index, dispatcher, tag);
    }

    /// Pushes tasks with a created continuation.
    ///
    /// # Arguments
    /// * `tasks` - Tasks (must not have continuations set).
    /// * `worker_index` - Worker index.
    /// * `dispatcher` - Thread dispatcher.
    /// * `tag` - Job tag.
    /// * `on_complete` - Task to run when all tasks complete (optional).
    ///
    /// # Returns
    /// Handle of the created continuation.
    pub fn allocate_continuation_and_push(
        &mut self,
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
    /// # Arguments
    /// * `filter` - Filter for jobs to work on while waiting.
    /// * `continuation` - Continuation to wait on.
    /// * `worker_index` - Index of the executing worker.
    /// * `dispatcher` - Thread dispatcher.
    pub fn wait_for_completion<TJobFilter: IJobFilter>(
        &self,
        filter: &TJobFilter,
        continuation: ContinuationHandle,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        debug_assert!(
            continuation.initialized(),
            "This codepath should only run if the continuation was allocated earlier."
        );

        let mut spin_count = 0u32;

        while !continuation.completed() {
            let (result, task) = self.try_pop(filter);

            match result {
                PopTaskResult::Stop => return,
                PopTaskResult::Success => {
                    unsafe {
                        task.unwrap().run(worker_index, dispatcher);
                    }
                    spin_count = 0; // Reset spin count after successful work
                }
                PopTaskResult::Empty => {
                    // Adaptive spinning: start with spin_loop, escalate to yield
                    spin_count += 1;
                    if spin_count < 10 {
                        spin_loop();
                    } else if spin_count < 20 {
                        std::thread::yield_now();
                    } else {
                        // Brief sleep to avoid burning CPU
                        std::thread::sleep(std::time::Duration::from_micros(1));
                        spin_count = 15; // Don't let it grow unboundedly
                    }
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
    /// # Arguments
    /// * `tasks` - Tasks (must not have continuations set).
    /// * `worker_index` - Index of the executing worker.
    /// * `dispatcher` - Thread dispatcher.
    /// * `filter` - Filter for jobs to work on while waiting.
    /// * `tag` - Job tag.
    pub fn run_tasks<TJobFilter: IJobFilter>(
        &mut self,
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
            // Only submit tasks beyond the first to the stack.
            // The current thread is responsible for task 0.
            let task_count = (tasks.len() - 1) as i32;

            continuation_handle = unsafe {
                self.workers[worker_index].allocate_continuation(
                    task_count,
                    dispatcher,
                    Task::default(),
                )
            };

            // Set up tasks 1..n with the continuation and push them
            for task in tasks[1..].iter_mut() {
                debug_assert!(
                    !task.continuation.initialized(),
                    "None of the source tasks should have continuations when provided to run_tasks."
                );
                task.continuation = continuation_handle;
            }

            self.push(&tasks[1..], worker_index, dispatcher, tag);
        }

        // Execute task 0 directly on this thread
        let task0 = &tasks[0];
        debug_assert!(
            !task0.continuation.initialized(),
            "None of the source tasks should have continuations when provided to run_tasks."
        );

        unsafe {
            if let Some(func) = task0.function {
                func(task0.id, task0.context, worker_index, dispatcher);
            }
        }

        // If there were more tasks, wait for them to complete
        if tasks.len() > 1 {
            self.wait_for_completion(filter, continuation_handle, worker_index, dispatcher);
        }
    }

    /// Pushes tasks and returns when all are complete (no filtering).
    #[inline(always)]
    pub fn run_tasks_unfiltered(
        &mut self,
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
        &mut self,
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
        &mut self,
        task: Task,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        filter: &TJobFilter,
        tag: u64,
    ) {
        self.run_tasks(&mut [task], worker_index, dispatcher, filter, tag);
    }

    /// Requests that all workers stop.
    /// The next time a worker runs out of tasks, if it sees a stop command, it will stop.
    pub fn request_stop(&self) {
        unsafe {
            let stop_ptr = &self.padded.stop as *const bool as *mut bool;
            ptr::write_volatile(stop_ptr, true);
        }
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

    /// Gets a task that represents a stop request.
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
        &mut self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
        continuation: ContinuationHandle,
    ) {
        // Use stack allocation for small iteration counts
        if iteration_count <= 64 {
            let mut tasks = [Task::default(); 64];
            for i in 0..iteration_count as usize {
                tasks[i] = Task::new(
                    function,
                    context,
                    (inclusive_start_index + i as i32) as i64,
                    continuation,
                );
            }
            self.push_unsafely(&tasks[..iteration_count as usize], worker_index, dispatcher, tag);
        } else {
            // Heap allocate for larger counts
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
    }

    /// Pushes a for loop onto the task stack (thread safe).
    pub fn push_for(
        &mut self,
        function: super::task::TaskFunction,
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
        tag: u64,
        continuation: ContinuationHandle,
    ) {
        // Use stack allocation for small iteration counts
        if iteration_count <= 64 {
            let mut tasks = [Task::default(); 64];
            for i in 0..iteration_count as usize {
                tasks[i] = Task::new(
                    function,
                    context,
                    (inclusive_start_index + i as i32) as i64,
                    continuation,
                );
            }
            self.push(&tasks[..iteration_count as usize], worker_index, dispatcher, tag);
        } else {
            // Heap allocate for larger counts
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
    }

    /// Submits a for loop and returns when all iterations complete.
    pub fn for_loop<TJobFilter: IJobFilter>(
        &mut self,
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

        // Use stack allocation for small iteration counts
        if iteration_count <= 64 {
            let mut tasks = [Task::default(); 64];
            for i in 0..iteration_count as usize {
                tasks[i] = Task::with_context(
                    function,
                    context,
                    (inclusive_start_index + i as i32) as i64,
                );
            }
            self.run_tasks(
                &mut tasks[..iteration_count as usize],
                worker_index,
                dispatcher,
                filter,
                tag,
            );
        } else {
            // Heap allocate for larger counts
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
    }

    /// Submits a for loop and returns when all iterations complete (no filtering).
    #[inline(always)]
    pub fn for_loop_unfiltered(
        &mut self,
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
    /// # Arguments
    /// * `worker_index` - Index of the worker calling this function.
    /// * `dispatcher` - Thread dispatcher responsible for the invocation.
    pub fn dispatch_worker_function(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        let task_stack = unsafe { &*(dispatcher.unmanaged_context() as *const TaskStack) };
        let mut spin_count = 0u32;

        loop {
            match task_stack.try_pop_and_run_unfiltered(worker_index, dispatcher) {
                PopTaskResult::Stop => {
                    // Done!
                    return;
                }
                PopTaskResult::Success => {
                    // If we ran a task, reset spin count - more work may be available
                    spin_count = 0;
                }
                PopTaskResult::Empty => {
                    // No work available, but keep going with adaptive spin
                    spin_count += 1;
                    if spin_count < 10 {
                        spin_loop();
                    } else if spin_count < 20 {
                        std::thread::yield_now();
                    } else {
                        std::thread::sleep(std::time::Duration::from_micros(1));
                        spin_count = 15;
                    }
                }
            }
        }
    }

    /// Dispatches workers to execute tasks from the given stack.
    ///
    /// # Arguments
    /// * `dispatcher` - Thread dispatcher to dispatch workers with.
    /// * `task_stack` - Task stack to pull work from.
    /// * `maximum_worker_count` - Maximum number of workers to spin up.
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
