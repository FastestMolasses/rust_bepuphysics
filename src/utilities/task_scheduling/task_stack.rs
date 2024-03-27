extern crate core;

use crate::utilities::{
    memory::{buffer::Buffer, buffer_pool::BufferPool},
    task_scheduling::{
        continuation_handle::ContinuationHandle, job::Job, job_filter::*,
        pop_task_result::PopTaskResult, task::Task, worker::Worker,
    },
    thread_dispatcher::IThreadDispatcher,
};
// TODO: REMOVE THIS DEPENDENCY LATER
use crossbeam_utils::CachePadded;
use std::{
    ffi::c_void,
    hint::spin_loop,
    ptr::null_mut,
    sync::{
        atomic::{AtomicBool, AtomicPtr, Ordering},
        Arc,
    },
    thread::yield_now,
};

/// Manages a linked stack of tasks.
pub struct TaskStack {
    workers: Buffer<Worker>,
    /// Padding to prevent false sharing. Stop flag is volatile in the original implementation,
    /// here we use an AtomicBool for similar behavior.
    /// TODO: IN THE ORIGINAL CODE, THE PADDING IS SET TO 256 + 16. CONSIDER CHANGING THIS. CachePadded WILL GIVE 64 BYTES ON X86-64.
    /// The extra size might help in scenarios where prefetching mechanisms or specific CPU cache policies could otherwise lead to unintended data locality optimizations that negate the false sharing protections.
    padded_stop: CachePadded<AtomicBool>,
    /// Most recently pushed job on the stack. May be null if the stack is empty.
    head: AtomicPtr<Job>,
}

unsafe impl Send for TaskStack {}
unsafe impl Sync for TaskStack {}

impl TaskStack {
    /// Constructs a new parallel task stack.
    /// # Arguments
    /// * `pool` - Buffer pool to allocate non-thread allocated resources from.
    /// * `dispatcher` - Thread dispatcher to pull thread pools from for thread allocations.
    /// * `worker_count` - Number of workers to allocate space for.
    /// * `initial_worker_job_capacity` - Initial number of jobs (groups of tasks submitted together) to allocate space for in each worker.
    /// * `continuation_block_capacity` - Number of slots to allocate in each block of continuations in each worker.
    pub fn new(
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        worker_count: usize,
        initial_worker_job_capacity: usize,
        continuation_block_capacity: usize,
    ) -> Self {
        let mut workers = pool.take(worker_count);
        for i in 0..worker_count {
            workers[i] = Worker::new(
                i,
                dispatcher,
                initial_worker_job_capacity,
                continuation_block_capacity,
            );
        }
        let padded_stop = CachePadded::new(AtomicBool::new(false));
        let head = AtomicPtr::new(null_mut());

        let mut task_stack = TaskStack {
            workers,
            padded_stop,
            head,
        };
        task_stack.reset(dispatcher);
        task_stack
    }

    /// Returns the stack to a fresh state without reallocating.
    pub fn reset(&mut self, dispatcher: &dyn IThreadDispatcher) {
        for worker in self.workers.iter_mut() {
            worker.reset(&dispatcher.worker_pools(worker.worker_index()));
        }
        self.padded_stop.store(false, Ordering::SeqCst);
        self.head.store(null_mut(), Ordering::SeqCst);
    }

    /// Returns unmanaged resources held by the `TaskStack` to a pool.
    pub fn dispose(&mut self, pool: &mut BufferPool, dispatcher: &dyn IThreadDispatcher) {
        for worker in self.workers.iter_mut() {
            worker.dispose(&dispatcher.worker_pools(worker.worker_index()));
        }
        pool.return_buffer(&mut self.workers);
    }

    /// Gets the approximate number of active tasks. This is not guaranteed to actually measure
    /// the true number of tasks at any one point in time.
    pub fn approximate_task_count(&self) -> usize {
        let mut sum = 0;
        let mut job = self.head.load(Ordering::SeqCst);
        while !job.is_null() {
            unsafe {
                sum += std::cmp::max(0, (*job).counter() as isize) as usize;
                job = (*job).previous();
            }
        }
        sum
    }

    /// Gets the approximate number of active continuations. This is not guaranteed to actually
    /// measure the true number of continuations at any one point in time; it checks each worker
    /// in sequence, and the continuation counts could vary arbitrarily as the checks proceed.
    pub fn approximate_continuation_count(&self) -> usize {
        self.workers.iter().fold(0, |acc, worker| {
            acc + worker.approximate_continuation_count()
        })
    }

    /// Attempts to allocate a continuation for a set of tasks.
    pub fn allocate_continuation(
        &self,
        task_count: usize,
        worker_index: usize,
        dispatcher: &dyn IThreadDispatcher,
        on_completed: Option<Task>,
    ) -> ContinuationHandle {
        self.workers[worker_index].allocate_continuation(task_count, dispatcher, on_completed)
    }

    /// Attempts to pop a task.
    pub fn try_pop<TJobFilter>(&self, filter: &mut TJobFilter) -> (PopTaskResult, Option<Task>)
    where
        TJobFilter: JobFilter,
    {
        let mut job = self.head.load(Ordering::Acquire) as *mut Job;
        loop {
            if job.is_null() {
                // There is no job to pop from.
                return (
                    if self.padded_stop.load(Ordering::Relaxed) {
                        PopTaskResult::Stop
                    } else {
                        PopTaskResult::Empty
                    },
                    None,
                );
            }
            unsafe {
                // Try to pop a task from the current job.
                if !filter.allow_job((*job).tag) {
                    // This job isn't allowed for this pop; go to the next one.
                    job = (*job).previous;
                    continue;
                }
                if let Some(task) = (*job).try_pop() {
                    return (PopTaskResult::Success, Some(task));
                } else {
                    // There was no task available in this job, which means the sampled job should be removed from the stack.
                    let old_head =
                        self.head
                            .compare_and_swap(job, (*job).previous, Ordering::AcqRel);
                    if old_head == job {
                        // If this fails, the head has changed before we could remove it
                        job = self.head.load(Ordering::Acquire);
                    } else {
                        job = old_head;
                    }
                }
            }
        }
    }

    /// Attempts to pop a task and run it.
    pub fn try_pop_and_run<TJobFilter>(
        &self,
        filter: &mut TJobFilter,
        worker_index: usize,
        dispatcher: Arc<dyn IThreadDispatcher>,
    ) -> PopTaskResult
    where
        TJobFilter: JobFilter,
    {
        let (result, task) = self.try_pop(filter);
        if let Some(task) = task {
            task.run(worker_index, dispatcher);
        }
        result
    }

    /// Pushes a set of tasks onto the task stack. This function is not thread safe.
    pub unsafe fn push_unsafely(
        &mut self,
        tasks: &[Task],
        worker_index: usize,
        dispatcher: Arc<dyn IThreadDispatcher>,
        tag: u64,
    ) {
        let job = self.workers[worker_index].allocate_job(tasks, tag, dispatcher);
        (*job).previous = self.head.load(Ordering::Relaxed);
        self.head.store(job, Ordering::Release);
    }

    /// Pushes a set of tasks onto the task stack.
    pub fn push(
        &self,
        tasks: &[Task],
        worker_index: usize,
        dispatcher: Arc<dyn IThreadDispatcher>,
        tag: u64,
    ) {
        unsafe {
            let job = self.workers[worker_index].allocate_job(tasks, tag, dispatcher);
            loop {
                (*job).previous = self.head.load(Ordering::Relaxed);
                if self
                    .head
                    .compare_and_swap((*job).previous, job, Ordering::AcqRel)
                    == (*job).previous
                {
                    break;
                }
            }
        }
    }

    /// Pushes a set of tasks to the stack with a created continuation.
    pub fn allocate_continuation_and_push<T: IThreadDispatcher>(
        &self,
        tasks: &mut [Task],
        worker_index: usize,
        dispatcher: &T,
        tag: u64,
        on_complete: Option<Task>,
    ) -> ContinuationHandle {
        let continuation_handle =
            self.allocate_continuation(tasks.len(), worker_index, dispatcher, on_complete);
        for task in tasks.iter_mut() {
            debug_assert!(
                !task.continuation().initialized(),
                "This function creates a continuation for the tasks"
            );
            task.set_continuation(continuation_handle);
        }
        self.push(tasks, worker_index, dispatcher, tag);
        continuation_handle
    }

    /// Pushes a task to the stack with a created continuation.
    pub fn allocate_continuation_and_push_single<T: IThreadDispatcher>(
        &self,
        task: Task,
        worker_index: usize,
        dispatcher: &T,
        tag: u64,
        on_complete: Option<Task>,
    ) -> ContinuationHandle {
        self.allocate_continuation_and_push(&mut [task], worker_index, dispatcher, tag, on_complete)
    }

    /// Waits for a continuation to be completed.
    pub fn wait_for_completion<T: IThreadDispatcher, F: JobFilter>(
        &self,
        filter: &mut F,
        continuation: ContinuationHandle,
        worker_index: usize,
        dispatcher: &T,
    ) {
        debug_assert!(
            continuation.initialized(),
            "This codepath should only run if the continuation was allocated earlier."
        );
        while !continuation.completed() {
            match self.try_pop(filter) {
                PopTaskResult::Stop => return,
                PopTaskResult::Success(task) => {
                    task.run(worker_index, dispatcher);
                }
                PopTaskResult::Failure => yield_now(), // TODO: LOOK INTO spin_loop
            }
        }
    }

    /// Pushes a set of tasks to the worker stack and returns when all tasks are complete.
    pub fn run_tasks<T: IThreadDispatcher, F: JobFilter>(
        &self,
        tasks: &mut [Task],
        worker_index: usize,
        dispatcher: &T,
        filter: &mut F,
        tag: u64,
    ) {
        if tasks.is_empty() {
            return;
        }
        let mut continuation_handle = ContinuationHandle::default();
        if tasks.len() > 1 {
            let task_count = tasks.len() - 1;
            let mut tasks_to_push = Vec::with_capacity(task_count);
            let worker = &mut self.workers[worker_index];
            continuation_handle = worker.allocate_continuation(task_count, dispatcher);
            for task in tasks.iter_mut().skip(1) {
                debug_assert!(
                    !task.continuation().initialized(),
                    "None of the source tasks should have continuations when provided to run_tasks."
                );
                task.set_continuation(continuation_handle);
                tasks_to_push.push(*task);
            }
            self.push(&tasks_to_push, worker_index, dispatcher, tag);
        }
        // Task 0 execution.
        let task0 = &tasks[0];
        debug_assert!(
            !task0.continuation().initialized(),
            "None of the source tasks should have continuations when provided to run_tasks."
        );
        task0.function()(task0.id(), task0.context(), worker_index, dispatcher);

        if tasks.len() > 1 {
            self.wait_for_completion(filter, continuation_handle, worker_index, dispatcher);
        }
    }

    /// Pushes a set of tasks to the worker stack and returns when all tasks are complete.
    pub fn run_tasks<T: IThreadDispatcher>(
        &self,
        tasks: &mut [Task],
        worker_index: usize,
        dispatcher: &T,
        tag: u64,
    ) {
        let mut filter = AllowAllJobs::default();
        self.run_tasks_with_filter(tasks, worker_index, dispatcher, &mut filter, tag);
    }

    // TODO: THIS IS SUPPOSED TO BE AN OVERLOAD OF run_tasks
    /// Pushes a task to the worker stack and returns when it completes.
    // pub fn run_task<T: IThreadDispatcher>(
    //     &self,
    //     task: Task,
    //     worker_index: usize,
    //     dispatcher: &T,
    //     tag: u64,
    // ) {
    //     self.run_tasks(&mut [task], worker_index, dispatcher, tag);
    // }

    /// Pushes a task to the worker stack and returns when all tasks are complete.
    pub fn run_task_with_filter<T: IThreadDispatcher, F: JobFilter>(
        &self,
        task: Task,
        worker_index: usize,
        dispatcher: &T,
        filter: &mut F,
        tag: u64,
    ) {
        self.run_tasks_with_filter(&mut [task], worker_index, dispatcher, filter, tag);
    }

    /// Requests that all workers stop. The next time a worker runs out of tasks to run, if it sees a stop command, it will be reported.
    pub fn request_stop(&self) {
        self.padded_stop.store(true, Ordering::SeqCst);
    }

    /// Convenience function for requesting a stop. Requires the context to be a pointer to the expected `TaskStack`.
    pub fn request_stop_task_function(
        id: i64,
        context: *mut c_void,
        worker_index: usize,
        dispatcher: &(impl IThreadDispatcher + ?Sized),
    ) {
        unsafe { (*(context as *mut TaskStack)).request_stop() };
    }

    /// Convenience function for getting a task representing a stop request.
    pub fn get_request_stop_task(stack: *mut TaskStack) -> Task {
        Task::new(
            Self::request_stop_task_function as *const (),
            stack as *mut (),
        )
    }

    /// Pushes a for loop onto the task stack. Does not take a lock.
    pub fn push_for_unsafely<T: IThreadDispatcher>(
        &self,
        function: unsafe extern "C" fn(i64, *mut c_void, usize, &T),
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: usize,
        dispatcher: &T,
        tag: u64,
        continuation: Option<ContinuationHandle>,
    ) {
        let mut tasks = Vec::with_capacity(iteration_count as usize);
        for i in 0..iteration_count {
            tasks.push(Task::new(
                function,
                context,
                i + inclusive_start_index as i64,
                continuation,
            ));
        }
        unsafe {
            self.push_unsafely(&tasks, worker_index, dispatcher, tag);
        }
    }

    /// Pushes a for loop onto the task stack.
    pub fn push_for<T: IThreadDispatcher>(
        &self,
        function: unsafe extern "C" fn(i64, *mut c_void, usize, &T),
        context: *mut c_void,
        inclusive_start_index: i32,
        iteration_count: i32,
        worker_index: usize,
        dispatcher: &T,
        tag: u64,
        continuation: Option<ContinuationHandle>,
    ) {
        let mut tasks = Vec::with_capacity(iteration_count as usize);
        for i in 0..iteration_count {
            tasks.push(Task::new(
                function,
                context,
                i + inclusive_start_index as i64,
                continuation,
            ));
        }
        self.push(&tasks, worker_index, dispatcher, tag);
    }

    /// Submits a set of tasks representing a for loop over the given indices and returns when all loop iterations are complete.
    pub fn for_loop<TJobFilter, F>(
        &self,
        function: F,
        context: *mut c_void,
        inclusive_start_index: isize,
        iteration_count: isize,
        worker_index: usize,
        dispatcher: Arc<dyn IThreadDispatcher>,
        filter: &mut TJobFilter,
        tag: u64,
    ) where
        TJobFilter: JobFilter,
        F: Fn(isize, *mut c_void, isize, Arc<dyn IThreadDispatcher>)
            + Send
            + Sync
            + 'static,
    {
        if iteration_count <= 0 {
            return;
        }

        let tasks: Vec<Task> = (0..iteration_count)
            .map(|i| {
                Task::new(Box::new(move || {
                    function(inclusive_start_index + i, context, i, dispatcher.clone())
                }))
            })
            .collect();

        self.run_tasks(tasks, worker_index, dispatcher, filter, tag);
    }

    /// Worker function that pops tasks from the stack and executes them.
    pub fn dispatch_worker_function(worker_index: usize, dispatcher: Arc<dyn IThreadDispatcher>) {
        unsafe {
            let task_stack_ptr = dispatcher.unmanaged_context() as *mut TaskStack;
            let task_stack = &*task_stack_ptr;
            loop {
                match task_stack.try_pop_and_run(worker_index, &dispatcher) {
                    PopTaskResult::Stop => return,
                    PopTaskResult::Success => {}
                    _ => spin_loop(),
                }
            }
        }
    }

    /// Dispatches workers to execute tasks from the given stack.
    pub fn dispatch_workers(
        dispatcher: Arc<dyn IThreadDispatcher>,
        task_stack: Arc<TaskStack>,
        maximum_worker_count: usize,
    ) {
        unsafe {
            dispatcher.dispatch_workers(
                Box::new(move |worker_index, dispatcher| {
                    Self::dispatch_worker_function(worker_index, dispatcher)
                }),
                maximum_worker_count,
                Arc::into_raw(task_stack) as *mut _,
            );
        }
    }
}
