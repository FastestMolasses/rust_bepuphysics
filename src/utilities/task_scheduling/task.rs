//! Task definition for the task scheduling system.
//!
//! Tasks represent units of work that can be executed by worker threads.

use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::ffi::c_void;

use super::continuation_handle::ContinuationHandle;

/// Function pointer type for task execution.
/// Takes as arguments: task id, context pointer, worker index, and dispatcher reference.
pub type TaskFunction = unsafe fn(i64, *mut c_void, i32, &dyn IThreadDispatcher);

/// Description of a task to be submitted to a TaskStack.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Task {
    /// Function to be executed by the task. Takes as arguments the Id, Context pointer,
    /// executing worker index, and dispatcher.
    pub function: Option<TaskFunction>,
    /// Context to be passed into the Function.
    pub context: *mut c_void,
    /// Continuation to be notified after this task completes, if any.
    pub continuation: ContinuationHandle,
    /// User-provided identifier of this task.
    pub id: i64,
}

// Safety: Task contains raw pointers but is designed for cross-thread use
// in a controlled environment where the user ensures pointer validity.
unsafe impl Send for Task {}
unsafe impl Sync for Task {}

impl Default for Task {
    #[inline(always)]
    fn default() -> Self {
        Self {
            function: None,
            context: std::ptr::null_mut(),
            continuation: ContinuationHandle::default(),
            id: 0,
        }
    }
}

impl Task {
    /// Creates a new task.
    ///
    /// # Arguments
    /// * `function` - Function to be executed by the task.
    /// * `context` - Context pointer to pass to the function.
    /// * `task_id` - Id of this task to be passed into the function.
    /// * `continuation` - Continuation to notify after the completion of this task, if any.
    #[inline(always)]
    pub fn new(
        function: TaskFunction,
        context: *mut c_void,
        task_id: i64,
        continuation: ContinuationHandle,
    ) -> Self {
        Self {
            function: Some(function),
            context,
            continuation,
            id: task_id,
        }
    }

    /// Creates a new task with just a function.
    #[inline(always)]
    pub fn from_function(function: TaskFunction) -> Self {
        Self {
            function: Some(function),
            context: std::ptr::null_mut(),
            continuation: ContinuationHandle::default(),
            id: 0,
        }
    }

    /// Creates a task from a function with context and id.
    #[inline(always)]
    pub fn with_context(function: TaskFunction, context: *mut c_void, task_id: i64) -> Self {
        Self {
            function: Some(function),
            context,
            continuation: ContinuationHandle::default(),
            id: task_id,
        }
    }

    /// Runs the task and, if necessary, notifies the associated continuation of its completion.
    ///
    /// # Arguments
    /// * `worker_index` - Worker index to pass to the function.
    /// * `dispatcher` - Dispatcher running this task.
    ///
    /// # Safety
    /// The function pointer and context must be valid.
    #[inline(always)]
    pub unsafe fn run(&self, worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        debug_assert!(
            !self.continuation.completed() && self.function.is_some(),
            "Task must have a valid function and non-completed continuation"
        );

        // Execute the task function
        if let Some(func) = self.function {
            func(self.id, self.context, worker_index, dispatcher);
        }

        // Notify continuation if initialized
        if self.continuation.initialized() {
            self.continuation.notify_task_completed(worker_index, dispatcher);
        }
    }
}

impl From<TaskFunction> for Task {
    #[inline(always)]
    fn from(function: TaskFunction) -> Self {
        Task::from_function(function)
    }
}
