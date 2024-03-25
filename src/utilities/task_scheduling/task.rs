#![allow(unsafe_code)]

extern crate core;

use crate::utilities::task_scheduling::continuation_handle::ContinuationHandle;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use core::ffi::c_void;

pub struct Task {
    /// Function to be executed by the task. Takes as arguments the task id, context pointer,
    /// executing worker index, and executing thread dispatcher.
    pub function: Option<unsafe extern "C" fn(i64, *mut c_void, usize, &dyn IThreadDispatcher)>,
    /// Context to be passed into the function.
    pub context: *mut c_void,
    /// Continuation to be notified after this task completes, if any.
    pub continuation: ContinuationHandle,
    /// User-provided identifier of this task.
    pub id: i64,
}

impl Task {
    /// Creates a new task.
    pub fn new(
        function: unsafe extern "C" fn(i64, *mut c_void, usize, &dyn IThreadDispatcher),
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

    /// Runs the task and, if necessary, notifies the associated continuation of its completion.
    pub fn run(&self, worker_index: usize, dispatcher: &dyn IThreadDispatcher) {
        assert!(!self.continuation.completed && self.function.is_some());
        unsafe {
            // TODO: REMOVE UNWRAP OR HANDLE ERROR
            self.function.unwrap()(self.id, self.context, worker_index, dispatcher);
        }
        if self.continuation.initialized {
            unsafe {
                self.continuation
                    .notify_task_completed(worker_index, dispatcher);
            }
        }
    }
}

impl From<unsafe extern "C" fn(i64, *mut c_void, usize, &dyn IThreadDispatcher)> for Task {
    fn from(
        function: unsafe extern "C" fn(i64, *mut c_void, usize, &dyn IThreadDispatcher),
        continuation_handle: ContinuationHandle,
    ) -> Self {
        Task::new(function, std::ptr::null_mut(), 0, continuation_handle)
    }
}
