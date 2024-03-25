#![allow(unsafe_code)]

use crate::utilities::task_scheduling::task_continuation::TaskContinuation;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::ptr::NonNull;

/// Represents a continuation within a TaskStack.
pub struct ContinuationHandle {
    continuation: Option<NonNull<TaskContinuation>>,
}

impl ContinuationHandle {
    /// Creates a new `ContinuationHandle` wrapping a `TaskContinuation`.
    pub unsafe fn new(continuation: *mut TaskContinuation) -> Self {
        Self {
            continuation: NonNull::new(continuation),
        }
    }

    /// Returns whether the tasks associated with this continuation have completed.
    /// If the continuation has not been initialized, this will always return false.
    pub fn completed(&self) -> bool {
        self.initialized()
            && unsafe { self.continuation.unwrap().as_ref().remaining_task_counter <= 0 }
    }

    /// Retrieves a pointer to the continuation data for `ContinuationHandle`.
    /// This should not be used if the continuation handle is not known to be valid.
    pub unsafe fn continuation(&self) -> *const TaskContinuation {
        self.continuation.unwrap().as_ptr()
    }

    /// Returns a null continuation handle.
    pub fn null() -> Self {
        Self { continuation: None }
    }

    /// Returns whether this handle ever represented an allocated handle.
    pub fn initialized(&self) -> bool {
        self.continuation.is_some()
    }

    /// Notifies the continuation that one task was completed.
    pub unsafe fn notify_task_completed(
        &self,
        worker_index: usize,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let continuation = self.continuation.unwrap().as_ptr();
        let remaining_task_counter = &mut (*continuation).remaining_task_counter as *mut isize;
        *remaining_task_counter -= 1;
        debug_assert!(
            *remaining_task_counter >= 0,
            "The counter should not go negative. Was notify called too many times?"
        );
        if *remaining_task_counter == 0 {
            let on_completed = &(*continuation).on_completed;
            if let Some(function) = on_completed.function {
                function(
                    on_completed.id,
                    on_completed.context,
                    worker_index,
                    dispatcher,
                );
            }
        }
    }
}

impl PartialEq for ContinuationHandle {
    fn eq(&self, other: &Self) -> bool {
        self.continuation == other.continuation
    }
}

impl Eq for ContinuationHandle {}

impl std::hash::Hash for ContinuationHandle {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        std::ptr::hash(self.continuation.unwrap().as_ptr(), state);
    }
}
