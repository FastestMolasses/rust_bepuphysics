//! Task continuation for tracking completion of related tasks.
//!
//! Continuations allow grouping tasks and executing a callback when all complete.

use super::task::Task;

/// Stores data relevant to tracking task completion and reporting completion for a continuation.
#[repr(C)]
pub struct TaskContinuation {
    /// Task to run upon completion of the associated tasks.
    pub on_completed: Task,
    /// Number of tasks not yet reported as complete in the continuation.
    /// This is modified atomically by workers as tasks complete.
    pub remaining_task_counter: i32,
}

impl Default for TaskContinuation {
    #[inline(always)]
    fn default() -> Self {
        Self {
            on_completed: Task::default(),
            remaining_task_counter: 0,
        }
    }
}
