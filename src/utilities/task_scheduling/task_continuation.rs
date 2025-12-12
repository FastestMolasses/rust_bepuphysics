//! Task continuation for tracking completion of related tasks.
//!
//! Continuations allow grouping tasks and executing a callback when all complete.

use std::cell::UnsafeCell;

use super::task::Task;

/// Stores data relevant to tracking task completion and reporting completion for a continuation.
#[repr(C)]
pub struct TaskContinuation {
    /// Task to run upon completion of the associated tasks.
    pub on_completed: Task,
    /// Number of tasks not yet reported as complete in the continuation.
    /// Plain i32 (matching C#), with atomic operations performed explicitly
    /// via `AtomicI32::from_ptr()` only where C# uses `Interlocked`.
    pub remaining_task_counter: UnsafeCell<i32>,
}

impl Default for TaskContinuation {
    #[inline(always)]
    fn default() -> Self {
        Self {
            on_completed: Task::default(),
            remaining_task_counter: UnsafeCell::new(0),
        }
    }
}
