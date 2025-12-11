//! Task scheduling system for parallel physics simulation.
//!
//! This module provides a lock-free work-stealing task scheduler optimized for
//! cache locality and minimal synchronization overhead. It includes:
//!
//! - `TaskStack`: The main task queue with lock-free push/pop operations
//! - `Task`: A unit of work with optional continuation support
//! - `Job`: A group of tasks submitted together
//! - `ContinuationHandle`: Track completion of task groups
//! - `IJobFilter`: Filter which jobs can be popped based on tags

mod continuation_block;
mod continuation_handle;
mod job;
mod job_filter;
mod pop_task_result;
mod task;
mod task_continuation;
mod task_stack;
mod worker;

// Re-export public API
pub use continuation_handle::ContinuationHandle;
pub use job_filter::{AllowAllJobs, DelegateJobFilter, ExactTagFilter, FunctionPointerJobFilter, IJobFilter, MinimumTagFilter};
pub use pop_task_result::PopTaskResult;
pub use task::{Task, TaskFunction};
pub use task_stack::TaskStack;

// Internal but exposed for advanced use
pub use task_continuation::TaskContinuation;
