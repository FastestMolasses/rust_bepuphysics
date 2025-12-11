//! Result status of a pop attempt from the TaskStack.

/// Describes the result status of a pop attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PopTaskResult {
    /// A task was successfully popped.
    Success = 0,
    /// The stack was empty, but may have more tasks in the future.
    Empty = 1,
    /// The stack has been terminated and all threads seeking work should stop.
    Stop = 2,
}
