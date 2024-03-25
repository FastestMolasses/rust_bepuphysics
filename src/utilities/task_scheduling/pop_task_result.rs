pub enum PopTaskResult {
    /// A task was successfully popped.
    Success,
    /// The stack was empty, but may have more tasks in the future.
    Empty,
    /// The stack has been terminatd and all threads seeking work should stop.
    Stop,
}
