use crate::utilities::task_scheduling::task::Task;

pub struct TaskContinuation {
    /// Task to run upon completion of the associated task.
    on_completed: Task,
    /// Number of tasks not yet reported as complete in the continuation.
    remaining_task_counter: usize,
}
