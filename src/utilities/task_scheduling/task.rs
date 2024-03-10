use std::{
    cell::{Ref, RefCell},
    fmt::Debug,
    marker::PhantomData,
    mem::ManuallyDrop,
    rc::Rc,
};

use crate::{
    collections::{QuickList, RemovePtr},
    memory::{Buffer, BufferPool},
};

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Task {
    /// Function to be executed by the task. Takes as arguments the `Id`, `Context` pointer, and executing worker index.
    pub function: extern "C" fn(long, *mut c_void, i32, IThreadDispatcher) -> (),
    /// Context to be passed into the `Function`.
    pub context: *mut c_void,
    /// Continuation to be notified after this task completes, if any.
    pub continuation: ContinuationHandle,
    /// User-provided identifier of this task.
    pub id: long,
}

/// Description of a task to be submitted to a `TaskStack`.
impl Task {
    /// Creates a new task.
    ///
    /// # Arguments
    /// * `function`: Function to be executed by the task. Takes as arguments the `Id`, `Context` pointer, executing worker index, and executing `<IThreadDispatcher>`.
    /// * `context`: Context pointer to pass into the `Function`.
    /// * `taskId`: Id of this task to be passed into the `Function`.
    /// * `continuation`: Continuation to notify after the completion of this task, if any.
    pub fn new(function: extern "C" fn(long, *mut c_void, i32, IThreadDispatcher) -> (), context: *mut c_void, taskId: long, continuation: ContinuationHandle) -> Self {
        Self { function, context, continuation, id: taskId }
    }

    /// Creates a task from a function.
    ///
    /// # Arguments
    /// * `function`: Function to turn into a task.
    pub fn from_fn(function: extern "C" fn(long, *mut c_void, i32, IThreadDispatcher) -> ()) -> Self {
        Self { function, context: std::ptr::null_mut(), continuation: ContinuationHandle::null(), id: 0 }
    }

    /// Runs the task and, if necessary, notifies the associated continuation of its completion.
    ///
    /// # Arguments
    /// * `workerIndex`: Worker index to pass to the function.
    /// * `dispatcher`: Dispatcher running this task.
    pub fn run(&self, workerIndex: i32, dispatcher: &IThreadDispatcher) {
        debug_assert!(!self.continuation.completed && self.function != std::ptr::null_mut());
        (self.function)(self.id, self.context, workerIndex, dispatcher);
        if self.continuation.initialized() {
            self.continuation.notify_task_completed(workerIndex, dispatcher);
        }
    }
}

// This is a bit odd. We're presenting this pointer as a handle, even though it's not.
// Hiding the implementation detail makes it a little easier to change later if we need to.
#[repr(transparent)]
pub struct ContinuationHandle(*mut c_void);

/// Refers to a continuation within a `<TaskStack>`.
impl ContinuationHandle {
    /// Gets whether the tasks associated with this continuation have completed. If the continuation has not been initialized, this will always return false.
    pub fn completed(&self) -> bool {
        self.initialized() && self.remaining_task_counter() <= 0
    }

    /// Retrieves a pointer to the continuation data for `<ContinuationHandle>`.
    ///
    /// # Returns
    /// Pointer to the continuation backing the given handle.
    ///
    /// # Remarks
    /// This should not be used if the continuation handle is not known to be valid. The data pointed to by the data could become invalidated if the continuation completes.
    pub fn continuation(&self) -> *mut c_void {
        self.0
    }

    /// Gets a null continuation handle.
    pub fn null() -> Self {
        Self(std::ptr::null_mut())
    }

    /// Gets whether this handle ever represented an allocated handle. This does not guarantee that the continuation's associated tasks are active in the `<TaskStack>` that it was allocated from.
    pub fn initialized(&self) -> bool {
        self.0 != std::ptr::null_mut()
    }

    /// Notifies the continuation that one task was completed.
    ///
    /// # Arguments
    /// * `workerIndex`: Worker index to pass to the continuation's delegate, if any.
    /// * `dispatcher`: Dispatcher to pass to the continuation's delegate, if any.
    pub fn notify_task_completed(&self, workerIndex: i32, dispatcher: &IThreadDispatcher) {
        let continuation = unsafe { &mut *(self.0 as *mut TaskContinuation) };
        debug_assert!(!self.completed());
        let counter = continuation.remaining_task_counter.fetch_sub(1, Ordering::Relaxed);
        debug_assert!(counter >= 0, "The counter should not go negative. Was notify called too many times?");
        if counter == 0 {
            // This entire job has completed.
            if !continuation.on_completed.function.is_null() {
                (continuation.on_completed.function)(continuation.on_completed.id, continuation.on_completed.context, workerIndex, dispatcher);
            }
        }
    }

    fn remaining_task_counter(&self) -> i32 {
        let continuation = unsafe { &mut *(self.0 as *mut TaskContinuation) };
        continuation.remaining_task_counter.load(Ordering::Relaxed)
    }
}

impl PartialEq for ContinuationHandle {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Eq for ContinuationHandle {}

impl Debug for ContinuationHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ContinuationHandle {{ 0x{:X} }}", self.0 as usize)
    }
}

/// Describes the result status of a pop attempt.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PopTaskResult {
    /// A task was successfully popped.
    Success,
    /// The stack was empty, but may have more tasks in the future.
    Empty,
    /// The stack has been terminated and all threads seeking work should stop.
    Stop,
}

/// Worker function that pops tasks from the stack and executes them.
///
/// # Arguments
/// * `workerIndex`: Index of the worker calling this function.
/// * `dispatcher`: Thread dispatcher responsible for the invocation.
#[no_mangle]
pub extern "C" fn dispatch_worker_function(workerIndex: i32, dispatcher: &IThreadDispatcher) {
    // TODO: INCOMPLETE FUNCTION, CHECK ORIGINAL IMPLEMENTATION
    let task_stack = unsafe { &mut *(&mut dispatcher.task_stack as *mut TaskStack) };
    let mut task = Task::new(std::ptr::null_mut(), std::ptr::null_mut(), 0, ContinuationHandle::null());
    loop {
        match task_stack.pop(&mut task) {
            PopTaskResult::Success => {
                task.run(workerIndex, dispatcher);
            }
            PopTaskResult::Empty => {
                // No work to do, so we'll just wait for more.
                dispatcher.wait_for_work();
            }
            PopTaskResult::Stop => {
                // The stack has been terminated, so we should stop.
                break;
            }
        }
    }
}
