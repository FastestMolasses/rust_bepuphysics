//! Handle to a continuation within a TaskStack.
//!
//! ContinuationHandle provides a wrapper around a pointer to TaskContinuation.

use std::sync::atomic::{AtomicI32, Ordering};

use crate::utilities::thread_dispatcher::IThreadDispatcher;

use super::task_continuation::TaskContinuation;

/// Refers to a continuation within a TaskStack.
///
/// This is a thin wrapper around a pointer to TaskContinuation, presenting it as a handle
/// to hide implementation details and make future changes easier.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct ContinuationHandle {
    continuation: *mut TaskContinuation,
}

// Safety: ContinuationHandle is designed for cross-thread use.
// The pointer is only dereferenced when known to be valid.
unsafe impl Send for ContinuationHandle {}
unsafe impl Sync for ContinuationHandle {}

impl Default for ContinuationHandle {
    #[inline(always)]
    fn default() -> Self {
        Self {
            continuation: std::ptr::null_mut(),
        }
    }
}

impl ContinuationHandle {
    /// Creates a new ContinuationHandle wrapping a TaskContinuation pointer.
    #[inline(always)]
    pub(crate) fn new(continuation: *mut TaskContinuation) -> Self {
        Self { continuation }
    }

    /// Gets a null continuation handle.
    #[inline(always)]
    pub fn null() -> Self {
        Self::default()
    }

    /// Gets whether this handle ever represented an allocated handle.
    /// This does not guarantee that the continuation's associated tasks are
    /// active in the TaskStack that it was allocated from.
    #[inline(always)]
    pub fn initialized(&self) -> bool {
        !self.continuation.is_null()
    }

    /// Gets whether the tasks associated with this continuation have completed.
    /// If the continuation has not been initialized, this will always return false.
    #[inline(always)]
    pub fn completed(&self) -> bool {
        // C#: continuation->RemainingTaskCounter <= 0 (plain read, NOT volatile/Interlocked)
        // Relaxed = plain load on all architectures, avoids Rust data-race UB.
        self.initialized()
            && unsafe {
                AtomicI32::from_ptr((*self.continuation).remaining_task_counter.get())
                    .load(Ordering::Relaxed)
                    <= 0
            }
    }

    /// Retrieves a pointer to the continuation data.
    ///
    /// # Safety
    /// This should not be used if the continuation handle is not known to be valid.
    /// The data pointed to could become invalidated if the continuation completes.
    #[inline(always)]
    pub unsafe fn continuation(&self) -> *mut TaskContinuation {
        self.continuation
    }

    /// Notifies the continuation that one task was completed.
    ///
    /// # Safety
    /// The continuation must be valid and initialized.
    #[inline(always)]
    pub unsafe fn notify_task_completed(
        &self,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let continuation = self.continuation;
        debug_assert!(!self.completed());

        // Interlocked.Decrement(ref continuation->RemainingTaskCounter)
        // fetch_sub returns OLD value; new = old - 1.
        let old_count = AtomicI32::from_ptr((*continuation).remaining_task_counter.get())
            .fetch_sub(1, Ordering::AcqRel);
        let new_count = old_count - 1;

        debug_assert!(
            new_count >= 0,
            "The counter should not go negative. Was notify called too many times?"
        );

        if new_count == 0 {
            // This entire job has completed.
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
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.continuation == other.continuation
    }
}

impl Eq for ContinuationHandle {}

impl std::hash::Hash for ContinuationHandle {
    #[inline(always)]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        std::ptr::hash(self.continuation, state);
    }
}
