//! Job definition for grouping tasks in the task scheduling system.
//!
//! A Job represents a collection of tasks submitted together with a common tag.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::task::Task;

/// Job structure with specific layout for cache optimization.
/// The counter is placed at offset 160 (separate cache line from Tasks/Previous/Tag)
/// to avoid false sharing during concurrent task popping.
///
/// Memory layout (292 bytes total, matching C# StructLayout):
/// - Offset 0: Tasks buffer (16 bytes)
/// - Offset 16: Previous pointer (8 bytes)
/// - Offset 24: Tag (8 bytes)
/// - Offset 32-159: Padding (128 bytes for cache line separation)
/// - Offset 160: Counter (4 bytes, UnsafeCell<i32> is repr(transparent) over i32)
/// - Offset 164-291: Padding (128 bytes)
#[repr(C)]
pub struct Job {
    /// Buffer containing the tasks.
    pub tasks: Buffer<Task>,
    /// Pointer to the previous job in the stack (for linked-list traversal).
    pub previous: *mut Job,
    /// User-defined tag for job filtering.
    pub tag: u64,
    /// Padding to isolate counter on its own cache line.
    _padding1: [u8; 128],
    /// Counter for remaining tasks. Plain i32 (matching C#), with atomic operations
    /// performed explicitly via `AtomicI32::from_ptr()` only where C# uses `Interlocked`.
    /// UnsafeCell enables mutation through shared references.
    pub counter: UnsafeCell<i32>,
    /// Trailing padding for full cache line isolation.
    _padding2: [u8; 128],
}

// Verify the layout matches C# at compile time.
// Note: C# uses [StructLayout(Size = 292)], but Rust's repr(C) rounds up to
// 8-byte alignment = 296. The extra 4 bytes of trailing padding are harmless;
// the critical constraint is that `counter` is at offset 160 for cache isolation.
const _: () = {
    assert!(std::mem::offset_of!(Job, counter) == 160);
    assert!(std::mem::size_of::<Job>() == 296);
};

// Safety: Job is designed for cross-thread access with atomic counter.
unsafe impl Send for Job {}
unsafe impl Sync for Job {}

impl Job {
    /// Creates a new job from a slice of tasks.
    ///
    /// The job and task buffer are allocated together in one contiguous block
    /// to minimize allocations and improve cache locality.
    ///
    /// # Safety
    /// - The pool must have sufficient memory available.
    /// - The source_tasks slice must not be empty.
    /// - Caller must have exclusive access to the pool.
    pub unsafe fn create(source_tasks: &[Task], tag: u64, pool: &mut BufferPool) -> *mut Job {
        debug_assert!(
            !source_tasks.is_empty(),
            "Probably shouldn't be trying to push zero tasks."
        );

        // Note that the job and the buffer of tasks are allocated together as one block.
        // This ensures we only need to perform one allocation.
        let size_to_allocate =
            std::mem::size_of::<Job>() + source_tasks.len() * std::mem::size_of::<Task>();

        let raw_buffer: Buffer<u8> = pool.take(size_to_allocate as i32);
        let job_ptr = raw_buffer.as_ptr() as *mut Job;

        // Set up the tasks buffer to point just after the job struct
        let tasks_ptr =
            (raw_buffer.as_ptr() as *mut u8).add(std::mem::size_of::<Job>()) as *mut Task;

        // Initialize fields through raw pointer writes to avoid creating references
        // to uninitialized memory.
        std::ptr::addr_of_mut!((*job_ptr).tasks).write(Buffer::new(
            tasks_ptr,
            source_tasks.len() as i32,
            raw_buffer.id(),
        ));
        std::ptr::addr_of_mut!((*job_ptr).tag).write(tag);
        std::ptr::addr_of_mut!((*job_ptr).previous).write(std::ptr::null_mut());
        std::ptr::addr_of_mut!((*job_ptr)._padding1).write([0u8; 128]);
        std::ptr::addr_of_mut!((*job_ptr).counter)
            .write(UnsafeCell::new(source_tasks.len() as i32));
        std::ptr::addr_of_mut!((*job_ptr)._padding2).write([0u8; 128]);

        // Copy tasks into the buffer
        std::ptr::copy_nonoverlapping(source_tasks.as_ptr(), tasks_ptr, source_tasks.len());

        job_ptr
    }

    /// Attempts to pop a task from the job.
    ///
    /// Uses atomic decrement for thread-safe task claiming, matching C#'s
    /// `Interlocked.Decrement(ref Counter)`.
    ///
    /// # Safety
    /// The job must be valid and its tasks buffer must be properly initialized.
    ///
    /// # Returns
    /// `true` if a task was available and written to `task`, `false` otherwise.
    #[inline(always)]
    pub unsafe fn try_pop(&self, task: &mut Task) -> bool {
        // Interlocked.Decrement(ref Counter): fetch_sub returns OLD value, subtract 1 for new.
        let old_count = AtomicI32::from_ptr(self.counter.get()).fetch_sub(1, Ordering::AcqRel);
        let new_count = old_count - 1;

        if new_count >= 0 {
            *task = *self.tasks.as_ptr().add(new_count as usize);
            debug_assert!(task.function.is_some(), "Task function should not be null");
            true
        } else {
            // We went negative, meaning no tasks were available.
            false
        }
    }

    /// Disposes of the job, returning its memory to the pool.
    ///
    /// # Safety
    /// - The pool must be the same pool used to allocate this job.
    /// - The job must not be accessed after disposal.
    /// - Caller must have exclusive access to the pool.
    #[inline(always)]
    pub unsafe fn dispose(&mut self, pool: &mut BufferPool) {
        // The instance is allocated from the same memory as the tasks buffer,
        // so disposing it returns the Job memory too.
        let id = self.tasks.id();
        // Zero out to help catch use-after-free in debug
        *self = std::mem::zeroed();
        pool.return_unsafely(id);
    }
}
