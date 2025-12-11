//! Job definition for grouping tasks in the task scheduling system.
//!
//! A Job represents a collection of tasks submitted together with a common tag.

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::task::Task;

/// Job structure with specific layout for cache optimization.
/// The counter is placed at offset 160 (separate cache line from Tasks/Previous/Tag)
/// to avoid false sharing during concurrent task popping.
///
/// Memory layout (292 bytes total in C#):
/// - Offset 0: Tasks buffer (16 bytes)
/// - Offset 16: Previous pointer (8 bytes)
/// - Offset 24: Tag (8 bytes)
/// - Offset 32-159: Padding (128 bytes for cache line separation)
/// - Offset 160: Counter (4 bytes)
/// - Offset 164-291: Padding (128 bytes)
#[repr(C)]
pub struct Job {
    /// Buffer containing the tasks.
    pub tasks: Buffer<Task>,
    /// Pointer to the previous job in the stack (for linked-list traversal).
    pub previous: *mut Job,
    /// User-defined tag for job filtering.
    pub tag: u64,
    /// Padding to isolate counter on its own cache line (128 bytes to offset 160).
    _padding1: [u8; 128],
    /// Counter for remaining tasks. Modified using Interlocked operations.
    pub counter: i32,
    /// Trailing padding for full cache line isolation.
    _padding2: [u8; 128],
}

// Verify the layout is correct
const _: () = {
    // Counter should be at offset 160 for cache line isolation
    assert!(std::mem::offset_of!(Job, counter) == 160);
    // Size should be 292 bytes to match C# layout
    assert!(std::mem::size_of::<Job>() == 292);
};

// Safety: Job is designed for cross-thread access with atomic counter
unsafe impl Send for Job {}
unsafe impl Sync for Job {}

impl Job {
    /// Creates a new job from a slice of tasks.
    ///
    /// The job and task buffer are allocated together in one contiguous block
    /// to minimize allocations and improve cache locality.
    ///
    /// # Safety
    /// - The pool must have sufficient memory available
    /// - The source_tasks slice must not be empty
    pub unsafe fn create(source_tasks: &[Task], tag: u64, pool: &BufferPool) -> *mut Job {
        debug_assert!(
            !source_tasks.is_empty(),
            "Probably shouldn't be trying to push zero tasks."
        );

        // Allocate memory for both the job and the tasks buffer in one block
        let size_to_allocate =
            std::mem::size_of::<Job>() + source_tasks.len() * std::mem::size_of::<Task>();

        let raw_buffer: Buffer<u8> = pool.take(size_to_allocate as i32);
        let job_ptr = raw_buffer.as_ptr() as *mut Job;

        // Set up the tasks buffer to point just after the job struct
        let tasks_ptr = (raw_buffer.as_ptr() as *mut u8).add(std::mem::size_of::<Job>()) as *mut Task;

        (*job_ptr).tasks = Buffer::new(tasks_ptr, source_tasks.len() as i32, raw_buffer.id());
        (*job_ptr).tag = tag;

        // Copy tasks into the buffer
        std::ptr::copy_nonoverlapping(source_tasks.as_ptr(), tasks_ptr, source_tasks.len());

        // Initialize counter to task count
        (*job_ptr).counter = source_tasks.len() as i32;
        (*job_ptr).previous = std::ptr::null_mut();

        job_ptr
    }

    /// Attempts to pop a task from the job.
    ///
    /// Uses Interlocked.Decrement equivalent for thread-safe task claiming.
    ///
    /// # Returns
    /// Some(task) if a task was available, None otherwise.
    #[inline(always)]
    pub unsafe fn try_pop(&mut self, task: &mut Task) -> bool {
        // Interlocked.Decrement equivalent - returns the NEW value after decrement
        let counter_ptr = &mut self.counter as *mut i32;
        let new_count = core::intrinsics::atomic_xsub_acqrel(counter_ptr, 1) - 1;

        if new_count >= 0 {
            *task = *self.tasks.get_unchecked(new_count);
            debug_assert!(task.function.is_some(), "Task function should not be null");
            true
        } else {
            // We went negative, meaning no tasks were available
            false
        }
    }

    /// Disposes of the job, returning its memory to the pool.
    ///
    /// # Safety
    /// - The pool must be the same pool used to allocate this job
    /// - The job must not be accessed after disposal
    #[inline(always)]
    pub unsafe fn dispose(&mut self, pool: &BufferPool) {
        // The instance is allocated from the same memory as the tasks buffer,
        // so disposing it returns the Job memory too.
        let id = self.tasks.id();
        // Zero out to help catch use-after-free in debug
        *self = std::mem::zeroed();
        pool.return_unsafely(id);
    }
}
