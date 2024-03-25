use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::task::Task;
use std::ptr::{self, NonNull};

/// Represents a job consisting of multiple tasks.
#[repr(C)]
pub struct Job {
    tasks: Buffer<Task>,
    previous: *mut Job,
    tag: u64,
    counter: isize, // Using isize for simplicity in arithmetic operations, closely aligns with the original use of `int`.
}

impl Job {
    /// Creates a new job from a slice of tasks.
    pub unsafe fn create(source_tasks: &[Task], tag: u64, pool: &BufferPool) -> NonNull<Job> {
        let size_to_allocate =
            std::mem::size_of::<Job>() + source_tasks.len() * std::mem::size_of::<Task>();
        let raw_buffer = pool.take::<u8>(size_to_allocate);
        let job_ptr = raw_buffer.as_ptr() as *mut Job;
        (*job_ptr).tasks = Buffer::new(
            raw_buffer.offset(std::mem::size_of::<Job>() as isize),
            source_tasks.len(),
            raw_buffer.id(),
        );
        (*job_ptr).tag = tag;
        ptr::copy_nonoverlapping(
            source_tasks.as_ptr(),
            (*job_ptr).tasks.as_mut_ptr(),
            source_tasks.len(),
        );
        (*job_ptr).counter = source_tasks.len() as isize;
        (*job_ptr).previous = ptr::null_mut();
        NonNull::new_unchecked(job_ptr)
    }

    /// Attempts to pop a task from the job.
    pub unsafe fn try_pop(&mut self) -> Option<Task> {
        let new_count = self.counter - 1;
        if new_count >= 0 {
            self.counter = new_count;
            Some(self.tasks[new_count as usize])
        } else {
            None
        }
    }

    /// Disposes of the job, returning its memory to the pool.
    pub unsafe fn dispose(self, pool: &BufferPool) {
        let id = self.tasks.id();
        std::mem::forget(self); // Prevents Rust from automatically dropping the Job which we manually manage.
        pool.return_unsafely(id);
    }
}
