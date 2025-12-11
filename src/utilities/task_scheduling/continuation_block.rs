//! Block storage for task continuations.
//!
//! ContinuationBlocks form a linked list to allow growing continuation storage
//! without reallocation (which would introduce race conditions).

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::task_continuation::TaskContinuation;

/// Stores a block of task continuations that maintains a pointer to previous blocks.
#[repr(C)]
pub struct ContinuationBlock {
    /// Pointer to the previous block in the linked list.
    pub previous: *mut ContinuationBlock,
    /// Number of continuations allocated in this block.
    pub count: i32,
    /// Buffer holding the continuation data.
    pub continuations: Buffer<TaskContinuation>,
}

impl ContinuationBlock {
    /// Creates a new block of task continuations.
    ///
    /// # Safety
    /// The pool must have sufficient memory available.
    pub unsafe fn create(continuation_capacity: i32, pool: &BufferPool) -> *mut ContinuationBlock {
        // Allocate memory for both the block header and continuations in one allocation
        let total_size = std::mem::size_of::<ContinuationBlock>()
            + std::mem::size_of::<TaskContinuation>() * continuation_capacity as usize;

        let raw_buffer: Buffer<u8> = pool.take(total_size as i32);
        let block = raw_buffer.as_ptr() as *mut ContinuationBlock;

        // Set up the continuations buffer to point just after the block header
        let continuations_ptr = (raw_buffer.as_ptr() as *mut u8)
            .add(std::mem::size_of::<ContinuationBlock>())
            as *mut TaskContinuation;

        (*block).continuations =
            Buffer::new(continuations_ptr, continuation_capacity, raw_buffer.id());
        (*block).count = 0;
        (*block).previous = std::ptr::null_mut();

        block
    }

    /// Tries to allocate a continuation from this block.
    ///
    /// # Returns
    /// Some(pointer) if allocation succeeded, None if the block is full.
    #[inline(always)]
    pub unsafe fn try_allocate_continuation(&mut self) -> Option<*mut TaskContinuation> {
        if self.count < self.continuations.len() {
            let continuation = self.continuations.as_mut_ptr().add(self.count as usize);
            self.count += 1;
            Some(continuation)
        } else {
            None
        }
    }

    /// Disposes of this continuation block and all previous blocks in the chain.
    ///
    /// # Safety
    /// The pool must be the same pool used to allocate this block.
    pub unsafe fn dispose(&mut self, pool: &BufferPool) {
        let id = self.continuations.id();
        pool.return_unsafely(id);
        if !self.previous.is_null() {
            (*self.previous).dispose(pool);
        }
        *self = std::mem::zeroed();
    }
}
