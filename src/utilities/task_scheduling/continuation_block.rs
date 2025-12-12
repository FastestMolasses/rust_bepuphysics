//! Block storage for task continuations.
//!
//! ContinuationBlocks form a linked list to allow growing continuation storage
//! without reallocation (which would introduce race conditions).

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::task_continuation::TaskContinuation;

/// Stores a block of task continuations that maintains a pointer to previous blocks.
///
/// Layout matches C# default sequential layout:
/// - Previous pointer (8 bytes)
/// - Count (4 bytes)
/// - Padding (4 bytes for alignment)
/// - Continuations buffer (16 bytes)
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
    /// - The pool must have sufficient memory available.
    /// - Caller must have exclusive access to the pool.
    pub unsafe fn create(
        continuation_capacity: i32,
        pool: &mut BufferPool,
    ) -> *mut ContinuationBlock {
        // Allocate memory for both the block header and continuations in one allocation
        let total_size = std::mem::size_of::<TaskContinuation>() * continuation_capacity as usize
            + std::mem::size_of::<ContinuationBlock>();

        let raw_buffer: Buffer<u8> = pool.take(total_size as i32);
        let block = raw_buffer.as_ptr() as *mut ContinuationBlock;

        // Set up the continuations buffer to point just after the block header
        let continuations_ptr = (raw_buffer.as_ptr() as *mut u8)
            .add(std::mem::size_of::<ContinuationBlock>())
            as *mut TaskContinuation;

        std::ptr::addr_of_mut!((*block).continuations).write(Buffer::new(
            continuations_ptr,
            continuation_capacity,
            raw_buffer.id(),
        ));
        std::ptr::addr_of_mut!((*block).count).write(0);
        std::ptr::addr_of_mut!((*block).previous).write(std::ptr::null_mut());

        block
    }

    /// Tries to allocate a continuation from this block.
    ///
    /// # Returns
    /// `true` if allocation succeeded (pointer written to `continuation`), `false` if the block is full.
    #[inline(always)]
    pub unsafe fn try_allocate_continuation(
        &mut self,
        continuation: &mut *mut TaskContinuation,
    ) -> bool {
        if self.count < self.continuations.len() {
            *continuation = self.continuations.as_mut_ptr().add(self.count as usize);
            self.count += 1;
            true
        } else {
            *continuation = std::ptr::null_mut();
            false
        }
    }

    /// Disposes of this continuation block and all previous blocks in the chain.
    ///
    /// # Safety
    /// - The pool must be the same pool used to allocate this block.
    /// - Caller must have exclusive access to the pool.
    pub unsafe fn dispose(&mut self, pool: &mut BufferPool) {
        let id = self.continuations.id();
        pool.return_unsafely(id);
        if !self.previous.is_null() {
            (*self.previous).dispose(pool);
        }
        *self = std::mem::zeroed();
    }
}
