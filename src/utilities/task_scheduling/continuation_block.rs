use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::task_continuation::TaskContinuation;
use std::ptr::null_mut;

/// Stores a block of task continuations that maintains a pointer to previous blocks.
#[repr(C)]
pub struct ContinuationBlock {
    previous: *mut ContinuationBlock,
    count: i32,
    pub continuations: Buffer<TaskContinuation>,
}

impl ContinuationBlock {
    /// Creates a new block of task continuations.
    pub unsafe fn create(continuation_capacity: i32, pool: &mut BufferPool) -> *mut ContinuationBlock {
        let total_size = std::mem::size_of::<TaskContinuation>() * continuation_capacity as usize + std::mem::size_of::<ContinuationBlock>();
        let raw_buffer = pool.take::<u8>(total_size);
        let block = raw_buffer.as_mut_ptr() as *mut ContinuationBlock;
        (*block).continuations = Buffer::new(raw_buffer.offset(std::mem::size_of::<ContinuationBlock>() as isize), continuation_capacity, raw_buffer.id());
        (*block).count = 0;
        (*block).previous = null_mut();
        block
    }

    /// Tries to allocate a continuation, returning a mutable pointer to it if successful.
    pub unsafe fn try_allocate_continuation(&mut self) -> Option<*mut TaskContinuation> {
        if self.count < self.continuations.length() {
            let continuation = self.continuations.as_mut_ptr().offset(self.count as isize);
            self.count += 1;
            Some(continuation)
        } else {
            None
        }
    }

    /// Disposes of the continuation block, returning its resources to the pool.
    pub unsafe fn dispose(&mut self, pool: &mut BufferPool) {
        let id = self.continuations.id();
        pool.return_unsafely(id);
        if !self.previous.is_null() {
            (*self.previous).dispose(pool);
        }
        *self = std::mem::zeroed();
    }
}

// Necessary for correct Rust memory management
impl Drop for ContinuationBlock {
    fn drop(&mut self) {
        // Logic to properly release resources, if needed, goes here.
        // Since `ContinuationBlock::dispose` must be called explicitly and may involve unsafe code,
        // careful consideration is required to decide whether any logic should be placed here.
        // TODO: This might remain empty to enforce explicit management through `dispose`.
    }
}
