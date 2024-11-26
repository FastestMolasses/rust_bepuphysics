use crate::utilities::memory::buffer::Buffer;

/// Trait for types that are capable of rapidly serving requests for allocation and deallocation of unmanaged memory.
pub trait UnmanagedMemoryPool {
    /// Takes a buffer large enough to contain a number of elements of a given type. Capacity may be larger than requested.
    /// # Type Parameters
    /// * `T`: Type of the elements in the buffer.
    /// # Parameters
    /// * `count`: Desired minimum capacity of the buffer in typed elements.
    /// * `buffer`: Buffer large enough to contain the requested number of elements.
    fn take_at_least<T>(count: i32, buffer: &mut Buffer<T>)
    where
        T: Copy + Sized;

    /// Takes a typed buffer of the requested size from the pool.
    /// # Type Parameters
    /// * `T`: Type of the instances in the buffer.
    /// # Parameters
    /// * `count`: Desired capacity of the buffer in typed elements.
    /// * `buffer`: Typed buffer of the requested size.
    fn take<T>(count: i32, buffer: &mut Buffer<T>)
    where
        T: Copy + Sized;

    /// Returns a buffer to the pool.
    /// # Type Parameters
    /// * `T`: Type of the buffer's elements.
    /// # Parameters
    /// * `buffer`: Buffer to return to the pool. The reference will be cleared.
    fn return_to_pool<T>(buffer: &mut Buffer<T>)
    where
        T: Copy + Sized;

    /// Gets the capacity of a buffer that would be returned by the pool if a given element count was requested from take_at_least.
    /// # Type Parameters
    /// * `T`: Type of the elements being requested.
    /// # Parameters
    /// * `count`: Number of elements to request.
    /// # Returns
    /// Capacity of a buffer that would be returned if the given element count was requested.
    fn get_capacity_for_count<T>(count: i32) -> i32
    where
        T: Copy + Sized;

    /// Returns a buffer to the pool by id.
    /// # Parameters
    /// * `id`: Id of the buffer to return to the pool.
    /// # Safety
    /// Pools zero out the passed-in buffer by convention. This costs very little and avoids a wide variety of bugs (either directly or by forcing fast failure).
    /// This "Unsafe" method should be used only in cases where there's a reason to bypass the clear; the naming is intended to dissuade casual use.
    unsafe fn return_unsafely(id: i32);

    /// Resizes a typed buffer to the smallest size available in the pool which contains the target size. Copies a subset of elements into the new buffer.
    /// Final buffer size is at least as large as the target size and may be larger.
    /// # Type Parameters
    /// * `T`: Type of the buffer to resize.
    /// # Parameters
    /// * `buffer`: Buffer reference to resize.
    /// * `target_size`: Number of elements to resize the buffer for.
    /// * `copy_count`: Number of elements to copy into the new buffer from the old buffer. Contents of slots outside the copied range in the resized buffer are undefined.
    fn resize_to_at_least<T>(buffer: &mut Buffer<T>, target_size: i32, copy_count: i32)
    where
        T: Copy + Sized;

    /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
    /// # Type Parameters
    /// * `T`: Type of the buffer to resize.
    /// # Parameters
    /// * `buffer`: Buffer reference to resize.
    /// * `target_size`: Number of elements to resize the buffer for.
    /// * `copy_count`: Number of elements to copy into the new buffer from the old buffer.
    fn resize<T>(buffer: &mut Buffer<T>, target_size: i32, copy_count: i32)
    where
        T: Copy + Sized;

    /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated silently.
    /// The pool will remain in a usable state after clearing.
    fn clear();

    /// Computes the total number of bytes allocated from the memory source in this pool.
    /// Includes allocated memory regardless of whether it currently has outstanding references.
    /// # Returns
    /// Total number of bytes allocated from the backing memory source in this pool.
    fn get_total_allocated_byte_count() -> u64;
}
