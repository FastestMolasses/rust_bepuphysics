use crate::utilities::memory::buffer::Buffer;

/// Defines a type that can quickly manage requests for unmanaged memory allocation and deallocation.
pub trait IUnmanagedMemoryPool: Drop {
    /// Attempts to allocate a buffer large enough to contain a number of elements of a given type.
    /// Returns `None` if allocation fails.
    fn take_at_least<T: Sized>(&mut self, count: usize) -> Option<Buffer<T>>;

    /// Takes a typed buffer of the requested size from the pool.
    /// Returns `None` if allocation fails.
    fn take<T: Sized>(&mut self, count: usize) -> Option<Buffer<T>>;

    /// Returns a buffer to the pool.
    fn return_buffer<T: Sized>(&mut self, buffer: Buffer<T>);

    /// Gets the capacity of a buffer that would be returned by the pool
    /// for a given element count.
    fn get_capacity_for_count<T: Sized>(&self, count: usize) -> usize;

    /// Resizes a typed buffer to the smallest available size in the pool
    /// while containing the target size. Copies a subset of elements into the new buffer.
    /// Final buffer size is at least as large as the target size and may be larger.
    /// If resizing fails, the original buffer remains unchanged, and the method returns `false`.
    fn resize_to_at_least<T: Sized>(
        &mut self,
        buffer: &mut Buffer<T>,
        target_size: usize,
        copy_count: usize,
    ) -> bool;

    /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
    /// If resizing fails, the original buffer remains unchanged, and the method returns `false`.
    fn resize<T: Sized>(
        &mut self,
        buffer: &mut Buffer<T>,
        target_size: usize,
        copy_count: usize,
    ) -> bool;

    /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated.
    fn clear(&mut self);

    /// Computes the total number of bytes allocated from the memory source in this pool.
    fn get_total_allocated_byte_count(&self) -> usize;
}
