//! Trait for types that are capable of rapidly serving requests for allocation and
//! deallocation of unmanaged memory.

use super::buffer::Buffer;

/// Defines a type that is capable of rapidly serving requests for allocation and
/// deallocation of unmanaged memory.
pub trait UnmanagedMemoryPool {
    /// Takes a buffer large enough to contain a number of elements of a given type.
    /// Capacity may be larger than requested.
    fn take_at_least<T>(&mut self, count: i32) -> Buffer<T>;

    /// Takes a typed buffer of the requested size from the pool.
    fn take<T>(&mut self, count: i32) -> Buffer<T>;

    /// Returns a buffer to the pool.
    fn return_buffer<T>(&mut self, buffer: &mut Buffer<T>);

    /// Gets the capacity of a buffer that would be returned by the pool if a given
    /// element count was requested from take_at_least.
    fn get_capacity_for_count<T>(count: i32) -> i32;

    /// Returns a buffer to the pool by id.
    ///
    /// Pools zero out the passed-in buffer by convention. This costs very little and
    /// avoids a wide variety of bugs. This "unsafe" method should be used only in cases
    /// where there's a reason to bypass the clear.
    fn return_unsafely(&mut self, id: i32);

    /// Resizes a typed buffer to the smallest size available in the pool which contains
    /// the target size. Copies a subset of elements into the new buffer.
    /// Final buffer size is at least as large as the target size and may be larger.
    fn resize_to_at_least<T: Copy>(
        &mut self,
        buffer: &mut Buffer<T>,
        target_size: i32,
        copy_count: i32,
    );

    /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
    fn resize<T: Copy>(&mut self, buffer: &mut Buffer<T>, target_size: i32, copy_count: i32);

    /// Returns all allocations in the pool to sources. Any outstanding buffers will be
    /// invalidated silently. The pool will remain in a usable state after clearing.
    fn clear(&mut self);

    /// Computes the total number of bytes allocated from the memory source in this pool.
    /// Includes allocated memory regardless of whether it currently has outstanding references.
    fn get_total_allocated_byte_count(&self) -> u64;
}
