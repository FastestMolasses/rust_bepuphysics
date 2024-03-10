use std::ops::{Deref, DerefMut, Index, IndexMut};
use std::{marker::PhantomData, mem, ptr};

use crate::utilities::memory::unmanaged_mempool::IUnmanagedMemoryPool;

/// Represents a span of unmanaged memory.
pub struct Buffer<T> {
    memory: ptr::NonNull<T>,
    length: usize,
    id: i32,
    /// PhantomData is used to indicate that the Buffer<T> struct is logically owning data of type T.
    _marker: PhantomData<T>,
}

impl<T: std::cmp::PartialEq> Buffer<T> {
    /// Creates a new buffer.
    pub fn new(memory: *mut T, length: usize, id: i32) -> Self {
        Buffer {
            memory,
            length,
            id,
            _marker: PhantomData,
        }
    }

    /// Allocates a new buffer from a pool.
    pub fn from_pool(
        length: usize,
        pool: &mut impl IUnmanagedMemoryPool,
    ) -> Result<Self, &'static str> {
        pool.take(length)
    }

    /// Returns the length of the buffer in typed elements.
    pub fn len(&self) -> usize {
        self.length
    }

    /// Gets whether the buffer references non-null memory.
    pub fn is_allocated(&self) -> bool {
        !self.memory.as_ptr().is_null()
    }

    /// Returns a buffer to a pool.
    pub fn dispose(self, pool: &mut impl IUnmanagedMemoryPool) {
        pool.return_buffer(self);
    }

    /// Zeroes out the buffer's memory.
    pub fn clear(&mut self, start: usize, count: usize) {
        debug_assert!(start + count <= self.length, "Clear region out of bounds.");
        unsafe {
            ptr::write_bytes(
                self.memory.as_ptr().add(start),
                0,
                count * mem::size_of::<T>(),
            );
        }
    }

    /// Creates a view of a subset of the buffer's memory.
    pub fn slice(&self, start: usize, count: usize) -> Self {
        debug_assert!(start + count <= self.length, "Slice region out of bounds.");
        let new_memory = unsafe { self.memory.as_ptr().add(start) };
        Buffer {
            memory: new_memory,
            length: count,
            id: self.id,
            _marker: PhantomData,
        }
    }

    /// Copies buffer data into another buffer.
    pub fn copy_to(
        &self,
        source_start: usize,
        target: &mut Buffer<T>,
        target_start: usize,
        count: usize,
    ) {
        debug_assert!(
            source_start + count <= self.length,
            "Source region out of bounds"
        );
        debug_assert!(
            target_start + count <= target.length,
            "Target region out of bounds"
        );

        unsafe {
            ptr::copy_nonoverlapping(
                self.memory.as_ptr().add(source_start),
                target.memory.as_ptr().add(target_start),
                count * mem::size_of::<T>(),
            );
        }
    }

    /// Copies span data into this buffer.
    pub fn copy_from(
        &mut self,
        source: &[T],
        source_start: usize,
        target_start: usize,
        count: usize,
    ) {
        debug_assert!(
            source_start + count <= source.len(),
            "Source region out of bounds"
        );
        debug_assert!(
            target_start + count <= self.length,
            "Target region out of bounds"
        );

        unsafe {
            ptr::copy_nonoverlapping(
                source.as_ptr().add(source_start),
                self.memory.as_ptr().add(target_start),
                count * mem::size_of::<T>(),
            );
        }
    }

    /// Gets the index of an element, if it exists, using the type's default equality comparison.
    pub fn index_of(&self, element: &T, start: usize, count: usize) -> Option<usize> {
        debug_assert!(start + count <= self.length, "Index region out of bounds");

        for i in start..(start + count) {
            if self[i] == *element {
                return Some(i);
            }
        }
        None
    }

    /// Creates a typed region from the raw buffer with the largest capacity that can fit within the allocated bytes.
    pub fn as_cast<TCast>(&self) -> Buffer<TCast>
    where
        TCast: Sized,
    {
        let count = self.length * mem::size_of::<T>() / mem::size_of::<TCast>();
        Buffer {
            memory: self.memory as *mut TCast,
            length: count,
            id: self.id,
            _marker: PhantomData,
        }
    }
}

// Implement indexing
impl<T> Index<usize> for Buffer<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        debug_assert!(index < self.length);
        unsafe { &*self.memory.as_ptr().add(index) }
    }
}

impl<T> IndexMut<usize> for Buffer<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        debug_assert!(index < self.length);
        unsafe { &mut *self.memory.as_ptr().add(index) }
    }
}

// Allow treating Buffer<T> similarly to slices with Deref
impl<T> Deref for Buffer<T> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        unsafe { std::slice::from_raw_parts(self.memory, self.length) }
    }
}

impl<T> DerefMut for Buffer<T> {
    fn deref_mut(&mut self) -> &mut [T] {
        unsafe { std::slice::from_raw_parts_mut(self.memory, self.length) }
    }
}
