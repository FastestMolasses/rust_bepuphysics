use std::ops::{Deref, DerefMut, Index, IndexMut};
use std::{
    marker::PhantomData,
    ptr::{copy_nonoverlapping, write_bytes, NonNull},
};

use crate::utilities::memory::unmanaged_mempool::IUnmanagedMemoryPool;

/// Represents a span of unmanaged memory.
pub struct Buffer<T> {
    memory: NonNull<T>,
    length: usize,
    id: i32,
    /// PhantomData is used to indicate that the Buffer<T> struct is logically owning data of type T.
    _marker: PhantomData<T>,
}

impl<T: std::cmp::PartialEq> Buffer<T> {
    /// Creates a new buffer.
    pub unsafe fn new(memory: *mut T, length: usize, id: i32) -> Self {
        Buffer {
            memory: NonNull::new(memory).expect("Null pointer passed to Buffer::new"),
            length,
            id,
            _marker: PhantomData,
        }
    }

    /// Allocates a new buffer from a pool.
    pub fn from_pool(length: usize, pool: &mut impl IUnmanagedMemoryPool) -> Option<Self> {
        pool.take(length)
    }

    /// Returns the length of the buffer in typed elements.
    #[inline]
    pub fn len(&self) -> usize {
        self.length
    }

    /// Gets whether the buffer references non-null memory.
    #[inline]
    pub fn is_allocated(&self) -> bool {
        !self.memory.as_ptr().is_null()
    }

    /// Returns a buffer to a pool.
    pub fn dispose(self, pool: &mut impl IUnmanagedMemoryPool) {
        pool.return_buffer(self);
    }

    /// Zeroes out the buffer's memory.
    #[inline]
    pub fn clear(&mut self, start: usize, count: usize) {
        #[cfg(debug_assertions)]
        debug_assert!(start + count <= self.length, "Clear region out of bounds.");
        unsafe {
            write_bytes::<T>(self.memory.as_ptr().add(start), 0, count);
        }
    }

    /// Creates a view of a subset of the buffer's memory.
    #[inline]
    pub fn slice(&self, start: usize, count: usize) -> Self {
        #[cfg(debug_assertions)]
        debug_assert!(start + count <= self.length, "Slice region out of bounds.");
        unsafe {
            Buffer::new(
                NonNull::new_unchecked(self.memory.as_ptr().add(start)),
                count,
                self.id,
            )
        }
    }

    /// Creates a view of a subset of the buffer's memory, starting from the first index.
    #[inline]
    pub fn slice_from_start(&self, count: usize) -> Self {
        self.slice(0, count)
    }

    /// Copies buffer data into another buffer.
    pub unsafe fn copy_to(
        &self,
        source_start: usize,
        target: &mut Buffer<T>,
        target_start: usize,
        count: usize,
    ) where
        T: Copy,
    {
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                source_start + count <= self.length,
                "Source region out of bounds"
            );
            debug_assert!(
                target_start + count <= target.length,
                "Target region out of bounds"
            );
        }

        unsafe {
            copy_nonoverlapping::<T>(
                self.memory.as_ptr().add(source_start),
                target.memory.as_ptr().add(target_start),
                count,
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
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                source_start + count <= source.len(),
                "Source region out of bounds"
            );
            debug_assert!(
                target_start + count <= self.length,
                "Target region out of bounds"
            );
        }

        unsafe {
            copy_nonoverlapping::<T>(
                source.as_ptr().add(source_start),
                self.memory.as_ptr().add(target_start),
                count,
            );
        }
    }

    /// Gets the index of an element, if it exists, using the type's default equality comparison.
    pub fn index_of(&self, element: &T, start: usize, count: usize) -> Option<usize> {
        #[cfg(debug_assertions)]
        debug_assert!(start + count <= self.length, "Index region out of bounds");

        for i in start..(start + count) {
            if self[i] == *element {
                return Some(i);
            }
        }
        None
    }
}

impl<T: Copy> Buffer<T> {
    /// Creates a typed region from the raw buffer with the largest capacity that can fit within the allocated bytes.
    /// This is a safe operation only if `T` and `TCast` have compatible memory layouts and alignment requirements.
    pub fn as_cast<TCast>(&self) -> Buffer<TCast>
    where
        TCast: Sized,
    {
        let new_length = (self.length * std::mem::size_of::<T>()) / std::mem::size_of::<TCast>();
        unsafe { Buffer::new(self.memory.cast(), new_length, self.id) }
    }
}

// Implement indexing
impl<T> Index<usize> for Buffer<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        #[cfg(debug_assertions)]
        debug_assert!(index < self.length);
        unsafe { &*self.memory.as_ptr().add(index) }
    }
}

impl<T> IndexMut<usize> for Buffer<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        #[cfg(debug_assertions)]
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
