use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;
use core::marker::PhantomData;
use std::{
    mem::size_of,
    ops::{Index, IndexMut},
    slice,
};

#[repr(C)]
#[derive(Debug)]
/// Span over an unmanaged memory region.
pub struct Buffer<T> {
    /// Pointer to the beginning of the memory backing this buffer.
    ptr: *mut T,
    /// Length of the buffer in terms of elements
    length: i32,
    /// We're primarily interested in x64, so memory + length is 12 bytes. This struct would/should get padded to 16 bytes for alignment reasons anyway,
    /// so making use of the last 4 bytes to speed up the case where the raw buffer is taken from a pool (which is basically always) is a good option.
    pub id: i32,
    _phantom: PhantomData<T>,
}

unsafe impl<T: Send> Send for Buffer<T> {}
unsafe impl<T: Sync> Sync for Buffer<T> {}

impl<T> Buffer<T> {
    /// Creates a new buffer.
    /// # Safety
    /// - `ptr` must be valid for reads and writes for `length` elements of T
    /// - `ptr` must be properly aligned for T
    /// - The memory referenced by `ptr` must not be accessed through any other pointer while this Buffer exists
    #[inline(always)]
    pub fn new(ptr: *mut T, length: i32, id: i32) -> Self {
        debug_assert!(!ptr.is_null());
        debug_assert!(length >= 0);
        Self {
            ptr,
            length,
            id,
            _phantom: PhantomData,
        }
    }

    /// Takes a buffer large enough to contain a number of elements of type T.
    #[inline(always)]
    pub fn new_from_pool<P: UnmanagedMemoryPool>(length: i32) -> Self {
        let mut buffer = unsafe { Self::new(std::ptr::null_mut(), 0, -1) };
        P::take(length, &mut buffer);
        buffer
    }

    /// Returns a buffer to a pool. This should only be used if the specified
    /// pool is the same as the one used to allocate the buffer.
    #[inline(always)]
    pub fn dispose<P: UnmanagedMemoryPool>(&mut self) {
        P::return_to_pool(self);
    }

    /// Gets a reference to the element at the given index.
    #[inline(always)]
    pub fn get(&self, index: i32) -> &T {
        debug_assert!(index >= 0 && index < self.length);
        unsafe { &*self.ptr.add(index as usize) }
    }

    /// Gets a mutable reference to the element at the given index.
    #[inline(always)]
    pub fn get_mut(&mut self, index: i32) -> &mut T {
        debug_assert!(index >= 0 && index < self.length);
        unsafe { &mut *self.ptr.add(index as usize) }
    }

    /// Gets a pointer to the element at the given index.
    #[inline(always)]
    pub fn get_ptr(&self, index: i32) -> *const T {
        debug_assert!(index >= 0 && index < self.length);
        unsafe { self.ptr.add(index as usize) }
    }

    /// Gets a mutable pointer to the element at the given index.
    #[inline(always)]
    pub fn get_mut_ptr(&mut self, index: i32) -> *mut T {
        debug_assert!(index >= 0 && index < self.length);
        unsafe { self.ptr.add(index as usize) }
    }

    /// Creates a view of a subset of the buffer's memory.
    #[inline(always)]
    pub fn slice_offset(&self, start: i32, count: i32) -> Buffer<T> {
        debug_assert!(start >= 0 && start + count <= self.length);
        unsafe { Buffer::new(self.ptr.add(start as usize), count, self.id) }
    }

    /// Creates a view of a subset of the buffer's memory.
    #[inline(always)]
    pub fn slice_offset_into(&self, start: i32, count: i32, sliced: &mut Buffer<T>) {
        debug_assert!(start >= 0 && start + count <= self.length);
        *sliced = unsafe { Buffer::new(self.ptr.add(start as usize), count, self.id) };
    }

    /// Creates a view of a subset of the buffer's memory, starting from the first index.
    #[inline(always)]
    pub fn slice_count(&self, count: i32) -> Buffer<T> {
        debug_assert!(count >= 0 && count <= self.length);
        unsafe { Buffer::new(self.ptr, count, self.id) }
    }

    /// Creates a view of a subset of the buffer's memory, starting from the first index.
    #[inline(always)]
    pub fn slice_count_into(&self, count: i32, sliced: &mut Buffer<T>) {
        debug_assert!(count >= 0 && count <= self.length);
        *sliced = unsafe { Buffer::new(self.ptr, count, self.id) };
    }

    /// Gets the length of the buffer in typed elements.
    #[inline(always)]
    pub fn len(&self) -> i32 {
        self.length
    }

    /// Gets whether the buffer is empty.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.length == 0
    }

    /// Gets whether the buffer references non-null memory.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        !self.ptr.is_null()
    }

    /// Zeroes out the buffer's memory.
    #[inline(always)]
    pub fn clear(&mut self, start: i32, count: i32) {
        debug_assert!(start >= 0 && start + count <= self.length);
        unsafe {
            std::ptr::write_bytes(self.ptr.add(start as usize), 0, count as usize);
        }
    }

    /// Copies buffer data into another buffer.
    #[inline(always)]
    pub fn copy_to(
        &self,
        source_start: i32,
        target: &mut Buffer<T>,
        target_start: i32,
        count: i32,
    ) {
        debug_assert!(source_start >= 0 && source_start + count <= self.length);
        debug_assert!(target_start >= 0 && target_start + count <= target.length);
        unsafe {
            std::ptr::copy_nonoverlapping(
                self.ptr.add(source_start as usize),
                target.ptr.add(target_start as usize),
                count as usize,
            );
        }
    }

    /// Copies buffer data into a slice.
    #[inline(always)]
    pub fn copy_to_slice(&self, source_start: i32, target: &mut [T], target_start: i32, count: i32)
    where
        T: Copy,
    {
        debug_assert!(source_start >= 0 && source_start + count <= self.length);
        debug_assert!(target_start >= 0 && (target_start + count) as usize <= target.len());
        unsafe {
            std::ptr::copy_nonoverlapping(
                self.ptr.add(source_start as usize),
                target.as_mut_ptr().add(target_start as usize),
                count as usize,
            );
        }
    }

    /// Copies slice data into this buffer.
    #[inline(always)]
    pub fn copy_from_slice(
        &mut self,
        source: &[T],
        source_start: i32,
        target_start: i32,
        count: i32,
    ) where
        T: Copy,
    {
        debug_assert!(source_start >= 0 && (source_start + count) as usize <= source.len());
        debug_assert!(target_start >= 0 && target_start + count <= self.length);
        unsafe {
            std::ptr::copy_nonoverlapping(
                source.as_ptr().add(source_start as usize),
                self.ptr.add(target_start as usize),
                count as usize,
            );
        }
    }

    /// Creates a typed region from the raw buffer with the largest capacity that can fit within the allocated bytes.
    #[inline(always)]
    pub fn cast<U>(&self) -> Buffer<U> {
        let count = (self.length as usize * size_of::<T>() / size_of::<U>()) as i32;
        unsafe { Buffer::new(self.ptr as *mut U, count, self.id) }
    }

    /// Returns a mutable slice to the buffer's contents.
    #[inline(always)]
    pub fn as_slice_mut(&mut self) -> &mut [T] {
        unsafe { slice::from_raw_parts_mut(self.ptr, self.length as usize) }
    }

    /// Returns a slice to the buffer's contents.
    #[inline(always)]
    pub fn as_slice(&self) -> &[T] {
        unsafe { slice::from_raw_parts(self.ptr, self.length as usize) }
    }
}

impl<T> Index<i32> for Buffer<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: i32) -> &Self::Output {
        unsafe { self.get(index) }
    }
}

impl<T> Index<u32> for Buffer<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: u32) -> &Self::Output {
        self.get(index)
    }
}

impl<T> IndexMut<i32> for Buffer<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: i32) -> &mut Self::Output {
        self.get_mut(index)
    }
}

impl<T> IndexMut<u32> for Buffer<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: u32) -> &mut Self::Output {
        self.get_mut(index)
    }
}

impl<T> Drop for Buffer<T> {
    fn drop(&mut self) {
        // We don't deallocate memory here since buffers are managed by pools
    }
}
