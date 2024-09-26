use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::unmanaged_mempool::IUnmanagedMemoryPool;
use std::marker::PhantomData;
use std::mem::{self, MaybeUninit};
use std::ops::{Index, IndexMut};
use std::ptr;

// Rust's equivalent of C#'s unmanaged constraint is not explicitly required.
// Rust inherently supports operations on types that are safe to store in contiguous memory.
pub struct QuickList<T> {
    span: Buffer<T>,
    count: usize,
    // PhantomData to tell Rust about T without storing it.
    _marker: PhantomData<T>,
}

impl<T> QuickList<T> {
    pub fn new(initial_span: Buffer<T>) -> Self {
        // BufferPool
        Self {
            span: initial_span,
            count: 0,
            _marker: PhantomData,
        }
    }

    pub fn with_capacity(
        minimum_initial_count: usize,
        pool: &mut dyn IUnmanagedMemoryPool,
    ) -> Self {
        let span = pool.take_at_least(minimum_initial_count);
        Self {
            span,
            count: 0,
            _marker: PhantomData,
        }
    }

    pub fn resize(&mut self, new_size: usize, pool: &mut dyn IUnmanagedMemoryPool) {
        let target_size = pool.get_capacity_for_count::<T>(new_size);
        if target_size != self.span.len() {
            let old_span = mem::replace(&mut self.span, pool.take_at_least(target_size));
            self.span.copy_from_slice(&old_span[..self.count]);
            pool.return_span(old_span);
        }
    }

    pub fn ensure_capacity(&mut self, count: usize, pool: &mut dyn IUnmanagedMemoryPool) {
        if self.span.is_allocated() && count > self.span.len() {
            self.resize(count, pool);
        } else if !self.span.is_allocated() {
            self.span = pool.take_at_least(count);
        }
    }

    pub unsafe fn get_pointer(&self, index: usize) -> *const T {
        self.span.as_ptr().add(index)
    }

    pub fn add(&mut self, element: T, pool: &mut dyn IUnmanagedMemoryPool) {
        self.ensure_capacity(self.count + 1, pool);
        unsafe {
            ptr::write(self.span.as_mut_ptr().add(self.count), element);
        }
        self.count += 1;
    }

    // Indexing
    pub fn get(&self, index: usize) -> &T {
        assert!(index < self.count, "Index out of bounds");
        unsafe { &*self.span.as_ptr().add(index) }
    }

    pub fn get_mut(&mut self, index: usize) -> &mut T {
        assert!(index < self.count, "Index out of bounds");
        unsafe { &mut *self.span.as_mut_ptr().add(index) }
    }

    // Add range from a slice
    pub fn add_range(&mut self, elements: &[T], pool: &mut dyn IUnmanagedMemoryPool) {
        let additional_len = elements.len();
        self.ensure_capacity(self.count + additional_len, pool);
        let dst = unsafe { self.span.as_mut_ptr().add(self.count) };
        unsafe {
            ptr::copy_nonoverlapping(elements.as_ptr(), dst, additional_len);
        }
        self.count += additional_len;
    }

    // Remove an element at a specific index
    pub fn remove_at(&mut self, index: usize) {
        assert!(index < self.count, "Index out of bounds");
        if index < self.count - 1 {
            let ptr = self.span.as_mut_ptr();
            unsafe {
                // Shift elements after index to the left by one.
                ptr::copy(ptr.add(index + 1), ptr.add(index), self.count - index - 1);
            }
        }
        self.count -= 1;
        // Manually drop the last element
        unsafe {
            ptr::drop_in_place(self.span.as_mut_ptr().add(self.count));
        }
    }

    // Fast remove at index by swapping with the last element
    pub fn fast_remove_at(&mut self, index: usize) {
        assert!(index < self.count, "Index out of bounds");
        let last_index = self.count - 1;
        if index != last_index {
            unsafe {
                let ptr = self.span.as_mut_ptr();
                ptr::swap(ptr.add(index), ptr.add(last_index));
            }
        }
        self.count -= 1;
        // Manually drop the last element
        unsafe {
            ptr::drop_in_place(self.span.as_mut_ptr().add(self.count));
        }
    }

    // Check if the list contains an element
    pub fn contains(&self, element: &T) -> bool
    where
        T: PartialEq,
    {
        self.index_of(element).is_some()
    }

    // Get index of an element
    pub fn index_of(&self, element: &T) -> Option<usize>
    where
        T: PartialEq,
    {
        let ptr = self.span.as_ptr();
        for i in 0..self.count {
            unsafe {
                if &*ptr.add(i) == element {
                    return Some(i);
                }
            }
        }
        None
    }

    // Clear the list
    pub fn clear(&mut self) {
        // Dropping all elements manually if T implements Drop
        if mem::needs_drop::<T>() {
            for i in 0..self.count {
                unsafe {
                    ptr::drop_in_place(self.span.as_mut_ptr().add(i));
                }
            }
        }
        self.count = 0;
    }
}

impl<T> Index<usize> for QuickList<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        self.get(index)
    }
}

impl<T> IndexMut<usize> for QuickList<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        self.get_mut(index)
    }
}
