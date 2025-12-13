use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;

use std::ops::{Index, IndexMut};
use std::ptr;

/// Container supporting list-like behaviors built on top of unmanaged buffers.
///
/// Be very careful when using this type. It has sacrificed a lot upon the altar of performance.
#[repr(C)]
pub struct QuickList<T> {
    /// Backing memory containing the elements of the list.
    /// Indices from 0 to count-1 hold actual data. All other data is undefined.
    pub span: Buffer<T>,
    /// Number of elements in the list.
    pub count: i32,
}

impl<T: Copy> QuickList<T> {
    /// Creates a new list.
    #[inline(always)]
    pub fn new(initial_span: Buffer<T>) -> Self {
        Self {
            span: initial_span,
            count: 0,
        }
    }

    /// Creates a new list with minimum capacity from a pool.
    #[inline(always)]
    pub fn with_capacity(minimum_initial_count: i32, pool: &mut impl UnmanagedMemoryPool) -> Self {
        let span = pool.take_at_least::<T>(minimum_initial_count);
        Self { span, count: 0 }
    }

    /// Gets the capacity of the backing span.
    #[inline(always)]
    pub fn capacity(&self) -> i32 {
        self.span.len()
    }

    /// Returns true if this list's backing memory is allocated.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.span.allocated()
    }

    /// Swaps out the list's backing memory span for a new span.
    #[inline(always)]
    pub fn resize_span(&mut self, new_span: &mut Buffer<T>, old_span: &mut Buffer<T>) {
        *old_span = self.span;
        self.span = *new_span;
        if self.count > self.span.len() {
            self.count = self.span.len();
        }
        old_span.copy_to(0, &mut self.span, 0, self.count);
    }

    /// Resizes the list's backing array for the given size.
    #[inline(always)]
    pub fn resize(&mut self, new_size: i32, pool: &mut impl UnmanagedMemoryPool) {
        let target_size = BufferPool::get_capacity_for_count::<T>(new_size);
        if target_size != self.span.len() {
            let mut old_span = self.span;
            let mut new_span = pool.take_at_least::<T>(target_size);
            self.resize_span(&mut new_span, &mut old_span);
            pool.return_buffer(&mut old_span);
        }
    }

    /// Returns the resources associated with the list to pools.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        pool.return_buffer(&mut self.span);
    }

    /// Ensures that the list has enough room to hold the specified number of elements.
    #[inline(always)]
    pub fn ensure_capacity(&mut self, count: i32, pool: &mut impl UnmanagedMemoryPool) {
        if self.span.allocated() {
            if count > self.span.len() {
                self.resize(count, pool);
            }
        } else {
            self.span = pool.take_at_least(count);
        }
    }

    /// Compacts the internal buffer to the minimum size required.
    #[inline(always)]
    pub fn compact(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        let target_length = BufferPool::get_capacity_for_count::<T>(self.count);
        if target_length != self.span.len() {
            self.resize(target_length, pool);
        }
    }

    /// Gets a pointer to the element at the given index in the list.
    #[inline(always)]
    pub unsafe fn get_pointer(&self, index: i32) -> *const T {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        self.span.as_ptr().add(index as usize)
    }

    /// Gets a mutable pointer to the element at the given index in the list.
    #[inline(always)]
    pub unsafe fn get_pointer_mut(&mut self, index: i32) -> *mut T {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        self.span.as_mut_ptr().add(index as usize)
    }

    /// Gets a reference to the element at the given index.
    #[inline(always)]
    pub fn get(&self, index: i32) -> &T {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        &self.span[index]
    }

    /// Gets a mutable reference to the element at the given index.
    #[inline(always)]
    pub fn get_mut(&mut self, index: i32) -> &mut T {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        &mut self.span[index]
    }

    /// Adds an element to the list without checking capacity.
    #[inline(always)]
    pub fn add_unsafely(&mut self, element: T) {
        debug_assert!(
            self.count < self.span.len(),
            "Adding would exceed capacity"
        );
        self.span[self.count] = element;
        self.count += 1;
    }

    /// Adds an element to the list, resizing if necessary.
    #[inline(always)]
    pub fn add(&mut self, element: T, pool: &mut impl UnmanagedMemoryPool) {
        self.ensure_capacity(self.count + 1, pool);
        self.add_unsafely(element);
    }

    /// Adds a range of elements to the list without checking capacity.
    #[inline(always)]
    pub fn add_range_unsafely(&mut self, span: &Buffer<T>, start: i32, count: i32) {
        debug_assert!(
            self.count + count <= self.span.len(),
            "Adding would exceed capacity"
        );
        span.copy_to(start, &mut self.span, self.count, count);
        self.count += count;
    }

    /// Adds a range of elements to the list.
    #[inline(always)]
    pub fn add_range(
        &mut self,
        span: &Buffer<T>,
        start: i32,
        count: i32,
        pool: &mut impl UnmanagedMemoryPool,
    ) {
        self.ensure_capacity(self.count + count, pool);
        self.add_range_unsafely(span, start, count);
    }

    /// Removes an element at a specific index by shifting subsequent elements.
    #[inline(always)]
    pub fn remove_at(&mut self, index: i32) {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        self.count -= 1;
        if index < self.count {
            // Shift elements down
            let ptr = self.span.as_mut_ptr();
            unsafe {
                ptr::copy(
                    ptr.add((index + 1) as usize),
                    ptr.add(index as usize),
                    (self.count - index) as usize,
                );
            }
        }
    }

    /// Removes an element at a specific index by swapping with the last element (faster).
    #[inline(always)]
    pub fn fast_remove_at(&mut self, index: i32) {
        debug_assert!(index >= 0 && index < self.count, "Index out of bounds");
        self.count -= 1;
        if index < self.count {
            self.span[index] = self.span[self.count];
        }
    }

    /// Removes the first occurrence of an element from the list.
    #[inline(always)]
    pub fn remove(&mut self, element: &T) -> bool
    where
        T: PartialEq,
    {
        if let Some(index) = self.index_of(element) {
            self.remove_at(index);
            true
        } else {
            false
        }
    }

    /// Removes the first occurrence of an element using fast swap-remove.
    #[inline(always)]
    pub fn fast_remove(&mut self, element: &T) -> bool
    where
        T: PartialEq,
    {
        if let Some(index) = self.index_of(element) {
            self.fast_remove_at(index);
            true
        } else {
            false
        }
    }

    /// Gets the index of an element in the list.
    #[inline(always)]
    pub fn index_of(&self, element: &T) -> Option<i32>
    where
        T: PartialEq,
    {
        for i in 0..self.count {
            if self.span[i] == *element {
                return Some(i);
            }
        }
        None
    }

    /// Checks if the list contains an element.
    #[inline(always)]
    pub fn contains(&self, element: &T) -> bool
    where
        T: PartialEq,
    {
        self.index_of(element).is_some()
    }

    /// Clears the list.
    #[inline(always)]
    pub fn clear(&mut self) {
        self.count = 0;
    }

    /// Gets the number of elements in the list.
    #[inline(always)]
    pub fn len(&self) -> i32 {
        self.count
    }

    /// Returns true if the list is empty.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Inserts an element at the given index without checking capacity.
    #[inline(always)]
    pub fn insert_unsafely(&mut self, element: T, index: i32) {
        debug_assert!(index >= 0 && index <= self.count, "Index out of bounds");
        debug_assert!(
            self.count < self.span.len(),
            "Inserting would exceed capacity"
        );
        if index < self.count {
            let ptr = self.span.as_mut_ptr();
            unsafe {
                ptr::copy(
                    ptr.add(index as usize),
                    ptr.add((index + 1) as usize),
                    (self.count - index) as usize,
                );
            }
        }
        self.span[index] = element;
        self.count += 1;
    }

    /// Inserts an element at the given index.
    #[inline(always)]
    pub fn insert(&mut self, element: T, index: i32, pool: &mut impl UnmanagedMemoryPool) {
        self.ensure_capacity(self.count + 1, pool);
        self.insert_unsafely(element, index);
    }

    /// Allocates and returns a slot in the list without checking capacity.
    #[inline(always)]
    pub fn allocate_unsafely(&mut self) -> &mut T {
        debug_assert!(
            self.count < self.span.len(),
            "Allocating would exceed capacity"
        );
        let index = self.count;
        self.count += 1;
        &mut self.span[index]
    }

    /// Allocates and returns a slot in the list.
    #[inline(always)]
    pub fn allocate(&mut self, pool: &mut impl UnmanagedMemoryPool) -> &mut T {
        self.ensure_capacity(self.count + 1, pool);
        self.allocate_unsafely()
    }

    /// Pops the last element from the list.
    #[inline(always)]
    pub fn pop(&mut self) -> T {
        debug_assert!(self.count > 0, "Cannot pop from empty list");
        self.count -= 1;
        self.span[self.count]
    }

    /// Tries to pop the last element from the list.
    #[inline(always)]
    pub fn try_pop(&mut self) -> Option<T> {
        if self.count > 0 {
            Some(self.pop())
        } else {
            None
        }
    }
}

impl<T: Copy> Index<i32> for QuickList<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: i32) -> &Self::Output {
        self.get(index)
    }
}

impl<T: Copy> IndexMut<i32> for QuickList<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: i32) -> &mut Self::Output {
        self.get_mut(index)
    }
}

impl<T: Copy> Default for QuickList<T> {
    fn default() -> Self {
        Self {
            span: Buffer::default(),
            count: 0,
        }
    }
}

impl<T> Clone for QuickList<T> {
    fn clone(&self) -> Self {
        Self {
            span: self.span,
            count: self.count,
        }
    }
}

impl<T> Copy for QuickList<T> {}
