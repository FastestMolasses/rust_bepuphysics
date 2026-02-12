use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::unmanaged_mempool::UnmanagedMemoryPool;

/// Container supporting double-ended queue behaviors built on top of unmanaged buffers.
/// Implemented as a circular buffer with power-of-2 capacity for efficient modulo operations.
///
/// Be very careful when using this type. It has sacrificed a lot upon the altar of performance.
#[repr(C)]
pub struct QuickQueue<T> {
    /// Number of elements in the queue.
    pub count: i32,
    /// Index of the first element in the queue.
    pub first_index: i32,
    /// Index of the last element in the queue.
    pub last_index: i32,
    /// Mask for use in performing fast modulo operations. Requires that the span is a power of 2.
    pub capacity_mask: i32,
    /// Backing memory containing the elements of the queue.
    pub span: Buffer<T>,
}

impl<T: Copy> QuickQueue<T> {
    /// Creates a new queue.
    #[inline(always)]
    pub fn new(span: Buffer<T>) -> Self {
        debug_assert!(
            !span.is_empty() && (span.len() & (span.len() - 1)) == 0,
            "Queues depend upon power of 2 capacities for efficient modulo operations."
        );
        let capacity_mask = span.len() - 1;
        Self {
            count: 0,
            first_index: 0,
            last_index: capacity_mask,
            capacity_mask,
            span,
        }
    }

    /// Creates a new queue with minimum capacity from a pool.
    #[inline(always)]
    pub fn with_capacity(
        minimum_initial_count: i32,
        pool: &mut impl UnmanagedMemoryPool,
    ) -> Self {
        let span = pool.take_at_least::<T>(minimum_initial_count);
        Self::new(span)
    }

    /// Gets the capacity of the backing span.
    #[inline(always)]
    pub fn capacity(&self) -> i32 {
        self.span.len()
    }

    /// Returns true if this queue's backing memory is allocated.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.span.allocated()
    }

    /// Gets the index in the backing array from a queue index.
    #[inline(always)]
    pub fn get_backing_array_index(&self, queue_index: i32) -> i32 {
        debug_assert!(
            queue_index >= 0 && queue_index < self.count,
            "Queue index must be in bounds."
        );
        (self.first_index + queue_index) & self.capacity_mask
    }

    /// Gets a reference to the element at the given queue index.
    #[inline(always)]
    pub fn get(&self, queue_index: i32) -> &T {
        let array_index = self.get_backing_array_index(queue_index);
        &self.span[array_index]
    }

    /// Gets a mutable reference to the element at the given queue index.
    #[inline(always)]
    pub fn get_mut(&mut self, queue_index: i32) -> &mut T {
        let array_index = self.get_backing_array_index(queue_index);
        &mut self.span[array_index]
    }

    /// Gets a reference to the first element in the queue.
    #[inline(always)]
    pub fn first(&self) -> &T {
        debug_assert!(self.count > 0, "Can't get from an empty queue.");
        &self.span[self.first_index]
    }

    /// Gets a mutable reference to the first element in the queue.
    #[inline(always)]
    pub fn first_mut(&mut self) -> &mut T {
        debug_assert!(self.count > 0, "Can't get from an empty queue.");
        &mut self.span[self.first_index]
    }

    /// Gets a reference to the last element in the queue.
    #[inline(always)]
    pub fn last(&self) -> &T {
        debug_assert!(self.count > 0, "Can't get from an empty queue.");
        &self.span[self.last_index]
    }

    /// Gets a mutable reference to the last element in the queue.
    #[inline(always)]
    pub fn last_mut(&mut self) -> &mut T {
        debug_assert!(self.count > 0, "Can't get from an empty queue.");
        &mut self.span[self.last_index]
    }

    #[inline(always)]
    fn increment_last(&mut self) {
        self.last_index = (self.last_index + 1) & self.capacity_mask;
        self.count += 1;
    }

    #[inline(always)]
    fn decrement_first(&mut self) {
        self.first_index = (self.first_index - 1) & self.capacity_mask;
        self.count += 1;
    }

    #[inline(always)]
    fn increment_first(&mut self) {
        self.first_index = (self.first_index + 1) & self.capacity_mask;
        self.count -= 1;
    }

    #[inline(always)]
    fn decrement_last(&mut self) {
        self.last_index = (self.last_index - 1) & self.capacity_mask;
        self.count -= 1;
    }

    /// Adds an element to the end of the queue without checking capacity.
    #[inline(always)]
    pub fn enqueue_unsafely(&mut self, element: T) {
        debug_assert!(
            self.count <= self.capacity_mask,
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.increment_last();
        self.span[self.last_index] = element;
    }

    /// Allocates a slot at the end of the queue and returns a mutable reference to it.
    #[inline(always)]
    pub fn allocate_unsafely(&mut self) -> &mut T {
        debug_assert!(
            self.count <= self.capacity_mask,
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.increment_last();
        &mut self.span[self.last_index]
    }

    /// Adds an element to the front of the queue without checking capacity.
    #[inline(always)]
    pub fn enqueue_first_unsafely(&mut self, element: T) {
        debug_assert!(
            self.count <= self.capacity_mask,
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.decrement_first();
        self.span[self.first_index] = element;
    }

    /// Allocates a slot at the front of the queue and returns a mutable reference to it.
    #[inline(always)]
    pub fn allocate_first_unsafely(&mut self) -> &mut T {
        debug_assert!(
            self.count <= self.capacity_mask,
            "Unsafe adders can only be used if capacity is guaranteed."
        );
        self.decrement_first();
        &mut self.span[self.first_index]
    }

    /// Resizes the queue's backing memory.
    fn resize(&mut self, new_size: i32, pool: &mut impl UnmanagedMemoryPool) {
        let mut new_span = pool.take_at_least::<T>(new_size);

        // Truncate count if shrinking (matches C#: Count = Math.Min(Count, newSpan.Length))
        if self.count > new_span.len() {
            self.count = new_span.len();
        }

        // Copy elements from old span to new span, handling wrap-around
        if self.last_index >= self.first_index {
            // No wrap-around: elements are contiguous
            self.span.copy_to(self.first_index, &mut new_span, 0, self.count);
        } else if self.count > 0 {
            // Wrap-around: copy in two parts
            let elements_at_end = self.span.len() - self.first_index;
            self.span.copy_to(self.first_index, &mut new_span, 0, elements_at_end);
            self.span.copy_to(0, &mut new_span, elements_at_end, self.last_index + 1);
        }

        pool.return_buffer(&mut self.span);

        self.span = new_span;
        self.capacity_mask = new_span.len() - 1;
        self.first_index = 0;
        self.last_index = if self.count > 0 {
            self.count - 1
        } else {
            self.capacity_mask
        };
    }

    /// Adds an element to the end of the queue, resizing if necessary.
    #[inline(always)]
    pub fn enqueue(&mut self, element: T, pool: &mut impl UnmanagedMemoryPool) {
        if self.count == self.span.len() {
            let new_size = if self.count == 0 {
                BufferPool::get_capacity_for_count::<T>(4)
            } else {
                self.count * 2
            };
            self.resize(new_size, pool);
        }
        self.enqueue_unsafely(element);
    }

    /// Adds an element to the front of the queue, resizing if necessary.
    #[inline(always)]
    pub fn enqueue_first(&mut self, element: T, pool: &mut impl UnmanagedMemoryPool) {
        if self.count == self.span.len() {
            let new_size = if self.count == 0 {
                BufferPool::get_capacity_for_count::<T>(4)
            } else {
                self.count * 2
            };
            self.resize(new_size, pool);
        }
        self.enqueue_first_unsafely(element);
    }

    /// Removes and returns the first element from the queue.
    #[inline(always)]
    pub fn dequeue(&mut self) -> T {
        debug_assert!(self.count > 0, "Can't dequeue from an empty queue.");
        let element = self.span[self.first_index];
        self.increment_first();
        element
    }

    /// Removes and returns the last element from the queue.
    #[inline(always)]
    pub fn dequeue_last(&mut self) -> T {
        debug_assert!(self.count > 0, "Can't dequeue from an empty queue.");
        let element = self.span[self.last_index];
        self.decrement_last();
        element
    }

    /// Tries to dequeue the first element. Returns false if the queue is empty.
    #[inline(always)]
    pub fn try_dequeue(&mut self, element: &mut T) -> bool {
        if self.count > 0 {
            *element = self.dequeue();
            true
        } else {
            false
        }
    }

    /// Tries to dequeue the last element. Returns false if the queue is empty.
    #[inline(always)]
    pub fn try_dequeue_last(&mut self, element: &mut T) -> bool {
        if self.count > 0 {
            *element = self.dequeue_last();
            true
        } else {
            false
        }
    }

    /// Dequeues a slot from the start of the queue and returns a pointer to it.
    /// Does not check count before attempting to dequeue.
    #[inline(always)]
    pub fn dequeue_unsafely(&mut self) -> *mut T {
        debug_assert!(self.count > 0, "Can't dequeue from an empty queue.");
        let ptr = self.span.get_mut_ptr(self.first_index);
        self.increment_first();
        ptr
    }

    /// Dequeues a slot from the end of the queue and returns a pointer to it.
    /// Does not check count before attempting to dequeue.
    #[inline(always)]
    pub fn dequeue_last_unsafely(&mut self) -> *mut T {
        debug_assert!(self.count > 0, "Can't dequeue from an empty queue.");
        let ptr = self.span.get_mut_ptr(self.last_index);
        self.decrement_last();
        ptr
    }

    /// Removes the element at the given queue index, preserving order.
    pub fn remove_at(&mut self, queue_index: i32) {
        debug_assert!(
            queue_index >= 0 && queue_index < self.count,
            "Index must be in bounds."
        );
        let array_index = self.get_backing_array_index(queue_index);

        if self.last_index == array_index {
            self.decrement_last();
            return;
        }
        if self.first_index == array_index {
            self.increment_first();
            return;
        }

        // Internal removal: need to shift elements
        // Four possible cases for which direction to shift
        if (self.first_index > self.last_index && array_index < self.last_index)
            || (self.first_index < self.last_index
                && (self.last_index - array_index) < (array_index - self.first_index))
        {
            // Shift elements from [array_index + 1, last_index] left
            // Use ptr::copy since regions may overlap (same buffer)
            unsafe {
                let count = (self.last_index - array_index) as usize;
                std::ptr::copy(
                    self.span.as_ptr().add((array_index + 1) as usize),
                    self.span.as_mut_ptr().add(array_index as usize),
                    count,
                );
            }
            self.decrement_last();
        } else {
            // Shift elements from [first_index, array_index - 1] right
            // Use ptr::copy since regions may overlap (same buffer)
            unsafe {
                let count = (array_index - self.first_index) as usize;
                std::ptr::copy(
                    self.span.as_ptr().add(self.first_index as usize),
                    self.span.as_mut_ptr().add((self.first_index + 1) as usize),
                    count,
                );
            }
            self.increment_first();
        }
    }

    fn clear_span(span: &mut Buffer<T>, first_index: i32, last_index: i32, count: i32) {
        if last_index >= first_index {
            span.clear(first_index, count);
        } else if count > 0 {
            span.clear(first_index, span.len() - first_index);
            span.clear(0, last_index + 1);
        }
    }

    /// Clears the queue, zeroing out all elements.
    #[inline(always)]
    pub fn clear(&mut self) {
        Self::clear_span(&mut self.span, self.first_index, self.last_index, self.count);
        self.count = 0;
        self.first_index = 0;
        self.last_index = self.capacity_mask;
    }

    /// Clears the queue without modifying the backing memory.
    #[inline(always)]
    pub fn fast_clear(&mut self) {
        self.count = 0;
        self.first_index = 0;
        self.last_index = self.capacity_mask;
    }

    /// Returns the queue's resources to the pool.
    #[inline(always)]
    pub fn dispose(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        pool.return_buffer(&mut self.span);
    }

    /// Ensures the queue has enough capacity to hold the specified number of elements.
    /// Uses C#'s conservative threshold: resizes when count >= capacity_mask (span.len() - 1).
    #[inline(always)]
    pub fn ensure_capacity(&mut self, count: i32, pool: &mut impl UnmanagedMemoryPool) {
        if !self.span.allocated() || count >= self.capacity_mask {
            self.resize(count, pool);
        }
    }

    /// Compacts the queue to the minimum required size.
    #[inline(always)]
    pub fn compact(&mut self, pool: &mut impl UnmanagedMemoryPool) {
        let target_capacity = BufferPool::get_capacity_for_count::<T>(self.count);
        if target_capacity < self.span.len() {
            self.resize(target_capacity, pool);
        }
    }

    /// Returns an iterator over the queue elements.
    pub fn iter(&self) -> QuickQueueIterator<'_, T> {
        QuickQueueIterator {
            queue: self,
            index: 0,
        }
    }
}

impl<T: Copy> std::ops::Index<i32> for QuickQueue<T> {
    type Output = T;

    #[inline(always)]
    fn index(&self, index: i32) -> &Self::Output {
        self.get(index)
    }
}

impl<T: Copy> std::ops::IndexMut<i32> for QuickQueue<T> {
    #[inline(always)]
    fn index_mut(&mut self, index: i32) -> &mut Self::Output {
        self.get_mut(index)
    }
}

/// Iterator over QuickQueue elements.
pub struct QuickQueueIterator<'a, T: Copy> {
    queue: &'a QuickQueue<T>,
    index: i32,
}

impl<'a, T: Copy> Iterator for QuickQueueIterator<'a, T> {
    type Item = T;

    #[inline(always)]
    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.queue.count {
            let array_index = (self.queue.first_index + self.index) & self.queue.capacity_mask;
            self.index += 1;
            Some(self.queue.span[array_index])
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = (self.queue.count - self.index) as usize;
        (remaining, Some(remaining))
    }
}

impl<'a, T: Copy> ExactSizeIterator for QuickQueueIterator<'a, T> {}
