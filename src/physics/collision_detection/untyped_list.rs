// Translated from BepuPhysics/CollisionDetection/UntypedList.cs

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use std::mem;

/// A type-erased list storing elements as raw bytes, used in the collision pipeline.
#[repr(C)]
pub struct UntypedList {
    pub buffer: Buffer<u8>,
    pub count: i32,
    pub byte_count: i32,
    pub element_size_in_bytes: i32,
}

impl Default for UntypedList {
    fn default() -> Self {
        Self {
            buffer: Buffer::default(),
            count: 0,
            byte_count: 0,
            element_size_in_bytes: 0,
        }
    }
}

impl UntypedList {
    /// Creates a new untyped list with the given element size and initial capacity.
    #[inline(always)]
    pub fn new(
        element_size_in_bytes: i32,
        initial_capacity_in_elements: i32,
        pool: &mut BufferPool,
    ) -> Self {
        let buffer = pool.take_at_least(initial_capacity_in_elements * element_size_in_bytes);
        Self {
            buffer,
            count: 0,
            byte_count: 0,
            element_size_in_bytes,
        }
    }

    /// Ensures the list has capacity for at least the target byte count.
    #[inline(always)]
    pub fn ensure_capacity_in_bytes(
        &mut self,
        element_size_in_bytes: i32,
        target_capacity_in_bytes: i32,
        pool: &mut BufferPool,
    ) {
        debug_assert!(
            self.element_size_in_bytes == 0 || self.element_size_in_bytes == element_size_in_bytes,
            "Ensuring capacity should not change an already existing list's element size in bytes."
        );
        self.element_size_in_bytes = element_size_in_bytes;
        if self.buffer.len() < target_capacity_in_bytes {
            pool.resize_to_at_least(&mut self.buffer, target_capacity_in_bytes, self.byte_count);
        }
    }

    #[cfg(debug_assertions)]
    fn validate(&self) {
        debug_assert!(self.element_size_in_bytes > 0);
    }

    #[cfg(not(debug_assertions))]
    fn validate(&self) {}

    /// Gets a reference to an element at the given byte index.
    #[inline(always)]
    pub unsafe fn get_from_bytes<T>(&self, byte_index: i32) -> &T {
        self.validate();
        &*(self.buffer.as_ptr().add(byte_index as usize) as *const T)
    }

    /// Gets a mutable reference to an element at the given byte index.
    #[inline(always)]
    pub unsafe fn get_from_bytes_mut<T>(&mut self, byte_index: i32) -> &mut T {
        self.validate();
        &mut *(self.buffer.as_mut_ptr().add(byte_index as usize) as *mut T)
    }

    /// Gets a reference to an element at the given element index.
    #[inline(always)]
    pub unsafe fn get<T>(&self, index: i32) -> &T {
        self.validate();
        &*(self.buffer.as_ptr() as *const T).add(index as usize)
    }

    /// Gets a mutable reference to an element at the given element index.
    #[inline(always)]
    pub unsafe fn get_mut<T>(&mut self, index: i32) -> &mut T {
        self.validate();
        &mut *(self.buffer.as_mut_ptr() as *mut T).add(index as usize)
    }

    /// Allocates space for one element without bounds checking. Returns a raw pointer to the allocated space.
    #[inline(always)]
    pub unsafe fn allocate_unsafely_raw(&mut self) -> *mut u8 {
        self.validate();
        let new_size = self.byte_count + self.element_size_in_bytes;
        self.count += 1;
        let byte_index = self.byte_count;
        self.byte_count = new_size;
        self.buffer.as_mut_ptr().add(byte_index as usize)
    }

    /// Allocates space for one element without bounds checking. Returns a mutable reference to the allocated space.
    #[inline(always)]
    pub unsafe fn allocate_unsafely<T>(&mut self) -> &mut T {
        self.validate();
        debug_assert!(mem::size_of::<T>() as i32 == self.element_size_in_bytes);
        let new_size = self.byte_count + mem::size_of::<T>() as i32;
        // If we store only byte count, we'd have to divide to get the element index.
        // If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
        // So we just store both. It's pretty cheap and simple.
        self.count += 1;
        let byte_index = self.byte_count;
        self.byte_count = new_size;
        &mut *(self.buffer.as_mut_ptr().add(byte_index as usize) as *mut T)
    }

    /// Allocates an element in the list, initializing the backing buffer if needed.
    /// Returns the byte index of the allocated element.
    #[inline(always)]
    pub unsafe fn allocate(
        &mut self,
        element_size_in_bytes: i32,
        minimum_element_count: i32,
        pool: &mut BufferPool,
    ) -> i32 {
        let new_size = self.byte_count + element_size_in_bytes;
        if !self.buffer.allocated() {
            // This didn't exist at all before; create a new entry for this type.
            self.element_size_in_bytes = element_size_in_bytes;
            self.buffer = pool.take_at_least(i32::max(
                new_size,
                minimum_element_count * element_size_in_bytes,
            ));
        } else {
            debug_assert!(element_size_in_bytes == self.element_size_in_bytes);
            if new_size > self.buffer.len() {
                // This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                let mut new_buffer: Buffer<u8> = pool.take_at_least(new_size);
                std::ptr::copy_nonoverlapping(
                    self.buffer.as_ptr(),
                    new_buffer.as_mut_ptr(),
                    self.buffer.len() as usize,
                );
                pool.return_unsafely(self.buffer.id());
                self.buffer = new_buffer;
            }
        }
        debug_assert!(self.buffer.len() >= new_size);
        // If we store only byte count, we'd have to divide to get the element index.
        // If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
        // So we just store both. It's pretty cheap and simple.
        self.count += 1;
        let byte_index = self.byte_count;
        self.byte_count = new_size;
        byte_index
    }

    /// Allocates an element of type T.
    #[inline(always)]
    pub unsafe fn allocate_typed<T>(
        &mut self,
        minimum_element_count: i32,
        pool: &mut BufferPool,
    ) -> i32 {
        let element_size_in_bytes = mem::size_of::<T>() as i32;
        self.allocate(element_size_in_bytes, minimum_element_count, pool)
    }

    /// Adds an element of type T to the list, returning the byte index.
    #[inline(always)]
    pub unsafe fn add<T: Copy>(
        &mut self,
        data: &T,
        minimum_count: i32,
        pool: &mut BufferPool,
    ) -> i32 {
        let byte_index = self.allocate_typed::<T>(minimum_count, pool);
        *(self.buffer.as_mut_ptr().add(byte_index as usize) as *mut T) = *data;
        byte_index
    }
}
