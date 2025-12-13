// Translated from BepuPhysics/ConstraintBatch.cs

use crate::physics::constraints::type_batch::TypeBatch;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Contains a set of type batches whose constraints share no body references.
#[derive(Clone, Copy)]
pub struct ConstraintBatch {
    // Note that both active and inactive constraint batches share the same data layout.
    // This means we have a type id->index mapping in inactive islands.
    // The type id->index mapping is required because the solver's handle->constraint indirection
    // stores a type id. If it stored a batch-specific type *index*, then all mappings associated
    // with a type batch's constraints would have to be updated when a type batch changes slots
    // due to a removal.
    pub type_index_to_type_batch_index: Buffer<i32>,
    pub type_batches: QuickList<TypeBatch>,
}

impl ConstraintBatch {
    pub fn new(pool: &mut BufferPool, initial_type_count_estimate: i32) -> Self {
        let mut result = Self {
            type_index_to_type_batch_index: Buffer::default(),
            type_batches: QuickList::with_capacity(initial_type_count_estimate, pool),
        };
        result.resize_type_map(pool, initial_type_count_estimate);
        result
    }

    fn resize_type_map(&mut self, pool: &mut BufferPool, new_size: i32) {
        let old_length = self.type_index_to_type_batch_index.len();
        debug_assert!(
            old_length != BufferPool::get_capacity_for_count::<i32>(new_size),
            "Shouldn't resize if nothing changes."
        );
        pool.resize_to_at_least(&mut self.type_index_to_type_batch_index, new_size, old_length);
        for i in old_length..self.type_index_to_type_batch_index.len() {
            *self.type_index_to_type_batch_index.get_mut(i) = -1;
        }
    }

    pub(crate) fn ensure_type_map_size(&mut self, pool: &mut BufferPool, target_size: i32) {
        if target_size > self.type_index_to_type_batch_index.len() {
            self.resize_type_map(pool, target_size);
        }
    }

    /// Gets a reference to the type batch matching the given type id.
    /// Requires that there exists at least one constraint in the type batch.
    #[inline(always)]
    pub fn get_type_batch(&self, type_id: i32) -> &TypeBatch {
        let type_batch_index = *self.type_index_to_type_batch_index.get(type_id);
        self.type_batches.get(type_batch_index)
    }

    /// Gets a mutable reference to the type batch matching the given type id.
    #[inline(always)]
    pub fn get_type_batch_mut(&mut self, type_id: i32) -> &mut TypeBatch {
        let type_batch_index = *self.type_index_to_type_batch_index.get(type_id);
        self.type_batches.get_mut(type_batch_index)
    }

    /// Gets a pointer to the type batch matching the given type id.
    #[inline(always)]
    pub unsafe fn get_type_batch_pointer(&mut self, type_id: i32) -> *mut TypeBatch {
        let type_batch_index = *self.type_index_to_type_batch_index.get(type_id);
        self.type_batches.get_pointer_mut(type_batch_index)
    }

    pub(crate) unsafe fn create_new_type_batch(
        &mut self,
        type_id: i32,
        type_processor: &dyn crate::physics::constraints::type_processor::ITypeProcessor,
        initial_capacity: i32,
        pool: &mut BufferPool,
    ) -> *mut TypeBatch {
        let new_index = self.type_batches.count;
        self.type_batches.ensure_capacity(self.type_batches.count + 1, pool);
        *self.type_index_to_type_batch_index.get_mut(type_id) = new_index;
        let type_batch = self.type_batches.allocate_unsafely();
        *type_batch = TypeBatch::default();
        type_processor.initialize(type_batch, initial_capacity, pool);
        type_batch as *mut TypeBatch
    }

    pub(crate) unsafe fn get_or_create_type_batch(
        &mut self,
        type_id: i32,
        type_processor: &dyn crate::physics::constraints::type_processor::ITypeProcessor,
        initial_capacity: i32,
        pool: &mut BufferPool,
    ) -> *mut TypeBatch {
        if type_id >= self.type_index_to_type_batch_index.len() {
            // While we only request a capacity one slot larger, buffer pools always return a
            // power of 2, so this isn't going to cause tons of unnecessary resizing.
            self.resize_type_map(pool, type_id + 1);
            return self.create_new_type_batch(type_id, type_processor, initial_capacity, pool);
        }

        let type_batch_index = *self.type_index_to_type_batch_index.get(type_id);
        if type_batch_index == -1 {
            self.create_new_type_batch(type_id, type_processor, initial_capacity, pool)
        } else {
            self.type_batches.get_pointer_mut(type_batch_index)
        }
    }

    /// Removes a type batch if it has no more constraints.
    pub fn remove_type_batch_if_empty(
        &mut self,
        type_batch_index: i32,
        pool: &mut BufferPool,
    ) {
        let constraint_count = self.type_batches.get(type_batch_index).constraint_count;
        if constraint_count == 0 {
            let constraint_type_id = self.type_batches.get(type_batch_index).type_id;
            *self.type_index_to_type_batch_index.get_mut(constraint_type_id) = -1;
            // Dispose before removal, or else we'll end up disposing whatever type batch
            // moves to occupy the newly empty slot.
            self.type_batches.get_mut(type_batch_index).dispose(pool);
            self.type_batches.fast_remove_at(type_batch_index);
            if type_batch_index < self.type_batches.count {
                // If we swapped anything into the removed slot, update the type index mapping.
                let swapped_type_id = self.type_batches.get(type_batch_index).type_id;
                *self.type_index_to_type_batch_index.get_mut(swapped_type_id) = type_batch_index;
            }
        }
    }

    /// Releases all memory used by all type batches and clears the mapping.
    pub fn clear(&mut self, pool: &mut BufferPool) {
        for i in 0..self.type_batches.count {
            self.type_batches.get_mut(i).dispose(pool);
        }
        // Since there are no more type batches, the mapping must be cleared out.
        for type_id in 0..self.type_index_to_type_batch_index.len() {
            *self.type_index_to_type_batch_index.get_mut(type_id) = -1;
        }
        self.type_batches.clear();
    }

    /// Releases all memory used by the batch.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.type_batches.count {
            self.type_batches.get_mut(i).dispose(pool);
        }
        pool.return_buffer(&mut self.type_index_to_type_batch_index);
        self.type_index_to_type_batch_index = Buffer::default();
        self.type_batches.dispose(pool);
        self.type_batches = QuickList::default();
    }
}

impl Default for ConstraintBatch {
    fn default() -> Self {
        Self {
            type_index_to_type_batch_index: Buffer::default(),
            type_batches: QuickList::default(),
        }
    }
}
