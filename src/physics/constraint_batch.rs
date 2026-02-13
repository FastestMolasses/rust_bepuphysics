// Translated from BepuPhysics/ConstraintBatch.cs

use crate::physics::bodies::Bodies;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Used by `RemoveBodyHandlesFromBatchForConstraint` to remove each body's handle
/// from the batch's referenced handle set.
struct ActiveBodyHandleRemover {
    bodies: *const Bodies,
    handles: *mut IndexSet,
}

impl IForEach<i32> for ActiveBodyHandleRemover {
    #[inline(always)]
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            if Bodies::is_encoded_dynamic_reference(encoded_body_index) {
                let body_index = encoded_body_index & Bodies::BODY_REFERENCE_MASK;
                let handle = (*self.bodies)
                    .active_set()
                    .index_to_handle
                    .get(body_index)
                    .0;
                (*self.handles).unset(handle);
            }
        }
    }
}

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
        pool.resize_to_at_least(
            &mut self.type_index_to_type_batch_index,
            new_size,
            old_length,
        );
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
        self.type_batches
            .ensure_capacity(self.type_batches.count + 1, pool);
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
    pub fn remove_type_batch_if_empty(&mut self, type_batch_index: i32, pool: &mut BufferPool) {
        let constraint_count = self.type_batches.get(type_batch_index).constraint_count;
        if constraint_count == 0 {
            let constraint_type_id = self.type_batches.get(type_batch_index).type_id;
            *self
                .type_index_to_type_batch_index
                .get_mut(constraint_type_id) = -1;
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

    /// Removes body handles associated with a constraint from the batch's referenced handles set.
    /// Only used for non-fallback batches.
    pub unsafe fn remove_body_handles_from_batch_for_constraint(
        &self,
        constraint_type_id: i32,
        index_in_type_batch: i32,
        batch_index: i32,
        solver: &mut crate::physics::solver::Solver,
    ) {
        debug_assert!(
            batch_index <= solver.fallback_batch_threshold(),
            "This should only be used for non-fallback batches"
        );
        let index_set = solver.batch_referenced_handles.get_pointer_mut(batch_index);
        let type_batch_index = *self.type_index_to_type_batch_index.get(constraint_type_id);
        let type_batch = self.type_batches.get(type_batch_index);
        let mut handle_remover = ActiveBodyHandleRemover {
            bodies: solver.bodies,
            handles: index_set,
        };
        solver.enumerate_connected_raw_body_references_from_type_batch(
            type_batch,
            index_in_type_batch,
            &mut handle_remover,
        );
    }

    /// Removes a constraint from this batch.
    pub unsafe fn remove(
        &mut self,
        constraint_type_id: i32,
        index_in_type_batch: i32,
        is_fallback: bool,
        solver: &mut crate::physics::solver::Solver,
    ) {
        let type_batch_index = *self.type_index_to_type_batch_index.get(constraint_type_id);
        let type_batch = self.type_batches.get_mut(type_batch_index);
        debug_assert!(
            *self.type_index_to_type_batch_index.get(constraint_type_id) >= 0,
            "Type index must actually exist within this batch."
        );
        debug_assert!(type_batch.constraint_count > index_in_type_batch);
        let type_processor = solver.type_processors[constraint_type_id as usize]
            .as_ref()
            .unwrap();
        type_processor.inner().remove(
            type_batch,
            index_in_type_batch,
            &mut solver.handle_to_constraint,
            is_fallback,
        );
        self.remove_type_batch_if_empty(type_batch_index, &mut *solver.pool);
    }

    #[inline(always)]
    fn get_target_capacity(type_batch: &TypeBatch, solver: &crate::physics::solver::Solver) -> i32 {
        i32::max(
            type_batch.constraint_count,
            solver.get_minimum_capacity_for_type(type_batch.type_id),
        )
    }

    /// Ensures all type batches have enough capacity for the solver's minimum requirements.
    pub fn ensure_type_batch_capacities(&mut self, solver: &crate::physics::solver::Solver) {
        for i in 0..self.type_batches.count {
            let type_batch = self.type_batches.get(i);
            let target_capacity = Self::get_target_capacity(type_batch, solver);
            if target_capacity > type_batch.index_to_handle.len() {
                let type_id = type_batch.type_id;
                let type_processor = solver.type_processors[type_id as usize].as_ref().unwrap();
                let type_batch_mut = self.type_batches.get_mut(i);
                type_processor
                    .inner()
                    .resize(type_batch_mut, target_capacity, unsafe {
                        &mut *solver.pool
                    });
            }
        }
    }

    /// Resizes all type batches to match the solver's target capacity.
    pub fn resize_type_batch_capacities(&mut self, solver: &crate::physics::solver::Solver) {
        for i in 0..self.type_batches.count {
            let type_batch = self.type_batches.get(i);
            let target_capacity = Self::get_target_capacity(type_batch, solver);
            let type_id = type_batch.type_id;
            let type_processor = solver.type_processors[type_id as usize].as_ref().unwrap();
            let type_batch_mut = self.type_batches.get_mut(i);
            type_processor
                .inner()
                .resize(type_batch_mut, target_capacity, unsafe {
                    &mut *solver.pool
                });
        }
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
