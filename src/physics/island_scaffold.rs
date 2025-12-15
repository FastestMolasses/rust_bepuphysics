// Translated from BepuPhysics/IslandScaffold.cs

use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::sequential_fallback_batch::SequentialFallbackBatch;
use crate::physics::solver::Solver;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Helper enumerator that collects body indices from a constraint.
pub(crate) struct ConstraintHandleEnumerator {
    pub body_indices: *mut i32,
    pub count: i32,
}

impl IForEach<i32> for ConstraintHandleEnumerator {
    #[inline(always)]
    fn loop_body(&mut self, i: i32) {
        unsafe {
            *self.body_indices.add(self.count as usize) = i;
            self.count += 1;
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct IslandScaffoldTypeBatch {
    pub type_id: i32,
    pub handles: QuickList<i32>,
}

impl IslandScaffoldTypeBatch {
    pub fn new(pool: &mut BufferPool, type_id: i32, initial_type_batch_size: i32) -> Self {
        Self {
            type_id,
            handles: QuickList::with_capacity(initial_type_batch_size, pool),
        }
    }
}

// TODO: There's quite a bit of redundant logic here with the constraint batch and solver. Very likely that we could share more.
#[derive(Clone, Copy)]
pub(crate) struct IslandScaffoldConstraintBatch {
    pub type_id_to_index: Buffer<i32>,
    pub type_batches: QuickList<IslandScaffoldTypeBatch>,
    // Note that we use *indices* during island construction, not handles.
    pub referenced_body_indices: IndexSet,
}

impl IslandScaffoldConstraintBatch {
    pub unsafe fn new(solver: &Solver, pool: &mut BufferPool, batch_index: i32) -> Self {
        let type_proc_len = solver.type_processors.len() as i32;
        let type_id_to_index: Buffer<i32> = pool.take_at_least(type_proc_len);
        // Initialize all to -1 (0xFF bytes)
        std::ptr::write_bytes(type_id_to_index.as_ptr() as *mut u8, 0xFF, type_id_to_index.len() as usize * std::mem::size_of::<i32>());

        let type_batches = QuickList::with_capacity(type_proc_len, pool);
        let active_count = (*solver.bodies).active_set().count;
        let referenced_body_indices = if batch_index < solver.fallback_batch_threshold() {
            IndexSet::new(pool, active_count)
        } else {
            IndexSet::empty()
        };

        Self {
            type_id_to_index,
            type_batches,
            referenced_body_indices,
        }
    }

    #[inline(always)]
    fn get_or_create_type_batch(&mut self, type_id: i32, solver: &Solver, pool: &mut BufferPool) -> &mut IslandScaffoldTypeBatch {
        let id_map = self.type_id_to_index.get_mut(type_id);
        if *id_map == -1 {
            *id_map = self.type_batches.count;
            let type_batch = self.type_batches.allocate_unsafely();
            *type_batch = IslandScaffoldTypeBatch::new(pool, type_id, solver.get_minimum_capacity_for_type(type_id));
            type_batch
        } else {
            self.type_batches.get_mut(*id_map)
        }
    }

    pub unsafe fn try_add(
        &mut self,
        constraint_handle: ConstraintHandle,
        dynamic_body_indices: &[i32],
        type_id: i32,
        batch_index: i32,
        solver: &Solver,
        pool: &mut BufferPool,
        fallback_batch: &mut SequentialFallbackBatch,
    ) -> bool {
        if batch_index == solver.fallback_batch_threshold() || self.referenced_body_indices.can_fit(dynamic_body_indices) {
            let type_batch = self.get_or_create_type_batch(type_id, solver, pool);
            debug_assert!(type_batch.type_id == type_id);
            type_batch.handles.add(constraint_handle.0, pool);
            if batch_index < solver.fallback_batch_threshold() {
                for &idx in dynamic_body_indices {
                    self.referenced_body_indices.add_unsafely(idx);
                }
            } else {
                // This is the fallback batch, so we need to fill it with relevant information.
                let bodies = &*solver.bodies;
                let mut dynamic_body_handles: Vec<BodyHandle> = Vec::with_capacity(dynamic_body_indices.len());
                for &idx in dynamic_body_indices {
                    dynamic_body_handles.push(*bodies.active_set().index_to_handle.get(idx));
                }
                fallback_batch.allocate_for_inactive(&dynamic_body_handles, bodies, pool, bodies.minimum_constraint_capacity_per_body);
            }
            true
        } else {
            false
        }
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.type_batches.count {
            self.type_batches.get_mut(i).handles.dispose(pool);
        }
        self.type_batches.dispose(pool);
        pool.return_buffer(&mut self.type_id_to_index);
        if self.referenced_body_indices.flags.allocated() {
            self.referenced_body_indices.dispose(pool);
        }
    }
}

/// Represents the constraint batch structure and all references in an island.
/// Holds everything necessary to create and gather a full island.
pub(crate) struct IslandScaffold {
    pub body_indices: QuickList<i32>,
    pub protobatches: QuickList<IslandScaffoldConstraintBatch>,
    pub fallback_batch: SequentialFallbackBatch,
}

impl IslandScaffold {
    pub unsafe fn new(
        body_indices: &QuickList<i32>,
        constraint_handles: &QuickList<ConstraintHandle>,
        solver: &Solver,
        pool: &mut BufferPool,
    ) -> Self {
        debug_assert!(body_indices.count > 0, "Don't create islands with no bodies!");
        // Create a copy of the body indices with just enough space.
        let mut island_body_indices = QuickList::with_capacity(body_indices.count, pool);
        body_indices.span.copy_to(0, &mut island_body_indices.span, 0, body_indices.count);
        island_body_indices.count = body_indices.count;

        let mut scaffold = Self {
            body_indices: island_body_indices,
            protobatches: QuickList::with_capacity(solver.active_set().batches.count, pool),
            fallback_batch: SequentialFallbackBatch::default(),
        };

        for i in 0..constraint_handles.count {
            scaffold.add_constraint(*constraint_handles.get(i), solver, pool);
        }

        scaffold
    }

    unsafe fn add_constraint(&mut self, constraint_handle: ConstraintHandle, solver: &Solver, pool: &mut BufferPool) {
        let location = *solver.handle_to_constraint.get(constraint_handle.0);
        let type_id = location.type_id;
        let type_processor = solver.type_processors[type_id as usize].as_ref().unwrap();
        let bodies_per_constraint = type_processor.bodies_per_constraint;
        let mut body_indices_buf = vec![0i32; bodies_per_constraint as usize];

        let mut enumerator = ConstraintHandleEnumerator {
            body_indices: body_indices_buf.as_mut_ptr(),
            count: 0,
        };
        solver.enumerate_connected_dynamic_bodies(constraint_handle, &mut enumerator);
        let dynamic_body_indices = &body_indices_buf[..enumerator.count as usize];

        for batch_index in 0..self.protobatches.count {
            if self.protobatches.get_mut(batch_index).try_add(
                constraint_handle,
                dynamic_body_indices,
                type_id,
                batch_index,
                solver,
                pool,
                &mut self.fallback_batch,
            ) {
                return;
            }
        }

        // No existing batch could hold it; create a new one.
        if self.protobatches.span.len() == self.protobatches.count {
            self.protobatches.ensure_capacity(self.protobatches.count + 1, pool);
        }
        let new_batch_index = self.protobatches.count;
        let new_batch = self.protobatches.allocate_unsafely();
        *new_batch = IslandScaffoldConstraintBatch::new(solver, pool, new_batch_index);
        let added = new_batch.try_add(
            constraint_handle,
            dynamic_body_indices,
            type_id,
            new_batch_index,
            solver,
            pool,
            &mut self.fallback_batch,
        );
        debug_assert!(added, "If we created a new batch for a constraint, it must successfully add.");
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        self.body_indices.dispose(pool);
        for k in 0..self.protobatches.count {
            self.protobatches.get_mut(k).dispose(pool);
        }
        self.protobatches.dispose(pool);
        self.fallback_batch.dispose(pool);
    }
}
