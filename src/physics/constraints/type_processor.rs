// Translated from BepuPhysics/Constraints/TypeProcessor.cs
//
// TypeProcessor is an abstract class in C# that acts as a vtable / function-pointer-like
// processor for TypeBatch data. It holds no constraint state itself.
// In Rust, we model this as a trait + a concrete struct holding cached metadata.

use crate::physics::bodies::Bodies;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Collects body references from active constraints and converts them into properly flagged body handles.
/// Used by the convenience `transfer_constraint_auto_collect` to gather dynamic body handles
/// and encoded body indices before calling the full `transfer_constraint`.
struct ActiveKinematicFlaggedBodyHandleCollector<'a> {
    bodies: &'a Bodies,
    dynamic_body_handles: *mut i32,
    dynamic_count: *mut usize,
    encoded_body_indices: *mut i32,
    index_count: usize,
}

impl<'a> IForEach<i32> for ActiveKinematicFlaggedBodyHandleCollector<'a> {
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            if Bodies::is_encoded_dynamic_reference(encoded_body_index) {
                let count = *self.dynamic_count;
                *self.dynamic_body_handles.add(count) = self.bodies.active_set().index_to_handle.get(encoded_body_index).0;
                *self.dynamic_count = count + 1;
            }
            *self.encoded_body_indices.add(self.index_count) = encoded_body_index;
            self.index_count += 1;
        }
    }
}

/// Superclass of constraint type batch processors. Responsible for interpreting raw type batches
/// for the purposes of bookkeeping and solving.
///
/// This holds no actual constraint state. A solver creates a unique type processor for each
/// registered constraint type, and all instances are held in untyped memory.
/// Conceptually, the solver's array of TypeProcessors are like C function pointers.
pub trait ITypeProcessor {
    /// Gets the number of bodies associated with each constraint in this type processor.
    fn bodies_per_constraint(&self) -> i32;

    /// Gets the number of degrees of freedom that each constraint constrains.
    fn constrained_degrees_of_freedom(&self) -> i32;

    /// Gets the type id assigned to this processor.
    fn type_id(&self) -> i32;

    /// Whether this constraint type requires incremental substep updates (e.g. contacts).
    fn requires_incremental_substep_updates(&self) -> bool {
        false
    }

    /// Allocates a slot in the type batch (non-fallback).
    fn allocate_in_type_batch(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: &mut BufferPool,
    ) -> i32;

    /// Allocates a slot in the type batch for a fallback batch.
    fn allocate_in_type_batch_for_fallback(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: &mut BufferPool,
    ) -> i32;

    /// Removes a constraint at the given index.
    fn remove(
        &self,
        type_batch: &mut TypeBatch,
        index: i32,
        handles_to_constraints: &mut Buffer<ConstraintLocation>,
        is_fallback: bool,
    );

    /// Initializes a type batch with the given capacity.
    fn initialize(&self, type_batch: &mut TypeBatch, initial_capacity: i32, pool: &mut BufferPool);

    /// Resizes a type batch.
    fn resize(&self, type_batch: &mut TypeBatch, new_capacity: i32, pool: &mut BufferPool);

    /// Scales all accumulated impulses by the given factor.
    fn scale_accumulated_impulses(&self, type_batch: &mut TypeBatch, scale: f32);

    /// Copies constraint data from a sleeping set's type batch into the active set's type batch.
    fn copy_sleeping_to_active(
        &self,
        _source_set: i32,
        _batch_index: i32,
        _source_type_batch_index: i32,
        _target_type_batch_index: i32,
        _source_start: i32,
        _target_start: i32,
        _count: i32,
        _bodies: &crate::physics::bodies::Bodies,
        _solver: &crate::physics::solver::Solver,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Adds body handles referenced by the waking type batch to the batch's referenced handles set.
    fn add_waking_body_handles_to_batch_references(
        &self,
        _type_batch: &TypeBatch,
        _target_batch_referenced_handles: &mut IndexSet,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Adds constraints from a sleeping fallback type batch into the active set's fallback type batch.
    /// Unlike bulk copy, this goes through per-constraint allocation to maintain the fallback batch invariant
    /// (no duplicate bodies per bundle).
    fn add_sleeping_to_active_for_fallback(
        &self,
        _source_set: i32,
        _source_type_batch_index: i32,
        _target_type_batch_index: i32,
        _bodies: &crate::physics::bodies::Bodies,
        _solver: &crate::physics::solver::Solver,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Transfers a constraint from one batch's type batch to another batch's type batch of the same type.
    /// Uses pre-computed dynamic body handles and encoded body indices.
    fn transfer_constraint(
        &self,
        _source_type_batch: &mut TypeBatch,
        _source_batch_index: i32,
        _index_in_type_batch: i32,
        _solver: *mut crate::physics::solver::Solver,
        _bodies: *mut crate::physics::bodies::Bodies,
        _target_batch_index: i32,
        _dynamic_body_handles: &[crate::physics::handles::BodyHandle],
        _encoded_body_indices: &[i32],
    ) {
        // Default: no-op stub. Concrete type processors will override with lane-copy logic.
    }

    /// Convenience wrapper that auto-collects dynamic body handles and encoded body indices,
    /// then calls the full `transfer_constraint`.
    fn transfer_constraint_auto_collect(
        &self,
        source_type_batch: &mut TypeBatch,
        source_batch_index: i32,
        index_in_type_batch: i32,
        solver: *mut crate::physics::solver::Solver,
        bodies: *mut crate::physics::bodies::Bodies,
        target_batch_index: i32,
    ) {
        unsafe {
            let bodies_per_constraint = self.bodies_per_constraint() as usize;
            let mut dynamic_body_handles = vec![0i32; bodies_per_constraint];
            let mut encoded_body_indices = vec![0i32; bodies_per_constraint];
            let mut dynamic_count = 0usize;
            let bodies_ref = &*bodies;

            // Enumerate raw body references to collect handles and indices
            (*solver).enumerate_connected_raw_body_references_from_type_batch(
                &*source_type_batch,
                index_in_type_batch,
                &mut ActiveKinematicFlaggedBodyHandleCollector {
                    bodies: bodies_ref,
                    dynamic_body_handles: dynamic_body_handles.as_mut_ptr(),
                    dynamic_count: &mut dynamic_count,
                    encoded_body_indices: encoded_body_indices.as_mut_ptr(),
                    index_count: 0,
                },
            );

            let dynamic_handles: Vec<crate::physics::handles::BodyHandle> = dynamic_body_handles[..dynamic_count]
                .iter()
                .map(|&v| crate::physics::handles::BodyHandle(v))
                .collect();

            self.transfer_constraint(
                source_type_batch,
                source_batch_index,
                index_in_type_batch,
                solver,
                bodies,
                target_batch_index,
                &dynamic_handles,
                &encoded_body_indices,
            );
        }
    }

    /// Updates body references within a constraint when a body is moved in memory.
    /// Returns true if the moved body is kinematic.
    fn update_for_body_memory_move(
        &self,
        _type_batch: &mut TypeBatch,
        _index_in_type_batch: i32,
        _body_index_in_constraint: i32,
        _new_body_location: i32,
    ) -> bool {
        // Default: no-op stub.
        false
    }

    /// Returns the capacity of the given type batch.
    fn capacity(&self, _type_batch: &TypeBatch) -> i32 {
        // Default stub
        0
    }

    /// Warm-starts the constraints in the given type batch. The warm start applies previously
    /// accumulated impulses to the body velocities to accelerate convergence.
    ///
    /// The integration flags, callbacks, and integration mode are handled internally by each
    /// concrete type processor. This simplified signature is for the solver's main dispatch.
    fn warm_start(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        // Default: no-op. Concrete implementations will override.
        let _ = (type_batch, bodies, dt, inverse_dt, start_bundle, exclusive_end_bundle);
    }

    /// Solves the constraint velocity iteration for the given bundle range.
    fn solve(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        // Default: no-op. Concrete implementations will override.
        let _ = (type_batch, bodies, dt, inverse_dt, start_bundle, exclusive_end_bundle);
    }

    /// Performs an incremental update for substeps beyond the first.
    /// Only called for constraint types where `requires_incremental_substep_updates` returns true.
    fn incrementally_update_for_substep(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        // Default: debug fail, should not be called unless the type supports it.
        debug_assert!(false, "An incremental update was scheduled for a type batch that does not have a contact data update implementation.");
        let _ = (type_batch, bodies, dt, inverse_dt, start_bundle, exclusive_end_bundle);
    }
}

/// Concrete metadata holder for a type processor. Stores cached type id and bodies-per-constraint
/// alongside a boxed trait implementation for virtual dispatch.
pub struct TypeProcessor {
    pub type_id: i32,
    pub bodies_per_constraint: i32,
    pub constrained_degrees_of_freedom: i32,
    inner: Box<dyn ITypeProcessor>,
}

impl TypeProcessor {
    pub fn new(type_id: i32, inner: Box<dyn ITypeProcessor>) -> Self {
        let bodies_per_constraint = inner.bodies_per_constraint();
        let constrained_degrees_of_freedom = inner.constrained_degrees_of_freedom();
        Self {
            type_id,
            bodies_per_constraint,
            constrained_degrees_of_freedom,
            inner,
        }
    }

    #[inline(always)]
    pub fn inner(&self) -> &dyn ITypeProcessor {
        &*self.inner
    }

    #[inline(always)]
    pub fn inner_mut(&mut self) -> &mut dyn ITypeProcessor {
        &mut *self.inner
    }
}
