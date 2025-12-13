// Translated from BepuPhysics/Constraints/TypeProcessor.cs
//
// TypeProcessor is an abstract class in C# that acts as a vtable / function-pointer-like
// processor for TypeBatch data. It holds no constraint state itself.
// In Rust, we model this as a trait + a concrete struct holding cached metadata.

use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

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
