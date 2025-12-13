// Translated from BepuPhysics/ConstraintReference.cs

use crate::physics::constraints::type_batch::TypeBatch;

/// Reference to a constraint's memory location in the solver.
#[repr(C)]
pub struct ConstraintReference {
    pub(crate) type_batch_pointer: *mut TypeBatch,
    /// Index in the type batch where the constraint is allocated.
    pub index_in_type_batch: i32,
}

impl ConstraintReference {
    /// Creates a new constraint reference from a constraint memory location.
    #[inline(always)]
    pub fn new(type_batch_pointer: *mut TypeBatch, index_in_type_batch: i32) -> Self {
        Self {
            type_batch_pointer,
            index_in_type_batch,
        }
    }

    /// Gets a reference to the type batch holding the constraint.
    #[inline(always)]
    pub unsafe fn type_batch(&self) -> &TypeBatch {
        &*self.type_batch_pointer
    }

    /// Gets a mutable reference to the type batch holding the constraint.
    #[inline(always)]
    pub unsafe fn type_batch_mut(&mut self) -> &mut TypeBatch {
        &mut *self.type_batch_pointer
    }
}
