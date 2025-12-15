// Translated from BepuPhysics/Constraints/TypeBatch.cs

use crate::physics::handles::ConstraintHandle;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Stores the raw AOSOA formatted data associated with constraints in a type batch.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TypeBatch {
    // Note the constraint data is all stored untyped. It is up to the user to read from these pointers correctly.
    pub body_references: Buffer<u8>,
    pub prestep_data: Buffer<u8>,
    pub accumulated_impulses: Buffer<u8>,
    pub index_to_handle: Buffer<ConstraintHandle>,
    pub constraint_count: i32,
    pub type_id: i32,
}

impl TypeBatch {
    /// Returns the capacity of the type batch in terms of individual constraints.
    /// This is determined by the length of the IndexToHandle buffer.
    #[inline(always)]
    pub fn capacity(&self) -> i32 {
        self.index_to_handle.len()
    }

    #[inline(always)]
    pub fn bundle_count(&self) -> usize {
        BundleIndexing::get_bundle_count(self.constraint_count as usize)
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.body_references);
        pool.return_buffer(&mut self.prestep_data);
        pool.return_buffer(&mut self.accumulated_impulses);
        pool.return_buffer(&mut self.index_to_handle);
    }
}

impl Default for TypeBatch {
    fn default() -> Self {
        Self {
            body_references: Buffer::default(),
            prestep_data: Buffer::default(),
            accumulated_impulses: Buffer::default(),
            index_to_handle: Buffer::default(),
            constraint_count: 0,
            type_id: 0,
        }
    }
}
