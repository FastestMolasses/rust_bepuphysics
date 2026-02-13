// Translated from BepuPhysics/CollidableProperty.cs
//
// Convenience collection that stores extra properties about bodies and statics,
// indexed by the body or static handle.
// This is built for use cases relying on random access like the narrow phase.

use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::handles::{BodyHandle, StaticHandle};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Convenience collection that stores extra properties about bodies and statics,
/// indexed by the body or static handle.
///
/// This stores separate buffers for body data and static data since body handles
/// and static handles occupy independent namespaces.
pub struct CollidableProperty<T: Copy + Default> {
    pool: *mut BufferPool,
    body_data: Buffer<T>,
    static_data: Buffer<T>,
}

impl<T: Copy + Default> CollidableProperty<T> {
    /// Constructs a new collection with deferred initialization.
    pub fn new(pool: *mut BufferPool) -> Self {
        Self {
            pool,
            body_data: Buffer::default(),
            static_data: Buffer::default(),
        }
    }

    /// Constructs a new collection with initial capacity.
    pub fn with_capacity(
        pool: *mut BufferPool,
        body_handle_capacity: i32,
        static_handle_capacity: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        Self {
            pool,
            body_data: pool_ref.take_at_least(body_handle_capacity),
            static_data: pool_ref.take_at_least(static_handle_capacity),
        }
    }

    /// Initializes the property collection if the deferred constructor was used.
    pub fn initialize(&mut self, body_handle_capacity: i32, static_handle_capacity: i32) {
        let pool = unsafe { &mut *self.pool };
        self.body_data = pool.take_at_least(body_handle_capacity);
        self.static_data = pool.take_at_least(static_handle_capacity);
    }

    fn pool_mut(&self) -> &mut BufferPool {
        unsafe { &mut *self.pool }
    }

    /// Gets a reference to the properties associated with a body handle.
    #[inline(always)]
    pub fn get_body(&self, body_handle: BodyHandle) -> &T {
        self.body_data.get(body_handle.0)
    }

    /// Gets a mutable reference to the properties associated with a body handle.
    #[inline(always)]
    pub fn get_body_mut(&mut self, body_handle: BodyHandle) -> &mut T {
        self.body_data.get_mut(body_handle.0)
    }

    /// Gets a reference to the properties associated with a static handle.
    #[inline(always)]
    pub fn get_static(&self, static_handle: StaticHandle) -> &T {
        self.static_data.get(static_handle.0)
    }

    /// Gets a mutable reference to the properties associated with a static handle.
    #[inline(always)]
    pub fn get_static_mut(&mut self, static_handle: StaticHandle) -> &mut T {
        self.static_data.get_mut(static_handle.0)
    }

    /// Gets a reference to the properties associated with a collidable reference.
    #[inline(always)]
    pub fn get_collidable(&self, collidable: CollidableReference) -> &T {
        if collidable.mobility() == CollidableMobility::Static {
            self.get_static(collidable.static_handle())
        } else {
            self.get_body(collidable.body_handle())
        }
    }

    /// Gets a mutable reference to the properties associated with a collidable reference.
    #[inline(always)]
    pub fn get_collidable_mut(&mut self, collidable: CollidableReference) -> &mut T {
        if collidable.mobility() == CollidableMobility::Static {
            self.get_static_mut(collidable.static_handle())
        } else {
            self.get_body_mut(collidable.body_handle())
        }
    }

    /// Ensures there is space for a given body handle and returns a mutable reference.
    #[inline(always)]
    pub fn allocate_body(
        &mut self,
        body_handle: BodyHandle,
        highest_possibly_claimed_id: i32,
    ) -> &mut T {
        if body_handle.0 >= self.body_data.len() {
            let target_count =
                BufferPool::get_capacity_for_count::<T>(highest_possibly_claimed_id + 1);
            debug_assert!(target_count > self.body_data.len());
            let copy_count = self.body_data.len().min(highest_possibly_claimed_id + 1);
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(&mut self.body_data, target_count, copy_count);
        }
        self.body_data.get_mut(body_handle.0)
    }

    /// Ensures there is space for a given static handle and returns a mutable reference.
    #[inline(always)]
    pub fn allocate_static(
        &mut self,
        static_handle: StaticHandle,
        highest_possibly_claimed_id: i32,
    ) -> &mut T {
        if static_handle.0 >= self.static_data.len() {
            let target_count =
                BufferPool::get_capacity_for_count::<T>(highest_possibly_claimed_id + 1);
            debug_assert!(target_count > self.static_data.len());
            let copy_count = self.static_data.len().min(highest_possibly_claimed_id + 1);
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(&mut self.static_data, target_count, copy_count);
        }
        self.static_data.get_mut(static_handle.0)
    }

    /// Ensures that the internal structures have at least the given capacity for bodies.
    pub fn ensure_body_capacity(&mut self, capacity: i32, highest_possibly_claimed_id: i32) {
        let target_count = BufferPool::get_capacity_for_count::<T>(
            (highest_possibly_claimed_id + 1).max(capacity),
        );
        if target_count > self.body_data.len() {
            let copy_count = self.body_data.len().min(highest_possibly_claimed_id + 1);
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(&mut self.body_data, target_count, copy_count);
        }
    }

    /// Ensures that the internal structures have at least the given capacity for statics.
    pub fn ensure_static_capacity(&mut self, capacity: i32, highest_possibly_claimed_id: i32) {
        let target_count = BufferPool::get_capacity_for_count::<T>(
            (highest_possibly_claimed_id + 1).max(capacity),
        );
        if target_count > self.static_data.len() {
            let copy_count = self.static_data.len().min(highest_possibly_claimed_id + 1);
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(&mut self.static_data, target_count, copy_count);
        }
    }

    /// Compacts the memory used by the collection for bodies.
    pub fn compact_bodies(&mut self, highest_possibly_claimed_id: i32) {
        let target_count = BufferPool::get_capacity_for_count::<T>(highest_possibly_claimed_id + 1);
        if target_count < self.body_data.len() {
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(
                &mut self.body_data,
                target_count,
                highest_possibly_claimed_id + 1,
            );
        }
    }

    /// Compacts the memory used by the collection for statics.
    pub fn compact_statics(&mut self, highest_possibly_claimed_id: i32) {
        let target_count = BufferPool::get_capacity_for_count::<T>(highest_possibly_claimed_id + 1);
        if target_count < self.static_data.len() {
            let pool = unsafe { &mut *self.pool };
            pool.resize_to_at_least(
                &mut self.static_data,
                target_count,
                highest_possibly_claimed_id + 1,
            );
        }
    }

    /// Returns all held resources.
    pub fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        pool.return_buffer(&mut self.body_data);
        pool.return_buffer(&mut self.static_data);
    }
}
