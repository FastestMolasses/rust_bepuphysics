// Translated from BepuPhysics/Statics.cs

use crate::physics::bodies::{Bodies, BroadPhase, IslandAwakener};
use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::collidable::ContinuousDetection;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::StaticHandle;
use crate::physics::static_description::StaticDescription;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;

/// Stores data for a static collidable in the simulation.
/// Statics can be posed and collide, but have no velocity and no dynamic behavior.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Static {
    /// Pose of the static collidable.
    pub pose: RigidPose,
    /// Continuous collision detection settings for this collidable.
    pub continuity: ContinuousDetection,
    /// Shape index used by the static.
    pub shape: TypedIndex,
    /// Index of the collidable in the broad phase.
    pub broad_phase_index: i32,
}

impl Default for Static {
    fn default() -> Self {
        Self {
            pose: RigidPose::default(),
            continuity: ContinuousDetection::discrete(),
            shape: TypedIndex::default(),
            broad_phase_index: -1,
        }
    }
}

/// Trait for filtering which bodies to awaken when a static's state changes.
///
/// Statics do not maintain constraint lists, so we look up bodies in the
/// bounding box of the static. This filter decides which bodies to wake.
pub trait StaticChangeAwakeningFilter {
    /// Whether to allow awakening at all. If false, `should_awaken` is never called.
    fn allow_awakening(&self) -> bool;
    /// Determines whether a specific sleeping body should be forced awake.
    fn should_awaken_body_index(&self, _body_set_index: i32) -> bool;
}

/// Default filter that wakes all candidate bodies (dynamic and kinematic alike).
pub struct DefaultAwakeningFilter;
impl StaticChangeAwakeningFilter for DefaultAwakeningFilter {
    fn allow_awakening(&self) -> bool {
        true
    }
    fn should_awaken_body_index(&self, _body_set_index: i32) -> bool {
        true
    }
}

/// Filter that prevents any bodies from being awoken.
pub struct NoAwakeningFilter;
impl StaticChangeAwakeningFilter for NoAwakeningFilter {
    fn allow_awakening(&self) -> bool {
        false
    }
    fn should_awaken_body_index(&self, _body_set_index: i32) -> bool {
        false
    }
}

/// Collection of allocated statics.
pub struct Statics {
    /// Remaps a static handle integer value to the actual array index of the static.
    pub handle_to_index: Buffer<i32>,
    /// Remaps a static index to its handle.
    pub index_to_handle: Buffer<StaticHandle>,
    /// The set of collidables owned by each static.
    pub statics_buffer: Buffer<Static>,
    /// Pool from which static handles are drawn.
    pub handle_pool: IdPool,
    /// Number of currently allocated statics.
    pub count: i32,

    pool: *mut BufferPool,
    shapes: *mut Shapes,
    bodies: *mut Bodies,
    pub(crate) broad_phase: *mut BroadPhase,
    pub(crate) awakener: *mut IslandAwakener,
}

impl Statics {
    /// Creates a new Statics collection.
    pub fn new(
        pool: *mut BufferPool,
        shapes: *mut Shapes,
        bodies: *mut Bodies,
        broad_phase: *mut BroadPhase,
        initial_capacity: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let handle_pool = IdPool::new(initial_capacity, pool_ref);
        let mut statics = Statics {
            handle_to_index: Buffer::default(),
            index_to_handle: Buffer::default(),
            statics_buffer: Buffer::default(),
            handle_pool,
            count: 0,
            pool,
            shapes,
            bodies,
            broad_phase: broad_phase,
            awakener: std::ptr::null_mut(),
        };
        statics.internal_resize(initial_capacity.max(1));
        statics
    }

    /// Gets the backing pool.
    #[inline(always)]
    fn pool(&self) -> &mut BufferPool {
        unsafe { &mut *self.pool }
    }

    /// Resizes internal buffers to the target capacity.
    fn internal_resize(&mut self, target_capacity: i32) {
        debug_assert!(
            target_capacity > 0,
            "Resize is not meant to be used as Dispose."
        );
        let target_capacity = BufferPool::get_capacity_for_count::<i32>(target_capacity);
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        pool.resize_to_at_least(&mut self.statics_buffer, target_capacity, self.count);
        let pool = unsafe { &mut *pool_ptr };
        pool.resize_to_at_least(&mut self.index_to_handle, target_capacity, self.count);
        let count = self.count;
        let pool = unsafe { &mut *pool_ptr };
        let old_h2i_len = self.handle_to_index.len();
        pool.resize_to_at_least(&mut self.handle_to_index, target_capacity, count);
        // Initialize newly allocated handle→index slots to -1.
        if self.handle_to_index.len() > old_h2i_len || count < self.handle_to_index.len() {
            let start = count;
            let byte_count = ((self.handle_to_index.len() - start) as usize)
                * std::mem::size_of::<i32>();
            if byte_count > 0 {
                unsafe {
                    let ptr = self.handle_to_index.as_mut_ptr().add(start as usize) as *mut u8;
                    std::ptr::write_bytes(ptr, 0xFF, byte_count);
                }
            }
        }
    }

    /// Checks whether a static handle is currently registered.
    #[inline(always)]
    pub fn static_exists(&self, handle: StaticHandle) -> bool {
        if handle.0 < 0 || handle.0 >= self.handle_to_index.len() {
            return false;
        }
        *self.handle_to_index.get(handle.0) >= 0
    }

    /// Debug-mode validation that a handle exists and mappings are consistent.
    #[inline(always)]
    pub fn validate_existing_handle(&self, handle: StaticHandle) {
        debug_assert!(self.static_exists(handle), "Handle must exist.");
        debug_assert!(handle.0 >= 0, "Handles must be nonnegative.");
        let idx = *self.handle_to_index.get(handle.0);
        debug_assert!(
            idx >= 0
                && self.index_to_handle.get(idx).0 == handle.0,
            "Mappings are out of sync."
        );
    }

    /// Gets a direct reference to the Static at the given index.
    #[inline(always)]
    pub fn get_direct_reference(&self, handle: StaticHandle) -> &Static {
        self.validate_existing_handle(handle);
        let index = *self.handle_to_index.get(handle.0);
        self.statics_buffer.get(index)
    }

    /// Gets a mutable direct reference to the Static at the given index.
    #[inline(always)]
    pub fn get_direct_reference_mut(&mut self, handle: StaticHandle) -> &mut Static {
        self.validate_existing_handle(handle);
        let index = *self.handle_to_index.get(handle.0);
        self.statics_buffer.get_mut(index)
    }

    /// Gets a reference to the static at a given storage index.
    #[inline(always)]
    pub fn get_by_index(&self, index: i32) -> &Static {
        debug_assert!(index >= 0 && index < self.count);
        self.statics_buffer.get(index)
    }

    /// Gets a mutable reference to the static at a given storage index.
    #[inline(always)]
    pub fn get_by_index_mut(&mut self, index: i32) -> &mut Static {
        debug_assert!(index >= 0 && index < self.count);
        self.statics_buffer.get_mut(index)
    }

    /// Applies a description to a static at the given index without modifying broad phase entries.
    fn apply_description_by_index_without_broad_phase(
        &mut self,
        index: i32,
        description: &StaticDescription,
    ) {
        let s = self.statics_buffer.get_mut(index);
        s.pose = description.pose;
        debug_assert!(
            description.shape.exists(),
            "Static collidables must have a shape."
        );
        s.continuity = description.continuity;
        s.shape = description.shape;
    }

    /// Adds a new static body to the simulation.
    pub fn add(&mut self, description: &StaticDescription) -> StaticHandle {
        if self.count == self.handle_to_index.len() {
            debug_assert!(
                self.handle_to_index.allocated(),
                "Statics backing memory should be initialized before use."
            );
            let new_size = self.handle_to_index.len() << 1;
            self.internal_resize(new_size);
        }

        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        let handle = StaticHandle(self.handle_pool.take());
        let index = self.count;
        self.count += 1;

        // Ensure handle_to_index can hold the new handle.
        if handle.0 >= self.handle_to_index.len() {
            let new_cap = (handle.0 + 1).max(self.handle_to_index.len() * 2);
            let old_len = self.handle_to_index.len();
            let pool = unsafe { &mut *pool_ptr };
            pool.resize_to_at_least(&mut self.handle_to_index, new_cap, old_len);
            unsafe {
                let ptr = self.handle_to_index.as_mut_ptr().add(old_len as usize) as *mut u8;
                let count = ((self.handle_to_index.len() - old_len) as usize)
                    * std::mem::size_of::<i32>();
                std::ptr::write_bytes(ptr, 0xFF, count);
            }
        }

        *self.handle_to_index.get_mut(handle.0) = index;
        *self.index_to_handle.get_mut(index) = handle;
        self.apply_description_by_index_without_broad_phase(index, description);
        // TODO: broad_phase.add_static + set broad_phase_index — requires BroadPhase implementation
        self.statics_buffer.get_mut(index).broad_phase_index = -1;
        handle
    }

    /// Removes a static by its storage index.
    pub fn remove_at(&mut self, index: i32) {
        debug_assert!(index >= 0 && index < self.count);
        self.validate_existing_handle(*self.index_to_handle.get(index));
        let handle = *self.index_to_handle.get(index);

        let collidable = *self.statics_buffer.get(index);
        debug_assert!(
            collidable.shape.exists(),
            "Static collidables cannot lack a shape."
        );

        // TODO: Remove from broad phase and awaken nearby bodies.

        // Move the last static into the removed slot.
        self.count -= 1;
        let static_moved = index < self.count;
        if static_moved {
            let moved_index = self.count;
            *self.statics_buffer.get_mut(index) = *self.statics_buffer.get(moved_index);
            let last_handle = *self.index_to_handle.get(moved_index);
            *self.handle_to_index.get_mut(last_handle.0) = index;
            *self.index_to_handle.get_mut(index) = last_handle;
        }
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.handle_pool.return_id(handle.0, pool);
        *self.handle_to_index.get_mut(handle.0) = -1;
    }

    /// Removes a static from the set by its handle.
    pub fn remove(&mut self, handle: StaticHandle) {
        self.validate_existing_handle(handle);
        let index = *self.handle_to_index.get(handle.0);
        self.remove_at(index);
    }

    /// Updates the bounds of a static in the broad phase for its current state.
    pub fn update_bounds(&mut self, handle: StaticHandle) {
        let _index = *self.handle_to_index.get(handle.0);
        // TODO: shapes.update_bounds + broadPhase.update_static_bounds
    }

    /// Changes the shape of a static and updates its bounds in the broad phase.
    pub fn set_shape(&mut self, handle: StaticHandle, new_shape: TypedIndex) {
        self.validate_existing_handle(handle);
        debug_assert!(new_shape.exists(), "Statics must have a shape.");
        let index = *self.handle_to_index.get(handle.0);
        self.statics_buffer.get_mut(index).shape = new_shape;
        self.update_bounds(handle);
    }

    /// Applies a new description to an existing static.
    pub fn apply_description(&mut self, handle: StaticHandle, description: &StaticDescription) {
        self.validate_existing_handle(handle);
        debug_assert!(
            description.shape.exists(),
            "Static collidables cannot lack a shape."
        );
        let index = *self.handle_to_index.get(handle.0);
        self.apply_description_by_index_without_broad_phase(index, description);
        // TODO: Update broad phase bounds.
    }

    /// Gets the current description of the static referred to by a given handle.
    pub fn get_description(&self, handle: StaticHandle, description: &mut StaticDescription) {
        self.validate_existing_handle(handle);
        let index = *self.handle_to_index.get(handle.0);
        let s = self.statics_buffer.get(index);
        description.pose = s.pose;
        description.continuity = s.continuity;
        description.shape = s.shape;
    }

    /// Gets the current description of a static by handle (returned by value).
    pub fn get_description_value(&self, handle: StaticHandle) -> StaticDescription {
        let mut desc = StaticDescription::with_discrete(RigidPose::default(), TypedIndex::default());
        self.get_description(handle, &mut desc);
        desc
    }

    /// Clears all statics without returning memory to the pool.
    pub fn clear(&mut self) {
        self.count = 0;
        unsafe {
            let ptr = self.handle_to_index.as_mut_ptr() as *mut u8;
            let count =
                self.handle_to_index.len() as usize * std::mem::size_of::<i32>();
            std::ptr::write_bytes(ptr, 0xFF, count);
        }
        self.handle_pool.clear();
    }

    /// Resizes allocated spans for static data. Conservative — never orphans existing objects.
    pub fn resize(&mut self, capacity: i32) {
        let target = BufferPool::get_capacity_for_count::<i32>(capacity.max(self.count));
        if self.index_to_handle.len() != target {
            self.internal_resize(target);
        }
    }

    /// Increases buffer sizes if needed to hold the target capacity.
    pub fn ensure_capacity(&mut self, capacity: i32) {
        if self.index_to_handle.len() < capacity {
            self.internal_resize(capacity);
        }
    }

    /// Returns all static resources to the pool.
    pub fn dispose(&mut self) {
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        pool.return_buffer(&mut self.statics_buffer);
        let pool = unsafe { &mut *pool_ptr };
        pool.return_buffer(&mut self.handle_to_index);
        let pool = unsafe { &mut *pool_ptr };
        pool.return_buffer(&mut self.index_to_handle);
        let pool = unsafe { &mut *pool_ptr };
        self.handle_pool.dispose(pool);
    }
}
