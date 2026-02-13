// Translated from BepuPhysics/Statics.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::collidable::ContinuousDetection;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::handles::{BodyHandle, StaticHandle};
use crate::physics::island_awakener::IslandAwakener;
use crate::physics::static_description::StaticDescription;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::for_each_ref::IBreakableForEach;
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
    fn should_awaken(&self, body_handle: BodyHandle, bodies: &Bodies) -> bool;
}

/// Default awakening filter that only wakes up dynamic bodies.
/// Kinematic bodies do not respond to any kind of dynamic simulation,
/// so they won't respond to the change in statics.
pub struct DefaultAwakeningFilter;
impl StaticChangeAwakeningFilter for DefaultAwakeningFilter {
    fn allow_awakening(&self) -> bool {
        true
    }
    fn should_awaken(&self, body_handle: BodyHandle, bodies: &Bodies) -> bool {
        unsafe {
            let location = *bodies.handle_to_location.get(body_handle.0);
            let set = bodies.sets.get(location.set_index);
            !Bodies::is_kinematic_unsafe_gc_hole(
                &set.dynamics_state.get(location.index).inertia.local,
            )
        }
    }
}

/// Filter that prevents any bodies from being awoken.
pub struct NoAwakeningFilter;
impl StaticChangeAwakeningFilter for NoAwakeningFilter {
    fn allow_awakening(&self) -> bool {
        false
    }
    fn should_awaken(&self, _body_handle: BodyHandle, _bodies: &Bodies) -> bool {
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
            let byte_count =
                ((self.handle_to_index.len() - start) as usize) * std::mem::size_of::<i32>();
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
            idx >= 0 && self.index_to_handle.get(idx).0 == handle.0,
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

    /// Adds a new static body to the simulation. Sleeping dynamic bodies whose bounding boxes
    /// overlap the new static are forced active.
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
                let count =
                    ((self.handle_to_index.len() - old_len) as usize) * std::mem::size_of::<i32>();
                std::ptr::write_bytes(ptr, 0xFF, count);
            }
        }

        *self.handle_to_index.get_mut(handle.0) = index;
        *self.index_to_handle.get_mut(index) = handle;
        self.apply_description_by_index_without_broad_phase(index, description);
        // Compute bounds and add to broad phase.
        let shapes = unsafe { &*self.shapes };
        let s = self.statics_buffer.get(index);
        let mut bounds = BoundingBox::default();
        shapes.update_bounds(&s.pose, &s.shape, &mut bounds);
        let broad_phase = unsafe { &mut *self.broad_phase };
        self.statics_buffer.get_mut(index).broad_phase_index = broad_phase.add_static(
            CollidableReference::from_static(handle),
            &bounds.min,
            &bounds.max,
        );
        // Awaken sleeping bodies near the new static's bounds.
        unsafe {
            self.awaken_bodies_in_bounds(&bounds, &DefaultAwakeningFilter);
        }
        handle
    }

    /// Awakens sleeping bodies whose broad phase bounds overlap the given bounding box.
    unsafe fn awaken_bodies_in_bounds<F: StaticChangeAwakeningFilter>(
        &mut self,
        bounds: &BoundingBox,
        filter: &F,
    ) {
        if !filter.allow_awakening() {
            return;
        }
        let pool = &mut *self.pool;
        let bodies = &*self.bodies;
        let broad_phase = &*self.broad_phase;
        let mut sleeping_sets = QuickList::<i32>::with_capacity(32, pool);

        // Collect sleeping body set indices by querying the static tree for overlaps.
        struct SleepingBodyCollector<'a, F: StaticChangeAwakeningFilter> {
            bodies: &'a Bodies,
            broad_phase: &'a BroadPhase,
            pool: &'a mut BufferPool,
            sleeping_sets: &'a mut QuickList<i32>,
            filter: &'a F,
        }
        impl<F: StaticChangeAwakeningFilter> IBreakableForEach<i32> for SleepingBodyCollector<'_, F> {
            #[inline(always)]
            fn loop_body(&mut self, leaf_index: i32) -> bool {
                let leaf = unsafe { *self.broad_phase.static_leaves.get(leaf_index) };
                if leaf.mobility() != CollidableMobility::Static {
                    let body_handle = leaf.body_handle();
                    let location = unsafe { *self.bodies.handle_to_location.get(body_handle.0) };
                    if location.set_index > 0 && self.filter.should_awaken(body_handle, self.bodies)
                    {
                        // This is a sleeping body that passes the filter — add its set index.
                        self.sleeping_sets.add(location.set_index, self.pool);
                    }
                }
                true
            }
        }
        {
            let mut collector = SleepingBodyCollector {
                bodies,
                broad_phase,
                pool,
                sleeping_sets: &mut sleeping_sets,
                filter,
            };
            // Query the static tree for overlapping leaves.
            broad_phase
                .static_tree
                .get_overlaps(*bounds, &mut collector);
        }

        if sleeping_sets.count > 0 {
            let awakener = &mut *self.awakener;
            awakener.awaken_sets(&mut sleeping_sets, None);
        }
        let pool = &mut *self.pool;
        sleeping_sets.dispose(pool);
    }

    /// Awakens sleeping bodies that overlap the existing broad phase bounds of the given static leaf.
    unsafe fn awaken_bodies_in_existing_bounds<F: StaticChangeAwakeningFilter>(
        &mut self,
        broad_phase_index: i32,
        filter: &F,
    ) {
        let broad_phase = &*self.broad_phase;
        let (min_ptr, max_ptr) = broad_phase.get_static_bounds_pointers(broad_phase_index);
        let mut old_bounds = BoundingBox::default();
        old_bounds.min = *min_ptr;
        old_bounds.max = *max_ptr;
        self.awaken_bodies_in_bounds(&old_bounds, filter);
    }

    /// Removes a static by its storage index. Wakes up nearby sleeping bodies.
    pub fn remove_at(&mut self, index: i32) {
        self.remove_at_filtered(index, &DefaultAwakeningFilter);
    }

    /// Removes a static by its storage index with a custom awakening filter.
    pub fn remove_at_filtered<F: StaticChangeAwakeningFilter>(&mut self, index: i32, filter: &F) {
        debug_assert!(index >= 0 && index < self.count);
        self.validate_existing_handle(*self.index_to_handle.get(index));
        let handle = *self.index_to_handle.get(index);

        let collidable = *self.statics_buffer.get(index);
        debug_assert!(
            collidable.shape.exists(),
            "Static collidables cannot lack a shape."
        );

        // Awaken sleeping bodies near the static's current bounds before removing from broad phase.
        unsafe {
            self.awaken_bodies_in_existing_bounds(collidable.broad_phase_index, filter);
        }

        // Remove from broad phase.
        let removed_broad_phase_index = collidable.broad_phase_index;
        let broad_phase = unsafe { &mut *self.broad_phase };
        let mut moved_leaf = CollidableReference::default();
        if broad_phase.remove_static_at(removed_broad_phase_index, &mut moved_leaf) {
            // Update the collidable->leaf index pointer for the leaf that was moved.
            if moved_leaf.mobility() == CollidableMobility::Static {
                // This is a static collidable.
                self.get_direct_reference_mut(moved_leaf.static_handle())
                    .broad_phase_index = removed_broad_phase_index;
            } else {
                // This is a sleeping body.
                let bodies = unsafe { &mut *self.bodies };
                bodies.update_collidable_broad_phase_index(
                    moved_leaf.body_handle(),
                    removed_broad_phase_index,
                );
            }
        }

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
        let index = *self.handle_to_index.get(handle.0);
        let s = self.statics_buffer.get(index);
        let shapes = unsafe { &*self.shapes };
        let mut bounds = BoundingBox::default();
        shapes.update_bounds(&s.pose, &s.shape, &mut bounds);
        let broad_phase = unsafe { &mut *self.broad_phase };
        broad_phase.update_static_bounds(s.broad_phase_index, bounds.min, bounds.max);
    }

    /// Changes the shape of a static and updates its bounds in the broad phase.
    /// Sleeping bodies with bounding boxes overlapping the old or new bounds are forced active.
    pub fn set_shape(&mut self, handle: StaticHandle, new_shape: TypedIndex) {
        self.validate_existing_handle(handle);
        debug_assert!(new_shape.exists(), "Statics must have a shape.");
        let index = *self.handle_to_index.get(handle.0);
        // Wake sleeping bodies near the old bounds.
        unsafe {
            let bp_index = self.statics_buffer.get(index).broad_phase_index;
            self.awaken_bodies_in_existing_bounds(bp_index, &DefaultAwakeningFilter);
        }
        self.statics_buffer.get_mut(index).shape = new_shape;
        self.update_bounds(handle);
        // Wake sleeping bodies near the new bounds.
        unsafe {
            let bp_index = self.statics_buffer.get(index).broad_phase_index;
            self.awaken_bodies_in_existing_bounds(bp_index, &DefaultAwakeningFilter);
        }
    }

    /// Applies a new description to an existing static.
    /// Sleeping bodies near old and new bounds are forced active.
    pub fn apply_description(&mut self, handle: StaticHandle, description: &StaticDescription) {
        self.validate_existing_handle(handle);
        debug_assert!(
            description.shape.exists(),
            "Static collidables cannot lack a shape."
        );
        let index = *self.handle_to_index.get(handle.0);
        // Wake sleeping bodies near the old bounds.
        unsafe {
            let bp_index = self.statics_buffer.get(index).broad_phase_index;
            self.awaken_bodies_in_existing_bounds(bp_index, &DefaultAwakeningFilter);
        }
        self.apply_description_by_index_without_broad_phase(index, description);
        // Update broad phase bounds.
        let s = self.statics_buffer.get(index);
        let shapes = unsafe { &*self.shapes };
        let mut bounds = BoundingBox::default();
        shapes.update_bounds(&s.pose, &s.shape, &mut bounds);
        let broad_phase = unsafe { &mut *self.broad_phase };
        broad_phase.update_static_bounds(s.broad_phase_index, bounds.min, bounds.max);
        // Wake sleeping bodies near the new bounds.
        unsafe {
            self.awaken_bodies_in_existing_bounds(s.broad_phase_index, &DefaultAwakeningFilter);
        }
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
        let mut desc =
            StaticDescription::with_discrete(RigidPose::default(), TypedIndex::default());
        self.get_description(handle, &mut desc);
        desc
    }

    /// Clears all statics without returning memory to the pool.
    pub fn clear(&mut self) {
        self.count = 0;
        unsafe {
            let ptr = self.handle_to_index.as_mut_ptr() as *mut u8;
            let count = self.handle_to_index.len() as usize * std::mem::size_of::<i32>();
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
