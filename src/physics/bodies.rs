// Translated from BepuPhysics/Bodies.cs and BepuPhysics/Bodies_GatherScatter.cs (constants only)

use crate::physics::body_description::BodyDescription;
use crate::physics::body_properties::{BodyDynamics, BodyInertia, BodyInertiaWide, RigidPose};
use crate::physics::body_set::{BodyConstraintReference, BodySet};
use crate::physics::collidables::collidable::Collidable;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::utilities::vector::Vector;
use std::simd::prelude::*;

// Forward-declared opaque types for circular dependencies.
// These will be replaced with actual implementations when their files are translated.

/// Placeholder for BroadPhase (from CollisionDetection).
pub struct BroadPhase {
    _opaque: [u8; 0],
}

/// Placeholder for Solver.
pub struct Solver {
    _opaque: [u8; 0],
}

/// Placeholder for IslandAwakener.
pub struct IslandAwakener {
    _opaque: [u8; 0],
}

/// Placeholder for IslandSleeper.
pub struct IslandSleeper {
    _opaque: [u8; 0],
}

/// Location of a body in memory.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyMemoryLocation {
    /// Index of the set owning the body reference. 0 = awake, >0 = sleeping island.
    pub set_index: i32,
    /// Index of the body within its owning set.
    pub index: i32,
}

/// Collection of all allocated bodies.
pub struct Bodies {
    /// Remaps a body handle integer value to the actual array index of the body.
    pub handle_to_location: Buffer<BodyMemoryLocation>,
    /// Pool from which handles are pulled for new bodies.
    pub handle_pool: IdPool,
    /// The set of existing bodies. Slot 0 = active bodies; later slots = inactive islands.
    /// Note that this buffer does not necessarily contain contiguous elements.
    pub sets: Buffer<BodySet>,

    pool: *mut BufferPool,
    pub(crate) awakener: *mut IslandAwakener,
    pub(crate) sleeper: *mut IslandSleeper,
    pub(crate) shapes: *mut Shapes,
    pub(crate) broad_phase: *mut BroadPhase,
    pub(crate) solver: *mut Solver,

    /// Minimum constraint capacity for each body.
    pub minimum_constraint_capacity_per_body: i32,
}

// === Constants from Bodies_GatherScatter.cs ===
impl Bodies {
    pub const DOESNT_EXIST_FLAG_INDEX: i32 = 31;
    pub const KINEMATIC_FLAG_INDEX: i32 = 30;
    pub const KINEMATIC_MASK: u32 = 1u32 << 30;
    /// Constraint body references greater than this unsigned value are either kinematic or empty lane.
    pub const DYNAMIC_LIMIT: u32 = Self::KINEMATIC_MASK;
    pub const BODY_REFERENCE_METADATA_MASK: u32 = (1u32 << 31) | Self::KINEMATIC_MASK;
    /// Mask of bits containing the decoded body reference in a constraint body reference.
    pub const BODY_REFERENCE_MASK: i32 = !(Self::BODY_REFERENCE_METADATA_MASK as i32);

    /// Checks whether a constraint encoded body reference value refers to a dynamic body.
    #[inline(always)]
    pub fn is_encoded_dynamic_reference(encoded_body_reference_value: i32) -> bool {
        (encoded_body_reference_value as u32) < Self::DYNAMIC_LIMIT
    }

    /// Checks whether a constraint encoded body reference value refers to a kinematic body.
    #[inline(always)]
    pub fn is_encoded_kinematic_reference(encoded_body_reference_value: i32) -> bool {
        let unsigned = encoded_body_reference_value as u32;
        unsigned >= Self::DYNAMIC_LIMIT && unsigned < (1u32 << 31)
    }
}

// === Core Bodies implementation ===
impl Bodies {
    /// Returns a reference to the active set (index 0 of the Sets buffer).
    #[inline(always)]
    pub fn active_set(&self) -> &BodySet {
        self.sets.get(0)
    }

    /// Returns a mutable reference to the active set.
    #[inline(always)]
    pub fn active_set_mut(&mut self) -> &mut BodySet {
        self.sets.get_mut(0)
    }

    /// Gets the pool used by the bodies collection.
    #[inline(always)]
    pub fn pool(&self) -> &mut BufferPool {
        unsafe { &mut *self.pool }
    }

    /// Creates a new Bodies collection.
    pub fn new(
        pool: *mut BufferPool,
        shapes: *mut Shapes,
        broad_phase: *mut BroadPhase,
        initial_body_capacity: i32,
        initial_island_capacity: i32,
        initial_constraint_capacity_per_body: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let handle_pool = IdPool::new(initial_body_capacity, pool_ref);
        let mut handle_to_location: Buffer<BodyMemoryLocation> = Buffer::default();
        Self::resize_handles_impl(pool_ref, &mut handle_to_location, initial_body_capacity);
        let mut sets: Buffer<BodySet> = pool_ref.take_at_least(initial_island_capacity + 1);
        // Initialize all set slots to default (unallocated).
        sets.clear(0, sets.len());
        let active_set = BodySet::new(initial_body_capacity, pool_ref);
        *sets.get_mut(0) = active_set;

        Bodies {
            handle_to_location,
            handle_pool,
            sets,
            pool,
            awakener: std::ptr::null_mut(),
            sleeper: std::ptr::null_mut(),
            shapes,
            broad_phase,
            solver: std::ptr::null_mut(),
            minimum_constraint_capacity_per_body: initial_constraint_capacity_per_body,
        }
    }

    /// Initializes bidirectional dependencies.
    pub fn initialize(
        &mut self,
        solver: *mut Solver,
        awakener: *mut IslandAwakener,
        sleeper: *mut IslandSleeper,
    ) {
        self.solver = solver;
        self.awakener = awakener;
        self.sleeper = sleeper;
    }

    /// Gets whether the inertia matches that of a kinematic body (all inverse mass/inertia zero).
    #[inline(always)]
    pub fn is_kinematic(inertia: &BodyInertia) -> bool {
        inertia.inverse_mass == 0.0
            && inertia.inverse_inertia_tensor.xx == 0.0
            && inertia.inverse_inertia_tensor.yx == 0.0
            && inertia.inverse_inertia_tensor.yy == 0.0
            && inertia.inverse_inertia_tensor.zx == 0.0
            && inertia.inverse_inertia_tensor.zy == 0.0
            && inertia.inverse_inertia_tensor.zz == 0.0
    }

    /// Checks inertia lanes for kinematicity (all inverse mass and inertia values are zero).
    #[inline(always)]
    pub fn is_kinematic_wide(inertia: &BodyInertiaWide) -> Vector<i32> {
        use crate::utilities::vector::Vector;
        let zero = Vector::<f32>::splat(0.0);
        let result = inertia.inverse_mass.simd_eq(zero)
            & inertia.inverse_inertia_tensor.xx.simd_eq(zero)
            & inertia.inverse_inertia_tensor.yx.simd_eq(zero)
            & inertia.inverse_inertia_tensor.yy.simd_eq(zero)
            & inertia.inverse_inertia_tensor.zx.simd_eq(zero)
            & inertia.inverse_inertia_tensor.zy.simd_eq(zero)
            & inertia.inverse_inertia_tensor.zz.simd_eq(zero);
        result.to_int()
    }

    /// Has locked inertia (all inverse inertia tensor components zero).
    #[inline(always)]
    pub fn has_locked_inertia(
        inertia: &crate::utilities::symmetric3x3::Symmetric3x3,
    ) -> bool {
        inertia.xx == 0.0
            && inertia.yx == 0.0
            && inertia.yy == 0.0
            && inertia.zx == 0.0
            && inertia.zy == 0.0
            && inertia.zz == 0.0
    }

    /// Adds a new active body to the simulation.
    pub fn add(&mut self, description: &BodyDescription) -> BodyHandle {
        debug_assert!(
            self.handle_to_location.allocated(),
            "The backing memory of the bodies set should be initialized before use."
        );
        let handle_index = self.handle_pool.take();
        debug_assert!(handle_index <= self.handle_to_location.len());
        if handle_index == self.handle_to_location.len() {
            let pool_ptr = self.pool;
            let new_capacity = self.handle_to_location.len() << 1;
            Self::resize_handles_impl(
                unsafe { &mut *pool_ptr },
                &mut self.handle_to_location,
                new_capacity,
            );
        }

        let handle = BodyHandle(handle_index);
        let min_cap = self.minimum_constraint_capacity_per_body;
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        let index = self.sets.get_mut(0).add(
            description,
            handle,
            min_cap,
            pool,
        );
        *self.handle_to_location.get_mut(handle_index) = BodyMemoryLocation {
            set_index: 0,
            index,
        };

        if description.collidable.shape.exists() {
            // TODO: AddCollidableToBroadPhase - requires BroadPhase implementation
        } else {
            self.sets.get_mut(0).collidables.get_mut(index).broad_phase_index = -1;
        }
        handle
    }

    /// Adds a constraint to an active body's constraint list.
    pub(crate) fn add_constraint(
        &mut self,
        body_index: i32,
        constraint_handle: ConstraintHandle,
        index_in_constraint: i32,
    ) {
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.sets.get_mut(0).add_constraint(
            body_index,
            constraint_handle,
            index_in_constraint,
            pool,
        );
    }

    /// Removes a constraint from an active body's constraint list.
    pub(crate) fn remove_constraint_reference(
        &mut self,
        body_index: i32,
        constraint_handle: ConstraintHandle,
    ) -> bool {
        let min_cap = self.minimum_constraint_capacity_per_body;
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.sets.get_mut(0)
            .remove_constraint_reference(body_index, constraint_handle, min_cap, pool)
    }

    /// Checks whether a body handle is currently registered.
    #[inline(always)]
    pub fn body_exists(&self, body_handle: BodyHandle) -> bool {
        body_handle.0 >= 0
            && body_handle.0 < self.handle_to_location.len()
            && self.handle_to_location.get(body_handle.0).set_index >= 0
    }

    /// Computes the total number of bodies in the simulation across all sets.
    pub fn count_bodies(&self) -> i32 {
        let mut count = 0;
        for i in 0..self.sets.len() {
            let set = self.sets.get(i);
            if set.allocated() {
                count += set.count;
            }
        }
        count
    }

    /// Gets the description of a body by handle.
    pub fn get_description(&self, handle: BodyHandle, description: &mut BodyDescription) {
        let location = self.handle_to_location.get(handle.0);
        let set = self.sets.get(location.set_index);
        set.get_description(location.index, description);
    }

    fn resize_handles_impl(
        pool: &mut BufferPool,
        handle_to_location: &mut Buffer<BodyMemoryLocation>,
        new_capacity: i32,
    ) {
        let new_capacity = BufferPool::get_capacity_for_count::<BodyMemoryLocation>(new_capacity);
        if new_capacity != handle_to_location.len() {
            let old_capacity = handle_to_location.len();
            pool.resize_to_at_least(
                handle_to_location,
                new_capacity,
                old_capacity.min(new_capacity),
            );
            if handle_to_location.len() > old_capacity {
                // Initialize new slots to -1 (0xFF bytes).
                unsafe {
                    let ptr = handle_to_location.as_mut_ptr().add(old_capacity as usize)
                        as *mut u8;
                    let count = (handle_to_location.len() - old_capacity) as usize
                        * std::mem::size_of::<BodyMemoryLocation>();
                    std::ptr::write_bytes(ptr, 0xFF, count);
                }
            }
        }
    }

    pub(crate) fn resize_sets_capacity(
        &mut self,
        sets_capacity: i32,
        potentially_allocated_count: i32,
    ) {
        debug_assert!(
            sets_capacity >= potentially_allocated_count
                && potentially_allocated_count <= self.sets.len()
        );
        let sets_capacity = BufferPool::get_capacity_for_count::<BodySet>(sets_capacity);
        if self.sets.len() != sets_capacity {
            let old_capacity = self.sets.len();
            let pool_ptr = self.pool;
            let pool = unsafe { &mut *pool_ptr };
            // BodySet is not Copy, so we do a raw memory resize.
            let mut new_sets: Buffer<BodySet> = pool.take_at_least(sets_capacity);
            let copy_count = potentially_allocated_count.min(new_sets.len());
            if copy_count > 0 {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        self.sets.as_ptr(),
                        new_sets.as_mut_ptr(),
                        copy_count as usize,
                    );
                }
            }
            // Zero-init new slots.
            if new_sets.len() > copy_count {
                new_sets.clear(copy_count, new_sets.len() - copy_count);
            }
            pool.return_buffer(&mut self.sets);
            self.sets = new_sets;
        }
    }

    /// Resizes allocated active body buffers. Conservative — never orphans existing objects.
    pub fn resize(&mut self, capacity: i32) {
        let active_count = self.sets.get(0).count;
        let target_body_capacity =
            BufferPool::get_capacity_for_count::<i32>(capacity.max(active_count));
        if self.sets.get(0).index_to_handle.len() != target_body_capacity {
            let pool_ptr = self.pool;
            let pool = unsafe { &mut *pool_ptr };
            self.sets.get_mut(0)
                .internal_resize(target_body_capacity, pool);
        }
        let highest = self.handle_pool.highest_possibly_claimed_id() + 1;
        let target_handle_capacity =
            BufferPool::get_capacity_for_count::<i32>(capacity.max(highest));
        if self.handle_to_location.len() != target_handle_capacity {
            let pool_ptr = self.pool;
            Self::resize_handles_impl(unsafe { &mut *pool_ptr }, &mut self.handle_to_location, target_handle_capacity);
        }
    }

    /// Resizes all active body constraint lists to meet the minimum constraint capacity per body.
    pub fn resize_constraint_list_capacities(&mut self) {
        let min_cap = self.minimum_constraint_capacity_per_body;
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        let count = self.sets.get(0).count;
        for i in 0..count {
            let list = self.sets.get_mut(0).constraints.get_mut(i);
            let target = BufferPool::get_capacity_for_count::<BodyConstraintReference>(
                if list.count > min_cap {
                    list.count
                } else {
                    min_cap
                },
            );
            if list.span.len() != target {
                list.resize(target, pool);
            }
        }
    }

    /// Increases the size of active body buffers if needed.
    pub fn ensure_capacity(&mut self, capacity: i32) {
        if self.sets.get(0).index_to_handle.len() < capacity {
            let pool_ptr = self.pool;
            let pool = unsafe { &mut *pool_ptr };
            self.sets.get_mut(0).internal_resize(capacity, pool);
        }
        if self.handle_to_location.len() < capacity {
            let pool_ptr = self.pool;
            Self::resize_handles_impl(unsafe { &mut *pool_ptr }, &mut self.handle_to_location, capacity);
        }
    }

    /// Ensures all active body constraint lists can hold at least minimum capacity.
    pub fn ensure_constraint_list_capacities(&mut self) {
        let min_cap = self.minimum_constraint_capacity_per_body;
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        let count = self.sets.get(0).count;
        for i in 0..count {
            let list = self.sets.get_mut(0).constraints.get_mut(i);
            if list.span.len() < min_cap {
                list.resize(min_cap, pool);
            }
        }
    }

    /// Validates that the given body handle refers to a body that actually exists.
    /// Debug-only in the C# source ([Conditional("DEBUG")]), so here it's always a debug_assert.
    #[inline(always)]
    pub fn validate_existing_handle(&self, handle: BodyHandle) {
        debug_assert!(handle.0 >= 0, "Handles must be nonnegative.");
        debug_assert!(
            handle.0 <= self.handle_pool.highest_possibly_claimed_id()
                && self.handle_pool.highest_possibly_claimed_id() < self.handle_to_location.len(),
            "Existing handles must fit within the body handle->index mapping."
        );
        let location = self.handle_to_location.get(handle.0);
        debug_assert!(
            location.set_index >= 0 && location.set_index < self.sets.len(),
            "Body set index must be nonnegative and within the sets buffer length."
        );
        let set = self.sets.get(location.set_index);
        debug_assert!(set.allocated());
        debug_assert!(set.count <= set.index_to_handle.len());
        debug_assert!(
            location.index >= 0 && location.index < set.count,
            "Body index must fall within the existing body set."
        );
        debug_assert!(
            set.index_to_handle.get(location.index).0 == handle.0,
            "Handle->index must match index->handle map."
        );
        debug_assert!(self.body_exists(handle), "Body must exist according to the body_exists test.");
    }

    /// Alias for `is_kinematic` that takes a reference. In C# this was a separate
    /// `IsKinematicUnsafeGCHole` to avoid GC pin issues; in Rust references are always safe.
    #[inline(always)]
    pub fn is_kinematic_unsafe_gc_hole(inertia: &BodyInertia) -> bool {
        Self::is_kinematic(inertia)
    }

    /// Updates the body's bounds in the broad phase for its current state.
    pub fn update_bounds(&mut self, body_handle: BodyHandle) {
        let location = *self.handle_to_location.get(body_handle.0);
        let set = self.sets.get(location.set_index);
        let collidable = *set.collidables.get(location.index);
        if collidable.shape.exists() {
            // TODO: shapes.update_bounds + broadPhase.update_active_bounds / update_static_bounds
            // Requires BroadPhase and Shapes implementations
        }
    }

    fn update_for_kinematic_state_change(
        &mut self,
        _handle: BodyHandle,
        location: &BodyMemoryLocation,
        _set: &mut BodySet,
        previously_kinematic: bool,
        currently_kinematic: bool,
    ) {
        debug_assert!(
            location.set_index == 0,
            "If we're changing kinematic state, we should have already awoken the body."
        );
        if previously_kinematic != currently_kinematic {
            // TODO: Update broad phase collidable references and solver references.
            // Requires BroadPhase and Solver implementations.
        }
    }

    fn update_for_shape_change(
        &mut self,
        _handle: BodyHandle,
        _active_body_index: i32,
        old_shape: TypedIndex,
        new_shape: TypedIndex,
    ) {
        if old_shape.exists() != new_shape.exists() {
            if new_shape.exists() {
                // TODO: Add collidable to broad phase for the new shape.
            } else {
                // TODO: Remove the now-unused collidable from the broad phase.
            }
        }
    }

    /// Changes the local mass and inertia tensor associated with a body.
    /// Properly handles the transition between kinematic and dynamic.
    /// Wakes up the body.
    pub fn set_local_inertia(&mut self, handle: BodyHandle, local_inertia: &BodyInertia) {
        let location = *self.handle_to_location.get(handle.0);
        if location.set_index > 0 {
            // Body is inactive — wake it up.
            // TODO: awakener.awaken_body(handle) — requires IslandAwakener implementation
        }
        let location = *self.handle_to_location.get(handle.0);
        let set = self.sets.get_mut(location.set_index);
        let inertias = &mut set.dynamics_state.get_mut(location.index).inertia;
        let _previously_kinematic = Self::is_kinematic(&inertias.local);
        let _now_kinematic = Self::is_kinematic(local_inertia);
        inertias.local = *local_inertia;
        inertias.world = BodyInertia::default();
        // TODO: update_for_kinematic_state_change — requires BroadPhase and Solver
    }

    /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful.
    /// Wakes up the body and updates bounds.
    pub fn set_shape(&mut self, handle: BodyHandle, new_shape: TypedIndex) {
        let location = *self.handle_to_location.get(handle.0);
        if location.set_index > 0 {
            // TODO: awakener.awaken_body(handle)
        }
        let location = *self.handle_to_location.get(handle.0);
        debug_assert!(location.set_index == 0, "We should be working with an active shape.");
        let set = self.sets.get_mut(0);
        let collidable = set.collidables.get_mut(location.index);
        let _old_shape = collidable.shape;
        collidable.shape = new_shape;
        // TODO: update_for_shape_change + update_bounds — requires BroadPhase
    }

    /// Applies a description to a body. Handles transitions between dynamic/kinematic and shapeless/shapeful.
    pub fn apply_description(&mut self, handle: BodyHandle, description: &BodyDescription) {
        self.validate_existing_handle(handle);
        let location = *self.handle_to_location.get(handle.0);
        if location.set_index > 0 {
            // TODO: awakener.awaken_body(handle)
        }
        let location = *self.handle_to_location.get(handle.0);
        let set = self.sets.get_mut(location.set_index);
        let _old_shape = set.collidables.get(location.index).shape;
        let _now_kinematic = Self::is_kinematic(&description.local_inertia);
        let _previously_kinematic = Self::is_kinematic(&set.dynamics_state.get(location.index).inertia.local);
        set.apply_description_by_index(location.index, description);
        // TODO: update_for_shape_change + update_for_kinematic_state_change + update_bounds
        // Requires BroadPhase and Solver implementations.
    }

    /// Gets a BodyReference for the given handle.
    pub fn get_body_reference(&self, handle: BodyHandle) -> BodyHandle {
        self.validate_existing_handle(handle);
        handle
    }

    /// Removes an active body by its index. Connected constraints are removed.
    pub fn remove_at(&mut self, active_body_index: i32) {
        // Remove constraints first.
        let constraints_count = self.sets.get(0).constraints.get(active_body_index).count;
        for i in (0..constraints_count).rev() {
            let _constraint_handle =
                self.sets.get(0).constraints.get(active_body_index).span.get(i).connecting_constraint_handle;
            // TODO: solver.remove(constraint_handle) — requires Solver implementation
        }
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.sets.get_mut(0)
            .constraints
            .get_mut(active_body_index)
            .dispose(pool);

        let handle = self.remove_from_active_set(active_body_index);
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.handle_pool.return_id(handle.0, pool);
        let removed_location = self.handle_to_location.get_mut(handle.0);
        removed_location.set_index = -1;
        removed_location.index = -1;
    }

    fn remove_from_active_set(&mut self, active_body_index: i32) -> BodyHandle {
        debug_assert!(active_body_index >= 0 && active_body_index < self.sets.get(0).count);
        self.validate_existing_handle(*self.sets.get(0).index_to_handle.get(active_body_index));
        let collidable = *self.sets.get(0).collidables.get(active_body_index);
        if collidable.shape.exists() {
            // TODO: Remove collidable from broad phase
        }
        let mut handle = BodyHandle(-1);
        let mut moved_body_index = -1i32;
        let mut moved_body_handle = BodyHandle(-1);
        let body_moved = self.sets.get_mut(0).remove_at(
            active_body_index,
            &mut handle,
            &mut moved_body_index,
            &mut moved_body_handle,
        );
        if body_moved {
            // TODO: solver.update_for_body_memory_move(moved_body_index, active_body_index)
            self.handle_to_location.get_mut(moved_body_handle.0).index = active_body_index;
        }
        handle
    }

    /// Removes a body from the simulation by its handle.
    pub fn remove(&mut self, handle: BodyHandle) {
        self.validate_existing_handle(handle);
        // TODO: awakener.awaken_body(handle)
        let index = self.handle_to_location.get(handle.0).index;
        self.remove_at(index);
    }

    fn add_collidable_to_broad_phase(
        &mut self,
        _handle: BodyHandle,
        _pose: &RigidPose,
        _local_inertia: &BodyInertia,
        _collidable: &mut Collidable,
    ) {
        // TODO: Requires BroadPhase and Shapes implementations.
    }

    /// Updates the broad phase index stored on a body's collidable.
    pub(crate) fn update_collidable_broad_phase_index(
        &mut self,
        handle: BodyHandle,
        new_broad_phase_index: i32,
    ) {
        let location = *self.handle_to_location.get(handle.0);
        self.sets
            .get_mut(location.set_index)
            .collidables
            .get_mut(location.index)
            .broad_phase_index = new_broad_phase_index;
    }

    /// Clears all bodies from all sets without releasing persistent memory.
    pub fn clear(&mut self) {
        let pool_ptr = self.pool;
        let pool = unsafe { &mut *pool_ptr };
        self.sets.get_mut(0).clear(pool);
        for i in 1..self.sets.len() {
            let pool = unsafe { &mut *pool_ptr };
            let set = self.sets.get_mut(i);
            if set.allocated() {
                set.dispose(pool);
            }
        }
        unsafe {
            let ptr = self.handle_to_location.as_mut_ptr() as *mut u8;
            let count =
                self.handle_to_location.len() as usize * std::mem::size_of::<BodyMemoryLocation>();
            std::ptr::write_bytes(ptr, 0xFF, count);
        }
        self.handle_pool.clear();
    }

    /// Returns all body resources to the pool.
    pub fn dispose(&mut self) {
        let pool_ptr = self.pool;
        for i in 0..self.sets.len() {
            let pool = unsafe { &mut *pool_ptr };
            let set = self.sets.get_mut(i);
            if set.allocated() {
                set.dispose(pool);
            }
        }
        let pool = unsafe { &mut *pool_ptr };
        pool.return_buffer(&mut self.sets);
        let pool = unsafe { &mut *pool_ptr };
        pool.return_buffer(&mut self.handle_to_location);
        let pool = unsafe { &mut *pool_ptr };
        self.handle_pool.dispose(pool);
    }
}
