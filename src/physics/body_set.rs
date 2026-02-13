// Translated from BepuPhysics/BodySet.cs

use crate::physics::body_description::BodyDescription;
use crate::physics::body_properties::{BodyActivity, BodyDynamics};
use crate::physics::collidables::collidable::Collidable;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Reference connecting a body to one of its constraints.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyConstraintReference {
    /// Handle of the constraint connected to the body.
    pub connecting_constraint_handle: ConstraintHandle,
    /// The body's index within the constraint (e.g. 0 for body A, 1 for body B).
    pub body_index_in_constraint: i32,
}

/// Stores a group of bodies — either the set of active bodies, or the bodies involved in an inactive simulation island.
///
/// Note that all body information is stored in AOS format.
/// While the pose integrator would technically benefit from (AO)SOA, it would only help in a magical infinite bandwidth scenario.
/// In practice, the pose integrator's actual AOSOA-benefitting chunk can't even scale to 2 threads, even with only 4-wide SIMD.
/// On top of that, the narrow phase and solver both need to access the body's information in a noncontiguous way.
pub struct BodySet {
    /// Remaps a body index to its handle.
    pub index_to_handle: Buffer<BodyHandle>,
    /// Stores all data involved in solving constraints for a body, including pose, velocity, and inertia.
    pub dynamics_state: Buffer<BodyDynamics>,
    /// The collidables owned by each body in the set. Speculative margins, continuity settings, and shape indices can be changed directly.
    /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase.
    pub collidables: Buffer<Collidable>,
    /// Activity states of bodies in the set.
    pub activity: Buffer<BodyActivity>,
    /// List of constraints associated with each body in the set.
    pub constraints: Buffer<QuickList<BodyConstraintReference>>,
    /// Current number of bodies in the set.
    pub count: i32,
}

impl BodySet {
    /// Gets whether this instance is backed by allocated memory.
    #[inline(always)]
    pub fn allocated(&self) -> bool {
        self.index_to_handle.allocated()
    }

    /// Creates a new body set with the given initial capacity.
    pub fn new(initial_capacity: i32, pool: &mut BufferPool) -> Self {
        let mut set = Self {
            index_to_handle: Buffer::default(),
            dynamics_state: Buffer::default(),
            collidables: Buffer::default(),
            activity: Buffer::default(),
            constraints: Buffer::default(),
            count: 0,
        };
        set.internal_resize(initial_capacity, pool);
        set
    }

    /// Adds a body to the set. Returns the index of the added body.
    pub(crate) fn add(
        &mut self,
        body_description: &BodyDescription,
        handle: BodyHandle,
        minimum_constraint_capacity: i32,
        pool: &mut BufferPool,
    ) -> i32 {
        let index = self.count;
        if index == self.index_to_handle.len() {
            self.internal_resize(self.index_to_handle.len() * 2, pool);
        }
        self.count += 1;
        *self.index_to_handle.get_mut(index) = handle;
        // Collidable's broad phase index is left unset. The Bodies collection is responsible for attaching that data.
        *self.constraints.get_mut(index) =
            QuickList::with_capacity(minimum_constraint_capacity, pool);
        self.apply_description_by_index(index, body_description);
        index
    }

    /// Removes a body at the given index, moving the last body into its slot if needed.
    ///
    /// Returns `true` if a body was moved to fill the gap.
    pub(crate) fn remove_at(
        &mut self,
        body_index: i32,
        handle: &mut BodyHandle,
        moved_body_index: &mut i32,
        moved_body_handle: &mut BodyHandle,
    ) -> bool {
        *handle = *self.index_to_handle.get(body_index);
        // Move the last body into the removed slot.
        self.count -= 1;
        let body_moved = body_index < self.count;
        if body_moved {
            *moved_body_index = self.count;
            let mi = *moved_body_index;
            // Copy the memory state of the last element down.
            *self.dynamics_state.get_mut(body_index) = *self.dynamics_state.get(mi);
            *self.activity.get_mut(body_index) = *self.activity.get(mi);
            *self.collidables.get_mut(body_index) = *self.collidables.get(mi);
            // Note that the constraint list is NOT disposed before being overwritten.
            // The two callers for this function are 'true' removal, and sleeping.
            // During true removal, the caller is responsible for removing all constraints and disposing the list.
            // In sleeping, the reference to the list is simply copied into the sleeping set.
            unsafe {
                // Use raw pointer copy since QuickList is not Copy
                let src = self.constraints.as_ptr().add(mi as usize);
                let dst = self.constraints.as_mut_ptr().add(body_index as usize);
                std::ptr::copy_nonoverlapping(src, dst, 1);
            }
            // Point the body handles at the new location.
            *moved_body_handle = *self.index_to_handle.get(mi);
            *self.index_to_handle.get_mut(body_index) = *moved_body_handle;
        } else {
            *moved_body_index = -1;
            *moved_body_handle = BodyHandle(-1);
        }
        body_moved
    }

    /// Applies a body description to the body at the given index.
    pub(crate) fn apply_description_by_index(&mut self, index: i32, description: &BodyDescription) {
        let state = self.dynamics_state.get_mut(index);
        state.motion.pose = description.pose;
        state.motion.velocity = description.velocity;
        state.inertia.local = description.local_inertia;
        // Note that the world inertia is only valid in the velocity integration->pose integration interval,
        // so we don't need to initialize it here for dynamics.
        // Kinematics, though, can have their inertia updates skipped at runtime since the world inverse inertia
        // should always be a bunch of zeroes, so we pre-zero it.
        state.inertia.world = Default::default();

        let collidable = self.collidables.get_mut(index);
        // Note that we change the shape here. If the collidable transitions from shapeless->shapeful or shapeful->shapeless,
        // the broad phase has to be notified so that it can create/remove an entry. That's why this function isn't public.
        collidable.shape = description.collidable.shape;
        collidable.continuity = description.collidable.continuity;
        collidable.minimum_speculative_margin = description.collidable.minimum_speculative_margin;
        collidable.maximum_speculative_margin = description.collidable.maximum_speculative_margin;
        // To avoid leaking undefined data, initialize the speculative margin to zero.
        collidable.speculative_margin = 0.0;

        let activity = self.activity.get_mut(index);
        activity.sleep_threshold = description.activity.sleep_threshold;
        activity.minimum_timesteps_under_threshold =
            description.activity.minimum_timestep_count_under_threshold;
        activity.timesteps_under_threshold_count = 0;
        activity.sleep_candidate = false;
    }

    /// Reads out a full body description from the set.
    pub fn get_description(&self, index: i32, description: &mut BodyDescription) {
        let state = self.dynamics_state.get(index);
        description.pose = state.motion.pose;
        description.velocity = state.motion.velocity;
        description.local_inertia = state.inertia.local;

        let collidable = self.collidables.get(index);
        description.collidable.shape = collidable.shape;
        description.collidable.continuity = collidable.continuity;
        description.collidable.minimum_speculative_margin = collidable.minimum_speculative_margin;
        description.collidable.maximum_speculative_margin = collidable.maximum_speculative_margin;

        let activity = self.activity.get(index);
        description.activity.sleep_threshold = activity.sleep_threshold;
        description.activity.minimum_timestep_count_under_threshold =
            activity.minimum_timesteps_under_threshold;
    }

    /// Adds a constraint reference to a body's constraint list.
    #[inline(always)]
    pub(crate) fn add_constraint(
        &mut self,
        body_index: i32,
        constraint_handle: ConstraintHandle,
        body_index_in_constraint: i32,
        pool: &mut BufferPool,
    ) {
        let constraint_ref = BodyConstraintReference {
            connecting_constraint_handle: constraint_handle,
            body_index_in_constraint,
        };
        let constraints = self.constraints.get_mut(body_index);
        debug_assert!(
            constraints.span.allocated(),
            "Any time a body is created, a list should be built to support it."
        );
        if constraints.span.len() == constraints.count {
            constraints.resize(constraints.span.len() * 2, pool);
        }
        *constraints.allocate_unsafely() = constraint_ref;
    }

    /// Removes a constraint from a body's constraint list.
    ///
    /// Returns true if the number of constraints remaining attached to the body is 0.
    #[inline(always)]
    pub(crate) fn remove_constraint_reference(
        &mut self,
        body_index: i32,
        constraint_handle: ConstraintHandle,
        minimum_constraint_capacity_per_body: i32,
        pool: &mut BufferPool,
    ) -> bool {
        // This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
        let list = self.constraints.get_mut(body_index);
        for i in 0..list.count {
            if list[i].connecting_constraint_handle.0 == constraint_handle.0 {
                list.fast_remove_at(i);
                break;
            }
        }
        // Note the conservative resizing threshold.
        let conservative_count = 2 * list.count;
        let target_capacity = if conservative_count > minimum_constraint_capacity_per_body {
            conservative_count
        } else {
            minimum_constraint_capacity_per_body
        };
        // Don't bother trying to resize if it would end up just being the same power of 2.
        if list.span.len() >= 2 * target_capacity {
            list.resize(target_capacity, pool);
        }
        list.count == 0
    }

    /// Checks if a body is constrained by a specific constraint.
    pub fn body_is_constrained_by(
        &self,
        body_index: i32,
        constraint_handle: ConstraintHandle,
    ) -> bool {
        let list = self.constraints.get(body_index);
        for i in 0..list.count {
            if list[i].connecting_constraint_handle.0 == constraint_handle.0 {
                return true;
            }
        }
        false
    }

    /// Resizes the internal buffers to hold the target body capacity.
    pub(crate) fn internal_resize(&mut self, target_body_capacity: i32, pool: &mut BufferPool) {
        debug_assert!(
            target_body_capacity > 0,
            "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead."
        );
        let target_body_capacity = BufferPool::get_capacity_for_count::<i32>(target_body_capacity);
        pool.resize_to_at_least(&mut self.dynamics_state, target_body_capacity, self.count);
        pool.resize_to_at_least(&mut self.index_to_handle, target_body_capacity, self.count);
        pool.resize_to_at_least(&mut self.collidables, target_body_capacity, self.count);
        pool.resize_to_at_least(&mut self.activity, target_body_capacity, self.count);
        // Constraints buffer uses raw pointer copy since QuickList<T> is not Copy
        unsafe {
            let old_ptr = self.constraints.as_ptr();
            let old_len = self.constraints.len();
            let mut new_buf =
                pool.take_at_least::<QuickList<BodyConstraintReference>>(target_body_capacity);
            let copy_count = self.count.min(old_len);
            if copy_count > 0 && !old_ptr.is_null() {
                std::ptr::copy_nonoverlapping(old_ptr, new_buf.as_mut_ptr(), copy_count as usize);
            }
            if self.constraints.allocated() {
                pool.return_buffer(&mut self.constraints);
            }
            self.constraints = new_buf;
        }
    }

    /// Clears the set, disposing of all per-body constraint lists.
    pub fn clear(&mut self, pool: &mut BufferPool) {
        for i in 0..self.count {
            self.constraints.get_mut(i).dispose(pool);
        }
        self.count = 0;
    }

    /// Disposes the buffers, but nothing inside of the buffers.
    /// Per-body constraint lists stored in the set will not be returned.
    pub fn dispose_buffers(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.dynamics_state);
        pool.return_buffer(&mut self.index_to_handle);
        pool.return_buffer(&mut self.collidables);
        pool.return_buffer(&mut self.activity);
        pool.return_buffer(&mut self.constraints);
    }

    /// Disposes the body set's buffers and any resources within them.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        for i in 0..self.count {
            self.constraints.get_mut(i).dispose(pool);
        }
        self.dispose_buffers(pool);
        *self = Self {
            index_to_handle: Buffer::default(),
            dynamics_state: Buffer::default(),
            collidables: Buffer::default(),
            activity: Buffer::default(),
            constraints: Buffer::default(),
            count: 0,
        };
    }
}
