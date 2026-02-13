// Translated from BepuPhysics/BodyReference.cs

use crate::physics::bodies::{Bodies, BodyMemoryLocation};
use crate::physics::body_description::BodyDescription;
use crate::physics::body_properties::{
    BodyActivity, BodyDynamics, BodyInertia, BodyVelocity, MotionState, RigidPose,
};
use crate::physics::body_set::{BodyConstraintReference, BodySet};
use crate::physics::collidables::collidable::Collidable;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::BodyHandle;
use crate::physics::pose_integration::PoseIntegration;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::symmetric3x3::Symmetric3x3;
use glam::Vec3;

/// Convenience structure for directly referring to a body's properties.
///
/// Note that this type makes no attempt to protect against unsafe modification of body properties,
/// nor does modifying its properties try to wake up bodies if they are asleep.
pub struct BodyReference {
    /// Handle of the body that this reference refers to.
    pub handle: BodyHandle,
    /// The bodies collection containing the body.
    pub bodies: *mut Bodies,
}

impl BodyReference {
    /// Constructs a new body reference.
    #[inline(always)]
    pub fn new(handle: BodyHandle, bodies: *mut Bodies) -> Self {
        Self { handle, bodies }
    }

    /// Gets the bodies collection as a shared reference.
    #[inline(always)]
    fn bodies(&self) -> &Bodies {
        unsafe { &*self.bodies }
    }

    /// Gets the bodies collection as a mutable reference.
    #[inline(always)]
    fn bodies_mut(&self) -> &mut Bodies {
        unsafe { &mut *self.bodies }
    }

    /// Gets whether the body reference exists within the body set.
    pub fn exists(&self) -> bool {
        if self.bodies.is_null() {
            return false;
        }
        self.bodies().body_exists(self.handle)
    }

    /// Gets a reference to the body's memory location stored in the handle-to-location mapping.
    #[inline(always)]
    pub fn memory_location(&self) -> &BodyMemoryLocation {
        self.bodies().validate_existing_handle(self.handle);
        self.bodies().handle_to_location.get(self.handle.0)
    }

    /// Gets whether the body is in the active set (set index 0).
    #[inline(always)]
    pub fn awake(&self) -> bool {
        self.memory_location().set_index == 0
    }

    /// Sets whether the body is awake. Setting to true will attempt to wake the body;
    /// setting to false will force the body and any constraint-connected bodies asleep.
    pub fn set_awake(&self, value: bool) {
        if self.awake() {
            if !value {
                let loc = self.memory_location();
                let sleeper = unsafe { &mut *self.bodies().sleeper };
                unsafe {
                    sleeper.sleep_body(loc.index);
                }
            }
        } else {
            if value {
                let awakener = unsafe { &mut *self.bodies().awakener };
                awakener.awaken_body(self.handle);
            }
        }
    }

    /// Gets direct pointers to the body's bounding box minimum and maximum in the broad phase.
    /// Returns `(true, min_ptr, max_ptr)` if the body has a shape and bounds, or `(false, null, null)` otherwise.
    pub unsafe fn get_bounds_references_from_broad_phase(&self) -> (bool, *mut Vec3, *mut Vec3) {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        let collidable = set.collidables.get(loc.index);
        if collidable.shape.exists() {
            let broad_phase = &*self.bodies().broad_phase;
            if loc.set_index == 0 {
                let (min_ptr, max_ptr) =
                    broad_phase.get_active_bounds_pointers(collidable.broad_phase_index);
                (true, min_ptr, max_ptr)
            } else {
                let (min_ptr, max_ptr) =
                    broad_phase.get_static_bounds_pointers(collidable.broad_phase_index);
                (true, min_ptr, max_ptr)
            }
        } else {
            // There is no shape, so there can be no bounds.
            (false, std::ptr::null_mut(), std::ptr::null_mut())
        }
    }

    /// Gets a copy of the body's bounding box. If the body has no shape,
    /// the bounding box has a min at f32::MAX and a max at f32::MIN.
    pub fn bounding_box(&self) -> BoundingBox {
        unsafe {
            let (has_bounds, min_ptr, max_ptr) = self.get_bounds_references_from_broad_phase();
            if has_bounds {
                BoundingBox {
                    min: *min_ptr,
                    _pad0: 0.0,
                    max: *max_ptr,
                    _pad1: 0.0,
                }
            } else {
                BoundingBox {
                    min: Vec3::splat(f32::MAX),
                    _pad0: 0.0,
                    max: Vec3::splat(f32::MIN),
                    _pad1: 0.0,
                }
            }
        }
    }

    /// Gets a reference to the body's velocity.
    #[inline(always)]
    pub fn velocity(&self) -> &BodyVelocity {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        &set.dynamics_state.get(loc.index).motion.velocity
    }

    /// Gets a mutable reference to the body's velocity.
    #[inline(always)]
    pub fn velocity_mut(&self) -> &mut BodyVelocity {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        &mut set.dynamics_state.get_mut(loc.index).motion.velocity
    }

    /// Gets a reference to the body's pose.
    #[inline(never)]
    pub fn pose(&self) -> &RigidPose {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        &set.dynamics_state.get(loc.index).motion.pose
    }

    /// Gets a mutable reference to the body's pose.
    #[inline(never)]
    pub fn pose_mut(&self) -> &mut RigidPose {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        &mut set.dynamics_state.get_mut(loc.index).motion.pose
    }

    /// Gets a reference to the body's motion state, including both pose and velocity.
    #[inline(never)]
    pub fn motion_state(&self) -> &MotionState {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        &set.dynamics_state.get(loc.index).motion
    }

    /// Gets a mutable reference to the body's motion state.
    #[inline(never)]
    pub fn motion_state_mut(&self) -> &mut MotionState {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        &mut set.dynamics_state.get_mut(loc.index).motion
    }

    /// Gets a reference to the body's solver-relevant state (pose, velocity, and inertia).
    #[inline(never)]
    pub fn dynamics(&self) -> &BodyDynamics {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        set.dynamics_state.get(loc.index)
    }

    /// Gets a mutable reference to the body's dynamics state.
    #[inline(never)]
    pub fn dynamics_mut(&self) -> &mut BodyDynamics {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        set.dynamics_state.get_mut(loc.index)
    }

    /// Gets a reference to the body's collidable.
    #[inline(always)]
    pub fn collidable(&self) -> &Collidable {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        set.collidables.get(loc.index)
    }

    /// Gets a mutable reference to the body's collidable.
    #[inline(always)]
    pub fn collidable_mut(&self) -> &mut Collidable {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        set.collidables.get_mut(loc.index)
    }

    /// Gets a reference to the body's local inertia.
    #[inline(always)]
    pub fn local_inertia(&self) -> &BodyInertia {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        &set.dynamics_state.get(loc.index).inertia.local
    }

    /// Gets a mutable reference to the body's local inertia.
    #[inline(always)]
    pub fn local_inertia_mut(&self) -> &mut BodyInertia {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        &mut set.dynamics_state.get_mut(loc.index).inertia.local
    }

    /// Gets a reference to the body's activity state.
    #[inline(always)]
    pub fn activity(&self) -> &BodyActivity {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        set.activity.get(loc.index)
    }

    /// Gets a mutable reference to the body's activity state.
    #[inline(always)]
    pub fn activity_mut(&self) -> &mut BodyActivity {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        set.activity.get_mut(loc.index)
    }

    /// Gets a reference to the list of the body's connected constraints.
    #[inline(always)]
    pub fn constraints(&self) -> &QuickList<BodyConstraintReference> {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        set.constraints.get(loc.index)
    }

    /// Gets a mutable reference to the list of the body's connected constraints.
    #[inline(always)]
    pub fn constraints_mut(&self) -> &mut QuickList<BodyConstraintReference> {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        set.constraints.get_mut(loc.index)
    }

    /// Gets a `CollidableReference` for this body, encoding mobility and handle.
    pub fn collidable_reference(&self) -> CollidableReference {
        let mobility = if self.kinematic() {
            CollidableMobility::Kinematic
        } else {
            CollidableMobility::Dynamic
        };
        CollidableReference::from_body(mobility, self.handle)
    }

    /// Gets whether the body is kinematic (all inverse inertia and mass are zero).
    #[inline(always)]
    pub fn kinematic(&self) -> bool {
        Bodies::is_kinematic(self.local_inertia())
    }

    /// Gets whether the body has locked inertia (inverse inertia tensor is zero).
    #[inline(always)]
    pub fn has_locked_inertia(&self) -> bool {
        Bodies::has_locked_inertia(&self.local_inertia().inverse_inertia_tensor)
    }

    /// If the body is dynamic, turns it kinematic by setting all inverse inertia and mass values
    /// to zero and activates it. Connected constraints that now contain only kinematics are removed.
    pub fn become_kinematic(&self) {
        if !self.kinematic() {
            self.bodies_mut()
                .set_local_inertia(self.handle, &BodyInertia::default());
        }
    }

    /// Sets the body's local inertia. Wakes up the body and correctly handles dynamic/kinematic transitions.
    pub fn set_local_inertia(&self, local_inertia: &BodyInertia) {
        self.bodies_mut()
            .set_local_inertia(self.handle, local_inertia);
    }

    /// Computes the world space inverse inertia tensor for the body based on LocalInertia and Pose.
    #[inline(always)]
    pub fn compute_inverse_inertia(&self, inverse_inertia: &mut Symmetric3x3) {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        let state = set.dynamics_state.get(loc.index);
        PoseIntegration::rotate_inverse_inertia(
            &state.inertia.local.inverse_inertia_tensor,
            state.motion.pose.orientation,
            inverse_inertia,
        );
    }

    /// Gets a description of the body.
    pub fn get_description(&self, description: &mut BodyDescription) {
        self.bodies().get_description(self.handle, description);
    }

    /// Sets a body's properties according to a description.
    pub fn apply_description(&self, description: &BodyDescription) {
        self.bodies_mut()
            .apply_description(self.handle, description);
    }

    /// Changes the shape of a body.
    pub fn set_shape(&self, new_shape: TypedIndex) {
        self.bodies_mut().set_shape(self.handle, new_shape);
    }

    /// Updates the body's bounds in the broad phase for its current state.
    pub fn update_bounds(&self) {
        self.bodies_mut().update_bounds(self.handle);
    }

    // === Static impulse application methods ===

    /// Applies an impulse to a body given its state references.
    #[inline(always)]
    pub fn apply_impulse_static(
        impulse: Vec3,
        impulse_offset: Vec3,
        local_inertia: &mut BodyInertia,
        pose: &RigidPose,
        velocity: &mut BodyVelocity,
    ) {
        let mut inverse_inertia_tensor = Symmetric3x3::default();
        PoseIntegration::rotate_inverse_inertia(
            &local_inertia.inverse_inertia_tensor,
            pose.orientation,
            &mut inverse_inertia_tensor,
        );
        Self::apply_linear_impulse_to(impulse, local_inertia.inverse_mass, &mut velocity.linear);
        let angular_impulse = impulse_offset.cross(impulse);
        Self::apply_angular_impulse_to(
            angular_impulse,
            &inverse_inertia_tensor,
            &mut velocity.angular,
        );
    }

    /// Applies an impulse to a body by index in a body set.
    #[inline(always)]
    pub fn apply_impulse_by_index(set: &BodySet, index: i32, impulse: Vec3, impulse_offset: Vec3) {
        // Use raw pointer access to avoid creating &T then casting to *mut T (UB under Stacked Borrows).
        // Buffer wraps a raw pointer, so we go through it directly.
        let state_ptr =
            unsafe { set.dynamics_state.as_ptr().add(index as usize) as *mut BodyDynamics };
        unsafe {
            Self::apply_impulse_static(
                impulse,
                impulse_offset,
                &mut (*state_ptr).inertia.local,
                &(*state_ptr).motion.pose,
                &mut (*state_ptr).motion.velocity,
            );
        }
    }

    /// Applies an angular impulse to an angular velocity.
    #[inline(always)]
    pub fn apply_angular_impulse_to(
        angular_impulse: Vec3,
        inverse_inertia_tensor: &Symmetric3x3,
        angular_velocity: &mut Vec3,
    ) {
        let mut angular_velocity_change = Vec3::ZERO;
        Symmetric3x3::transform_without_overlap(
            &angular_impulse,
            inverse_inertia_tensor,
            &mut angular_velocity_change,
        );
        *angular_velocity += angular_velocity_change;
    }

    /// Applies a linear impulse to a linear velocity.
    #[inline(always)]
    pub fn apply_linear_impulse_to(impulse: Vec3, inverse_mass: f32, linear_velocity: &mut Vec3) {
        *linear_velocity += impulse * inverse_mass;
    }

    // === Instance impulse methods ===

    /// Applies an impulse to this body at the given world space offset. Does not modify activity state.
    #[inline(always)]
    pub fn apply_impulse(&self, impulse: Vec3, impulse_offset: Vec3) {
        let loc = self.memory_location();
        let set = self.bodies().sets.get(loc.set_index);
        Self::apply_impulse_by_index(set, loc.index, impulse, impulse_offset);
    }

    /// Applies a linear impulse to the body. Does not wake the body.
    #[inline(always)]
    pub fn apply_linear_impulse(&self, impulse: Vec3) {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        let state = set.dynamics_state.get_mut(loc.index);
        let inverse_mass = state.inertia.local.inverse_mass;
        Self::apply_linear_impulse_to(impulse, inverse_mass, &mut state.motion.velocity.linear);
    }

    /// Computes the velocity of a world space offset point attached to the body.
    #[inline(always)]
    pub fn get_velocity_for_offset(&self, offset: Vec3, velocity: &mut Vec3) {
        let vel = self.velocity();
        *velocity = vel.linear + vel.angular.cross(offset);
    }

    /// Applies an angular impulse to the body. Does not wake the body.
    #[inline(always)]
    pub fn apply_angular_impulse(&self, angular_impulse: Vec3) {
        let loc = self.memory_location();
        let set = self.bodies_mut().sets.get_mut(loc.set_index);
        let state = set.dynamics_state.get_mut(loc.index);
        let mut inverse_inertia = Symmetric3x3::default();
        PoseIntegration::rotate_inverse_inertia(
            &state.inertia.local.inverse_inertia_tensor,
            state.motion.pose.orientation,
            &mut inverse_inertia,
        );
        Self::apply_angular_impulse_to(
            angular_impulse,
            &inverse_inertia,
            &mut state.motion.velocity.angular,
        );
    }
}

/// Implicit conversion: `BodyReference` → `BodyHandle`.
impl From<BodyReference> for BodyHandle {
    fn from(reference: BodyReference) -> Self {
        reference.handle
    }
}
