// Translated from BepuPhysics/Bodies_GatherScatter.cs
//
// This file extends Bodies with gather/scatter operations for the solver.
// The C# version uses AVX intrinsics for 8-wide SIMD on x86. For ARM compatibility
// (Apple Silicon), we translate only the fallback (scalar) paths. Platform-specific
// SIMD paths can be added later behind #[cfg(target_arch = "x86_64")] gates.

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{
    BodyDynamics, BodyInertiaWide, BodyVelocityWide, MotionState,
};
use crate::physics::constraints::body_access_filter::IBodyAccessFilter;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::{Quat, Vec3};

// Float offsets within a MotionState (64 bytes = 16 floats):
//   [0..3]  = orientation (x, y, z, w)
//   [4..6]  = position (x, y, z)
//   [7]     = padding
//   [8..10] = linear velocity (x, y, z)
//   [11]    = padding
//   [12..14] = angular velocity (x, y, z)
//   [15]    = padding
impl MotionState {
    pub const OFFSET_TO_ORIENTATION_X: usize = 0;
    pub const OFFSET_TO_ORIENTATION_Y: usize = 1;
    pub const OFFSET_TO_ORIENTATION_Z: usize = 2;
    pub const OFFSET_TO_ORIENTATION_W: usize = 3;
    pub const OFFSET_TO_POSITION_X: usize = 4;
    pub const OFFSET_TO_POSITION_Y: usize = 5;
    pub const OFFSET_TO_POSITION_Z: usize = 6;
    pub const OFFSET_TO_LINEAR_X: usize = 8;
    pub const OFFSET_TO_LINEAR_Y: usize = 9;
    pub const OFFSET_TO_LINEAR_Z: usize = 10;
    pub const OFFSET_TO_ANGULAR_X: usize = 12;
    pub const OFFSET_TO_ANGULAR_Y: usize = 13;
    pub const OFFSET_TO_ANGULAR_Z: usize = 14;
}

impl Bodies {
    #[inline(always)]
    fn write_gather_inertia(
        index: i32,
        body_index_in_bundle: usize,
        states: &Buffer<BodyDynamics>,
        gathered_inertias: &mut BodyInertiaWide,
    ) {
        let source = &states.get(index).inertia.world;
        unsafe {
            let target_slot =
                GatherScatter::get_offset_instance_mut(gathered_inertias, body_index_in_bundle);
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.xx) =
                source.inverse_inertia_tensor.xx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.yx) =
                source.inverse_inertia_tensor.yx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.yy) =
                source.inverse_inertia_tensor.yy;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zx) =
                source.inverse_inertia_tensor.zx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zy) =
                source.inverse_inertia_tensor.zy;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zz) =
                source.inverse_inertia_tensor.zz;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_mass) = source.inverse_mass;
        }
    }

    #[inline(always)]
    fn write_gather_motion_state(
        index: i32,
        body_index_in_bundle: usize,
        states: &Buffer<BodyDynamics>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        let state = &states.get(index).motion;
        unsafe {
            Vector3Wide::write_first(
                state.pose.position,
                GatherScatter::get_offset_instance_mut(position, body_index_in_bundle),
            );
            QuaternionWide::write_first(
                state.pose.orientation,
                GatherScatter::get_offset_instance_mut(orientation, body_index_in_bundle),
            );
            Vector3Wide::write_first(
                state.velocity.linear,
                GatherScatter::get_offset_instance_mut(&mut velocity.linear, body_index_in_bundle),
            );
            Vector3Wide::write_first(
                state.velocity.angular,
                GatherScatter::get_offset_instance_mut(&mut velocity.angular, body_index_in_bundle),
            );
        }
    }

    unsafe fn fallback_gather_motion_state(
        states: *const BodyDynamics,
        encoded_body_indices: &Vector<i32>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        let p_position_x = &mut position.x as *mut Vector<f32> as *mut f32;
        let p_position_y = &mut position.y as *mut Vector<f32> as *mut f32;
        let p_position_z = &mut position.z as *mut Vector<f32> as *mut f32;
        let p_orientation_x = &mut orientation.x as *mut Vector<f32> as *mut f32;
        let p_orientation_y = &mut orientation.y as *mut Vector<f32> as *mut f32;
        let p_orientation_z = &mut orientation.z as *mut Vector<f32> as *mut f32;
        let p_orientation_w = &mut orientation.w as *mut Vector<f32> as *mut f32;
        let p_linear_x = &mut velocity.linear.x as *mut Vector<f32> as *mut f32;
        let p_linear_y = &mut velocity.linear.y as *mut Vector<f32> as *mut f32;
        let p_linear_z = &mut velocity.linear.z as *mut Vector<f32> as *mut f32;
        let p_angular_x = &mut velocity.angular.x as *mut Vector<f32> as *mut f32;
        let p_angular_y = &mut velocity.angular.y as *mut Vector<f32> as *mut f32;
        let p_angular_z = &mut velocity.angular.z as *mut Vector<f32> as *mut f32;

        for i in 0..Vector::<i32>::LEN {
            let encoded_body_index = encoded_body_indices[i];
            if encoded_body_index < 0 {
                continue;
            }
            let state_values = states
                .offset((encoded_body_index & Bodies::BODY_REFERENCE_MASK) as isize)
                as *const f32;
            *p_position_x.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_X);
            *p_position_y.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_Y);
            *p_position_z.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_Z);
            *p_orientation_x.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_X);
            *p_orientation_y.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_Y);
            *p_orientation_z.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_Z);
            *p_orientation_w.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_W);
            *p_linear_x.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_X);
            *p_linear_y.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_Y);
            *p_linear_z.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_Z);
            *p_angular_x.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_X);
            *p_angular_y.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_Y);
            *p_angular_z.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_Z);
        }
    }

    unsafe fn fallback_gather_inertia(
        states: *const BodyDynamics,
        encoded_body_indices: &Vector<i32>,
        inertia: &mut BodyInertiaWide,
        offset_in_floats: usize,
    ) {
        let p_mass = &mut inertia.inverse_mass as *mut Vector<f32> as *mut f32;
        let p_inertia_xx = &mut inertia.inverse_inertia_tensor.xx as *mut Vector<f32> as *mut f32;
        let p_inertia_yx = &mut inertia.inverse_inertia_tensor.yx as *mut Vector<f32> as *mut f32;
        let p_inertia_yy = &mut inertia.inverse_inertia_tensor.yy as *mut Vector<f32> as *mut f32;
        let p_inertia_zx = &mut inertia.inverse_inertia_tensor.zx as *mut Vector<f32> as *mut f32;
        let p_inertia_zy = &mut inertia.inverse_inertia_tensor.zy as *mut Vector<f32> as *mut f32;
        let p_inertia_zz = &mut inertia.inverse_inertia_tensor.zz as *mut Vector<f32> as *mut f32;

        for i in 0..Vector::<i32>::LEN {
            let encoded_body_index = encoded_body_indices[i];
            if encoded_body_index < 0 {
                continue;
            }
            let inertia_values = (states
                .offset((encoded_body_index & Bodies::BODY_REFERENCE_MASK) as isize)
                as *const f32)
                .add(offset_in_floats);
            *p_inertia_xx.add(i) = *inertia_values.add(0);
            *p_inertia_yx.add(i) = *inertia_values.add(1);
            *p_inertia_yy.add(i) = *inertia_values.add(2);
            *p_inertia_zx.add(i) = *inertia_values.add(3);
            *p_inertia_zy.add(i) = *inertia_values.add(4);
            *p_inertia_zz.add(i) = *inertia_values.add(5);
            *p_mass.add(i) = *inertia_values.add(6);
        }
    }

    /// Transposes a bundle of array-of-structures layout motion states into a bundle of
    /// array-of-structures-of-arrays layout.
    /// Buffer size must be no larger than `Vector::<f32>::LEN`.
    pub unsafe fn transpose_motion_states(
        states: &Buffer<MotionState>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        debug_assert!(states.len() > 0 && (states.len() as usize) <= Vector::<f32>::LEN);
        // Scalar fallback (works on all architectures).
        for i in 0..states.len() as usize {
            let state = states.get(i as i32);
            Vector3Wide::write_slot(state.pose.position, i, position);
            QuaternionWide::write_slot(state.pose.orientation, i, orientation);
            Vector3Wide::write_slot(state.velocity.linear, i, &mut velocity.linear);
            Vector3Wide::write_slot(state.velocity.angular, i, &mut velocity.angular);
        }
    }

    /// Gathers state (position, orientation, velocity, inertia) from per-body AOS storage into
    /// AOSOA wide bundles, using the fallback scalar path.
    pub unsafe fn gather_state<TAccessFilter: IBodyAccessFilter>(
        &self,
        encoded_body_indices: &Vector<i32>,
        world_inertia: bool,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
        inertia: &mut BodyInertiaWide,
    ) {
        let solver_states = self.active_set().dynamics_state.as_ptr();

        // Scalar fallback — works on all architectures.
        Self::fallback_gather_motion_state(
            solver_states,
            encoded_body_indices,
            position,
            orientation,
            velocity,
        );
        let offset = if world_inertia { 24 } else { 16 };
        Self::fallback_gather_inertia(solver_states, encoded_body_indices, inertia, offset);
    }

    /// Scatters pose (position + orientation) from wide bundles back to per-body AOS storage.
    pub unsafe fn scatter_pose(
        &self,
        position: &Vector3Wide,
        orientation: &QuaternionWide,
        encoded_body_indices: &Vector<i32>,
        mask: &Vector<i32>,
    ) {
        // Scalar fallback.
        for inner_index in 0..Vector::<i32>::LEN {
            if mask[inner_index] == 0 {
                continue;
            }
            let body_index = encoded_body_indices[inner_index];
            let pose = &mut (*self.active_set_dynamics_ptr().add(body_index as usize))
                .motion
                .pose;
            pose.position = Vec3::new(
                position.x[inner_index],
                position.y[inner_index],
                position.z[inner_index],
            );
            pose.orientation = Quat::from_xyzw(
                orientation.x[inner_index],
                orientation.y[inner_index],
                orientation.z[inner_index],
                orientation.w[inner_index],
            );
        }
    }

    /// Scatters inertia from wide bundles back to per-body AOS storage (world inertia).
    pub unsafe fn scatter_inertia(
        &self,
        inertia: &BodyInertiaWide,
        encoded_body_indices: &Vector<i32>,
        mask: &Vector<i32>,
    ) {
        // Scalar fallback.
        for inner_index in 0..Vector::<i32>::LEN {
            if mask[inner_index] == 0 {
                continue;
            }
            let body_index = encoded_body_indices[inner_index];
            let target = &mut (*self.active_set_dynamics_ptr().add(body_index as usize))
                .inertia
                .world;
            target.inverse_inertia_tensor.xx = inertia.inverse_inertia_tensor.xx[inner_index];
            target.inverse_inertia_tensor.yx = inertia.inverse_inertia_tensor.yx[inner_index];
            target.inverse_inertia_tensor.yy = inertia.inverse_inertia_tensor.yy[inner_index];
            target.inverse_inertia_tensor.zx = inertia.inverse_inertia_tensor.zx[inner_index];
            target.inverse_inertia_tensor.zy = inertia.inverse_inertia_tensor.zy[inner_index];
            target.inverse_inertia_tensor.zz = inertia.inverse_inertia_tensor.zz[inner_index];
            target.inverse_mass = inertia.inverse_mass[inner_index];
        }
    }

    /// Scatters velocities from wide bundles back to per-body AOS storage.
    pub unsafe fn scatter_velocities<TAccessFilter: IBodyAccessFilter>(
        &self,
        source_velocities: &BodyVelocityWide,
        encoded_body_indices: &Vector<i32>,
    ) {
        let indices = encoded_body_indices;
        // Scalar fallback.
        for inner_index in 0..Vector::<i32>::LEN {
            if (indices[inner_index] as u32) >= Bodies::DYNAMIC_LIMIT {
                continue;
            }
            let body_index = indices[inner_index];
            let target = &mut (*self.active_set_dynamics_ptr().add(body_index as usize))
                .motion
                .velocity;
            if TAccessFilter::access_linear_velocity() {
                target.linear = Vec3::new(
                    source_velocities.linear.x[inner_index],
                    source_velocities.linear.y[inner_index],
                    source_velocities.linear.z[inner_index],
                );
            }
            if TAccessFilter::access_angular_velocity() {
                target.angular = Vec3::new(
                    source_velocities.angular.x[inner_index],
                    source_velocities.angular.y[inner_index],
                    source_velocities.angular.z[inner_index],
                );
            }
        }
    }

    /// Helper to get a raw pointer to the DynamicsState buffer of the active set.
    #[inline(always)]
    unsafe fn active_set_dynamics_ptr(&self) -> *mut BodyDynamics {
        self.active_set().dynamics_state.as_ptr() as *mut BodyDynamics
    }
}
