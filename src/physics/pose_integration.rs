// Translated from BepuPhysics/PoseIntegrator.cs
// Only the static helper class `PoseIntegration` and related enums/traits are included here.
// The generic `PoseIntegrator<TCallbacks>` class has heavy dependencies and is deferred.

use std::simd::prelude::*;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocity, BodyVelocityWide, RigidPose};
use crate::utilities::math_helper;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3::Symmetric3x3;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Defines how a pose integrator should handle angular velocity integration.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AngularIntegrationMode {
    /// Angular velocity is directly integrated and does not change as the body pose changes.
    /// Does not conserve angular momentum.
    Nonconserving = 0,
    /// Approximately conserves angular momentum by updating the angular velocity according to the change in orientation.
    /// Does a decent job for gyroscopes, but angular velocities will tend to drift towards a minimal inertia axis.
    ConserveMomentum = 1,
    /// Approximately conserves angular momentum by including an implicit gyroscopic torque.
    /// Best option for Dzhanibekov effect simulation, but applies a damping effect that can make gyroscopes less useful.
    ConserveMomentumWithGyroscopicTorque = 2,
}

/// Defines a type that handles callbacks for body pose integration.
///
/// Corresponds to `IPoseIntegratorCallbacks` in C#.
pub trait IPoseIntegratorCallbacks {
    /// Gets how the pose integrator should handle angular velocity integration.
    fn angular_integration_mode(&self) -> AngularIntegrationMode;

    /// Gets whether the integrator should use only one step for unconstrained bodies when using a substepping solver.
    /// If true, unconstrained bodies use a single step of length equal to the dt provided to Simulation.Timestep.
    /// If false, unconstrained bodies will be integrated with the same number of substeps as the constrained bodies in the solver.
    fn allow_substeps_for_unconstrained_bodies(&self) -> bool;

    /// Gets whether the velocity integration callback should be called for kinematic bodies.
    /// If true, `integrate_velocity` will be called for bundles including kinematic bodies.
    /// If false, kinematic bodies will just continue using whatever velocity they have set.
    /// Most use cases should return false.
    fn integrate_velocity_for_kinematics(&self) -> bool;

    /// Performs any required initialization logic after the Simulation instance has been constructed.
    ///
    /// # Safety
    /// The caller must ensure that `simulation` is a valid pointer.
    unsafe fn initialize(&mut self, simulation: *mut u8);

    /// Callback invoked ahead of dispatches that may call into `integrate_velocity`.
    /// It may be called more than once with different values over a frame.
    fn prepare_for_integration(&mut self, dt: f32);

    /// Callback for a bundle of bodies being integrated.
    fn integrate_velocity(
        &self,
        body_indices: Vector<i32>,
        position: Vector3Wide,
        orientation: QuaternionWide,
        local_inertia: BodyInertiaWide,
        integration_mask: Vector<i32>,
        worker_index: i32,
        dt: Vector<f32>,
        velocity: &mut BodyVelocityWide,
    );
}

/// Provides helper functions for integrating body poses.
///
/// Corresponds to the `static class PoseIntegration` in C#.
pub struct PoseIntegration;

impl PoseIntegration {
    /// Rotates an inverse inertia tensor from local space to world space (scalar version).
    #[inline(always)]
    pub fn rotate_inverse_inertia(
        local_inverse_inertia_tensor: &Symmetric3x3,
        orientation: glam::Quat,
        rotated_inverse_inertia_tensor: &mut Symmetric3x3,
    ) {
        use crate::utilities::matrix3x3::Matrix3x3;
        let mut orientation_matrix = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&orientation, &mut orientation_matrix);
        // I^-1 = RT * Ilocal^-1 * R
        // NOTE: If you were willing to confuse users a little bit, the local inertia could be required to be diagonal.
        // This would be totally fine for all the primitive types which happen to have diagonal inertias, but for more complex shapes (convex hulls, meshes),
        // there would need to be a reorientation step. That could be confusing, and it's probably not worth it.
        Symmetric3x3::rotation_sandwich(
            &orientation_matrix,
            local_inverse_inertia_tensor,
            rotated_inverse_inertia_tensor,
        );
    }

    /// Integrates a position by linear velocity over dt (scalar version).
    #[inline(always)]
    pub fn integrate_position(
        position: glam::Vec3,
        linear_velocity: glam::Vec3,
        dt: f32,
        integrated_position: &mut glam::Vec3,
    ) {
        let displacement = linear_velocity * dt;
        *integrated_position = position + displacement;
    }

    /// Integrates an orientation by angular velocity over dt (scalar version).
    #[inline(always)]
    pub fn integrate_orientation(
        orientation: glam::Quat,
        angular_velocity: glam::Vec3,
        dt: f32,
        integrated_orientation: &mut glam::Quat,
    ) {
        // Note that we don't bother with conservation of angular momentum or the gyroscopic term or anything else.
        // All orientation integration assumes a series of piecewise linear integrations.
        // That's not entirely correct, but it's a reasonable approximation that means we don't have to worry about
        // conservation of angular momentum or gyroscopic terms when dealing with CCD sweeps.
        let speed = angular_velocity.length();
        if speed > 1e-15f32 {
            let half_angle = speed * dt * 0.5;
            let scale = math_helper::sin(half_angle) / speed;
            let q = glam::Quat::from_xyzw(
                angular_velocity.x * scale,
                angular_velocity.y * scale,
                angular_velocity.z * scale,
                math_helper::cos(half_angle),
            );
            // Note that the input and output may overlap.
            *integrated_orientation = quaternion_ex::concatenate(orientation, q);
            quaternion_ex::normalize_into(integrated_orientation);
        } else {
            *integrated_orientation = orientation;
        }
    }

    /// Integrates an orientation by angular velocity over half_dt (wide/SIMD version).
    pub fn integrate_orientation_wide(
        start: &QuaternionWide,
        angular_velocity: &Vector3Wide,
        half_dt: &Vector<f32>,
        integrated: &mut QuaternionWide,
    ) {
        let speed = angular_velocity.length();
        let half_angle = speed * half_dt;
        let s = math_helper::sin_simd(half_angle);
        let scale = s / speed;
        let q = QuaternionWide {
            x: angular_velocity.x * scale,
            y: angular_velocity.y * scale,
            z: angular_velocity.z * scale,
            w: math_helper::cos_simd(half_angle),
        };
        let mut end = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(start, &q, &mut end);
        end = QuaternionWide::normalize(end);
        let speed_valid = speed.simd_gt(Vector::<f32>::splat(1e-15));
        integrated.x = speed_valid.select(end.x, start.x);
        integrated.y = speed_valid.select(end.y, start.y);
        integrated.z = speed_valid.select(end.z, start.z);
        integrated.w = speed_valid.select(end.w, start.w);
    }

    /// Rotates an inverse inertia tensor from local space to world space (wide/SIMD version).
    #[inline(always)]
    pub fn rotate_inverse_inertia_wide(
        local_inverse_inertia_tensor: &Symmetric3x3Wide,
        orientation: &QuaternionWide,
        rotated_inverse_inertia_tensor: &mut Symmetric3x3Wide,
    ) {
        let mut orientation_matrix = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation, &mut orientation_matrix);
        // I^-1 = RT * Ilocal^-1 * R
        Symmetric3x3Wide::rotation_sandwich(
            &orientation_matrix,
            local_inverse_inertia_tensor,
            rotated_inverse_inertia_tensor,
        );
    }

    /// Uses the previous angular velocity if attempting to conserve angular momentum introduced infinities or NaNs.
    /// Happens when attempting to conserve momentum with a kinematic or partially inertia locked body.
    #[inline(always)]
    fn fallback_if_inertia_incompatible(
        previous_angular_velocity: &Vector3Wide,
        angular_velocity: &mut Vector3Wide,
    ) {
        let infinity = Vector::<f32>::splat(f32::INFINITY);
        let use_new_velocity = angular_velocity.x.abs().simd_lt(infinity)
            & angular_velocity.y.abs().simd_lt(infinity)
            & angular_velocity.z.abs().simd_lt(infinity);
        angular_velocity.x =
            use_new_velocity.select(angular_velocity.x, previous_angular_velocity.x);
        angular_velocity.y =
            use_new_velocity.select(angular_velocity.y, previous_angular_velocity.y);
        angular_velocity.z =
            use_new_velocity.select(angular_velocity.z, previous_angular_velocity.z);
    }

    /// Approximately conserves angular momentum by updating the angular velocity according to the change in orientation.
    #[inline(always)]
    pub fn integrate_angular_velocity_conserve_momentum(
        previous_orientation: &QuaternionWide,
        local_inverse_inertia: &Symmetric3x3Wide,
        world_inverse_inertia: &Symmetric3x3Wide,
        angular_velocity: &mut Vector3Wide,
    ) {
        // Note that this effectively recomputes the previous frame's inertia. There may not have been a previous inertia stored in the inertias buffer.
        // This just avoids the need for quite a bit of complexity around keeping the world inertias buffer updated with adds/removes/moves and other state changes that we can't easily track.
        // Also, even if it were cached, the memory bandwidth requirements of loading another inertia tensor would hurt multithreaded scaling enough to eliminate any performance advantage.
        let mut previous_orientation_matrix = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(
            previous_orientation,
            &mut previous_orientation_matrix,
        );

        let mut local_previous_angular_velocity = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            angular_velocity,
            &previous_orientation_matrix,
            &mut local_previous_angular_velocity,
        );

        let mut local_inertia_tensor: Symmetric3x3Wide = unsafe { std::mem::zeroed() };
        Symmetric3x3Wide::invert(local_inverse_inertia, &mut local_inertia_tensor);

        let mut local_angular_momentum = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &local_previous_angular_velocity,
            &local_inertia_tensor,
            &mut local_angular_momentum,
        );

        let mut angular_momentum = Vector3Wide::default();
        Matrix3x3Wide::transform(
            &local_angular_momentum,
            &previous_orientation_matrix,
            &mut angular_momentum,
        );

        let previous_velocity = *angular_velocity;
        Symmetric3x3Wide::transform_without_overlap(
            &angular_momentum,
            world_inverse_inertia,
            angular_velocity,
        );
        Self::fallback_if_inertia_incompatible(&previous_velocity, angular_velocity);
    }

    /// Approximately conserves angular momentum by including an implicit gyroscopic torque.
    #[inline(always)]
    pub fn integrate_angular_velocity_conserve_momentum_with_gyroscopic_torque(
        orientation: &QuaternionWide,
        local_inverse_inertia: &Symmetric3x3Wide,
        angular_velocity: &mut Vector3Wide,
        dt: &Vector<f32>,
    ) {
        // Integrating the gyroscopic force explicitly can result in some instability, so we'll use an approximate implicit approach.
        // angularVelocity1 * inertia1 = angularVelocity0 * inertia1 + dt * ((angularVelocity1 * inertia1) x angularVelocity1)
        // We transform all velocities into local space using the current orientation for the calculation.
        // f(localAngularVelocity1) = (localAngularVelocity1 - localAngularVelocity0) * localInertia + dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia))
        // Not trivial to solve for localAngularVelocity1 so we'll do so numerically with a newton iteration.
        // df/dw1(f(localAngularVelocity1)) = localInertia + dt * (skew(localAngularVelocity1) * localInertia - skew(localAngularVelocity1 * localInertia))
        let mut orientation_matrix = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation, &mut orientation_matrix);

        // Using localAngularVelocity0 as the first guess for localAngularVelocity1.
        let mut local_angular_velocity = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            angular_velocity,
            &orientation_matrix,
            &mut local_angular_velocity,
        );

        let mut local_inertia_tensor: Symmetric3x3Wide = unsafe { std::mem::zeroed() };
        Symmetric3x3Wide::invert(local_inverse_inertia, &mut local_inertia_tensor);

        let mut local_angular_momentum = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &local_angular_velocity,
            &local_inertia_tensor,
            &mut local_angular_momentum,
        );

        let residual =
            *dt * Vector3Wide::cross_new(&local_angular_momentum, &local_angular_velocity);

        let skew_momentum = Matrix3x3Wide::create_cross_product(&local_angular_momentum);
        let skew_velocity = Matrix3x3Wide::create_cross_product(&local_angular_velocity);
        let transformed_skew_velocity = skew_velocity * local_inertia_tensor;
        let mut change_over_dt = Matrix3x3Wide::default();
        Matrix3x3Wide::subtract(
            &transformed_skew_velocity,
            &skew_momentum,
            &mut change_over_dt,
        );
        let mut change = Matrix3x3Wide::default();
        Matrix3x3Wide::scale(&change_over_dt, dt, &mut change);
        let jacobian = local_inertia_tensor + change;

        let mut inverse_jacobian = Matrix3x3Wide::default();
        Matrix3x3Wide::invert(&jacobian, &mut inverse_jacobian);
        let mut newton_step = Vector3Wide::default();
        Matrix3x3Wide::transform(&residual, &inverse_jacobian, &mut newton_step);
        // Use a temporary to avoid aliased borrows.
        let mut updated = Vector3Wide::default();
        Vector3Wide::subtract(&local_angular_velocity, &newton_step, &mut updated);
        local_angular_velocity = updated;

        let previous_velocity = *angular_velocity;
        Matrix3x3Wide::transform(
            &local_angular_velocity,
            &orientation_matrix,
            angular_velocity,
        );
        Self::fallback_if_inertia_incompatible(&previous_velocity, angular_velocity);
    }

    /// Integrates a rigid pose by body velocity over dt (scalar version).
    #[inline(always)]
    pub fn integrate_pose(
        pose: &RigidPose,
        velocity: &BodyVelocity,
        dt: f32,
        integrated_pose: &mut RigidPose,
    ) {
        Self::integrate_position(
            pose.position,
            velocity.linear,
            dt,
            &mut integrated_pose.position,
        );
        Self::integrate_orientation(
            pose.orientation,
            velocity.angular,
            dt,
            &mut integrated_pose.orientation,
        );
    }
}
