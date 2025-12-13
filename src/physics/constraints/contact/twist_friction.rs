// Translated from BepuPhysics/Constraints/Contact/TwistFriction.cs

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::cmp::SimdPartialEq;
use std::simd::num::SimdFloat;

/// Handles the twist friction implementation.
pub struct TwistFriction;

impl TwistFriction {
    /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
    #[inline(always)]
    pub fn apply_impulse(
        angular_jacobian_a: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        corrective_impulse: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let world_corrective_impulse_a =
            Vector3Wide::scale(angular_jacobian_a, corrective_impulse);
        let mut world_corrective_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &world_corrective_impulse_a,
            &inertia_a.inverse_inertia_tensor,
            &mut world_corrective_velocity_a,
        );
        let mut world_corrective_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &world_corrective_impulse_a,
            &inertia_b.inverse_inertia_tensor,
            &mut world_corrective_velocity_b,
        );
        let temp = wsv_a.angular;
        Vector3Wide::add(
            &temp,
            &world_corrective_velocity_a,
            &mut wsv_a.angular,
        );
        let temp = wsv_b.angular;
        Vector3Wide::subtract(
            &temp,
            &world_corrective_velocity_b,
            &mut wsv_b.angular,
        );
    }

    #[inline(always)]
    pub fn compute_corrective_impulse(
        angular_jacobian_a: &Vector3Wide,
        effective_mass: &Vector<f32>,
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector<f32>,
        corrective_csi: &mut Vector<f32>,
    ) {
        let mut csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.angular, angular_jacobian_a, &mut csv_a);
        let mut negated_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.angular, angular_jacobian_a, &mut negated_csv_b);
        let negated_csi = (csv_a - negated_csv_b) * *effective_mass; //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

        let previous_accumulated = *accumulated_impulse;
        *accumulated_impulse = (*accumulated_impulse - negated_csi)
            .simd_min(*maximum_impulse)
            .simd_max(-*maximum_impulse);

        *corrective_csi = *accumulated_impulse - previous_accumulated;
    }

    #[inline(always)]
    pub fn warm_start(
        angular_jacobian_a: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        accumulated_impulse: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        Self::apply_impulse(
            angular_jacobian_a,
            inertia_a,
            inertia_b,
            accumulated_impulse,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        angular_jacobian_a: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        //Compute effective mass matrix contributions. No linear contributions for the twist constraint.
        //Note that we use the angularJacobianA (that is, the normal) for both, despite angularJacobianB = -angularJacobianA. That's fine- J * M * JT is going to be positive regardless.
        let mut angular_a = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(
            angular_jacobian_a,
            &inertia_a.inverse_inertia_tensor,
            &mut angular_a,
        );
        let mut angular_b = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(
            angular_jacobian_a,
            &inertia_b.inverse_inertia_tensor,
            &mut angular_b,
        );

        //No softening; this constraint is rigid by design.
        //Note that we have to guard against two bodies with infinite inertias. This is a valid state!
        //(We do not have to do such guarding on constraints with linear jacobians; dynamic bodies cannot have zero *mass*.)
        let inverse_effective_mass = angular_a + angular_b;
        let inverse_is_zero = inverse_effective_mass.simd_eq(Vector::<f32>::splat(0.0));
        let effective_mass = inverse_is_zero.select(
            Vector::<f32>::splat(0.0),
            Vector::<f32>::splat(1.0) / inverse_effective_mass,
        );

        //Note that friction constraints have no bias velocity. They target zero velocity.
        let mut corrective_csi = Vector::<f32>::splat(0.0);
        Self::compute_corrective_impulse(
            angular_jacobian_a,
            &effective_mass,
            wsv_a,
            wsv_b,
            maximum_impulse,
            accumulated_impulse,
            &mut corrective_csi,
        );
        Self::apply_impulse(
            angular_jacobian_a,
            inertia_a,
            inertia_b,
            &corrective_csi,
            wsv_a,
            wsv_b,
        );
    }
}
