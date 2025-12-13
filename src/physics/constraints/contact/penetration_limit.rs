// Translated from BepuPhysics/Constraints/Contact/PenetrationLimit.cs

use crate::out_unsafe;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::num::SimdFloat;

pub struct PenetrationLimit;

impl PenetrationLimit {
    #[inline(always)]
    pub fn compute_corrective_impulse(
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        normal: &Vector3Wide,
        angular_a: &Vector3Wide,
        angular_b: &Vector3Wide,
        bias_velocity: &Vector<f32>,
        softness_impulse_scale: &Vector<f32>,
        effective_mass: &Vector<f32>,
        accumulated_impulse: &mut Vector<f32>,
        corrective_csi: &mut Vector<f32>,
    ) {
        //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
        let mut csva_linear = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.linear, normal, &mut csva_linear);
        let mut csva_angular = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.angular, angular_a, &mut csva_angular);
        let mut negated_csvb_linear = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.linear, normal, &mut negated_csvb_linear);
        let mut csvb_angular = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.angular, angular_b, &mut csvb_angular);
        //Compute negated version to avoid the need for an explicit negate.
        let negated_csi = *accumulated_impulse * *softness_impulse_scale
            + (csva_linear - negated_csvb_linear + csva_angular + csvb_angular - *bias_velocity)
                * *effective_mass;

        let previous_accumulated = *accumulated_impulse;
        *accumulated_impulse =
            (*accumulated_impulse - negated_csi).simd_max(Vector::<f32>::splat(0.0));

        *corrective_csi = *accumulated_impulse - previous_accumulated;
    }

    #[inline(always)]
    pub fn update_penetration_depth(
        dt: &Vector<f32>,
        contact_offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        normal: &Vector3Wide,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        penetration_depth: &mut Vector<f32>,
    ) {
        //The normal is calibrated to point from B to A. Any movement of A along N results in a decrease in depth. Any movement of B along N results in an increase in depth.
        //estimatedPenetrationDepthChange = dot(normal, velocityDtA.Linear + velocityDtA.Angular x contactOffsetA) - dot(normal, velocityDtB.Linear + velocityDtB.Angular x contactOffsetB)
        let wxra = out_unsafe!(Vector3Wide::cross_without_overlap(&velocity_a.angular, contact_offset_a));
        let mut contact_velocity_a = Vector3Wide::default();
        Vector3Wide::add(&wxra, &velocity_a.linear, &mut contact_velocity_a);

        let mut contact_offset_b = Vector3Wide::default();
        Vector3Wide::subtract(contact_offset_a, offset_b, &mut contact_offset_b);
        let wxrb = out_unsafe!(Vector3Wide::cross_without_overlap(&velocity_b.angular, &contact_offset_b));
        let mut contact_velocity_b = Vector3Wide::default();
        Vector3Wide::add(&wxrb, &velocity_b.linear, &mut contact_velocity_b);

        let mut contact_velocity_difference = Vector3Wide::default();
        Vector3Wide::subtract(
            &contact_velocity_a,
            &contact_velocity_b,
            &mut contact_velocity_difference,
        );
        let mut estimated_depth_change_velocity = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            normal,
            &contact_velocity_difference,
            &mut estimated_depth_change_velocity,
        );
        *penetration_depth = *penetration_depth - estimated_depth_change_velocity * *dt;
    }

    #[inline(always)]
    pub fn apply_impulse(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        normal: &Vector3Wide,
        angular_a: &Vector3Wide,
        angular_b: &Vector3Wide,
        corrective_impulse: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let linear_velocity_change_a = *corrective_impulse * inertia_a.inverse_mass;
        let corrective_velocity_a_linear_velocity =
            Vector3Wide::scale(normal, &linear_velocity_change_a);
        let corrective_angular_impulse_a = Vector3Wide::scale(angular_a, corrective_impulse);
        let mut corrective_velocity_a_angular_velocity = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &corrective_angular_impulse_a,
            &inertia_a.inverse_inertia_tensor,
            &mut corrective_velocity_a_angular_velocity,
        );

        let linear_velocity_change_b = *corrective_impulse * inertia_b.inverse_mass;
        let corrective_velocity_b_linear_velocity =
            Vector3Wide::scale(normal, &linear_velocity_change_b);
        let corrective_angular_impulse_b = Vector3Wide::scale(angular_b, corrective_impulse);
        let mut corrective_velocity_b_angular_velocity = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &corrective_angular_impulse_b,
            &inertia_b.inverse_inertia_tensor,
            &mut corrective_velocity_b_angular_velocity,
        );

        let temp = wsv_a.linear;
        Vector3Wide::add(
            &temp,
            &corrective_velocity_a_linear_velocity,
            &mut wsv_a.linear,
        );
        let temp = wsv_a.angular;
        Vector3Wide::add(
            &temp,
            &corrective_velocity_a_angular_velocity,
            &mut wsv_a.angular,
        );
        let temp = wsv_b.linear;
        Vector3Wide::subtract(
            &temp,
            &corrective_velocity_b_linear_velocity,
            &mut wsv_b.linear,
        ); //Note subtract; normal = -jacobianLinearB
        let temp = wsv_b.angular;
        Vector3Wide::add(
            &temp,
            &corrective_velocity_b_angular_velocity,
            &mut wsv_b.angular,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        normal: &Vector3Wide,
        contact_offset_a: &Vector3Wide,
        contact_offset_b: &Vector3Wide,
        accumulated_impulse: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let angular_a = out_unsafe!(Vector3Wide::cross_without_overlap(contact_offset_a, normal));
        let angular_b = out_unsafe!(Vector3Wide::cross_without_overlap(normal, contact_offset_b));
        Self::apply_impulse(
            inertia_a,
            inertia_b,
            normal,
            &angular_a,
            &angular_b,
            accumulated_impulse,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        normal: &Vector3Wide,
        contact_offset_a: &Vector3Wide,
        contact_offset_b: &Vector3Wide,
        depth: &Vector<f32>,
        position_error_to_velocity: &Vector<f32>,
        effective_mass_cfm_scale: &Vector<f32>,
        maximum_recovery_velocity: &Vector<f32>,
        inverse_dt: &Vector<f32>,
        softness_impulse_scale: &Vector<f32>,
        accumulated_impulse: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
        let angular_a = out_unsafe!(Vector3Wide::cross_without_overlap(contact_offset_a, normal));
        let angular_b = out_unsafe!(Vector3Wide::cross_without_overlap(normal, contact_offset_b));

        //effective mass
        let mut angular_a0 = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(
            &angular_a,
            &inertia_a.inverse_inertia_tensor,
            &mut angular_a0,
        );
        let mut angular_b0 = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(
            &angular_b,
            &inertia_b.inverse_inertia_tensor,
            &mut angular_b0,
        );

        //Linear effective mass contribution notes:
        //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
        //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
        let linear = inertia_a.inverse_mass + inertia_b.inverse_mass;
        //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
        let effective_mass = *effective_mass_cfm_scale / (linear + angular_a0 + angular_b0);

        //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
        let bias_velocity = (*depth * *inverse_dt)
            .simd_min((*depth * *position_error_to_velocity).simd_min(*maximum_recovery_velocity));

        let mut corrective_csi = Vector::<f32>::splat(0.0);
        Self::compute_corrective_impulse(
            wsv_a,
            wsv_b,
            normal,
            &angular_a,
            &angular_b,
            &bias_velocity,
            softness_impulse_scale,
            &effective_mass,
            accumulated_impulse,
            &mut corrective_csi,
        );
        Self::apply_impulse(
            inertia_a,
            inertia_b,
            normal,
            &angular_a,
            &angular_b,
            &corrective_csi,
            wsv_a,
            wsv_b,
        );
    }
}
