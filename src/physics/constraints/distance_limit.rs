use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::constraints::inequality_helpers::InequalityHelpers;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use glam::Vec3;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Constrains points on two bodies to be separated by a distance within a range.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DistanceLimit {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Minimum distance permitted between the attachment points.
    pub minimum_distance: f32,
    /// Maximum distance permitted between the attachment points.
    pub maximum_distance: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl DistanceLimit {
    #[inline(always)]
    pub fn new(local_offset_a: Vec3, local_offset_b: Vec3, minimum_distance: f32, maximum_distance: f32, spring_settings: SpringSettings) -> Self {
        Self {
            local_offset_a,
            local_offset_b,
            minimum_distance,
            maximum_distance,
            spring_settings,
        }
    }

    pub fn apply_description(
        &self,
        prestep_data: &mut DistanceLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(self.minimum_distance >= 0.0, "DistanceLimit.MinimumDistance must be nonnegative.");
            debug_assert!(self.maximum_distance >= 0.0, "DistanceLimit.MaximumDistance must be nonnegative.");
            debug_assert!(self.maximum_distance >= self.minimum_distance, "DistanceLimit.MaximumDistance must be >= MinimumDistance.");
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "DistanceLimit");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.minimum_distance) = self.minimum_distance;
            *GatherScatter::get_first_mut(&mut target.maximum_distance) = self.maximum_distance;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &DistanceLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut DistanceLimit,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        description.minimum_distance = unsafe { *GatherScatter::get_first(&source.minimum_distance) };
        description.maximum_distance = unsafe { *GatherScatter::get_first(&source.maximum_distance) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct DistanceLimitPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub minimum_distance: Vector<f32>,
    pub maximum_distance: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct DistanceLimitFunctions;

impl DistanceLimitFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        linear_jacobian_a: &Vector3Wide,
        angular_jacobian_a: &Vector3Wide,
        angular_jacobian_b: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        csi: &Vector<f32>,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
    ) {
        let impulse_scaled_linear_jacobian = *linear_jacobian_a * *csi;
        // velocityA.Linear += impulseScaledLinearJacobian * inverseMassA
        let linear_change_a = impulse_scaled_linear_jacobian * inertia_a.inverse_mass;
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&velocity_a.linear, &linear_change_a, &mut tmp);
        velocity_a.linear = tmp;
        // velocityB.Linear -= impulseScaledLinearJacobian * inverseMassB
        let linear_change_b = impulse_scaled_linear_jacobian * inertia_b.inverse_mass;
        Vector3Wide::subtract(&velocity_b.linear, &linear_change_b, &mut tmp);
        velocity_b.linear = tmp;
        // velocityA.Angular += (angularJacobianA * csi) * inertiaA.InverseInertiaTensor
        let angular_impulse_a = *angular_jacobian_a * *csi;
        let mut angular_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_a, &inertia_a.inverse_inertia_tensor, &mut angular_change_a);
        Vector3Wide::add(&velocity_a.angular, &angular_change_a, &mut tmp);
        velocity_a.angular = tmp;
        // velocityB.Angular += (angularJacobianB * csi) * inertiaB.InverseInertiaTensor
        let angular_impulse_b = *angular_jacobian_b * *csi;
        let mut angular_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_b, &inertia_b.inverse_inertia_tensor, &mut angular_change_b);
        Vector3Wide::add(&velocity_b.angular, &angular_change_b, &mut tmp);
        velocity_b.angular = tmp;
    }

    #[inline(always)]
    pub fn compute_jacobians(
        local_offset_a: &Vector3Wide,
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        local_offset_b: &Vector3Wide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        minimum_distance: &Vector<f32>,
        maximum_distance: &Vector<f32>,
        use_minimum: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        direction: &mut Vector3Wide,
        angular_ja: &mut Vector3Wide,
        angular_jb: &mut Vector3Wide,
    ) {
        let mut offset_a = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_offset_a, orientation_a, &mut offset_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_offset_b, orientation_b, &mut offset_b);
        // anchorOffset = (offsetB - offsetA) + (positionB - positionA)
        let mut anchor_offset = Vector3Wide::default();
        let mut tmp1 = Vector3Wide::default();
        Vector3Wide::subtract(&offset_b, &offset_a, &mut tmp1);
        let mut tmp2 = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut tmp2);
        Vector3Wide::add(&tmp1, &tmp2, &mut anchor_offset);

        Vector3Wide::length_into(&anchor_offset, distance);
        // If the current distance is closer to the minimum, calibrate for the minimum.
        *use_minimum = (*distance - *minimum_distance).abs().simd_lt((*distance - *maximum_distance).abs()).to_int();
        let sign = use_minimum.simd_lt(Vector::<i32>::splat(0)).select(Vector::<f32>::splat(-1.0), Vector::<f32>::splat(1.0));
        let scale_factor = sign / *distance;
        Vector3Wide::scale_to(&anchor_offset, &scale_factor, direction);
        // If the distance is too short to extract a direction, use a fallback.
        let need_fallback = distance.simd_lt(Vector::<f32>::splat(1e-9));
        direction.x = need_fallback.select(Vector::<f32>::splat(1.0), direction.x);
        direction.y = need_fallback.select(Vector::<f32>::splat(0.0), direction.y);
        direction.z = need_fallback.select(Vector::<f32>::splat(0.0), direction.z);

        unsafe {
            Vector3Wide::cross_without_overlap(&offset_a, direction, angular_ja);
            Vector3Wide::cross_without_overlap(direction, &offset_b, angular_jb); // Note flip negation
        }
    }

    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &DistanceLimitPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut use_minimum = Vector::<i32>::splat(0);
        let mut distance = Vector::<f32>::splat(0.0);
        let mut direction = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &prestep.local_offset_a, position_a, orientation_a,
            &prestep.local_offset_b, position_b, orientation_b,
            &prestep.minimum_distance, &prestep.maximum_distance,
            &mut use_minimum, &mut distance, &mut direction, &mut angular_ja, &mut angular_jb,
        );
        Self::apply_impulse(&direction, &angular_ja, &angular_jb, inertia_a, inertia_b, accumulated_impulses, wsv_a, wsv_b);
    }

    #[inline(always)]
    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &DistanceLimitPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut use_minimum = Vector::<i32>::splat(0);
        let mut distance = Vector::<f32>::splat(0.0);
        let mut direction = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &prestep.local_offset_a, position_a, orientation_a,
            &prestep.local_offset_b, position_b, orientation_b,
            &prestep.minimum_distance, &prestep.maximum_distance,
            &mut use_minimum, &mut distance, &mut direction, &mut angular_ja, &mut angular_jb,
        );

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csv
        let mut linear_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.linear, &direction, &mut linear_csv_a);
        let mut negated_linear_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.linear, &direction, &mut negated_linear_csv_b);
        let mut angular_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.angular, &angular_ja, &mut angular_csv_a);
        let mut angular_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.angular, &angular_jb, &mut angular_csv_b);
        let csv = linear_csv_a - negated_linear_csv_b + angular_csv_a + angular_csv_b;

        // The linear jacobian contributions are just a scalar multiplication by 1 since it's unit length.
        let mut angular_contribution_a = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(&angular_ja, &inertia_a.inverse_inertia_tensor, &mut angular_contribution_a);
        let mut angular_contribution_b = Vector::<f32>::splat(0.0);
        Symmetric3x3Wide::vector_sandwich(&angular_jb, &inertia_b.inverse_inertia_tensor, &mut angular_contribution_b);
        let inverse_effective_mass = inertia_a.inverse_mass + inertia_b.inverse_mass + angular_contribution_a + angular_contribution_b;

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(&prestep.spring_settings, dt, &mut position_error_to_velocity, &mut effective_mass_cfm_scale, &mut softness_impulse_scale);
        let effective_mass = effective_mass_cfm_scale / inverse_effective_mass;
        let error = use_minimum.simd_lt(Vector::<i32>::splat(0)).select(
            prestep.minimum_distance - distance,
            distance - prestep.maximum_distance,
        );
        let mut bias_velocity = Vector::<f32>::splat(0.0);
        InequalityHelpers::compute_bias_velocity(error, &position_error_to_velocity, inverse_dt, &mut bias_velocity);
        let mut csi = -*accumulated_impulses * softness_impulse_scale - effective_mass * (csv - bias_velocity);
        InequalityHelpers::clamp_positive(accumulated_impulses, &mut csi);

        Self::apply_impulse(&direction, &angular_ja, &angular_jb, inertia_a, inertia_b, &csi, wsv_a, wsv_b);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut DistanceLimitPrestepData,
    ) {
    }
}

/// Handles the solve iterations of a bunch of distance limit constraints.
pub struct DistanceLimitTypeProcessor;

impl DistanceLimitTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 34;
}
