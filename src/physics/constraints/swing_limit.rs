use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::inequality_helpers::InequalityHelpers;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::helpers::Helpers;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Restricts axes attached to two bodies to fall within a maximum swing angle.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SwingLimit {
    /// Axis attached to body A in its local space.
    pub axis_local_a: Vec3,
    /// Axis attached to body B in its local space.
    pub axis_local_b: Vec3,
    /// Minimum dot product between the world space A and B axes.
    pub minimum_dot: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl SwingLimit {
    /// Gets the maximum swing angle based on MinimumDot.
    pub fn maximum_swing_angle(&self) -> f32 {
        self.minimum_dot.acos()
    }

    /// Sets the maximum swing angle, updating MinimumDot.
    pub fn set_maximum_swing_angle(&mut self, value: f32) {
        self.minimum_dot = value.cos();
    }

    pub fn apply_description(
        &self,
        prestep_data: &mut SwingLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(
                self.axis_local_a,
                "SwingLimit",
                "axis_local_a",
            );
            ConstraintChecker::assert_unit_length_vec3(
                self.axis_local_b,
                "SwingLimit",
                "axis_local_b",
            );
            debug_assert!(
                self.minimum_dot >= -1.0 && self.minimum_dot <= 1.0,
                "SwingLimit.MinimumDot must be from -1 to 1 inclusive."
            );
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "SwingLimit");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.axis_local_a, &mut target.axis_local_a);
        Vector3Wide::write_first(self.axis_local_b, &mut target.axis_local_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.minimum_dot) = self.minimum_dot;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &SwingLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut SwingLimit,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.axis_local_a, &mut description.axis_local_a);
        Vector3Wide::read_first(&source.axis_local_b, &mut description.axis_local_b);
        description.minimum_dot = unsafe { *GatherScatter::get_first(&source.minimum_dot) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SwingLimitPrestepData {
    pub axis_local_a: Vector3Wide,
    pub axis_local_b: Vector3Wide,
    pub minimum_dot: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct SwingLimitFunctions;

impl SwingLimitFunctions {
    #[inline(always)]
    fn apply_impulse(
        impulse_to_velocity_a: &Vector3Wide,
        negated_impulse_to_velocity_b: &Vector3Wide,
        csi: &Vector<f32>,
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
    ) {
        let mut velocity_change_a = Vector3Wide::default();
        Vector3Wide::scale_to(impulse_to_velocity_a, csi, &mut velocity_change_a);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity_a, &velocity_change_a, &mut tmp);
        *angular_velocity_a = tmp;
        let mut negated_velocity_change_b = Vector3Wide::default();
        Vector3Wide::scale_to(
            negated_impulse_to_velocity_b,
            csi,
            &mut negated_velocity_change_b,
        );
        Vector3Wide::subtract(angular_velocity_b, &negated_velocity_change_b, &mut tmp);
        *angular_velocity_b = tmp;
    }

    #[inline(always)]
    fn compute_jacobian(
        axis_local_a: &Vector3Wide,
        axis_local_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        axis_a: &mut Vector3Wide,
        axis_b: &mut Vector3Wide,
        jacobian_a: &mut Vector3Wide,
    ) {
        QuaternionWide::transform_without_overlap(axis_local_a, orientation_a, axis_a);
        QuaternionWide::transform_without_overlap(axis_local_b, orientation_b, axis_b);
        unsafe { Vector3Wide::cross_without_overlap(axis_a, axis_b, jacobian_a) };
        // In the event that the axes are parallel, use a fallback.
        let mut fallback_jacobian = Vector3Wide::default();
        Helpers::find_perpendicular(axis_a, &mut fallback_jacobian);
        let mut jacobian_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(jacobian_a, jacobian_a, &mut jacobian_length_squared);
        let use_fallback = jacobian_length_squared
            .simd_lt(Vector::<f32>::splat(1e-7))
            .to_int();
        *jacobian_a =
            Vector3Wide::conditional_select(&use_fallback, &fallback_jacobian, jacobian_a);
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &SwingLimitPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut axis_a = Vector3Wide::default();
        let mut axis_b = Vector3Wide::default();
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.axis_local_a,
            &prestep.axis_local_b,
            orientation_a,
            orientation_b,
            &mut axis_a,
            &mut axis_b,
            &mut jacobian_a,
        );
        let mut impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &jacobian_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &jacobian_a,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            accumulated_impulses,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &SwingLimitPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut axis_a = Vector3Wide::default();
        let mut axis_b = Vector3Wide::default();
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.axis_local_a,
            &prestep.axis_local_b,
            orientation_a,
            orientation_b,
            &mut axis_a,
            &mut axis_b,
            &mut jacobian_a,
        );

        let mut impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &jacobian_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &jacobian_a,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        let mut angular_contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &impulse_to_velocity_a,
            &jacobian_a,
            &mut angular_contribution_a,
        );
        let mut angular_contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &negated_impulse_to_velocity_b,
            &jacobian_a,
            &mut angular_contribution_b,
        );

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            &prestep.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let effective_mass =
            effective_mass_cfm_scale / (angular_contribution_a + angular_contribution_b);

        let mut axis_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&axis_a, &axis_b, &mut axis_dot);
        let error = axis_dot - prestep.minimum_dot;
        // Note the negation: we want to oppose the separation.
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let bias_velocity = -(error * inverse_dt_wide).simd_min(error * position_error_to_velocity);

        // JB = -JA. (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
        let mut difference = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut difference);
        let mut csv = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&difference, &jacobian_a, &mut csv);
        let mut csi =
            effective_mass * (bias_velocity - csv) - *accumulated_impulses * softness_impulse_scale;

        InequalityHelpers::clamp_positive(accumulated_impulses, &mut csi);
        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            &csi,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut SwingLimitPrestepData,
    ) {
    }
}

pub struct SwingLimitTypeProcessor;

impl SwingLimitTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 25;
}
