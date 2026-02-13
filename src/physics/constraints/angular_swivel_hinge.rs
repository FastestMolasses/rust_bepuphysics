use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::helpers::Helpers;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::cmp::SimdPartialOrd;

/// Constrains two bodies with the angular component of a swivel hinge that allows rotation around two axes.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AngularSwivelHinge {
    /// Swivel axis in the local space of body A.
    pub local_swivel_axis_a: Vec3,
    /// Hinge axis in the local space of body B.
    pub local_hinge_axis_b: Vec3,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl AngularSwivelHinge {
    pub fn apply_description(
        &self,
        prestep_data: &mut AngularSwivelHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(
                self.local_swivel_axis_a,
                "AngularSwivelHinge",
                "local_swivel_axis_a",
            );
            ConstraintChecker::assert_unit_length_vec3(
                self.local_hinge_axis_b,
                "AngularSwivelHinge",
                "local_hinge_axis_b",
            );
            ConstraintChecker::assert_valid_spring_settings(
                &self.spring_settings,
                "AngularSwivelHinge",
            );
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_swivel_axis_a, &mut target.local_swivel_axis_a);
        Vector3Wide::write_first(self.local_hinge_axis_b, &mut target.local_hinge_axis_b);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &AngularSwivelHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut AngularSwivelHinge,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(
            &source.local_swivel_axis_a,
            &mut description.local_swivel_axis_a,
        );
        Vector3Wide::read_first(
            &source.local_hinge_axis_b,
            &mut description.local_hinge_axis_b,
        );
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularSwivelHingePrestepData {
    pub local_swivel_axis_a: Vector3Wide,
    pub local_hinge_axis_b: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
}

pub struct AngularSwivelHingeFunctions;

impl AngularSwivelHingeFunctions {
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
        local_swivel_axis_a: &Vector3Wide,
        local_hinge_axis_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        swivel_axis: &mut Vector3Wide,
        hinge_axis: &mut Vector3Wide,
        jacobian_a: &mut Vector3Wide,
    ) {
        QuaternionWide::transform_without_overlap(local_swivel_axis_a, orientation_a, swivel_axis);
        QuaternionWide::transform_without_overlap(local_hinge_axis_b, orientation_b, hinge_axis);
        unsafe { Vector3Wide::cross_without_overlap(swivel_axis, hinge_axis, jacobian_a) };
        // In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
        let mut fallback_jacobian = Vector3Wide::default();
        Helpers::find_perpendicular(swivel_axis, &mut fallback_jacobian);
        let mut jacobian_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(jacobian_a, jacobian_a, &mut jacobian_length_squared);
        let use_fallback = jacobian_length_squared
            .simd_lt(Vector::<f32>::splat(1e-3))
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
        prestep: &AngularSwivelHingePrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut swivel_axis = Vector3Wide::default();
        let mut hinge_axis = Vector3Wide::default();
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.local_swivel_axis_a,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            &mut swivel_axis,
            &mut hinge_axis,
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
        _inverse_dt: f32,
        prestep: &AngularSwivelHingePrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut swivel_axis = Vector3Wide::default();
        let mut hinge_axis = Vector3Wide::default();
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.local_swivel_axis_a,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            &mut swivel_axis,
            &mut hinge_axis,
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
        let mut angular_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&impulse_to_velocity_a, &jacobian_a, &mut angular_a);
        let mut angular_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&negated_impulse_to_velocity_b, &jacobian_a, &mut angular_b);

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
        let effective_mass = effective_mass_cfm_scale / (angular_a + angular_b);

        let mut error = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&hinge_axis, &swivel_axis, &mut error);
        // Note the negation: we want to oppose the separation.
        let bias_velocity = -(position_error_to_velocity * error);

        // JB = -JA. (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
        let mut difference = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut difference);
        let mut csv = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&difference, &jacobian_a, &mut csv);
        let csi =
            effective_mass * (bias_velocity - csv) - *accumulated_impulses * softness_impulse_scale;

        *accumulated_impulses = *accumulated_impulses + csi;
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
        _prestep_data: &mut AngularSwivelHingePrestepData,
    ) {
    }
}

pub struct AngularSwivelHingeTypeProcessor;

impl AngularSwivelHingeTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 24;
}
