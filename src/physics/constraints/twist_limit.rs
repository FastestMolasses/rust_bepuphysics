use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::constraints::inequality_helpers::InequalityHelpers;
use crate::physics::constraints::twist_servo::TwistServoFunctions;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use glam::Quat;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Constrains two bodies' rotations around attached twist axes to a range of permitted twist angles.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct TwistLimit {
    /// Local space basis attached to body A. The Z axis is the twist axis.
    pub local_basis_a: Quat,
    /// Local space basis attached to body B. The Z axis is the twist axis.
    pub local_basis_b: Quat,
    /// Minimum angle between B's axis and A's measurement axis.
    pub minimum_angle: f32,
    /// Maximum angle between B's axis and A's measurement axis.
    pub maximum_angle: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl TwistLimit {
    pub fn apply_description(
        &self,
        prestep_data: &mut TwistLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_quat(self.local_basis_a, "TwistLimit", "local_basis_a");
            ConstraintChecker::assert_unit_length_quat(self.local_basis_b, "TwistLimit", "local_basis_b");
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "TwistLimit");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        QuaternionWide::write_first(self.local_basis_a, &mut target.local_basis_a);
        QuaternionWide::write_first(self.local_basis_b, &mut target.local_basis_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.minimum_angle) = self.minimum_angle;
            *GatherScatter::get_first_mut(&mut target.maximum_angle) = self.maximum_angle;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &TwistLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        QuaternionWide::read_first(&source.local_basis_a, &mut description.local_basis_a);
        QuaternionWide::read_first(&source.local_basis_b, &mut description.local_basis_b);
        description.minimum_angle = unsafe { *GatherScatter::get_first(&source.minimum_angle) };
        description.maximum_angle = unsafe { *GatherScatter::get_first(&source.maximum_angle) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TwistLimitPrestepData {
    pub local_basis_a: QuaternionWide,
    pub local_basis_b: QuaternionWide,
    pub minimum_angle: Vector<f32>,
    pub maximum_angle: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct TwistLimitFunctions;

impl TwistLimitFunctions {
    #[inline(always)]
    fn compute_jacobian(
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_basis_a: &QuaternionWide,
        local_basis_b: &QuaternionWide,
        minimum_angle: &Vector<f32>,
        maximum_angle: &Vector<f32>,
        error: &mut Vector<f32>,
        jacobian_a: &mut Vector3Wide,
    ) {
        let mut basis_bx = Vector3Wide::default();
        let mut basis_bz = Vector3Wide::default();
        let mut basis_a = Matrix3x3Wide::default();
        TwistServoFunctions::compute_jacobian_full(
            orientation_a, orientation_b, local_basis_a, local_basis_b,
            &mut basis_bx, &mut basis_bz, &mut basis_a, jacobian_a,
        );
        let mut angle = Vector::<f32>::splat(0.0);
        TwistServoFunctions::compute_current_angle(&basis_bx, &basis_bz, &basis_a, &mut angle);
        // For simplicity, the solve iterations can only apply a positive impulse.
        // So, the jacobians get flipped when necessary.
        let mut min_error = Vector::<f32>::splat(0.0);
        math_helper::get_signed_angle_difference(minimum_angle, &angle, &mut min_error);
        let mut max_error = Vector::<f32>::splat(0.0);
        math_helper::get_signed_angle_difference(maximum_angle, &angle, &mut max_error);
        let use_min = min_error.abs().simd_lt(max_error.abs()).to_int();

        // If we use the maximum bound, flip the jacobian.
        *error = use_min.simd_lt(Vector::<i32>::splat(0)).select(-min_error, max_error);
        let mut negated_jacobian_a = Vector3Wide::default();
        Vector3Wide::negate(jacobian_a, &mut negated_jacobian_a);
        *jacobian_a = Vector3Wide::conditional_select(&use_min, &negated_jacobian_a, jacobian_a);
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &TwistLimitPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut error = Vector::<f32>::splat(0.0);
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            orientation_a, orientation_b, &prestep.local_basis_a, &prestep.local_basis_b,
            &prestep.minimum_angle, &prestep.maximum_angle, &mut error, &mut jacobian_a,
        );
        let mut impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&jacobian_a, &inertia_a.inverse_inertia_tensor, &mut impulse_to_velocity_a);
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&jacobian_a, &inertia_b.inverse_inertia_tensor, &mut negated_impulse_to_velocity_b);
        TwistServoFunctions::apply_impulse(&mut wsv_a.angular, &mut wsv_b.angular, &impulse_to_velocity_a, &negated_impulse_to_velocity_b, accumulated_impulses);
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
        prestep: &TwistLimitPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut error = Vector::<f32>::splat(0.0);
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(
            orientation_a, orientation_b, &prestep.local_basis_a, &prestep.local_basis_b,
            &prestep.minimum_angle, &prestep.maximum_angle, &mut error, &mut jacobian_a,
        );

        let mut impulse_to_velocity_a = Vector3Wide::default();
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut effective_mass = Vector::<f32>::splat(0.0);
        let mut velocity_to_impulse_a = Vector3Wide::default();
        TwistServoFunctions::compute_effective_mass(
            dt,
            &prestep.spring_settings,
            &inertia_a.inverse_inertia_tensor,
            &inertia_b.inverse_inertia_tensor,
            &jacobian_a,
            &mut impulse_to_velocity_a,
            &mut negated_impulse_to_velocity_b,
            &mut position_error_to_velocity,
            &mut softness_impulse_scale,
            &mut effective_mass,
            &mut velocity_to_impulse_a,
        );

        // In the speculative case, allow the limit to be approached.
        let zero = Vector::<f32>::splat(0.0);
        let speculative_mask = error.simd_lt(zero);
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let bias_velocity = speculative_mask.select(error * inverse_dt_wide, error * position_error_to_velocity);
        let bias_impulse = bias_velocity * effective_mass;

        let mut net_velocity = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut net_velocity);
        let mut csi_velocity_component = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&net_velocity, &velocity_to_impulse_a, &mut csi_velocity_component);
        // csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - csiVelocityComponent
        let mut csi = bias_impulse - *accumulated_impulses * softness_impulse_scale - csi_velocity_component;
        InequalityHelpers::clamp_positive(accumulated_impulses, &mut csi);

        TwistServoFunctions::apply_impulse(&mut wsv_a.angular, &mut wsv_b.angular, &impulse_to_velocity_a, &negated_impulse_to_velocity_b, &csi);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut TwistLimitPrestepData,
    ) {
    }
}

pub struct TwistLimitTypeProcessor;

impl TwistLimitTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 27;
}
