use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Quat;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Constrains two bodies to maintain a target twist angle around body-attached axes.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct TwistServo {
    /// Local space basis attached to body A. The Z axis is the twist axis.
    pub local_basis_a: Quat,
    /// Local space basis attached to body B. The Z axis is the twist axis.
    pub local_basis_b: Quat,
    /// Target angle between B's axis and A's measurement axis.
    pub target_angle: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
}

impl TwistServo {
    pub fn apply_description(
        &self,
        prestep_data: &mut TwistServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_quat(
                self.local_basis_a,
                "TwistServo",
                "local_basis_a",
            );
            ConstraintChecker::assert_unit_length_quat(
                self.local_basis_b,
                "TwistServo",
                "local_basis_b",
            );
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "TwistServo");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        QuaternionWide::write_first(self.local_basis_a, &mut target.local_basis_a);
        QuaternionWide::write_first(self.local_basis_b, &mut target.local_basis_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.target_angle) = self.target_angle;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
    }

    pub fn build_description(
        prestep_data: &TwistServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        QuaternionWide::read_first(&source.local_basis_a, &mut description.local_basis_a);
        QuaternionWide::read_first(&source.local_basis_b, &mut description.local_basis_b);
        description.target_angle = unsafe { *GatherScatter::get_first(&source.target_angle) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TwistServoPrestepData {
    pub local_basis_a: QuaternionWide,
    pub local_basis_b: QuaternionWide,
    pub target_angle: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
    pub servo_settings: ServoSettingsWide,
}

pub struct TwistServoFunctions;

impl TwistServoFunctions {
    /// Computes the jacobian and intermediate basis data (4-arg overload with full basis output).
    #[inline(always)]
    pub fn compute_jacobian_full(
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_basis_a: &QuaternionWide,
        local_basis_b: &QuaternionWide,
        basis_bx: &mut Vector3Wide,
        basis_bz: &mut Vector3Wide,
        basis_a: &mut Matrix3x3Wide,
        jacobian_a: &mut Vector3Wide,
    ) {
        let mut basis_quaternion_a = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            local_basis_a,
            orientation_a,
            &mut basis_quaternion_a,
        );
        let mut basis_quaternion_b = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            local_basis_b,
            orientation_b,
            &mut basis_quaternion_b,
        );

        QuaternionWide::transform_unit_xz(&basis_quaternion_b, basis_bx, basis_bz);
        Matrix3x3Wide::create_from_quaternion(&basis_quaternion_a, basis_a);
        // Protect against singularity when axes point at each other.
        Vector3Wide::add(&basis_a.z, basis_bz, jacobian_a);
        let mut length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(jacobian_a, &mut length);
        let inv_length = Vector::<f32>::splat(1.0) / length;
        let mut scaled = Vector3Wide::default();
        Vector3Wide::scale_to(jacobian_a, &inv_length, &mut scaled);
        let use_fallback = length.simd_lt(Vector::<f32>::splat(1e-10)).to_int();
        *jacobian_a = Vector3Wide::conditional_select(&use_fallback, &basis_a.z, &scaled);
    }

    /// Computes the current twist angle from basis data.
    #[inline(always)]
    pub fn compute_current_angle(
        basis_bx: &Vector3Wide,
        basis_bz: &Vector3Wide,
        basis_a: &Matrix3x3Wide,
        angle: &mut Vector<f32>,
    ) {
        let mut aligning_rotation = QuaternionWide::default();
        QuaternionWide::get_quaternion_between_normalized_vectors(
            basis_bz,
            &basis_a.z,
            &mut aligning_rotation,
        );
        let mut aligned_basis_bx = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            basis_bx,
            &aligning_rotation,
            &mut aligned_basis_bx,
        );
        let mut x = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&aligned_basis_bx, &basis_a.x, &mut x);
        let mut y = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&aligned_basis_bx, &basis_a.y, &mut y);
        let abs_angle = math_helper::acos_simd(x);
        let zero = Vector::<f32>::splat(0.0);
        *angle = y.simd_lt(zero).select(-abs_angle, abs_angle);
    }

    /// Computes intermediate effective mass contributions (shared between servo, motor, and limit).
    #[inline(always)]
    pub fn compute_effective_mass_contributions(
        inverse_inertia_a: &Symmetric3x3Wide,
        inverse_inertia_b: &Symmetric3x3Wide,
        jacobian_a: &Vector3Wide,
        impulse_to_velocity_a: &mut Vector3Wide,
        negated_impulse_to_velocity_b: &mut Vector3Wide,
        unsoftened_inverse_effective_mass: &mut Vector<f32>,
    ) {
        Symmetric3x3Wide::transform_without_overlap(
            jacobian_a,
            inverse_inertia_a,
            impulse_to_velocity_a,
        );
        Symmetric3x3Wide::transform_without_overlap(
            jacobian_a,
            inverse_inertia_b,
            negated_impulse_to_velocity_b,
        );
        let mut angular_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(impulse_to_velocity_a, jacobian_a, &mut angular_a);
        let mut angular_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(negated_impulse_to_velocity_b, jacobian_a, &mut angular_b);
        *unsoftened_inverse_effective_mass = angular_a + angular_b;
    }

    /// Computes the full effective mass, including spring softness.
    #[inline(always)]
    pub fn compute_effective_mass(
        dt: f32,
        spring_settings: &SpringSettingsWide,
        inverse_inertia_a: &Symmetric3x3Wide,
        inverse_inertia_b: &Symmetric3x3Wide,
        jacobian_a: &Vector3Wide,
        impulse_to_velocity_a: &mut Vector3Wide,
        negated_impulse_to_velocity_b: &mut Vector3Wide,
        position_error_to_velocity: &mut Vector<f32>,
        softness_impulse_scale: &mut Vector<f32>,
        effective_mass: &mut Vector<f32>,
        velocity_to_impulse_a: &mut Vector3Wide,
    ) {
        let mut unsoftened_inverse_effective_mass = Vector::<f32>::splat(0.0);
        Self::compute_effective_mass_contributions(
            inverse_inertia_a,
            inverse_inertia_b,
            jacobian_a,
            impulse_to_velocity_a,
            negated_impulse_to_velocity_b,
            &mut unsoftened_inverse_effective_mass,
        );

        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            spring_settings,
            dt,
            position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            softness_impulse_scale,
        );
        *effective_mass = effective_mass_cfm_scale / unsoftened_inverse_effective_mass;
        Vector3Wide::scale_to(jacobian_a, effective_mass, velocity_to_impulse_a);
    }

    /// Applies a 1D scalar impulse along the twist jacobian.
    #[inline(always)]
    pub fn apply_impulse(
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
        impulse_to_velocity_a: &Vector3Wide,
        negated_impulse_to_velocity_b: &Vector3Wide,
        csi: &Vector<f32>,
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

    /// Computes the jacobian (simplified 1-arg overload, only returns jacobianA).
    #[inline(always)]
    pub fn compute_jacobian_simple(
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_basis_a: &QuaternionWide,
        local_basis_b: &QuaternionWide,
        jacobian_a: &mut Vector3Wide,
    ) {
        let mut basis_quaternion_a = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            local_basis_a,
            orientation_a,
            &mut basis_quaternion_a,
        );
        let mut basis_quaternion_b = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            local_basis_b,
            orientation_b,
            &mut basis_quaternion_b,
        );

        let basis_a_z = QuaternionWide::transform_unit_z(basis_quaternion_a);
        let basis_b_z = QuaternionWide::transform_unit_z(basis_quaternion_b);
        // Protect against singularity when axes point at each other.
        Vector3Wide::add(&basis_a_z, &basis_b_z, jacobian_a);
        let mut length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(jacobian_a, &mut length);
        let inv_length = Vector::<f32>::splat(1.0) / length;
        let mut scaled = Vector3Wide::default();
        Vector3Wide::scale_to(jacobian_a, &inv_length, &mut scaled);
        let use_fallback = length.simd_lt(Vector::<f32>::splat(1e-10)).to_int();
        *jacobian_a = Vector3Wide::conditional_select(&use_fallback, &basis_a_z, &scaled);
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &TwistServoPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian_simple(
            orientation_a,
            orientation_b,
            &prestep.local_basis_a,
            &prestep.local_basis_b,
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
            &mut wsv_a.angular,
            &mut wsv_b.angular,
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            accumulated_impulses,
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
        prestep: &TwistServoPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut basis_bx = Vector3Wide::default();
        let mut basis_bz = Vector3Wide::default();
        let mut basis_a = Matrix3x3Wide::default();
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian_full(
            orientation_a,
            orientation_b,
            &prestep.local_basis_a,
            &prestep.local_basis_b,
            &mut basis_bx,
            &mut basis_bz,
            &mut basis_a,
            &mut jacobian_a,
        );

        let mut impulse_to_velocity_a = Vector3Wide::default();
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut effective_mass = Vector::<f32>::splat(0.0);
        let mut velocity_to_impulse_a = Vector3Wide::default();
        Self::compute_effective_mass(
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

        let mut angle = Vector::<f32>::splat(0.0);
        Self::compute_current_angle(&basis_bx, &basis_bz, &basis_a, &mut angle);

        let mut error = Vector::<f32>::splat(0.0);
        math_helper::get_signed_angle_difference(&prestep.target_angle, &angle, &mut error);

        let mut clamped_bias_velocity = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_1d(
            &error,
            &position_error_to_velocity,
            &prestep.servo_settings,
            dt,
            inverse_dt,
            &mut clamped_bias_velocity,
            &mut maximum_impulse,
        );
        let bias_impulse = clamped_bias_velocity * effective_mass;

        let mut net_velocity = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut net_velocity);
        let mut csi_velocity_component = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &net_velocity,
            &velocity_to_impulse_a,
            &mut csi_velocity_component,
        );
        // csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - csiVelocityComponent
        let mut csi =
            bias_impulse - *accumulated_impulses * softness_impulse_scale - csi_velocity_component;
        let previous_accumulated_impulse = *accumulated_impulses;
        *accumulated_impulses = (*accumulated_impulses + csi)
            .simd_max(-maximum_impulse)
            .simd_min(maximum_impulse);
        csi = *accumulated_impulses - previous_accumulated_impulse;

        Self::apply_impulse(
            &mut wsv_a.angular,
            &mut wsv_b.angular,
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            &csi,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut TwistServoPrestepData,
    ) {
    }
}

pub struct TwistServoTypeProcessor;

impl TwistServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 26;
}
