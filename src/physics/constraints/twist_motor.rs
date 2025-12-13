use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::twist_servo::TwistServoFunctions;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use glam::Vec3;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Constrains the twist velocity between two bodies to a target.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct TwistMotor {
    /// Local twist axis attached to body A.
    pub local_axis_a: Vec3,
    /// Local twist axis attached to body B.
    pub local_axis_b: Vec3,
    /// Goal relative twist velocity around the body axes.
    pub target_velocity: f32,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl TwistMotor {
    pub fn apply_description(
        &self,
        prestep_data: &mut TwistMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(self.local_axis_a, "TwistMotor", "local_axis_a");
            ConstraintChecker::assert_unit_length_vec3(self.local_axis_b, "TwistMotor", "local_axis_b");
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "TwistMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_axis_a, &mut target.local_axis_a);
        Vector3Wide::write_first(self.local_axis_b, &mut target.local_axis_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.target_velocity) = self.target_velocity;
        }
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &TwistMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_axis_a, &mut description.local_axis_a);
        Vector3Wide::read_first(&source.local_axis_b, &mut description.local_axis_b);
        description.target_velocity = unsafe { *GatherScatter::get_first(&source.target_velocity) };
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TwistMotorPrestepData {
    pub local_axis_a: Vector3Wide,
    pub local_axis_b: Vector3Wide,
    pub target_velocity: Vector<f32>,
    pub settings: MotorSettingsWide,
}

pub struct TwistMotorFunctions;

impl TwistMotorFunctions {
    #[inline(always)]
    fn compute_jacobian(
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_axis_a: &Vector3Wide,
        local_axis_b: &Vector3Wide,
        jacobian_a: &mut Vector3Wide,
    ) {
        // We don't need any measurement basis in a velocity motor, so the prestep data needs only the axes.
        let mut axis_a = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_axis_a, orientation_a, &mut axis_a);
        let mut axis_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_axis_b, orientation_b, &mut axis_b);
        Vector3Wide::add(&axis_a, &axis_b, jacobian_a);
        let mut length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(jacobian_a, &mut length);
        let inv_length = Vector::<f32>::splat(1.0) / length;
        let mut scaled = Vector3Wide::default();
        Vector3Wide::scale_to(jacobian_a, &inv_length, &mut scaled);
        let use_fallback = length.simd_lt(Vector::<f32>::splat(1e-10)).to_int();
        *jacobian_a = Vector3Wide::conditional_select(&use_fallback, &axis_a, &scaled);
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &TwistMotorPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(orientation_a, orientation_b, &prestep.local_axis_a, &prestep.local_axis_b, &mut jacobian_a);
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
        _inverse_dt: f32,
        prestep: &TwistMotorPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut jacobian_a = Vector3Wide::default();
        Self::compute_jacobian(orientation_a, orientation_b, &prestep.local_axis_a, &prestep.local_axis_b, &mut jacobian_a);

        let mut impulse_to_velocity_a = Vector3Wide::default();
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        let mut unsoftened_inverse_effective_mass = Vector::<f32>::splat(0.0);
        TwistServoFunctions::compute_effective_mass_contributions(
            &inertia_a.inverse_inertia_tensor,
            &inertia_b.inverse_inertia_tensor,
            &jacobian_a,
            &mut impulse_to_velocity_a,
            &mut negated_impulse_to_velocity_b,
            &mut unsoftened_inverse_effective_mass,
        );

        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        MotorSettingsWide::compute_softness(
            &prestep.settings,
            dt,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
            &mut maximum_impulse,
        );
        let effective_mass = effective_mass_cfm_scale / unsoftened_inverse_effective_mass;
        let mut velocity_to_impulse_a = Vector3Wide::default();
        Vector3Wide::scale_to(&jacobian_a, &effective_mass, &mut velocity_to_impulse_a);

        let bias_impulse = prestep.target_velocity * effective_mass;

        let mut net_velocity = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut net_velocity);
        let mut csi_velocity_component = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&net_velocity, &velocity_to_impulse_a, &mut csi_velocity_component);
        // csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - csiVelocityComponent
        let csi = bias_impulse - *accumulated_impulses * softness_impulse_scale - csi_velocity_component;
        let previous_accumulated_impulse = *accumulated_impulses;
        *accumulated_impulses = (*accumulated_impulses + csi).simd_max(-maximum_impulse).simd_min(maximum_impulse);
        let csi = *accumulated_impulses - previous_accumulated_impulse;

        TwistServoFunctions::apply_impulse(&mut wsv_a.angular, &mut wsv_b.angular, &impulse_to_velocity_a, &negated_impulse_to_velocity_b, &csi);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut TwistMotorPrestepData,
    ) {
    }
}

pub struct TwistMotorTypeProcessor;

impl TwistMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 28;
}
