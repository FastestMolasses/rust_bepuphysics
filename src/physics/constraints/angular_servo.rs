// Translated from BepuPhysics/Constraints/AngularServo.cs

use glam::Quat;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains two bodies to have a target relative rotation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AngularServo {
    /// The target relative rotation from body A to body B in body A's local space.
    /// The constraint tries to maintain OrientationB = TargetRelativeRotationLocalA * OrientationA.
    pub target_relative_rotation_local_a: Quat,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
}

impl AngularServo {
    pub const CONSTRAINT_TYPE_ID: i32 = AngularServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut AngularServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_quat(
                self.target_relative_rotation_local_a,
                "AngularServo",
                "target_relative_rotation_local_a",
            );
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "AngularServo");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        QuaternionWide::write_first(
            self.target_relative_rotation_local_a,
            &mut target.target_relative_rotation_local_a,
        );
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
    }

    pub fn build_description(
        prestep_data: &AngularServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut AngularServo,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        QuaternionWide::read_first(
            &source.target_relative_rotation_local_a,
            &mut description.target_relative_rotation_local_a,
        );
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularServoPrestepData {
    pub target_relative_rotation_local_a: QuaternionWide,
    pub spring_settings: SpringSettingsWide,
    pub servo_settings: ServoSettingsWide,
}

pub struct AngularServoFunctions;

impl AngularServoFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
        impulse_to_velocity_a: &Symmetric3x3Wide,
        negated_impulse_to_velocity_b: &Symmetric3x3Wide,
        csi: &Vector3Wide,
    ) {
        let mut velocity_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            csi,
            impulse_to_velocity_a,
            &mut velocity_change_a,
        );
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity_a, &velocity_change_a, &mut tmp);
        *angular_velocity_a = tmp;
        let mut negated_velocity_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            csi,
            negated_impulse_to_velocity_b,
            &mut negated_velocity_change_b,
        );
        Vector3Wide::subtract(angular_velocity_b, &negated_velocity_change_b, &mut tmp);
        *angular_velocity_b = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        _prestep: &AngularServoPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        Self::apply_impulse(
            &mut wsv_a.angular,
            &mut wsv_b.angular,
            &inertia_a.inverse_inertia_tensor,
            &inertia_b.inverse_inertia_tensor,
            accumulated_impulses,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &AngularServoPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // Jacobians are just I and -I.
        let mut target_orientation_b = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            &prestep.target_relative_rotation_local_a,
            orientation_a,
            &mut target_orientation_b,
        );
        let inverse_target = QuaternionWide::conjugate(&target_orientation_b);
        let mut error_rotation = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            &inverse_target,
            _orientation_b,
            &mut error_rotation,
        );

        let mut error_axis = Vector3Wide::default();
        let mut error_length = Vector::<f32>::splat(0.0);
        QuaternionWide::get_axis_angle_from_quaternion(
            &error_rotation,
            &mut error_axis,
            &mut error_length,
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
        let mut unsoftened_inverse_effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::add(
            &inertia_a.inverse_inertia_tensor,
            &inertia_b.inverse_inertia_tensor,
            &mut unsoftened_inverse_effective_mass,
        );
        let mut unsoftened_effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::invert(
            &unsoftened_inverse_effective_mass,
            &mut unsoftened_effective_mass,
        );
        // Note effective mass is not directly scaled by CFM scale; instead scale the CSI.

        let mut clamped_bias_velocity = Vector3Wide::default();
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_3d_axis(
            &error_axis,
            &error_length,
            &position_error_to_velocity,
            &prestep.servo_settings,
            dt,
            inverse_dt,
            &mut clamped_bias_velocity,
            &mut maximum_impulse,
        );

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale
        //     - (csiaLinear + csiaAngular + csibLinear + csibAngular);
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut csv);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&clamped_bias_velocity, &csv, &mut tmp);
        csv = tmp;
        let mut csi = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&csv, &unsoftened_effective_mass, &mut csi);
        csi = csi * effective_mass_cfm_scale;
        let softness_component = Vector3Wide::scale(accumulated_impulses, &softness_impulse_scale);
        Vector3Wide::subtract(&csi, &softness_component, &mut tmp);
        csi = tmp;

        ServoSettingsWide::clamp_impulse_3d(&maximum_impulse, accumulated_impulses, &mut csi);

        Self::apply_impulse(
            &mut wsv_a.angular,
            &mut wsv_b.angular,
            &inertia_a.inverse_inertia_tensor,
            &inertia_b.inverse_inertia_tensor,
            &csi,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut AngularServoPrestepData,
    ) {
    }
}

pub struct AngularServoTypeProcessor;

impl AngularServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 29;
}
