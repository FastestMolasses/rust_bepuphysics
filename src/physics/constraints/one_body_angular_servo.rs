// Translated from BepuPhysics/Constraints/OneBodyAngularServo.cs

use glam::Quat;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains a single body to a target orientation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct OneBodyAngularServo {
    /// Target orientation of the constraint.
    pub target_orientation: Quat,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
}

impl OneBodyAngularServo {
    pub const CONSTRAINT_TYPE_ID: i32 = OneBodyAngularServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut OneBodyAngularServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_quat(
                self.target_orientation,
                "OneBodyAngularServo",
                "target_orientation",
            );
            ConstraintChecker::assert_valid_servo_settings(
                &self.servo_settings,
                "OneBodyAngularServo",
            );
        }
        QuaternionWide::write_slot(self.target_orientation, inner_index, &mut prestep_data.target_orientation);
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.spring_settings.angular_frequency, inner_index) = self.spring_settings.angular_frequency;
            *GatherScatter::get_mut(&mut prestep_data.spring_settings.twice_damping_ratio, inner_index) = self.spring_settings.twice_damping_ratio;
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.maximum_speed, inner_index) = self.servo_settings.maximum_speed;
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.base_speed, inner_index) = self.servo_settings.base_speed;
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.maximum_force, inner_index) = self.servo_settings.maximum_force;
        }
    }

    pub fn build_description(
        prestep_data: &OneBodyAngularServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut OneBodyAngularServo,
    ) {
        QuaternionWide::read_slot(
            &prestep_data.target_orientation,
            inner_index,
            &mut description.target_orientation,
        );
        unsafe {
            description.spring_settings.angular_frequency = *GatherScatter::get(&prestep_data.spring_settings.angular_frequency, inner_index);
            description.spring_settings.twice_damping_ratio = *GatherScatter::get(&prestep_data.spring_settings.twice_damping_ratio, inner_index);
            description.servo_settings.maximum_speed = *GatherScatter::get(&prestep_data.servo_settings.maximum_speed, inner_index);
            description.servo_settings.base_speed = *GatherScatter::get(&prestep_data.servo_settings.base_speed, inner_index);
            description.servo_settings.maximum_force = *GatherScatter::get(&prestep_data.servo_settings.maximum_force, inner_index);
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct OneBodyAngularServoPrestepData {
    pub target_orientation: QuaternionWide,
    pub spring_settings: SpringSettingsWide,
    pub servo_settings: ServoSettingsWide,
}

pub struct OneBodyAngularServoFunctions;

impl OneBodyAngularServoFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        inverse_inertia: &Symmetric3x3Wide,
        csi: &Vector3Wide,
        angular_velocity: &mut Vector3Wide,
    ) {
        let mut velocity_change = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(csi, inverse_inertia, &mut velocity_change);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity, &velocity_change, &mut tmp);
        *angular_velocity = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _prestep: &OneBodyAngularServoPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        Self::apply_impulse(
            &inertia_a.inverse_inertia_tensor,
            accumulated_impulses,
            &mut wsv_a.angular,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &OneBodyAngularServoPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        // Jacobians are just the identity matrix.
        let inverse_orientation = QuaternionWide::conjugate(orientation_a);
        let mut error_rotation = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            &inverse_orientation,
            &prestep.target_orientation,
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
        let mut effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::invert(&inertia_a.inverse_inertia_tensor, &mut effective_mass);

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

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&clamped_bias_velocity, &wsv_a.angular, &mut csv);
        let mut csi = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&csv, &effective_mass, &mut csi);
        let scaled_csi = csi * effective_mass_cfm_scale;
        let scaled_accumulated = *accumulated_impulses * softness_impulse_scale;
        Vector3Wide::subtract(&scaled_csi, &scaled_accumulated, &mut csi);

        ServoSettingsWide::clamp_impulse_3d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(&inertia_a.inverse_inertia_tensor, &csi, &mut wsv_a.angular);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _prestep_data: &mut OneBodyAngularServoPrestepData,
    ) {
    }
}

pub struct OneBodyAngularServoTypeProcessor;

impl OneBodyAngularServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 42;
}
