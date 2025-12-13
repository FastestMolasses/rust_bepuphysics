// Translated from BepuPhysics/Constraints/BallSocketServo.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::ball_socket_shared::BallSocketShared;
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains a point on one body to a point on another body.
/// Provides speed and force configuration that the BallSocket joint does not.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BallSocketServo {
    /// Offset from the center of body A to its attachment in A's local space.
    pub local_offset_a: Vec3,
    /// Offset from the center of body B to its attachment in B's local space.
    pub local_offset_b: Vec3,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
}

impl BallSocketServo {
    pub const CONSTRAINT_TYPE_ID: i32 = BallSocketServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut BallSocketServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_servo_settings(
                &self.servo_settings,
                "BallSocketServo",
            );
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
    }

    pub fn build_description(
        prestep_data: &BallSocketServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut BallSocketServo,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct BallSocketServoPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
    pub servo_settings: ServoSettingsWide,
}

pub struct BallSocketServoFunctions;

impl BallSocketServoFunctions {
    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &BallSocketServoPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut offset_a = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset_a, orientation_a, &mut offset_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset_b, orientation_b, &mut offset_b);
        BallSocketShared::apply_impulse(
            wsv_a,
            wsv_b,
            &offset_a,
            &offset_b,
            inertia_a,
            inertia_b,
            accumulated_impulses,
        );
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
        prestep: &BallSocketServoPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut offset_a = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset_a, orientation_a, &mut offset_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset_b, orientation_b, &mut offset_b);

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

        let mut effective_mass = crate::utilities::symmetric3x3_wide::Symmetric3x3Wide::default();
        BallSocketShared::compute_effective_mass(
            inertia_a,
            inertia_b,
            &offset_a,
            &offset_b,
            &effective_mass_cfm_scale,
            &mut effective_mass,
        );

        // Compute the position error and bias velocities.
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(&ab, &offset_b, &mut anchor_b);
        let mut error = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &offset_a, &mut error);
        let mut bias_velocity = Vector3Wide::default();
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_3d(
            &error,
            &position_error_to_velocity,
            &prestep.servo_settings,
            dt,
            inverse_dt,
            &mut bias_velocity,
            &mut maximum_impulse,
        );

        BallSocketShared::solve_with_max_impulse(
            wsv_a,
            wsv_b,
            &offset_a,
            &offset_b,
            &bias_velocity,
            &effective_mass,
            &softness_impulse_scale,
            &maximum_impulse,
            accumulated_impulses,
            inertia_a,
            inertia_b,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut BallSocketServoPrestepData,
    ) {
    }
}

/// Handles the solve iterations of a bunch of ball socket servo constraints.
pub struct BallSocketServoTypeProcessor;

impl BallSocketServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 53;
}
