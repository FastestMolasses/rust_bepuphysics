// Translated from BepuPhysics/Constraints/OneBodyLinearServo.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains a point on a body to a target location.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct OneBodyLinearServo {
    /// Offset to the attachment point in the local space of the body.
    pub local_offset: Vec3,
    /// Target position.
    pub target: Vec3,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
}

impl OneBodyLinearServo {
    pub const CONSTRAINT_TYPE_ID: i32 = OneBodyLinearServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut OneBodyLinearServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_servo_settings(
                &self.servo_settings,
                "OneBodyLinearServo",
            );
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset, &mut target.local_offset);
        Vector3Wide::write_first(self.target, &mut target.target);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
    }

    pub fn build_description(
        prestep_data: &OneBodyLinearServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut OneBodyLinearServo,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset, &mut description.local_offset);
        Vector3Wide::read_first(&source.target, &mut description.target);
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct OneBodyLinearServoPrestepData {
    pub local_offset: Vector3Wide,
    pub target: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
    pub servo_settings: ServoSettingsWide,
}

pub struct OneBodyLinearServoFunctions;

impl OneBodyLinearServoFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        offset: &Vector3Wide,
        inertia: &BodyInertiaWide,
        velocity_a: &mut BodyVelocityWide,
        csi: &Vector3Wide,
    ) {
        let mut wsi = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(offset, csi, &mut wsi);
        }
        let mut change = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &wsi,
            &inertia.inverse_inertia_tensor,
            &mut change,
        );
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&velocity_a.angular, &change, &mut tmp);
        velocity_a.angular = tmp;

        Vector3Wide::scale_to(csi, &inertia.inverse_mass, &mut change);
        Vector3Wide::add(&velocity_a.linear, &change, &mut tmp);
        velocity_a.linear = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &OneBodyLinearServoPrestepData,
        _accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.local_offset,
            orientation_a,
            &mut offset,
        );
        Self::apply_impulse(&offset, inertia_a, wsv_a, _accumulated_impulses);
    }

    #[inline(always)]
    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &OneBodyLinearServoPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.local_offset,
            orientation_a,
            &mut offset,
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

        // Compute the position error and bias velocities.
        let mut world_grab_point = Vector3Wide::default();
        Vector3Wide::add(&offset, position_a, &mut world_grab_point);
        let mut error = Vector3Wide::default();
        Vector3Wide::subtract(&prestep.target, &world_grab_point, &mut error);
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

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
        let cross_term = Vector3Wide::cross_new(&wsv_a.angular, &offset);
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&bias_velocity, &cross_term, &mut csv);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&csv, &wsv_a.linear, &mut tmp);
        csv = tmp;

        // The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).
        let mut inverse_effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(
            &offset,
            &inertia_a.inverse_inertia_tensor,
            &mut inverse_effective_mass,
        );

        // Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
        inverse_effective_mass.xx += inertia_a.inverse_mass;
        inverse_effective_mass.yy += inertia_a.inverse_mass;
        inverse_effective_mass.zz += inertia_a.inverse_mass;
        let mut effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::invert(&inverse_effective_mass, &mut effective_mass);
        let mut csi = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&csv, &effective_mass, &mut csi);
        let scaled_csi = csi * effective_mass_cfm_scale;
        let scaled_accumulated = *accumulated_impulses * softness_impulse_scale;
        Vector3Wide::subtract(&scaled_csi, &scaled_accumulated, &mut csi);

        // The motor has a limited maximum force, so clamp the accumulated impulse.
        ServoSettingsWide::clamp_impulse_3d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(&offset, inertia_a, wsv_a, &csi);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _prestep_data: &mut OneBodyLinearServoPrestepData,
    ) {
    }
}

pub struct OneBodyLinearServoTypeProcessor;

impl OneBodyLinearServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 44;
}
