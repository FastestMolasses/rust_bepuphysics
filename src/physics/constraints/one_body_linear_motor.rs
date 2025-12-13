// Translated from BepuPhysics/Constraints/OneBodyLinearMotor.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::one_body_linear_servo::OneBodyLinearServoFunctions;
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains a point on a body to have a target linear velocity.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct OneBodyLinearMotor {
    /// Offset to the attachment point in the local space of the body.
    pub local_offset: Vec3,
    /// Target velocity of the attachment point.
    pub target_velocity: Vec3,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl OneBodyLinearMotor {
    pub const CONSTRAINT_TYPE_ID: i32 = OneBodyLinearMotorTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut OneBodyLinearMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "OneBodyLinearMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset, &mut target.local_offset);
        Vector3Wide::write_first(self.target_velocity, &mut target.target_velocity);
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &OneBodyLinearMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut OneBodyLinearMotor,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset, &mut description.local_offset);
        Vector3Wide::read_first(&source.target_velocity, &mut description.target_velocity);
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct OneBodyLinearMotorPrestepData {
    pub local_offset: Vector3Wide,
    pub target_velocity: Vector3Wide,
    pub settings: MotorSettingsWide,
}

pub struct OneBodyLinearMotorFunctions;

impl OneBodyLinearMotorFunctions {
    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &OneBodyLinearMotorPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset, orientation_a, &mut offset);
        OneBodyLinearServoFunctions::apply_impulse(&offset, inertia_a, wsv_a, accumulated_impulses);
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &OneBodyLinearMotorPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset, orientation_a, &mut offset);

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

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
        let cross_term = Vector3Wide::cross_new(&wsv_a.angular, &offset);
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&prestep.target_velocity, &cross_term, &mut csv);
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

        ServoSettingsWide::clamp_impulse_3d(
            &maximum_impulse,
            accumulated_impulses,
            &mut csi,
        );
        OneBodyLinearServoFunctions::apply_impulse(&offset, inertia_a, wsv_a, &csi);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _prestep_data: &mut OneBodyLinearMotorPrestepData,
    ) {
    }
}

pub struct OneBodyLinearMotorTypeProcessor;

impl OneBodyLinearMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 45;
}
