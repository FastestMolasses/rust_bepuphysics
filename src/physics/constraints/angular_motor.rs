// Translated from BepuPhysics/Constraints/AngularMotor.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::angular_servo::AngularServoFunctions;
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains the relative angular velocity between two bodies to a target.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AngularMotor {
    /// Target relative angular velocity between A and B, stored in A's local space.
    /// Target world space angular velocity of B is AngularVelocityA + TargetVelocityLocalA * OrientationA.
    pub target_velocity_local_a: Vec3,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl AngularMotor {
    pub const CONSTRAINT_TYPE_ID: i32 = AngularMotorTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut AngularMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "AngularMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(
            self.target_velocity_local_a,
            &mut target.target_velocity_local_a,
        );
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &AngularMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut AngularMotor,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(
            &source.target_velocity_local_a,
            &mut description.target_velocity_local_a,
        );
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularMotorPrestepData {
    pub target_velocity_local_a: Vector3Wide,
    pub settings: MotorSettingsWide,
}

pub struct AngularMotorFunctions;

impl AngularMotorFunctions {
    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        _prestep: &AngularMotorPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        AngularServoFunctions::apply_impulse(
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
        _inverse_dt: f32,
        prestep: &AngularMotorPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // Jacobians are just the identity matrix.
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
        // Note that we don't scale the effective mass directly; instead scale CSI.

        let mut bias_velocity = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.target_velocity_local_a,
            orientation_a,
            &mut bias_velocity,
        );

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale
        //     - (csiaLinear + csiaAngular + csibLinear + csibAngular);
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut csv);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&bias_velocity, &csv, &mut tmp);
        csv = tmp;
        let mut csi = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&csv, &unsoftened_effective_mass, &mut csi);
        csi = csi * effective_mass_cfm_scale;
        let softness_component = Vector3Wide::scale(accumulated_impulses, &softness_impulse_scale);
        Vector3Wide::subtract(&csi, &softness_component, &mut tmp);
        csi = tmp;

        ServoSettingsWide::clamp_impulse_3d(&maximum_impulse, accumulated_impulses, &mut csi);
        AngularServoFunctions::apply_impulse(
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
        _prestep_data: &mut AngularMotorPrestepData,
    ) {
    }
}

pub struct AngularMotorTypeProcessor;

impl AngularMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 30;
}
