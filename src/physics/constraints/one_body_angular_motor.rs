// Translated from BepuPhysics/Constraints/OneBodyAngularMotor.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains the angular velocity of one body to the target.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct OneBodyAngularMotor {
    /// Target angular velocity.
    pub target_velocity: Vec3,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl OneBodyAngularMotor {
    pub const CONSTRAINT_TYPE_ID: i32 = OneBodyAngularMotorTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut OneBodyAngularMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "OneBodyAngularMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.target_velocity, &mut target.target_velocity);
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &OneBodyAngularMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut OneBodyAngularMotor,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.target_velocity, &mut description.target_velocity);
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct OneBodyAngularMotorPrestepData {
    pub target_velocity: Vector3Wide,
    pub settings: MotorSettingsWide,
}

pub struct OneBodyAngularMotorFunctions;

impl OneBodyAngularMotorFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        angular_velocity: &mut Vector3Wide,
        impulse_to_velocity: &Symmetric3x3Wide,
        csi: &Vector3Wide,
    ) {
        let mut velocity_change = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(csi, impulse_to_velocity, &mut velocity_change);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity, &velocity_change, &mut tmp);
        *angular_velocity = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _prestep: &OneBodyAngularMotorPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
    ) {
        Self::apply_impulse(
            &mut wsv_a.angular,
            &inertia_a.inverse_inertia_tensor,
            accumulated_impulses,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &OneBodyAngularMotorPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
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
        let mut unsoftened_effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::invert(
            &inertia_a.inverse_inertia_tensor,
            &mut unsoftened_effective_mass,
        );

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
        let mut diff = Vector3Wide::default();
        Vector3Wide::subtract(&prestep.target_velocity, &wsv_a.angular, &mut diff);
        let mut csi = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&diff, &unsoftened_effective_mass, &mut csi);
        let scaled_csi = csi * effective_mass_cfm_scale;
        let scaled_accumulated = *accumulated_impulses * softness_impulse_scale;
        Vector3Wide::subtract(&scaled_csi, &scaled_accumulated, &mut csi);

        ServoSettingsWide::clamp_impulse_3d(&maximum_impulse, accumulated_impulses, &mut csi);

        Self::apply_impulse(&mut wsv_a.angular, &inertia_a.inverse_inertia_tensor, &csi);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _prestep_data: &mut OneBodyAngularMotorPrestepData,
    ) {
    }
}

pub struct OneBodyAngularMotorTypeProcessor;

impl OneBodyAngularMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 43;
}
