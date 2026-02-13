// Translated from BepuPhysics/Constraints/AngularAxisMotor.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains the relative angular velocity of two bodies around a local axis attached to body A to a target velocity.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AngularAxisMotor {
    /// Axis of rotation in body A's local space.
    pub local_axis_a: Vec3,
    /// Target relative angular velocity around the axis.
    pub target_velocity: f32,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl AngularAxisMotor {
    pub const CONSTRAINT_TYPE_ID: i32 = AngularAxisMotorTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut AngularAxisMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(
                self.local_axis_a,
                "AngularAxisMotor",
                "local_axis_a",
            );
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "AngularAxisMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_axis_a, &mut target.local_axis_a);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.target_velocity) = self.target_velocity;
        }
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &AngularAxisMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut AngularAxisMotor,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_axis_a, &mut description.local_axis_a);
        description.target_velocity = source.target_velocity[0];
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularAxisMotorPrestepData {
    pub local_axis_a: Vector3Wide,
    pub target_velocity: Vector<f32>,
    pub settings: MotorSettingsWide,
}

pub struct AngularAxisMotorFunctions;

impl AngularAxisMotorFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        impulse_to_velocity_a: &Vector3Wide,
        negated_impulse_to_velocity_b: &Vector3Wide,
        csi: &Vector<f32>,
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
    ) {
        // angularVelocityA += impulseToVelocityA * csi
        let delta_a = Vector3Wide::scale(impulse_to_velocity_a, csi);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity_a, &delta_a, &mut tmp);
        *angular_velocity_a = tmp;
        // angularVelocityB -= negatedImpulseToVelocityB * csi
        let delta_b = Vector3Wide::scale(negated_impulse_to_velocity_b, csi);
        Vector3Wide::subtract(angular_velocity_b, &delta_b, &mut tmp);
        *angular_velocity_b = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &AngularAxisMotorPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut axis = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_axis_a, orientation_a, &mut axis);
        let mut j_ia = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &axis,
            &inertia_a.inverse_inertia_tensor,
            &mut j_ia,
        );
        let mut j_ib = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &axis,
            &inertia_b.inverse_inertia_tensor,
            &mut j_ib,
        );
        Self::apply_impulse(
            &j_ia,
            &j_ib,
            accumulated_impulses,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
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
        prestep: &AngularAxisMotorPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut j_a = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_axis_a, orientation_a, &mut j_a);
        let mut j_ia = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &j_a,
            &inertia_a.inverse_inertia_tensor,
            &mut j_ia,
        );
        let mut contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&j_a, &j_ia, &mut contribution_a);
        let mut j_ib = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &j_a,
            &inertia_b.inverse_inertia_tensor,
            &mut j_ib,
        );
        let mut contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&j_a, &j_ib, &mut contribution_b);

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

        // csi = projection.BiasImpulse - accumulatedImpulse * softnessImpulseScale
        //     - (csiaLinear + csiaAngular + csibLinear + csibAngular);
        let dot_b = Vector3Wide::dot_val(&wsv_b.angular, &j_a);
        let dot_a = Vector3Wide::dot_val(&wsv_a.angular, &j_a);
        let mut csi = (prestep.target_velocity + dot_b - dot_a) * effective_mass_cfm_scale
            / (contribution_a + contribution_b)
            - *accumulated_impulses * softness_impulse_scale;
        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(&j_ia, &j_ib, &csi, &mut wsv_a.angular, &mut wsv_b.angular);
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut AngularAxisMotorPrestepData,
    ) {
    }
}

pub struct AngularAxisMotorTypeProcessor;

impl AngularAxisMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 41;
}
