use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;

/// Constrains body B's angular velocity around an axis anchored to body A to equal body A's velocity
/// around that axis with a scaling factor applied.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AngularAxisGearMotor {
    /// Axis of rotation in body A's local space.
    pub local_axis_a: Vec3,
    /// Scale to apply to body A's velocity around the axis to get body B's target velocity.
    /// A VelocityScale of 2 means body A could have a velocity of 3 while body B has a velocity of 6.
    pub velocity_scale: f32,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl AngularAxisGearMotor {
    pub fn apply_description(
        &self,
        prestep_data: &mut AngularAxisGearMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(
                self.local_axis_a,
                "AngularAxisGearMotor",
                "local_axis_a",
            );
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "AngularAxisGearMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_axis_a, &mut target.local_axis_a);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.velocity_scale) = self.velocity_scale;
        }
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &AngularAxisGearMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_axis_a, &mut description.local_axis_a);
        description.velocity_scale = unsafe { *GatherScatter::get_first(&source.velocity_scale) };
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularAxisGearMotorPrestepData {
    pub local_axis_a: Vector3Wide,
    pub velocity_scale: Vector<f32>,
    pub settings: MotorSettingsWide,
}

pub struct AngularAxisGearMotorFunctions;

impl AngularAxisGearMotorFunctions {
    #[inline(always)]
    fn apply_impulse(
        impulse_to_velocity_a: &Vector3Wide,
        negated_impulse_to_velocity_b: &Vector3Wide,
        csi: &Vector<f32>,
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
    ) {
        let change_a = *impulse_to_velocity_a * *csi;
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity_a, &change_a, &mut tmp);
        *angular_velocity_a = tmp;
        let change_b = *negated_impulse_to_velocity_b * *csi;
        Vector3Wide::subtract(angular_velocity_b, &change_b, &mut tmp);
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
        prestep: &AngularAxisGearMotorPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut axis = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_axis_a, orientation_a, &mut axis);
        let mut j_a = Vector3Wide::default();
        Vector3Wide::scale_to(&axis, &prestep.velocity_scale, &mut j_a);
        let mut impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &j_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &axis,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
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
        prestep: &AngularAxisGearMotorPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // dot(wa, axis) * velocityScale - dot(wb, axis) = 0, so jacobianA is axis * velocityScale
        let mut axis = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_axis_a, orientation_a, &mut axis);
        let mut j_a = Vector3Wide::default();
        Vector3Wide::scale_to(&axis, &prestep.velocity_scale, &mut j_a);
        let mut impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &j_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&j_a, &impulse_to_velocity_a, &mut contribution_a);
        let mut negated_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &axis,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        let mut contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&axis, &negated_impulse_to_velocity_b, &mut contribution_b);
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
        let effective_mass = effective_mass_cfm_scale / (contribution_a + contribution_b);

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csv
        let mut unscaled_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.angular, &j_a, &mut unscaled_csv_a);
        let mut negated_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.angular, &axis, &mut negated_csv_b);
        let mut csi = (negated_csv_b - unscaled_csv_a) * effective_mass
            - *accumulated_impulses * softness_impulse_scale;
        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            accumulated_impulses,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut AngularAxisGearMotorPrestepData,
    ) {
    }
}

pub struct AngularAxisGearMotorTypeProcessor;

impl AngularAxisGearMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 54;
}
