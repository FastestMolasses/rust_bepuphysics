use crate::physics::constraints::linear_axis_servo::LinearAxisServoFunctions;
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::physics::constraints::spring_settings::SpringSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;
use glam::Vec3;

pub const BATCH_TYPE_ID: i32 = 39;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisMotor {
    pub local_offset_a: Vec3,
    pub local_offset_b: Vec3,
    pub local_axis: Vec3,
    pub target_velocity: f32,
    pub settings: MotorSettings,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisMotorPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_plane_normal: Vector3Wide,
    pub target_velocity: Vector<f32>,
    pub settings: MotorSettingsWide,
}

impl LinearAxisMotorPrestepData {
    #[inline(always)]
    pub fn build_description(&self, description: &mut LinearAxisMotor, _bundle_index: usize) {
        Vector3Wide::read_first(&self.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&self.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&self.local_plane_normal, &mut description.local_axis);
        description.target_velocity = unsafe { *GatherScatter::get_first(&self.target_velocity) };
        MotorSettingsWide::read_first(&self.settings, &mut description.settings);
    }
}

pub struct LinearAxisMotorFunctions;

impl LinearAxisMotorFunctions {
    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &crate::physics::body_properties::BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &crate::physics::body_properties::BodyInertiaWide,
        prestep: &LinearAxisMotorPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut crate::physics::body_properties::BodyVelocityWide,
        wsv_b: &mut crate::physics::body_properties::BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut _plane_normal_dot = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        LinearAxisServoFunctions::compute_jacobians(
            &ab, orientation_a, orientation_b,
            &prestep.local_plane_normal, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut _plane_normal_dot, &mut normal, &mut angular_ja, &mut angular_jb,
        );
        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_ja, &inertia_a.inverse_inertia_tensor, &mut angular_impulse_to_velocity_a);
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_jb, &inertia_b.inverse_inertia_tensor, &mut angular_impulse_to_velocity_b);
        LinearAxisServoFunctions::apply_impulse(
            &normal, &angular_impulse_to_velocity_a, &angular_impulse_to_velocity_b,
            inertia_a, inertia_b, accumulated_impulses, wsv_a, wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &crate::physics::body_properties::BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &crate::physics::body_properties::BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &LinearAxisMotorPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut crate::physics::body_properties::BodyVelocityWide,
        wsv_b: &mut crate::physics::body_properties::BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut _plane_normal_dot = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        LinearAxisServoFunctions::compute_jacobians(
            &ab, orientation_a, orientation_b,
            &prestep.local_plane_normal, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut _plane_normal_dot, &mut normal, &mut angular_ja, &mut angular_jb,
        );
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        MotorSettingsWide::compute_softness(
            &prestep.settings, dt,
            &mut effective_mass_cfm_scale, &mut softness_impulse_scale, &mut maximum_impulse,
        );
        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        let mut effective_mass = Vector::<f32>::splat(0.0);
        LinearAxisServoFunctions::compute_effective_mass(
            &angular_ja, &angular_jb, inertia_a, inertia_b,
            &effective_mass_cfm_scale,
            &mut angular_impulse_to_velocity_a, &mut angular_impulse_to_velocity_b, &mut effective_mass,
        );

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let mut linear_diff = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear, &mut linear_diff);
        let csv = Vector3Wide::dot_val(&linear_diff, &normal)
            + Vector3Wide::dot_val(&wsv_a.angular, &angular_ja)
            + Vector3Wide::dot_val(&wsv_b.angular, &angular_jb);

        let mut csi = effective_mass * (-prestep.target_velocity - csv)
            - *accumulated_impulses * softness_impulse_scale;

        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        LinearAxisServoFunctions::apply_impulse(
            &normal, &angular_impulse_to_velocity_a, &angular_impulse_to_velocity_b,
            inertia_a, inertia_b, &csi, wsv_a, wsv_b,
        );
    }
}
