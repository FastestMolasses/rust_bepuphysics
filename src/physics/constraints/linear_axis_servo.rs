// Translated from BepuPhysics/Constraints/LinearAxisServo.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains points on two bodies to be on a plane defined in the local space of one of the bodies.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LinearAxisServo {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Direction of the plane normal in the local space of body A.
    pub local_plane_normal: Vec3,
    /// Target offset from A's plane anchor to B's anchor along the plane normal.
    pub target_offset: f32,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl LinearAxisServo {
    pub const CONSTRAINT_TYPE_ID: i32 = LinearAxisServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut LinearAxisServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            ConstraintChecker::assert_unit_length_vec3(self.local_plane_normal, "LinearAxisServo", "local_plane_normal");
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "LinearAxisServo");
        }

        let target = unsafe {
            GatherScatter::get_offset_instance_mut(prestep_data, inner_index)
        };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        Vector3Wide::write_first(self.local_plane_normal, &mut target.local_plane_normal);
        unsafe { *GatherScatter::get_first_mut(&mut target.target_offset) = self.target_offset; }
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &LinearAxisServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut LinearAxisServo,
    ) {
        let source = unsafe {
            GatherScatter::get_offset_instance(prestep_data, inner_index)
        };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&source.local_plane_normal, &mut description.local_plane_normal);
        description.target_offset = source.target_offset[0];
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisServoPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_plane_normal: Vector3Wide,
    pub target_offset: Vector<f32>,
    pub servo_settings: ServoSettingsWide,
    pub spring_settings: SpringSettingsWide,
}

/// Provides shared functions for LinearAxisServo and LinearAxisLimit.
pub struct LinearAxisServoFunctions;

impl LinearAxisServoFunctions {
    /// Computes jacobians for the linear axis constraint.
    #[inline(always)]
    pub fn compute_jacobians(
        ab: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_plane_normal_a: &Vector3Wide,
        local_offset_a: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        plane_normal_dot: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        angular_ja: &mut Vector3Wide,
        angular_jb: &mut Vector3Wide,
    ) {
        // Linear jacobians are just normal and -normal. Angular jacobians are offsetA x normal and offsetB x normal.
        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        Matrix3x3Wide::transform_without_overlap(local_plane_normal_a, &orientation_matrix_a, normal);
        let mut anchor_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(local_offset_a, &orientation_matrix_a, &mut anchor_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_offset_b, orientation_b, &mut offset_b);
        // Note that the angular jacobian for A uses the offset from A to the attachment point on B.
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(ab, &offset_b, &mut anchor_b);
        let mut diff = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &anchor_a, &mut diff);
        Vector3Wide::dot(&diff, normal, plane_normal_dot);
        // offsetFromAToClosestPointOnPlaneToB = anchorB - planeNormalDot * normal
        let scaled_normal = *normal * *plane_normal_dot;
        let mut offset_from_a = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &scaled_normal, &mut offset_from_a);
        unsafe {
            Vector3Wide::cross_without_overlap(&offset_from_a, normal, angular_ja);
            Vector3Wide::cross_without_overlap(normal, &offset_b, angular_jb);
        }
    }

    /// Computes effective mass for the linear axis constraint.
    #[inline(always)]
    pub fn compute_effective_mass(
        angular_ja: &Vector3Wide,
        angular_jb: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        effective_mass_cfm_scale: &Vector<f32>,
        angular_impulse_to_velocity_a: &mut Vector3Wide,
        angular_impulse_to_velocity_b: &mut Vector3Wide,
        effective_mass: &mut Vector<f32>,
    ) {
        Symmetric3x3Wide::transform_without_overlap(angular_ja, &inertia_a.inverse_inertia_tensor, angular_impulse_to_velocity_a);
        Symmetric3x3Wide::transform_without_overlap(angular_jb, &inertia_b.inverse_inertia_tensor, angular_impulse_to_velocity_b);
        let mut angular_contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(angular_ja, angular_impulse_to_velocity_a, &mut angular_contribution_a);
        let mut angular_contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(angular_jb, angular_impulse_to_velocity_b, &mut angular_contribution_b);
        *effective_mass = *effective_mass_cfm_scale / (inertia_a.inverse_mass + inertia_b.inverse_mass + angular_contribution_a + angular_contribution_b);
    }

    /// Applies an impulse for the linear axis constraint.
    #[inline(always)]
    pub fn apply_impulse(
        linear_ja: &Vector3Wide,
        angular_impulse_to_velocity_a: &Vector3Wide,
        angular_impulse_to_velocity_b: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        csi: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // velocityA.Linear += linearJA * (csi * inertiaA.InverseMass);
        let linear_change_a = *linear_ja * (*csi * inertia_a.inverse_mass);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&linear_change_a, &wsv_a.linear, &mut tmp);
        wsv_a.linear = tmp;

        // velocityB.Linear -= linearJA * (csi * inertiaB.InverseMass);
        let negated_linear_change_b = *linear_ja * (*csi * inertia_b.inverse_mass);
        Vector3Wide::subtract(&wsv_b.linear, &negated_linear_change_b, &mut tmp);
        wsv_b.linear = tmp;

        // velocityA.Angular += angularImpulseToVelocityA * csi;
        let angular_change_a = *angular_impulse_to_velocity_a * *csi;
        Vector3Wide::add(&angular_change_a, &wsv_a.angular, &mut tmp);
        wsv_a.angular = tmp;

        // velocityB.Angular += angularImpulseToVelocityB * csi;
        let angular_change_b = *angular_impulse_to_velocity_b * *csi;
        Vector3Wide::add(&angular_change_b, &wsv_b.angular, &mut tmp);
        wsv_b.angular = tmp;
    }

    /// Warm starts the constraint.
    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &LinearAxisServoPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut _plane_normal_dot = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &ab, orientation_a, orientation_b,
            &prestep.local_plane_normal, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut _plane_normal_dot, &mut normal, &mut angular_ja, &mut angular_jb,
        );
        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_ja, &inertia_a.inverse_inertia_tensor, &mut angular_impulse_to_velocity_a);
        Symmetric3x3Wide::transform_without_overlap(&angular_jb, &inertia_b.inverse_inertia_tensor, &mut angular_impulse_to_velocity_b);
        Self::apply_impulse(&normal, &angular_impulse_to_velocity_a, &angular_impulse_to_velocity_b, inertia_a, inertia_b, accumulated_impulses, wsv_a, wsv_b);
    }

    /// Solves the constraint.
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
        prestep: &LinearAxisServoPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut plane_normal_dot = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &ab, orientation_a, orientation_b,
            &prestep.local_plane_normal, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut plane_normal_dot, &mut normal, &mut angular_ja, &mut angular_jb,
        );

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(&prestep.spring_settings, dt, &mut position_error_to_velocity, &mut effective_mass_cfm_scale, &mut softness_impulse_scale);

        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        let mut effective_mass = Vector::<f32>::splat(0.0);
        Self::compute_effective_mass(
            &angular_ja, &angular_jb, inertia_a, inertia_b, &effective_mass_cfm_scale,
            &mut angular_impulse_to_velocity_a, &mut angular_impulse_to_velocity_b, &mut effective_mass,
        );

        // Compute the position error and bias velocities.
        let error = plane_normal_dot - prestep.target_offset;
        let mut bias_velocity = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_1d(
            &error, &position_error_to_velocity, &prestep.servo_settings, dt, inverse_dt,
            &mut bias_velocity, &mut maximum_impulse,
        );

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let mut linear_diff = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear, &mut linear_diff);
        let csv_linear = Vector3Wide::dot_val(&linear_diff, &normal);
        let csv_angular_a = Vector3Wide::dot_val(&wsv_a.angular, &angular_ja);
        let csv_angular_b = Vector3Wide::dot_val(&wsv_b.angular, &angular_jb);
        let csv = csv_linear + csv_angular_a + csv_angular_b;

        let mut csi = effective_mass * (bias_velocity - csv) - *accumulated_impulses * softness_impulse_scale;

        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(&normal, &angular_impulse_to_velocity_a, &angular_impulse_to_velocity_b, inertia_a, inertia_b, &csi, wsv_a, wsv_b);
    }
}

pub struct LinearAxisServoTypeProcessor;

impl LinearAxisServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 38;
}
