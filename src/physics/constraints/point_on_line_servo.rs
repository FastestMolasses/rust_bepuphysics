// Translated from BepuPhysics/Constraints/PointOnLineServo.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::helpers::Helpers;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains a point on body B to be on a line attached to body A.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PointOnLineServo {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Direction of the line in the local space of body A.
    pub local_direction: Vec3,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl PointOnLineServo {
    pub const CONSTRAINT_TYPE_ID: i32 = PointOnLineServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut PointOnLineServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            ConstraintChecker::assert_unit_length_vec3(self.local_direction, "PointOnLineServo", "local_direction");
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "PointOnLineServo");
        }

        let target = unsafe {
            GatherScatter::get_offset_instance_mut(prestep_data, inner_index)
        };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        Vector3Wide::write_first(self.local_direction, &mut target.local_direction);
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &PointOnLineServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut PointOnLineServo,
    ) {
        let source = unsafe {
            GatherScatter::get_offset_instance(prestep_data, inner_index)
        };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&source.local_direction, &mut description.local_direction);
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct PointOnLineServoPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_direction: Vector3Wide,
    pub servo_settings: ServoSettingsWide,
    pub spring_settings: SpringSettingsWide,
}

pub struct PointOnLineServoFunctions;

impl PointOnLineServoFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
        linear_jacobian: &Matrix2x3Wide,
        angular_jacobian_a: &Matrix2x3Wide,
        angular_jacobian_b: &Matrix2x3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        csi: &Vector2Wide,
    ) {
        let mut linear_impulse_a = Vector3Wide::default();
        Matrix2x3Wide::transform(csi, linear_jacobian, &mut linear_impulse_a);
        let mut angular_impulse_a = Vector3Wide::default();
        Matrix2x3Wide::transform(csi, angular_jacobian_a, &mut angular_impulse_a);
        let mut angular_impulse_b = Vector3Wide::default();
        Matrix2x3Wide::transform(csi, angular_jacobian_b, &mut angular_impulse_b);
        let mut angular_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_a, &inertia_a.inverse_inertia_tensor, &mut angular_change_a);
        let mut angular_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_b, &inertia_b.inverse_inertia_tensor, &mut angular_change_b);
        let linear_change_a = Vector3Wide::scale(&linear_impulse_a, &inertia_a.inverse_mass);
        let negated_linear_change_b = Vector3Wide::scale(&linear_impulse_a, &inertia_b.inverse_mass);

        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&linear_change_a, &velocity_a.linear, &mut tmp);
        velocity_a.linear = tmp;
        Vector3Wide::add(&angular_change_a, &velocity_a.angular, &mut tmp);
        velocity_a.angular = tmp;
        Vector3Wide::subtract(&velocity_b.linear, &negated_linear_change_b, &mut tmp);
        velocity_b.linear = tmp;
        Vector3Wide::add(&angular_change_b, &velocity_b.angular, &mut tmp);
        velocity_b.angular = tmp;
    }

    #[inline(always)]
    pub fn compute_jacobians(
        ab: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_direction: &Vector3Wide,
        local_offset_a: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        anchor_offset: &mut Vector3Wide,
        linear_jacobian: &mut Matrix2x3Wide,
        angular_ja: &mut Matrix2x3Wide,
        angular_jb: &mut Matrix2x3Wide,
    ) {
        let mut local_tangent_x = Vector3Wide::default();
        let mut local_tangent_y = Vector3Wide::default();
        Helpers::build_orthonormal_basis(local_direction, &mut local_tangent_x, &mut local_tangent_y);
        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        let mut anchor_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(local_offset_a, &orientation_matrix_a, &mut anchor_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_offset_b, orientation_b, &mut offset_b);

        // Find offsetA by computing the closest point on the line to anchorB.
        let mut direction = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(local_direction, &orientation_matrix_a, &mut direction);
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(&offset_b, ab, &mut anchor_b);
        Vector3Wide::subtract(&anchor_b, &anchor_a, anchor_offset);
        let mut d = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(anchor_offset, &direction, &mut d);
        let line_start_to_closest = Vector3Wide::scale(&direction, &d);
        let mut offset_a = Vector3Wide::default();
        Vector3Wide::add(&line_start_to_closest, &anchor_a, &mut offset_a);

        Matrix3x3Wide::transform_without_overlap(&local_tangent_x, &orientation_matrix_a, &mut linear_jacobian.x);
        Matrix3x3Wide::transform_without_overlap(&local_tangent_y, &orientation_matrix_a, &mut linear_jacobian.y);

        unsafe {
            Vector3Wide::cross_without_overlap(&offset_a, &linear_jacobian.x, &mut angular_ja.x);
            Vector3Wide::cross_without_overlap(&offset_a, &linear_jacobian.y, &mut angular_ja.y);
            Vector3Wide::cross_without_overlap(&linear_jacobian.x, &offset_b, &mut angular_jb.x);
            Vector3Wide::cross_without_overlap(&linear_jacobian.y, &offset_b, &mut angular_jb.y);
        }
    }

    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &PointOnLineServoPrestepData,
        accumulated_impulses: &Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut _anchor_offset = Vector3Wide::default();
        let mut linear_jacobian = Matrix2x3Wide::default();
        let mut angular_ja = Matrix2x3Wide::default();
        let mut angular_jb = Matrix2x3Wide::default();
        Self::compute_jacobians(
            &ab, orientation_a, orientation_b, &prestep.local_direction, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut _anchor_offset, &mut linear_jacobian, &mut angular_ja, &mut angular_jb,
        );
        Self::apply_impulse(wsv_a, wsv_b, &linear_jacobian, &angular_ja, &angular_jb, inertia_a, inertia_b, accumulated_impulses);
    }

    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &PointOnLineServoPrestepData,
        accumulated_impulses: &mut Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut anchor_offset = Vector3Wide::default();
        let mut linear_jacobian = Matrix2x3Wide::default();
        let mut angular_ja = Matrix2x3Wide::default();
        let mut angular_jb = Matrix2x3Wide::default();
        Self::compute_jacobians(
            &ab, orientation_a, orientation_b, &prestep.local_direction, &prestep.local_offset_a, &prestep.local_offset_b,
            &mut anchor_offset, &mut linear_jacobian, &mut angular_ja, &mut angular_jb,
        );

        let inverse_mass_sum = inertia_a.inverse_mass + inertia_b.inverse_mass;
        let mut linear_contribution = Symmetric2x2Wide::default();
        Symmetric2x2Wide::sandwich_scale(&linear_jacobian, &inverse_mass_sum, &mut linear_contribution);
        let mut angular_contribution_a = Symmetric2x2Wide::default();
        Symmetric3x3Wide::matrix_sandwich(&angular_ja, &inertia_a.inverse_inertia_tensor, &mut angular_contribution_a);
        let mut angular_contribution_b = Symmetric2x2Wide::default();
        Symmetric3x3Wide::matrix_sandwich(&angular_jb, &inertia_b.inverse_inertia_tensor, &mut angular_contribution_b);
        let mut inverse_effective_mass = Symmetric2x2Wide::default();
        Symmetric2x2Wide::add(&angular_contribution_a, &angular_contribution_b, &mut inverse_effective_mass);
        let mut tmp_sym = Symmetric2x2Wide::default();
        Symmetric2x2Wide::add(&inverse_effective_mass, &linear_contribution, &mut tmp_sym);
        inverse_effective_mass = tmp_sym;

        let mut effective_mass = Symmetric2x2Wide::default();
        Symmetric2x2Wide::invert_without_overlap(&inverse_effective_mass, &mut effective_mass);

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(&prestep.spring_settings, dt, &mut position_error_to_velocity, &mut effective_mass_cfm_scale, &mut softness_impulse_scale);
        let mut effective_mass_scaled = Symmetric2x2Wide::default();
        Symmetric2x2Wide::scale(&effective_mass, &effective_mass_cfm_scale, &mut effective_mass_scaled);
        effective_mass = effective_mass_scaled;

        // CSV computation
        let mut linear_csv_a = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_a.linear, &linear_jacobian, &mut linear_csv_a);
        let mut negated_linear_csv_b = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_b.linear, &linear_jacobian, &mut negated_linear_csv_b);
        let mut angular_csv_a = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_a.angular, &angular_ja, &mut angular_csv_a);
        let mut angular_csv_b = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_b.angular, &angular_jb, &mut angular_csv_b);
        let mut linear_csv = Vector2Wide::default();
        Vector2Wide::subtract(&linear_csv_a, &negated_linear_csv_b, &mut linear_csv);
        let mut angular_csv = Vector2Wide::default();
        Vector2Wide::add(&angular_csv_a, &angular_csv_b, &mut angular_csv);
        let mut csv = Vector2Wide::default();
        Vector2Wide::add(&linear_csv, &angular_csv, &mut csv);

        // Position error and bias velocity.
        let mut error = Vector2Wide::default();
        Vector3Wide::dot(&anchor_offset, &linear_jacobian.x, &mut error.x);
        Vector3Wide::dot(&anchor_offset, &linear_jacobian.y, &mut error.y);
        let mut bias_velocity = Vector2Wide::default();
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_2d(
            &error, &position_error_to_velocity, &prestep.servo_settings, dt, inverse_dt,
            &mut bias_velocity, &mut maximum_impulse,
        );

        let mut bias_minus_csv = Vector2Wide::default();
        Vector2Wide::subtract(&bias_velocity, &csv, &mut bias_minus_csv);
        let mut csi = Vector2Wide::default();
        Symmetric2x2Wide::transform_without_overlap(&bias_minus_csv, &effective_mass, &mut csi);
        let mut softness_contribution = Vector2Wide::default();
        Vector2Wide::scale(accumulated_impulses, &softness_impulse_scale, &mut softness_contribution);
        let mut tmp_v2 = Vector2Wide::default();
        Vector2Wide::subtract(&csi, &softness_contribution, &mut tmp_v2);
        csi = tmp_v2;
        ServoSettingsWide::clamp_impulse_2d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(wsv_a, wsv_b, &linear_jacobian, &angular_ja, &angular_jb, inertia_a, inertia_b, &csi);
    }
}

pub struct PointOnLineServoTypeProcessor;

impl PointOnLineServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 37;
}
