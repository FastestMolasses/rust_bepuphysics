// Translated from BepuPhysics/Constraints/LinearAxisLimit.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::inequality_helpers::InequalityHelpers;
use crate::physics::constraints::linear_axis_servo::LinearAxisServoFunctions;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains points on two bodies to a range of offsets from each other along a direction anchored to body A.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LinearAxisLimit {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Direction of the motorized axis in the local space of body A.
    pub local_axis: Vec3,
    /// Minimum offset along the world axis between A and B's anchor points.
    pub minimum_offset: f32,
    /// Maximum offset along the world axis between A and B's anchor points.
    pub maximum_offset: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl LinearAxisLimit {
    pub const CONSTRAINT_TYPE_ID: i32 = LinearAxisLimitTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut LinearAxisLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                self.maximum_offset >= self.minimum_offset,
                "LinearAxisLimit.maximum_offset must be >= minimum_offset"
            );
            ConstraintChecker::assert_unit_length_vec3(
                self.local_axis,
                "LinearAxisLimit",
                "local_axis",
            );
            ConstraintChecker::assert_valid_spring_settings(
                &self.spring_settings,
                "LinearAxisLimit",
            );
        }

        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        Vector3Wide::write_first(self.local_axis, &mut target.local_plane_normal);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.minimum_offset) = self.minimum_offset;
            *GatherScatter::get_first_mut(&mut target.maximum_offset) = self.maximum_offset;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &LinearAxisLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut LinearAxisLimit,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&source.local_plane_normal, &mut description.local_axis);
        description.minimum_offset = source.minimum_offset[0];
        description.maximum_offset = source.maximum_offset[0];
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisLimitPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_plane_normal: Vector3Wide,
    pub minimum_offset: Vector<f32>,
    pub maximum_offset: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct LinearAxisLimitFunctions;

impl LinearAxisLimitFunctions {
    #[inline(always)]
    fn compute_jacobians(
        ab: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_plane_normal: &Vector3Wide,
        local_offset_a: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        minimum_offset: &Vector<f32>,
        maximum_offset: &Vector<f32>,
        error: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        angular_ja: &mut Vector3Wide,
        angular_jb: &mut Vector3Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        Matrix3x3Wide::transform_without_overlap(local_plane_normal, &orientation_matrix_a, normal);
        let mut anchor_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(
            local_offset_a,
            &orientation_matrix_a,
            &mut anchor_a,
        );
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(local_offset_b, orientation_b, &mut offset_b);
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(ab, &offset_b, &mut anchor_b);
        let mut diff = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &anchor_a, &mut diff);
        let mut plane_normal_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&diff, normal, &mut plane_normal_dot);

        // The limit chooses the normal's sign depending on which limit is closer.
        let minimum_error = *minimum_offset - plane_normal_dot;
        let maximum_error = plane_normal_dot - *maximum_offset;
        let use_min = minimum_error.abs().simd_lt(maximum_error.abs());
        *error = use_min.select(minimum_error, maximum_error);
        normal.x = use_min.select(-normal.x, normal.x);
        normal.y = use_min.select(-normal.y, normal.y);
        normal.z = use_min.select(-normal.z, normal.z);

        // offsetFromAToClosestPointOnPlaneToB = anchorB - planeNormalDot * normal
        let scaled_normal = *normal * plane_normal_dot;
        let mut offset_from_a = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &scaled_normal, &mut offset_from_a);
        unsafe {
            Vector3Wide::cross_without_overlap(&offset_from_a, normal, angular_ja);
            Vector3Wide::cross_without_overlap(normal, &offset_b, angular_jb);
        }
    }

    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &LinearAxisLimitPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut _error = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &ab,
            orientation_a,
            orientation_b,
            &prestep.local_plane_normal,
            &prestep.local_offset_a,
            &prestep.local_offset_b,
            &prestep.minimum_offset,
            &prestep.maximum_offset,
            &mut _error,
            &mut normal,
            &mut angular_ja,
            &mut angular_jb,
        );
        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &angular_ja,
            &inertia_a.inverse_inertia_tensor,
            &mut angular_impulse_to_velocity_a,
        );
        Symmetric3x3Wide::transform_without_overlap(
            &angular_jb,
            &inertia_b.inverse_inertia_tensor,
            &mut angular_impulse_to_velocity_b,
        );
        LinearAxisServoFunctions::apply_impulse(
            &normal,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
            inertia_a,
            inertia_b,
            accumulated_impulses,
            wsv_a,
            wsv_b,
        );
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
        prestep: &LinearAxisLimitPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut error = Vector::<f32>::splat(0.0);
        let mut normal = Vector3Wide::default();
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobians(
            &ab,
            orientation_a,
            orientation_b,
            &prestep.local_plane_normal,
            &prestep.local_offset_a,
            &prestep.local_offset_b,
            &prestep.minimum_offset,
            &prestep.maximum_offset,
            &mut error,
            &mut normal,
            &mut angular_ja,
            &mut angular_jb,
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

        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        let mut effective_mass = Vector::<f32>::splat(0.0);
        LinearAxisServoFunctions::compute_effective_mass(
            &angular_ja,
            &angular_jb,
            inertia_a,
            inertia_b,
            &effective_mass_cfm_scale,
            &mut angular_impulse_to_velocity_a,
            &mut angular_impulse_to_velocity_b,
            &mut effective_mass,
        );

        let mut bias_velocity = Vector::<f32>::splat(0.0);
        InequalityHelpers::compute_bias_velocity(
            error,
            &position_error_to_velocity,
            inverse_dt,
            &mut bias_velocity,
        );

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let mut linear_diff = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear, &mut linear_diff);
        let csv_linear = Vector3Wide::dot_val(&linear_diff, &normal);
        let csv_angular_a = Vector3Wide::dot_val(&wsv_a.angular, &angular_ja);
        let csv_angular_b = Vector3Wide::dot_val(&wsv_b.angular, &angular_jb);
        let csv = csv_linear + csv_angular_a + csv_angular_b;

        let mut csi =
            effective_mass * (bias_velocity - csv) - *accumulated_impulses * softness_impulse_scale;

        InequalityHelpers::clamp_positive(accumulated_impulses, &mut csi);
        LinearAxisServoFunctions::apply_impulse(
            &normal,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
            inertia_a,
            inertia_b,
            &csi,
            wsv_a,
            wsv_b,
        );
    }
}

pub struct LinearAxisLimitTypeProcessor;

impl LinearAxisLimitTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 40;
}
