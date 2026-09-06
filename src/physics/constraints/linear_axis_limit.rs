// Translated from BepuPhysics/Constraints/LinearAxisLimit.cs

use glam::Vec3;

use crate::out;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
#[cfg(debug_assertions)]
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
use std::mem::MaybeUninit;
use std::simd::Select;

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

        Vector3Wide::write_slot(
            self.local_offset_a,
            inner_index,
            &mut prestep_data.local_offset_a,
        );
        Vector3Wide::write_slot(
            self.local_offset_b,
            inner_index,
            &mut prestep_data.local_offset_b,
        );
        Vector3Wide::write_slot(
            self.local_axis,
            inner_index,
            &mut prestep_data.local_plane_normal,
        );
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.minimum_offset, inner_index) =
                self.minimum_offset;
            *GatherScatter::get_mut(&mut prestep_data.maximum_offset, inner_index) =
                self.maximum_offset;
            *GatherScatter::get_mut(
                &mut prestep_data.spring_settings.angular_frequency,
                inner_index,
            ) = self.spring_settings.angular_frequency;
            *GatherScatter::get_mut(
                &mut prestep_data.spring_settings.twice_damping_ratio,
                inner_index,
            ) = self.spring_settings.twice_damping_ratio;
        }
    }

    pub fn build_description(
        prestep_data: &LinearAxisLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut LinearAxisLimit,
    ) {
        Vector3Wide::read_slot(
            &prestep_data.local_offset_a,
            inner_index,
            &mut description.local_offset_a,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_offset_b,
            inner_index,
            &mut description.local_offset_b,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_plane_normal,
            inner_index,
            &mut description.local_axis,
        );
        unsafe {
            description.minimum_offset =
                *GatherScatter::get(&prestep_data.minimum_offset, inner_index);
            description.maximum_offset =
                *GatherScatter::get(&prestep_data.maximum_offset, inner_index);
            description.spring_settings.angular_frequency =
                *GatherScatter::get(&prestep_data.spring_settings.angular_frequency, inner_index);
            description.spring_settings.twice_damping_ratio = *GatherScatter::get(
                &prestep_data.spring_settings.twice_damping_ratio,
                inner_index,
            );
        }
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

        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        Matrix3x3Wide::transform_without_overlap(local_plane_normal, &orientation_matrix_a, normal);
        let anchor_a = out!(Matrix3x3Wide::transform_without_overlap(
            local_offset_a,
            &orientation_matrix_a
        ));
        let offset_b = out!(QuaternionWide::transform_without_overlap(
            local_offset_b,
            orientation_b
        ));
        let anchor_b = out!(Vector3Wide::add(ab, &offset_b));
        let diff = out!(Vector3Wide::subtract(&anchor_b, &anchor_a));
        let plane_normal_dot = out!(Vector3Wide::dot(&diff, normal));

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
        let offset_from_a = out!(Vector3Wide::subtract(&anchor_b, &scaled_normal));
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
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut error = MaybeUninit::<Vector<f32>>::uninit();
        let mut normal = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Vector3Wide>::uninit();
        unsafe {
            Self::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_plane_normal,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &prestep.minimum_offset,
                &prestep.maximum_offset,
                &mut *error.as_mut_ptr(),
                &mut *normal.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let normal = unsafe { normal.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };
        let angular_impulse_to_velocity_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_ja,
            &inertia_a.inverse_inertia_tensor
        ));
        let angular_impulse_to_velocity_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_jb,
            &inertia_b.inverse_inertia_tensor
        ));
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
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut error = MaybeUninit::<Vector<f32>>::uninit();
        let mut normal = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Vector3Wide>::uninit();
        unsafe {
            Self::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_plane_normal,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &prestep.minimum_offset,
                &prestep.maximum_offset,
                &mut *error.as_mut_ptr(),
                &mut *normal.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let error = unsafe { error.assume_init() };
        let normal = unsafe { normal.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };

        let mut position_error_to_velocity = MaybeUninit::<Vector<f32>>::uninit();
        let mut effective_mass_cfm_scale = MaybeUninit::<Vector<f32>>::uninit();
        let mut softness_impulse_scale = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            SpringSettingsWide::compute_springiness(
                &prestep.spring_settings,
                dt,
                &mut *position_error_to_velocity.as_mut_ptr(),
                &mut *effective_mass_cfm_scale.as_mut_ptr(),
                &mut *softness_impulse_scale.as_mut_ptr(),
            );
        }
        let position_error_to_velocity = unsafe { position_error_to_velocity.assume_init() };
        let effective_mass_cfm_scale = unsafe { effective_mass_cfm_scale.assume_init() };
        let softness_impulse_scale = unsafe { softness_impulse_scale.assume_init() };

        let mut angular_impulse_to_velocity_a = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_impulse_to_velocity_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut effective_mass = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            LinearAxisServoFunctions::compute_effective_mass(
                &angular_ja,
                &angular_jb,
                inertia_a,
                inertia_b,
                &effective_mass_cfm_scale,
                &mut *angular_impulse_to_velocity_a.as_mut_ptr(),
                &mut *angular_impulse_to_velocity_b.as_mut_ptr(),
                &mut *effective_mass.as_mut_ptr(),
            );
        }
        let angular_impulse_to_velocity_a = unsafe { angular_impulse_to_velocity_a.assume_init() };
        let angular_impulse_to_velocity_b = unsafe { angular_impulse_to_velocity_b.assume_init() };
        let effective_mass = unsafe { effective_mass.assume_init() };

        let bias_velocity = out!(InequalityHelpers::compute_bias_velocity(
            error,
            &position_error_to_velocity,
            inverse_dt
        ));

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let linear_diff = out!(Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear));
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
