// Translated from BepuPhysics/Constraints/PointOnLineServo.cs

use glam::Vec3;

use crate::out;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
#[cfg(debug_assertions)]
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
use std::mem::MaybeUninit;

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
            ConstraintChecker::assert_unit_length_vec3(
                self.local_direction,
                "PointOnLineServo",
                "local_direction",
            );
            ConstraintChecker::assert_valid_servo_settings(
                &self.servo_settings,
                "PointOnLineServo",
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
            self.local_direction,
            inner_index,
            &mut prestep_data.local_direction,
        );
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.maximum_speed, inner_index) =
                self.servo_settings.maximum_speed;
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.base_speed, inner_index) =
                self.servo_settings.base_speed;
            *GatherScatter::get_mut(&mut prestep_data.servo_settings.maximum_force, inner_index) =
                self.servo_settings.maximum_force;
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
        prestep_data: &PointOnLineServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut PointOnLineServo,
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
            &prestep_data.local_direction,
            inner_index,
            &mut description.local_direction,
        );
        unsafe {
            description.servo_settings.maximum_speed =
                *GatherScatter::get(&prestep_data.servo_settings.maximum_speed, inner_index);
            description.servo_settings.base_speed =
                *GatherScatter::get(&prestep_data.servo_settings.base_speed, inner_index);
            description.servo_settings.maximum_force =
                *GatherScatter::get(&prestep_data.servo_settings.maximum_force, inner_index);
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
        let linear_impulse_a = out!(Matrix2x3Wide::transform(csi, linear_jacobian));
        let angular_impulse_a = out!(Matrix2x3Wide::transform(csi, angular_jacobian_a));
        let angular_impulse_b = out!(Matrix2x3Wide::transform(csi, angular_jacobian_b));
        let angular_change_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_a,
            &inertia_a.inverse_inertia_tensor
        ));
        let angular_change_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_b,
            &inertia_b.inverse_inertia_tensor
        ));
        let linear_change_a = Vector3Wide::scale(&linear_impulse_a, &inertia_a.inverse_mass);
        let negated_linear_change_b =
            Vector3Wide::scale(&linear_impulse_a, &inertia_b.inverse_mass);

        velocity_a.linear = out!(Vector3Wide::add(&linear_change_a, &velocity_a.linear));
        velocity_a.angular = out!(Vector3Wide::add(&angular_change_a, &velocity_a.angular));
        velocity_b.linear = out!(Vector3Wide::subtract(
            &velocity_b.linear,
            &negated_linear_change_b
        ));
        velocity_b.angular = out!(Vector3Wide::add(&angular_change_b, &velocity_b.angular));
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
        let mut local_tangent_x = MaybeUninit::<Vector3Wide>::uninit();
        let mut local_tangent_y = MaybeUninit::<Vector3Wide>::uninit();
        unsafe {
            Helpers::build_orthonormal_basis(
                local_direction,
                &mut *local_tangent_x.as_mut_ptr(),
                &mut *local_tangent_y.as_mut_ptr(),
            );
        }
        let local_tangent_x = unsafe { local_tangent_x.assume_init() };
        let local_tangent_y = unsafe { local_tangent_y.assume_init() };
        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        let anchor_a = out!(Matrix3x3Wide::transform_without_overlap(
            local_offset_a,
            &orientation_matrix_a
        ));
        let offset_b = out!(QuaternionWide::transform_without_overlap(
            local_offset_b,
            orientation_b
        ));

        // Find offsetA by computing the closest point on the line to anchorB.
        let direction = out!(Matrix3x3Wide::transform_without_overlap(
            local_direction,
            &orientation_matrix_a
        ));
        let anchor_b = out!(Vector3Wide::add(&offset_b, ab));
        Vector3Wide::subtract(&anchor_b, &anchor_a, anchor_offset);
        let d = out!(Vector3Wide::dot(anchor_offset, &direction));
        let line_start_to_closest = Vector3Wide::scale(&direction, &d);
        let offset_a = out!(Vector3Wide::add(&line_start_to_closest, &anchor_a));

        Matrix3x3Wide::transform_without_overlap(
            &local_tangent_x,
            &orientation_matrix_a,
            &mut linear_jacobian.x,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_tangent_y,
            &orientation_matrix_a,
            &mut linear_jacobian.y,
        );

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
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut anchor_offset = MaybeUninit::<Vector3Wide>::uninit();
        let mut linear_jacobian = MaybeUninit::<Matrix2x3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Matrix2x3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Matrix2x3Wide>::uninit();
        unsafe {
            Self::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_direction,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &mut *anchor_offset.as_mut_ptr(),
                &mut *linear_jacobian.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let linear_jacobian = unsafe { linear_jacobian.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };
        Self::apply_impulse(
            wsv_a,
            wsv_b,
            &linear_jacobian,
            &angular_ja,
            &angular_jb,
            inertia_a,
            inertia_b,
            accumulated_impulses,
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
        prestep: &PointOnLineServoPrestepData,
        accumulated_impulses: &mut Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut anchor_offset = MaybeUninit::<Vector3Wide>::uninit();
        let mut linear_jacobian = MaybeUninit::<Matrix2x3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Matrix2x3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Matrix2x3Wide>::uninit();
        unsafe {
            Self::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_direction,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &mut *anchor_offset.as_mut_ptr(),
                &mut *linear_jacobian.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let anchor_offset = unsafe { anchor_offset.assume_init() };
        let linear_jacobian = unsafe { linear_jacobian.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };

        let inverse_mass_sum = inertia_a.inverse_mass + inertia_b.inverse_mass;
        let linear_contribution = out!(Symmetric2x2Wide::sandwich_scale(
            &linear_jacobian,
            &inverse_mass_sum
        ));
        let angular_contribution_a = out!(Symmetric3x3Wide::matrix_sandwich(
            &angular_ja,
            &inertia_a.inverse_inertia_tensor
        ));
        let angular_contribution_b = out!(Symmetric3x3Wide::matrix_sandwich(
            &angular_jb,
            &inertia_b.inverse_inertia_tensor
        ));
        let inverse_effective_mass = out!(Symmetric2x2Wide::add(
            &angular_contribution_a,
            &angular_contribution_b
        ));
        let inverse_effective_mass = out!(Symmetric2x2Wide::add(
            &inverse_effective_mass,
            &linear_contribution
        ));

        let effective_mass = out!(Symmetric2x2Wide::invert_without_overlap(
            &inverse_effective_mass
        ));

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
        let effective_mass = out!(Symmetric2x2Wide::scale(
            &effective_mass,
            &effective_mass_cfm_scale
        ));

        // CSV computation
        let linear_csv_a = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_a.linear,
            &linear_jacobian
        ));
        let negated_linear_csv_b = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_b.linear,
            &linear_jacobian
        ));
        let angular_csv_a = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_a.angular,
            &angular_ja
        ));
        let angular_csv_b = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_b.angular,
            &angular_jb
        ));
        let linear_csv = out!(Vector2Wide::subtract(&linear_csv_a, &negated_linear_csv_b));
        let angular_csv = out!(Vector2Wide::add(&angular_csv_a, &angular_csv_b));
        let csv = out!(Vector2Wide::add(&linear_csv, &angular_csv));

        // Position error and bias velocity.
        let mut error = MaybeUninit::<Vector2Wide>::uninit();
        unsafe {
            Vector3Wide::dot(
                &anchor_offset,
                &linear_jacobian.x,
                &mut (*error.as_mut_ptr()).x,
            );
            Vector3Wide::dot(
                &anchor_offset,
                &linear_jacobian.y,
                &mut (*error.as_mut_ptr()).y,
            );
        }
        let error = unsafe { error.assume_init() };
        let mut bias_velocity = MaybeUninit::<Vector2Wide>::uninit();
        let mut maximum_impulse = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            ServoSettingsWide::compute_clamped_bias_velocity_2d(
                &error,
                &position_error_to_velocity,
                &prestep.servo_settings,
                dt,
                inverse_dt,
                &mut *bias_velocity.as_mut_ptr(),
                &mut *maximum_impulse.as_mut_ptr(),
            );
        }
        let bias_velocity = unsafe { bias_velocity.assume_init() };
        let maximum_impulse = unsafe { maximum_impulse.assume_init() };

        let bias_minus_csv = out!(Vector2Wide::subtract(&bias_velocity, &csv));
        let csi = out!(Symmetric2x2Wide::transform_without_overlap(
            &bias_minus_csv,
            &effective_mass
        ));
        let softness_contribution = out!(Vector2Wide::scale(
            accumulated_impulses,
            &softness_impulse_scale
        ));
        let mut csi = out!(Vector2Wide::subtract(&csi, &softness_contribution));
        ServoSettingsWide::clamp_impulse_2d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(
            wsv_a,
            wsv_b,
            &linear_jacobian,
            &angular_ja,
            &angular_jb,
            inertia_a,
            inertia_b,
            &csi,
        );
    }
}

pub struct PointOnLineServoTypeProcessor;

impl PointOnLineServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 37;
}
