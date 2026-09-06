// Translated from BepuPhysics/Constraints/LinearAxisServo.cs

use glam::Vec3;

use crate::out;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
#[cfg(debug_assertions)]
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::mem::MaybeUninit;

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
            ConstraintChecker::assert_unit_length_vec3(
                self.local_plane_normal,
                "LinearAxisServo",
                "local_plane_normal",
            );
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "LinearAxisServo");
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
            self.local_plane_normal,
            inner_index,
            &mut prestep_data.local_plane_normal,
        );
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.target_offset, inner_index) =
                self.target_offset;
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
        prestep_data: &LinearAxisServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut LinearAxisServo,
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
            &mut description.local_plane_normal,
        );
        unsafe {
            description.target_offset =
                *GatherScatter::get(&prestep_data.target_offset, inner_index);
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
        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        Matrix3x3Wide::transform_without_overlap(
            local_plane_normal_a,
            &orientation_matrix_a,
            normal,
        );
        let anchor_a = out!(Matrix3x3Wide::transform_without_overlap(
            local_offset_a,
            &orientation_matrix_a
        ));
        let offset_b = out!(QuaternionWide::transform_without_overlap(
            local_offset_b,
            orientation_b
        ));
        // Note that the angular jacobian for A uses the offset from A to the attachment point on B.
        let anchor_b = out!(Vector3Wide::add(ab, &offset_b));
        let diff = out!(Vector3Wide::subtract(&anchor_b, &anchor_a));
        Vector3Wide::dot(&diff, normal, plane_normal_dot);
        // offsetFromAToClosestPointOnPlaneToB = anchorB - planeNormalDot * normal
        let scaled_normal = *normal * *plane_normal_dot;
        let offset_from_a = out!(Vector3Wide::subtract(&anchor_b, &scaled_normal));
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
        Symmetric3x3Wide::transform_without_overlap(
            angular_ja,
            &inertia_a.inverse_inertia_tensor,
            angular_impulse_to_velocity_a,
        );
        Symmetric3x3Wide::transform_without_overlap(
            angular_jb,
            &inertia_b.inverse_inertia_tensor,
            angular_impulse_to_velocity_b,
        );
        let angular_contribution_a =
            out!(Vector3Wide::dot(angular_ja, angular_impulse_to_velocity_a));
        let angular_contribution_b =
            out!(Vector3Wide::dot(angular_jb, angular_impulse_to_velocity_b));
        *effective_mass = *effective_mass_cfm_scale
            / (inertia_a.inverse_mass
                + inertia_b.inverse_mass
                + angular_contribution_a
                + angular_contribution_b);
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
        wsv_a.linear = out!(Vector3Wide::add(&linear_change_a, &wsv_a.linear));

        // velocityB.Linear -= linearJA * (csi * inertiaB.InverseMass);
        let negated_linear_change_b = *linear_ja * (*csi * inertia_b.inverse_mass);
        wsv_b.linear = out!(Vector3Wide::subtract(
            &wsv_b.linear,
            &negated_linear_change_b
        ));

        // velocityA.Angular += angularImpulseToVelocityA * csi;
        let angular_change_a = *angular_impulse_to_velocity_a * *csi;
        wsv_a.angular = out!(Vector3Wide::add(&angular_change_a, &wsv_a.angular));

        // velocityB.Angular += angularImpulseToVelocityB * csi;
        let angular_change_b = *angular_impulse_to_velocity_b * *csi;
        wsv_b.angular = out!(Vector3Wide::add(&angular_change_b, &wsv_b.angular));
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
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut plane_normal_dot = MaybeUninit::<Vector<f32>>::uninit();
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
                &mut *plane_normal_dot.as_mut_ptr(),
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
        Self::apply_impulse(
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
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut plane_normal_dot = MaybeUninit::<Vector<f32>>::uninit();
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
                &mut *plane_normal_dot.as_mut_ptr(),
                &mut *normal.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let plane_normal_dot = unsafe { plane_normal_dot.assume_init() };
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
            Self::compute_effective_mass(
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

        // Compute the position error and bias velocities.
        let error = plane_normal_dot - prestep.target_offset;
        let mut bias_velocity = MaybeUninit::<Vector<f32>>::uninit();
        let mut maximum_impulse = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            ServoSettingsWide::compute_clamped_bias_velocity_1d(
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

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let linear_diff = out!(Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear));
        let csv_linear = Vector3Wide::dot_val(&linear_diff, &normal);
        let csv_angular_a = Vector3Wide::dot_val(&wsv_a.angular, &angular_ja);
        let csv_angular_b = Vector3Wide::dot_val(&wsv_b.angular, &angular_jb);
        let csv = csv_linear + csv_angular_a + csv_angular_b;

        let mut csi =
            effective_mass * (bias_velocity - csv) - *accumulated_impulses * softness_impulse_scale;

        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        Self::apply_impulse(
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

pub struct LinearAxisServoTypeProcessor;

impl LinearAxisServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 38;
}
