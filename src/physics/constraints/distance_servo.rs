// Translated from BepuPhysics/Constraints/DistanceServo.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::servo_settings::{ServoSettings, ServoSettingsWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Constrains points on two bodies to be separated by a goal distance.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct DistanceServo {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Distance that the constraint will try to reach between the attachment points.
    pub target_distance: f32,
    /// Servo control parameters.
    pub servo_settings: ServoSettings,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl DistanceServo {
    pub const CONSTRAINT_TYPE_ID: i32 = DistanceServoTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut DistanceServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                self.target_distance >= 0.0,
                "DistanceServo.target_distance must be nonnegative."
            );
            ConstraintChecker::assert_valid_servo_settings(&self.servo_settings, "DistanceServo");
        }

        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.target_distance) = self.target_distance;
        }
        ServoSettingsWide::write_first(&self.servo_settings, &mut target.servo_settings);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &DistanceServoPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut DistanceServo,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        description.target_distance = source.target_distance[0];
        ServoSettingsWide::read_first(&source.servo_settings, &mut description.servo_settings);
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct DistanceServoPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub target_distance: Vector<f32>,
    pub servo_settings: ServoSettingsWide,
    pub spring_settings: SpringSettingsWide,
}

pub struct DistanceServoFunctions;

impl DistanceServoFunctions {
    #[inline(always)]
    pub fn get_distance(
        orientation_a: &QuaternionWide,
        ab: &Vector3Wide,
        orientation_b: &QuaternionWide,
        local_offset_a: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        anchor_offset_a: &mut Vector3Wide,
        anchor_offset_b: &mut Vector3Wide,
        anchor_offset: &mut Vector3Wide,
        distance: &mut Vector<f32>,
    ) {
        QuaternionWide::transform_without_overlap(local_offset_a, orientation_a, anchor_offset_a);
        QuaternionWide::transform_without_overlap(local_offset_b, orientation_b, anchor_offset_b);
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(anchor_offset_b, ab, &mut anchor_b);
        Vector3Wide::subtract(&anchor_b, anchor_offset_a, anchor_offset);
        Vector3Wide::length_into(anchor_offset, distance);
    }

    #[inline(always)]
    pub fn compute_jacobian(
        distance: &Vector<f32>,
        anchor_offset_a: &Vector3Wide,
        anchor_offset_b: &Vector3Wide,
        direction: &mut Vector3Wide,
        angular_ja: &mut Vector3Wide,
        angular_jb: &mut Vector3Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;

        // If the distance is zero, there is no valid offset direction. Pick one arbitrarily.
        let need_fallback = distance.simd_lt(Vector::<f32>::splat(1e-9));
        direction.x = need_fallback.select(Vector::<f32>::splat(1.0), direction.x);
        direction.y = need_fallback.select(Vector::<f32>::splat(0.0), direction.y);
        direction.z = need_fallback.select(Vector::<f32>::splat(0.0), direction.z);

        unsafe {
            Vector3Wide::cross_without_overlap(anchor_offset_a, direction, angular_ja);
            Vector3Wide::cross_without_overlap(direction, anchor_offset_b, angular_jb);
            // Note flip negation.
        }
    }

    #[inline(always)]
    pub fn compute_transforms(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        anchor_offset_a: &Vector3Wide,
        anchor_offset_b: &Vector3Wide,
        distance: &Vector<f32>,
        direction: &mut Vector3Wide,
        dt: f32,
        spring_settings: &SpringSettingsWide,
        position_error_to_velocity: &mut Vector<f32>,
        softness_impulse_scale: &mut Vector<f32>,
        effective_mass: &mut Vector<f32>,
        angular_ja: &mut Vector3Wide,
        angular_jb: &mut Vector3Wide,
        angular_impulse_to_velocity_a: &mut Vector3Wide,
        angular_impulse_to_velocity_b: &mut Vector3Wide,
    ) {
        Self::compute_jacobian(
            distance,
            anchor_offset_a,
            anchor_offset_b,
            direction,
            angular_ja,
            angular_jb,
        );

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
        let mut angular_contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            angular_ja,
            angular_impulse_to_velocity_a,
            &mut angular_contribution_a,
        );
        let mut angular_contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            angular_jb,
            angular_impulse_to_velocity_b,
            &mut angular_contribution_b,
        );
        let inverse_effective_mass = inertia_a.inverse_mass
            + inertia_b.inverse_mass
            + angular_contribution_a
            + angular_contribution_b;

        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            spring_settings,
            dt,
            position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            softness_impulse_scale,
        );
        *effective_mass = effective_mass_cfm_scale / inverse_effective_mass;
    }

    #[inline(always)]
    pub fn apply_impulse(
        inverse_mass_a: &Vector<f32>,
        inverse_mass_b: &Vector<f32>,
        direction: &Vector3Wide,
        angular_impulse_to_velocity_a: &Vector3Wide,
        angular_impulse_to_velocity_b: &Vector3Wide,
        csi: &Vector<f32>,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
    ) {
        let linear_velocity_change_a = Vector3Wide::scale(direction, &(*csi * *inverse_mass_a));
        let angular_velocity_change_a = *angular_impulse_to_velocity_a * *csi;
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&linear_velocity_change_a, &velocity_a.linear, &mut tmp);
        velocity_a.linear = tmp;
        Vector3Wide::add(&angular_velocity_change_a, &velocity_a.angular, &mut tmp);
        velocity_a.angular = tmp;

        let negated_linear_velocity_change_b =
            Vector3Wide::scale(direction, &(*csi * *inverse_mass_b));
        let angular_velocity_change_b = *angular_impulse_to_velocity_b * *csi;
        Vector3Wide::subtract(
            &velocity_b.linear,
            &negated_linear_velocity_change_b,
            &mut tmp,
        );
        velocity_b.linear = tmp;
        Vector3Wide::add(&angular_velocity_change_b, &velocity_b.angular, &mut tmp);
        velocity_b.angular = tmp;
    }

    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &DistanceServoPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut anchor_offset_a = Vector3Wide::default();
        let mut anchor_offset_b = Vector3Wide::default();
        let mut anchor_offset = Vector3Wide::default();
        let mut distance = Vector::<f32>::splat(0.0);
        Self::get_distance(
            orientation_a,
            &ab,
            orientation_b,
            &prestep.local_offset_a,
            &prestep.local_offset_b,
            &mut anchor_offset_a,
            &mut anchor_offset_b,
            &mut anchor_offset,
            &mut distance,
        );
        let mut direction =
            Vector3Wide::scale(&anchor_offset, &(Vector::<f32>::splat(1.0) / distance));
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        Self::compute_jacobian(
            &distance,
            &anchor_offset_a,
            &anchor_offset_b,
            &mut direction,
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
        Self::apply_impulse(
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            &direction,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
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
        prestep: &DistanceServoPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut anchor_offset_a = Vector3Wide::default();
        let mut anchor_offset_b = Vector3Wide::default();
        let mut anchor_offset = Vector3Wide::default();
        let mut distance = Vector::<f32>::splat(0.0);
        Self::get_distance(
            orientation_a,
            &ab,
            orientation_b,
            &prestep.local_offset_a,
            &prestep.local_offset_b,
            &mut anchor_offset_a,
            &mut anchor_offset_b,
            &mut anchor_offset,
            &mut distance,
        );

        let mut direction =
            Vector3Wide::scale(&anchor_offset, &(Vector::<f32>::splat(1.0) / distance));

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut effective_mass = Vector::<f32>::splat(0.0);
        let mut angular_ja = Vector3Wide::default();
        let mut angular_jb = Vector3Wide::default();
        let mut angular_impulse_to_velocity_a = Vector3Wide::default();
        let mut angular_impulse_to_velocity_b = Vector3Wide::default();
        Self::compute_transforms(
            inertia_a,
            inertia_b,
            &anchor_offset_a,
            &anchor_offset_b,
            &distance,
            &mut direction,
            dt,
            &prestep.spring_settings,
            &mut position_error_to_velocity,
            &mut softness_impulse_scale,
            &mut effective_mass,
            &mut angular_ja,
            &mut angular_jb,
            &mut angular_impulse_to_velocity_a,
            &mut angular_impulse_to_velocity_b,
        );

        // Compute the position error and bias velocities.
        let error = distance - prestep.target_distance;
        let mut clamped_bias_velocity = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        ServoSettingsWide::compute_clamped_bias_velocity_1d(
            &error,
            &position_error_to_velocity,
            &prestep.servo_settings,
            dt,
            inverse_dt,
            &mut clamped_bias_velocity,
            &mut maximum_impulse,
        );

        let mut linear_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.linear, &direction, &mut linear_csv_a);
        let mut negated_linear_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.linear, &direction, &mut negated_linear_csv_b);
        let mut angular_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.angular, &angular_ja, &mut angular_csv_a);
        let mut angular_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.angular, &angular_jb, &mut angular_csv_b);
        let mut csi = (clamped_bias_velocity - linear_csv_a - angular_csv_a + negated_linear_csv_b
            - angular_csv_b)
            * effective_mass
            - *accumulated_impulses * softness_impulse_scale;
        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);

        Self::apply_impulse(
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            &direction,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
            &csi,
            wsv_a,
            wsv_b,
        );
    }
}

pub struct DistanceServoTypeProcessor;

impl DistanceServoTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 33;
}
