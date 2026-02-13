// Translated from BepuPhysics/Constraints/ServoSettings.cs

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// Describes how quickly and strongly a servo constraint should move towards a position target.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ServoSettings {
    /// Maximum speed that the constraint can try to use to move towards the target.
    pub maximum_speed: f32,
    /// Minimum speed that the constraint will try to use to move towards the target.
    /// If the speed implied by the spring configuration is higher than this, the servo will
    /// attempt to use the higher speed. Will be clamped by the MaximumSpeed.
    pub base_speed: f32,
    /// The maximum force that the constraint can apply to move towards the target.
    pub maximum_force: f32,
}

impl ServoSettings {
    /// Gets settings representing a servo with unlimited force, speed, and no base speed.
    /// A servo with these settings will behave like a conventional position-level constraint.
    pub fn default_settings() -> Self {
        Self::new(f32::MAX, 0.0, f32::MAX)
    }

    /// Checks servo settings to ensure valid values.
    #[inline(always)]
    pub fn validate(settings: &ServoSettings) -> bool {
        use super::constraint_checker::ConstraintChecker;
        ConstraintChecker::is_nonnegative_number(settings.maximum_speed)
            && ConstraintChecker::is_nonnegative_number(settings.base_speed)
            && ConstraintChecker::is_nonnegative_number(settings.maximum_force)
    }

    /// Creates a new servo settings instance with the specified properties.
    pub fn new(maximum_speed: f32, base_speed: f32, maximum_force: f32) -> Self {
        let settings = Self {
            maximum_speed,
            base_speed,
            maximum_force,
        };
        debug_assert!(
            Self::validate(&settings),
            "Servo settings must have nonnegative maximum speed, base speed, and maximum force."
        );
        settings
    }
}

/// SIMD-wide servo settings.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ServoSettingsWide {
    pub maximum_speed: Vector<f32>,
    pub base_speed: Vector<f32>,
    pub maximum_force: Vector<f32>,
}

impl ServoSettingsWide {
    /// Computes a clamped bias velocity for a 1D error.
    #[inline(always)]
    pub fn compute_clamped_bias_velocity_1d(
        error: &Vector<f32>,
        position_error_to_velocity: &Vector<f32>,
        servo_settings: &ServoSettingsWide,
        dt: f32,
        inverse_dt: f32,
        clamped_bias_velocity: &mut Vector<f32>,
        maximum_impulse: &mut Vector<f32>,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        // Can't request speed that would cause an overshoot.
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let base_speed = servo_settings
            .base_speed
            .simd_min(error.abs() * inverse_dt_wide);
        let bias_velocity = *error * *position_error_to_velocity;
        let zero = Vector::<f32>::splat(0.0);
        let neg_mask = bias_velocity.simd_lt(zero);
        let neg_branch =
            (-servo_settings.maximum_speed).simd_max((-base_speed).simd_min(bias_velocity));
        let pos_branch = servo_settings
            .maximum_speed
            .simd_min(base_speed.simd_max(bias_velocity));
        *clamped_bias_velocity = neg_mask.select(neg_branch, pos_branch);
        *maximum_impulse = servo_settings.maximum_force * Vector::<f32>::splat(dt);
    }

    /// Computes a clamped bias velocity for a 3D error using axis + length.
    #[inline(always)]
    pub fn compute_clamped_bias_velocity_3d_axis(
        error_axis: &Vector3Wide,
        error_length: &Vector<f32>,
        position_error_to_bias_velocity: &Vector<f32>,
        servo_settings: &ServoSettingsWide,
        dt: f32,
        inverse_dt: f32,
        clamped_bias_velocity: &mut Vector3Wide,
        maximum_impulse: &mut Vector<f32>,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let one = Vector::<f32>::splat(1.0);
        let epsilon = Vector::<f32>::splat(1e-10);

        // Can't request speed that would cause an overshoot.
        let base_speed = servo_settings
            .base_speed
            .simd_min(*error_length * inverse_dt_wide);
        let unclamped_bias_speed = *error_length * *position_error_to_bias_velocity;
        let target_speed = base_speed.simd_max(unclamped_bias_speed);
        let scale_raw = one.simd_min(servo_settings.maximum_speed / target_speed);
        // Protect against division by zero.
        let use_fallback = target_speed.simd_lt(epsilon);
        let scale = use_fallback.select(one, scale_raw);
        Vector3Wide::scale_to(
            error_axis,
            &(scale * unclamped_bias_speed),
            clamped_bias_velocity,
        );
        *maximum_impulse = servo_settings.maximum_force * Vector::<f32>::splat(dt);
    }

    /// Computes a clamped bias velocity for a 3D error vector.
    #[inline(always)]
    pub fn compute_clamped_bias_velocity_3d(
        error: &Vector3Wide,
        position_error_to_bias_velocity: &Vector<f32>,
        servo_settings: &ServoSettingsWide,
        dt: f32,
        inverse_dt: f32,
        clamped_bias_velocity: &mut Vector3Wide,
        maximum_impulse: &mut Vector<f32>,
    ) {
        use std::simd::cmp::SimdPartialOrd;

        let mut error_length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(error, &mut error_length);
        let one = Vector::<f32>::splat(1.0);
        let epsilon = Vector::<f32>::splat(1e-10);
        let inv_length = one / error_length;
        let mut error_axis = Vector3Wide::default();
        Vector3Wide::scale_to(error, &inv_length, &mut error_axis);
        let use_fallback = error_length.simd_lt(epsilon);
        let zero = Vector::<f32>::splat(0.0);
        error_axis.x = use_fallback.select(zero, error_axis.x);
        error_axis.y = use_fallback.select(zero, error_axis.y);
        error_axis.z = use_fallback.select(zero, error_axis.z);
        Self::compute_clamped_bias_velocity_3d_axis(
            &error_axis,
            &error_length,
            position_error_to_bias_velocity,
            servo_settings,
            dt,
            inverse_dt,
            clamped_bias_velocity,
            maximum_impulse,
        );
    }

    /// Computes a clamped bias velocity for a 2D error using axis + length.
    #[inline(always)]
    pub fn compute_clamped_bias_velocity_2d_axis(
        error_axis: &Vector2Wide,
        error_length: &Vector<f32>,
        position_error_to_bias_velocity: &Vector<f32>,
        servo_settings: &ServoSettingsWide,
        dt: f32,
        inverse_dt: f32,
        clamped_bias_velocity: &mut Vector2Wide,
        maximum_impulse: &mut Vector<f32>,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let base_speed = servo_settings
            .base_speed
            .simd_min(*error_length * inverse_dt_wide);
        let unclamped_bias_speed = *error_length * *position_error_to_bias_velocity;
        let target_speed = base_speed.simd_max(unclamped_bias_speed);
        let scale_raw =
            Vector::<f32>::splat(1.0).simd_min(servo_settings.maximum_speed / target_speed);
        let epsilon = Vector::<f32>::splat(1e-10);
        let use_fallback = target_speed.simd_lt(epsilon);
        let scale = use_fallback.select(Vector::<f32>::splat(1.0), scale_raw);
        let bias_scale = scale * unclamped_bias_speed;
        Vector2Wide::scale(error_axis, &bias_scale, clamped_bias_velocity);
        *maximum_impulse = servo_settings.maximum_force * Vector::<f32>::splat(dt);
    }

    /// Computes a clamped bias velocity for a 2D error.
    #[inline(always)]
    pub fn compute_clamped_bias_velocity_2d(
        error: &Vector2Wide,
        position_error_to_bias_velocity: &Vector<f32>,
        servo_settings: &ServoSettingsWide,
        dt: f32,
        inverse_dt: f32,
        clamped_bias_velocity: &mut Vector2Wide,
        maximum_impulse: &mut Vector<f32>,
    ) {
        use std::simd::cmp::SimdPartialOrd;

        let mut error_length = Vector::<f32>::splat(0.0);
        Vector2Wide::length(error, &mut error_length);
        let inv_length = Vector::<f32>::splat(1.0) / error_length;
        let mut error_axis = Vector2Wide::default();
        Vector2Wide::scale(error, &inv_length, &mut error_axis);
        let epsilon = Vector::<f32>::splat(1e-10);
        let use_fallback = error_length.simd_lt(epsilon);
        let zero = Vector::<f32>::splat(0.0);
        error_axis.x = use_fallback.select(zero, error_axis.x);
        error_axis.y = use_fallback.select(zero, error_axis.y);
        Self::compute_clamped_bias_velocity_2d_axis(
            &error_axis,
            &error_length,
            position_error_to_bias_velocity,
            servo_settings,
            dt,
            inverse_dt,
            clamped_bias_velocity,
            maximum_impulse,
        );
    }

    /// Clamps a 2D impulse magnitude within maximum_impulse.
    #[inline(always)]
    pub fn clamp_impulse_2d(
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector2Wide,
        csi: &mut Vector2Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        let previous = *accumulated_impulse;
        let mut unclamped = Vector2Wide::default();
        Vector2Wide::add(accumulated_impulse, csi, &mut unclamped);
        let mut impulse_magnitude = Vector::<f32>::splat(0.0);
        Vector2Wide::length(&unclamped, &mut impulse_magnitude);
        let epsilon = Vector::<f32>::splat(1e-10);
        let one = Vector::<f32>::splat(1.0);
        let impulse_scale_raw = one.simd_min(*maximum_impulse / impulse_magnitude);
        let use_fallback = impulse_magnitude.abs().simd_lt(epsilon);
        let impulse_scale = use_fallback.select(one, impulse_scale_raw);
        Vector2Wide::scale(&unclamped, &impulse_scale, accumulated_impulse);
        Vector2Wide::subtract(accumulated_impulse, &previous, csi);
    }

    /// Clamps a 1D impulse within [-maximum_impulse, maximum_impulse].
    #[inline(always)]
    pub fn clamp_impulse_1d(
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector<f32>,
        csi: &mut Vector<f32>,
    ) {
        use std::simd::num::SimdFloat;
        let previous_impulse = *accumulated_impulse;
        *accumulated_impulse =
            (-*maximum_impulse).simd_max((*maximum_impulse).simd_min(*accumulated_impulse + *csi));
        *csi = *accumulated_impulse - previous_impulse;
    }

    /// Clamps a 3D impulse magnitude within maximum_impulse.
    #[inline(always)]
    pub fn clamp_impulse_3d(
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector3Wide,
        csi: &mut Vector3Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;
        use std::simd::num::SimdFloat;

        let previous = *accumulated_impulse;
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(accumulated_impulse, csi, &mut tmp);
        *accumulated_impulse = tmp;
        let mut impulse_magnitude = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(accumulated_impulse, &mut impulse_magnitude);
        let epsilon = Vector::<f32>::splat(1e-10);
        let one = Vector::<f32>::splat(1.0);
        let impulse_scale_raw = one.simd_min(*maximum_impulse / impulse_magnitude);
        let use_fallback = impulse_magnitude.abs().simd_lt(epsilon);
        let impulse_scale = use_fallback.select(one, impulse_scale_raw);
        *accumulated_impulse = Vector3Wide::scale(accumulated_impulse, &impulse_scale);
        Vector3Wide::subtract(accumulated_impulse, &previous, csi);
    }

    #[inline(always)]
    pub fn write_first(source: &ServoSettings, target: &mut ServoSettingsWide) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target.maximum_speed) = source.maximum_speed;
            *GatherScatter::get_first_mut(&mut target.base_speed) = source.base_speed;
            *GatherScatter::get_first_mut(&mut target.maximum_force) = source.maximum_force;
        }
    }

    #[inline(always)]
    pub fn read_first(source: &ServoSettingsWide, target: &mut ServoSettings) {
        target.maximum_speed = source.maximum_speed[0];
        target.base_speed = source.base_speed[0];
        target.maximum_force = source.maximum_force[0];
    }
}
