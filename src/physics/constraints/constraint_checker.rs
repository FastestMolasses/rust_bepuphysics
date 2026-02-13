// Translated from BepuPhysics/Constraints/ConstraintChecker.cs

use glam::{Quat, Vec3};

/// Provides helper functions for validating constraint parameter values.
pub struct ConstraintChecker;

impl ConstraintChecker {
    /// Checks if a value is a finite number — neither infinite nor NaN.
    #[inline(always)]
    pub fn is_finite_number(value: f32) -> bool {
        !value.is_infinite() && !value.is_nan()
    }

    /// Checks if a value is a finite value greater than zero and not NaN.
    #[inline(always)]
    pub fn is_positive_number(value: f32) -> bool {
        Self::is_finite_number(value) && value > 0.0
    }

    /// Checks if a value is a finite value greater than or equal to zero and not NaN.
    #[inline(always)]
    pub fn is_nonnegative_number(value: f32) -> bool {
        Self::is_finite_number(value) && value >= 0.0
    }

    /// Checks if a value is a finite value less than zero and not NaN.
    #[inline(always)]
    pub fn is_negative_number(value: f32) -> bool {
        Self::is_finite_number(value) && value < 0.0
    }

    /// Checks if a value is a finite value less than or equal to zero and not NaN.
    #[inline(always)]
    pub fn is_nonpositive_number(value: f32) -> bool {
        Self::is_finite_number(value) && value <= 0.0
    }

    #[cfg(debug_assertions)]
    pub fn assert_unit_length_vec3(v: Vec3, type_name: &str, property_name: &str) {
        let length_squared = v.length_squared();
        debug_assert!(
            length_squared <= 1.0 + 1e-5
                && length_squared >= 1.0 - 1e-5
                && Self::is_finite_number(length_squared),
            "{}.{} must be unit length.",
            type_name,
            property_name
        );
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_unit_length_vec3(_v: Vec3, _type_name: &str, _property_name: &str) {}

    #[cfg(debug_assertions)]
    pub fn assert_unit_length_quat(q: Quat, type_name: &str, property_name: &str) {
        let length_squared = q.length_squared();
        debug_assert!(
            length_squared <= 1.0 + 1e-5
                && length_squared >= 1.0 - 1e-5
                && Self::is_finite_number(length_squared),
            "{}.{} must be unit length.",
            type_name,
            property_name
        );
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_unit_length_quat(_q: Quat, _type_name: &str, _property_name: &str) {}

    #[cfg(debug_assertions)]
    pub fn assert_valid_spring_settings(
        settings: &super::spring_settings::SpringSettings,
        type_name: &str,
    ) {
        debug_assert!(
            super::spring_settings::SpringSettings::validate(settings),
            "{}.SpringSettings must have positive frequency and nonnegative damping ratio.",
            type_name
        );
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_valid_spring_settings(
        _settings: &super::spring_settings::SpringSettings,
        _type_name: &str,
    ) {
    }

    #[cfg(debug_assertions)]
    pub fn assert_valid_motor_settings(
        settings: &super::motor_settings::MotorSettings,
        type_name: &str,
    ) {
        debug_assert!(
            super::motor_settings::MotorSettings::validate(settings),
            "{}.MotorSettings must have nonnegative maximum force and damping.",
            type_name
        );
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_valid_motor_settings(
        _settings: &super::motor_settings::MotorSettings,
        _type_name: &str,
    ) {
    }

    #[cfg(debug_assertions)]
    pub fn assert_valid_servo_settings(
        settings: &super::servo_settings::ServoSettings,
        type_name: &str,
    ) {
        debug_assert!(
            super::servo_settings::ServoSettings::validate(settings),
            "{}.ServoSettings must have nonnegative maximum speed, base speed, and maximum force.",
            type_name
        );
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_valid_servo_settings(
        _settings: &super::servo_settings::ServoSettings,
        _type_name: &str,
    ) {
    }

    #[cfg(debug_assertions)]
    pub fn assert_valid_servo_and_spring_settings(
        servo_settings: &super::servo_settings::ServoSettings,
        spring_settings: &super::spring_settings::SpringSettings,
        type_name: &str,
    ) {
        Self::assert_valid_servo_settings(servo_settings, type_name);
        Self::assert_valid_spring_settings(spring_settings, type_name);
    }

    #[cfg(not(debug_assertions))]
    #[inline(always)]
    pub fn assert_valid_servo_and_spring_settings(
        _servo_settings: &super::servo_settings::ServoSettings,
        _spring_settings: &super::spring_settings::SpringSettings,
        _type_name: &str,
    ) {
    }
}
