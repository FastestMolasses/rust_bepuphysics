use glam::Vec3;
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::math_helper;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

use crate::physics::body_properties::BodyVelocityWide;

/// Helper functions for computing bounding box expansions.
pub struct BoundingBoxHelpers;

impl BoundingBoxHelpers {
    /// Computes the angular bounds expansion for bundled data (SIMD).
    #[inline(always)]
    pub fn get_angular_bounds_expansion_wide(
        angular_speed: Vector<f32>,
        vector_dt: Vector<f32>,
        maximum_radius: Vector<f32>,
        maximum_angular_expansion: Vector<f32>,
    ) -> Vector<f32> {
        /*
        Angular requires a bit more care. Since the goal is to create a tight bound, simply using a v = w * r approximation isn't ideal.
        A slightly tighter can be found:
        1) The maximum displacement along ANY axis during an intermediate time is equal to the distance from a starting position at MaximumRadius
           to the position of that point at the intermediate time.
        2) The expansion cannot exceed the maximum radius, so angular deltas greater than pi/3 do not need to be considered.
        3) The largest displacement along any axis, at any time, is the distance from the starting position to the position at dt.
        4) position(time) = {radius * sin(angular speed * time), radius * cos(angular speed * time)}
        5) largest expansion required = ||position(dt) - position(0)|| = sqrt(2 * radius^2 * (1 - cos(dt * w)))
        6) cos(x) ≈ 1 - x^2 / 2! + x^4 / 4! - x^6 / 6!
        */
        let pi_over_3 = Vector::<f32>::splat(std::f32::consts::FRAC_PI_3);
        let a = (angular_speed * vector_dt).simd_min(pi_over_3);
        let a2 = a * a;
        let a4 = a2 * a2;
        let a6 = a4 * a2;
        let cos_angle_minus_one = a2 * Vector::<f32>::splat(-1.0 / 2.0)
            + a4 * Vector::<f32>::splat(1.0 / 24.0)
            - a6 * Vector::<f32>::splat(1.0 / 720.0);
        // Note that it's impossible for angular motion to cause an increase in bounding box size
        // beyond (maximumRadius-minimumRadius) on any given axis.
        maximum_angular_expansion.simd_min(
            (Vector::<f32>::splat(-2.0) * maximum_radius * maximum_radius * cos_angle_minus_one)
                .sqrt(),
        )
    }

    /// Computes bounds expansion from linear velocity and angular expansion (SIMD).
    #[inline(always)]
    pub fn get_bounds_expansion_wide(
        linear_velocity: &Vector3Wide,
        dt_wide: Vector<f32>,
        angular_expansion: Vector<f32>,
        min_expansion: &mut Vector3Wide,
        max_expansion: &mut Vector3Wide,
    ) {
        let mut linear_displacement = Vector3Wide::default();
        Vector3Wide::scale_to(linear_velocity, &dt_wide, &mut linear_displacement);
        let zero = Vector::<f32>::splat(0.0);
        Vector3Wide::min_scalar(&zero, &linear_displacement, min_expansion);
        Vector3Wide::max_scalar(&zero, &linear_displacement, max_expansion);
        // Subtract/add angular expansion in-place
        min_expansion.x -= angular_expansion;
        min_expansion.y -= angular_expansion;
        min_expansion.z -= angular_expansion;
        max_expansion.x += angular_expansion;
        max_expansion.y += angular_expansion;
        max_expansion.z += angular_expansion;
    }

    /// Computes bounds expansion from linear and angular velocities (SIMD).
    #[inline(always)]
    pub fn get_bounds_expansion_wide_full(
        linear_velocity: &Vector3Wide,
        angular_velocity: &Vector3Wide,
        dt_wide: Vector<f32>,
        maximum_radius: Vector<f32>,
        maximum_angular_expansion: Vector<f32>,
        min_bounds_expansion: &mut Vector3Wide,
        max_bounds_expansion: &mut Vector3Wide,
    ) {
        let mut angular_speed = Vector::<f32>::default();
        Vector3Wide::length_into(angular_velocity, &mut angular_speed);
        let angular_expansion = Self::get_angular_bounds_expansion_wide(
            angular_speed,
            dt_wide,
            maximum_radius,
            maximum_angular_expansion,
        );
        Self::get_bounds_expansion_wide(
            linear_velocity,
            dt_wide,
            angular_expansion,
            min_bounds_expansion,
            max_bounds_expansion,
        );
    }

    /// Expands bounding boxes using velocities (SIMD).
    #[inline(always)]
    pub fn expand_bounding_boxes_wide(
        velocities: &BodyVelocityWide,
        dt_wide: Vector<f32>,
        maximum_radius: Vector<f32>,
        maximum_angular_expansion: Vector<f32>,
        maximum_expansion: Vector<f32>,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    ) {
        let mut min_displacement = Vector3Wide::default();
        let mut max_displacement = Vector3Wide::default();
        Self::get_bounds_expansion_wide_full(
            &velocities.linear,
            &velocities.angular,
            dt_wide,
            maximum_radius,
            maximum_angular_expansion,
            &mut min_displacement,
            &mut max_displacement,
        );
        let neg_max_expansion = -maximum_expansion;
        // Clamp displacement to maximum expansion (in-place to avoid aliasing)
        min_displacement.x = min_displacement.x.simd_max(neg_max_expansion);
        min_displacement.y = min_displacement.y.simd_max(neg_max_expansion);
        min_displacement.z = min_displacement.z.simd_max(neg_max_expansion);
        max_displacement.x = max_displacement.x.simd_min(maximum_expansion);
        max_displacement.y = max_displacement.y.simd_min(maximum_expansion);
        max_displacement.z = max_displacement.z.simd_min(maximum_expansion);

        min.x += min_displacement.x;
        min.y += min_displacement.y;
        min.z += min_displacement.z;
        max.x += max_displacement.x;
        max.y += max_displacement.y;
        max.z += max_displacement.z;
    }

    // --- Scalar versions ---

    /// Computes the angular bounds expansion (scalar version).
    #[inline(always)]
    pub fn get_angular_bounds_expansion(
        angular_velocity_magnitude: f32,
        dt: f32,
        maximum_radius: f32,
        maximum_angular_expansion: f32,
    ) -> f32 {
        let a = math_helper::min(
            angular_velocity_magnitude * dt,
            std::f32::consts::FRAC_PI_3,
        );
        let a2 = a * a;
        let a4 = a2 * a2;
        let a6 = a4 * a2;
        let cos_angle_minus_one = a2 * (-1.0 / 2.0) + a4 * (1.0 / 24.0) - a6 * (1.0 / 720.0);
        math_helper::min(
            maximum_angular_expansion,
            (-2.0f32 * maximum_radius * maximum_radius * cos_angle_minus_one).sqrt(),
        )
    }

    /// Computes bounds expansion from linear velocity and angular expansion (scalar).
    #[inline(always)]
    pub fn get_bounds_expansion_scalar(
        linear_velocity: Vec3,
        dt: f32,
        angular_expansion: f32,
        min_expansion: &mut Vec3,
        max_expansion: &mut Vec3,
    ) {
        let linear_displacement = linear_velocity * dt;
        let zero = Vec3::ZERO;
        let broadcast_expansion = Vec3::splat(angular_expansion);
        *min_expansion = linear_displacement.min(zero) - broadcast_expansion;
        *max_expansion = linear_displacement.max(zero) + broadcast_expansion;
    }

    /// Computes bounds expansion from linear and angular velocities (scalar) with clamping.
    #[inline(always)]
    pub fn get_bounds_expansion_scalar_full(
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        dt: f32,
        maximum_radius: f32,
        maximum_angular_expansion: f32,
        maximum_allowed_expansion: f32,
        min_expansion: &mut Vec3,
        max_expansion: &mut Vec3,
    ) {
        let linear_displacement = linear_velocity * dt;
        let zero = Vec3::ZERO;
        *min_expansion = linear_displacement.min(zero);
        *max_expansion = linear_displacement.max(zero);
        let angular_expansion = Vec3::splat(Self::get_angular_bounds_expansion(
            angular_velocity.length(),
            dt,
            maximum_radius,
            maximum_angular_expansion,
        ));

        let maximum_allowed_expansion_broadcasted = Vec3::splat(maximum_allowed_expansion);
        *min_expansion = (-maximum_allowed_expansion_broadcasted).max(*min_expansion - angular_expansion);
        *max_expansion = maximum_allowed_expansion_broadcasted.min(*max_expansion + angular_expansion);
    }

    /// Expands min/max bounding box using linear and angular velocities (scalar).
    #[inline(always)]
    pub fn expand_bounding_box(
        min: &mut Vec3,
        max: &mut Vec3,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        dt: f32,
        maximum_radius: f32,
        maximum_angular_expansion: f32,
        maximum_allowed_expansion: f32,
    ) {
        let mut min_expansion = Vec3::ZERO;
        let mut max_expansion = Vec3::ZERO;
        Self::get_bounds_expansion_scalar_full(
            linear_velocity,
            angular_velocity,
            dt,
            maximum_radius,
            maximum_angular_expansion,
            maximum_allowed_expansion,
            &mut min_expansion,
            &mut max_expansion,
        );
        *min += min_expansion;
        *max += max_expansion;
    }

    /// Expands bounding box by an expansion vector (SIMD).
    #[inline(always)]
    pub fn expand_bounding_box_by_expansion_wide(
        expansion: &Vector3Wide,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        let mut min_expansion = Vector3Wide::default();
        let mut max_expansion = Vector3Wide::default();
        Vector3Wide::min_scalar(&zero, expansion, &mut min_expansion);
        Vector3Wide::max_scalar(&zero, expansion, &mut max_expansion);
        min.x += min_expansion.x;
        min.y += min_expansion.y;
        min.z += min_expansion.z;
        max.x += max_expansion.x;
        max.y += max_expansion.y;
        max.z += max_expansion.z;
    }

    /// Expands bounding box by an expansion vector (scalar).
    #[inline(always)]
    pub fn expand_bounding_box_by_expansion(
        expansion: Vec3,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        let min_expansion = expansion.min(Vec3::ZERO);
        let max_expansion = expansion.max(Vec3::ZERO);
        *min += min_expansion;
        *max += max_expansion;
    }
}
