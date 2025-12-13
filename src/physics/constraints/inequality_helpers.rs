// Translated from BepuPhysics/Constraints/InequalityHelpers.cs

use crate::utilities::vector::Vector;
use std::simd::num::SimdFloat;

/// Helpers for inequality constraint clamping.
pub struct InequalityHelpers;

impl InequalityHelpers {
    #[inline(always)]
    pub fn compute_bias_velocity(
        error: Vector<f32>,
        position_error_to_velocity: &Vector<f32>,
        inverse_dt: f32,
        bias_velocity: &mut Vector<f32>,
    ) {
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        *bias_velocity = (error * inverse_dt_wide).simd_min(error * *position_error_to_velocity);
    }

    #[inline(always)]
    pub fn clamp_positive(accumulated_impulse: &mut Vector<f32>, impulse: &mut Vector<f32>) {
        let previous = *accumulated_impulse;
        let zero = Vector::<f32>::splat(0.0);
        *accumulated_impulse = zero.simd_max(*accumulated_impulse + *impulse);
        *impulse = *accumulated_impulse - previous;
    }
}
