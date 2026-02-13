use glam::Vec3;
use std::simd::prelude::*;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Shared miscellaneous helper functions.
pub struct Helpers;

impl Helpers {
    #[inline(always)]
    pub fn swap<T>(a: &mut T, b: &mut T) {
        std::mem::swap(a, b);
    }

    /// Builds an orthonormal basis from a normal vector (wide/SIMD version).
    #[inline(always)]
    pub fn build_orthonormal_basis(
        normal: &Vector3Wide,
        t1: &mut Vector3Wide,
        t2: &mut Vector3Wide,
    ) {
        // This could probably be improved.
        let neg_one = Vector::<f32>::splat(-1.0);
        let one = Vector::<f32>::splat(1.0);
        let zero = Vector::<f32>::splat(0.0);

        let sign = normal.z.simd_lt(zero).select(neg_one, one);

        // This has a discontinuity at z==0. Raw frisvad has only one discontinuity,
        // though that region is more unpredictable than the revised version.
        let scale = neg_one / (sign + normal.z);
        t1.x = normal.x * normal.y * scale;
        t1.y = sign + normal.y * normal.y * scale;
        t1.z = -normal.y;

        t2.x = one + sign * normal.x * normal.x * scale;
        t2.y = sign * t1.x;
        t2.z = -sign * normal.x;
    }

    /// Finds a vector perpendicular to the given normal (wide/SIMD version).
    #[inline(always)]
    pub fn find_perpendicular(normal: &Vector3Wide, perpendicular: &mut Vector3Wide) {
        let neg_one = Vector::<f32>::splat(-1.0);
        let one = Vector::<f32>::splat(1.0);
        let zero = Vector::<f32>::splat(0.0);

        let sign = normal.z.simd_lt(zero).select(neg_one, one);

        let scale = neg_one / (sign + normal.z);
        perpendicular.x = normal.x * normal.y * scale;
        perpendicular.y = sign + normal.y * normal.y * scale;
        perpendicular.z = -normal.y;
    }

    /// Builds an orthonormal basis from a normal vector (scalar version).
    #[inline(always)]
    pub fn build_orthonormal_basis_scalar(normal: Vec3, t1: &mut Vec3, t2: &mut Vec3) {
        let sign = if normal.z < 0.0 { -1.0f32 } else { 1.0f32 };

        let scale = -1.0 / (sign + normal.z);
        *t1 = Vec3::new(
            normal.x * normal.y * scale,
            sign + normal.y * normal.y * scale,
            -normal.y,
        );

        *t2 = Vec3::new(
            1.0 + sign * normal.x * normal.x * scale,
            sign * t1.x,
            -sign * normal.x,
        );
    }

    /// Fills a SIMD vector with lane indices (0, 1, 2, ...).
    #[inline(always)]
    pub fn fill_vector_with_lane_indices(indices: &mut Vector<i32>) {
        let mut values = [0i32; Vector::<i32>::LEN];
        for i in 0..Vector::<i32>::LEN {
            values[i] = i as i32;
        }
        *indices = Vector::<i32>::from_array(values);
    }
}
