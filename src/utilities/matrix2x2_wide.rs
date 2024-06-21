use crate::utilities::vector2_wide::Vector2Wide;
use packed_simd::*;

#[derive(Clone, Copy)]
pub struct Matrix2x2Wide {
    /// First row of the matrix.
    pub x: Vector2Wide,
    /// Second row of the matrix.
    pub y: Vector2Wide,
}

impl Matrix2x2Wide {
    /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
    #[inline(always)]
    pub fn multiply_by_transpose_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.x.y;
        result.x.y = a.x.x * b.y.x + a.x.y * b.y.y;
        result.y.x = a.y.x * b.x.x + a.y.y * b.x.y;
        result.y.y = a.y.x * b.y.x + a.y.y * b.y.y;
    }

    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector2Wide, m: &Self, result: &mut Vector2Wide) {
        result.x = v.x * m.x.x + v.y * m.y.x;
        result.y = v.x * m.x.y + v.y * m.y.y;
    }

    #[inline(always)]
    pub fn transform(v: &Vector2Wide, m: &Self, result: &mut Vector2Wide) {
        let mut temp = Vector2Wide::default();
        Self::transform_without_overlap(v, m, &mut temp);
        *result = temp;
    }

    /// Multiplies every component in the matrix by the given scalar value.
    #[inline(always)]
    pub fn scale(m: &Self, scale: f32x4, result: &mut Self) {
        result.x.x = m.x.x * scale;
        result.x.y = m.x.y * scale;
        result.y.x = m.y.x * scale;
        result.y.y = m.y.y * scale;
    }

    /// Adds the components of one matrix to another.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        Vector2Wide::add(&a.x, &b.x, &mut result.x);
        Vector2Wide::add(&a.y, &b.y, &mut result.y);
    }

    /// Subtracts the components of one matrix from another.
    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        Vector2Wide::subtract(&a.x, &b.x, &mut result.x);
        Vector2Wide::subtract(&a.y, &b.y, &mut result.y);
    }

    /// Inverts the given matrix.
    #[inline(always)]
    pub fn invert_without_overlap(m: &Self, inverse: &mut Self) {
        let determinant_inverse = f32x4::splat(1.0) / (m.x.x * m.y.y - m.x.y * m.y.x);
        inverse.x.x = m.y.y * determinant_inverse;
        inverse.x.y = -m.x.y * determinant_inverse;

        inverse.y.x = -m.y.x * determinant_inverse;
        inverse.y.y = m.x.x * determinant_inverse;
    }
}
