use crate::utilities::{
    matrix2x2_wide::Matrix2x2Wide, matrix3x3_wide::Matrix3x3Wide, vector::Vector,
    vector2_wide::Vector2Wide, vector3_wide::Vector3Wide,
};

#[derive(Clone, Copy, Debug)]
pub struct Matrix2x3Wide {
    /// First row of the matrix.
    pub x: Vector3Wide,
    /// Second row of the matrix.
    pub y: Vector3Wide,
}

impl Matrix2x3Wide {
    #[inline(always)]
    pub fn multiply_without_overlap(a: &Self, b: &Matrix3x3Wide, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.y.x + a.x.z * b.z.x;
        result.x.y = a.x.x * b.x.y + a.x.y * b.y.y + a.x.z * b.z.y;
        result.x.z = a.x.x * b.x.z + a.x.y * b.y.z + a.x.z * b.z.z;
        result.y.x = a.y.x * b.x.x + a.y.y * b.y.x + a.y.z * b.z.x;
        result.y.y = a.y.x * b.x.y + a.y.y * b.y.y + a.y.z * b.z.y;
        result.y.z = a.y.x * b.x.z + a.y.y * b.y.z + a.y.z * b.z.z;
    }

    #[inline(always)]
    pub fn multiply_without_overlap_2x2(a: &Matrix2x2Wide, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.y.x;
        result.x.y = a.x.x * b.x.y + a.x.y * b.y.y;
        result.x.z = a.x.x * b.x.z + a.x.y * b.y.z;
        result.y.x = a.y.x * b.x.x + a.y.y * b.y.x;
        result.y.y = a.y.x * b.x.y + a.y.y * b.y.y;
        result.y.z = a.y.x * b.x.z + a.y.y * b.y.z;
    }

    /// Multiplies a matrix by another matrix, where the first matrix is sampled as if it were transposed: result = transpose(a) * b.
    #[inline(always)]
    pub fn multiply_transposed_without_overlap(a: &Matrix2x2Wide, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.y.x * b.y.x;
        result.x.y = a.x.x * b.x.y + a.y.x * b.y.y;
        result.x.z = a.x.x * b.x.z + a.y.x * b.y.z;
        result.y.x = a.x.y * b.x.x + a.y.y * b.y.x;
        result.y.y = a.x.y * b.x.y + a.y.y * b.y.y;
        result.y.z = a.x.y * b.x.z + a.y.y * b.y.z;
    }

    /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
    #[inline(always)]
    pub fn multiply_by_transpose_without_overlap(a: &Self, b: &Self, result: &mut Matrix2x2Wide) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.x.y + a.x.z * b.x.z;
        result.x.y = a.x.x * b.y.x + a.x.y * b.y.y + a.x.z * b.y.z;
        result.y.x = a.y.x * b.x.x + a.y.y * b.x.y + a.y.z * b.x.z;
        result.y.y = a.y.x * b.y.x + a.y.y * b.y.y + a.y.z * b.y.z;
    }

    #[inline(always)]
    pub fn transform_by_transpose_without_overlap(
        v: &Vector3Wide,
        m: &Self,
        result: &mut Vector2Wide,
    ) {
        result.x = v.x * m.x.x + v.y * m.x.y + v.z * m.x.z;
        result.y = v.x * m.y.x + v.y * m.y.y + v.z * m.y.z;
    }

    #[inline(always)]
    pub fn negate(m: &Self, result: &mut Self) {
        Vector3Wide::negate(&m.x, &mut result.x);
        Vector3Wide::negate(&m.y, &mut result.y);
    }

    /// Multiplies every component in the matrix by the given scalar value.
    #[inline(always)]
    pub fn scale(m: &Self, scale: &Vector<f32>, result: &mut Self) {
        result.x.x = m.x.x * scale;
        result.x.y = m.x.y * scale;
        result.x.z = m.x.z * scale;
        result.y.x = m.y.x * scale;
        result.y.y = m.y.y * scale;
        result.y.z = m.y.z * scale;
    }

    #[inline(always)]
    pub fn transform(v: &Vector2Wide, m: &Self, result: &mut Vector3Wide) {
        result.x = v.x * m.x.x + v.y * m.y.x;
        result.y = v.x * m.x.y + v.y * m.y.y;
        result.z = v.x * m.x.z + v.y * m.y.z;
    }

    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        Vector3Wide::add(&a.x, &b.x, &mut result.x);
        Vector3Wide::add(&a.y, &b.y, &mut result.y);
    }
}
