use crate::utilities::matrix::Matrix;
use glam::{Quat, Vec3};
use std::ops::{Add, Mul, Sub};

/// 3 row, 3 column matrix.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(C, align(16))]
pub struct Matrix3x3 {
    /// First row of the matrix.
    pub x: Vec3,
    /// Second row of the matrix.
    pub y: Vec3,
    /// Third row of the matrix.
    pub z: Vec3,
}

/// A representation of a 3x3 matrix that will be mapped linearly in memory.
#[repr(C)]
struct M {
    m11: f32,
    m12: f32,
    m13: f32,
    m21: f32,
    m22: f32,
    m23: f32,
    m31: f32,
    m32: f32,
    m33: f32,
}

impl Matrix3x3 {
    /// Gets the 3x3 identity matrix.
    #[inline(always)]
    pub const fn identity() -> Self {
        Self {
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        }
    }

    /// Adds the components of two matrices together.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
    }

    /// Scales the components of a matrix by a scalar.
    #[inline(always)]
    pub fn scale(matrix: &Self, scale: f32, result: &mut Self) {
        result.x = matrix.x * scale;
        result.y = matrix.y * scale;
        result.z = matrix.z * scale;
    }

    /// Subtracts the components of one matrix from another.
    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x - b.x;
        result.y = a.y - b.y;
        result.z = a.z - b.z;
    }

    #[inline(always)]
    unsafe fn transpose_m(m: *const M, transposed: *mut M) {
        // From original code:
        // A weird function! Why?
        // 1) Missing some helpful instructions for actual SIMD accelerated transposition.
        // 2) Difficult to get SIMD types to generate competitive codegen due to lots of componentwise access.

        let m = &*m;
        let transposed = &mut *transposed;

        let m12 = m.m12;
        let m13 = m.m13;
        let m23 = m.m23;
        transposed.m11 = m.m11;
        transposed.m12 = m.m21;
        transposed.m13 = m.m31;

        transposed.m21 = m12;
        transposed.m22 = m.m22;
        transposed.m23 = m.m32;

        transposed.m31 = m13;
        transposed.m32 = m23;
        transposed.m33 = m.m33;
    }

    #[inline(always)]
    pub unsafe fn transpose(m: *const Self, transposed: *mut Self) {
        Self::transpose_m(m as *const M, transposed as *mut M);
    }

    /// Computes the transposed matrix of a matrix.
    #[inline(always)]
    pub fn transpose_ref(m: &Self, transposed: &mut Self) {
        let xy = m.x.y;
        let xz = m.x.z;
        let yz = m.y.z;
        transposed.x = Vec3::new(m.x.x, m.y.x, m.z.x);
        transposed.y = Vec3::new(xy, m.y.y, m.z.y);
        transposed.z = Vec3::new(xz, yz, m.z.z);
    }

    /// Calculates the determinant of the matrix.
    #[inline(always)]
    pub fn determinant(&self) -> f32 {
        self.x.dot(self.y.cross(self.z))
    }

    /// Inverts the given matix.
    #[inline(always)]
    pub fn invert(m: &Self, inverse: &mut Self) -> Self {
        let yz = self.y.cross(self.z);
        let zx = self.z.cross(self.x);
        let xy = self.x.cross(self.y);
        let inverse_determinant = 1.0 / self.x.dot(yz);
        inverse.x = yz * inverse_determinant;
        inverse.y = zx * inverse_determinant;
        inverse.z = xy * inverse_determinant;
        unsafe {
            Matrix3x3::transpose(&inverse, inverse)
        }
    }

    /// Inverts the given matix.
    #[inline(always)]
    pub unsafe fn invert(m: *const Self, inverse: *mut Self) {
        // TODO: CHECK IF THIS IS ACTUALLY MODIFYING THE INVERSE VARIABLE
        let m_scalar = m as *const M;
        let inverse_scalar = inverse as *mut M;

        let m11 = (*m_scalar).m22 * (*m_scalar).m33 - (*m_scalar).m32 * (*m_scalar).m23;
        let m21 = (*m_scalar).m23 * (*m_scalar).m31 - (*m_scalar).m33 * (*m_scalar).m21;
        let m31 = (*m_scalar).m21 * (*m_scalar).m32 - (*m_scalar).m31 * (*m_scalar).m22;
        let determinant_inverse =
            1.0 / (m11 * (*m_scalar).m11 + m21 * (*m_scalar).m12 + m31 * (*m_scalar).m13);

        let m12 = (*m_scalar).m32 * (*m_scalar).m13 - (*m_scalar).m12 * (*m_scalar).m33;
        let m22 = (*m_scalar).m33 * (*m_scalar).m11 - (*m_scalar).m13 * (*m_scalar).m31;
        let m32 = (*m_scalar).m31 * (*m_scalar).m12 - (*m_scalar).m11 * (*m_scalar).m32;

        let m13 = (*m_scalar).m12 * (*m_scalar).m23 - (*m_scalar).m22 * (*m_scalar).m13;
        let m23 = (*m_scalar).m13 * (*m_scalar).m21 - (*m_scalar).m23 * (*m_scalar).m11;
        let m33 = (*m_scalar).m11 * (*m_scalar).m22 - (*m_scalar).m21 * (*m_scalar).m12;

        (*inverse_scalar).m11 = m11 * determinant_inverse;
        (*inverse_scalar).m21 = m21 * determinant_inverse;
        (*inverse_scalar).m31 = m31 * determinant_inverse;

        (*inverse_scalar).m12 = m12 * determinant_inverse;
        (*inverse_scalar).m22 = m22 * determinant_inverse;
        (*inverse_scalar).m32 = m32 * determinant_inverse;

        (*inverse_scalar).m13 = m13 * determinant_inverse;
        (*inverse_scalar).m23 = m23 * determinant_inverse;
        (*inverse_scalar).m33 = m33 * determinant_inverse;
    }

    /// Transforms the vector by the matrix.
    #[inline(always)]
    pub fn transform(v: &Vec3, m: &Self, result: &mut Vec3) {
        let x = Vec3::splat(v.x);
        let y = Vec3::splat(v.y);
        let z = Vec3::splat(v.z);
        *result = m.x * x + m.y * y + m.z * z;
    }

    /// Transforms the vector by the matrix's transpose.
    #[inline(always)]
    pub fn transform_transpose(v: &Vec3, m: &Self, result: &mut Vec3) {
        *result = Vec3::new(v.dot(m.x), v.dot(m.y), v.dot(m.z));
    }

    /// Multiplies the two matrices.
    #[inline(always)]
    pub fn multiply(a: &Self, b: &Self, result: &mut Self) {
        let b_x = b.x;
        let b_y = b.y;

        {
            let x = Vec3::splat(a.x.x);
            let y = Vec3::splat(a.x.y);
            let z = Vec3::splat(a.x.z);
            result.x = x * b_x + y * b_y + z * b.z;
        }

        {
            let x = Vec3::splat(a.y.x);
            let y = Vec3::splat(a.y.y);
            let z = Vec3::splat(a.y.z);
            result.y = x * b_x + y * b_y + z * b.z;
        }

        {
            let x = Vec3::splat(a.z.x);
            let y = Vec3::splat(a.z.y);
            let z = Vec3::splat(a.z.z);
            result.z = x * b_x + y * b_y + z * b.z;
        }
    }

    /// Multiplies the two matrices, where a is treated as transposed: result = transpose(a) * b
    #[inline(always)]
    pub fn multiply_transposed(a: &Self, b: &Self, result: &mut Self) {
        let b_x = b.x;
        let b_y = b.y;
        {
            let x = Vec3::splat(a.x.x);
            let y = Vec3::splat(a.y.x);
            let z = Vec3::splat(a.z.x);
            result.x = x * b_x + y * b_y + z * b.z;
        }

        {
            let x = Vec3::splat(a.x.y);
            let y = Vec3::splat(a.y.y);
            let z = Vec3::splat(a.z.y);
            result.y = x * b_x + y * b_y + z * b.z;
        }

        {
            let x = Vec3::splat(a.x.z);
            let y = Vec3::splat(a.y.z);
            let z = Vec3::splat(a.z.z);
            result.z = x * b_x + y * b_y + z * b.z;
        }
    }

    #[inline(always)]
    pub fn create_from_matrix(matrix: &Matrix, matrix3x3: &mut Self) {
        matrix3x3.x = Vec3::new(matrix.x.x, matrix.x.y, matrix.x.z);
        matrix3x3.y = Vec3::new(matrix.y.x, matrix.y.y, matrix.y.z);
        matrix3x3.z = Vec3::new(matrix.z.x, matrix.z.y, matrix.z.z);
    }

    #[inline(always)]
    pub fn create_from_quaternion(q: &Quat, result: &mut Self) {
        let qx2 = q.x + q.x;
        let qy2 = q.y + q.y;
        let qz2 = q.z + q.z;
        let xx = qx2 * q.x;
        let yy = qy2 * q.y;
        let zz = qz2 * q.z;
        let xy = qx2 * q.y;
        let xz = qx2 * q.z;
        let xw = qx2 * q.w;
        let yz = qy2 * q.z;
        let yw = qy2 * q.w;
        let zw = qz2 * q.w;

        result.x = Vec3::new(1.0 - yy - zz, xy + zw, xz - yw);
        result.y = Vec3::new(xy - zw, 1.0 - xx - zz, yz + xw);
        result.z = Vec3::new(xz + yw, yz - xw, 1.0 - xx - yy);
    }

    #[inline(always)]
    pub fn create_value_from_quaternion(q: &Quat) -> Self {
        let result;
        Matrix3x3::create_from_quaternion(q, result);
        return result;
    }

    /// Creates a 3x3 matrix representing the given scale along its local axes.
    #[inline(always)]
    pub fn create_scale(scale: &Vec3, linear_transform: &mut Self) {
        linear_transform.x = Vec3::new(scale.x, 0.0, 0.0);
        linear_transform.y = Vec3::new(0.0, scale.y, 0.0);
        linear_transform.z = Vec3::new(0.0, 0.0, scale.z);
    }

    /// Creates a 3x3 matrix representing the given scale along its local axes.
    #[inline(always)]
    pub fn create_scale_value(scale: &Vec3) -> Self {
        let result;
        Matrix3x3::create_scale(scale, result);
        return result;
    }

    /// Creates a matrix representing a rotation derived from an axis and angle.
    #[inline(always)]
    pub fn create_from_axis_angle(axis: &Vec3, angle: f32, result: &mut Self) {
        let xx = axis.x * axis.x;
        let yy = axis.y * axis.y;
        let zz = axis.z * axis.z;
        let xy = axis.x * axis.y;
        let xz = axis.x * axis.z;
        let yz = axis.y * axis.z;

        let sin_angle = angle.sin();
        let one_minus_cos_angle = 1.0 - angle.cos();

        result.x = Vec3::new(
            1.0 + one_minus_cos_angle * (xx - 1.0),
            axis.z * sin_angle + one_minus_cos_angle * xy,
            -axis.y * sin_angle + one_minus_cos_angle * xz,
        );
        result.y = Vec3::new(
            -axis.z * sin_angle + one_minus_cos_angle * xy,
            1.0 + one_minus_cos_angle * (yy - 1.0),
            axis.x * sin_angle + one_minus_cos_angle * yz,
        );
        result.z = Vec3::new(
            axis.y * sin_angle + one_minus_cos_angle * xz,
            -axis.x * sin_angle + one_minus_cos_angle * yz,
            1.0 + one_minus_cos_angle * (zz - 1.0),
        );
    }

    #[inline(always)]
    pub fn create_value_from_axis_angle(axis: &Vec3, angle: f32) -> Self {
        let result;
        Matrix3x3::create_from_axis_angle(axis, angle, result);
        return result;
    }

    /// Creates a matrix such that a x v = a * result. Returns
    /// the skew symmetric matrix representing the cross product.
    #[inline(always)]
    pub fn create_cross_product(v: &Vec3, result: &mut Self) {
        result.x.x = 0.0;
        result.x.y = -v.z;
        result.x.z = v.y;
        result.y.x = v.z;
        result.y.y = 0.0;
        result.y.z = -v.x;
        result.z.x = -v.y;
        result.z.y = v.x;
        result.z.z = 0.0;
    }
}

impl Add for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        let result;
        Self::add(&self, &other, result);
        return result;
    }
}

impl Sub for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        let result;
        Self::subtract(&self, &other, result);
        return result;
    }
}

impl Mul for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        let result;
        Self::multiply(&self, &other, result);
        return result;
    }
}
