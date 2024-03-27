#![allow(unsafe_code)]

use crate::utilities::vector3::Vector3;
use packed_simd::{f32x4, m32x4};
use std::ops::{Add, Mul, Sub};

#[derive(Copy, Clone, Debug)]
pub struct Matrix3x3 {
    x: f32x4,
    y: f32x4,
    z: f32x4,
}

impl Matrix3x3 {
    /// Returns the identity matrix.
    #[inline(always)]
    pub fn identity() -> Self {
        Self {
            x: f32x4::new(1.0, 0.0, 0.0, 0.0),
            y: f32x4::new(0.0, 1.0, 0.0, 0.0),
            z: f32x4::new(0.0, 0.0, 1.0, 0.0),
        }
    }

    /// Adds two matrices.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
        }
    }

    /// Scales the matrix by a scalar.
    #[inline(always)]
    pub fn scale(matrix: &Self, scale: f32) -> Self {
        Self {
            x: matrix.x * scale,
            y: matrix.y * scale,
            z: matrix.z * scale,
        }
    }

    /// Subtracts matrix b from matrix a.
    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x - b.x,
            y: a.y - b.y,
            z: a.z - b.z,
        }
    }

    /// Transposes a matrix using SIMD operations.
    #[inline(always)]
    pub unsafe fn transpose(m: &Self) -> Self {
        let tmp0 = m.x.extract(1);
        let tmp1 = m.x.extract(2);
        let tmp2 = m.y.extract(2);

        Self {
            x: f32x4::new(m.x.extract(0), m.y.extract(0), m.z.extract(0), 0.0),
            y: f32x4::new(tmp0, m.y.extract(1), m.z.extract(1), 0.0),
            z: f32x4::new(tmp1, tmp2, m.z.extract(2), 0.0),
        }
    }

    /// Computes the determinant of the matrix.
    #[inline(always)]
    pub fn determinant(&self) -> f32 {
        let y_cross_z = self.y.cross(self.z);
        self.x.dot(y_cross_z)
    }

    /// Inverts the matrix.
    #[inline(always)]
    pub fn invert(m: &Self) -> Self {
        let yz = m.y.cross(m.z);
        let zx = m.z.cross(m.x);
        let xy = m.x.cross(m.y);
        let inverse_determinant = 1.0 / m.x.dot(yz);
        let mut inverse = Self {
            x: yz * inverse_determinant,
            y: zx * inverse_determinant,
            z: xy * inverse_determinant,
        };
        unsafe {
            inverse = Self::transpose(&inverse);
        }
        inverse
    }

    /// Transforms a vector by the matrix.
    #[inline(always)]
    pub fn transform(v: &Vector3, m: &Self) -> Vector3 {
        let x = Vector3::new(v.x, v.x, v.x);
        let y = Vector3::new(v.y, v.y, v.y);
        let z = Vector3::new(v.z, v.z, v.z);
        (m.x * x) + (m.y * y) + (m.z * z)
    }

    /// Transforms a vector by the transposed matrix.
    #[inline(always)]
    pub fn transform_transpose(v: &Vector3, m: &Self) -> Vector3 {
        Vector3::new(v.dot(&m.x), v.dot(&m.y), v.dot(&m.z))
    }
}

impl Add for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        // Matrix3x3::add(&self, &other)
    }
}

impl Sub for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        // Matrix3x3::subtract(&self, &other)
    }
}

impl Mul<f32> for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f32) -> Self::Output {
        // Matrix3x3::scale(&self, scalar)
    }
}
