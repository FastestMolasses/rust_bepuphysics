use packed_simd::*;
use std::ops::{Add, Mul};

use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3::Symmetric3x3;
use crate::utilities::vector3_wide::Vector3Wide;

#[repr(C)]
pub struct Symmetric3x3Wide {
    pub xx: f32x8,
    pub yx: f32x8,
    pub yy: f32x8,
    pub zx: f32x8,
    pub zy: f32x8,
    pub zz: f32x8,
}

impl Symmetric3x3Wide {
    #[inline(always)]
    pub fn invert(m: &Self, inverse: &mut Self) {
        let xx = m.yy * m.zz - m.zy * m.zy;
        let yx = m.zy * m.zx - m.zz * m.yx;
        let zx = m.yx * m.zy - m.zx * m.yy;
        let determinant_inverse = f32x8::splat(1.0) / (xx * m.xx + yx * m.yx + zx * m.zx);

        let yy = m.zz * m.xx - m.zx * m.zx;
        let zy = m.zx * m.yx - m.xx * m.zy;

        let zz = m.xx * m.yy - m.yx * m.yx;

        inverse.xx = xx * determinant_inverse;
        inverse.yx = yx * determinant_inverse;
        inverse.zx = zx * determinant_inverse;
        inverse.yy = yy * determinant_inverse;
        inverse.zy = zy * determinant_inverse;
        inverse.zz = zz * determinant_inverse;
    }

    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        result.xx = a.xx + b.xx;
        result.yx = a.yx + b.yx;
        result.yy = a.yy + b.yy;
        result.zx = a.zx + b.zx;
        result.zy = a.zy + b.zy;
        result.zz = a.zz + b.zz;
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        result.xx = a.xx - b.xx;
        result.yx = a.yx - b.yx;
        result.yy = a.yy - b.yy;
        result.zx = a.zx - b.zx;
        result.zy = a.zy - b.zy;
        result.zz = a.zz - b.zz;
    }

    #[inline(always)]
    pub fn scale(m: &Self, scale: f32x8, result: &mut Self) {
        result.xx = m.xx * scale;
        result.yx = m.yx * scale;
        result.yy = m.yy * scale;
        result.zx = m.zx * scale;
        result.zy = m.zy * scale;
        result.zz = m.zz * scale;
    }

    #[inline(always)]
    pub fn skew_sandwich_without_overlap(v: &Vector3Wide, m: &Self, sandwich: &mut Self) {
        let xzy = v.x * m.zy;
        let yzx = v.y * m.zx;
        let zyx = v.z * m.yx;
        let ixx = yzx - zyx;
        let ixy = v.y * m.zy - v.z * m.yy;
        let ixz = v.y * m.zz - v.z * m.zy;
        let iyx = v.z * m.xx - v.x * m.zx;
        let iyy = zyx - xzy;
        let iyz = v.z * m.zx - v.x * m.zz;
        let izx = v.x * m.yx - v.y * m.xx;
        let izy = v.x * m.yy - v.y * m.yx;
        let izz = xzy - yzx;

        sandwich.xx = v.y * ixz - v.z * ixy;
        sandwich.yx = v.y * iyz - v.z * iyy;
        sandwich.yy = v.z * iyx - v.x * iyz;
        sandwich.zx = v.y * izz - v.z * izy;
        sandwich.zy = v.z * izx - v.x * izz;
        sandwich.zz = v.x * izy - v.y * izx;
    }

    #[inline(always)]
    pub fn vector_sandwich(v: &Vector3Wide, m: &Self, sandwich: &mut f32x8) {
        let x = v.x * m.xx + v.y * m.yx + v.z * m.zx;
        let y = v.x * m.yx + v.y * m.yy + v.z * m.zy;
        let z = v.x * m.zx + v.y * m.zy + v.z * m.zz;
        *sandwich = x * v.x + y * v.y + z * v.z;
    }

    #[inline(always)]
    pub fn rotation_sandwich(r: &Matrix3x3Wide, m: &Self, sandwich: &mut Self) {
        let ixx = r.x.x * m.xx + r.y.x * m.yx + r.z.x * m.zx;
        let ixy = r.x.x * m.yx + r.y.x * m.yy + r.z.x * m.zy;
        let ixz = r.x.x * m.zx + r.y.x * m.zy + r.z.x * m.zz;

        let iyx = r.x.y * m.xx + r.y.y * m.yx + r.z.y * m.zx;
        let iyy = r.x.y * m.yx + r.y.y * m.yy + r.z.y * m.zy;
        let iyz = r.x.y * m.zx + r.y.y * m.zy + r.z.y * m.zz;

        let izx = r.x.z * m.xx + r.y.z * m.yx + r.z.z * m.zx;
        let izy = r.x.z * m.yx + r.y.z * m.yy + r.z.z * m.zy;
        let izz = r.x.z * m.zx + r.y.z * m.zy + r.z.z * m.zz;

        sandwich.xx = ixx * r.x.x + ixy * r.y.x + ixz * r.z.x;
        sandwich.yx = iyx * r.x.x + iyy * r.y.x + iyz * r.z.x;
        sandwich.yy = iyx * r.x.y + iyy * r.y.y + iyz * r.z.y;
        sandwich.zx = izx * r.x.x + izy * r.y.x + izz * r.z.x;
        sandwich.zy = izx * r.x.y + izy * r.y.y + izz * r.z.y;
        sandwich.zz = izx * r.x.z + izy * r.y.z + izz * r.z.z;
    }

    #[inline(always)]
    pub fn multiply_without_overlap(a: &Matrix2x3Wide, b: &Self, result: &mut Matrix2x3Wide) {
        result.x.x = a.x.x * b.xx + a.x.y * b.yx + a.x.z * b.zx;
        result.x.y = a.x.x * b.yx + a.x.y * b.yy + a.x.z * b.zy;
        result.x.z = a.x.x * b.zx + a.x.y * b.zy + a.x.z * b.zz;
        result.y.x = a.y.x * b.xx + a.y.y * b.yx + a.y.z * b.zx;
        result.y.y = a.y.x * b.yx + a.y.y * b.yy + a.y.z * b.zy;
        result.y.z = a.y.x * b.zx + a.y.y * b.zy + a.y.z * b.zz;
    }

    #[inline(always)]
    pub fn multiply_without_overlap_3x3(a: &Matrix3x3Wide, b: &Self, result: &mut Matrix3x3Wide) {
        result.x.x = a.x.x * b.xx + a.x.y * b.yx + a.x.z * b.zx;
        result.x.y = a.x.x * b.yx + a.x.y * b.yy + a.x.z * b.zy;
        result.x.z = a.x.x * b.zx + a.x.y * b.zy + a.x.z * b.zz;

        result.y.x = a.y.x * b.xx + a.y.y * b.yx + a.y.z * b.zx;
        result.y.y = a.y.x * b.yx + a.y.y * b.yy + a.y.z * b.zy;
        result.y.z = a.y.x * b.zx + a.y.y * b.zy + a.y.z * b.zz;

        result.z.x = a.z.x * b.xx + a.z.y * b.yx + a.z.z * b.zx;
        result.z.y = a.z.x * b.yx + a.z.y * b.yy + a.z.z * b.zy;
        result.z.z = a.z.x * b.zx + a.z.y * b.zy + a.z.z * b.zz;
    }

    #[inline(always)]
    pub fn multiply(a: &Self, b: &Matrix3x3Wide, result: &mut Matrix3x3Wide) {
        result.x.x = a.xx * b.x.x + a.yx * b.y.x + a.zx * b.z.x;
        result.x.y = a.xx * b.x.y + a.yx * b.y.y + a.zx * b.z.y;
        result.x.z = a.xx * b.x.z + a.yx * b.y.z + a.zx * b.z.z;

        result.y.x = a.yx * b.x.x + a.yy * b.y.x + a.zy * b.z.x;
        result.y.y = a.yx * b.x.y + a.yy * b.y.y + a.zy * b.z.y;
        result.y.z = a.yx * b.x.z + a.yy * b.y.z + a.zy * b.z.z;

        result.z.x = a.zx * b.x.x + a.zy * b.y.x + a.zz * b.z.x;
        result.z.y = a.zx * b.x.y + a.zy * b.y.y + a.zz * b.z.y;
        result.z.z = a.zx * b.x.z + a.zy * b.y.z + a.zz * b.z.z;
    }

    #[inline(always)]
    pub fn multiply_by_transposed(a: &Self, b: &Matrix3x3Wide, result: &mut Matrix3x3Wide) {
        result.x.x = a.xx * b.x.x + a.yx * b.x.y + a.zx * b.x.z;
        result.x.y = a.xx * b.y.x + a.yx * b.y.y + a.zx * b.y.z;
        result.x.z = a.xx * b.z.x + a.yx * b.z.y + a.zx * b.z.z;

        result.y.x = a.yx * b.x.x + a.yy * b.x.y + a.zy * b.x.z;
        result.y.y = a.yx * b.y.x + a.yy * b.y.y + a.zy * b.y.z;
        result.y.z = a.yx * b.z.x + a.yy * b.z.y + a.zy * b.z.z;

        result.z.x = a.zx * b.x.x + a.zy * b.x.y + a.zz * b.x.z;
        result.z.y = a.zx * b.y.x + a.zy * b.y.y + a.zz * b.y.z;
        result.z.z = a.zx * b.z.x + a.zy * b.z.y + a.zz * b.z.z;
    }

    #[inline(always)]
    pub fn multiply_by_transposed_2x3(a: &Self, b: &Matrix2x3Wide, result: &mut Matrix2x3Wide) {
        result.x.x = a.xx * b.x.x + a.yx * b.x.y + a.zx * b.x.z;
        result.y.x = a.xx * b.y.x + a.yx * b.y.y + a.zx * b.y.z;

        result.x.y = a.yx * b.x.x + a.yy * b.x.y + a.zy * b.x.z;
        result.y.y = a.yx * b.y.x + a.yy * b.y.y + a.zy * b.y.z;

        result.x.z = a.zx * b.x.x + a.zy * b.x.y + a.zz * b.x.z;
        result.y.z = a.zx * b.y.x + a.zy * b.y.y + a.zz * b.y.z;
    }

    #[inline(always)]
    pub fn matrix_sandwich(m: &Matrix2x3Wide, t: &Self, result: &mut Symmetric2x2Wide) {
        let ixx = m.x.x * t.xx + m.x.y * t.yx + m.x.z * t.zx;
        let ixy = m.x.x * t.yx + m.x.y * t.yy + m.x.z * t.zy;
        let ixz = m.x.x * t.zx + m.x.y * t.zy + m.x.z * t.zz;
        let iyx = m.y.x * t.xx + m.y.y * t.yx + m.y.z * t.zx;
        let iyy = m.y.x * t.yx + m.y.y * t.yy + m.y.z * t.zy;
        let iyz = m.y.x * t.zx + m.y.y * t.zy + m.y.z * t.zz;
        result.xx = ixx * m.x.x + ixy * m.x.y + ixz * m.x.z;
        result.yx = iyx * m.x.x + iyy * m.x.y + iyz * m.x.z;
        result.yy = iyx * m.y.x + iyy * m.y.y + iyz * m.y.z;
    }

    #[inline(always)]
    pub fn complete_matrix_sandwich(a: &Matrix3x3Wide, b: &Matrix3x3Wide, result: &mut Self) {
        result.xx = a.x.x * b.x.x + a.x.y * b.y.x + a.x.z * b.z.x;

        result.yx = a.y.x * b.x.x + a.y.y * b.y.x + a.y.z * b.z.x;
        result.yy = a.y.x * b.x.y + a.y.y * b.y.y + a.y.z * b.z.y;

        result.zx = a.z.x * b.x.x + a.z.y * b.y.x + a.z.z * b.z.x;
        result.zy = a.z.x * b.x.y + a.z.y * b.y.y + a.z.z * b.z.y;
        result.zz = a.z.x * b.x.z + a.z.y * b.y.z + a.z.z * b.z.z;
    }

    #[inline(always)]
    pub fn complete_matrix_sandwich_2x3(a: &Matrix2x3Wide, b: &Matrix2x3Wide, result: &mut Self) {
        result.xx = a.x.x * b.x.x + a.y.x * b.y.x;

        result.yx = a.x.y * b.x.x + a.y.y * b.y.x;
        result.yy = a.x.y * b.x.y + a.y.y * b.y.y;

        result.zx = a.x.z * b.x.x + a.y.z * b.y.x;
        result.zy = a.x.z * b.x.y + a.y.z * b.y.y;
        result.zz = a.x.z * b.x.z + a.y.z * b.y.z;
    }

    #[inline(always)]
    pub fn complete_matrix_sandwich_by_transpose(
        a: &Matrix3x3Wide,
        b: &Matrix3x3Wide,
        result: &mut Self,
    ) {
        result.xx = a.x.x * b.x.x + a.x.y * b.x.y + a.x.z * b.x.z;

        result.yx = a.y.x * b.x.x + a.y.y * b.x.y + a.y.z * b.x.z;
        result.yy = a.y.x * b.y.x + a.y.y * b.y.y + a.y.z * b.y.z;

        result.zx = a.z.x * b.x.x + a.z.y * b.x.y + a.z.z * b.x.z;
        result.zy = a.z.x * b.y.x + a.z.y * b.y.y + a.z.z * b.y.z;
        result.zz = a.z.x * b.z.x + a.z.y * b.z.y + a.z.z * b.z.z;
    }

    #[inline(always)]
    pub fn complete_matrix_sandwich_transpose(
        a: &Matrix3x3Wide,
        b: &Matrix3x3Wide,
        result: &mut Self,
    ) {
        result.xx = a.x.x * b.x.x + a.y.x * b.y.x + a.z.x * b.z.x;

        result.yx = a.x.y * b.x.x + a.y.y * b.y.x + a.z.y * b.z.x;
        result.yy = a.x.y * b.x.y + a.y.y * b.y.y + a.z.y * b.z.y;

        result.zx = a.x.z * b.x.x + a.y.z * b.y.x + a.z.z * b.z.x;
        result.zy = a.x.z * b.x.y + a.y.z * b.y.y + a.z.z * b.z.y;
        result.zz = a.x.z * b.x.z + a.y.z * b.y.z + a.z.z * b.z.z;
    }

    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector3Wide, m: &Self, result: &mut Vector3Wide) {
        result.x = v.x * m.xx + v.y * m.yx + v.z * m.zx;
        result.y = v.x * m.yx + v.y * m.yy + v.z * m.zy;
        result.z = v.x * m.zx + v.y * m.zy + v.z * m.zz;
    }

    #[inline(always)]
    pub fn write_first(scalar: &Symmetric3x3, wide: &mut Self) {
        wide.xx[0] = scalar.xx;
        wide.yx[0] = scalar.yx;
        wide.yy[0] = scalar.yy;
        wide.zx[0] = scalar.zx;
        wide.zy[0] = scalar.zy;
        wide.zz[0] = scalar.zz;
    }
}

impl Add<Matrix3x3Wide> for Symmetric3x3Wide {
    type Output = Matrix3x3Wide;

    #[inline(always)]
    fn add(self, rhs: Matrix3x3Wide) -> Self::Output {
        Matrix3x3Wide {
            x: Vector3Wide {
                x: self.xx + rhs.x.x,
                y: self.yx + rhs.x.y,
                z: self.zx + rhs.x.z,
            },
            y: Vector3Wide {
                x: self.yx + rhs.y.x,
                y: self.yy + rhs.y.y,
                z: self.zy + rhs.y.z,
            },
            z: Vector3Wide {
                x: self.zx + rhs.z.x,
                y: self.zy + rhs.z.y,
                z: self.zz + rhs.z.z,
            },
        }
    }
}

impl Add<Symmetric3x3Wide> for Matrix3x3Wide {
    type Output = Matrix3x3Wide;

    #[inline(always)]
    fn add(self, rhs: Symmetric3x3Wide) -> Self::Output {
        Matrix3x3Wide {
            x: Vector3Wide {
                x: self.x.x + rhs.xx,
                y: self.x.y + rhs.yx,
                z: self.x.z + rhs.zx,
            },
            y: Vector3Wide {
                x: self.y.x + rhs.yx,
                y: self.y.y + rhs.yy,
                z: self.y.z + rhs.zy,
            },
            z: Vector3Wide {
                x: self.z.x + rhs.zx,
                y: self.z.y + rhs.zy,
                z: self.z.z + rhs.zz,
            },
        }
    }
}

impl Mul<Symmetric3x3Wide> for Vector3Wide {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, rhs: Symmetric3x3Wide) -> Self::Output {
        Vector3Wide {
            x: self.x * rhs.xx + self.y * rhs.yx + self.z * rhs.zx,
            y: self.x * rhs.yx + self.y * rhs.yy + self.z * rhs.zy,
            z: self.x * rhs.zx + self.y * rhs.zy + self.z * rhs.zz,
        }
    }
}

impl Mul<f32x8> for Symmetric3x3Wide {
    type Output = Symmetric3x3Wide;

    #[inline(always)]
    fn mul(self, rhs: f32x8) -> Self::Output {
        Symmetric3x3Wide {
            xx: self.xx * rhs,
            yx: self.yx * rhs,
            yy: self.yy * rhs,
            zx: self.zx * rhs,
            zy: self.zy * rhs,
            zz: self.zz * rhs,
        }
    }
}

impl Mul<Symmetric3x3Wide> for Matrix2x3Wide {
    type Output = Matrix2x3Wide;

    #[inline(always)]
    fn mul(self, rhs: Symmetric3x3Wide) -> Self::Output {
        let mut result = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap(&self, &rhs, &mut result);
        result
    }
}

impl Mul<Symmetric3x3Wide> for Matrix3x3Wide {
    type Output = Matrix3x3Wide;

    #[inline(always)]
    fn mul(self, rhs: Symmetric3x3Wide) -> Self::Output {
        let mut result = Matrix3x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_3x3(&self, &rhs, &mut result);
        result
    }
}

impl Mul<Matrix3x3Wide> for Symmetric3x3Wide {
    type Output = Matrix3x3Wide;

    #[inline(always)]
    fn mul(self, rhs: Matrix3x3Wide) -> Self::Output {
        let mut result = Matrix3x3Wide::default();
        Symmetric3x3Wide::multiply(&self, &rhs, &mut result);
        result
    }
}
