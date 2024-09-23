use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Mat3;
use std::ops::{Mul, Sub};

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Matrix3x3Wide {
    pub x: Vector3Wide,
    pub y: Vector3Wide,
    pub z: Vector3Wide,
}

impl Matrix3x3Wide {
    #[inline(always)]
    pub fn broadcast(source: &Mat3) -> Self {
        Self {
            x: Vector3Wide::new(
                Vector::splat(source.x_axis.x),
                Vector::splat(source.x_axis.y),
                Vector::splat(source.x_axis.z),
            ),
            y: Vector3Wide::new(
                Vector::splat(source.y.x),
                Vector::splat(source.y.y),
                Vector::splat(source.y.z),
            ),
            z: Vector3Wide::new(
                Vector::splat(source.z.x),
                Vector::splat(source.z.y),
                Vector::splat(source.z.z),
            ),
        }
    }

    #[inline(always)]
    pub fn identity() -> Self {
        Self {
            x: Vector3Wide::new(Vector::splat(1.0), Vector::splat(0.0), Vector::splat(0.0)),
            y: Vector3Wide::new(Vector::splat(0.0), Vector::splat(1.0), Vector::splat(0.0)),
            z: Vector3Wide::new(Vector::splat(0.0), Vector::splat(0.0), Vector::splat(1.0)),
        }
    }

    #[inline(always)]
    pub fn multiply_without_overlap(&self, other: &Self) -> Self {
        Self {
            x: Vector3Wide::new(
                self.x.x * other.x.x + self.x.y * other.y.x + self.x.z * other.z.x,
                self.x.x * other.x.y + self.x.y * other.y.y + self.x.z * other.z.y,
                self.x.x * other.x.z + self.x.y * other.y.z + self.x.z * other.z.z,
            ),
            y: Vector3Wide::new(
                self.y.x * other.x.x + self.y.y * other.y.x + self.y.z * other.z.x,
                self.y.x * other.x.y + self.y.y * other.y.y + self.y.z * other.z.y,
                self.y.x * other.x.z + self.y.y * other.y.z + self.y.z * other.z.z,
            ),
            z: Vector3Wide::new(
                self.z.x * other.x.x + self.z.y * other.y.x + self.z.z * other.z.x,
                self.z.x * other.x.y + self.z.y * other.y.y + self.z.z * other.z.y,
                self.z.x * other.x.z + self.z.y * other.y.z + self.z.z * other.z.z,
            ),
        }
    }

    #[inline(always)]
    pub fn multiply_transposed_without_overlap(&self, other: &Self) -> Self {
        Self {
            x: Vector3Wide::new(
                self.x.x * other.x.x + self.y.x * other.y.x + self.z.x * other.z.x,
                self.x.x * other.x.y + self.y.x * other.y.y + self.z.x * other.z.y,
                self.x.x * other.x.z + self.y.x * other.y.z + self.z.x * other.z.z,
            ),
            y: Vector3Wide::new(
                self.x.y * other.x.x + self.y.y * other.y.x + self.z.y * other.z.x,
                self.x.y * other.x.y + self.y.y * other.y.y + self.z.y * other.z.y,
                self.x.y * other.x.z + self.y.y * other.y.z + self.z.y * other.z.z,
            ),
            z: Vector3Wide::new(
                self.x.z * other.x.x + self.y.z * other.y.x + self.z.z * other.z.x,
                self.x.z * other.x.y + self.y.z * other.y.y + self.z.z * other.z.y,
                self.x.z * other.x.z + self.y.z * other.y.z + self.z.z * other.z.z,
            ),
        }
    }

    #[inline(always)]
    pub fn multiply_by_transpose_without_overlap(&self, other: &Self) -> Self {
        Self {
            x: Vector3Wide::new(
                self.x.x * other.x.x + self.x.y * other.x.y + self.x.z * other.x.z,
                self.x.x * other.y.x + self.x.y * other.y.y + self.x.z * other.y.z,
                self.x.x * other.z.x + self.x.y * other.z.y + self.x.z * other.z.z,
            ),
            y: Vector3Wide::new(
                self.y.x * other.x.x + self.y.y * other.x.y + self.y.z * other.x.z,
                self.y.x * other.y.x + self.y.y * other.y.y + self.y.z * other.y.z,
                self.y.x * other.z.x + self.y.y * other.z.y + self.y.z * other.z.z,
            ),
            z: Vector3Wide::new(
                self.z.x * other.x.x + self.z.y * other.x.y + self.z.z * other.x.z,
                self.z.x * other.y.x + self.z.y * other.y.y + self.z.z * other.y.z,
                self.z.x * other.z.x + self.z.y * other.z.y + self.z.z * other.z.z,
            ),
        }
    }

    #[inline(always)]
    pub fn transform_without_overlap(&self, v: &Vector3Wide) -> Vector3Wide {
        Vector3Wide::new(
            v.x * self.x.x + v.y * self.y.x + v.z * self.z.x,
            v.x * self.x.y + v.y * self.y.y + v.z * self.z.y,
            v.x * self.x.z + v.y * self.y.z + v.z * self.z.z,
        )
    }

    #[inline(always)]
    pub fn transform_by_transposed_without_overlap(&self, v: &Vector3Wide) -> Vector3Wide {
        Vector3Wide::new(
            v.x * self.x.x + v.y * self.x.y + v.z * self.x.z,
            v.x * self.y.x + v.y * self.y.y + v.z * self.y.z,
            v.x * self.z.x + v.y * self.z.y + v.z * self.z.z,
        )
    }

    #[inline(always)]
    pub fn invert(&self) -> Self {
        let m11 = self.y.y * self.z.z - self.z.y * self.y.z;
        let m21 = self.y.z * self.z.x - self.z.z * self.y.x;
        let m31 = self.y.x * self.z.y - self.z.x * self.y.y;
        let determinant_inverse =
            Vector::splat(1.0) / (m11 * self.x.x + m21 * self.x.y + m31 * self.x.z);

        let m12 = self.z.y * self.x.z - self.x.y * self.z.z;
        let m22 = self.z.z * self.x.x - self.x.z * self.z.x;
        let m32 = self.z.x * self.x.y - self.x.x * self.z.y;

        let m13 = self.x.y * self.y.z - self.y.y * self.x.z;
        let m23 = self.x.z * self.y.x - self.y.z * self.x.x;
        let m33 = self.x.x * self.y.y - self.y.x * self.x.y;

        Self {
            x: Vector3Wide::new(
                m11 * determinant_inverse,
                m12 * determinant_inverse,
                m13 * determinant_inverse,
            ),
            y: Vector3Wide::new(
                m21 * determinant_inverse,
                m22 * determinant_inverse,
                m23 * determinant_inverse,
            ),
            z: Vector3Wide::new(
                m31 * determinant_inverse,
                m32 * determinant_inverse,
                m33 * determinant_inverse,
            ),
        }
    }

    #[inline(always)]
    pub fn create_cross_product(v: &Vector3Wide) -> Self {
        Self {
            x: Vector3Wide::new(Vector::splat(0.0), -v.z, v.y),
            y: Vector3Wide::new(v.z, Vector::splat(0.0), -v.x),
            z: Vector3Wide::new(-v.y, v.x, Vector::splat(0.0)),
        }
    }
    // TODO: THESE FUNCTIONS ARE CREATING NEW VALUES INSTEAD OF MODIFYING THE SPECIFIED VALUES.

    #[inline(always)]
    pub fn negate(&self, result: &mut Self) {
        result.x.x = -self.x.x;
        result.x.y = -self.x.y;
        result.x.z = -self.x.z;
        result.y.x = -self.y.x;
        result.y.y = -self.y.y;
        result.y.z = -self.y.z;
        result.z.x = -self.z.x;
        result.z.y = -self.z.y;
        result.z.z = -self.z.z;
    }

    #[inline(always)]
    pub fn scale(&self, scale: Vector) -> Self {
        Self {
            x: Vector3Wide::new(self.x.x * scale, self.x.y * scale, self.x.z * scale),
            y: Vector3Wide::new(self.y.x * scale, self.y.y * scale, self.y.z * scale),
            z: Vector3Wide::new(self.z.x * scale, self.z.y * scale, self.z.z * scale),
        }
    }

    #[inline(always)]
    pub fn from_quaternion(quaternion: &QuaternionWide) -> Self {
        let qx2 = quaternion.x + quaternion.x;
        let qy2 = quaternion.y + quaternion.y;
        let qz2 = quaternion.z + quaternion.z;

        let yy = qy2 * quaternion.y;
        let zz = qz2 * quaternion.z;
        let xy = qx2 * quaternion.y;
        let zw = qz2 * quaternion.w;
        let xz = qx2 * quaternion.z;
        let yw = qy2 * quaternion.w;
        let xx = qx2 * quaternion.x;
        let xw = qx2 * quaternion.w;
        let yz = qy2 * quaternion.z;

        Self {
            x: Vector3Wide::new(Vector::splat(1.0) - yy - zz, xy + zw, xz - yw),
            y: Vector3Wide::new(xy - zw, Vector::splat(1.0) - xx - zz, yz + xw),
            z: Vector3Wide::new(xz + yw, yz - xw, Vector::splat(1.0) - xx - yy),
        }
    }

    #[inline(always)]
    pub fn read_first(&self) -> Mat3 {
        Mat3 {
            x_axis: self.x.read_first(),
            y_axis: self.y.read_first(),
            z_axis: self.z.read_first(),
        }
    }

    #[inline(always)]
    pub fn read_slot(&self, slot_index: usize) -> Mat3 {
        unsafe {
            Mat3 {
                x_axis: self.x.read_slot(slot_index),
                y_axis: self.y.read_slot(slot_index),
                z_axis: self.z.read_slot(slot_index),
            }
        }
    }
}

impl Mul<Vector3Wide> for Matrix3x3Wide {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, rhs: Vector3Wide) -> Self::Output {
        self.transform_without_overlap(&rhs)
    }
}

impl Sub for Matrix3x3Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: Vector3Wide::new(self.x.x - rhs.x.x, self.x.y - rhs.x.y, self.x.z - rhs.x.z),
            y: Vector3Wide::new(self.y.x - rhs.y.x, self.y.y - rhs.y.y, self.y.z - rhs.y.z),
            z: Vector3Wide::new(self.z.x - rhs.z.x, self.z.y - rhs.z.y, self.z.z - rhs.z.z),
        }
    }
}
