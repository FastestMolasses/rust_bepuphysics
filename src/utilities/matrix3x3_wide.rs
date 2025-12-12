use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::ops::{Mul, Sub};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Matrix3x3Wide {
    pub x: Vector3Wide,
    pub y: Vector3Wide,
    pub z: Vector3Wide,
}

impl Matrix3x3Wide {
    #[inline(always)]
    pub fn broadcast(source: &Matrix3x3, broadcasted: &mut Self) {
        broadcasted.x = Vector3Wide {
            x: Vector::splat(source.x.x),
            y: Vector::splat(source.x.y),
            z: Vector::splat(source.x.z),
        };
        broadcasted.y = Vector3Wide {
            x: Vector::splat(source.y.x),
            y: Vector::splat(source.y.y),
            z: Vector::splat(source.y.z),
        };
        broadcasted.z = Vector3Wide {
            x: Vector::splat(source.z.x),
            y: Vector::splat(source.z.y),
            z: Vector::splat(source.z.z),
        };
    }

    #[inline(always)]
    pub fn create_identity() -> Self {
        Self {
            x: Vector3Wide {
                x: Vector::<f32>::splat(1.0),
                y: Vector::<f32>::splat(0.0),
                z: Vector::<f32>::splat(0.0),
            },
            y: Vector3Wide {
                x: Vector::<f32>::splat(0.0),
                y: Vector::<f32>::splat(1.0),
                z: Vector::<f32>::splat(0.0),
            },
            z: Vector3Wide {
                x: Vector::<f32>::splat(0.0),
                y: Vector::<f32>::splat(0.0),
                z: Vector::<f32>::splat(1.0),
            },
        }
    }

    #[inline(always)]
    pub fn multiply_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.y.x + a.x.z * b.z.x;
        result.x.y = a.x.x * b.x.y + a.x.y * b.y.y + a.x.z * b.z.y;
        result.x.z = a.x.x * b.x.z + a.x.y * b.y.z + a.x.z * b.z.z;

        result.y.x = a.y.x * b.x.x + a.y.y * b.y.x + a.y.z * b.z.x;
        result.y.y = a.y.x * b.x.y + a.y.y * b.y.y + a.y.z * b.z.y;
        result.y.z = a.y.x * b.x.z + a.y.y * b.y.z + a.y.z * b.z.z;

        result.z.x = a.z.x * b.x.x + a.z.y * b.y.x + a.z.z * b.z.x;
        result.z.y = a.z.x * b.x.y + a.z.y * b.y.y + a.z.z * b.z.y;
        result.z.z = a.z.x * b.x.z + a.z.y * b.y.z + a.z.z * b.z.z;
    }

    /// Multiplies a matrix by another matrix, where the first matrix is sampled as if it were transposed: result = transpose(a) * b.
    #[inline(always)]
    pub fn multiply_transposed_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.y.x * b.y.x + a.z.x * b.z.x;
        result.x.y = a.x.x * b.x.y + a.y.x * b.y.y + a.z.x * b.z.y;
        result.x.z = a.x.x * b.x.z + a.y.x * b.y.z + a.z.x * b.z.z;

        result.y.x = a.x.y * b.x.x + a.y.y * b.y.x + a.z.y * b.z.x;
        result.y.y = a.x.y * b.x.y + a.y.y * b.y.y + a.z.y * b.z.y;
        result.y.z = a.x.y * b.x.z + a.y.y * b.y.z + a.z.y * b.z.z;

        result.z.x = a.x.z * b.x.x + a.y.z * b.y.x + a.z.z * b.z.x;
        result.z.y = a.x.z * b.x.y + a.y.z * b.y.y + a.z.z * b.z.y;
        result.z.z = a.x.z * b.x.z + a.y.z * b.y.z + a.z.z * b.z.z;
    }

    /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
    #[inline(always)]
    pub fn multiply_by_transpose_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        result.x.x = a.x.x * b.x.x + a.x.y * b.x.y + a.x.z * b.x.z;
        result.x.y = a.x.x * b.y.x + a.x.y * b.y.y + a.x.z * b.y.z;
        result.x.z = a.x.x * b.z.x + a.x.y * b.z.y + a.x.z * b.z.z;

        result.y.x = a.y.x * b.x.x + a.y.y * b.x.y + a.y.z * b.x.z;
        result.y.y = a.y.x * b.y.x + a.y.y * b.y.y + a.y.z * b.y.z;
        result.y.z = a.y.x * b.z.x + a.y.y * b.z.y + a.y.z * b.z.z;

        result.z.x = a.z.x * b.x.x + a.z.y * b.x.y + a.z.z * b.x.z;
        result.z.y = a.z.x * b.y.x + a.z.y * b.y.y + a.z.z * b.y.z;
        result.z.z = a.z.x * b.z.x + a.z.y * b.z.y + a.z.z * b.z.z;
    }

    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector3Wide, m: &Self, result: &mut Vector3Wide) {
        result.x = v.x * m.x.x + v.y * m.y.x + v.z * m.z.x;
        result.y = v.x * m.x.y + v.y * m.y.y + v.z * m.z.y;
        result.z = v.x * m.x.z + v.y * m.y.z + v.z * m.z.z;
    }

    #[inline(always)]
    pub fn transform_by_transposed_without_overlap(
        v: &Vector3Wide,
        m: &Self,
        result: &mut Vector3Wide,
    ) {
        result.x = v.x * m.x.x + v.y * m.x.y + v.z * m.x.z;
        result.y = v.x * m.y.x + v.y * m.y.y + v.z * m.y.z;
        result.z = v.x * m.z.x + v.y * m.z.y + v.z * m.z.z;
    }

    #[inline(always)]
    pub fn transform(v: &Vector3Wide, m: &Self, result: &mut Vector3Wide) {
        Self::transform_without_overlap(v, m, result);
    }

    #[inline(always)]
    pub fn invert(m: &Self, inverse: &mut Self) {
        let m11 = m.y.y * m.z.z - m.z.y * m.y.z;
        let m21 = m.y.z * m.z.x - m.z.z * m.y.x;
        let m31 = m.y.x * m.z.y - m.z.x * m.y.y;
        let determinant_inverse =
            Vector::<f32>::splat(1.0) / (m11 * m.x.x + m21 * m.x.y + m31 * m.x.z);

        let m12 = m.z.y * m.x.z - m.x.y * m.z.z;
        let m22 = m.z.z * m.x.x - m.x.z * m.z.x;
        let m32 = m.z.x * m.x.y - m.x.x * m.z.y;

        let m13 = m.x.y * m.y.z - m.y.y * m.x.z;
        let m23 = m.x.z * m.y.x - m.y.z * m.x.x;
        let m33 = m.x.x * m.y.y - m.y.x * m.x.y;

        inverse.x.x = m11 * determinant_inverse;
        inverse.y.x = m21 * determinant_inverse;
        inverse.z.x = m31 * determinant_inverse;
        inverse.x.y = m12 * determinant_inverse;
        inverse.y.y = m22 * determinant_inverse;
        inverse.z.y = m32 * determinant_inverse;
        inverse.x.z = m13 * determinant_inverse;
        inverse.y.z = m23 * determinant_inverse;
        inverse.z.z = m33 * determinant_inverse;
    }

    #[inline(always)]
    pub fn create_cross_product_to(v: &Vector3Wide, skew: &mut Self) {
        skew.x = Vector3Wide {
            x: Vector::<f32>::splat(0.0),
            y: -v.z,
            z: v.y,
        };
        skew.y = Vector3Wide {
            x: v.z,
            y: Vector::<f32>::splat(0.0),
            z: -v.x,
        };
        skew.z = Vector3Wide {
            x: -v.y,
            y: v.x,
            z: Vector::<f32>::splat(0.0),
        };
    }

    #[inline(always)]
    pub fn create_cross_product(v: &Vector3Wide) -> Self {
        Self {
            x: Vector3Wide {
                x: Vector::<f32>::splat(0.0),
                y: -v.z,
                z: v.y,
            },
            y: Vector3Wide {
                x: v.z,
                y: Vector::<f32>::splat(0.0),
                z: -v.x,
            },
            z: Vector3Wide {
                x: -v.y,
                y: v.x,
                z: Vector::<f32>::splat(0.0),
            },
        }
    }

    /// Negates the components of a matrix.
    #[inline(always)]
    pub fn negate(m: &Self, result: &mut Self) {
        result.x = Vector3Wide {
            x: -m.x.x,
            y: -m.x.y,
            z: -m.x.z,
        };
        result.y = Vector3Wide {
            x: -m.y.x,
            y: -m.y.y,
            z: -m.y.z,
        };
        result.z = Vector3Wide {
            x: -m.z.x,
            y: -m.z.y,
            z: -m.z.z,
        };
    }

    /// Multiplies every component in the matrix by the given scalar value.
    #[inline(always)]
    pub fn scale(m: &Self, scale: &Vector<f32>, result: &mut Self) {
        result.x = Vector3Wide {
            x: m.x.x * scale,
            y: m.x.y * scale,
            z: m.x.z * scale,
        };
        result.y = Vector3Wide {
            x: m.y.x * scale,
            y: m.y.y * scale,
            z: m.y.z * scale,
        };
        result.z = Vector3Wide {
            x: m.z.x * scale,
            y: m.z.y * scale,
            z: m.z.z * scale,
        };
    }

    #[inline(always)]
    pub fn create_from_quaternion(quaternion: &QuaternionWide, result: &mut Self) {
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

        result.x = Vector3Wide {
            x: Vector::splat(1.0) - yy - zz,
            y: xy + zw,
            z: xz - yw,
        };
        result.y = Vector3Wide {
            x: xy - zw,
            y: Vector::splat(1.0) - xx - zz,
            z: yz + xw,
        };
        result.z = Vector3Wide {
            x: xz + yw,
            y: yz - xw,
            z: Vector::splat(1.0) - xx - yy,
        };
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        result.x.x = a.x.x - b.x.x;
        result.x.y = a.x.y - b.x.y;
        result.x.z = a.x.z - b.x.z;
        result.y.x = a.y.x - b.y.x;
        result.y.y = a.y.y - b.y.y;
        result.y.z = a.y.z - b.y.z;
        result.z.x = a.z.x - b.z.x;
        result.z.y = a.z.y - b.z.y;
        result.z.z = a.z.z - b.z.z;
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Matrix3x3) {
        Vector3Wide::read_first(&source.x, &mut target.x);
        Vector3Wide::read_first(&source.y, &mut target.y);
        Vector3Wide::read_first(&source.z, &mut target.z);
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Matrix3x3) {
        let offset = unsafe { GatherScatter::get_offset_instance(wide, slot_index) };
        Self::read_first(offset, narrow);
    }
}

impl Mul<Vector3Wide> for Matrix3x3Wide {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, rhs: Vector3Wide) -> Self::Output {
        Vector3Wide {
            x: rhs.x * self.x.x + rhs.y * self.y.x + rhs.z * self.z.x,
            y: rhs.x * self.x.y + rhs.y * self.y.y + rhs.z * self.z.y,
            z: rhs.x * self.x.z + rhs.y * self.y.z + rhs.z * self.z.z,
        }
    }
}

impl Sub for Matrix3x3Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: Vector3Wide {
                x: self.x.x - rhs.x.x,
                y: self.x.y - rhs.x.y,
                z: self.x.z - rhs.x.z,
            },
            y: Vector3Wide {
                x: self.y.x - rhs.y.x,
                y: self.y.y - rhs.y.y,
                z: self.y.z - rhs.y.z,
            },
            z: Vector3Wide {
                x: self.z.x - rhs.z.x,
                y: self.z.y - rhs.z.y,
                z: self.z.z - rhs.z.z,
            },
        }
    }
}
