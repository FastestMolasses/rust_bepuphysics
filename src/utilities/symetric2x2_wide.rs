use crate::utilities::matrix2x3::Matrix2x3Wide;
use crate::utilities::vector2::Vector2Wide;
use core::ops::*;
use packed_simd::*;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric2x2Wide {
    pub xx: f32x8,
    pub yx: f32x8,
    pub yy: f32x8,
}

impl Symmetric2x2Wide {
    #[inline(always)]
    pub fn sandwich_scale(m: &Matrix2x3Wide, scale: f32x8) -> Self {
        Self {
            xx: scale * (m.x.x * m.x.x + m.x.y * m.x.y + m.x.z * m.x.z),
            yx: scale * (m.y.x * m.x.x + m.y.y * m.x.y + m.y.z * m.x.z),
            yy: scale * (m.y.x * m.y.x + m.y.y * m.y.y + m.y.z * m.y.z),
        }
    }

    #[inline(always)]
    pub fn scale(&self, scale: f32x8) -> Self {
        Self {
            xx: self.xx * scale,
            yx: self.yx * scale,
            yy: self.yy * scale,
        }
    }

    #[inline(always)]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            xx: self.xx + other.xx,
            yx: self.yx + other.yx,
            yy: self.yy + other.yy,
        }
    }

    #[inline(always)]
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            xx: self.xx - other.xx,
            yx: self.yx - other.yx,
            yy: self.yy - other.yy,
        }
    }

    #[inline(always)]
    pub fn invert_without_overlap(&self) -> Self {
        let denom = f32x8::splat(1.0) / (self.yx * self.yx - self.xx * self.yy);
        Self {
            xx: -self.yy * denom,
            yx: self.yx * denom,
            yy: -self.xx * denom,
        }
    }

    #[inline(always)]
    pub fn transform_without_overlap(&self, v: &Vector2Wide) -> Vector2Wide {
        Vector2Wide {
            x: v.x * self.xx + v.y * self.yx,
            y: v.x * self.yx + v.y * self.yy,
        }
    }

    #[inline(always)]
    pub fn multiply_transposed(&self, a: &Matrix2x3Wide) -> Matrix2x3Wide {
        Matrix2x3Wide {
            x: Vector3Wide {
                x: a.x.x * self.xx + a.y.x * self.yx,
                y: a.x.y * self.xx + a.y.y * self.yx,
                z: a.x.z * self.xx + a.y.z * self.yx,
            },
            y: Vector3Wide {
                x: a.x.x * self.yx + a.y.x * self.yy,
                y: a.x.y * self.yx + a.y.y * self.yy,
                z: a.x.z * self.yx + a.y.z * self.yy,
            },
        }
    }

    #[inline(always)]
    pub fn complete_matrix_sandwich(a: &Matrix2x3Wide, b: &Matrix2x3Wide) -> Self {
        Self {
            xx: a.x.x * b.x.x + a.x.y * b.x.y + a.x.z * b.x.z,
            yx: a.y.x * b.x.x + a.y.y * b.x.y + a.y.z * b.x.z,
            yy: a.y.x * b.y.x + a.y.y * b.y.y + a.y.z * b.y.z,
        }
    }
}

impl Add for Symmetric2x2Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        self.add(&other)
    }
}

impl Sub for Symmetric2x2Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        self.sub(&other)
    }
}

impl Mul<f32x8> for Symmetric2x2Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scale: f32x8) -> Self {
        self.scale(scale)
    }
}
