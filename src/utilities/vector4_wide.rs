#[cfg(target_arch = "aarch64")]
use core::arch::aarch64::*;
#[cfg(target_arch = "x86_64")]
use core::arch::x86_64::*;
use core::mem::transmute;
use crate::utilities::vector::Vector;
use glam::Vec4;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Vector4Wide {
    pub x: Vector<f32>,
    pub y: Vector<f32>,
    pub z: Vector<f32>,
    pub w: Vector<f32>,
}

impl Vector4Wide {
    #[inline(always)]
    pub fn broadcast(source: &Vec4) -> Self {
        Self {
            x: Vector::splat(source.x),
            y: Vector::splat(source.y),
            z: Vector::splat(source.z),
            w: Vector::splat(source.w),
        }
    }

    #[inline(always)]
    pub fn add(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
            w: a.w + b.w,
        }
    }

    #[inline(always)]
    pub fn add_scalar(v: &Self, s: Vector) -> Self {
        Self {
            x: v.x + s,
            y: v.y + s,
            z: v.z + s,
            w: v.w + s,
        }
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x - b.x,
            y: a.y - b.y,
            z: a.z - b.z,
            w: a.w - b.w,
        }
    }

    #[inline(always)]
    pub fn dot(a: &Self, b: &Self) -> Vector {
        a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
    }

    #[inline(always)]
    pub fn min(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.min(&b.x),
            y: a.y.min(&b.y),
            z: a.z.min(&b.z),
            w: a.w.min(&b.w),
        }
    }

    #[inline(always)]
    pub fn max(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.max(&b.x),
            y: a.y.max(&b.y),
            z: a.z.max(&b.z),
            w: a.w.max(&b.w),
        }
    }

    #[inline(always)]
    pub fn scale(vector: &Self, scalar: Vector) -> Self {
        Self {
            x: vector.x * scalar,
            y: vector.y * scalar,
            z: vector.z * scalar,
            w: vector.w * scalar,
        }
    }

    #[inline(always)]
    pub fn abs(&self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
            w: self.w.abs(),
        }
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: -self.w,
        }
    }

    #[inline(always)]
    pub fn length_squared(&self) -> Vector {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    #[inline(always)]
    pub fn length(&self) -> Vector {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn distance(a: &Self, b: &Self) -> Vector {
        let offset = Self::subtract(b, a);
        offset.length()
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        let scale = Vector::splat(1.0) / length;
        Self::scale(self, scale)
    }

    #[inline(always)]
    pub fn conditional_select(condition: VectorI, left: &Self, right: &Self) -> Self {
        unsafe {
            #[cfg(target_arch = "x86_64")]
            {
                Self {
                    x: transmute(_mm256_blendv_ps(transmute(right.x), transmute(left.x), transmute(condition))),
                    y: transmute(_mm256_blendv_ps(transmute(right.y), transmute(left.y), transmute(condition))),
                    z: transmute(_mm256_blendv_ps(transmute(right.z), transmute(left.z), transmute(condition))),
                    w: transmute(_mm256_blendv_ps(transmute(right.w), transmute(left.w), transmute(condition))),
                }
            }
            #[cfg(target_arch = "aarch64")]
            {
                Self {
                    x: transmute(vbslq_f32(transmute(condition), transmute(left.x), transmute(right.x))),
                    y: transmute(vbslq_f32(transmute(condition), transmute(left.y), transmute(right.y))),
                    z: transmute(vbslq_f32(transmute(condition), transmute(left.z), transmute(right.z))),
                    w: transmute(vbslq_f32(transmute(condition), transmute(left.w), transmute(right.w))),
                }
            }
            #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
            {
                Self {
                    x: Vector::select(condition.cast(), left.x, right.x),
                    y: Vector::select(condition.cast(), left.y, right.y),
                    z: Vector::select(condition.cast(), left.z, right.z),
                    w: Vector::select(condition.cast(), left.w, right.w),
                }
            }
        }
    }

    #[inline(always)]
    pub fn read_slot(&self, slot_index: usize) -> Vec4 {
        Vec4::new(
            self.x[slot_index],
            self.y[slot_index],
            self.z[slot_index],
            self.w[slot_index],
        )
    }

    #[inline(always)]
    pub fn write_first(source: &Vec4, target_slot: &mut Self) {
        target_slot.x[0] = source.x;
        target_slot.y[0] = source.y;
        target_slot.z[0] = source.z;
        target_slot.w[0] = source.w;
    }
}

impl std::ops::Add for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Self::add(&self, &rhs)
    }
}

impl std::ops::Add<Vector<f32>> for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Vector) -> Self::Output {
        Self::add_scalar(&self, rhs)
    }
}

impl std::ops::Sub for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::subtract(&self, &rhs)
    }
}

impl std::ops::Mul<Vector<f32>> for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: Vector) -> Self::Output {
        Self::scale(&self, rhs)
    }
}

impl std::ops::Neg for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        self.negate()
    }
}

impl std::fmt::Display for Vector4Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "<{}, {}, {}, {}>", self.x, self.y, self.z, self.w)
    }
}
