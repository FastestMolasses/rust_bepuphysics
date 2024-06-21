#[cfg(target_arch = "aarch64")]
use core::arch::aarch64::*;
#[cfg(target_arch = "x86_64")]
use core::arch::x86_64::*;

use crate::utilities::quaternion::Quaternion;
use crate::utilities::vector3::Vector3;
use core::mem::transmute;
use packed_simd::f32x8 as Vector;
use packed_simd::i32x8 as VectorI;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Vector3Wide {
    pub x: Vector,
    pub y: Vector,
    pub z: Vector,
}

impl Vector3Wide {
    #[inline(always)]
    pub fn new(s: Vector) -> Self {
        Self { x: s, y: s, z: s }
    }

    #[inline(always)]
    pub fn add(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
        }
    }

    #[inline(always)]
    pub fn add_scalar(v: &Self, s: Vector) -> Self {
        Self {
            x: v.x + s,
            y: v.y + s,
            z: v.z + s,
        }
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x - b.x,
            y: a.y - b.y,
            z: a.z - b.z,
        }
    }

    #[inline(always)]
    pub fn subtract_scalar(v: &Self, s: Vector) -> Self {
        Self {
            x: v.x - s,
            y: v.y - s,
            z: v.z - s,
        }
    }

    #[inline(always)]
    pub fn subtract_from_scalar(s: Vector, v: &Self) -> Self {
        Self {
            x: s - v.x,
            y: s - v.y,
            z: s - v.z,
        }
    }

    #[inline(always)]
    pub fn dot(a: &Self, b: &Self) -> Vector {
        a.x * b.x + a.y * b.y + a.z * b.z
    }

    #[inline(always)]
    pub fn min(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.min(b.x),
            y: a.y.min(b.y),
            z: a.z.min(b.z),
        }
    }

    #[inline(always)]
    pub fn min_scalar(s: Vector, v: &Self) -> Self {
        Self {
            x: s.min(v.x),
            y: s.min(v.y),
            z: s.min(v.z),
        }
    }

    #[inline(always)]
    pub fn max(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.max(b.x),
            y: a.y.max(b.y),
            z: a.z.max(b.z),
        }
    }

    #[inline(always)]
    pub fn max_scalar(s: Vector, v: &Self) -> Self {
        Self {
            x: s.max(v.x),
            y: s.max(v.y),
            z: s.max(v.z),
        }
    }

    #[inline(always)]
    pub fn scale(vector: &Self, scalar: Vector) -> Self {
        Self {
            x: vector.x * scalar,
            y: vector.y * scalar,
            z: vector.z * scalar,
        }
    }

    #[inline(always)]
    pub fn abs(&self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    #[inline(always)]
    pub fn conditionally_negate(&self, should_negate: VectorI) -> Self {
        unsafe {
            #[cfg(target_arch = "x86_64")]
            {
                Self {
                    x: transmute(_mm256_xor_ps(
                        transmute(self.x),
                        _mm256_and_ps(transmute(should_negate), transmute(Vector::splat(-0.0))),
                    )),
                    y: transmute(_mm256_xor_ps(
                        transmute(self.y),
                        _mm256_and_ps(transmute(should_negate), transmute(Vector::splat(-0.0))),
                    )),
                    z: transmute(_mm256_xor_ps(
                        transmute(self.z),
                        _mm256_and_ps(transmute(should_negate), transmute(Vector::splat(-0.0))),
                    )),
                }
            }
            #[cfg(target_arch = "aarch64")]
            {
                Self {
                    x: transmute(vreinterpretq_f32_u32(veorq_u32(
                        vreinterpretq_u32_f32(transmute(self.x)),
                        vandq_u32(
                            transmute(should_negate),
                            vreinterpretq_u32_f32(vdupq_n_f32(-0.0)),
                        ),
                    ))),
                    y: transmute(vreinterpretq_f32_u32(veorq_u32(
                        vreinterpretq_u32_f32(transmute(self.y)),
                        vandq_u32(
                            transmute(should_negate),
                            vreinterpretq_u32_f32(vdupq_n_f32(-0.0)),
                        ),
                    ))),
                    z: transmute(vreinterpretq_f32_u32(veorq_u32(
                        vreinterpretq_u32_f32(transmute(self.z)),
                        vandq_u32(
                            transmute(should_negate),
                            vreinterpretq_u32_f32(vdupq_n_f32(-0.0)),
                        ),
                    ))),
                }
            }
            #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
            {
                Self {
                    x: self.x.select(should_negate.cast(), -self.x, self.x),
                    y: self.y.select(should_negate.cast(), -self.y, self.y),
                    z: self.z.select(should_negate.cast(), -self.z, self.z),
                }
            }
        }
    }

    #[inline(always)]
    pub fn cross(a: &Self, b: &Self) -> Self {
        Self {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x,
        }
    }

    #[inline(always)]
    pub fn length_squared(&self) -> Vector {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline(always)]
    pub fn length(&self) -> Vector {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn distance(a: &Self, b: &Self) -> Vector {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        (x * x + y * y + z * z).sqrt()
    }

    #[inline(always)]
    pub fn distance_squared(a: &Self, b: &Self) -> Vector {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        x * x + y * y + z * z
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
                    x: transmute(_mm256_blendv_ps(
                        transmute(right.x),
                        transmute(left.x),
                        transmute(condition),
                    )),
                    y: transmute(_mm256_blendv_ps(
                        transmute(right.y),
                        transmute(left.y),
                        transmute(condition),
                    )),
                    z: transmute(_mm256_blendv_ps(
                        transmute(right.z),
                        transmute(left.z),
                        transmute(condition),
                    )),
                }
            }
            #[cfg(target_arch = "aarch64")]
            {
                Self {
                    x: transmute(vbslq_f32(
                        transmute(condition),
                        transmute(left.x),
                        transmute(right.x),
                    )),
                    y: transmute(vbslq_f32(
                        transmute(condition),
                        transmute(left.y),
                        transmute(right.y),
                    )),
                    z: transmute(vbslq_f32(
                        transmute(condition),
                        transmute(left.z),
                        transmute(right.z),
                    )),
                }
            }
            #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
            {
                Self {
                    x: Vector::select(condition, left.x, right.x),
                    y: Vector::select(condition, left.y, right.y),
                    z: Vector::select(condition, left.z, right.z),
                }
            }
        }
    }

    #[inline(always)]
    pub fn multiply(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x * b.x,
            y: a.y * b.y,
            z: a.z * b.z,
        }
    }

    #[inline(always)]
    pub fn read_slot(&self, slot_index: usize) -> Vector3 {
        Vector3::new(self.x[slot_index], self.y[slot_index], self.z[slot_index])
    }

    #[inline(always)]
    pub fn write_slot(&mut self, source: Vector3, slot_index: usize) {
        self.x[slot_index] = source.x;
        self.y[slot_index] = source.y;
        self.z[slot_index] = source.z;
    }

    #[inline(always)]
    pub fn broadcast(source: Vector3) -> Self {
        Self {
            x: Vector::splat(source.x),
            y: Vector::splat(source.y),
            z: Vector::splat(source.z),
        }
    }

    #[inline(always)]
    pub fn rebroadcast(&self, slot_index: usize) -> Self {
        Self {
            x: Vector::splat(self.x[slot_index]),
            y: Vector::splat(self.y[slot_index]),
            z: Vector::splat(self.z[slot_index]),
        }
    }

    #[inline(always)]
    pub fn copy_slot(
        source: &Self,
        source_slot_index: usize,
        target: &mut Self,
        target_slot_index: usize,
    ) {
        target.x[target_slot_index] = source.x[source_slot_index];
        target.y[target_slot_index] = source.y[source_slot_index];
        target.z[target_slot_index] = source.z[source_slot_index];
    }
}

impl std::ops::Add for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Self::add(&self, &rhs)
    }
}

impl std::ops::Sub for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::subtract(&self, &rhs)
    }
}

impl std::ops::Mul<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: Vector) -> Self::Output {
        Self::scale(&self, rhs)
    }
}

impl std::ops::Mul<Vector3Wide> for Vector {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, rhs: Vector3Wide) -> Self::Output {
        Vector3Wide::scale(&rhs, self)
    }
}

impl std::ops::Div<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn div(self, rhs: Vector) -> Self::Output {
        let inverse = Vector::splat(1.0) / rhs;
        Self::scale(&self, inverse)
    }
}

impl std::ops::Neg for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        self.negate()
    }
}

impl std::fmt::Display for Vector3Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "<{}, {}, {}>", self.x, self.y, self.z)
    }
}
