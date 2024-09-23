#[cfg(target_arch = "aarch64")]
use core::arch::aarch64::*;
#[cfg(target_arch = "x86_64")]
use core::arch::x86_64::*;

use crate::utilities::vector::Vector;
use core::mem::transmute;
use glam::Vec3;
use std::ops::{Add, Div, Mul, Neg, Sub};
// TODO: REPLACE CUSTOM VECTOR TYPE
// use portable_simd::i32x8 as VectorI;

#[repr(C)]
#[derive(Clone, Copy)]
/// Three dimensional vector with SIMD lanes.
pub struct Vector3Wide {
    /// First component of the vector.
    pub x: Vector,
    /// Second component of the vector.
    pub y: Vector,
    /// Third component of the vector.
    pub z: Vector,
}

impl From<Vector> for Vector3Wide {
    #[inline(always)]
    fn from(s: Vector) -> Self {
        Self::new(s)
    }
}

impl Vector3Wide {
    /// Creates a vector by populating each component with the given scalars.
    #[inline(always)]
    pub fn new(s: Vector) -> Self {
        Self { x: s, y: s, z: s }
    }

    /// Creates a vector by populating each component with the given scalar.
    #[inline(always)]
    pub fn from_ref(s: &Vector) -> Self {
        Self {
            x: *s,
            y: *s,
            z: *s,
        }
    }

    /// Performs a componentwise add between two vectors.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
    }

    /// Finds the result of adding a scalar to every component of a vector.
    #[inline(always)]
    pub fn add_scalar(v: &Self, s: &Vector, result: &mut Self) {
        result.x = v.x + s;
        result.y = v.y + s;
        result.z = v.z + s;
    }

    /// Subtracts one vector from another.
    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x - b.x;
        result.y = a.y - b.y;
        result.z = a.z - b.z;
    }

    /// Finds the result of subtracting a scalar from every component of a vector.
    #[inline(always)]
    pub fn subtract_scalar(v: &Self, s: &Vector, result: &mut Self) {
        result.x = v.x - s;
        result.y = v.y - s;
        result.z = v.z - s;
    }

    /// Finds the result of subtracting the components of a vector from a scalar.
    #[inline(always)]
    pub fn subtract_from_scalar(s: &Vector, v: &Self, result: &mut Self) {
        result.x = s - v.x;
        result.y = s - v.y;
        result.z = s - v.z;
    }

    /// Computes the inner product between two vectors by modifying the result vector.
    #[inline(always)]
    pub fn dot(a: &Self, b: &Self, result: &mut Vector) {
        *result = a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Computes the inner product between two vectors.
    #[inline(always)]
    pub fn dot_val(a: &Self, b: &Self) -> Vector {
        a.x * b.x + a.y * b.y + a.z * b.z
    }

    /// Computes the per-component minimum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn min_scalar(s: &Vector, v: &Self, result: &mut Self) {
        result.x = s.min(&v.x);
        result.y = s.min(&v.y);
        result.z = s.min(&v.z);
    }

    /// Computes the per-component minimum of two vectors.
    #[inline(always)]
    pub fn min(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.min(&b.x);
        result.y = a.y.min(&b.y);
        result.z = a.z.min(&b.z);
    }

    /// Computes the per-component minimum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn min_scalar_new(s: &Vector, v: &Self) -> Self {
        Self {
            x: s.min(&v.x),
            y: s.min(&v.y),
            z: s.min(&v.z),
        }
    }

    /// Computes the per-component minimum of two vectors.
    #[inline(always)]
    pub fn min_new(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.min(&b.x),
            y: a.y.min(&b.y),
            z: a.z.min(&b.z),
        }
    }

    /// Computes the per-component maximum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn max_scalar(s: &Vector, v: &Self, result: &mut Self) {
        result.x = s.max(&v.x);
        result.y = s.max(&v.y);
        result.z = s.max(&v.z);
    }

    /// Computes the per-component maximum of two vectors.
    #[inline(always)]
    pub fn max(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.max(&b.x);
        result.y = a.y.max(&b.y);
        result.z = a.z.max(&b.z);
    }

    /// Computes the per-component maximum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn max_scalar_new(s: &Vector, v: &Self) -> Self {
        Self {
            x: s.max(&v.x),
            y: s.max(&v.y),
            z: s.max(&v.z),
        }
    }

    /// Computes the per-component maximum of two vectors.
    #[inline(always)]
    pub fn max_new(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.max(&b.x),
            y: a.y.max(&b.y),
            z: a.z.max(&b.z),
        }
    }

    /// Scales a vector by a scalar.
    #[inline(always)]
    pub fn scale(vector: &Self, scalar: &Vector, result: &mut Self) {
        result.x = vector.x * *scalar;
        result.y = vector.y * *scalar;
        result.z = vector.z * *scalar;
    }

    /// Computes the absolute value of a vector.
    #[inline(always)]
    pub fn abs(vector: &Self, result: &mut Self) {
        result.x = vector.x.abs();
        result.y = vector.y.abs();
        result.z = vector.z.abs();
    }

    /// Computes the absolute value of a vector.
    #[inline(always)]
    pub fn abs_new(vector: &Self) -> Self {
        Self {
            x: vector.x.abs(),
            y: vector.y.abs(),
            z: vector.z.abs(),
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

    /// Negates a vector in place and returns a reference to it.
    #[inline(always)]
    pub fn negate(&self) -> &Self {
        self.x = -self.x;
        self.y = -self.y;
        self.z = -self.z;
        self
    }

    /// Negates a vector into another vector.
    #[inline(always)]
    pub fn negate_static(&self, result: &mut self) -> Self {
        result.x = -self.x;
        result.y = -self.y;
        result.z = -self.z;
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
    pub fn read_slot(&self, slot_index: usize) -> Vec3 {
        Vec3::new(self.x[slot_index], self.y[slot_index], self.z[slot_index])
    }

    #[inline(always)]
    pub fn write_slot(&mut self, source: Vec3, slot_index: usize) {
        self.x[slot_index] = source.x;
        self.y[slot_index] = source.y;
        self.z[slot_index] = source.z;
    }

    #[inline(always)]
    pub fn broadcast(source: Vec3) -> Self {
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

impl Add for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Add<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, s: Vector) -> Self {
        Self {
            x: self.x + s,
            y: self.y + s,
            z: self.z + s,
        }
    }
}

impl Add<Vector3Wide> for Vector {
    type Output = Vector3Wide;

    #[inline(always)]
    fn add(self, v: Vector3Wide) -> Vector3Wide {
        Vector3Wide {
            x: v.x + self,
            y: v.y + self,
            z: v.z + self,
        }
    }
}

impl Sub<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, s: Vector) -> Self {
        Self {
            x: self.x - s,
            y: self.y - s,
            z: self.z - s,
        }
    }
}

impl Sub<Vector3Wide> for Vector {
    type Output = Vector3Wide;

    #[inline(always)]
    fn sub(self, v: Vector3Wide) -> Vector3Wide {
        Vector3Wide {
            x: self - v.x,
            y: self - v.y,
            z: self - v.z,
        }
    }
}

impl Div<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: Vector) -> Self::Output {
        let inverse = Vector::ONE / scalar;
        Self {
            x: self.x * inverse,
            y: self.y * inverse,
            z: self.z * inverse,
        }
    }
}

impl Mul<Vector> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: Vector) -> Self::Output {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Mul<Vector3Wide> for Vector {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, vector: Vector3Wide) -> Self::Output {
        Vector3Wide {
            x: vector.x * self,
            y: vector.y * self,
            z: vector.z * self,
        }
    }
}

impl Neg for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl std::fmt::Display for Vector3Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "<{}, {}, {}>", self.x, self.y, self.z)
    }
}
