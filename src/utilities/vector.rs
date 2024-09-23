use std::{ops::{Add, Div, Mul, Sub}, simd::num::SimdFloat};
use crate::utilities::arch::VectorWidth;

pub trait SimdOps: Copy + Debug {
    fn splat(value: Self) -> Simd<Self, 4>;
    fn simd_min(self, other: Self) -> Self;
    fn simd_max(self, other: Self) -> Self;
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
}

impl SimdOps for f32 {
    fn splat(value: Self) -> Simd<Self, 4> {
        Simd::from_array([value; 4])
    }
    fn simd_min(self, other: Self) -> Self { self.min(other) }
    fn simd_max(self, other: Self) -> Self { self.max(other) }
    fn sqrt(self) -> Self { self.sqrt() }
    fn abs(self) -> Self { self.abs() }
}

impl SimdOps for i32 {
    fn splat(value: Self) -> Simd<Self, 4> {
        Simd::from_array([value; 4])
    }
    fn simd_min(self, other: Self) -> Self { self.min(other) }
    fn simd_max(self, other: Self) -> Self { self.max(other) }
    fn sqrt(self) -> Self { (self as f32).sqrt() as i32 }
    fn abs(self) -> Self { self.abs() }
}

// Align to 16 bytes for NEON and SSE
// TODO: THIS NEEDS TO BE SPECIFIC TO ARCHITECTURE
// TODO: NEED VECTOR IMPLEMENTATION FOR INTEGER TYPES, MAKE THIS MORE GENERIC
#[repr(align(16))]
#[derive(Clone, Copy, Debug)]
pub struct Vector(pub VectorWidth);

impl Vector {
    pub const ONE: Vector = Vector::splat(1.0);

    #[inline(always)]
    pub fn splat(value: f32) -> Self {
        Vector(VectorWidth::splat(value))
    }

    #[inline(always)]
    pub fn add(&self, other: &Self) -> Self {
        Vector(self.0 + other.0)
    }

    #[inline(always)]
    pub fn sub(&self, other: &Self) -> Self {
        Vector(self.0 - other.0)
    }

    #[inline(always)]
    pub fn mul(&self, other: &Self) -> Self {
        Vector(self.0 * other.0)
    }

    #[inline(always)]
    pub fn div(&self, other: &Self) -> Self {
        Vector(self.0 / other.0)
    }

    #[inline(always)]
    pub fn min(&self, other: &Self) -> Self {
        Vector(self.0.simd_min(other.0))
    }

    #[inline(always)]
    pub fn max(&self, other: &Self) -> Self {
        Vector(self.0.simd_max(other.0))
    }

    #[inline(always)]
    pub fn sqrt(&self) -> Self {
        Vector(self.0.sqrt())
    }

    #[inline(always)]
    pub fn dot(&self, other: &Self) -> f32 {
        (self.0 * other.0).reduce_sum()
    }

    #[inline(always)]
    pub fn length_squared(&self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    pub fn length(&self) -> f32 {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn abs(&self) -> Self {
        Vector(self.0.abs())
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        if length != 0.0 {
            self.div(&Vector::splat(length))
        } else {
            *self
        }
    }

    #[inline(always)]
    pub fn cross(&self, other: &Self) -> Self {
        let (ax, ay, az, _) = self.0.to_array();
        let (bx, by, bz, _) = other.0.to_array();
        Vector(VectorWidth::from_array([
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx,
            0.0,
        ]))
    }
}

// Implement Add, Sub, Mul, Div traits
impl Add for Vector {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        self.add(&other)
    }
}

impl Sub for Vector {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        self.sub(&other)
    }
}

impl Mul for Vector {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self {
        self.mul(&other)
    }
}

impl Div for Vector {
    type Output = Self;

    #[inline(always)]
    fn div(self, other: Self) -> Self {
        self.div(&other)
    }
}

// Scalar operations
impl Mul<f32> for Vector {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f32) -> Self {
        Vector(self.0 * VectorWidth::splat(scalar))
    }
}

impl Div<f32> for Vector {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: f32) -> Self {
        Vector(self.0 / VectorWidth::splat(scalar))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector_operations() {
        let a = Vector::splat(1.0);
        let b = Vector::splat(2.0);

        let c = a + b;
        assert_eq!(c.0.to_array()[0], 3.0);

        let d = a * b;
        assert_eq!(d.0.to_array()[0], 2.0);

        let e = b / a;
        assert_eq!(e.0.to_array()[0], 2.0);

        let f = b - a;
        assert_eq!(f.0.to_array()[0], 1.0);

        let g = a.dot(&b);
        assert_eq!(g, 2.0 * VectorWidth::lanes() as f32);

        let h = a.cross(&b);
        assert_eq!(h.0.to_array()[0], 0.0);
    }
}
