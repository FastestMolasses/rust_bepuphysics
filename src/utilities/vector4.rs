use portable_simd::f32x4;
use std::ops::{Add, Sub, Mul, Div, Neg};

#[derive(Copy, Clone, Debug)]
pub struct Vector4(pub f32x4);

impl Vector4 {
    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self(f32x4::new(x, y, z, w))
    }

    #[inline(always)]
    pub fn splat(value: f32) -> Self {
        Self(f32x4::splat(value))
    }

    #[inline(always)]
    pub fn zero() -> Self {
        Self(f32x4::splat(0.0))
    }

    #[inline(always)]
    pub fn x(&self) -> f32 {
        self.0.extract(0)
    }

    #[inline(always)]
    pub fn y(&self) -> f32 {
        self.0.extract(1)
    }

    #[inline(always)]
    pub fn z(&self) -> f32 {
        self.0.extract(2)
    }

    #[inline(always)]
    pub fn w(&self) -> f32 {
        self.0.extract(3)
    }

    #[inline(always)]
    pub fn dot(&self, other: &Self) -> f32 {
        (self.0 * other.0).sum()
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
    pub fn normalize(&self) -> Self {
        self / self.length()
    }

    #[inline(always)]
    pub fn min(&self, other: &Self) -> Self {
        Self(self.0.min(other.0))
    }

    #[inline(always)]
    pub fn max(&self, other: &Self) -> Self {
        Self(self.0.max(other.0))
    }

    #[inline(always)]
    pub fn abs(&self) -> Self {
        Self(self.0.abs())
    }

    #[inline(always)]
    pub fn floor(&self) -> Self {
        Self(self.0.floor())
    }

    #[inline(always)]
    pub fn ceil(&self) -> Self {
        Self(self.0.ceil())
    }

    #[inline(always)]
    pub fn round(&self) -> Self {
        Self(self.0.round())
    }

    #[inline(always)]
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

impl Add for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        Self(self.0 + other.0)
    }
}

impl Sub for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        Self(self.0 - other.0)
    }
}

impl Mul for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        Self(self.0 * other.0)
    }
}

impl Mul<f32> for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f32) -> Self::Output {
        Self(self.0 * f32x4::splat(scalar))
    }
}

impl Div for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn div(self, other: Self) -> Self::Output {
        Self(self.0 / other.0)
    }
}

impl Div<f32> for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: f32) -> Self::Output {
        Self(self.0 / f32x4::splat(scalar))
    }
}

impl Neg for Vector4 {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl From<[f32; 4]> for Vector4 {
    #[inline(always)]
    fn from(array: [f32; 4]) -> Self {
        Self(f32x4::from_slice_unaligned(&array))
    }
}

impl Into<[f32; 4]> for Vector4 {
    #[inline(always)]
    fn into(self) -> [f32; 4] {
        let mut result = [0.0; 4];
        self.0.write_to_slice_unaligned(&mut result);
        result
    }
}
