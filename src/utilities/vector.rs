use num_traits::{Float, One, PrimInt, Zero};
use std::fmt::Debug;
use std::ops::{Add, Div, Mul, Sub};
use std::simd::SimdElement;

use crate::utilities::arch::VectorWidth;

/// Trait for SIMD operations
pub trait SimdOps: Copy + Debug + SimdElement + PartialOrd {
    type VectorType: Copy + Debug;

    /// Creates SIMD lanes initialized with `value`
    fn splat(value: Self) -> Self::VectorType;
    /// Add 2 SIMD lanes
    fn add(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Substract 2 SIMD lanes
    fn sub(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Multiply 2 SIMD lanes
    fn mul(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Divide 2 SIMD lanes
    fn div(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Return the minimum values between the 2 SIMD lanes
    fn min(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Return the maximum values between the 2 SIMD lanes
    fn max(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType;
    /// Reduce the SIMD lanes to a single value
    fn reduce_sum(a: Self::VectorType) -> Self;
}

impl SimdOps for f32 {
    type VectorType = VectorWidth;

    fn splat(value: Self) -> Self::VectorType {
        VectorWidth::splat(value)
    }
    fn add(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a + b
    }
    fn sub(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a - b
    }
    fn mul(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a * b
    }
    fn div(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a / b
    }
    fn min(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a.simd_min(b)
    }
    fn max(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a.simd_max(b)
    }
    fn reduce_sum(a: Self::VectorType) -> Self {
        a.reduce_sum()
    }
}

impl SimdOps for i32 {
    // TODO: THIS NEEDS TO BE SPECIFIC TO ARCHITECTURE
    type VectorType = std::simd::Simd<i32, 4>;

    fn splat(value: Self) -> Self::VectorType {
        Self::VectorType::splat(value)
    }
    fn add(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a + b
    }
    fn sub(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a - b
    }
    fn mul(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a * b
    }
    fn div(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a / b
    }
    fn min(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a.simd_min(b)
    }
    fn max(a: Self::VectorType, b: Self::VectorType) -> Self::VectorType {
        a.simd_max(b)
    }
    fn reduce_sum(a: Self::VectorType) -> Self {
        a.reduce_sum()
    }
}

// TODO: THIS NEEDS TO BE SPECIFIC TO ARCHITECTURE
#[repr(align(16))]
#[derive(Clone, Copy, Debug)]
pub struct Vector<T: SimdOps>(pub T::VectorType);

impl<
        T: SimdOps
            + Add<Output = T>
            + Sub<Output = T>
            + Mul<Output = T>
            + Div<Output = T>
            + Zero
            + One,
    > Vector<T>
{
    pub const LANES: usize = <<T as SimdOps>::VectorType as std::simd::Simd>::LEN;

    pub fn splat(value: T) -> Self {
        Vector(T::splat(value))
    }

    #[inline(always)]
    pub fn add(&self, other: &Self) -> Self {
        Vector(T::add(self.0, other.0))
    }

    #[inline(always)]
    pub fn sub(&self, other: &Self) -> Self {
        Vector(T::sub(self.0, other.0))
    }

    #[inline(always)]
    pub fn mul(&self, other: &Self) -> Self {
        Vector(T::mul(self.0, other.0))
    }

    #[inline(always)]
    pub fn div(&self, other: &Self) -> Self {
        Vector(T::div(self.0, other.0))
    }

    #[inline(always)]
    pub fn min(&self, other: &Self) -> Self {
        Vector(T::min(self.0, other.0))
    }

    #[inline(always)]
    pub fn max(&self, other: &Self) -> Self {
        Vector(T::max(self.0, other.0))
    }

    #[inline(always)]
    pub fn dot(&self, other: &Self) -> T {
        T::reduce_sum(T::mul(self.0, other.0))
    }

    #[inline(always)]
    pub fn length_squared(&self) -> T {
        self.dot(self)
    }
}

// Implement additional methods for floating-point types
impl<T: SimdOps + Float> Vector<T> {
    pub const ONE: Vector<T> = Vector(SimdOps::splat(T::one()));

    #[inline(always)]
    pub fn sqrt(&self) -> Self {
        Vector(self.0.map(|x| x.sqrt()))
    }

    #[inline(always)]
    pub fn length(&self) -> T {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let length = self.length();
        if length != T::zero() {
            self.div(&Vector::splat(length))
        } else {
            *self
        }
    }

    #[inline(always)]
    pub fn cross(&self, other: &Self) -> Self {
        let (ax, ay, az, _) = T::to_array(self.0);
        let (bx, by, bz, _) = T::to_array(other.0);
        Vector(T::from_array([
            ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx,
            T::zero(),
        ]))
    }
}

// Implement additional methods for integer types
impl<T: SimdOps + PrimInt> Vector<T> {
    pub const ONE: Vector<T> = Vector(SimdOps::splat(T::one()));

    #[inline(always)]
    pub fn abs(&self) -> Self {
        Vector(self.0.map(|x| x.abs()))
    }
}

// MARK: Operator traits

impl<T: SimdOps + Add<Output = T>> Add for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        self.add(&other)
    }
}

impl<T: SimdOps + Sub<Output = T>> Sub for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        self.sub(&other)
    }
}

impl<T: SimdOps + Mul<Output = T>> Mul for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self {
        self.mul(&other)
    }
}

impl<T: SimdOps + Div<Output = T>> Div for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, other: Self) -> Self {
        self.div(&other)
    }
}

impl<T: SimdOps + Mul<Output = T> + Copy> Mul<T> for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: T) -> Self {
        Vector(T::mul(self.0, T::splat(scalar)))
    }
}

impl<T: SimdOps + Div<Output = T> + Copy> Div<T> for Vector<T> {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: T) -> Self {
        Vector(T::div(self.0, T::splat(scalar)))
    }
}
