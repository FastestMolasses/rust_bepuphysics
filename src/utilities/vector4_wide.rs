use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;
use glam::Vec4;
use std::simd::{num::SimdFloat, Mask, StdFloat};

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
    pub fn broadcast(source: &Vec4, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x);
        broadcasted.y = Vector::splat(source.y);
        broadcasted.z = Vector::splat(source.z);
        broadcasted.w = Vector::splat(source.w);
    }

    /// Performs a componentwise add between two vectors.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
            w: a.w + b.w,
        }
    }

    /// Performs a componentwise add between two vectors and stores the result in the `result` parameter.
    #[inline(always)]
    pub fn add_to(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
        result.w = a.w + b.w;
    }

    /// Performs a componentwise add between a vector and a scalar.
    #[inline(always)]
    pub fn add_scalar(v: &Self, s: Vector<f32>) -> Self {
        Self {
            x: v.x + s,
            y: v.y + s,
            z: v.z + s,
            w: v.w + s,
        }
    }

    /// Performs a componentwise add between a vector and a scalar and stores the result in the `result` parameter.
    #[inline(always)]
    pub fn add_scalar_to(v: &Self, s: Vector<f32>, result: &mut Self) {
        result.x = v.x + s;
        result.y = v.y + s;
        result.z = v.z + s;
        result.w = v.w + s;
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
    pub fn subtract_to(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x - b.x;
        result.y = a.y - b.y;
        result.z = a.z - b.z;
        result.w = a.w - b.w;
    }

    #[inline(always)]
    pub fn dot(a: &Self, b: &Self, result: &mut Vector<f32>) {
        *result = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    /// Computes the per-component minimum of two vectors.
    #[inline(always)]
    pub fn min(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.simd_min(b.x);
        result.y = a.y.simd_min(b.y);
        result.z = a.z.simd_min(b.z);
        result.w = a.w.simd_min(b.w);
    }

    /// Computes the per-component maximum of two vectors.
    #[inline(always)]
    pub fn max(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.simd_max(b.x);
        result.y = a.y.simd_max(b.y);
        result.z = a.z.simd_max(b.z);
        result.w = a.w.simd_max(b.w);
    }

    #[inline(always)]
    pub fn scale(vector: &Self, scalar: Vector<f32>, result: &mut Self) {
        result.x = vector.x * scalar;
        result.y = vector.y * scalar;
        result.z = vector.z * scalar;
        result.w = vector.w * scalar;
    }

    #[inline(always)]
    pub fn abs(vector: &Self, result: &mut Self) {
        result.x = vector.x.abs();
        result.y = vector.y.abs();
        result.z = vector.z.abs();
        result.w = vector.w.abs();
    }

    #[inline(always)]
    pub fn negate(v: &Self) -> Self {
        Self {
            x: -v.x,
            y: -v.y,
            z: -v.z,
            w: -v.w,
        }
    }

    #[inline(always)]
    pub fn negate_to(v: &Self, result: &mut Self) {
        result.x = -v.x;
        result.y = -v.y;
        result.z = -v.z;
        result.w = -v.w;
    }

    #[inline(always)]
    pub fn length_squared(v: &Self, length_squared: &mut Vector<f32>) {
        *length_squared = v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
    }

    #[inline(always)]
    pub fn length(v: &Self, length: &mut Vector<f32>) {
        *length = Vector::<f32>::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    }

    #[inline(always)]
    pub fn distance(a: &Self, b: &Self, distance: &mut Vector<f32>) {
        let offset = Self::subtract(b, a);
        Self::length(&offset, distance);
    }

    #[inline(always)]
    pub fn normalize(v: &Self, result: &mut Self) {
        let mut length = Vector::splat(0.0);
        Self::length(v, &mut length);
        Self::scale(v, Vector::splat(1.0) / length, result);
    }

    #[inline(always)]
    pub fn conditional_select(
        condition: &Vector<i32>,
        left: &Self,
        right: &Self,
        result: &mut Self,
    ) {
        let mask = Mask::from_int(*condition);
        result.x = mask.select(left.x, right.x);
        result.y = mask.select(left.y, right.y);
        result.z = mask.select(left.z, right.z);
        result.w = mask.select(left.w, right.w);
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Vec4) {
        let offset = unsafe { GatherScatter::get_offset_instance(wide, slot_index) };
        Self::read_first(offset, narrow);
    }

    /// Pulls the first lane out of the wide representation.
    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Vec4) {
        target.x = source.x[0];
        target.y = source.y[0];
        target.z = source.z[0];
        target.w = source.w[0];
    }

    /// Gathers values from a vector and places them into the first indices of the target vector.
    #[inline(always)]
    pub fn write_first(source: &Vec4, target_slot: &mut Self) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target_slot.x) = source.x;
            *GatherScatter::get_first_mut(&mut target_slot.y) = source.y;
            *GatherScatter::get_first_mut(&mut target_slot.z) = source.z;
            *GatherScatter::get_first_mut(&mut target_slot.w) = source.w;
        }
    }
}

impl std::ops::Add<Vector4Wide> for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Self) -> Self::Output {
        Self::add(&self, &rhs)
    }
}

impl std::ops::Add<Vector<f32>> for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, rhs: Vector<f32>) -> Self::Output {
        Self::add_scalar(&self, rhs)
    }
}

impl std::ops::Add<Vector4Wide> for Vector<f32> {
    type Output = Vector4Wide;

    #[inline(always)]
    fn add(self, rhs: Vector4Wide) -> Self::Output {
        Vector4Wide::add_scalar(&rhs, self)
    }
}

impl std::ops::Sub for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::subtract(&self, &rhs)
    }
}

impl std::ops::Neg for Vector4Wide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self::Output {
        Vector4Wide::negate(&self)
    }
}

impl std::fmt::Display for Vector4Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "<{:?}, {:?}, {:?}, {:?}>",
            self.x, self.y, self.z, self.w
        )
    }
}
