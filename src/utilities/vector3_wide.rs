use crate::{out_unsafe, utilities::gather_scatter::GatherScatter};
use crate::utilities::vector::Vector;
use glam::Vec3;
use std::{
    ops::{Add, Div, Mul, Neg, Sub},
    simd::{num::SimdFloat, Mask},
};

#[repr(C)]
#[derive(Clone, Copy, Debug)]
/// Three dimensional vector with SIMD lanes.
pub struct Vector3Wide {
    /// First component of the vector.
    pub x: Vector<f32>,
    /// Second component of the vector.
    pub y: Vector<f32>,
    /// Third component of the vector.
    pub z: Vector<f32>,
}

impl From<Vector<f32>> for Vector3Wide {
    #[inline(always)]
    fn from(s: Vector<f32>) -> Self {
        Self { x: s, y: s, z: s }
    }
}

impl From<&Vector<f32>> for Vector3Wide {
    #[inline(always)]
    fn from(s: &Vector<f32>) -> Self {
        Self {
            x: *s,
            y: *s,
            z: *s,
        }
    }
}

impl Vector3Wide {
    /// Creates a vector by populating each component with the given scalars.
    #[inline(always)]
    pub fn new(s: Vector<f32>) -> Self {
        Self { x: s, y: s, z: s }
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
    pub fn add_scalar(v: &Self, s: &Vector<f32>, result: &mut Self) {
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
    pub fn subtract_scalar(v: &Self, s: &Vector<f32>, result: &mut Self) {
        result.x = v.x - s;
        result.y = v.y - s;
        result.z = v.z - s;
    }

    /// Finds the result of subtracting the components of a vector from a scalar.
    #[inline(always)]
    pub fn subtract_from_scalar(s: &Vector<f32>, v: &Self, result: &mut Self) {
        result.x = s - v.x;
        result.y = s - v.y;
        result.z = s - v.z;
    }

    /// Computes the inner product between two vectors by modifying the result vector.
    #[inline(always)]
    pub fn dot(a: &Self, b: &Self, result: &mut Vector<f32>) {
        *result = a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Computes the inner product between two vectors.
    #[inline(always)]
    pub fn dot_val(a: &Self, b: &Self) -> Vector<f32> {
        a.x * b.x + a.y * b.y + a.z * b.z
    }

    /// Computes the per-component minimum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn min_scalar(s: &Vector<f32>, v: &Self, result: &mut Self) {
        result.x = s.simd_min(v.x);
        result.y = s.simd_min(v.y);
        result.z = s.simd_min(v.z);
    }

    /// Computes the per-component minimum of two vectors.
    #[inline(always)]
    pub fn min(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.simd_min(b.x);
        result.y = a.y.simd_min(b.y);
        result.z = a.z.simd_min(b.z);
    }

    /// Computes the per-component minimum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn min_scalar_new(s: &Vector<f32>, v: &Self) -> Self {
        Self {
            x: s.simd_min(v.x),
            y: s.simd_min(v.y),
            z: s.simd_min(v.z),
        }
    }

    /// Computes the per-component minimum of two vectors.
    #[inline(always)]
    pub fn min_new(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.simd_min(b.x),
            y: a.y.simd_min(b.y),
            z: a.z.simd_min(b.z),
        }
    }

    /// Computes the per-component maximum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn max_scalar(s: &Vector<f32>, v: &Self, result: &mut Self) {
        result.x = s.simd_max(v.x);
        result.y = s.simd_max(v.y);
        result.z = s.simd_max(v.z);
    }

    /// Computes the per-component maximum of two vectors.
    #[inline(always)]
    pub fn max(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x.simd_max(b.x);
        result.y = a.y.simd_max(b.y);
        result.z = a.z.simd_max(b.z);
    }

    /// Computes the per-component maximum between a scalar value and the components of a vector.
    #[inline(always)]
    pub fn max_scalar_new(s: &Vector<f32>, v: &Self) -> Self {
        Self {
            x: s.simd_max(v.x),
            y: s.simd_max(v.y),
            z: s.simd_max(v.z),
        }
    }

    /// Computes the per-component maximum of two vectors.
    #[inline(always)]
    pub fn max_new(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x.simd_max(b.x),
            y: a.y.simd_max(b.y),
            z: a.z.simd_max(b.z),
        }
    }

    /// Scales a vector by a scalar.
    #[inline(always)]
    pub fn scale(vector: &Self, scalar: &Vector<f32>) -> Self {
        Self {
            x: vector.x * *scalar,
            y: vector.y * *scalar,
            z: vector.z * *scalar,
        }
    }

    /// Scales a vector by a scalar into another vector.
    #[inline(always)]
    pub fn scale_to(vector: &Self, scalar: &Vector<f32>, result: &mut Self) {
        result.x = vector.x * *scalar;
        result.y = vector.y * *scalar;
        result.z = vector.z * *scalar;
    }

    /// Computes the absolute value of a vector and stores it in another vector.
    #[inline(always)]
    pub fn abs_to(vector: &Self, result: &mut Self) {
        result.x = vector.x.abs();
        result.y = vector.y.abs();
        result.z = vector.z.abs();
    }

    /// Computes the absolute value of a vector.
    #[inline(always)]
    pub fn abs(&self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    /// Negates a vector.
    #[inline(always)]
    pub fn negate(v: &Self, result: &mut Self) {
        result.x = -v.x;
        result.y = -v.y;
        result.z = -v.z;
    }

    /// Negates a vector in place and returns a reference to it.
    pub fn negate_self(&mut self) -> &Self {
        self.x = -self.x;
        self.y = -self.y;
        self.z = -self.z;
        self
    }

    /// Conditionally negates lanes of the vector.
    #[inline(always)]
    pub fn conditionally_negate(should_negate: &Vector<i32>, v: &mut Self) {
        let mask = Mask::from_int(*should_negate);
        v.x = mask.select(-v.x, v.x);
        v.y = mask.select(-v.y, v.y);
        v.z = mask.select(-v.z, v.z);
    }

    /// Conditionally negates lanes of the vector and stores the result in another vector.
    #[inline(always)]
    pub fn conditionally_negate_to(should_negate: &Vector<i32>, v: &Self, negated: &mut Self) {
        let mask = Mask::from_int(*should_negate);
        negated.x = mask.select(-v.x, v.x);
        negated.y = mask.select(-v.y, v.y);
        negated.z = mask.select(-v.z, v.z);
    }

    /// Conditionally negates lanes of the vector and stores the result in another vector.
    #[inline(always)]
    pub fn conditionally_negate_to_new(should_negate: &Vector<i32>, v: &Self) -> Self {
        let mask = Mask::from_int(*should_negate);
        Self {
            x: mask.select(-v.x, v.x),
            y: mask.select(-v.y, v.y),
            z: mask.select(-v.z, v.z),
        }
    }

    /// Computes the cross product between two vectors, assuming that the vector references are not aliased.
    #[inline(always)]
    pub unsafe fn cross_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        // This will fail if the result reference is actually a or b!
        result.x = a.y * b.z - a.z * b.y;
        result.y = a.z * b.x - a.x * b.z;
        result.z = a.x * b.y - a.y * b.x;
    }

    /// Computes the cross product between two vectors.
    #[inline(always)]
    pub fn cross(a: &Self, b: &Self, result: &mut Self) {
        *result = out_unsafe!(Self::cross_without_overlap(a, b));
    }

    /// Computes the cross product between two vectors.
    #[inline(always)]
    pub fn cross_new(a: &Self, b: &Self) -> Self {
        Self {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x,
        }
    }

    /// Computes the squared length of this vector.
    #[inline(always)]
    pub fn length_squared(&self) -> Vector<f32> {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Computes the squared length of a vector into another.
    #[inline(always)]
    pub fn length_squared_to(v: &Self, length_squared: &mut Vector<f32>) {
        *length_squared = v.x * v.x + v.y * v.y + v.z * v.z;
    }

    /// Computes the squared length of another vector.
    #[inline(always)]
    pub fn length_squared_of(v: &Self) -> Vector<f32> {
        v.x * v.x + v.y * v.y + v.z * v.z
    }

    /// Computes the length of this vector.
    #[inline(always)]
    pub fn length(&self) -> Vector<f32> {
        std::simd::StdFloat::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Computes the length of a vector into another.
    #[inline(always)]
    pub fn length_into(v: &Self, length: &mut Vector<f32>) {
        *length = std::simd::StdFloat::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    /// Computes the distance between two vectors.
    #[inline(always)]
    pub fn distance(a: &Self, b: &Self) -> Vector<f32> {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        std::simd::StdFloat::sqrt(x * x + y * y + z * z)
    }

    /// Computes the distance between two vectors, into another vector.
    #[inline(always)]
    pub fn distance_to(a: &Self, b: &Self, distance: &mut Vector<f32>) {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        *distance = std::simd::StdFloat::sqrt(x * x + y * y + z * z);
    }

    /// Computes the squared distance between two vectors.
    #[inline(always)]
    pub fn distance_squared(a: &Self, b: &Self) -> Vector<f32> {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        x * x + y * y + z * z
    }

    /// Computes the squared distance between two vectors into another.
    #[inline(always)]
    pub fn distance_squared_to(a: &Self, b: &Self, distance_squared: &mut Vector<f32>) {
        let x = b.x - a.x;
        let y = b.y - a.y;
        let z = b.z - a.z;
        *distance_squared = x * x + y * y + z * z;
    }

    /// Computes a unit length vector pointing in the same direction as the input.
    #[inline(always)]
    pub fn normalize(v: &Self) -> Self {
        let length = Self::length(&v);
        let scale = Vector::splat(1.0) / length;
        Self::scale(v, &scale)
    }

    /// Computes a unit length vector pointing in the same direction as the input, into another vector.
    #[inline(always)]
    pub fn normalize_to(v: &Self, result: &mut Self) {
        let length = Self::length(&v);
        let scale = Vector::splat(1.0) / length;
        Self::scale_to(v, &scale, result);
    }

    /// Selects the left or right input for each lane depending on a mask.
    #[inline(always)]
    pub fn conditional_select(condition: &Vector<i32>, left: &Self, right: &Self) -> Self {
        let mask = Mask::from_int(*condition);
        Self {
            x: mask.select(left.x, right.x),
            y: mask.select(left.y, right.y),
            z: mask.select(left.z, right.z),
        }
    }

    /// Selects the left or right input for each lane depending on a mask, into another vector.
    #[inline(always)]
    pub fn conditional_select_to(
        condition: &Vector<i32>,
        left: &Self,
        right: &Self,
        result: &mut Self,
    ) {
        let mask = Mask::from_int(*condition);
        result.x = mask.select(left.x, right.x);
        result.y = mask.select(left.y, right.y);
        result.z = mask.select(left.z, right.z);
    }

    /// Multiplies the components of one vector with another.
    #[inline(always)]
    pub fn multiply(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x * b.x;
        result.y = a.y * b.y;
        result.z = a.z * b.z;
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Vec3) {
        let offset = unsafe { GatherScatter::get_offset_instance(wide, slot_index) };
        Self::read_first(offset, narrow);
    }

    /// Pulls the first lane out of the wide representation.
    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Vec3) {
        target.x = source.x[0];
        target.y = source.y[0];
        target.z = source.z[0];
    }

    /// Writes a value into a slot of the target bundle.
    #[inline(always)]
    pub fn write_slot(source: Vec3, slot_index: usize, target: &mut Self) {
        Self::write_first(source, unsafe {
            GatherScatter::get_offset_instance_mut(target, slot_index)
        });
    }

    /// Gathers values from a vector and places them into the first indices of the target vector.
    pub fn write_first(source: Vec3, target_slot: &mut Self) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target_slot.x) = source.x;
            *GatherScatter::get_first_mut(&mut target_slot.y) = source.y;
            *GatherScatter::get_first_mut(&mut target_slot.z) = source.z;
        }
    }

    /// Expands each scalar value to every slot of the bundle.
    #[inline(always)]
    pub fn broadcast(source: Vec3) -> Self {
        Self {
            x: Vector::splat(source.x),
            y: Vector::splat(source.y),
            z: Vector::splat(source.z),
        }
    }

    /// Takes a slot from the source vector and broadcasts it into all slots of the target vector.
    #[inline(always)]
    pub fn broadcast_to(source: Vec3, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x);
        broadcasted.y = Vector::splat(source.y);
        broadcasted.z = Vector::splat(source.z);
    }

    /// Takes a slot from the source vector and broadcasts it into all slots of the target vector.
    #[inline(always)]
    pub fn rebroadcast(source: &Self, slot_index: usize) -> Self {
        Self {
            x: Vector::splat(source.x[slot_index]),
            y: Vector::splat(source.y[slot_index]),
            z: Vector::splat(source.z[slot_index]),
        }
    }

    /// Takes a slot from the source vector and broadcasts it into all slots of the target vector.
    #[inline(always)]
    pub fn rebroadcast_to(source: &Self, slot_index: usize, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x[slot_index]);
        broadcasted.y = Vector::splat(source.y[slot_index]);
        broadcasted.z = Vector::splat(source.z[slot_index]);
    }

    /// Takes a slot from the source vector and places it into a slot of the target.
    #[inline(always)]
    pub fn copy_slot(
        source: &Self,
        source_slot_index: usize,
        target: &mut Self,
        target_slot_index: usize,
    ) {
        unsafe {
            let source_slot = GatherScatter::get_offset_instance(source, source_slot_index);
            let target_slot = GatherScatter::get_offset_instance_mut(target, target_slot_index);
            *GatherScatter::get_first_mut(&mut target_slot.x) = source_slot.x[0];
            *GatherScatter::get_first_mut(&mut target_slot.y) = source_slot.y[0];
            *GatherScatter::get_first_mut(&mut target_slot.z) = source_slot.z[0];
        }
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

impl Add<Vector<f32>> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, s: Vector<f32>) -> Self {
        Self {
            x: self.x + s,
            y: self.y + s,
            z: self.z + s,
        }
    }
}

impl Add<Vector3Wide> for Vector<f32> {
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

impl Sub<Vector<f32>> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn sub(self, s: Vector<f32>) -> Self {
        Self {
            x: self.x - s,
            y: self.y - s,
            z: self.z - s,
        }
    }
}

impl Sub<Vector3Wide> for Vector<f32> {
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

impl Div<Vector<f32>> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: Vector<f32>) -> Self::Output {
        let inverse = Vector::<f32>::splat(1.0) / scalar;
        Self {
            x: self.x * inverse,
            y: self.y * inverse,
            z: self.z * inverse,
        }
    }
}

impl Mul<Vector<f32>> for Vector3Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: Vector<f32>) -> Self::Output {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Mul<Vector3Wide> for Vector<f32> {
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

impl Mul<&Vector3Wide> for Vector<f32> {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, vector: &Vector3Wide) -> Self::Output {
        Vector3Wide {
            x: vector.x * self,
            y: vector.y * self,
            z: vector.z * self,
        }
    }
}

impl Mul for &Vector3Wide {
    type Output = Vector3Wide;

    /// Multiplies the components of one vector with another.
    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        Vector3Wide {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
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
        write!(f, "<{:?}, {:?}, {:?}>", self.x, self.y, self.z)
    }
}
