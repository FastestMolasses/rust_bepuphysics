use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;
use glam::Vec2;
use std::simd::Mask;

/// Two dimensional vector with SIMD lanes.
#[derive(Clone, Copy, Debug)]
pub struct Vector2Wide {
    /// First component of the vector.
    pub x: Vector<f32>,
    /// Second component of the vector.
    pub y: Vector<f32>,
}

impl Vector2Wide {
    /// Performs a componentwise add between two vectors.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x + b.x;
        result.y = a.y + b.y;
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x - b.x;
        result.y = a.y - b.y;
    }

    #[inline(always)]
    pub fn dot(a: &Self, b: &Self, result: &mut Vector<f32>) {
        *result = a.x * b.x + a.y * b.y;
    }

    #[inline(always)]
    pub fn scale(vector: &Self, scalar: &Vector<f32>, result: &mut Self) {
        result.x = vector.x * scalar;
        result.y = vector.y * scalar;
    }

    #[inline(always)]
    pub fn negate(v: &Self, result: &mut Self) {
        result.x = -v.x;
        result.y = -v.y;
    }

    #[inline(always)]
    pub fn conditionally_negate(should_negate: &Vector<i32>, v: &mut Self) {
        let mask = Mask::from_int(*should_negate);
        v.x = mask.select(-v.x, v.x);
        v.y = mask.select(-v.y, v.y);
    }

    #[inline(always)]
    pub fn conditionally_negate_to(should_negate: &Vector<i32>, v: &Self, negated: &mut Self) {
        let mask = Mask::from_int(*should_negate);
        negated.x = mask.select(-v.x, v.x);
        negated.y = mask.select(-v.y, v.y);
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
    }

    #[inline(always)]
    pub fn length_squared(v: &Self, length_squared: &mut Vector<f32>) {
        *length_squared = v.x * v.x + v.y * v.y;
    }

    #[inline(always)]
    pub fn length(v: &Self, length: &mut Vector<f32>) {
        *length = std::simd::StdFloat::sqrt(v.x * v.x + v.y * v.y);
    }

    #[inline(always)]
    pub fn perp_dot(a: &Self, b: &Self, result: &mut Vector<f32>) {
        *result = a.y * b.x - a.x * b.y;
    }

    #[inline(always)]
    pub fn broadcast(source: &Vec2, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x);
        broadcasted.y = Vector::splat(source.y);
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Vec2) {
        target.x = source.x[0];
        target.y = source.y[0];
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Vec2) {
        let offset = unsafe { GatherScatter::get_offset_instance(wide, slot_index) };
        Self::read_first(offset, narrow);
    }

    /// Gathers values from a vector and places them into the first indices of the target vector.
    #[inline(always)]
    pub fn write_first(source: &Vec2, target_slot: &mut Self) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target_slot.x) = source.x;
            *GatherScatter::get_first_mut(&mut target_slot.y) = source.y;
        }
    }

    /// Writes a value into a slot of the target bundle.
    #[inline(always)]
    pub fn write_slot(source: &Vec2, slot_index: usize, target: &mut Self) {
        let offset = unsafe { GatherScatter::get_offset_instance_mut(target, slot_index) };
        Self::write_first(source, offset);
    }
}

impl std::ops::Add<Vector2Wide> for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Add<Vector<f32>> for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, scalar: Vector<f32>) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl std::ops::Add<Vector2Wide> for Vector<f32> {
    type Output = Vector2Wide;

    #[inline(always)]
    fn add(self, vector: Vector2Wide) -> Vector2Wide {
        Vector2Wide {
            x: vector.x + self,
            y: vector.y + self,
        }
    }
}

impl std::ops::Mul<Vector<f32>> for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: Vector<f32>) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl std::ops::Mul<Vector2Wide> for Vector<f32> {
    type Output = Vector2Wide;

    #[inline(always)]
    fn mul(self, vector: Vector2Wide) -> Vector2Wide {
        Vector2Wide {
            x: vector.x * self,
            y: vector.y * self,
        }
    }
}

impl std::fmt::Display for Vector2Wide {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "<{:?}, {:?}>", self.x, self.y)
    }
}
