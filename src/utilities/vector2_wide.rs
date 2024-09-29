use glam::Vec2;

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;

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
    pub fn scale(vector: &Self, scalar: Vector<f32>, result: &mut Self) {
        result.x = vector.x * scalar;
        result.y = vector.y * scalar;
    }

    #[inline(always)]
    pub fn negate(v: &Self, result: &mut Self) {
        result.x = -v.x;
        result.y = -v.y;
    }

    #[inline(always)]
    pub fn conditionally_negate(should_negate: i32x4, v: &mut Self) {
        // TODO:
        v.x = should_negate.select(-v.x, v.x);
        v.y = should_negate.select(-v.y, v.y);
    }

    #[inline(always)]
    pub fn conditionally_negate_to(should_negate: i32x4, v: &Self, negated: &mut Self) {
        // TODO:
        negated.x = should_negate.select(-v.x, v.x);
        negated.y = should_negate.select(-v.y, v.y);
    }

    #[inline(always)]
    pub fn conditional_select(condition: i32x4, left: &Self, right: &Self, result: &mut Self) {
        // TODO:
        result.x = condition.select(left.x, right.x);
        result.y = condition.select(left.y, right.y);
    }

    #[inline(always)]
    pub fn length_squared(v: &Self, length_squared: &mut Vector<f32>) {
        *length_squared = v.x * v.x + v.y * v.y;
    }

    #[inline(always)]
    pub fn length(v: &Self, length: &mut Vector<f32>) {
        *length = (v.x * v.x + v.y * v.y).sqrt();
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
        // TODO:
    }

    /// Pulls one lane out of the wide representation.
    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Vec2) {
        // TODO:
        let offset = GatherScatter::get_offset_instance(wide, slot_index);
        Self::read_first(offset, narrow);
    }

    /// Gathers values from a vector and places them into the first indices of the target vector.
    #[inline(always)]
    pub fn write_first(source: &Vec2, target_slot: &mut Self) {
        // TODO:
        GatherScatter::get_first(&mut target_slot.x).write(source.x);
        GatherScatter::get_first(&mut target_slot.y).write(source.y);
    }

    /// Writes a value into a slot of the target bundle.
    #[inline(always)]
    pub fn write_slot(source: &Vec2, slot_index: usize, target: &mut Self) {
        // TODO:
        let mut offset = GatherScatter::get_offset_instance(target, slot_index);
        Self::write_first(source, &mut offset);
    }
}

impl std::ops::Add for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Add<f32x4> for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn add(self, scalar: f32x4) -> Self {
        Self {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

impl std::ops::Mul<f32x4> for Vector2Wide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f32x4) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl std::ops::Mul<Vector2Wide> for f32x4 {
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
