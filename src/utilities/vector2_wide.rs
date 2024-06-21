use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector2::Vector2;
use packed_simd::*;

#[derive(Clone, Copy, Debug)]
pub struct Vector2Wide {
    /// First component of the vector.
    pub x: f32x4,
    /// Second component of the vector.
    pub y: f32x4,
}

impl Vector2Wide {
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
    pub fn dot(a: &Self, b: &Self, result: &mut f32x4) {
        *result = a.x * b.x + a.y * b.y;
    }

    #[inline(always)]
    pub fn scale(vector: &Self, scalar: f32x4, result: &mut Self) {
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
        v.x = should_negate.select(-v.x, v.x);
        v.y = should_negate.select(-v.y, v.y);
    }

    #[inline(always)]
    pub fn conditionally_negate_to(should_negate: i32x4, v: &Self, negated: &mut Self) {
        negated.x = should_negate.select(-v.x, v.x);
        negated.y = should_negate.select(-v.y, v.y);
    }

    #[inline(always)]
    pub fn conditional_select(condition: i32x4, left: &Self, right: &Self, result: &mut Self) {
        result.x = condition.select(left.x, right.x);
        result.y = condition.select(left.y, right.y);
    }

    #[inline(always)]
    pub fn length_squared(v: &Self, length_squared: &mut f32x4) {
        *length_squared = v.x * v.x + v.y * v.y;
    }

    #[inline(always)]
    pub fn length(v: &Self, length: &mut f32x4) {
        *length = (v.x * v.x + v.y * v.y).sqrt();
    }

    #[inline(always)]
    pub fn perp_dot(a: &Self, b: &Self, result: &mut f32x4) {
        *result = a.y * b.x - a.x * b.y;
    }

    #[inline(always)]
    pub fn broadcast(source: &Vector2, broadcasted: &mut Self) {
        broadcasted.x = f32x4::splat(source.x);
        broadcasted.y = f32x4::splat(source.y);
    }

    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Vector2) {
        target.x = source.x.extract(0);
        target.y = source.y.extract(0);
    }

    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Vector2) {
        let offset = GatherScatter::get_offset_instance(wide, slot_index);
        Self::read_first(offset, narrow);
    }

    #[inline(always)]
    pub fn write_first(source: &Vector2, target_slot: &mut Self) {
        GatherScatter::get_first(&mut target_slot.x).write(source.x);
        GatherScatter::get_first(&mut target_slot.y).write(source.y);
    }

    #[inline(always)]
    pub fn write_slot(source: &Vector2, slot_index: usize, target: &mut Self) {
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
