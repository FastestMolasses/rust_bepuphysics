// Translated from BepuPhysics/CollisionDetection/CollisionTasks/PairTypes.cs

use crate::physics::collision_detection::collision_batcher_continuations::PairContinuation;
use crate::physics::collision_detection::collision_task_registry::CollisionTaskPairType;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::{Quat, Vec3};

// These pair types define the scalar data that the collision batcher writes into batch buffers.
// Different pair types carry different amounts of information:
// - CollisionPair: standard pair with flip mask and two orientations
// - FliplessPair: same-type pair with no flip mask
// - SpherePair: sphere vs sphere, no orientations needed
// - SphereIncludingPair: sphere vs other shape, one orientation
// - BoundsTestedPair: pair with velocity data for bounds testing

/// Trait for collision pair types that the batcher writes into batch buffers.
pub trait ICollisionPair: Copy {
    /// Gets the pair type enumeration value.
    fn pair_type() -> CollisionTaskPairType;
    /// Gets a reference to the continuation stored in this pair.
    fn get_continuation(&self) -> &PairContinuation;
    /// Gets a mutable reference to the continuation stored in this pair.
    fn get_continuation_mut(&mut self) -> &mut PairContinuation;
}

/// Standard collision pair with flip mask and two orientations.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CollisionPair {
    pub a: *const u8,
    pub b: *const u8,
    /// Stores whether the types involved in pair require that the resulting contact manifold be
    /// flipped to be consistent with the user-requested pair order.
    pub flip_mask: i32,
    pub offset_b: Vec3,
    pub orientation_a: Quat,
    pub orientation_b: Quat,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

impl ICollisionPair for CollisionPair {
    #[inline(always)]
    fn pair_type() -> CollisionTaskPairType {
        CollisionTaskPairType::StandardPair
    }
    #[inline(always)]
    fn get_continuation(&self) -> &PairContinuation {
        &self.continuation
    }
    #[inline(always)]
    fn get_continuation_mut(&mut self) -> &mut PairContinuation {
        &mut self.continuation
    }
}

/// Same-type pair with no flip mask needed.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FliplessPair {
    pub a: *const u8,
    pub b: *const u8,
    pub offset_b: Vec3,
    pub orientation_a: Quat,
    pub orientation_b: Quat,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

impl ICollisionPair for FliplessPair {
    #[inline(always)]
    fn pair_type() -> CollisionTaskPairType {
        CollisionTaskPairType::FliplessPair
    }
    #[inline(always)]
    fn get_continuation(&self) -> &PairContinuation {
        &self.continuation
    }
    #[inline(always)]
    fn get_continuation_mut(&mut self) -> &mut PairContinuation {
        &mut self.continuation
    }
}

/// Sphere-only pair—no orientations needed.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SpherePair {
    pub a: f32, // Sphere radius
    pub b: f32, // Sphere radius
    pub offset_b: Vec3,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

impl ICollisionPair for SpherePair {
    #[inline(always)]
    fn pair_type() -> CollisionTaskPairType {
        CollisionTaskPairType::SpherePair
    }
    #[inline(always)]
    fn get_continuation(&self) -> &PairContinuation {
        &self.continuation
    }
    #[inline(always)]
    fn get_continuation_mut(&mut self) -> &mut PairContinuation {
        &mut self.continuation
    }
}

/// Sphere vs non-sphere pair—one orientation for the non-sphere shape.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SphereIncludingPair {
    pub a: f32, // Sphere radius
    pub b: *const u8,
    pub flip_mask: i32,
    pub offset_b: Vec3,
    pub orientation_b: Quat,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

impl ICollisionPair for SphereIncludingPair {
    #[inline(always)]
    fn pair_type() -> CollisionTaskPairType {
        CollisionTaskPairType::SphereIncludingPair
    }
    #[inline(always)]
    fn get_continuation(&self) -> &PairContinuation {
        &self.continuation
    }
    #[inline(always)]
    fn get_continuation_mut(&mut self) -> &mut PairContinuation {
        &mut self.continuation
    }
}

/// Pair with velocity data for bounds testing (compound/mesh pairs).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BoundsTestedPair {
    pub a: *const u8,
    pub b: *const u8,
    pub flip_mask: i32,
    pub offset_b: Vec3,
    pub orientation_b: Quat,
    pub orientation_a: Quat,
    pub relative_linear_velocity_a: Vec3,
    pub angular_velocity_a: Vec3,
    pub angular_velocity_b: Vec3,
    pub maximum_expansion: f32,
    pub speculative_margin: f32,
    pub continuation: PairContinuation,
}

impl ICollisionPair for BoundsTestedPair {
    #[inline(always)]
    fn pair_type() -> CollisionTaskPairType {
        CollisionTaskPairType::BoundsTestedPair
    }
    #[inline(always)]
    fn get_continuation(&self) -> &PairContinuation {
        &self.continuation
    }
    #[inline(always)]
    fn get_continuation_mut(&mut self) -> &mut PairContinuation {
        &mut self.continuation
    }
}

/// Trait for wide (SIMD) pair types used in batch collision testing.
pub trait ICollisionPairWide<TShapeWideA, TShapeWideB, TPair: ICollisionPair> {
    const HAS_FLIP_MASK: bool;
    const ORIENTATION_COUNT: i32;

    fn get_flip_mask(&self) -> &Vector<i32>;
    fn get_speculative_margin(&self) -> &Vector<f32>;
    fn get_shape_a(&self) -> &TShapeWideA;
    fn get_shape_b(&self) -> &TShapeWideB;
    fn get_shape_a_mut(&mut self) -> &mut TShapeWideA;
    fn get_shape_b_mut(&mut self) -> &mut TShapeWideB;
    fn get_orientation_a(&self) -> &QuaternionWide;
    fn get_orientation_b(&self) -> &QuaternionWide;
    fn get_offset_b(&self) -> &Vector3Wide;
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide;

    /// Writes a scalar pair into the specified SIMD lane.
    fn write_slot(&mut self, index: i32, source: &TPair);
}

/// Wide pair for standard convex vs convex with two orientations.
#[repr(C)]
pub struct ConvexPairWide<TShapeWideA, TShapeWideB> {
    pub a: TShapeWideA,
    pub b: TShapeWideB,
    pub flip_mask: Vector<i32>,
    pub offset_b: Vector3Wide,
    pub orientation_a: QuaternionWide,
    pub orientation_b: QuaternionWide,
    pub speculative_margin: Vector<f32>,
}

/// Wide pair for same-type convex pairs (no flip mask).
#[repr(C)]
pub struct FliplessPairWide<TShapeWide> {
    pub a: TShapeWide,
    pub b: TShapeWide,
    pub offset_b: Vector3Wide,
    pub orientation_a: QuaternionWide,
    pub orientation_b: QuaternionWide,
    pub speculative_margin: Vector<f32>,
}

/// Wide pair for sphere-including pairs (one orientation).
#[repr(C)]
pub struct SphereIncludingPairWide<TShapeWide> {
    pub a: Vector<f32>, // SphereWide = just radius
    pub b: TShapeWide,
    pub flip_mask: Vector<i32>,
    pub offset_b: Vector3Wide,
    pub orientation_b: QuaternionWide,
    pub speculative_margin: Vector<f32>,
}

/// Wide pair for sphere vs sphere (no orientations).
#[repr(C)]
pub struct SpherePairWide {
    pub a: Vector<f32>, // SphereWide = radius
    pub b: Vector<f32>, // SphereWide = radius
    pub offset_b: Vector3Wide,
    pub speculative_margin: Vector<f32>,
}
