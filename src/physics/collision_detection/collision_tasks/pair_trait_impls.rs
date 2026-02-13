// Auto-generated trait implementations connecting pair testers and pair wide types
// to the generic convex collision task infrastructure.
//
// In C#, these trait impls are part of the struct declarations (e.g., `struct SpherePairTester : IPairTester<...>`).
// In Rust, we separate them here for clarity.

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::convex_hull::ConvexHullWide;
use crate::physics::collidables::cylinder::CylinderWide;
use crate::physics::collidables::shape::IShapeWideAllocation;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::{
    Convex1ContactManifoldWide, Convex2ContactManifoldWide, Convex4ContactManifoldWide,
};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

use super::convex_collision_task::IPairTester;
use super::pair_types::{
    CollisionPair, ConvexPairWide, FliplessPair, FliplessPairWide, ICollisionPairWide,
    SphereIncludingPair, SphereIncludingPairWide, SpherePair, SpherePairWide,
};

// ═══════════════════════════════════════════════════════════════════════════
// IPairTester impls — delegates to each tester's inherent `test()` method.
// Methods not used for a particular orientation count are unreachable.
// ═══════════════════════════════════════════════════════════════════════════

/// Macro to implement IPairTester for a tester with 0 orientations (sphere-sphere).
macro_rules! impl_pair_tester_0orient {
    ($tester:ty, $wide_a:ty, $wide_b:ty, $manifold:ty) => {
        impl IPairTester<$wide_a, $wide_b, $manifold> for $tester {
            fn batch_size() -> i32 {
                Self::BATCH_SIZE
            }
            fn test_two_orientations(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _orientation_a: &QuaternionWide,
                _orientation_b: &QuaternionWide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_two_orientations called on 0-orientation tester");
            }
            fn test_one_orientation(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _orientation_b: &QuaternionWide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_one_orientation called on 0-orientation tester");
            }
            fn test_no_orientation(
                a: &$wide_a,
                b: &$wide_b,
                speculative_margin: &Vector<f32>,
                offset_b: &Vector3Wide,
                pair_count: i32,
                manifold: &mut $manifold,
            ) {
                unsafe {
                    <$tester>::test(a, b, speculative_margin, offset_b, pair_count, manifold);
                }
            }
        }
    };
}

/// Macro to implement IPairTester for a tester with 1 orientation (sphere-including).
macro_rules! impl_pair_tester_1orient {
    ($tester:ty, $wide_a:ty, $wide_b:ty, $manifold:ty) => {
        impl IPairTester<$wide_a, $wide_b, $manifold> for $tester {
            fn batch_size() -> i32 {
                Self::BATCH_SIZE
            }
            fn test_two_orientations(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _orientation_a: &QuaternionWide,
                _orientation_b: &QuaternionWide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_two_orientations called on 1-orientation tester");
            }
            fn test_one_orientation(
                a: &$wide_a,
                b: &$wide_b,
                speculative_margin: &Vector<f32>,
                offset_b: &Vector3Wide,
                orientation_b: &QuaternionWide,
                pair_count: i32,
                manifold: &mut $manifold,
            ) {
                unsafe {
                    <$tester>::test(
                        a,
                        b,
                        speculative_margin,
                        offset_b,
                        orientation_b,
                        pair_count,
                        manifold,
                    );
                }
            }
            fn test_no_orientation(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_no_orientation called on 1-orientation tester");
            }
        }
    };
}

/// Macro to implement IPairTester for a tester with 2 orientations (standard pair).
macro_rules! impl_pair_tester_2orient {
    ($tester:ty, $wide_a:ty, $wide_b:ty, $manifold:ty) => {
        impl IPairTester<$wide_a, $wide_b, $manifold> for $tester {
            fn batch_size() -> i32 {
                Self::BATCH_SIZE
            }
            fn test_two_orientations(
                a: &$wide_a,
                b: &$wide_b,
                speculative_margin: &Vector<f32>,
                offset_b: &Vector3Wide,
                orientation_a: &QuaternionWide,
                orientation_b: &QuaternionWide,
                pair_count: i32,
                manifold: &mut $manifold,
            ) {
                unsafe {
                    <$tester>::test(
                        a,
                        b,
                        speculative_margin,
                        offset_b,
                        orientation_a,
                        orientation_b,
                        pair_count,
                        manifold,
                    );
                }
            }
            fn test_one_orientation(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _orientation_b: &QuaternionWide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_one_orientation called on 2-orientation tester");
            }
            fn test_no_orientation(
                _a: &$wide_a,
                _b: &$wide_b,
                _speculative_margin: &Vector<f32>,
                _offset_b: &Vector3Wide,
                _pair_count: i32,
                _manifold: &mut $manifold,
            ) {
                unreachable!("test_no_orientation called on 2-orientation tester");
            }
        }
    };
}

// ── 0 orientations ──
use super::sphere_pair_tester::SpherePairTester;
impl_pair_tester_0orient!(
    SpherePairTester,
    SphereWide,
    SphereWide,
    Convex1ContactManifoldWide
);

// ── 1 orientation ──
use super::sphere_box_tester::SphereBoxTester;
use super::sphere_capsule_tester::SphereCapsuleTester;
use super::sphere_convex_hull_tester::SphereConvexHullTester;
use super::sphere_cylinder_tester::SphereCylinderTester;
use super::sphere_triangle_tester::SphereTriangleTester;

impl_pair_tester_1orient!(
    SphereCapsuleTester,
    SphereWide,
    CapsuleWide,
    Convex1ContactManifoldWide
);
impl_pair_tester_1orient!(
    SphereBoxTester,
    SphereWide,
    BoxWide,
    Convex1ContactManifoldWide
);
impl_pair_tester_1orient!(
    SphereTriangleTester,
    SphereWide,
    TriangleWide,
    Convex1ContactManifoldWide
);
impl_pair_tester_1orient!(
    SphereCylinderTester,
    SphereWide,
    CylinderWide,
    Convex1ContactManifoldWide
);
impl_pair_tester_1orient!(
    SphereConvexHullTester,
    SphereWide,
    ConvexHullWide,
    Convex1ContactManifoldWide
);

// ── 2 orientations ──
use super::box_convex_hull_tester::BoxConvexHullTester;
use super::box_cylinder_tester::BoxCylinderTester;
use super::box_pair_tester::BoxPairTester;
use super::box_triangle_tester::BoxTriangleTester;
use super::capsule_box_tester::CapsuleBoxTester;
use super::capsule_convex_hull_tester::CapsuleConvexHullTester;
use super::capsule_cylinder_tester::CapsuleCylinderTester;
use super::capsule_pair_tester::CapsulePairTester;
use super::capsule_triangle_tester::CapsuleTriangleTester;
use super::convex_hull_pair_tester::ConvexHullPairTester;
use super::cylinder_convex_hull_tester::CylinderConvexHullTester;
use super::cylinder_pair_tester::CylinderPairTester;
use super::triangle_convex_hull_tester::TriangleConvexHullTester;
use super::triangle_cylinder_tester::TriangleCylinderTester;
use super::triangle_pair_tester::TrianglePairTester;

impl_pair_tester_2orient!(
    CapsulePairTester,
    CapsuleWide,
    CapsuleWide,
    Convex2ContactManifoldWide
);
impl_pair_tester_2orient!(
    CapsuleBoxTester,
    CapsuleWide,
    BoxWide,
    Convex2ContactManifoldWide
);
impl_pair_tester_2orient!(
    CapsuleTriangleTester,
    CapsuleWide,
    TriangleWide,
    Convex2ContactManifoldWide
);
impl_pair_tester_2orient!(
    CapsuleCylinderTester,
    CapsuleWide,
    CylinderWide,
    Convex2ContactManifoldWide
);
impl_pair_tester_2orient!(
    CapsuleConvexHullTester,
    CapsuleWide,
    ConvexHullWide,
    Convex2ContactManifoldWide
);
impl_pair_tester_2orient!(BoxPairTester, BoxWide, BoxWide, Convex4ContactManifoldWide);
impl_pair_tester_2orient!(
    BoxTriangleTester,
    BoxWide,
    TriangleWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    BoxCylinderTester,
    BoxWide,
    CylinderWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    BoxConvexHullTester,
    BoxWide,
    ConvexHullWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    TrianglePairTester,
    TriangleWide,
    TriangleWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    TriangleCylinderTester,
    TriangleWide,
    CylinderWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    TriangleConvexHullTester,
    TriangleWide,
    ConvexHullWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    CylinderPairTester,
    CylinderWide,
    CylinderWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    CylinderConvexHullTester,
    CylinderWide,
    ConvexHullWide,
    Convex4ContactManifoldWide
);
impl_pair_tester_2orient!(
    ConvexHullPairTester,
    ConvexHullWide,
    ConvexHullWide,
    Convex4ContactManifoldWide
);

// ═══════════════════════════════════════════════════════════════════════════
// ICollisionPairWide impls — connect wide pair types to the generic machinery.
// ═══════════════════════════════════════════════════════════════════════════

// ── SpherePairWide (0 orientations, no flip mask) ──
// SpherePairWide.a/b are Vector<f32> (scalar radius), but SphereWide is { radius: Vector<f32> }.
// They are memory-identical so we reinterpret via pointer cast.
impl ICollisionPairWide<SphereWide, SphereWide, SpherePair> for SpherePairWide {
    const HAS_FLIP_MASK: bool = false;
    const ORIENTATION_COUNT: i32 = 0;

    fn get_flip_mask(&self) -> &Vector<i32> {
        unreachable!("SpherePairWide has no flip mask")
    }
    fn get_speculative_margin(&self) -> &Vector<f32> {
        &self.speculative_margin
    }
    fn get_shape_a(&self) -> &SphereWide {
        unsafe { &*(&self.a as *const Vector<f32> as *const SphereWide) }
    }
    fn get_shape_b(&self) -> &SphereWide {
        unsafe { &*(&self.b as *const Vector<f32> as *const SphereWide) }
    }
    fn get_shape_a_mut(&mut self) -> &mut SphereWide {
        unsafe { &mut *(&mut self.a as *mut Vector<f32> as *mut SphereWide) }
    }
    fn get_shape_b_mut(&mut self) -> &mut SphereWide {
        unsafe { &mut *(&mut self.b as *mut Vector<f32> as *mut SphereWide) }
    }
    fn get_orientation_a(&self) -> &QuaternionWide {
        unreachable!("SpherePairWide has no orientation")
    }
    fn get_orientation_b(&self) -> &QuaternionWide {
        unreachable!("SpherePairWide has no orientation")
    }
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }

    fn write_slot(&mut self, index: i32, source: &SpherePair) {
        let idx = index as usize;
        Vector3Wide::write_slot(source.offset_b, idx, &mut self.offset_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.speculative_margin,
                idx,
            )) = source.speculative_margin;
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.a,
                idx,
            )) = source.a;
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.b,
                idx,
            )) = source.b;
        }
    }
}

// ── SphereIncludingPairWide (1 orientation, has flip mask) ──
// SphereIncludingPairWide.a is Vector<f32> (sphere radius) = SphereWide by memory layout.
impl<TShapeWide: Default + IShapeWideAllocation>
    ICollisionPairWide<SphereWide, TShapeWide, SphereIncludingPair>
    for SphereIncludingPairWide<TShapeWide>
{
    const HAS_FLIP_MASK: bool = true;
    const ORIENTATION_COUNT: i32 = 1;

    fn get_flip_mask(&self) -> &Vector<i32> {
        &self.flip_mask
    }
    fn get_speculative_margin(&self) -> &Vector<f32> {
        &self.speculative_margin
    }
    fn get_shape_a(&self) -> &SphereWide {
        unsafe { &*(&self.a as *const Vector<f32> as *const SphereWide) }
    }
    fn get_shape_b(&self) -> &TShapeWide {
        &self.b
    }
    fn get_shape_a_mut(&mut self) -> &mut SphereWide {
        unsafe { &mut *(&mut self.a as *mut Vector<f32> as *mut SphereWide) }
    }
    fn get_shape_b_mut(&mut self) -> &mut TShapeWide {
        &mut self.b
    }
    fn get_orientation_a(&self) -> &QuaternionWide {
        unreachable!("SphereIncludingPairWide has no orientation A")
    }
    fn get_orientation_b(&self) -> &QuaternionWide {
        &self.orientation_b
    }
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }

    fn write_slot(&mut self, index: i32, source: &SphereIncludingPair) {
        let idx = index as usize;
        Vector3Wide::write_slot(source.offset_b, idx, &mut self.offset_b);
        QuaternionWide::write_slot(source.orientation_b, idx, &mut self.orientation_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.speculative_margin,
                idx,
            )) = source.speculative_margin;
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.a,
                idx,
            )) = source.a;
            *(GatherScatter::get_offset_instance_mut(&mut self.flip_mask, idx) as *mut Vector<i32>
                as *mut i32) = source.flip_mask;
            // Shape B: copy scalar shape into wide lane
            self.b.write_slot_raw(idx, source.b);
        }
    }
}

// ── FliplessPairWide (2 orientations, no flip mask) ──
impl<TShapeWide: Default + IShapeWideAllocation>
    ICollisionPairWide<TShapeWide, TShapeWide, FliplessPair> for FliplessPairWide<TShapeWide>
{
    const HAS_FLIP_MASK: bool = false;
    const ORIENTATION_COUNT: i32 = 2;

    fn get_flip_mask(&self) -> &Vector<i32> {
        unreachable!("FliplessPairWide has no flip mask")
    }
    fn get_speculative_margin(&self) -> &Vector<f32> {
        &self.speculative_margin
    }
    fn get_shape_a(&self) -> &TShapeWide {
        &self.a
    }
    fn get_shape_b(&self) -> &TShapeWide {
        &self.b
    }
    fn get_shape_a_mut(&mut self) -> &mut TShapeWide {
        &mut self.a
    }
    fn get_shape_b_mut(&mut self) -> &mut TShapeWide {
        &mut self.b
    }
    fn get_orientation_a(&self) -> &QuaternionWide {
        &self.orientation_a
    }
    fn get_orientation_b(&self) -> &QuaternionWide {
        &self.orientation_b
    }
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }

    fn write_slot(&mut self, index: i32, source: &FliplessPair) {
        let idx = index as usize;
        Vector3Wide::write_slot(source.offset_b, idx, &mut self.offset_b);
        QuaternionWide::write_slot(source.orientation_a, idx, &mut self.orientation_a);
        QuaternionWide::write_slot(source.orientation_b, idx, &mut self.orientation_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.speculative_margin,
                idx,
            )) = source.speculative_margin;
            self.a.write_slot_raw(idx, source.a);
            self.b.write_slot_raw(idx, source.b);
        }
    }
}

// ── ConvexPairWide (2 orientations, has flip mask) ──
impl<TShapeWideA: Default + IShapeWideAllocation, TShapeWideB: Default + IShapeWideAllocation>
    ICollisionPairWide<TShapeWideA, TShapeWideB, CollisionPair>
    for ConvexPairWide<TShapeWideA, TShapeWideB>
{
    const HAS_FLIP_MASK: bool = true;
    const ORIENTATION_COUNT: i32 = 2;

    fn get_flip_mask(&self) -> &Vector<i32> {
        &self.flip_mask
    }
    fn get_speculative_margin(&self) -> &Vector<f32> {
        &self.speculative_margin
    }
    fn get_shape_a(&self) -> &TShapeWideA {
        &self.a
    }
    fn get_shape_b(&self) -> &TShapeWideB {
        &self.b
    }
    fn get_shape_a_mut(&mut self) -> &mut TShapeWideA {
        &mut self.a
    }
    fn get_shape_b_mut(&mut self) -> &mut TShapeWideB {
        &mut self.b
    }
    fn get_orientation_a(&self) -> &QuaternionWide {
        &self.orientation_a
    }
    fn get_orientation_b(&self) -> &QuaternionWide {
        &self.orientation_b
    }
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }

    fn write_slot(&mut self, index: i32, source: &CollisionPair) {
        let idx = index as usize;
        Vector3Wide::write_slot(source.offset_b, idx, &mut self.offset_b);
        QuaternionWide::write_slot(source.orientation_a, idx, &mut self.orientation_a);
        QuaternionWide::write_slot(source.orientation_b, idx, &mut self.orientation_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut *GatherScatter::get_offset_instance_mut(
                &mut self.speculative_margin,
                idx,
            )) = source.speculative_margin;
            *(GatherScatter::get_offset_instance_mut(&mut self.flip_mask, idx) as *mut Vector<i32>
                as *mut i32) = source.flip_mask;
            self.a.write_slot_raw(idx, source.a);
            self.b.write_slot_raw(idx, source.b);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Default impls — required by the convex collision task's batch execution.
// ═══════════════════════════════════════════════════════════════════════════

impl Default for SpherePairWide {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl<TShapeWide: Default> Default for SphereIncludingPairWide<TShapeWide> {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl<TShapeWide: Default> Default for FliplessPairWide<TShapeWide> {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl<TShapeWideA: Default, TShapeWideB: Default> Default
    for ConvexPairWide<TShapeWideA, TShapeWideB>
{
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl Default for Convex1ContactManifoldWide {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl Default for Convex2ContactManifoldWide {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

impl Default for Convex4ContactManifoldWide {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}
