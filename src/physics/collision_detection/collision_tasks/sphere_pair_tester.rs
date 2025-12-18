// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SpherePairTester.cs

use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for sphere vs sphere collisions.
pub struct SpherePairTester;

impl SpherePairTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests sphere vs sphere collision (no orientations needed).
    #[inline(always)]
    pub fn test(
        a: &SphereWide,
        b: &SphereWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        _pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        let center_distance = offset_b.length();
        // Note the negative 1. By convention, the normal points from B to A.
        let inverse_distance = Vector::<f32>::splat(-1.0) / center_distance;
        Vector3Wide::scale_to(offset_b, &inverse_distance, &mut manifold.normal);
        let normal_is_valid = center_distance.simd_gt(Vector::<f32>::splat(0.0));
        // Arbitrarily choose (0,1,0) if the two spheres are in the same position.
        manifold.normal.x = normal_is_valid.select(manifold.normal.x, Vector::<f32>::splat(0.0));
        manifold.normal.y = normal_is_valid.select(manifold.normal.y, Vector::<f32>::splat(1.0));
        manifold.normal.z = normal_is_valid.select(manifold.normal.z, Vector::<f32>::splat(0.0));
        manifold.depth = a.radius + b.radius - center_distance;

        // The contact position relative to object A is computed as the average of the extreme points.
        let negative_offset_from_a = manifold.depth * Vector::<f32>::splat(0.5) - a.radius;
        Vector3Wide::scale_to(&manifold.normal, &negative_offset_from_a, &mut manifold.offset_a);
        manifold.contact_exists = manifold.depth.simd_gt(-*speculative_margin).to_int();
        manifold.feature_id = Vector::<i32>::splat(0);
    }
}
