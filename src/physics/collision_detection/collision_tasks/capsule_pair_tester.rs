// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CapsulePairTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex2ContactManifoldWide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for capsule vs capsule collisions.
pub struct CapsulePairTester;

impl CapsulePairTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests capsule vs capsule collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &CapsuleWide,
        b: &CapsuleWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _pair_count: i32,
        manifold: &mut Convex2ContactManifoldWide,
    ) {
        // Compute closest points between two line segments.
        let mut xa = Vector3Wide::default();
        let mut da = Vector3Wide::default();
        QuaternionWide::transform_unit_xy(orientation_a, &mut xa, &mut da);
        let db = QuaternionWide::transform_unit_y(orientation_b.clone());
        let mut da_offset_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&da, offset_b, &mut da_offset_b);
        let mut db_offset_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&db, offset_b, &mut db_offset_b);
        let mut dadb = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&da, &db, &mut dadb);
        // Note potential division by zero when axes are parallel.
        let mut ta = (da_offset_b - db_offset_b * dadb)
            / Vector::<f32>::splat(1e-15).simd_max(Vector::<f32>::splat(1.0) - dadb * dadb);
        let mut tb = ta * dadb - db_offset_b;

        // Project each line segment onto the other, clamping against the target's interval.
        let absdadb = dadb.abs();
        let b_onto_a_offset = b.half_length * absdadb;
        let a_onto_b_offset = a.half_length * absdadb;
        let mut a_min =
            (-a.half_length).simd_max((da_offset_b - b_onto_a_offset).simd_min(a.half_length));
        let mut a_max = a
            .half_length
            .simd_min((da_offset_b + b_onto_a_offset).simd_max(-a.half_length));
        let b_min =
            (-b.half_length).simd_max((-a_onto_b_offset - db_offset_b).simd_min(b.half_length));
        let b_max = b
            .half_length
            .simd_min((a_onto_b_offset - db_offset_b).simd_max(-b.half_length));
        ta = ta.simd_max(a_min).simd_min(a_max);
        tb = tb.simd_max(b_min).simd_min(b_max);

        let mut closest_point_on_a = Vector3Wide::default();
        Vector3Wide::scale_to(&da, &ta, &mut closest_point_on_a);
        let mut closest_point_on_b = Vector3Wide::default();
        Vector3Wide::scale_to(&db, &tb, &mut closest_point_on_b);
        closest_point_on_b = closest_point_on_b + *offset_b;
        // Normals point from B to A by convention.
        Vector3Wide::subtract(
            &closest_point_on_a,
            &closest_point_on_b,
            &mut manifold.normal,
        );
        let distance = manifold.normal.length();
        let inverse_distance = Vector::<f32>::splat(1.0) / distance;
        manifold.normal = Vector3Wide::scale(&manifold.normal, &inverse_distance);
        let normal_is_valid = distance.simd_gt(Vector::<f32>::splat(1e-7));
        manifold.normal =
            Vector3Wide::conditional_select(&normal_is_valid.to_int(), &manifold.normal, &xa);

        // Coplanarity-based interval weighting.
        let mut plane_normal = Vector3Wide::default();
        Vector3Wide::cross(&db, &manifold.normal, &mut plane_normal);
        let mut plane_normal_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&plane_normal, &mut plane_normal_length_squared);
        let mut numerator_unsquared = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&da, &plane_normal, &mut numerator_unsquared);
        let squared_angle = plane_normal_length_squared
            .simd_lt(Vector::<f32>::splat(1e-10))
            .select(
                Vector::<f32>::splat(0.0),
                numerator_unsquared * numerator_unsquared / plane_normal_length_squared,
            );

        const LOWER_THRESHOLD: f32 = 0.01 * 0.01;
        const UPPER_THRESHOLD: f32 = 0.05 * 0.05;
        let interval_weight =
            Vector::<f32>::splat(0.0).simd_max(Vector::<f32>::splat(1.0).simd_min(
                (Vector::<f32>::splat(UPPER_THRESHOLD) - squared_angle)
                    * Vector::<f32>::splat(1.0 / (UPPER_THRESHOLD - LOWER_THRESHOLD)),
            ));
        let weighted_ta = ta - ta * interval_weight;
        a_min = interval_weight * a_min + weighted_ta;
        a_max = interval_weight * a_max + weighted_ta;

        Vector3Wide::scale_to(&da, &a_min, &mut manifold.offset_a0);
        Vector3Wide::scale_to(&da, &a_max, &mut manifold.offset_a1);

        // Compute depths for both contacts.
        let mut db_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&db, &manifold.normal, &mut db_normal);
        let mut offset_b0 = Vector3Wide::default();
        Vector3Wide::subtract(&manifold.offset_a0, offset_b, &mut offset_b0);
        let mut offset_b1 = Vector3Wide::default();
        Vector3Wide::subtract(&manifold.offset_a1, offset_b, &mut offset_b1);
        let inverse_dadb = Vector::<f32>::splat(1.0) / dadb;
        let projected_tb0 = b_min.simd_max(b_max.simd_min((a_min - da_offset_b) * inverse_dadb));
        let projected_tb1 = b_min.simd_max(b_max.simd_min((a_max - da_offset_b) * inverse_dadb));
        let mut b0_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset_b0, &manifold.normal, &mut b0_normal);
        let mut b1_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset_b1, &manifold.normal, &mut b1_normal);
        let capsules_are_perpendicular = dadb.abs().simd_lt(Vector::<f32>::splat(1e-7));
        let distance0 =
            capsules_are_perpendicular.select(distance, b0_normal - db_normal * projected_tb0);
        let distance1 =
            capsules_are_perpendicular.select(distance, b1_normal - db_normal * projected_tb1);
        let combined_radius = a.radius + b.radius;
        manifold.depth0 = combined_radius - distance0;
        manifold.depth1 = combined_radius - distance1;

        // Apply normal offset to contact positions.
        let neg_offset0 = manifold.depth0 * Vector::<f32>::splat(0.5) - a.radius;
        let neg_offset1 = manifold.depth1 * Vector::<f32>::splat(0.5) - a.radius;
        let mut normal_push0 = Vector3Wide::default();
        Vector3Wide::scale_to(&manifold.normal, &neg_offset0, &mut normal_push0);
        let mut normal_push1 = Vector3Wide::default();
        Vector3Wide::scale_to(&manifold.normal, &neg_offset1, &mut normal_push1);
        manifold.offset_a0 = manifold.offset_a0 + normal_push0;
        manifold.offset_a1 = manifold.offset_a1 + normal_push1;
        manifold.feature_id0 = Vector::<i32>::splat(0);
        manifold.feature_id1 = Vector::<i32>::splat(1);
        let minimum_accepted_depth = -*speculative_margin;
        manifold.contact0_exists = manifold.depth0.simd_ge(minimum_accepted_depth).to_int();
        manifold.contact1_exists = manifold.depth1.simd_ge(minimum_accepted_depth).to_int()
            & (a_max - a_min)
                .simd_gt(Vector::<f32>::splat(1e-7) * a.half_length)
                .to_int();
    }
}
