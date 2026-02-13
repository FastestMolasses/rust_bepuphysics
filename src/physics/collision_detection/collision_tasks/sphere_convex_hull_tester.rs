// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SphereConvexHullTester.cs

use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collidables::sphere::{SphereSupportFinder, SphereWide};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for sphere vs convex hull collisions.
pub struct SphereConvexHullTester;

impl SphereConvexHullTester {
    pub const BATCH_SIZE: i32 = 16;

    /// Tests sphere vs convex hull collision (one orientation for the hull).
    #[inline(always)]
    pub unsafe fn test(
        a: &SphereWide,
        b: &ConvexHullWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);

        let mut hull_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut hull_orientation);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            &hull_orientation,
            &mut local_offset_b,
        );
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);
        let identity = Matrix3x3Wide::create_identity();
        let mut center_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&local_offset_a, &mut center_distance);
        let mut initial_normal = Vector3Wide::default();
        Vector3Wide::scale_to(
            &local_offset_a,
            &(one_f / center_distance),
            &mut initial_normal,
        );
        let use_initial_fallback = center_distance.simd_lt(Vector::<f32>::splat(1e-8));
        initial_normal.x = use_initial_fallback.select(zero_f, initial_normal.x);
        initial_normal.y = use_initial_fallback.select(one_f, initial_normal.y);
        initial_normal.z = use_initial_fallback.select(zero_f, initial_normal.z);

        let hull_support_finder = ConvexHullSupportFinder;
        let sphere_support_finder = SphereSupportFinder;
        let inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut hull_epsilon_scale = Vector::<f32>::splat(0.0);
        b.estimate_epsilon_scale(&inactive_lanes, &mut hull_epsilon_scale);
        let epsilon_scale = a.radius.simd_min(hull_epsilon_scale);

        let mut depth = Vector::<f32>::splat(0.0);
        let mut local_normal = Vector3Wide::default();
        let mut closest_on_hull = Vector3Wide::default();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &identity,
            &hull_support_finder,
            &sphere_support_finder,
            &initial_normal,
            &inactive_lanes,
            &(Vector::<f32>::splat(1e-5) * epsilon_scale),
            &(-*speculative_margin),
            &mut depth,
            &mut local_normal,
            &mut closest_on_hull,
            25,
        );

        let mut hull_to_contact = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(
            &closest_on_hull,
            &hull_orientation,
            &mut hull_to_contact,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &hull_orientation,
            &mut manifold.normal,
        );
        Vector3Wide::add(&hull_to_contact, offset_b, &mut manifold.offset_a);

        manifold.feature_id = Vector::<i32>::splat(0);
        manifold.depth = depth;
        manifold.contact_exists = manifold.depth.simd_ge(-*speculative_margin).to_int();
    }
}
