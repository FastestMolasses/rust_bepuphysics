// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SphereTriangleTester.cs

use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::physics::collision_detection::mesh_reduction::FACE_COLLISION_FLAG;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for sphere vs triangle collisions.
pub struct SphereTriangleTester;

impl SphereTriangleTester {
    pub const BATCH_SIZE: i32 = 32;

    #[inline(always)]
    fn select(
        distance_squared: &mut Vector<f32>,
        local_normal: &mut Vector3Wide,
        distance_squared_candidate: &Vector<f32>,
        local_normal_candidate: &Vector3Wide,
    ) {
        let use_candidate = distance_squared_candidate.simd_lt(*distance_squared);
        *distance_squared = distance_squared.simd_min(*distance_squared_candidate);
        *local_normal = Vector3Wide::conditional_select(
            &use_candidate.to_int(),
            local_normal_candidate,
            local_normal,
        );
    }

    /// Tests sphere vs triangle collision (one orientation for the triangle).
    #[inline(always)]
    pub fn test(
        a: &SphereWide,
        b: &TriangleWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        // Work in the local space of the triangle.
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &r_b, &mut local_offset_b);

        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(&b.b, &b.a, &mut ab);
        let mut ac = Vector3Wide::default();
        Vector3Wide::subtract(&b.c, &b.a, &mut ac);
        // localOffsetA = -localOffsetB, so pa = triangle.A + localOffsetB
        let mut pa = Vector3Wide::default();
        Vector3Wide::add(&b.a, &local_offset_b, &mut pa);
        let mut local_triangle_normal = Vector3Wide::default();
        Vector3Wide::cross(&ab, &ac, &mut local_triangle_normal);
        let mut triangle_normal_length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&local_triangle_normal, &mut triangle_normal_length);
        let inverse_triangle_normal_length = Vector::<f32>::splat(1.0) / triangle_normal_length;
        local_triangle_normal =
            Vector3Wide::scale(&local_triangle_normal, &inverse_triangle_normal_length);

        // Edge plane tests using barycentric coordinates
        let mut paxab = Vector3Wide::default();
        Vector3Wide::cross(&pa, &ab, &mut paxab);
        let mut acxpa = Vector3Wide::default();
        Vector3Wide::cross(&ac, &pa, &mut acxpa);
        let mut edge_plane_test_ab = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&paxab, &local_triangle_normal, &mut edge_plane_test_ab);
        let mut edge_plane_test_ac = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&acxpa, &local_triangle_normal, &mut edge_plane_test_ac);
        let edge_plane_test_bc = Vector::<f32>::splat(1.0)
            - (edge_plane_test_ab + edge_plane_test_ac) * inverse_triangle_normal_length;

        let outside_ab = edge_plane_test_ab.simd_lt(Vector::<f32>::splat(0.0));
        let outside_ac = edge_plane_test_ac.simd_lt(Vector::<f32>::splat(0.0));
        let outside_bc = edge_plane_test_bc.simd_lt(Vector::<f32>::splat(0.0));

        let outside_any_edge = outside_ab | outside_ac | outside_bc;
        let mut local_closest_on_triangle = Vector3Wide::default();
        let neg_one = Vector::<i32>::splat(-1);
        let active_lanes = BundleIndexing::create_mask_for_count_in_bundle(pair_count as usize);

        if (active_lanes & outside_any_edge.to_int())
            .simd_eq(neg_one)
            .any()
        {
            // At least one lane detected a point outside of the triangle.
            // Choose one edge which is outside as the representative.
            let edge_direction_ac_or_ab =
                Vector3Wide::conditional_select(&outside_ac.to_int(), &ac, &ab);
            let mut bc = Vector3Wide::default();
            Vector3Wide::subtract(&b.c, &b.b, &mut bc);
            let edge_direction = Vector3Wide::conditional_select(
                &outside_bc.to_int(),
                &bc,
                &edge_direction_ac_or_ab,
            );
            let edge_start = Vector3Wide::conditional_select(&outside_bc.to_int(), &b.b, &b.a);

            let mut neg_edge_start_to_p = Vector3Wide::default();
            Vector3Wide::add(&local_offset_b, &edge_start, &mut neg_edge_start_to_p);
            let mut neg_offset_dot_edge = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(
                &neg_edge_start_to_p,
                &edge_direction,
                &mut neg_offset_dot_edge,
            );
            let mut edge_dot_edge = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&edge_direction, &edge_direction, &mut edge_dot_edge);
            let edge_scale = Vector::<f32>::splat(0.0)
                .simd_max(Vector::<f32>::splat(1.0).simd_min(-neg_offset_dot_edge / edge_dot_edge));
            let mut point_on_edge = Vector3Wide::default();
            Vector3Wide::scale_to(&edge_direction, &edge_scale, &mut point_on_edge);
            let mut point_on_edge_final = Vector3Wide::default();
            Vector3Wide::add(&edge_start, &point_on_edge, &mut point_on_edge_final);

            local_closest_on_triangle = Vector3Wide::conditional_select(
                &outside_any_edge.to_int(),
                &point_on_edge_final,
                &local_closest_on_triangle,
            );
        }
        if (active_lanes & !outside_any_edge.to_int())
            .simd_eq(neg_one)
            .any()
        {
            // p + N * (pa * N) / ||N||^2 = N * (pa * N) / ||N||^2 - (-p)
            let mut pa_n = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_triangle_normal, &pa, &mut pa_n);
            let mut offset_to_plane = Vector3Wide::default();
            Vector3Wide::scale_to(&local_triangle_normal, &pa_n, &mut offset_to_plane);
            let mut point_on_face = Vector3Wide::default();
            Vector3Wide::subtract(&offset_to_plane, &local_offset_b, &mut point_on_face);

            local_closest_on_triangle = Vector3Wide::conditional_select(
                &outside_any_edge.to_int(),
                &local_closest_on_triangle,
                &point_on_face,
            );
        }

        manifold.feature_id = outside_any_edge
            .to_int()
            .simd_eq(Vector::<i32>::splat(0))
            .select(
                Vector::<i32>::splat(FACE_COLLISION_FLAG),
                Vector::<i32>::splat(0),
            );

        // Contact position on mesh surface
        Matrix3x3Wide::transform_without_overlap(
            &local_closest_on_triangle,
            &r_b,
            &mut manifold.offset_a,
        );
        manifold.offset_a = manifold.offset_a + *offset_b;
        let mut distance = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&manifold.offset_a, &mut distance);
        // Normal calibrated from B to A.
        let normal_scale = Vector::<f32>::splat(-1.0) / distance;
        manifold.normal = Vector3Wide::scale(&manifold.offset_a, &normal_scale);
        manifold.depth = a.radius - distance;

        // Triangle degenerate and backface checks
        let mut face_normal_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &local_triangle_normal,
            &manifold.normal,
            &mut face_normal_dot,
        );
        let mut epsilon_scale = Vector::<f32>::splat(0.0);
        let mut nondegenerate_mask = Vector::<i32>::splat(0);
        let mut ab_len_sq = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ab, &mut ab_len_sq);
        let mut ac_len_sq = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ac, &mut ac_len_sq);
        TriangleWide::compute_nondegenerate_triangle_mask(
            &ab_len_sq,
            &ac_len_sq,
            &triangle_normal_length,
            &mut epsilon_scale,
            &mut nondegenerate_mask,
        );
        manifold.contact_exists = distance.simd_gt(Vector::<f32>::splat(0.0)).to_int()
            & nondegenerate_mask
            & face_normal_dot
                .simd_le(Vector::<f32>::splat(
                    -TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                ))
                .to_int()
            & manifold.depth.simd_ge(-*speculative_margin).to_int();
    }
}
