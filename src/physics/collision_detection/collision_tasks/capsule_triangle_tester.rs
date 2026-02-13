// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CapsuleTriangleTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex2ContactManifoldWide;
use crate::physics::collision_detection::mesh_reduction::{
    FACE_COLLISION_FLAG, MINIMUM_DOT_FOR_FACE_COLLISION,
};
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for capsule vs triangle collisions.
pub struct CapsuleTriangleTester;

impl CapsuleTriangleTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests a triangle edge against the capsule axis, computing closest points, depth, and normal.
    #[inline(always)]
    pub fn test_edge(
        triangle: &TriangleWide,
        triangle_normal: &Vector3Wide,
        edge_start: &Vector3Wide,
        edge_offset: &Vector3Wide,
        capsule_center: &Vector3Wide,
        capsule_axis: &Vector3Wide,
        capsule_half_length: &Vector<f32>,
        edge_direction: &mut Vector3Wide,
        ta: &mut Vector<f32>,
        tb: &mut Vector<f32>,
        b_min: &mut Vector<f32>,
        b_max: &mut Vector<f32>,
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        let edge_length = edge_offset.length();
        Vector3Wide::scale_to(
            edge_offset,
            &(Vector::<f32>::splat(1.0) / edge_length),
            edge_direction,
        );
        let mut offset_b = Vector3Wide::default();
        Vector3Wide::subtract(edge_start, capsule_center, &mut offset_b);
        let mut da_offset_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(capsule_axis, &offset_b, &mut da_offset_b);
        let mut db_offset_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(edge_direction, &offset_b, &mut db_offset_b);
        let mut dadb = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(capsule_axis, edge_direction, &mut dadb);
        *ta = (da_offset_b - db_offset_b * dadb)
            / (Vector::<f32>::splat(1.0) - dadb * dadb).simd_max(Vector::<f32>::splat(1e-15));
        *tb = *ta * dadb - db_offset_b;

        let ta0 = (-*capsule_half_length).simd_max((*capsule_half_length).simd_min(da_offset_b));
        let ta1 = (*capsule_half_length)
            .simd_min((-*capsule_half_length).simd_max(da_offset_b + edge_length * dadb));
        let a_min = ta0.simd_min(ta1);
        let a_max = ta0.simd_max(ta1);
        let a_onto_b_offset = *capsule_half_length * dadb.abs();
        *b_min = Vector::<f32>::splat(0.0)
            .simd_max(edge_length.simd_min(-a_onto_b_offset - db_offset_b));
        *b_max =
            edge_length.simd_min(Vector::<f32>::splat(0.0).simd_max(a_onto_b_offset - db_offset_b));
        *ta = (*ta).simd_min(a_max).simd_max(a_min);
        *tb = (*tb).simd_min(*b_max).simd_max(*b_min);

        let mut closest_on_capsule = Vector3Wide::default();
        Vector3Wide::scale_to(capsule_axis, ta, &mut closest_on_capsule);
        closest_on_capsule = closest_on_capsule + *capsule_center;
        let mut closest_on_edge = Vector3Wide::default();
        Vector3Wide::scale_to(edge_direction, tb, &mut closest_on_edge);
        closest_on_edge = closest_on_edge + *edge_start;

        Vector3Wide::subtract(&closest_on_capsule, &closest_on_edge, normal);
        let mut normal_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(normal, &mut normal_length_squared);

        // Fallback: cross product of capsule axis and edge, calibrated to point away from triangle center.
        let mut fallback_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(capsule_axis, edge_offset, &mut fallback_normal)
        };
        let mut calibration_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&fallback_normal, capsule_center, &mut calibration_dot);
        Vector3Wide::conditionally_negate(
            &calibration_dot.simd_lt(Vector::<f32>::splat(0.0)).to_int(),
            &mut fallback_normal,
        );
        let mut fallback_normal_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&fallback_normal, &mut fallback_normal_length_squared);
        let use_fallback_normal = normal_length_squared.simd_lt(Vector::<f32>::splat(1e-13));
        *normal = Vector3Wide::conditional_select(
            &use_fallback_normal.to_int(),
            &fallback_normal,
            normal,
        );
        normal_length_squared =
            use_fallback_normal.select(fallback_normal_length_squared, normal_length_squared);

        // Second fallback: edge plane normal (triangle normal x edge).
        let mut second_fallback_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                triangle_normal,
                edge_offset,
                &mut second_fallback_normal,
            )
        };
        let mut second_fallback_length_squared = Vector::<f32>::splat(0.0);
        // Note: C# has a bug here using fallbackNormal instead of secondFallbackNormal for LengthSquared. We replicate the bug.
        Vector3Wide::length_squared_to(&fallback_normal, &mut second_fallback_length_squared);
        let use_second_fallback = normal_length_squared.simd_lt(Vector::<f32>::splat(1e-13));
        *normal = Vector3Wide::conditional_select(
            &use_second_fallback.to_int(),
            &second_fallback_normal,
            normal,
        );
        normal_length_squared =
            use_second_fallback.select(second_fallback_length_squared, normal_length_squared);

        *normal = Vector3Wide::scale(
            normal,
            &(Vector::<f32>::splat(1.0) / StdFloat::sqrt(normal_length_squared)),
        );

        // Compute depth including shape extent along normal.
        let mut n_axis = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(capsule_axis, normal, &mut n_axis);
        let mut n_capsule_center = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(normal, capsule_center, &mut n_capsule_center);
        let extreme_on_capsule = n_capsule_center - n_axis.abs() * *capsule_half_length;
        let mut na = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle.a, normal, &mut na);
        let mut nb = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle.b, normal, &mut nb);
        let mut nc = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle.c, normal, &mut nc);
        let extreme_on_triangle = na.simd_max(nb.simd_max(nc));
        *depth = extreme_on_triangle - extreme_on_capsule;
    }

    /// Clips the capsule axis against a triangle edge plane, returning entry/exit t values.
    #[inline(always)]
    pub fn clip_against_edge_plane(
        edge_start: &Vector3Wide,
        edge_offset: &Vector3Wide,
        face_normal: &Vector3Wide,
        capsule_center: &Vector3Wide,
        capsule_axis: &Vector3Wide,
        entry: &mut Vector<f32>,
        exit: &mut Vector<f32>,
    ) {
        let mut edge_plane_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(face_normal, edge_offset, &mut edge_plane_normal)
        };
        let mut edge_to_capsule = Vector3Wide::default();
        Vector3Wide::subtract(capsule_center, edge_start, &mut edge_to_capsule);
        let mut distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&edge_to_capsule, &edge_plane_normal, &mut distance);
        let mut velocity = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(capsule_axis, &edge_plane_normal, &mut velocity);
        let velocity_is_positive = velocity.simd_gt(Vector::<f32>::splat(0.0));
        let t_val = velocity_is_positive.select(-distance, distance)
            / velocity.abs().simd_max(Vector::<f32>::splat(1e-15));
        *entry = velocity_is_positive.select(Vector::<f32>::splat(-f32::MAX), t_val);
        *exit = velocity_is_positive.select(t_val, Vector::<f32>::splat(f32::MAX));
    }

    /// Tests capsule vs triangle collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &CapsuleWide,
        b: &TriangleWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex2ContactManifoldWide,
    ) {
        // Work in triangle local space, centered on the triangle centroid.
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let mut local_triangle_center = b.a + b.b;
        local_triangle_center = local_triangle_center + b.c;
        local_triangle_center =
            Vector3Wide::scale(&local_triangle_center, &Vector::<f32>::splat(1.0 / 3.0));
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &r_b, &mut local_offset_b);
        local_offset_b = local_offset_b + local_triangle_center;
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);
        let mut triangle = TriangleWide::default();
        Vector3Wide::subtract(&b.a, &local_triangle_center, &mut triangle.a);
        Vector3Wide::subtract(&b.b, &local_triangle_center, &mut triangle.b);
        Vector3Wide::subtract(&b.c, &local_triangle_center, &mut triangle.c);

        let world_capsule_axis = QuaternionWide::transform_unit_y(*orientation_a);
        let mut local_capsule_axis = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &world_capsule_axis,
            &r_b,
            &mut local_capsule_axis,
        );

        let mut ac = Vector3Wide::default();
        Vector3Wide::subtract(&b.c, &b.a, &mut ac);
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(&b.b, &b.a, &mut ab);

        let mut acxab = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&ac, &ab, &mut acxab) };
        let face_normal_length = acxab.length();
        let face_normal =
            Vector3Wide::scale(&acxab, &(Vector::<f32>::splat(1.0) / face_normal_length));

        // Face normal depth.
        let mut n_dot_axis = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&face_normal, &local_capsule_axis, &mut n_dot_axis);
        let mut capsule_offset_along_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &face_normal,
            &local_offset_a,
            &mut capsule_offset_along_normal,
        );
        let face_depth = a.half_length * n_dot_axis.abs() - capsule_offset_along_normal;

        // Test all three edges.
        let mut edge_direction = Vector3Wide::default();
        let mut ta = Vector::<f32>::splat(0.0);
        let mut tb = Vector::<f32>::splat(0.0);
        let mut b_min_val = Vector::<f32>::splat(0.0);
        let mut b_max_val = Vector::<f32>::splat(0.0);
        let mut edge_depth = Vector::<f32>::splat(0.0);
        let mut edge_normal = Vector3Wide::default();
        Self::test_edge(
            &triangle,
            &face_normal,
            &triangle.a,
            &ab,
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &mut edge_direction,
            &mut ta,
            &mut tb,
            &mut b_min_val,
            &mut b_max_val,
            &mut edge_depth,
            &mut edge_normal,
        );

        // Test AC edge.
        let mut edge_direction_cand = Vector3Wide::default();
        let mut ta_cand = Vector::<f32>::splat(0.0);
        let mut tb_cand = Vector::<f32>::splat(0.0);
        let mut b_min_cand = Vector::<f32>::splat(0.0);
        let mut b_max_cand = Vector::<f32>::splat(0.0);
        let mut edge_depth_cand = Vector::<f32>::splat(0.0);
        let mut edge_normal_cand = Vector3Wide::default();
        Self::test_edge(
            &triangle,
            &face_normal,
            &triangle.a,
            &ac,
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &mut edge_direction_cand,
            &mut ta_cand,
            &mut tb_cand,
            &mut b_min_cand,
            &mut b_max_cand,
            &mut edge_depth_cand,
            &mut edge_normal_cand,
        );
        let use_ac = edge_depth_cand.simd_lt(edge_depth);
        edge_direction = Vector3Wide::conditional_select(
            &use_ac.to_int(),
            &edge_direction_cand,
            &edge_direction,
        );
        edge_normal =
            Vector3Wide::conditional_select(&use_ac.to_int(), &edge_normal_cand, &edge_normal);
        ta = use_ac.select(ta_cand, ta);
        tb = use_ac.select(tb_cand, tb);
        b_min_val = use_ac.select(b_min_cand, b_min_val);
        b_max_val = use_ac.select(b_max_cand, b_max_val);
        edge_depth = edge_depth_cand.simd_min(edge_depth);

        // Test BC edge.
        let mut bc = Vector3Wide::default();
        Vector3Wide::subtract(&b.c, &b.b, &mut bc);
        Self::test_edge(
            &triangle,
            &face_normal,
            &triangle.b,
            &bc,
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &mut edge_direction_cand,
            &mut ta_cand,
            &mut tb_cand,
            &mut b_min_cand,
            &mut b_max_cand,
            &mut edge_depth_cand,
            &mut edge_normal_cand,
        );
        let use_bc = edge_depth_cand.simd_lt(edge_depth);
        let edge_start =
            Vector3Wide::conditional_select(&use_bc.to_int(), &triangle.b, &triangle.a);
        edge_direction = Vector3Wide::conditional_select(
            &use_bc.to_int(),
            &edge_direction_cand,
            &edge_direction,
        );
        edge_normal =
            Vector3Wide::conditional_select(&use_bc.to_int(), &edge_normal_cand, &edge_normal);
        ta = use_bc.select(ta_cand, ta);
        tb = use_bc.select(tb_cand, tb);
        b_min_val = use_bc.select(b_min_cand, b_min_val);
        b_max_val = use_bc.select(b_max_cand, b_max_val);
        edge_depth = edge_depth_cand.simd_min(edge_depth);

        let depth = edge_depth.simd_min(face_depth);
        let use_edge = edge_depth.simd_lt(face_depth);
        let local_normal =
            Vector3Wide::conditional_select(&use_edge.to_int(), &edge_normal, &face_normal);
        let mut local_normal_dot_face_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &local_normal,
            &face_normal,
            &mut local_normal_dot_face_normal,
        );
        let colliding_with_solid_side = local_normal_dot_face_normal.simd_ge(Vector::<f32>::splat(
            TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
        ));
        let active_lanes = BundleIndexing::create_mask_for_count_in_bundle(pair_count as usize);
        let mut _epsilon_scale = Vector::<f32>::splat(0.0);
        let mut nondegenerate_mask = Vector::<i32>::splat(0);
        let mut ab_length_sq = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ab, &mut ab_length_sq);
        let mut ca = Vector3Wide::default();
        Vector3Wide::negate(&ac, &mut ca);
        let mut ca_length_sq = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ca, &mut ca_length_sq);
        TriangleWide::compute_nondegenerate_triangle_mask(
            &ab_length_sq,
            &ca_length_sq,
            &face_normal_length,
            &mut _epsilon_scale,
            &mut nondegenerate_mask,
        );
        let negative_margin = -*speculative_margin;
        let allow_contacts = (depth + a.radius).simd_ge(negative_margin).to_int()
            & active_lanes
            & colliding_with_solid_side.to_int()
            & nondegenerate_mask;
        if allow_contacts.simd_eq(Vector::<i32>::splat(0)).all() {
            manifold.contact0_exists = Vector::<i32>::splat(0);
            manifold.contact1_exists = Vector::<i32>::splat(0);
            return;
        }

        let mut b0 = Vector3Wide::default();
        let mut b1 = Vector3Wide::default();
        let mut contact_count: Vector<i32>;
        let use_edge_masked = use_edge.to_int() & allow_contacts;
        if use_edge_masked.simd_lt(Vector::<i32>::splat(0)).any() {
            // Edge contact generation with coplanarity-based interval weighting.
            let mut plane_normal = Vector3Wide::default();
            unsafe {
                Vector3Wide::cross_without_overlap(&edge_direction, &edge_normal, &mut plane_normal)
            };
            let mut plane_normal_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&plane_normal, &mut plane_normal_length_squared);
            let mut numerator_unsquared = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_capsule_axis, &plane_normal, &mut numerator_unsquared);
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
            let weighted_tb = tb - tb * interval_weight;
            let b_min_weighted = interval_weight * b_min_val + weighted_tb;
            let b_max_weighted = interval_weight * b_max_val + weighted_tb;

            Vector3Wide::scale_to(&edge_direction, &b_min_weighted, &mut b0);
            b0 = b0 + edge_start;
            Vector3Wide::scale_to(&edge_direction, &b_max_weighted, &mut b1);
            b1 = b1 + edge_start;
            contact_count = use_edge_masked.simd_lt(Vector::<i32>::splat(0)).select(
                b_max_weighted
                    .simd_gt(b_min_weighted)
                    .select(Vector::<i32>::splat(2), Vector::<i32>::splat(1)),
                Vector::<i32>::splat(0),
            );
        } else {
            contact_count = Vector::<i32>::splat(0);
        }

        // Face contact generation via clipping.
        if ((contact_count.simd_le(Vector::<i32>::splat(1)).to_int()) & allow_contacts)
            .simd_lt(Vector::<i32>::splat(0))
            .any()
        {
            let mut ab_entry = Vector::<f32>::splat(0.0);
            let mut ab_exit = Vector::<f32>::splat(0.0);
            Self::clip_against_edge_plane(
                &triangle.a,
                &ab,
                &face_normal,
                &local_offset_a,
                &local_capsule_axis,
                &mut ab_entry,
                &mut ab_exit,
            );
            let mut bc_entry = Vector::<f32>::splat(0.0);
            let mut bc_exit = Vector::<f32>::splat(0.0);
            Self::clip_against_edge_plane(
                &triangle.b,
                &bc,
                &face_normal,
                &local_offset_a,
                &local_capsule_axis,
                &mut bc_entry,
                &mut bc_exit,
            );
            let mut ca_local = Vector3Wide::default();
            Vector3Wide::negate(&ac, &mut ca_local);
            let mut ca_entry = Vector::<f32>::splat(0.0);
            let mut ca_exit = Vector::<f32>::splat(0.0);
            Self::clip_against_edge_plane(
                &triangle.a,
                &ca_local,
                &face_normal,
                &local_offset_a,
                &local_capsule_axis,
                &mut ca_entry,
                &mut ca_exit,
            );
            let triangle_interval_min = ab_entry.simd_max(bc_entry.simd_max(ca_entry));
            let triangle_interval_max = ab_exit.simd_min(bc_exit.simd_min(ca_exit));

            let negative_half_length = -a.half_length;
            let overlap_interval_min_clamped = triangle_interval_min.simd_max(negative_half_length);
            let overlap_interval_max_clamped = triangle_interval_max.simd_min(a.half_length);
            let interval_is_valid =
                overlap_interval_max_clamped.simd_ge(overlap_interval_min_clamped);
            let overlap_interval_min_final = overlap_interval_min_clamped.simd_min(a.half_length);
            let overlap_interval_max_final =
                overlap_interval_max_clamped.simd_max(negative_half_length);

            let mut clipped_on_a0 =
                Vector3Wide::scale(&local_capsule_axis, &overlap_interval_min_final);
            clipped_on_a0 = clipped_on_a0 + local_offset_a;
            let mut distance_along_normal_a0 = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&clipped_on_a0, &face_normal, &mut distance_along_normal_a0);
            let to_remove_a0 = Vector3Wide::scale(&face_normal, &distance_along_normal_a0);
            let face_candidate0 = clipped_on_a0 - to_remove_a0;

            let mut clipped_on_a1 =
                Vector3Wide::scale(&local_capsule_axis, &overlap_interval_max_final);
            clipped_on_a1 = clipped_on_a1 + local_offset_a;
            let mut distance_along_normal_a1 = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&clipped_on_a1, &face_normal, &mut distance_along_normal_a1);
            let to_remove_a1 = Vector3Wide::scale(&face_normal, &distance_along_normal_a1);
            let face_candidate1 = clipped_on_a1 - to_remove_a1;

            let no_edge_contacts = contact_count.simd_eq(Vector::<i32>::splat(0));
            let allow_face_contacts =
                capsule_offset_along_normal.simd_ge(Vector::<f32>::splat(0.0));
            let use_face_contacts = no_edge_contacts.to_int() & allow_face_contacts.to_int();
            b0 = Vector3Wide::conditional_select(&use_face_contacts, &face_candidate0, &b0);
            b1 = Vector3Wide::conditional_select(&use_face_contacts, &face_candidate1, &b1);
            contact_count = use_face_contacts
                .simd_lt(Vector::<i32>::splat(0))
                .select(Vector::<i32>::splat(2), contact_count);

            // Second contact from face if only 1 edge contact.
            let use_face1_for_second = (overlap_interval_max_final - ta)
                .abs()
                .simd_gt((overlap_interval_min_final - ta).abs());
            let second_contact_candidate = Vector3Wide::conditional_select(
                &use_face1_for_second.to_int(),
                &face_candidate1,
                &face_candidate0,
            );
            let second_contact_distance =
                use_face1_for_second.select(distance_along_normal_a1, distance_along_normal_a0);
            let use_candidate_for_second = interval_is_valid.to_int()
                & contact_count.simd_eq(Vector::<i32>::splat(1)).to_int()
                & second_contact_distance
                    .simd_gt(Vector::<f32>::splat(0.0))
                    .to_int();
            b1 = Vector3Wide::conditional_select(
                &use_candidate_for_second,
                &second_contact_candidate,
                &b1,
            );
            contact_count = use_candidate_for_second
                .simd_lt(Vector::<i32>::splat(0))
                .select(Vector::<i32>::splat(2), contact_count);
        }

        // Per-contact depth via capsule face normal projection.
        let mut capsule_tangent = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                &local_normal,
                &local_capsule_axis,
                &mut capsule_tangent,
            )
        };
        let mut face_normal_a = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                &capsule_tangent,
                &local_capsule_axis,
                &mut face_normal_a,
            )
        };
        let mut face_normal_a_dot_local_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &face_normal_a,
            &local_normal,
            &mut face_normal_a_dot_local_normal,
        );
        let inverse_fn_dot = Vector::<f32>::splat(1.0) / face_normal_a_dot_local_normal;
        let offset0 = local_offset_b + b0;
        let offset1 = local_offset_b + b1;
        let mut t0 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset0, &face_normal_a, &mut t0);
        let mut t1 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset1, &face_normal_a, &mut t1);
        t0 = t0 * inverse_fn_dot;
        t1 = t1 * inverse_fn_dot;
        manifold.depth0 = a.radius + t0;
        manifold.depth1 = a.radius + t1;

        let collapse = face_normal_a_dot_local_normal
            .abs()
            .simd_lt(Vector::<f32>::splat(1e-7));
        manifold.depth0 = collapse.select(a.radius + depth, manifold.depth0);
        contact_count = colliding_with_solid_side.select(contact_count, Vector::<i32>::splat(0));
        manifold.contact0_exists = allow_contacts
            & contact_count.simd_gt(Vector::<i32>::splat(0)).to_int()
            & manifold.depth0.simd_gt(negative_margin).to_int();
        manifold.contact1_exists = allow_contacts
            & (contact_count.simd_eq(Vector::<i32>::splat(2)).to_int() & (!collapse.to_int()))
            & manifold.depth1.simd_gt(negative_margin).to_int();

        // Feature IDs based on capsule-axis projection order.
        let mut local_offset_a0 = Vector3Wide::default();
        Vector3Wide::subtract(&b0, &local_offset_a, &mut local_offset_a0);
        let mut local_offset_a1 = Vector3Wide::default();
        Vector3Wide::subtract(&b1, &local_offset_a, &mut local_offset_a1);
        let mut ta0 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&local_offset_a0, &local_capsule_axis, &mut ta0);
        let mut ta1 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&local_offset_a1, &local_capsule_axis, &mut ta1);
        let flip_feature_ids = ta1.simd_lt(ta0);
        manifold.feature_id0 =
            flip_feature_ids.select(Vector::<i32>::splat(1), Vector::<i32>::splat(0));
        manifold.feature_id1 =
            flip_feature_ids.select(Vector::<i32>::splat(0), Vector::<i32>::splat(1));

        let face_flag = local_normal_dot_face_normal
            .simd_ge(Vector::<f32>::splat(MINIMUM_DOT_FOR_FACE_COLLISION))
            .select(
                Vector::<i32>::splat(FACE_COLLISION_FLAG),
                Vector::<i32>::splat(0),
            );
        manifold.feature_id0 = manifold.feature_id0 + face_flag;

        // Transform to world space.
        Matrix3x3Wide::transform_without_overlap(&local_offset_a0, &r_b, &mut manifold.offset_a0);
        Matrix3x3Wide::transform_without_overlap(&local_offset_a1, &r_b, &mut manifold.offset_a1);
        Matrix3x3Wide::transform_without_overlap(&local_normal, &r_b, &mut manifold.normal);
    }
}
