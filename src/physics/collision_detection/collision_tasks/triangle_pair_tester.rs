// Translated from BepuPhysics/CollisionDetection/CollisionTasks/TrianglePairTester.cs

use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidate, ManifoldCandidateHelper,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::mesh_reduction::{
    FACE_COLLISION_FLAG, MINIMUM_DOT_FOR_FACE_COLLISION,
};
use crate::physics::helpers::Helpers;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for triangle vs triangle collisions.
pub struct TrianglePairTester;

impl TrianglePairTester {
    pub const BATCH_SIZE: i32 = 32;

    #[inline(always)]
    fn get_interval_for_normal(
        a: &Vector3Wide,
        b: &Vector3Wide,
        c: &Vector3Wide,
        normal: &Vector3Wide,
        min: &mut Vector<f32>,
        max: &mut Vector<f32>,
    ) {
        let mut d_a = Vector::<f32>::splat(0.0);
        let mut d_b = Vector::<f32>::splat(0.0);
        let mut d_c = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(normal, a, &mut d_a);
        Vector3Wide::dot(normal, b, &mut d_b);
        Vector3Wide::dot(normal, c, &mut d_c);
        *min = d_a.simd_min(d_b.simd_min(d_c));
        *max = d_a.simd_max(d_b.simd_max(d_c));
    }

    #[inline(always)]
    fn get_depth_for_normal(
        a_a: &Vector3Wide,
        b_a: &Vector3Wide,
        c_a: &Vector3Wide,
        a_b: &Vector3Wide,
        b_b: &Vector3Wide,
        c_b: &Vector3Wide,
        normal: &Vector3Wide,
        depth: &mut Vector<f32>,
    ) {
        let mut min_a = Vector::<f32>::splat(0.0);
        let mut max_a = Vector::<f32>::splat(0.0);
        let mut min_b = Vector::<f32>::splat(0.0);
        let mut max_b = Vector::<f32>::splat(0.0);
        Self::get_interval_for_normal(a_a, b_a, c_a, normal, &mut min_a, &mut max_a);
        Self::get_interval_for_normal(a_b, b_b, c_b, normal, &mut min_b, &mut max_b);
        *depth = (max_a - min_b).simd_min(max_b - min_a);
    }

    #[inline(always)]
    fn test_edge_edge(
        edge_direction_a: &Vector3Wide,
        edge_direction_b: &Vector3Wide,
        a_a: &Vector3Wide,
        b_a: &Vector3Wide,
        c_a: &Vector3Wide,
        a_b: &Vector3Wide,
        b_b: &Vector3Wide,
        c_b: &Vector3Wide,
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        unsafe {
            Vector3Wide::cross_without_overlap(edge_direction_a, edge_direction_b, normal);
        }
        let normal_length = normal.length();
        let inv_len = Vector::<f32>::splat(1.0) / normal_length;
        *normal = Vector3Wide::scale(normal, &inv_len);
        Self::get_depth_for_normal(a_a, b_a, c_a, a_b, b_b, c_b, normal, depth);
        // Protect against degenerate normals (parallel edges).
        *depth = normal_length
            .simd_lt(Vector::<f32>::splat(1e-10))
            .select(Vector::<f32>::splat(f32::MAX), *depth);
    }

    #[inline(always)]
    fn select(
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        depth_candidate: &Vector<f32>,
        normal_candidate: &Vector3Wide,
    ) {
        let use_candidate = depth_candidate.simd_lt(*depth);
        *normal = Vector3Wide::conditional_select(
            &use_candidate.to_int(),
            normal_candidate,
            normal,
        );
        *depth = depth.simd_min(*depth_candidate);
    }

    #[inline(always)]
    fn select_components(
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        depth_candidate: &Vector<f32>,
        nx: &Vector<f32>,
        ny: &Vector<f32>,
        nz: &Vector<f32>,
    ) {
        let use_candidate = depth_candidate.simd_lt(*depth);
        normal.x = use_candidate.select(*nx, normal.x);
        normal.y = use_candidate.select(*ny, normal.y);
        normal.z = use_candidate.select(*nz, normal.z);
        *depth = depth.simd_min(*depth_candidate);
    }

    #[inline(always)]
    fn try_add_triangle_a_vertex(
        vertex: &Vector3Wide,
        flattened_vertex: &Vector2Wide,
        vertex_id: &Vector<i32>,
        tangent_bx: &Vector3Wide,
        tangent_by: &Vector3Wide,
        triangle_center_b: &Vector3Wide,
        contact_normal: &Vector3Wide,
        face_normal_b: &Vector3Wide,
        edge_ab: &Vector2Wide,
        edge_bc: &Vector2Wide,
        edge_ca: &Vector2Wide,
        b_a: &Vector2Wide,
        b_b: &Vector2Wide,
        allow_contacts: &Vector<i32>,
        inverse_contact_normal_dot_face_normal_b: &Vector<f32>,
        minimum_depth: &Vector<f32>,
        candidates: &mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        // Test edge plane sign for all 3 edges of B.
        let mut ba_to_vertex = Vector2Wide::default();
        Vector2Wide::subtract(flattened_vertex, b_a, &mut ba_to_vertex);
        let mut bb_to_vertex = Vector2Wide::default();
        Vector2Wide::subtract(flattened_vertex, b_b, &mut bb_to_vertex);
        let ab_edge_plane_dot = ba_to_vertex.y * edge_ab.x - ba_to_vertex.x * edge_ab.y;
        let bc_edge_plane_dot = bb_to_vertex.y * edge_bc.x - bb_to_vertex.x * edge_bc.y;
        let ca_edge_plane_dot = ba_to_vertex.y * edge_ca.x - ba_to_vertex.x * edge_ca.y;
        let ab_contained = ab_edge_plane_dot.simd_gt(Vector::<f32>::splat(0.0));
        let bc_contained = bc_edge_plane_dot.simd_gt(Vector::<f32>::splat(0.0));
        let ca_contained = ca_edge_plane_dot.simd_gt(Vector::<f32>::splat(0.0));
        let contained = ab_contained & bc_contained & ca_contained;

        // Cast ray from A's vertex along contact normal to plane of triangle B.
        // t = (vertexOnA - triangleCenterB) · faceNormalB / (contactNormal · faceNormalB)
        let offset = *triangle_center_b - *vertex;
        let mut distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset, face_normal_b, &mut distance);
        let mut candidate = ManifoldCandidate::default();
        candidate.depth = distance * *inverse_contact_normal_dot_face_normal_b;
        let unprojected_vertex =
            Vector3Wide::scale(contact_normal, &candidate.depth) + *vertex;

        let offset_on_b = unprojected_vertex - *triangle_center_b;
        Vector3Wide::dot(&offset_on_b, tangent_bx, &mut candidate.x);
        Vector3Wide::dot(&offset_on_b, tangent_by, &mut candidate.y);
        candidate.feature_id = *vertex_id;
        let exists = (candidate.depth.simd_ge(*minimum_depth) & contained)
            .to_int()
            & *allow_contacts;
        unsafe {
            ManifoldCandidateHelper::add_candidate_with_depth(
                candidates as *mut ManifoldCandidate,
                candidate_count,
                &candidate,
                &exists,
                pair_count,
            );
        }
    }

    fn clip_edge(
        edge_start_b: &Vector2Wide,
        edge_offset_b: &Vector2Wide,
        edge_start_a: &Vector2Wide,
        edge_offset_a: &Vector2Wide,
        inverse_edge_length_squared_a: &Vector<f32>,
        edge_start_a_dot_normal: &Vector<f32>,
        edge_offset_a_dot_normal: &Vector<f32>,
        intersection_exists: &mut Vector<i32>,
        t_b: &mut Vector<f32>,
        depth_contribution_a: &mut Vector<f32>,
    ) {
        let edge_plane_normal_dot = (edge_start_a.x - edge_start_b.x) * edge_offset_a.y
            - (edge_start_a.y - edge_start_b.y) * edge_offset_a.x;
        let velocity = edge_offset_b.x * edge_offset_a.y - edge_offset_b.y * edge_offset_a.x;
        let parallel_threshold = Vector::<f32>::splat(1e-20);
        let parallel = velocity.abs().simd_lt(parallel_threshold);
        let denominator = parallel.select(
            velocity
                .simd_lt(Vector::<f32>::splat(0.0))
                .select(-parallel_threshold, parallel_threshold),
            velocity,
        );
        *t_b = edge_plane_normal_dot / denominator;
        // Intersection must be within edge A bounds.
        let intersection_point_x = *t_b * edge_offset_b.x + edge_start_b.x;
        let intersection_point_y = *t_b * edge_offset_b.y + edge_start_b.y;
        let t_a = ((intersection_point_x - edge_start_a.x) * edge_offset_a.x
            + (intersection_point_y - edge_start_a.y) * edge_offset_a.y)
            * *inverse_edge_length_squared_a;
        *intersection_exists = (t_a.simd_ge(Vector::<f32>::splat(0.0))
            & t_a.simd_le(Vector::<f32>::splat(1.0)))
        .to_int();
        *depth_contribution_a = *edge_start_a_dot_normal + *edge_offset_a_dot_normal * t_a;
    }

    #[allow(clippy::too_many_arguments)]
    fn clip_b_edge_against_a_bounds(
        a_a: &Vector2Wide,
        a_b: &Vector2Wide,
        a_c: &Vector2Wide,
        edge_offset_ab_on_a: &Vector2Wide,
        edge_offset_bc_on_a: &Vector2Wide,
        edge_offset_ca_on_a: &Vector2Wide,
        inverse_edge_offset_ab_on_a_length_squared: &Vector<f32>,
        inverse_edge_offset_bc_on_a_length_squared: &Vector<f32>,
        inverse_edge_offset_ca_on_a_length_squared: &Vector<f32>,
        a_dot_normal_on_a: &Vector<f32>,
        b_dot_normal_on_a: &Vector<f32>,
        c_dot_normal_on_a: &Vector<f32>,
        ab_dot_normal_on_a: &Vector<f32>,
        bc_dot_normal_on_a: &Vector<f32>,
        ca_dot_normal_on_a: &Vector<f32>,
        flat_edge_start_b: &Vector2Wide,
        flat_edge_offset_b: &Vector2Wide,
        edge_start_b: &Vector3Wide,
        edge_offset_b: &Vector3Wide,
        entry_id: &Vector<i32>,
        exit_id_offset: &Vector<i32>,
        triangle_center_b: &Vector3Wide,
        tangent_bx: &Vector3Wide,
        tangent_by: &Vector3Wide,
        local_normal: &Vector3Wide,
        minimum_depth: &Vector<f32>,
        mut allow_contacts: Vector<i32>,
        candidates: &mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let mut ab_intersected = Vector::<i32>::splat(0);
        let mut t_ab = Vector::<f32>::splat(0.0);
        let mut depth_contribution_ab_on_a = Vector::<f32>::splat(0.0);
        let mut bc_intersected = Vector::<i32>::splat(0);
        let mut t_bc = Vector::<f32>::splat(0.0);
        let mut depth_contribution_bc_on_a = Vector::<f32>::splat(0.0);
        let mut ca_intersected = Vector::<i32>::splat(0);
        let mut t_ca = Vector::<f32>::splat(0.0);
        let mut depth_contribution_ca_on_a = Vector::<f32>::splat(0.0);

        Self::clip_edge(
            flat_edge_start_b,
            flat_edge_offset_b,
            a_a,
            edge_offset_ab_on_a,
            inverse_edge_offset_ab_on_a_length_squared,
            a_dot_normal_on_a,
            ab_dot_normal_on_a,
            &mut ab_intersected,
            &mut t_ab,
            &mut depth_contribution_ab_on_a,
        );
        Self::clip_edge(
            flat_edge_start_b,
            flat_edge_offset_b,
            a_b,
            edge_offset_bc_on_a,
            inverse_edge_offset_bc_on_a_length_squared,
            b_dot_normal_on_a,
            bc_dot_normal_on_a,
            &mut bc_intersected,
            &mut t_bc,
            &mut depth_contribution_bc_on_a,
        );
        Self::clip_edge(
            flat_edge_start_b,
            flat_edge_offset_b,
            a_c,
            edge_offset_ca_on_a,
            inverse_edge_offset_ca_on_a_length_squared,
            c_dot_normal_on_a,
            ca_dot_normal_on_a,
            &mut ca_intersected,
            &mut t_ca,
            &mut depth_contribution_ca_on_a,
        );

        let min_value = Vector::<f32>::splat(f32::MIN);
        let max_value = Vector::<f32>::splat(f32::MAX);
        let ab_mask = ab_intersected.simd_lt(Vector::<i32>::splat(0));
        let bc_mask = bc_intersected.simd_lt(Vector::<i32>::splat(0));
        let ca_mask = ca_intersected.simd_lt(Vector::<i32>::splat(0));
        let entry_ab = ab_mask.select(t_ab, max_value);
        let entry_bc = bc_mask.select(t_bc, max_value);
        let entry_ca = ca_mask.select(t_ca, max_value);
        let exit_ab = ab_mask.select(t_ab, min_value);
        let exit_bc = bc_mask.select(t_bc, min_value);
        let exit_ca = ca_mask.select(t_ca, min_value);
        let entry = entry_ab.simd_min(entry_bc.simd_min(entry_ca));
        let exit = exit_ab.simd_max(exit_bc.simd_max(exit_ca));
        let use_ab_as_entry = entry.simd_eq(t_ab);
        let use_bc_as_entry = entry.simd_eq(t_bc);
        let use_ab_as_exit = exit.simd_eq(t_ab);
        let use_bc_as_exit = exit.simd_eq(t_bc);
        let depth_contribution_a_at_entry = use_ab_as_entry.select(
            depth_contribution_ab_on_a,
            use_bc_as_entry.select(depth_contribution_bc_on_a, depth_contribution_ca_on_a),
        );
        let depth_contribution_a_at_exit = use_ab_as_exit.select(
            depth_contribution_ab_on_a,
            use_bc_as_exit.select(depth_contribution_bc_on_a, depth_contribution_ca_on_a),
        );
        // If an edge fails to generate any interval, it's not intersecting.
        allow_contacts =
            allow_contacts & !(entry.simd_eq(min_value) | exit.simd_eq(max_value)).to_int();
        let entry = entry.simd_max(Vector::<f32>::splat(0.0));
        let exit = exit.simd_min(Vector::<f32>::splat(1.0));

        let mut edge_start_b_dot_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(edge_start_b, local_normal, &mut edge_start_b_dot_normal);
        let mut edge_offset_b_dot_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(edge_offset_b, local_normal, &mut edge_offset_b_dot_normal);
        let depth_contribution_b_at_entry =
            edge_start_b_dot_normal + entry * edge_offset_b_dot_normal;
        let depth_contribution_b_at_exit =
            edge_start_b_dot_normal + exit * edge_offset_b_dot_normal;

        let offset = *edge_start_b - *triangle_center_b;
        let mut offset_x = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset, tangent_bx, &mut offset_x);
        let mut offset_y = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset, tangent_by, &mut offset_y);
        let mut edge_direction_x = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(tangent_bx, edge_offset_b, &mut edge_direction_x);
        let mut edge_direction_y = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(tangent_by, edge_offset_b, &mut edge_direction_y);

        let six = Vector::<i32>::splat(6);
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let interval_threshold = Vector::<f32>::splat(1e-5);

        // Entry contact
        let mut candidate = ManifoldCandidate::default();
        candidate.depth = depth_contribution_b_at_entry - depth_contribution_a_at_entry;
        let exists = (allow_contacts.simd_lt(Vector::<i32>::splat(0)))
            & (candidate.depth.simd_ge(*minimum_depth))
            & (candidate_count.simd_lt(six))
            & ((exit - entry).simd_ge(interval_threshold))
            & (entry.simd_lt(one_f))
            & (entry.simd_gt(zero_f));
        candidate.x = entry * edge_direction_x + offset_x;
        candidate.y = entry * edge_direction_y + offset_y;
        candidate.feature_id = *entry_id;
        unsafe {
            ManifoldCandidateHelper::add_candidate_with_depth(
                candidates as *mut ManifoldCandidate,
                candidate_count,
                &candidate,
                &exists.to_int(),
                pair_count,
            );
        }

        // Exit contact
        candidate.depth = depth_contribution_b_at_exit - depth_contribution_a_at_exit;
        let exists = (allow_contacts.simd_lt(Vector::<i32>::splat(0)))
            & (candidate.depth.simd_ge(*minimum_depth))
            & (candidate_count.simd_lt(six))
            & (exit.simd_ge(entry))
            & (exit.simd_le(one_f))
            & (exit.simd_ge(zero_f));
        candidate.x = exit * edge_direction_x + offset_x;
        candidate.y = exit * edge_direction_y + offset_y;
        candidate.feature_id = *entry_id + *exit_id_offset;
        unsafe {
            ManifoldCandidateHelper::add_candidate_with_depth(
                candidates as *mut ManifoldCandidate,
                candidate_count,
                &candidate,
                &exists.to_int(),
                pair_count,
            );
        }
    }

    #[inline(always)]
    fn transform_contact_to_manifold(
        raw_contact: &ManifoldCandidate,
        face_center_b: &Vector3Wide,
        tangent_bx: &Vector3Wide,
        tangent_by: &Vector3Wide,
        manifold_offset_a: &mut Vector3Wide,
        manifold_depth: &mut Vector<f32>,
        manifold_feature_id: &mut Vector<i32>,
    ) {
        *manifold_offset_a = Vector3Wide::scale(tangent_bx, &raw_contact.x);
        let y = Vector3Wide::scale(tangent_by, &raw_contact.y);
        *manifold_offset_a = *manifold_offset_a + y;
        *manifold_offset_a = *manifold_offset_a + *face_center_b;
        *manifold_depth = raw_contact.depth;
        *manifold_feature_id = raw_contact.feature_id;
    }

    /// Tests triangle vs triangle collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &TriangleWide,
        b: &TriangleWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        unsafe {
            // Transform into A's local space.
            let mut world_ra = Matrix3x3Wide::default();
            Matrix3x3Wide::create_from_quaternion(orientation_a, &mut world_ra);
            let mut world_rb = Matrix3x3Wide::default();
            Matrix3x3Wide::create_from_quaternion(orientation_b, &mut world_rb);
            let mut r_b = Matrix3x3Wide::default();
            Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_rb, &world_ra, &mut r_b);
            let mut local_offset_b = Vector3Wide::default();
            Matrix3x3Wide::transform_by_transposed_without_overlap(
                offset_b, &world_ra, &mut local_offset_b,
            );
            let mut b_a = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.a, &r_b, &mut b_a);
            b_a = b_a + local_offset_b;
            let mut b_b = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.b, &r_b, &mut b_b);
            b_b = b_b + local_offset_b;
            let mut b_c = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.c, &r_b, &mut b_c);
            b_c = b_c + local_offset_b;

            let local_triangle_center_b =
                Vector3Wide::scale(&(b_a + b_b + b_c), &Vector::<f32>::splat(1.0 / 3.0));

            let ab_b = b_b - b_a;
            let bc_b = b_c - b_b;
            let ca_b = b_a - b_c;

            let local_triangle_center_a =
                Vector3Wide::scale(&(a.a + a.b + a.c), &Vector::<f32>::splat(1.0 / 3.0));

            let ab_a = a.b - a.a;
            let bc_a = a.c - a.b;
            let ca_a = a.a - a.c;

            // 9 edge-edge SAT tests
            // A AB x *
            let mut depth = Vector::<f32>::splat(0.0);
            let mut local_normal = Vector3Wide::default();
            Self::test_edge_edge(
                &ab_a, &ab_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth, &mut local_normal,
            );
            let mut depth_candidate = Vector::<f32>::splat(0.0);
            let mut local_normal_candidate = Vector3Wide::default();
            Self::test_edge_edge(
                &ab_a, &bc_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);
            Self::test_edge_edge(
                &ab_a, &ca_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);

            // A BC x *
            Self::test_edge_edge(
                &bc_a, &ab_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);
            Self::test_edge_edge(
                &bc_a, &bc_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);
            Self::test_edge_edge(
                &bc_a, &ca_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);

            // A CA x *
            Self::test_edge_edge(
                &ca_a, &ab_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);
            Self::test_edge_edge(
                &ca_a, &bc_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);
            Self::test_edge_edge(
                &ca_a, &ca_b, &a.a, &a.b, &a.c, &b_a, &b_b, &b_c,
                &mut depth_candidate, &mut local_normal_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &local_normal_candidate);

            // Face normals
            let mut face_normal_a = Vector3Wide::default();
            Vector3Wide::cross_without_overlap(&ab_a, &ca_a, &mut face_normal_a);
            let face_normal_a_length = face_normal_a.length();
            face_normal_a = Vector3Wide::scale(
                &face_normal_a,
                &(Vector::<f32>::splat(1.0) / face_normal_a_length),
            );
            Self::get_depth_for_normal(
                &a.a, &a.b, &a.c, &b_a, &b_b, &b_c, &face_normal_a, &mut depth_candidate,
            );
            Self::select(&mut depth, &mut local_normal, &depth_candidate, &face_normal_a);

            let mut face_normal_b = Vector3Wide::default();
            Vector3Wide::cross_without_overlap(&ab_b, &ca_b, &mut face_normal_b);
            let face_normal_b_length = face_normal_b.length();
            face_normal_b = Vector3Wide::scale(
                &face_normal_b,
                &(Vector::<f32>::splat(1.0) / face_normal_b_length),
            );
            let mut face_depth_b = Vector::<f32>::splat(0.0);
            Self::get_depth_for_normal(
                &a.a, &a.b, &a.c, &b_a, &b_b, &b_c, &face_normal_b, &mut face_depth_b,
            );
            Self::select(&mut depth, &mut local_normal, &face_depth_b, &face_normal_b);

            let mut ab_a_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ab_a, &mut ab_a_length_squared);
            let mut ab_b_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ab_b, &mut ab_b_length_squared);
            let mut ca_a_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ca_a, &mut ca_a_length_squared);
            let mut ca_b_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ca_b, &mut ca_b_length_squared);
            let mut allow_contacts = BundleIndexing::create_mask_for_count_in_bundle(pair_count as usize);

            // Calibrate normal to point from B to A.
            let center_a_to_center_b = local_triangle_center_b - local_triangle_center_a;
            let mut calibration_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_normal, &center_a_to_center_b, &mut calibration_dot);
            let should_flip = calibration_dot.simd_gt(Vector::<f32>::splat(0.0));
            local_normal.x = should_flip.select(-local_normal.x, local_normal.x);
            local_normal.y = should_flip.select(-local_normal.y, local_normal.y);
            local_normal.z = should_flip.select(-local_normal.z, local_normal.z);

            let minimum_depth = -*speculative_margin;
            let mut local_normal_dot_face_normal_a = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_normal, &face_normal_a, &mut local_normal_dot_face_normal_a);
            let mut local_normal_dot_face_normal_b = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_normal, &face_normal_b, &mut local_normal_dot_face_normal_b);
            let mut epsilon_scale_a = Vector::<f32>::splat(0.0);
            let mut nondegenerate_mask_a = Vector::<i32>::splat(0);
            TriangleWide::compute_nondegenerate_triangle_mask(
                &ab_a_length_squared,
                &ca_a_length_squared,
                &face_normal_a_length,
                &mut epsilon_scale_a,
                &mut nondegenerate_mask_a,
            );
            let mut epsilon_scale_b = Vector::<f32>::splat(0.0);
            let mut nondegenerate_mask_b = Vector::<i32>::splat(0);
            TriangleWide::compute_nondegenerate_triangle_mask(
                &ab_b_length_squared,
                &ca_b_length_squared,
                &face_normal_b_length,
                &mut epsilon_scale_b,
                &mut nondegenerate_mask_b,
            );
            allow_contacts = (nondegenerate_mask_a & nondegenerate_mask_b)
                & ((depth.simd_ge(minimum_depth)).to_int() & allow_contacts)
                & (local_normal_dot_face_normal_a
                    .simd_lt(Vector::<f32>::splat(
                        -TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                    ))
                    .to_int()
                    & local_normal_dot_face_normal_b
                        .simd_gt(Vector::<f32>::splat(
                            TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                        ))
                        .to_int());

            if allow_contacts.simd_eq(Vector::<i32>::splat(0)).all() {
                manifold.contact0_exists = Vector::<i32>::splat(0);
                manifold.contact1_exists = Vector::<i32>::splat(0);
                manifold.contact2_exists = Vector::<i32>::splat(0);
                manifold.contact3_exists = Vector::<i32>::splat(0);
                return;
            }

            // Flatten both triangles onto a plane with the detected local normal.
            let mut flatten_x = Vector3Wide::default();
            let mut flatten_y = Vector3Wide::default();
            Helpers::build_orthonormal_basis(&local_normal, &mut flatten_x, &mut flatten_y);

            let mut flat_vertex_a_on_a = Vector2Wide::default();
            let mut flat_vertex_b_on_a = Vector2Wide::default();
            let mut flat_vertex_c_on_a = Vector2Wide::default();
            Vector3Wide::dot(&a.a, &flatten_x, &mut flat_vertex_a_on_a.x);
            Vector3Wide::dot(&a.a, &flatten_y, &mut flat_vertex_a_on_a.y);
            Vector3Wide::dot(&a.b, &flatten_x, &mut flat_vertex_b_on_a.x);
            Vector3Wide::dot(&a.b, &flatten_y, &mut flat_vertex_b_on_a.y);
            Vector3Wide::dot(&a.c, &flatten_x, &mut flat_vertex_c_on_a.x);
            Vector3Wide::dot(&a.c, &flatten_y, &mut flat_vertex_c_on_a.y);

            let mut flat_vertex_a_on_b = Vector2Wide::default();
            let mut flat_vertex_b_on_b = Vector2Wide::default();
            let mut flat_vertex_c_on_b = Vector2Wide::default();
            Vector3Wide::dot(&b_a, &flatten_x, &mut flat_vertex_a_on_b.x);
            Vector3Wide::dot(&b_a, &flatten_y, &mut flat_vertex_a_on_b.y);
            Vector3Wide::dot(&b_b, &flatten_x, &mut flat_vertex_b_on_b.x);
            Vector3Wide::dot(&b_b, &flatten_y, &mut flat_vertex_b_on_b.y);
            Vector3Wide::dot(&b_c, &flatten_x, &mut flat_vertex_c_on_b.x);
            Vector3Wide::dot(&b_c, &flatten_y, &mut flat_vertex_c_on_b.y);

            let mut flat_edge_ab_on_a = Vector2Wide::default();
            let mut flat_edge_bc_on_a = Vector2Wide::default();
            let mut flat_edge_ca_on_a = Vector2Wide::default();
            Vector2Wide::subtract(&flat_vertex_b_on_a, &flat_vertex_a_on_a, &mut flat_edge_ab_on_a);
            Vector2Wide::subtract(&flat_vertex_c_on_a, &flat_vertex_b_on_a, &mut flat_edge_bc_on_a);
            Vector2Wide::subtract(&flat_vertex_a_on_a, &flat_vertex_c_on_a, &mut flat_edge_ca_on_a);
            let mut flat_edge_ab_on_b = Vector2Wide::default();
            let mut flat_edge_bc_on_b = Vector2Wide::default();
            let mut flat_edge_ca_on_b = Vector2Wide::default();
            Vector2Wide::subtract(&flat_vertex_b_on_b, &flat_vertex_a_on_b, &mut flat_edge_ab_on_b);
            Vector2Wide::subtract(&flat_vertex_c_on_b, &flat_vertex_b_on_b, &mut flat_edge_bc_on_b);
            Vector2Wide::subtract(&flat_vertex_a_on_b, &flat_vertex_c_on_b, &mut flat_edge_ca_on_b);

            let edge_threshold = Vector::<f32>::splat(0.2);
            let use_edge_case_for_a =
                local_normal_dot_face_normal_a.abs().simd_lt(edge_threshold);
            let use_edge_case_for_b =
                local_normal_dot_face_normal_b.abs().simd_lt(edge_threshold);
            let use_face_case_for_b = !use_edge_case_for_b;
            let use_face_case_for_b =
                (allow_contacts.simd_lt(Vector::<i32>::splat(0))) & use_face_case_for_b;

            // Build tangent basis on triangle B surface for contact parameterization.
            let tangent_bx = Vector3Wide::scale(
                &ab_b,
                &(Vector::<f32>::splat(1.0)
                    / std::simd::StdFloat::sqrt(ab_b_length_squared)),
            );
            let mut tangent_by = Vector3Wide::default();
            Vector3Wide::cross_without_overlap(&tangent_bx, &face_normal_b, &mut tangent_by);

            // Allocate up to 6 candidates.
            let mut buffer = [ManifoldCandidate::default(); 6];
            let mut candidate_count = Vector::<i32>::splat(0);

            if use_face_case_for_b.any() {
                // Add A vertices contained within B's face.
                let inverse_contact_normal_dot_face_normal_b =
                    Vector::<f32>::splat(1.0) / local_normal_dot_face_normal_b;
                Self::try_add_triangle_a_vertex(
                    &a.a, &flat_vertex_a_on_a, &Vector::<i32>::splat(0),
                    &tangent_bx, &tangent_by, &local_triangle_center_b, &local_normal,
                    &face_normal_b, &flat_edge_ab_on_b, &flat_edge_bc_on_b, &flat_edge_ca_on_b,
                    &flat_vertex_a_on_b, &flat_vertex_b_on_b, &use_face_case_for_b.to_int(),
                    &inverse_contact_normal_dot_face_normal_b, &minimum_depth,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
                Self::try_add_triangle_a_vertex(
                    &a.b, &flat_vertex_b_on_a, &Vector::<i32>::splat(1),
                    &tangent_bx, &tangent_by, &local_triangle_center_b, &local_normal,
                    &face_normal_b, &flat_edge_ab_on_b, &flat_edge_bc_on_b, &flat_edge_ca_on_b,
                    &flat_vertex_a_on_b, &flat_vertex_b_on_b, &use_face_case_for_b.to_int(),
                    &inverse_contact_normal_dot_face_normal_b, &minimum_depth,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
                Self::try_add_triangle_a_vertex(
                    &a.c, &flat_vertex_c_on_a, &Vector::<i32>::splat(2),
                    &tangent_bx, &tangent_by, &local_triangle_center_b, &local_normal,
                    &face_normal_b, &flat_edge_ab_on_b, &flat_edge_bc_on_b, &flat_edge_ca_on_b,
                    &flat_vertex_a_on_b, &flat_vertex_b_on_b, &use_face_case_for_b.to_int(),
                    &inverse_contact_normal_dot_face_normal_b, &minimum_depth,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
            }

            // Edge clipping contacts.
            let three = Vector::<i32>::splat(3);
            let still_could_use_clipping_contacts =
                allow_contacts & candidate_count.simd_lt(three).to_int();
            if still_could_use_clipping_contacts
                .simd_lt(Vector::<i32>::splat(0))
                .any()
            {
                let mut flat_edge_offset_ab_on_a_length_squared = Vector::<f32>::splat(0.0);
                let mut flat_edge_offset_bc_on_a_length_squared = Vector::<f32>::splat(0.0);
                let mut flat_edge_offset_ca_on_a_length_squared = Vector::<f32>::splat(0.0);
                Vector2Wide::length_squared(
                    &flat_edge_ab_on_a, &mut flat_edge_offset_ab_on_a_length_squared,
                );
                Vector2Wide::length_squared(
                    &flat_edge_bc_on_a, &mut flat_edge_offset_bc_on_a_length_squared,
                );
                Vector2Wide::length_squared(
                    &flat_edge_ca_on_a, &mut flat_edge_offset_ca_on_a_length_squared,
                );
                let inverse_flat_edge_offset_ab_on_a_length_squared =
                    Vector::<f32>::splat(1.0) / flat_edge_offset_ab_on_a_length_squared;
                let inverse_flat_edge_offset_bc_on_a_length_squared =
                    Vector::<f32>::splat(1.0) / flat_edge_offset_bc_on_a_length_squared;
                let inverse_flat_edge_offset_ca_on_a_length_squared =
                    Vector::<f32>::splat(1.0) / flat_edge_offset_ca_on_a_length_squared;

                let mut a_dot_normal_on_a = Vector::<f32>::splat(0.0);
                let mut b_dot_normal_on_a = Vector::<f32>::splat(0.0);
                let mut c_dot_normal_on_a = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&local_normal, &a.a, &mut a_dot_normal_on_a);
                Vector3Wide::dot(&local_normal, &a.b, &mut b_dot_normal_on_a);
                Vector3Wide::dot(&local_normal, &a.c, &mut c_dot_normal_on_a);
                let mut ab_dot_normal_on_a = Vector::<f32>::splat(0.0);
                let mut bc_dot_normal_on_a = Vector::<f32>::splat(0.0);
                let mut ca_dot_normal_on_a = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&local_normal, &ab_a, &mut ab_dot_normal_on_a);
                Vector3Wide::dot(&local_normal, &bc_a, &mut bc_dot_normal_on_a);
                Vector3Wide::dot(&local_normal, &ca_a, &mut ca_dot_normal_on_a);

                Self::clip_b_edge_against_a_bounds(
                    &flat_vertex_a_on_a, &flat_vertex_b_on_a, &flat_vertex_c_on_a,
                    &flat_edge_ab_on_a, &flat_edge_bc_on_a, &flat_edge_ca_on_a,
                    &inverse_flat_edge_offset_ab_on_a_length_squared,
                    &inverse_flat_edge_offset_bc_on_a_length_squared,
                    &inverse_flat_edge_offset_ca_on_a_length_squared,
                    &a_dot_normal_on_a, &b_dot_normal_on_a, &c_dot_normal_on_a,
                    &ab_dot_normal_on_a, &bc_dot_normal_on_a, &ca_dot_normal_on_a,
                    &flat_vertex_a_on_b, &flat_edge_ab_on_b, &b_a, &ab_b,
                    &Vector::<i32>::splat(3), &three,
                    &local_triangle_center_b, &tangent_bx, &tangent_by,
                    &local_normal, &minimum_depth, still_could_use_clipping_contacts,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
                Self::clip_b_edge_against_a_bounds(
                    &flat_vertex_a_on_a, &flat_vertex_b_on_a, &flat_vertex_c_on_a,
                    &flat_edge_ab_on_a, &flat_edge_bc_on_a, &flat_edge_ca_on_a,
                    &inverse_flat_edge_offset_ab_on_a_length_squared,
                    &inverse_flat_edge_offset_bc_on_a_length_squared,
                    &inverse_flat_edge_offset_ca_on_a_length_squared,
                    &a_dot_normal_on_a, &b_dot_normal_on_a, &c_dot_normal_on_a,
                    &ab_dot_normal_on_a, &bc_dot_normal_on_a, &ca_dot_normal_on_a,
                    &flat_vertex_b_on_b, &flat_edge_bc_on_b, &b_b, &bc_b,
                    &Vector::<i32>::splat(4), &three,
                    &local_triangle_center_b, &tangent_bx, &tangent_by,
                    &local_normal, &minimum_depth, still_could_use_clipping_contacts,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
                Self::clip_b_edge_against_a_bounds(
                    &flat_vertex_a_on_a, &flat_vertex_b_on_a, &flat_vertex_c_on_a,
                    &flat_edge_ab_on_a, &flat_edge_bc_on_a, &flat_edge_ca_on_a,
                    &inverse_flat_edge_offset_ab_on_a_length_squared,
                    &inverse_flat_edge_offset_bc_on_a_length_squared,
                    &inverse_flat_edge_offset_ca_on_a_length_squared,
                    &a_dot_normal_on_a, &b_dot_normal_on_a, &c_dot_normal_on_a,
                    &ab_dot_normal_on_a, &bc_dot_normal_on_a, &ca_dot_normal_on_a,
                    &flat_vertex_c_on_b, &flat_edge_ca_on_b, &b_c, &ca_b,
                    &Vector::<i32>::splat(5), &three,
                    &local_triangle_center_b, &tangent_bx, &tangent_by,
                    &local_normal, &minimum_depth, still_could_use_clipping_contacts,
                    &mut buffer[0], &mut candidate_count, pair_count,
                );
            }

            // Reduce to 4 contacts.
            let epsilon_scale = epsilon_scale_a.simd_min(epsilon_scale_b);
            let mut contact0 = ManifoldCandidate::default();
            let mut contact1 = ManifoldCandidate::default();
            let mut contact2 = ManifoldCandidate::default();
            let mut contact3 = ManifoldCandidate::default();
            ManifoldCandidateHelper::reduce_without_computing_depths(
                &mut buffer[0] as *mut ManifoldCandidate,
                candidate_count,
                6,
                epsilon_scale,
                minimum_depth,
                pair_count,
                &mut contact0,
                &mut contact1,
                &mut contact2,
                &mut contact3,
                &mut manifold.contact0_exists,
                &mut manifold.contact1_exists,
                &mut manifold.contact2_exists,
                &mut manifold.contact3_exists,
            );

            // Transform contacts into manifold.
            let mut world_tangent_bx = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&tangent_bx, &world_ra, &mut world_tangent_bx);
            let mut world_tangent_by = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&tangent_by, &world_ra, &mut world_tangent_by);
            let mut world_triangle_center = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &local_triangle_center_b, &world_ra, &mut world_triangle_center,
            );
            Matrix3x3Wide::transform_without_overlap(
                &local_normal, &world_ra, &mut manifold.normal,
            );

            manifold.contact0_exists = manifold.contact0_exists & allow_contacts;
            manifold.contact1_exists = manifold.contact1_exists & allow_contacts;
            manifold.contact2_exists = manifold.contact2_exists & allow_contacts;
            manifold.contact3_exists = manifold.contact3_exists & allow_contacts;

            Self::transform_contact_to_manifold(
                &contact0, &world_triangle_center, &world_tangent_bx, &world_tangent_by,
                &mut manifold.offset_a0, &mut manifold.depth0, &mut manifold.feature_id0,
            );
            Self::transform_contact_to_manifold(
                &contact1, &world_triangle_center, &world_tangent_bx, &world_tangent_by,
                &mut manifold.offset_a1, &mut manifold.depth1, &mut manifold.feature_id1,
            );
            Self::transform_contact_to_manifold(
                &contact2, &world_triangle_center, &world_tangent_bx, &world_tangent_by,
                &mut manifold.offset_a2, &mut manifold.depth2, &mut manifold.feature_id2,
            );
            Self::transform_contact_to_manifold(
                &contact3, &world_triangle_center, &world_tangent_bx, &world_tangent_by,
                &mut manifold.offset_a3, &mut manifold.depth3, &mut manifold.feature_id3,
            );

            // Boundary smoothing face flag. Privilege triangle B.
            let face_flag = local_normal_dot_face_normal_b
                .simd_ge(Vector::<f32>::splat(MINIMUM_DOT_FOR_FACE_COLLISION))
                .select(
                    Vector::<i32>::splat(FACE_COLLISION_FLAG),
                    Vector::<i32>::splat(0),
                );
            manifold.feature_id0 = manifold.feature_id0 + face_flag;
        }
    }
}
