// Translated from BepuPhysics/CollisionDetection/CollisionTasks/BoxPairTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidate, ManifoldCandidateHelper,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for box vs box collisions.
pub struct BoxPairTester;

impl BoxPairTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests one edge direction of B against all three axes of A.
    #[inline(always)]
    fn test_edge_edge(
        half_width_a: &Vector<f32>,
        half_height_a: &Vector<f32>,
        half_length_a: &Vector<f32>,
        half_width_b: &Vector<f32>,
        half_height_b: &Vector<f32>,
        half_length_b: &Vector<f32>,
        offset_bx: &Vector<f32>,
        offset_by: &Vector<f32>,
        offset_bz: &Vector<f32>,
        r_bx: &Vector3Wide,
        r_by: &Vector3Wide,
        r_bz: &Vector3Wide,
        edge_b_direction: &Vector3Wide,
    ) -> (Vector<f32>, Vector<f32>, Vector<f32>, Vector<f32>) {
        // Returns (depth, local_normal_ax, local_normal_ay, local_normal_az)
        let x2 = edge_b_direction.x * edge_b_direction.x;
        let y2 = edge_b_direction.y * edge_b_direction.y;
        let z2 = edge_b_direction.z * edge_b_direction.z;
        let one = Vector::<f32>::splat(1.0);
        let eps = Vector::<f32>::splat(1e-7);
        let max_val = Vector::<f32>::splat(f32::MAX);
        let zero_f = Vector::<f32>::splat(0.0);

        // A.X x edgeB
        let mut depth;
        let mut local_normal_ax;
        let mut local_normal_ay;
        let mut local_normal_az;
        {
            let length = StdFloat::sqrt(y2 + z2);
            let inverse_length = one / length;
            local_normal_ax = zero_f;
            local_normal_ay = edge_b_direction.z * inverse_length;
            local_normal_az = -edge_b_direction.y * inverse_length;
            let extreme_a =
                local_normal_ay.abs() * *half_height_a + local_normal_az.abs() * *half_length_a;
            let n_bx = local_normal_ay * r_bx.y + local_normal_az * r_bx.z;
            let n_by = local_normal_ay * r_by.y + local_normal_az * r_by.z;
            let n_bz = local_normal_ay * r_bz.y + local_normal_az * r_bz.z;
            let extreme_b = n_bx.abs() * *half_width_b
                + n_by.abs() * *half_height_b
                + n_bz.abs() * *half_length_b;
            depth = extreme_a + extreme_b
                - (*offset_by * local_normal_ay + *offset_bz * local_normal_az).abs();
            depth = length.simd_lt(eps).select(max_val, depth);
        }
        // A.Y x edgeB
        {
            let length = StdFloat::sqrt(x2 + z2);
            let inverse_length = one / length;
            let nx = edge_b_direction.z * inverse_length;
            let nz = -edge_b_direction.x * inverse_length;
            let extreme_a = nx.abs() * *half_width_a + nz.abs() * *half_length_a;
            let n_bx = nx * r_bx.x + nz * r_bx.z;
            let n_by = nx * r_by.x + nz * r_by.z;
            let n_bz = nx * r_bz.x + nz * r_bz.z;
            let extreme_b = n_bx.abs() * *half_width_b
                + n_by.abs() * *half_height_b
                + n_bz.abs() * *half_length_b;
            let d = extreme_a + extreme_b - (*offset_bx * nx + *offset_bz * nz).abs();
            let d = length.simd_lt(eps).select(max_val, d);
            let use_y = d.simd_lt(depth);
            depth = use_y.select(d, depth);
            local_normal_ax = use_y.select(nx, local_normal_ax);
            local_normal_ay = use_y.select(zero_f, local_normal_ay);
            local_normal_az = use_y.select(nz, local_normal_az);
        }
        // A.Z x edgeB
        {
            let length = StdFloat::sqrt(x2 + y2);
            let inverse_length = one / length;
            let nx = edge_b_direction.y * inverse_length;
            let ny = -edge_b_direction.x * inverse_length;
            let extreme_a = nx.abs() * *half_width_a + ny.abs() * *half_height_a;
            let n_bx = nx * r_bx.x + ny * r_bx.y;
            let n_by = nx * r_by.x + ny * r_by.y;
            let n_bz = nx * r_bz.x + ny * r_bz.y;
            let extreme_b = n_bx.abs() * *half_width_b
                + n_by.abs() * *half_height_b
                + n_bz.abs() * *half_length_b;
            let d = extreme_a + extreme_b - (*offset_bx * nx + *offset_by * ny).abs();
            let d = length.simd_lt(eps).select(max_val, d);
            let use_z = d.simd_lt(depth);
            depth = use_z.select(d, depth);
            local_normal_ax = use_z.select(nx, local_normal_ax);
            local_normal_ay = use_z.select(ny, local_normal_ay);
            local_normal_az = use_z.select(zero_f, local_normal_az);
        }
        (depth, local_normal_ax, local_normal_ay, local_normal_az)
    }

    #[inline(always)]
    fn select(
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        candidate_depth: &Vector<f32>,
        candidate_nx: &Vector<f32>,
        candidate_ny: &Vector<f32>,
        candidate_nz: &Vector<f32>,
    ) {
        let use_candidate = candidate_depth.simd_lt(*depth);
        *depth = use_candidate.select(*candidate_depth, *depth);
        normal.x = use_candidate.select(*candidate_nx, normal.x);
        normal.y = use_candidate.select(*candidate_ny, normal.y);
        normal.z = use_candidate.select(*candidate_nz, normal.z);
    }

    /// Adds a single box A vertex projected onto box B's face.
    #[inline(always)]
    unsafe fn add_box_a_vertex(
        vertex: &Vector3Wide,
        feature_id: &Vector<i32>,
        face_normal_b: &Vector3Wide,
        contact_normal: &Vector3Wide,
        inverse_contact_normal_dot_face_normal_b: &Vector<f32>,
        face_center_b: &Vector3Wide,
        face_tangent_bx: &Vector3Wide,
        face_tangent_by: &Vector3Wide,
        half_span_bx: &Vector<f32>,
        half_span_by: &Vector<f32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
        allow_contacts: &Vector<i32>,
    ) {
        let point_on_b_to_vertex = *vertex - *face_center_b;
        let mut plane_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(face_normal_b, &point_on_b_to_vertex, &mut plane_distance);
        let offset = Vector3Wide::scale(
            contact_normal,
            &(plane_distance * *inverse_contact_normal_dot_face_normal_b),
        );
        let vertex_on_b_face = *vertex - offset;
        let vertex_offset_on_b_face = vertex_on_b_face - *face_center_b;
        let mut candidate = ManifoldCandidate::default();
        Vector3Wide::dot(&vertex_offset_on_b_face, face_tangent_bx, &mut candidate.x);
        Vector3Wide::dot(&vertex_offset_on_b_face, face_tangent_by, &mut candidate.y);
        candidate.feature_id = *feature_id;

        let contained =
            candidate.x.abs().simd_le(*half_span_bx) & candidate.y.abs().simd_le(*half_span_by);
        let below_buffer_capacity = (*candidate_count).simd_lt(Vector::<i32>::splat(8));
        let contact_exists = *allow_contacts & contained.to_int() & below_buffer_capacity.to_int();
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            &candidate,
            &contact_exists,
            pair_count,
        );
    }

    #[inline(always)]
    unsafe fn add_box_a_vertices(
        face_center_b: &Vector3Wide,
        face_tangent_bx: &Vector3Wide,
        face_tangent_by: &Vector3Wide,
        half_span_bx: &Vector<f32>,
        half_span_by: &Vector<f32>,
        face_normal_b: &Vector3Wide,
        contact_normal: &Vector3Wide,
        v00: &Vector3Wide,
        v01: &Vector3Wide,
        v10: &Vector3Wide,
        v11: &Vector3Wide,
        f00: &Vector<i32>,
        f01: &Vector<i32>,
        f10: &Vector<i32>,
        f11: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
        allow_contacts: &Vector<i32>,
    ) {
        let mut normal_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(face_normal_b, contact_normal, &mut normal_dot);
        let inverse_contact_normal_dot_face_normal_b = normal_dot
            .abs()
            .simd_gt(Vector::<f32>::splat(1e-10))
            .select(
                Vector::<f32>::splat(1.0) / normal_dot,
                Vector::<f32>::splat(f32::MAX),
            );
        Self::add_box_a_vertex(
            v00,
            f00,
            face_normal_b,
            contact_normal,
            &inverse_contact_normal_dot_face_normal_b,
            face_center_b,
            face_tangent_bx,
            face_tangent_by,
            half_span_bx,
            half_span_by,
            candidates,
            candidate_count,
            pair_count,
            allow_contacts,
        );
        Self::add_box_a_vertex(
            v01,
            f01,
            face_normal_b,
            contact_normal,
            &inverse_contact_normal_dot_face_normal_b,
            face_center_b,
            face_tangent_bx,
            face_tangent_by,
            half_span_bx,
            half_span_by,
            candidates,
            candidate_count,
            pair_count,
            allow_contacts,
        );
        Self::add_box_a_vertex(
            v10,
            f10,
            face_normal_b,
            contact_normal,
            &inverse_contact_normal_dot_face_normal_b,
            face_center_b,
            face_tangent_bx,
            face_tangent_by,
            half_span_bx,
            half_span_by,
            candidates,
            candidate_count,
            pair_count,
            allow_contacts,
        );
        Self::add_box_a_vertex(
            v11,
            f11,
            face_normal_b,
            contact_normal,
            &inverse_contact_normal_dot_face_normal_b,
            face_center_b,
            face_tangent_bx,
            face_tangent_by,
            half_span_bx,
            half_span_by,
            candidates,
            candidate_count,
            pair_count,
            allow_contacts,
        );
    }

    #[inline(always)]
    fn clip_box_b_edge_against_box_a_face(
        edge_direction: &Vector3Wide,
        edge_start_b0_to_edge_anchor_a00: &Vector3Wide,
        edge_start_b0_to_edge_anchor_a11: &Vector3Wide,
        edge_start_b1_to_edge_anchor_a00: &Vector3Wide,
        edge_start_b1_to_edge_anchor_a11: &Vector3Wide,
        box_edge_plane_normal: &Vector3Wide,
    ) -> (Vector<f32>, Vector<f32>, Vector<f32>, Vector<f32>) {
        // Returns (min0, max0, min1, max1)
        let mut distance00 = Vector::<f32>::splat(0.0);
        let mut distance01 = Vector::<f32>::splat(0.0);
        let mut distance10 = Vector::<f32>::splat(0.0);
        let mut distance11 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            edge_start_b0_to_edge_anchor_a00,
            box_edge_plane_normal,
            &mut distance00,
        );
        Vector3Wide::dot(
            edge_start_b0_to_edge_anchor_a11,
            box_edge_plane_normal,
            &mut distance01,
        );
        Vector3Wide::dot(
            edge_start_b1_to_edge_anchor_a00,
            box_edge_plane_normal,
            &mut distance10,
        );
        Vector3Wide::dot(
            edge_start_b1_to_edge_anchor_a11,
            box_edge_plane_normal,
            &mut distance11,
        );
        let mut velocity = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(box_edge_plane_normal, edge_direction, &mut velocity);
        let inverse_velocity = Vector::<f32>::splat(1.0) / velocity;

        let edge_start_is_inside0 = (distance00 * distance01).simd_le(Vector::<f32>::splat(0.0));
        let edge_start_is_inside1 = (distance10 * distance11).simd_le(Vector::<f32>::splat(0.0));
        let dont_use_fallback = velocity.abs().simd_gt(Vector::<f32>::splat(1e-15));
        let t00 = distance00 * inverse_velocity;
        let t01 = distance01 * inverse_velocity;
        let t10 = distance10 * inverse_velocity;
        let t11 = distance11 * inverse_velocity;
        let large_negative = Vector::<f32>::splat(-f32::MAX);
        let large_positive = Vector::<f32>::splat(f32::MAX);
        let min0 = dont_use_fallback.select(
            t00.simd_min(t01),
            edge_start_is_inside0.select(large_negative, large_positive),
        );
        let max0 = dont_use_fallback.select(
            t00.simd_max(t01),
            edge_start_is_inside0.select(large_positive, large_negative),
        );
        let min1 = dont_use_fallback.select(
            t10.simd_min(t11),
            edge_start_is_inside1.select(large_negative, large_positive),
        );
        let max1 = dont_use_fallback.select(
            t10.simd_max(t11),
            edge_start_is_inside1.select(large_positive, large_negative),
        );
        (min0, max0, min1, max1)
    }

    #[inline(always)]
    fn clip_box_b_edges_against_box_a_face(
        edge_start_b0: &Vector3Wide,
        edge_start_b1: &Vector3Wide,
        edge_direction_b: &Vector3Wide,
        half_span_b: &Vector<f32>,
        vertex_a00: &Vector3Wide,
        vertex_a11: &Vector3Wide,
        edge_plane_normal_ax: &Vector3Wide,
        edge_plane_normal_ay: &Vector3Wide,
    ) -> (Vector<f32>, Vector<f32>, Vector<f32>, Vector<f32>) {
        // Returns (min0, max0, min1, max1)
        let edge_start_b0_to_va00 = *vertex_a00 - *edge_start_b0;
        let edge_start_b0_to_va11 = *vertex_a11 - *edge_start_b0;
        let edge_start_b1_to_va00 = *vertex_a00 - *edge_start_b1;
        let edge_start_b1_to_va11 = *vertex_a11 - *edge_start_b1;
        let (min_x0, max_x0, min_x1, max_x1) = Self::clip_box_b_edge_against_box_a_face(
            edge_direction_b,
            &edge_start_b0_to_va00,
            &edge_start_b0_to_va11,
            &edge_start_b1_to_va00,
            &edge_start_b1_to_va11,
            edge_plane_normal_ax,
        );
        let (min_y0, max_y0, min_y1, max_y1) = Self::clip_box_b_edge_against_box_a_face(
            edge_direction_b,
            &edge_start_b0_to_va00,
            &edge_start_b0_to_va11,
            &edge_start_b1_to_va00,
            &edge_start_b1_to_va11,
            edge_plane_normal_ay,
        );
        let negative_half_span_b = -*half_span_b;
        // Intersection of the two intervals
        let min0 = negative_half_span_b.simd_max(min_x0.simd_max(min_y0));
        let max0 = half_span_b.simd_min(max_x0.simd_min(max_y0));
        let min1 = negative_half_span_b.simd_max(min_x1.simd_max(min_y1));
        let max1 = half_span_b.simd_min(max_x1.simd_min(max_y1));
        (min0, max0, min1, max1)
    }

    #[inline(always)]
    unsafe fn add_contacts_for_edge(
        min: &Vector<f32>,
        min_candidate: &ManifoldCandidate,
        max: &Vector<f32>,
        max_candidate: &ManifoldCandidate,
        half_span_b: &Vector<f32>,
        epsilon: &Vector<f32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        allow_contacts: &Vector<i32>,
        pair_count: i32,
    ) {
        let min_exists = *allow_contacts
            & ((*max - *min).simd_gt(*epsilon)).to_int()
            & min.abs().simd_lt(*half_span_b).to_int();
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            min_candidate,
            &min_exists,
            pair_count,
        );

        let max_exists =
            *allow_contacts & max.simd_ge(*min).to_int() & max.abs().simd_le(*half_span_b).to_int();
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            max_candidate,
            &max_exists,
            pair_count,
        );
    }

    #[inline(always)]
    unsafe fn create_edge_contacts(
        face_center_b: &Vector3Wide,
        face_tangent_bx: &Vector3Wide,
        face_tangent_by: &Vector3Wide,
        half_span_bx: &Vector<f32>,
        half_span_by: &Vector<f32>,
        vertex_a00: &Vector3Wide,
        vertex_a11: &Vector3Wide,
        face_tangent_ax: &Vector3Wide,
        face_tangent_ay: &Vector3Wide,
        contact_normal: &Vector3Wide,
        feature_id_x0: &Vector<i32>,
        feature_id_x1: &Vector<i32>,
        feature_id_y0: &Vector<i32>,
        feature_id_y1: &Vector<i32>,
        epsilon_scale: &Vector<f32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
        allow_contacts: &Vector<i32>,
    ) {
        // Compute edge plane normals perpendicular to the contact normal
        let mut edge_plane_normal_ax = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(
            face_tangent_ay,
            contact_normal,
            &mut edge_plane_normal_ax,
        );
        let mut edge_plane_normal_ay = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(
            face_tangent_ax,
            contact_normal,
            &mut edge_plane_normal_ay,
        );

        let edge_offset_bx = Vector3Wide::scale(face_tangent_by, half_span_by);
        let edge_offset_by = Vector3Wide::scale(face_tangent_bx, half_span_bx);
        let edge_start_bx0 = *face_center_b - edge_offset_bx;
        let edge_start_bx1 = *face_center_b + edge_offset_bx;
        let (min_x0, max_x0, unflipped_min_x1, unflipped_max_x1) =
            Self::clip_box_b_edges_against_box_a_face(
                &edge_start_bx0,
                &edge_start_bx1,
                face_tangent_bx,
                half_span_bx,
                vertex_a00,
                vertex_a11,
                &edge_plane_normal_ax,
                &edge_plane_normal_ay,
            );
        let edge_start_by0 = *face_center_b - edge_offset_by;
        let edge_start_by1 = *face_center_b + edge_offset_by;
        let (unflipped_min_y0, unflipped_max_y0, min_y1, max_y1) =
            Self::clip_box_b_edges_against_box_a_face(
                &edge_start_by0,
                &edge_start_by1,
                face_tangent_by,
                half_span_by,
                vertex_a00,
                vertex_a11,
                &edge_plane_normal_ax,
                &edge_plane_normal_ay,
            );

        // Flip intervals to ensure consistent winding
        let min_x1 = -unflipped_max_x1;
        let max_x1 = -unflipped_min_x1;
        let min_y0 = -unflipped_max_y0;
        let max_y0 = -unflipped_min_y0;

        let edge_feature_id_offset = Vector::<i32>::splat(64);
        let epsilon = *epsilon_scale * Vector::<f32>::splat(1e-5);
        let mut min_c = ManifoldCandidate::default();
        let mut max_c = ManifoldCandidate::default();

        // X0
        min_c.feature_id = *feature_id_x0;
        min_c.x = min_x0;
        min_c.y = -*half_span_by;
        max_c.feature_id = *feature_id_x0 + edge_feature_id_offset;
        max_c.x = max_x0;
        max_c.y = min_c.y;
        Self::add_contacts_for_edge(
            &min_x0,
            &min_c,
            &max_x0,
            &max_c,
            half_span_bx,
            &epsilon,
            candidates,
            candidate_count,
            allow_contacts,
            pair_count,
        );

        // Y1
        min_c.feature_id = *feature_id_y1;
        min_c.x = *half_span_bx;
        min_c.y = min_y1;
        max_c.feature_id = *feature_id_y1 + edge_feature_id_offset;
        max_c.x = *half_span_bx;
        max_c.y = max_y1;
        Self::add_contacts_for_edge(
            &min_y1,
            &min_c,
            &max_y1,
            &max_c,
            half_span_by,
            &epsilon,
            candidates,
            candidate_count,
            allow_contacts,
            pair_count,
        );

        // X1
        min_c.feature_id = *feature_id_x1;
        min_c.x = unflipped_max_x1;
        min_c.y = *half_span_by;
        max_c.feature_id = *feature_id_x1 + edge_feature_id_offset;
        max_c.x = unflipped_min_x1;
        max_c.y = *half_span_by;
        Self::add_contacts_for_edge(
            &min_x1,
            &min_c,
            &max_x1,
            &max_c,
            half_span_bx,
            &epsilon,
            candidates,
            candidate_count,
            allow_contacts,
            pair_count,
        );

        // Y0
        min_c.feature_id = *feature_id_y0;
        min_c.x = -*half_span_bx;
        min_c.y = unflipped_max_y0;
        max_c.feature_id = *feature_id_y0 + edge_feature_id_offset;
        max_c.x = -*half_span_bx;
        max_c.y = unflipped_min_y0;
        Self::add_contacts_for_edge(
            &min_y0,
            &min_c,
            &max_y0,
            &max_c,
            half_span_by,
            &epsilon,
            candidates,
            candidate_count,
            allow_contacts,
            pair_count,
        );
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
        let sx = Vector3Wide::scale(tangent_bx, &raw_contact.x);
        let sy = Vector3Wide::scale(tangent_by, &raw_contact.y);
        *manifold_offset_a = sx + sy + *face_center_b;
        *manifold_depth = raw_contact.depth;
        *manifold_feature_id = raw_contact.feature_id;
    }

    /// Tests box vs box collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &BoxWide,
        b: &BoxWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        unsafe {
            let mut world_ra = Matrix3x3Wide::default();
            Matrix3x3Wide::create_from_quaternion(orientation_a, &mut world_ra);
            let mut world_rb = Matrix3x3Wide::default();
            Matrix3x3Wide::create_from_quaternion(orientation_b, &mut world_rb);
            let mut r_b = Matrix3x3Wide::default();
            Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_rb, &world_ra, &mut r_b);
            let mut local_offset_b = Vector3Wide::default();
            Matrix3x3Wide::transform_by_transposed_without_overlap(
                offset_b,
                &world_ra,
                &mut local_offset_b,
            );

            let mut local_normal = Vector3Wide::default();

            // Test edge-edge axes: b.X
            let (mut depth, ln_x, ln_y, ln_z) = Self::test_edge_edge(
                &a.half_width,
                &a.half_height,
                &a.half_length,
                &b.half_width,
                &b.half_height,
                &b.half_length,
                &local_offset_b.x,
                &local_offset_b.y,
                &local_offset_b.z,
                &r_b.x,
                &r_b.y,
                &r_b.z,
                &r_b.x,
            );
            local_normal.x = ln_x;
            local_normal.y = ln_y;
            local_normal.z = ln_z;

            // b.Y
            let (ey_depth, ey_nx, ey_ny, ey_nz) = Self::test_edge_edge(
                &a.half_width,
                &a.half_height,
                &a.half_length,
                &b.half_width,
                &b.half_height,
                &b.half_length,
                &local_offset_b.x,
                &local_offset_b.y,
                &local_offset_b.z,
                &r_b.x,
                &r_b.y,
                &r_b.z,
                &r_b.y,
            );
            Self::select(
                &mut depth,
                &mut local_normal,
                &ey_depth,
                &ey_nx,
                &ey_ny,
                &ey_nz,
            );

            // b.Z
            let (ez_depth, ez_nx, ez_ny, ez_nz) = Self::test_edge_edge(
                &a.half_width,
                &a.half_height,
                &a.half_length,
                &b.half_width,
                &b.half_height,
                &b.half_length,
                &local_offset_b.x,
                &local_offset_b.y,
                &local_offset_b.z,
                &r_b.x,
                &r_b.y,
                &r_b.z,
                &r_b.z,
            );
            Self::select(
                &mut depth,
                &mut local_normal,
                &ez_depth,
                &ez_nx,
                &ez_ny,
                &ez_nz,
            );

            // Test face normals of A
            let abs_rbx = r_b.x.abs();
            let abs_rby = r_b.y.abs();
            let abs_rbz = r_b.z.abs();
            let one = Vector::<f32>::splat(1.0);
            let zero_f = Vector::<f32>::splat(0.0);

            let face_ax_depth = a.half_width
                + b.half_width * abs_rbx.x
                + b.half_height * abs_rby.x
                + b.half_length * abs_rbz.x
                - local_offset_b.x.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_ax_depth,
                &one,
                &zero_f,
                &zero_f,
            );

            let face_ay_depth = a.half_height
                + b.half_width * abs_rbx.y
                + b.half_height * abs_rby.y
                + b.half_length * abs_rbz.y
                - local_offset_b.y.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_ay_depth,
                &zero_f,
                &one,
                &zero_f,
            );

            let face_az_depth = a.half_length
                + b.half_width * abs_rbx.z
                + b.half_height * abs_rby.z
                + b.half_length * abs_rbz.z
                - local_offset_b.z.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_az_depth,
                &zero_f,
                &zero_f,
                &one,
            );

            // Test face normals of B
            let mut b_local_offset_b = Vector3Wide::default();
            Matrix3x3Wide::transform_by_transposed_without_overlap(
                &local_offset_b,
                &r_b,
                &mut b_local_offset_b,
            );
            let face_bx_depth = b.half_width
                + a.half_width * abs_rbx.x
                + a.half_height * abs_rbx.y
                + a.half_length * abs_rbx.z
                - b_local_offset_b.x.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_bx_depth,
                &r_b.x.x,
                &r_b.x.y,
                &r_b.x.z,
            );
            let face_by_depth = b.half_height
                + a.half_width * abs_rby.x
                + a.half_height * abs_rby.y
                + a.half_length * abs_rby.z
                - b_local_offset_b.y.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_by_depth,
                &r_b.y.x,
                &r_b.y.y,
                &r_b.y.z,
            );
            let face_bz_depth = b.half_length
                + a.half_width * abs_rbz.x
                + a.half_height * abs_rbz.y
                + a.half_length * abs_rbz.z
                - b_local_offset_b.z.abs();
            Self::select(
                &mut depth,
                &mut local_normal,
                &face_bz_depth,
                &r_b.z.x,
                &r_b.z.y,
                &r_b.z.z,
            );

            // Early out if no contacts
            let active_lanes = BundleIndexing::create_mask_for_count_in_bundle(pair_count as usize);
            let minimum_depth = -*speculative_margin;
            let allow_contacts = active_lanes & depth.simd_ge(minimum_depth).to_int();
            if allow_contacts.simd_eq(Vector::<i32>::splat(0)).all() {
                manifold.contact0_exists = Vector::<i32>::splat(0);
                manifold.contact1_exists = Vector::<i32>::splat(0);
                manifold.contact2_exists = Vector::<i32>::splat(0);
                manifold.contact3_exists = Vector::<i32>::splat(0);
                return;
            }

            // Calibrate normal to point from B to A
            let mut normal_dot_offset_b = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_normal, &local_offset_b, &mut normal_dot_offset_b);
            let should_negate_normal = normal_dot_offset_b.simd_gt(Vector::<f32>::splat(0.0));
            local_normal.x = should_negate_normal.select(-local_normal.x, local_normal.x);
            local_normal.y = should_negate_normal.select(-local_normal.y, local_normal.y);
            local_normal.z = should_negate_normal.select(-local_normal.z, local_normal.z);
            Matrix3x3Wide::transform_without_overlap(
                &local_normal,
                &world_ra,
                &mut manifold.normal,
            );

            // Choose representative face on each box based on max normal alignment
            let mut ax_dot = Vector::<f32>::splat(0.0);
            let mut ay_dot = Vector::<f32>::splat(0.0);
            let mut az_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&manifold.normal, &world_ra.x, &mut ax_dot);
            Vector3Wide::dot(&manifold.normal, &world_ra.y, &mut ay_dot);
            Vector3Wide::dot(&manifold.normal, &world_ra.z, &mut az_dot);
            let abs_ax_dot = ax_dot.abs();
            let abs_ay_dot = ay_dot.abs();
            let abs_az_dot = az_dot.abs();
            let max_a_dot = abs_ax_dot.simd_max(abs_ay_dot.simd_max(abs_az_dot));
            let use_ax = max_a_dot.simd_eq(abs_ax_dot).to_int();
            let use_ay = max_a_dot.simd_eq(abs_ay_dot).to_int() & !use_ax;

            let _normal_a = Vector3Wide::conditional_select(
                &use_ay,
                &Vector3Wide::conditional_select(&use_ax, &world_ra.x, &world_ra.z),
                &Vector3Wide::conditional_select(&use_ax, &world_ra.x, &world_ra.z),
            );
            let normal_a = Vector3Wide::conditional_select(
                &use_ax,
                &world_ra.x,
                &Vector3Wide::conditional_select(&use_ay, &world_ra.y, &world_ra.z),
            );
            let tangent_ax = Vector3Wide::conditional_select(
                &use_ax,
                &world_ra.z,
                &Vector3Wide::conditional_select(&use_ay, &world_ra.x, &world_ra.y),
            );
            let tangent_ay = Vector3Wide::conditional_select(
                &use_ax,
                &world_ra.y,
                &Vector3Wide::conditional_select(&use_ay, &world_ra.z, &world_ra.x),
            );

            let use_ax_f = use_ax.simd_lt(Vector::<i32>::splat(0));
            let use_ay_f = use_ay.simd_lt(Vector::<i32>::splat(0));
            let half_span_ax =
                use_ax_f.select(a.half_length, use_ay_f.select(a.half_width, a.half_height));
            let half_span_ay =
                use_ax_f.select(a.half_height, use_ay_f.select(a.half_length, a.half_width));
            let half_span_az =
                use_ax_f.select(a.half_width, use_ay_f.select(a.half_height, a.half_length));

            let local_x_id = Vector::<i32>::splat(1);
            let local_y_id = Vector::<i32>::splat(4);
            let local_z_id = Vector::<i32>::splat(16);
            let use_ax_i = use_ax.simd_lt(Vector::<i32>::splat(0));
            let use_ay_i = use_ay.simd_lt(Vector::<i32>::splat(0));
            let axis_id_ax = use_ax_i.select(local_z_id, use_ay_i.select(local_x_id, local_y_id));
            let axis_id_ay = use_ax_i.select(local_y_id, use_ay_i.select(local_z_id, local_x_id));
            let axis_id_az = use_ax_i.select(local_x_id, use_ay_i.select(local_y_id, local_z_id));

            // Box B face selection
            let mut bx_dot = Vector::<f32>::splat(0.0);
            let mut by_dot = Vector::<f32>::splat(0.0);
            let mut bz_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&manifold.normal, &world_rb.x, &mut bx_dot);
            Vector3Wide::dot(&manifold.normal, &world_rb.y, &mut by_dot);
            Vector3Wide::dot(&manifold.normal, &world_rb.z, &mut bz_dot);
            let abs_bx_dot = bx_dot.abs();
            let abs_by_dot = by_dot.abs();
            let abs_bz_dot = bz_dot.abs();
            let max_b_dot = abs_bx_dot.simd_max(abs_by_dot.simd_max(abs_bz_dot));
            let use_bx = max_b_dot.simd_eq(abs_bx_dot).to_int();
            let use_by = max_b_dot.simd_eq(abs_by_dot).to_int() & !use_bx;
            let use_bx_f = use_bx.simd_lt(Vector::<i32>::splat(0));
            let use_by_f = use_by.simd_lt(Vector::<i32>::splat(0));

            let mut normal_b = Vector3Wide::conditional_select(
                &use_bx,
                &world_rb.x,
                &Vector3Wide::conditional_select(&use_by, &world_rb.y, &world_rb.z),
            );
            let tangent_bx = Vector3Wide::conditional_select(
                &use_bx,
                &world_rb.z,
                &Vector3Wide::conditional_select(&use_by, &world_rb.x, &world_rb.y),
            );
            let tangent_by = Vector3Wide::conditional_select(
                &use_bx,
                &world_rb.y,
                &Vector3Wide::conditional_select(&use_by, &world_rb.z, &world_rb.x),
            );
            let half_span_bx =
                use_bx_f.select(b.half_length, use_by_f.select(b.half_width, b.half_height));
            let half_span_by =
                use_bx_f.select(b.half_height, use_by_f.select(b.half_length, b.half_width));
            let half_span_bz =
                use_bx_f.select(b.half_width, use_by_f.select(b.half_height, b.half_length));
            let use_bx_i = use_bx.simd_lt(Vector::<i32>::splat(0));
            let use_by_i = use_by.simd_lt(Vector::<i32>::splat(0));
            let axis_id_bx = use_bx_i.select(local_z_id, use_by_i.select(local_x_id, local_y_id));
            let axis_id_by = use_bx_i.select(local_y_id, use_by_i.select(local_z_id, local_x_id));
            let axis_id_bz = use_bx_i.select(local_x_id, use_by_i.select(local_y_id, local_z_id));

            // Calibrate face normals: normalA toward B, normalB toward A
            let mut calibration_dot_a = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&normal_a, &manifold.normal, &mut calibration_dot_a);
            let should_negate_normal_a = calibration_dot_a.simd_gt(Vector::<f32>::splat(0.0));
            let mut normal_a = normal_a;
            normal_a.x = should_negate_normal_a.select(-normal_a.x, normal_a.x);
            normal_a.y = should_negate_normal_a.select(-normal_a.y, normal_a.y);
            normal_a.z = should_negate_normal_a.select(-normal_a.z, normal_a.z);

            let mut calibration_dot_b = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&normal_b, &manifold.normal, &mut calibration_dot_b);
            let should_negate_normal_b = calibration_dot_b.simd_lt(Vector::<f32>::splat(0.0));
            normal_b.x = should_negate_normal_b.select(-normal_b.x, normal_b.x);
            normal_b.y = should_negate_normal_b.select(-normal_b.y, normal_b.y);
            normal_b.z = should_negate_normal_b.select(-normal_b.z, normal_b.z);

            // Allocate candidate buffer (up to 8 candidates)
            let mut candidate_buffer = [ManifoldCandidate::default(); 8];
            let candidates = candidate_buffer.as_mut_ptr();

            // Face centers
            let face_center_a = Vector3Wide::scale(&normal_a, &half_span_az);
            let face_center_b = Vector3Wide::scale(&normal_b, &half_span_bz) + *offset_b;
            let face_center_b_to_face_center_a = face_center_a - face_center_b;

            // Vertex construction for A face
            let edge_offset_ax = Vector3Wide::scale(&tangent_ay, &half_span_ay);
            let edge_offset_ay = Vector3Wide::scale(&tangent_ax, &half_span_ax);
            let vertex_a0 = face_center_a - edge_offset_ax;
            let vertex_a00 = vertex_a0 - edge_offset_ay;
            let vertex_a1 = face_center_a + edge_offset_ax;
            let vertex_a11 = vertex_a1 + edge_offset_ay;

            let epsilon_scale = half_span_ax
                .simd_max(half_span_ay.simd_max(half_span_az))
                .simd_min(half_span_bx.simd_max(half_span_by.simd_max(half_span_bz)));

            // Edge feature IDs
            let two = Vector::<i32>::splat(2);
            let three = Vector::<i32>::splat(3);
            let twice_axis_id_bx = axis_id_bx * two;
            let axis_z_edge_id_contribution = axis_id_bz * three;
            let edge_id_bx0 = twice_axis_id_bx + axis_id_by + axis_z_edge_id_contribution;
            let edge_id_bx1 = twice_axis_id_bx + axis_id_by * three + axis_z_edge_id_contribution;
            let twice_axis_id_by = axis_id_by * two;
            let edge_id_by0 = axis_id_bx + twice_axis_id_by + axis_z_edge_id_contribution;
            let edge_id_by1 = axis_id_bx * three + twice_axis_id_by + axis_z_edge_id_contribution;

            let mut candidate_count = Vector::<i32>::splat(0);
            Self::create_edge_contacts(
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &half_span_bx,
                &half_span_by,
                &vertex_a00,
                &vertex_a11,
                &tangent_ax,
                &tangent_ay,
                &manifold.normal,
                &edge_id_bx0,
                &edge_id_bx1,
                &edge_id_by0,
                &edge_id_by1,
                &epsilon_scale,
                candidates,
                &mut candidate_count,
                pair_count,
                &allow_contacts,
            );

            // Face A vertices — note feature ids are negated to disambiguate
            let vertex_id00 = -axis_id_az;
            let vertex_a01 = vertex_a0 + edge_offset_ay;
            let vertex_id01 = -(axis_id_az + axis_id_ay);
            let vertex_a10 = vertex_a1 - edge_offset_ay;
            let vertex_id10 = -(axis_id_az + axis_id_ax);
            let vertex_id11 = -(axis_id_az + axis_id_ax + axis_id_ay);
            Self::add_box_a_vertices(
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &half_span_bx,
                &half_span_by,
                &normal_b,
                &manifold.normal,
                &vertex_a00,
                &vertex_a01,
                &vertex_a10,
                &vertex_a11,
                &vertex_id00,
                &vertex_id01,
                &vertex_id10,
                &vertex_id11,
                candidates,
                &mut candidate_count,
                pair_count,
                &allow_contacts,
            );

            // Reduce to 4 contacts
            let mut contact0 = ManifoldCandidate::default();
            let mut contact1 = ManifoldCandidate::default();
            let mut contact2 = ManifoldCandidate::default();
            let mut contact3 = ManifoldCandidate::default();
            ManifoldCandidateHelper::reduce(
                candidates,
                candidate_count,
                8,
                &normal_a,
                Vector::<f32>::splat(-1.0) / calibration_dot_a.abs(),
                &face_center_b_to_face_center_a,
                &tangent_bx,
                &tangent_by,
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

            // Transform contacts into manifold
            Self::transform_contact_to_manifold(
                &contact0,
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &mut manifold.offset_a0,
                &mut manifold.depth0,
                &mut manifold.feature_id0,
            );
            Self::transform_contact_to_manifold(
                &contact1,
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &mut manifold.offset_a1,
                &mut manifold.depth1,
                &mut manifold.feature_id1,
            );
            Self::transform_contact_to_manifold(
                &contact2,
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &mut manifold.offset_a2,
                &mut manifold.depth2,
                &mut manifold.feature_id2,
            );
            Self::transform_contact_to_manifold(
                &contact3,
                &face_center_b,
                &tangent_bx,
                &tangent_by,
                &mut manifold.offset_a3,
                &mut manifold.depth3,
                &mut manifold.feature_id3,
            );
        }
    }
}
