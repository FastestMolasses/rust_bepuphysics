// Translated from BepuPhysics/CollisionDetection/CollisionTasks/BoxTriangleTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidate, ManifoldCandidateHelper,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
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

/// Pair tester for box vs triangle collisions.
pub struct BoxTriangleTester;

impl BoxTriangleTester {
    pub const BATCH_SIZE: i32 = 32;

    #[inline(always)]
    fn get_depth_for_interval(
        box_extreme: &Vector<f32>,
        va: &Vector<f32>,
        vb: &Vector<f32>,
        vc: &Vector<f32>,
    ) -> Vector<f32> {
        let min_b = va.simd_min(vb.simd_min(*vc));
        let max_b = va.simd_max(vb.simd_max(*vc));
        (*box_extreme - min_b).simd_min(max_b + *box_extreme)
    }

    #[inline(always)]
    fn test_box_edge_against_triangle_edge(
        tri_edge_offset_y: &Vector<f32>,
        tri_edge_offset_z: &Vector<f32>,
        tri_center_y: &Vector<f32>,
        tri_center_z: &Vector<f32>,
        edge_offset_y_sq: &Vector<f32>,
        edge_offset_z_sq: &Vector<f32>,
        half_height: &Vector<f32>,
        half_length: &Vector<f32>,
        va_y: &Vector<f32>,
        va_z: &Vector<f32>,
        vb_y: &Vector<f32>,
        vb_z: &Vector<f32>,
        vc_y: &Vector<f32>,
        vc_z: &Vector<f32>,
    ) -> (Vector<f32>, Vector<f32>, Vector<f32>, Vector<f32>) {
        // Returns (depth, local_normal_x, local_normal_y, local_normal_z)
        let local_normal_x = Vector::<f32>::splat(0.0);
        let mut local_normal_y = *tri_edge_offset_z;
        let mut local_normal_z = -*tri_edge_offset_y;

        let calibration_dot = *tri_center_y * local_normal_y + *tri_center_z * local_normal_z;
        let length = StdFloat::sqrt(*edge_offset_y_sq + *edge_offset_z_sq);
        let inverse_length = calibration_dot
            .simd_lt(Vector::<f32>::splat(0.0))
            .select(Vector::<f32>::splat(1.0), Vector::<f32>::splat(-1.0))
            / length;
        local_normal_y = local_normal_y * inverse_length;
        local_normal_z = local_normal_z * inverse_length;

        let extreme_a = local_normal_y.abs() * *half_height + local_normal_z.abs() * *half_length;
        let n_va = *va_y * local_normal_y + *va_z * local_normal_z;
        let n_vb = *vb_y * local_normal_y + *vb_z * local_normal_z;
        let n_vc = *vc_y * local_normal_y + *vc_z * local_normal_z;
        let mut depth = Self::get_depth_for_interval(&extreme_a, &n_va, &n_vb, &n_vc);
        depth = length
            .simd_lt(Vector::<f32>::splat(1e-7))
            .select(Vector::<f32>::splat(f32::MAX), depth);
        (depth, local_normal_x, local_normal_y, local_normal_z)
    }

    #[inline(always)]
    fn test_box_edges_against_triangle_edge(
        a: &BoxWide,
        tri_edge_offset: &Vector3Wide,
        tri_center: &Vector3Wide,
        va: &Vector3Wide,
        vb: &Vector3Wide,
        vc: &Vector3Wide,
    ) -> (Vector<f32>, Vector3Wide) {
        let x2 = tri_edge_offset.x * tri_edge_offset.x;
        let y2 = tri_edge_offset.y * tri_edge_offset.y;
        let z2 = tri_edge_offset.z * tri_edge_offset.z;

        // A.X x edgeB
        let (mut depth, mut ln_x, mut ln_y, mut ln_z) = Self::test_box_edge_against_triangle_edge(
            &tri_edge_offset.y,
            &tri_edge_offset.z,
            &tri_center.y,
            &tri_center.z,
            &y2,
            &z2,
            &a.half_height,
            &a.half_length,
            &va.y,
            &va.z,
            &vb.y,
            &vb.z,
            &vc.y,
            &vc.z,
        );

        // A.Y x edgeB (swizzle: Y,X,Z -> depth, normalY, normalX, normalZ)
        let (dc, nc_y, nc_x, nc_z) = Self::test_box_edge_against_triangle_edge(
            &tri_edge_offset.x,
            &tri_edge_offset.z,
            &tri_center.x,
            &tri_center.z,
            &x2,
            &z2,
            &a.half_width,
            &a.half_length,
            &va.x,
            &va.z,
            &vb.x,
            &vb.z,
            &vc.x,
            &vc.z,
        );
        let use_y = dc.simd_lt(depth);
        depth = depth.simd_min(dc);
        ln_x = use_y.select(nc_x, ln_x);
        ln_y = use_y.select(nc_y, ln_y);
        ln_z = use_y.select(nc_z, ln_z);

        // A.Z x edgeB (swizzle: Z,X,Y -> depth, normalZ, normalX, normalY)
        let (dc, nc_z, nc_x2, nc_y2) = Self::test_box_edge_against_triangle_edge(
            &tri_edge_offset.x,
            &tri_edge_offset.y,
            &tri_center.x,
            &tri_center.y,
            &x2,
            &y2,
            &a.half_width,
            &a.half_height,
            &va.x,
            &va.y,
            &vb.x,
            &vb.y,
            &vc.x,
            &vc.y,
        );
        let use_z = dc.simd_lt(depth);
        depth = depth.simd_min(dc);
        ln_x = use_z.select(nc_x2, ln_x);
        ln_y = use_z.select(nc_y2, ln_y);
        ln_z = use_z.select(nc_z, ln_z);

        (
            depth,
            Vector3Wide {
                x: ln_x,
                y: ln_y,
                z: ln_z,
            },
        )
    }

    #[inline(always)]
    fn select_vec(
        depth: &mut Vector<f32>,
        normal: &mut Vector3Wide,
        depth_candidate: &Vector<f32>,
        normal_candidate: &Vector3Wide,
    ) {
        let use_candidate = depth_candidate.simd_lt(*depth).to_int();
        *normal = Vector3Wide::conditional_select(&use_candidate, normal_candidate, normal);
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

    /// Adds a point on the triangle to the manifold candidates.
    #[inline(always)]
    unsafe fn add(
        point_on_triangle: &Vector3Wide,
        triangle_center: &Vector3Wide,
        triangle_tangent_x: &Vector3Wide,
        triangle_tangent_y: &Vector3Wide,
        feature_id: &Vector<i32>,
        exists: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let offset = *point_on_triangle - *triangle_center;
        let mut candidate = ManifoldCandidate::default();
        Vector3Wide::dot(&offset, triangle_tangent_x, &mut candidate.x);
        Vector3Wide::dot(&offset, triangle_tangent_y, &mut candidate.y);
        candidate.feature_id = *feature_id;
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            &candidate,
            exists,
            pair_count,
        );
    }

    #[inline(always)]
    fn clip_triangle_edge_against_planes(
        edge_direction: &Vector3Wide,
        tri_edge_start_to_box_edge_anchor0: &Vector3Wide,
        tri_edge_start_to_box_edge_anchor1: &Vector3Wide,
        box_edge_plane_normal: &Vector3Wide,
    ) -> (Vector<f32>, Vector<f32>) {
        let mut distance0 = Vector::<f32>::splat(0.0);
        let mut distance1 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            tri_edge_start_to_box_edge_anchor0,
            box_edge_plane_normal,
            &mut distance0,
        );
        Vector3Wide::dot(
            tri_edge_start_to_box_edge_anchor1,
            box_edge_plane_normal,
            &mut distance1,
        );
        let mut velocity = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(box_edge_plane_normal, edge_direction, &mut velocity);
        let inverse_velocity = Vector::<f32>::splat(1.0) / velocity;

        let edge_start_is_inside = (distance0 * distance1).simd_le(Vector::<f32>::splat(0.0));
        let dont_use_fallback = velocity.abs().simd_gt(Vector::<f32>::splat(1e-15));
        let t0 = distance0 * inverse_velocity;
        let t1 = distance1 * inverse_velocity;
        let large_negative = Vector::<f32>::splat(-f32::MAX);
        let large_positive = Vector::<f32>::splat(f32::MAX);
        let min = dont_use_fallback.select(
            t0.simd_min(t1),
            edge_start_is_inside.select(large_negative, large_positive),
        );
        let max = dont_use_fallback.select(
            t0.simd_max(t1),
            edge_start_is_inside.select(large_positive, large_negative),
        );
        (min, max)
    }

    #[inline(always)]
    unsafe fn clip_triangle_edge_against_box_face(
        edge_start: &Vector3Wide,
        edge_direction: &Vector3Wide,
        edge_id: &Vector<i32>,
        box_vertex00: &Vector3Wide,
        box_vertex11: &Vector3Wide,
        edge_plane_normal_x: &Vector3Wide,
        edge_plane_normal_y: &Vector3Wide,
        triangle_center: &Vector3Wide,
        triangle_tangent_x: &Vector3Wide,
        triangle_tangent_y: &Vector3Wide,
        allow_contacts: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let tri_edge_start_to_v00 = *box_vertex00 - *edge_start;
        let tri_edge_start_to_v11 = *box_vertex11 - *edge_start;
        let (min_x, max_x) = Self::clip_triangle_edge_against_planes(
            edge_direction,
            &tri_edge_start_to_v00,
            &tri_edge_start_to_v11,
            edge_plane_normal_x,
        );
        let (min_y, max_y) = Self::clip_triangle_edge_against_planes(
            edge_direction,
            &tri_edge_start_to_v00,
            &tri_edge_start_to_v11,
            edge_plane_normal_y,
        );
        let min = min_x.simd_max(min_y);
        let max = Vector::<f32>::splat(1.0).simd_min(max_x.simd_min(max_y));
        let min_location = *edge_start + Vector3Wide::scale(edge_direction, &min);
        let max_location = *edge_start + Vector3Wide::scale(edge_direction, &max);

        let six = Vector::<i32>::splat(6);
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let eps = Vector::<f32>::splat(1e-5);

        let min_exists = *allow_contacts
            & (*candidate_count).simd_lt(six).to_int()
            & (max - min).simd_ge(eps).to_int()
            & min.simd_lt(one_f).to_int()
            & min.simd_gt(zero_f).to_int();
        Self::add(
            &min_location,
            triangle_center,
            triangle_tangent_x,
            triangle_tangent_y,
            edge_id,
            &min_exists,
            candidates,
            candidate_count,
            pair_count,
        );

        let max_exists = *allow_contacts
            & (*candidate_count).simd_lt(six).to_int()
            & max.simd_ge(min).to_int()
            & max.simd_le(one_f).to_int()
            & max.simd_ge(zero_f).to_int();
        Self::add(
            &max_location,
            triangle_center,
            triangle_tangent_x,
            triangle_tangent_y,
            &(*edge_id + Vector::<i32>::splat(8)),
            &max_exists,
            candidates,
            candidate_count,
            pair_count,
        );
    }

    #[inline(always)]
    unsafe fn clip_triangle_edges_against_box_face(
        va: &Vector3Wide,
        vb: &Vector3Wide,
        vc: &Vector3Wide,
        triangle_center: &Vector3Wide,
        triangle_tangent_x: &Vector3Wide,
        triangle_tangent_y: &Vector3Wide,
        ab: &Vector3Wide,
        bc: &Vector3Wide,
        ca: &Vector3Wide,
        box_vertex00: &Vector3Wide,
        box_vertex11: &Vector3Wide,
        box_tangent_x: &Vector3Wide,
        box_tangent_y: &Vector3Wide,
        contact_normal: &Vector3Wide,
        allow_contacts: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let mut edge_plane_normal_x = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(box_tangent_y, contact_normal, &mut edge_plane_normal_x);
        let mut edge_plane_normal_y = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(box_tangent_x, contact_normal, &mut edge_plane_normal_y);

        let base_id = Vector::<i32>::splat(4);
        Self::clip_triangle_edge_against_box_face(
            va,
            ab,
            &base_id,
            box_vertex00,
            box_vertex11,
            &edge_plane_normal_x,
            &edge_plane_normal_y,
            triangle_center,
            triangle_tangent_x,
            triangle_tangent_y,
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
        Self::clip_triangle_edge_against_box_face(
            vb,
            bc,
            &(base_id + Vector::<i32>::splat(1)),
            box_vertex00,
            box_vertex11,
            &edge_plane_normal_x,
            &edge_plane_normal_y,
            triangle_center,
            triangle_tangent_x,
            triangle_tangent_y,
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
        Self::clip_triangle_edge_against_box_face(
            vc,
            ca,
            &(base_id + Vector::<i32>::splat(2)),
            box_vertex00,
            box_vertex11,
            &edge_plane_normal_x,
            &edge_plane_normal_y,
            triangle_center,
            triangle_tangent_x,
            triangle_tangent_y,
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
    }

    #[inline(always)]
    unsafe fn add_box_vertex(
        va: &Vector3Wide,
        vb: &Vector3Wide,
        v: &Vector3Wide,
        triangle_normal: &Vector3Wide,
        contact_normal: &Vector3Wide,
        inverse_normal_dot: &Vector<f32>,
        ab_edge_plane_normal: &Vector3Wide,
        bc_edge_plane_normal: &Vector3Wide,
        ca_edge_plane_normal: &Vector3Wide,
        triangle_center: &Vector3Wide,
        triangle_x: &Vector3Wide,
        triangle_y: &Vector3Wide,
        feature_id: &Vector<i32>,
        allow_contacts: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let point_on_tri_to_box_vertex = *v - *va;
        let mut plane_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            triangle_normal,
            &point_on_tri_to_box_vertex,
            &mut plane_distance,
        );
        let offset = Vector3Wide::scale(contact_normal, &(plane_distance * *inverse_normal_dot));
        let v_on_plane = *v - offset;

        let a_to_v = v_on_plane - *va;
        let b_to_v = v_on_plane - *vb;
        let mut ab_dot = Vector::<f32>::splat(0.0);
        let mut bc_dot = Vector::<f32>::splat(0.0);
        let mut ca_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&a_to_v, ab_edge_plane_normal, &mut ab_dot);
        Vector3Wide::dot(&b_to_v, bc_edge_plane_normal, &mut bc_dot);
        Vector3Wide::dot(&a_to_v, ca_edge_plane_normal, &mut ca_dot);

        let zero = Vector::<f32>::splat(0.0);
        let contained = *allow_contacts
            & ab_dot.simd_ge(zero).to_int()
            & bc_dot.simd_ge(zero).to_int()
            & ca_dot.simd_ge(zero).to_int();

        Self::add(
            &v_on_plane,
            triangle_center,
            triangle_x,
            triangle_y,
            feature_id,
            &contained,
            candidates,
            candidate_count,
            pair_count,
        );
    }

    #[inline(always)]
    unsafe fn add_box_vertices(
        va: &Vector3Wide,
        vb: &Vector3Wide,
        ab: &Vector3Wide,
        bc: &Vector3Wide,
        ca: &Vector3Wide,
        triangle_normal: &Vector3Wide,
        contact_normal: &Vector3Wide,
        v00: &Vector3Wide,
        v01: &Vector3Wide,
        v10: &Vector3Wide,
        v11: &Vector3Wide,
        triangle_center: &Vector3Wide,
        triangle_x: &Vector3Wide,
        triangle_y: &Vector3Wide,
        base_feature_id: &Vector<i32>,
        feature_id_x: &Vector<i32>,
        feature_id_y: &Vector<i32>,
        allow_contacts: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let mut normal_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(triangle_normal, contact_normal, &mut normal_dot);
        let inverse_normal_dot = normal_dot
            .abs()
            .simd_gt(Vector::<f32>::splat(1e-10))
            .select(
                Vector::<f32>::splat(1.0) / normal_dot,
                Vector::<f32>::splat(f32::MAX),
            );

        let mut ab_edge_plane_normal = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(ab, triangle_normal, &mut ab_edge_plane_normal);
        let mut bc_edge_plane_normal = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(bc, triangle_normal, &mut bc_edge_plane_normal);
        let mut ca_edge_plane_normal = Vector3Wide::default();
        Vector3Wide::cross_without_overlap(ca, triangle_normal, &mut ca_edge_plane_normal);

        Self::add_box_vertex(
            va,
            vb,
            v00,
            triangle_normal,
            contact_normal,
            &inverse_normal_dot,
            &ab_edge_plane_normal,
            &bc_edge_plane_normal,
            &ca_edge_plane_normal,
            triangle_center,
            triangle_x,
            triangle_y,
            base_feature_id,
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
        Self::add_box_vertex(
            va,
            vb,
            v01,
            triangle_normal,
            contact_normal,
            &inverse_normal_dot,
            &ab_edge_plane_normal,
            &bc_edge_plane_normal,
            &ca_edge_plane_normal,
            triangle_center,
            triangle_x,
            triangle_y,
            &(*base_feature_id + *feature_id_y),
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
        Self::add_box_vertex(
            va,
            vb,
            v10,
            triangle_normal,
            contact_normal,
            &inverse_normal_dot,
            &ab_edge_plane_normal,
            &bc_edge_plane_normal,
            &ca_edge_plane_normal,
            triangle_center,
            triangle_x,
            triangle_y,
            &(*base_feature_id + *feature_id_x),
            allow_contacts,
            candidates,
            candidate_count,
            pair_count,
        );
        Self::add_box_vertex(
            va,
            vb,
            v11,
            triangle_normal,
            contact_normal,
            &inverse_normal_dot,
            &ab_edge_plane_normal,
            &bc_edge_plane_normal,
            &ca_edge_plane_normal,
            triangle_center,
            triangle_x,
            triangle_y,
            &(*base_feature_id + *feature_id_x + *feature_id_y),
            allow_contacts,
            candidates,
            candidate_count,
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

    /// Tests box vs triangle collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &BoxWide,
        b: &TriangleWide,
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

            // Transform triangle vertices to box A's local space
            let mut va = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.a, &r_b, &mut va);
            va = va + local_offset_b;
            let mut vb = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.b, &r_b, &mut vb);
            vb = vb + local_offset_b;
            let mut vc = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&b.c, &r_b, &mut vc);
            vc = vc + local_offset_b;

            let local_triangle_center =
                Vector3Wide::scale(&(va + vb + vc), &Vector::<f32>::splat(1.0 / 3.0));

            let ab = vb - va;
            let bc = vc - vb;
            let ca = va - vc;

            // Test edge-edge axes
            let (mut depth, mut local_normal) = Self::test_box_edges_against_triangle_edge(
                a,
                &ab,
                &local_triangle_center,
                &va,
                &vb,
                &vc,
            );
            let (dc, nc) = Self::test_box_edges_against_triangle_edge(
                a,
                &bc,
                &local_triangle_center,
                &va,
                &vb,
                &vc,
            );
            Self::select_vec(&mut depth, &mut local_normal, &dc, &nc);
            let (dc, nc) = Self::test_box_edges_against_triangle_edge(
                a,
                &ca,
                &local_triangle_center,
                &va,
                &vb,
                &vc,
            );
            Self::select_vec(&mut depth, &mut local_normal, &dc, &nc);

            // Test face normals of A
            let zero_f = Vector::<f32>::splat(0.0);
            let one_f = Vector::<f32>::splat(1.0);
            let neg_one = Vector::<f32>::splat(-1.0);

            let x_normal_sign = local_triangle_center
                .x
                .simd_lt(zero_f)
                .select(one_f, neg_one);
            let y_normal_sign = local_triangle_center
                .y
                .simd_lt(zero_f)
                .select(one_f, neg_one);
            let z_normal_sign = local_triangle_center
                .z
                .simd_lt(zero_f)
                .select(one_f, neg_one);

            let face_ax_depth = Self::get_depth_for_interval(&a.half_width, &va.x, &vb.x, &vc.x);
            let face_ay_depth = Self::get_depth_for_interval(&a.half_height, &va.y, &vb.y, &vc.y);
            let face_az_depth = Self::get_depth_for_interval(&a.half_length, &va.z, &vb.z, &vc.z);
            Self::select_components(
                &mut depth,
                &mut local_normal,
                &face_ax_depth,
                &x_normal_sign,
                &zero_f,
                &zero_f,
            );
            Self::select_components(
                &mut depth,
                &mut local_normal,
                &face_ay_depth,
                &zero_f,
                &y_normal_sign,
                &zero_f,
            );
            Self::select_components(
                &mut depth,
                &mut local_normal,
                &face_az_depth,
                &zero_f,
                &zero_f,
                &z_normal_sign,
            );

            // Test triangle face normal
            let mut triangle_normal = Vector3Wide::default();
            Vector3Wide::cross_without_overlap(&ab, &ca, &mut triangle_normal);
            let triangle_normal_length = triangle_normal.length();
            let inv_tn_len = one_f / triangle_normal_length;
            triangle_normal = Vector3Wide::scale(&triangle_normal, &inv_tn_len);

            // Calibrate to point from B to A
            let mut triangle_plane_offset = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(
                &triangle_normal,
                &local_triangle_center,
                &mut triangle_plane_offset,
            );
            let neg_triangle_normal = Vector3Wide {
                x: -triangle_normal.x,
                y: -triangle_normal.y,
                z: -triangle_normal.z,
            };
            let should_negate = triangle_plane_offset.simd_gt(zero_f).to_int();
            let calibrated_triangle_normal = Vector3Wide::conditional_select(
                &should_negate,
                &neg_triangle_normal,
                &triangle_normal,
            );
            let triangle_face_depth = triangle_normal.x.abs() * a.half_width
                + triangle_normal.y.abs() * a.half_height
                + triangle_normal.z.abs() * a.half_length
                - triangle_plane_offset.abs();
            Self::select_vec(
                &mut depth,
                &mut local_normal,
                &triangle_face_depth,
                &calibrated_triangle_normal,
            );

            // Active lanes, degeneracy, backface rejection
            let active_lanes = BundleIndexing::create_mask_for_count_in_bundle(pair_count as usize);
            let minimum_depth = -*speculative_margin;
            let mut ab_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ab, &mut ab_length_squared);
            let mut ca_length_squared = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&ca, &mut ca_length_squared);
            let mut triangle_epsilon_scale = Vector::<f32>::splat(0.0);
            let mut nondegenerate_mask = Vector::<i32>::splat(0);
            TriangleWide::compute_nondegenerate_triangle_mask(
                &ab_length_squared,
                &ca_length_squared,
                &triangle_normal_length,
                &mut triangle_epsilon_scale,
                &mut nondegenerate_mask,
            );

            let mut normal_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_normal, &triangle_normal, &mut normal_dot);
            let allow_contacts = nondegenerate_mask
                & normal_dot
                    .simd_ge(Vector::<f32>::splat(
                        TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                    ))
                    .to_int()
                & depth.simd_ge(minimum_depth).to_int()
                & active_lanes;

            if allow_contacts.simd_eq(Vector::<i32>::splat(0)).all() {
                *manifold = std::mem::zeroed();
                return;
            }

            // Choose box face based on maximum dot with normal
            let abs_nx = local_normal.x.abs();
            let abs_ny = local_normal.y.abs();
            let abs_nz = local_normal.z.abs();
            let x_bigger_than_y = abs_nx.simd_gt(abs_ny).to_int();
            let x_bigger_than_z = abs_nx.simd_gt(abs_nz).to_int();
            let y_bigger_than_z = abs_ny.simd_gt(abs_nz).to_int();
            let use_ax = x_bigger_than_y & x_bigger_than_z;
            let use_ay = y_bigger_than_z & !use_ax;
            let use_az = !(use_ax | use_ay);
            let use_ax_f = use_ax.simd_lt(Vector::<i32>::splat(0));
            let use_ay_f = use_ay.simd_lt(Vector::<i32>::splat(0));
            let use_az_f = use_az.simd_lt(Vector::<i32>::splat(0));

            let normal_is_negative_x = local_normal.x.simd_lt(zero_f);
            let normal_is_negative_y = local_normal.y.simd_lt(zero_f);
            let normal_is_negative_z = local_normal.z.simd_lt(zero_f);

            // Box face tangents and normal
            let mut box_tangent_x = Vector3Wide::default();
            box_tangent_x.x = (use_ay_f | use_az_f).select(one_f, zero_f);
            box_tangent_x.y = zero_f;
            box_tangent_x.z = use_ax_f.select(one_f, zero_f);

            let mut box_tangent_y = Vector3Wide::default();
            box_tangent_y.x = zero_f;
            box_tangent_y.y = (use_ax_f | use_az_f).select(one_f, zero_f);
            box_tangent_y.z = use_ay_f.select(one_f, zero_f);

            let mut box_face_normal = Vector3Wide::default();
            box_face_normal.x =
                use_ax_f.select(normal_is_negative_x.select(one_f, neg_one), zero_f);
            box_face_normal.y =
                use_ay_f.select(normal_is_negative_y.select(one_f, neg_one), zero_f);
            box_face_normal.z =
                use_az_f.select(normal_is_negative_z.select(one_f, neg_one), zero_f);

            let half_extent_x = use_ax_f.select(a.half_length, a.half_width);
            let half_extent_y = use_ay_f.select(a.half_length, a.half_height);
            let half_extent_z =
                use_ax_f.select(a.half_width, use_ay_f.select(a.half_height, a.half_length));
            let box_face_center = Vector3Wide::scale(&box_face_normal, &half_extent_z);

            // Feature IDs
            let local_x_id = Vector::<i32>::splat(0);
            let local_y_id = Vector::<i32>::splat(1);
            let local_z_id = Vector::<i32>::splat(2);
            let use_ax_i = use_ax.simd_lt(Vector::<i32>::splat(0));
            let use_ay_i = use_ay.simd_lt(Vector::<i32>::splat(0));
            let axis_id_tangent_x = use_ax_i.select(local_z_id, local_x_id);
            let axis_id_tangent_y = use_ay_i.select(local_z_id, local_y_id);
            let axis_id_normal = use_ax_i.select(
                normal_is_negative_x
                    .to_int()
                    .simd_lt(Vector::<i32>::splat(0))
                    .select(local_x_id, Vector::<i32>::splat(0)),
                use_ay_i.select(
                    normal_is_negative_y
                        .to_int()
                        .simd_lt(Vector::<i32>::splat(0))
                        .select(local_y_id, Vector::<i32>::splat(0)),
                    normal_is_negative_z
                        .to_int()
                        .simd_lt(Vector::<i32>::splat(0))
                        .select(local_z_id, Vector::<i32>::splat(0)),
                ),
            );

            let epsilon_scale = a
                .half_width
                .simd_max(a.half_height.simd_max(a.half_length))
                .simd_min(triangle_epsilon_scale);

            // Triangle tangent frame
            let tri_tangent_x =
                Vector3Wide::scale(&ab, &(one_f / StdFloat::sqrt(ab_length_squared)));
            let mut tri_tangent_y = Vector3Wide::default();
            Vector3Wide::cross_without_overlap(
                &tri_tangent_x,
                &triangle_normal,
                &mut tri_tangent_y,
            );

            // Allocate candidates (up to 6)
            let mut candidate_buffer = [ManifoldCandidate::default(); 6];
            let candidates = candidate_buffer.as_mut_ptr();
            let mut candidate_count = Vector::<i32>::splat(0);

            // Box vertices on triangle face
            let box_edge_offset_x = Vector3Wide::scale(&box_tangent_x, &half_extent_x);
            let box_edge_offset_y = Vector3Wide::scale(&box_tangent_y, &half_extent_y);
            let positive_x = box_face_center + box_edge_offset_x;
            let negative_x = box_face_center - box_edge_offset_x;
            let box_vertex00 = negative_x - box_edge_offset_y;
            let box_vertex01 = negative_x + box_edge_offset_y;
            let box_vertex10 = positive_x - box_edge_offset_y;
            let box_vertex11 = positive_x + box_edge_offset_y;

            Self::add_box_vertices(
                &va,
                &vb,
                &ab,
                &bc,
                &ca,
                &triangle_normal,
                &local_normal,
                &box_vertex00,
                &box_vertex01,
                &box_vertex10,
                &box_vertex11,
                &local_triangle_center,
                &tri_tangent_x,
                &tri_tangent_y,
                &axis_id_normal,
                &axis_id_tangent_x,
                &axis_id_tangent_y,
                &allow_contacts,
                candidates,
                &mut candidate_count,
                pair_count,
            );

            // Triangle edges against box face
            Self::clip_triangle_edges_against_box_face(
                &va,
                &vb,
                &vc,
                &local_triangle_center,
                &tri_tangent_x,
                &tri_tangent_y,
                &ab,
                &bc,
                &ca,
                &box_vertex00,
                &box_vertex11,
                &box_tangent_x,
                &box_tangent_y,
                &local_normal,
                &allow_contacts,
                candidates,
                &mut candidate_count,
                pair_count,
            );

            // Reduce to 4 contacts
            let face_center_b_to_face_center_a = box_face_center - local_triangle_center;
            let mut face_normal_dot_normal = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&box_face_normal, &local_normal, &mut face_normal_dot_normal);
            let mut contact0 = ManifoldCandidate::default();
            let mut contact1 = ManifoldCandidate::default();
            let mut contact2 = ManifoldCandidate::default();
            let mut contact3 = ManifoldCandidate::default();
            ManifoldCandidateHelper::reduce(
                candidates,
                candidate_count,
                6,
                &box_face_normal,
                one_f / face_normal_dot_normal,
                &face_center_b_to_face_center_a,
                &tri_tangent_x,
                &tri_tangent_y,
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

            // Transform contacts to world space
            let mut world_tangent_bx = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &tri_tangent_x,
                &world_ra,
                &mut world_tangent_bx,
            );
            let mut world_tangent_by = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &tri_tangent_y,
                &world_ra,
                &mut world_tangent_by,
            );
            let mut world_triangle_center = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &local_triangle_center,
                &world_ra,
                &mut world_triangle_center,
            );
            Matrix3x3Wide::transform_without_overlap(
                &local_normal,
                &world_ra,
                &mut manifold.normal,
            );

            Self::transform_contact_to_manifold(
                &contact0,
                &world_triangle_center,
                &world_tangent_bx,
                &world_tangent_by,
                &mut manifold.offset_a0,
                &mut manifold.depth0,
                &mut manifold.feature_id0,
            );
            Self::transform_contact_to_manifold(
                &contact1,
                &world_triangle_center,
                &world_tangent_bx,
                &world_tangent_by,
                &mut manifold.offset_a1,
                &mut manifold.depth1,
                &mut manifold.feature_id1,
            );
            Self::transform_contact_to_manifold(
                &contact2,
                &world_triangle_center,
                &world_tangent_bx,
                &world_tangent_by,
                &mut manifold.offset_a2,
                &mut manifold.depth2,
                &mut manifold.feature_id2,
            );
            Self::transform_contact_to_manifold(
                &contact3,
                &world_triangle_center,
                &world_tangent_bx,
                &world_tangent_by,
                &mut manifold.offset_a3,
                &mut manifold.depth3,
                &mut manifold.feature_id3,
            );

            // Face collision flag for mesh boundary smoothing
            let face_flag = normal_dot
                .simd_ge(Vector::<f32>::splat(MINIMUM_DOT_FOR_FACE_COLLISION))
                .select(
                    Vector::<i32>::splat(FACE_COLLISION_FLAG),
                    Vector::<i32>::splat(0),
                );
            manifold.feature_id0 = manifold.feature_id0 + face_flag;
        }
    }
}
