// Translated from BepuPhysics/CollisionDetection/CollisionTasks/TriangleCylinderTester.cs

use crate::physics::collidables::cylinder::{CylinderSupportFinder, CylinderWide};
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::collision_tasks::box_cylinder_tester::BoxCylinderTester;
use crate::physics::collision_detection::collision_tasks::cylinder_pair_tester::CylinderPairTester;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidate, ManifoldCandidateHelper,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::physics::collision_detection::mesh_reduction;
use crate::physics::collision_detection::support_finder::ISupportFinder;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::mem::MaybeUninit;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Support finder for triangles that have already been pretransformed into the other shape's local space.
/// `compute_support` uses the already-transformed vertices directly.
pub struct PretransformedTriangleSupportFinder;

impl ISupportFinder<TriangleWide> for PretransformedTriangleSupportFinder {
    fn has_margin() -> bool {
        false
    }
    fn get_margin(_shape: &TriangleWide, _margin: &mut Vector<f32>) {
        unimplemented!();
    }
    fn compute_local_support(
        _shape: &TriangleWide,
        _direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        _support: &mut Vector3Wide,
    ) {
        unimplemented!();
    }
    #[inline(always)]
    fn compute_support(
        shape: &TriangleWide,
        _orientation: &Matrix3x3Wide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let mut a_dot = Vector::<f32>::splat(0.0);
        let mut b_dot = Vector::<f32>::splat(0.0);
        let mut c_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&shape.a, direction, &mut a_dot);
        Vector3Wide::dot(&shape.b, direction, &mut b_dot);
        Vector3Wide::dot(&shape.c, direction, &mut c_dot);
        let max = a_dot.simd_max(b_dot.simd_max(c_dot));
        *support = Vector3Wide::conditional_select(
            &max.simd_eq(a_dot).to_int(),
            &shape.a,
            &shape.b,
        );
        *support = Vector3Wide::conditional_select(
            &max.simd_eq(c_dot).to_int(),
            &shape.c,
            support,
        );
    }
}

/// Pair tester for triangle vs cylinder collisions.
pub struct TriangleCylinderTester;

impl TriangleCylinderTester {
    pub const BATCH_SIZE: i32 = 16;

    #[inline(always)]
    unsafe fn try_add_interior_point(
        point: &Vector2Wide,
        feature_id: &Vector<i32>,
        projected_a: &Vector2Wide,
        projected_ab: &Vector2Wide,
        projected_b: &Vector2Wide,
        projected_bc: &Vector2Wide,
        projected_c: &Vector2Wide,
        projected_ca: &Vector2Wide,
        allow_contact: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let mut ap = Vector2Wide::default();
        let mut bp = Vector2Wide::default();
        let mut cp = Vector2Wide::default();
        Vector2Wide::subtract(point, projected_a, &mut ap);
        Vector2Wide::subtract(point, projected_b, &mut bp);
        Vector2Wide::subtract(point, projected_c, &mut cp);
        let ab_dot = ap.x * projected_ab.y - ap.y * projected_ab.x;
        let bc_dot = bp.x * projected_bc.y - bp.y * projected_bc.x;
        let ca_dot = cp.x * projected_ca.y - cp.y * projected_ca.x;
        let sum = ab_dot.simd_gt(zero_f).to_int()
            + bc_dot.simd_gt(zero_f).to_int()
            + ca_dot.simd_gt(zero_f).to_int();
        let contained = *allow_contact
            & (sum.simd_eq(Vector::<i32>::splat(0)).to_int()
                | sum.simd_eq(Vector::<i32>::splat(-3)).to_int());
        let mut candidate = MaybeUninit::<ManifoldCandidate>::uninit();
        let c = candidate.assume_init_mut();
        c.x = point.x;
        c.y = point.y;
        c.feature_id = *feature_id;
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            candidate.assume_init_ref(),
            &contained,
            pair_count,
        );
    }

    #[inline(always)]
    pub fn create_effective_triangle_face_normal(
        triangle_normal: &Vector3Wide,
        normal: &Vector3Wide,
        face_normal_a_dot_normal: Vector<f32>,
        inactive_lanes: Vector<i32>,
        effective_face_normal: &mut Vector3Wide,
        inverse_effective_face_normal_dot_normal: &mut Vector<f32>,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let abs_face_normal_a_dot_normal = face_normal_a_dot_normal.abs();
        let face_normal_fallback_threshold = Vector::<f32>::splat(1e-4);
        let needs_fallback_face_normal = (!inactive_lanes.simd_lt(Vector::<i32>::splat(0)))
            & abs_face_normal_a_dot_normal.simd_lt(face_normal_fallback_threshold);
        let needs_fallback_i = needs_fallback_face_normal.to_int();
        if needs_fallback_i.simd_eq(Vector::<i32>::splat(-1)).any() {
            let push_scale = ((abs_face_normal_a_dot_normal - face_normal_fallback_threshold)
                / -face_normal_fallback_threshold)
                .simd_max(zero_f);
            let push = Vector3Wide::scale(normal, &(push_scale * Vector::<f32>::splat(0.1)));
            let mut pushed = Vector3Wide::default();
            Vector3Wide::add(triangle_normal, &push, &mut pushed);
            let normalized = Vector3Wide::normalize(&pushed);
            *effective_face_normal =
                Vector3Wide::conditional_select(&needs_fallback_i, &normalized, triangle_normal);
        } else {
            *effective_face_normal = triangle_normal.clone();
        }
        let mut effective_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(effective_face_normal, normal, &mut effective_dot);
        *inverse_effective_face_normal_dot_normal = one_f / effective_dot;
    }

    /// Tests triangle vs cylinder collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &TriangleWide,
        b: &CylinderWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let zero_i = Vector::<i32>::splat(0);
        let neg_one = Vector::<i32>::splat(-1);
        let one_f = Vector::<f32>::splat(1.0);

        let mut world_ra = Matrix3x3Wide::default();
        let mut world_rb = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut world_ra);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut world_rb);
        let mut r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_ra, &world_rb, &mut r_a);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &world_rb, &mut local_offset_b);

        // Pretransform and recenter the triangle.
        let mut triangle = TriangleWide::default();
        Matrix3x3Wide::transform_without_overlap(&a.a, &r_a, &mut triangle.a);
        Matrix3x3Wide::transform_without_overlap(&a.b, &r_a, &mut triangle.b);
        Matrix3x3Wide::transform_without_overlap(&a.c, &r_a, &mut triangle.c);
        let mut centroid = Vector3Wide::default();
        Vector3Wide::add(&triangle.a, &triangle.b, &mut centroid);
        let centroid_tmp = centroid.clone();
        Vector3Wide::add(&triangle.c, &centroid_tmp, &mut centroid);
        let centroid_unscaled = centroid.clone();
        Vector3Wide::scale_to(&centroid_unscaled, &Vector::<f32>::splat(1.0 / 3.0), &mut centroid);
        let triangle_a_tmp = triangle.a.clone();
        Vector3Wide::subtract(&triangle_a_tmp, &centroid, &mut triangle.a);
        let triangle_b_tmp = triangle.b.clone();
        Vector3Wide::subtract(&triangle_b_tmp, &centroid, &mut triangle.b);
        let triangle_c_tmp = triangle.c.clone();
        Vector3Wide::subtract(&triangle_c_tmp, &centroid, &mut triangle.c);
        let mut local_triangle_center = Vector3Wide::default();
        Vector3Wide::subtract(&centroid, &local_offset_b, &mut local_triangle_center);

        let mut length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&local_triangle_center, &mut length);
        let mut initial_normal = Vector3Wide::default();
        Vector3Wide::scale_to(&local_triangle_center, &(one_f / length), &mut initial_normal);
        let use_fallback = length.simd_lt(Vector::<f32>::splat(1e-10));
        initial_normal.x = use_fallback.select(zero_f, initial_normal.x);
        initial_normal.y = use_fallback.select(one_f, initial_normal.y);
        initial_normal.z = use_fallback.select(zero_f, initial_normal.z);

        let mut triangle_ab = Vector3Wide::default();
        let mut triangle_bc = Vector3Wide::default();
        let mut triangle_ca = Vector3Wide::default();
        Vector3Wide::subtract(&triangle.b, &triangle.a, &mut triangle_ab);
        Vector3Wide::subtract(&triangle.c, &triangle.b, &mut triangle_bc);
        Vector3Wide::subtract(&triangle.a, &triangle.c, &mut triangle_ca);
        let mut triangle_a_world = Vector3Wide::default();
        let mut triangle_b_world = Vector3Wide::default();
        let mut triangle_c_world = Vector3Wide::default();
        Vector3Wide::add(&triangle.a, &local_triangle_center, &mut triangle_a_world);
        Vector3Wide::add(&triangle.b, &local_triangle_center, &mut triangle_b_world);
        Vector3Wide::add(&triangle.c, &local_triangle_center, &mut triangle_c_world);
        let mut triangle_normal = Vector3Wide::default();
        Vector3Wide::cross(&triangle_ab, &triangle_ca, &mut triangle_normal);
        let mut triangle_normal_length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&triangle_normal, &mut triangle_normal_length);
        let tn_tmp = triangle_normal.clone();
        Vector3Wide::scale_to(&tn_tmp, &(one_f / triangle_normal_length), &mut triangle_normal);

        // Check if the cylinder position is within the triangle and below the plane.
        let mut cylinder_to_triangle_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle_normal, &local_triangle_center, &mut cylinder_to_triangle_dot);
        let cylinder_below_plane = cylinder_to_triangle_dot.simd_ge(zero_f);
        let mut edge_plane_ab = Vector3Wide::default();
        let mut edge_plane_bc = Vector3Wide::default();
        let mut edge_plane_ca = Vector3Wide::default();
        Vector3Wide::cross(&triangle_ab, &triangle_normal, &mut edge_plane_ab);
        Vector3Wide::cross(&triangle_bc, &triangle_normal, &mut edge_plane_bc);
        Vector3Wide::cross(&triangle_ca, &triangle_normal, &mut edge_plane_ca);
        let mut ab_plane_test = Vector::<f32>::splat(0.0);
        let mut bc_plane_test = Vector::<f32>::splat(0.0);
        let mut ca_plane_test = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&edge_plane_ab, &triangle_a_world, &mut ab_plane_test);
        Vector3Wide::dot(&edge_plane_bc, &triangle_b_world, &mut bc_plane_test);
        Vector3Wide::dot(&edge_plane_ca, &triangle_c_world, &mut ca_plane_test);
        let cylinder_inside_edge_planes =
            ab_plane_test.simd_le(zero_f).to_int()
                & bc_plane_test.simd_le(zero_f).to_int()
                & ca_plane_test.simd_le(zero_f).to_int();
        let cylinder_inside_and_below = cylinder_inside_edge_planes & cylinder_below_plane.to_int();

        let mut inactive_lanes = BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut triangle_epsilon_scale = Vector::<f32>::splat(0.0);
        let mut nondegenerate_mask = Vector::<i32>::splat(0);
        TriangleWide::compute_nondegenerate_triangle_mask_from_edges(
            &triangle_ab, &triangle_ca, &triangle_normal_length,
            &mut triangle_epsilon_scale, &mut nondegenerate_mask,
        );
        inactive_lanes = inactive_lanes | !nondegenerate_mask;
        inactive_lanes = inactive_lanes | cylinder_inside_and_below;
        if inactive_lanes.simd_lt(zero_i).all() {
            *manifold = std::mem::zeroed();
            return;
        }

        let cylinder_support_finder = CylinderSupportFinder;
        let triangle_support_finder = PretransformedTriangleSupportFinder;

        // Create a simplex entry for the triangle face normal.
        let mut negated_triangle_normal = Vector3Wide::default();
        Vector3Wide::negate(&triangle_normal, &mut negated_triangle_normal);
        let mut cylinder_support_along_negated_tn = Vector3Wide::default();
        <CylinderSupportFinder as ISupportFinder<CylinderWide>>::compute_local_support(
            b, &negated_triangle_normal, &inactive_lanes, &mut cylinder_support_along_negated_tn,
        );
        let mut negated_tn_support = Vector3Wide::default();
        Vector3Wide::subtract(
            &cylinder_support_along_negated_tn, &local_triangle_center, &mut negated_tn_support,
        );
        let mut triangle_face_depth = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&negated_tn_support, &negated_triangle_normal, &mut triangle_face_depth);

        // Check if the extreme point is within the triangle bounds.
        let mut closest_to_a = Vector3Wide::default();
        let mut closest_to_b = Vector3Wide::default();
        let mut closest_to_c = Vector3Wide::default();
        Vector3Wide::subtract(&triangle_a_world, &cylinder_support_along_negated_tn, &mut closest_to_a);
        Vector3Wide::subtract(&triangle_b_world, &cylinder_support_along_negated_tn, &mut closest_to_b);
        Vector3Wide::subtract(&triangle_c_world, &cylinder_support_along_negated_tn, &mut closest_to_c);
        let mut extreme_ab_test = Vector::<f32>::splat(0.0);
        let mut extreme_bc_test = Vector::<f32>::splat(0.0);
        let mut extreme_ca_test = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&edge_plane_ab, &closest_to_a, &mut extreme_ab_test);
        Vector3Wide::dot(&edge_plane_bc, &closest_to_b, &mut extreme_bc_test);
        Vector3Wide::dot(&edge_plane_ca, &closest_to_c, &mut extreme_ca_test);
        let triangle_normal_is_minimal = (cylinder_inside_edge_planes & (!cylinder_below_plane.to_int()))
            & extreme_ab_test.simd_le(zero_f).to_int()
            & extreme_bc_test.simd_le(zero_f).to_int()
            & extreme_ca_test.simd_le(zero_f).to_int();

        let depth_threshold = -*speculative_margin;
        let skip_depth_refine = triangle_normal_is_minimal | inactive_lanes;
        let epsilon_scale = b.half_length.simd_max(b.radius);
        let local_normal;
        let closest_on_b;
        let depth;

        if skip_depth_refine.simd_eq(zero_i).any() {
            let mut refined_depth = Vector::<f32>::splat(0.0);
            let mut refined_normal = Vector3Wide::default();
            let mut refined_closest_on_hull = Vector3Wide::default();
            DepthRefiner::find_minimum_depth_with_witness(
                b, &triangle, &local_triangle_center, &r_a,
                &cylinder_support_finder, &triangle_support_finder,
                &initial_normal, &skip_depth_refine,
                &(Vector::<f32>::splat(1e-5) * epsilon_scale),
                &depth_threshold,
                &mut refined_depth, &mut refined_normal, &mut refined_closest_on_hull, 25,
            );
            closest_on_b = Vector3Wide::conditional_select(
                &skip_depth_refine, &cylinder_support_along_negated_tn, &refined_closest_on_hull,
            );
            local_normal = Vector3Wide::conditional_select(
                &skip_depth_refine, &negated_triangle_normal, &refined_normal,
            );
            depth = skip_depth_refine.simd_lt(zero_i).select(triangle_face_depth, refined_depth);
        } else {
            local_normal = negated_triangle_normal.clone();
            closest_on_b = cylinder_support_along_negated_tn.clone();
            depth = triangle_face_depth;
        }

        let mut face_normal_a_dot_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle_normal, &local_normal, &mut face_normal_a_dot_normal);
        inactive_lanes = inactive_lanes
            | face_normal_a_dot_normal
                .simd_gt(Vector::<f32>::splat(
                    -TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                ))
                .to_int()
            | depth.simd_lt(depth_threshold).to_int();
        if inactive_lanes.simd_lt(zero_i).all() {
            *manifold = std::mem::zeroed();
            return;
        }

        let use_triangle_edge_case = (!inactive_lanes.simd_lt(zero_i))
            & face_normal_a_dot_normal.abs().simd_lt(Vector::<f32>::splat(0.2));
        let use_triangle_edge_case_i = use_triangle_edge_case.to_int();

        let cap_center_by = local_normal.y.simd_lt(zero_f).select(-b.half_length, b.half_length);
        let use_cap = (!inactive_lanes.simd_lt(zero_i))
            & local_normal.y.abs().simd_gt(Vector::<f32>::splat(0.70710678118));
        let use_cap_i = use_cap.to_int();

        let mut local_offset_b0 = Vector3Wide::default();
        let mut local_offset_b1 = Vector3Wide::default();
        let mut local_offset_b2 = Vector3Wide::default();
        let mut local_offset_b3 = Vector3Wide::default();

        if use_cap_i.simd_lt(zero_i).any() {
            // Cap-face manifold.
            let mut candidates_buf: [MaybeUninit<ManifoldCandidate>; 10] =
                MaybeUninit::uninit().assume_init();
            for c in candidates_buf.iter_mut() {
                c.write(ManifoldCandidate::default());
            }
            let candidates = candidates_buf[0].as_mut_ptr();
            let mut candidate_count = zero_i;

            let inverse_local_normal_y = one_f / local_normal.y;
            let mut p_a = Vector2Wide::default();
            let mut p_b = Vector2Wide::default();
            let mut p_c = Vector2Wide::default();
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by, &inverse_local_normal_y, &local_normal, &triangle_a_world, &mut p_a,
            );
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by, &inverse_local_normal_y, &local_normal, &triangle_b_world, &mut p_b,
            );
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by, &inverse_local_normal_y, &local_normal, &triangle_c_world, &mut p_c,
            );
            let mut projected_ab = Vector2Wide::default();
            let mut projected_bc = Vector2Wide::default();
            let mut projected_ca = Vector2Wide::default();
            Vector2Wide::subtract(&p_b, &p_a, &mut projected_ab);
            Vector2Wide::subtract(&p_c, &p_b, &mut projected_bc);
            Vector2Wide::subtract(&p_a, &p_c, &mut projected_ca);
            let mut t_min_ab = zero_f;
            let mut t_max_ab = zero_f;
            let mut intersected_ab = zero_i;
            let mut t_min_bc = zero_f;
            let mut t_max_bc = zero_f;
            let mut intersected_bc = zero_i;
            let mut t_min_ca = zero_f;
            let mut t_max_ca = zero_f;
            let mut intersected_ca = zero_i;
            BoxCylinderTester::intersect_line_circle(
                &p_a, &projected_ab, &b.radius,
                &mut t_min_ab, &mut t_max_ab, &mut intersected_ab,
            );
            BoxCylinderTester::intersect_line_circle(
                &p_b, &projected_bc, &b.radius,
                &mut t_min_bc, &mut t_max_bc, &mut intersected_bc,
            );
            BoxCylinderTester::intersect_line_circle(
                &p_c, &projected_ca, &b.radius,
                &mut t_min_ca, &mut t_max_ca, &mut intersected_ca,
            );
            t_min_ab = t_min_ab.simd_max(zero_f).simd_min(one_f);
            t_max_ab = t_max_ab.simd_max(zero_f).simd_min(one_f);
            t_min_bc = t_min_bc.simd_max(zero_f).simd_min(one_f);
            t_max_bc = t_max_bc.simd_max(zero_f).simd_min(one_f);
            t_min_ca = t_min_ca.simd_max(zero_f).simd_min(one_f);
            t_max_ca = t_max_ca.simd_max(zero_f).simd_min(one_f);

            // Use triangle tangent space for contact manifold.
            let mut triangle_ab_length = Vector::<f32>::splat(0.0);
            Vector3Wide::length_into(&triangle_ab, &mut triangle_ab_length);
            let mut triangle_tangent_x = Vector3Wide::default();
            Vector3Wide::scale_to(&triangle_ab, &(one_f / triangle_ab_length), &mut triangle_tangent_x);
            let mut triangle_tangent_y = Vector3Wide::default();
            Vector3Wide::cross(&triangle_tangent_x, &triangle_normal, &mut triangle_tangent_y);

            let mut tangent_a = Vector2Wide::default();
            let mut tangent_b = Vector2Wide::default();
            let mut tangent_c = Vector2Wide::default();
            Vector3Wide::dot(&triangle.a, &triangle_tangent_x, &mut tangent_a.x);
            Vector3Wide::dot(&triangle.a, &triangle_tangent_y, &mut tangent_a.y);
            Vector3Wide::dot(&triangle.b, &triangle_tangent_x, &mut tangent_b.x);
            Vector3Wide::dot(&triangle.b, &triangle_tangent_y, &mut tangent_b.y);
            Vector3Wide::dot(&triangle.c, &triangle_tangent_x, &mut tangent_c.x);
            Vector3Wide::dot(&triangle.c, &triangle_tangent_y, &mut tangent_c.y);

            let mut tangent_ab_dir = Vector2Wide::default();
            let mut tangent_bc_dir = Vector2Wide::default();
            let mut tangent_ca_dir = Vector2Wide::default();
            Vector2Wide::subtract(&tangent_b, &tangent_a, &mut tangent_ab_dir);
            Vector2Wide::subtract(&tangent_c, &tangent_b, &mut tangent_bc_dir);
            Vector2Wide::subtract(&tangent_a, &tangent_c, &mut tangent_ca_dir);

            BoxCylinderTester::add_candidate_for_edge(
                &tangent_a, &tangent_ab_dir, &t_min_ab, &t_max_ab, &intersected_ab,
                &zero_i, &use_cap_i, pair_count, candidates, &mut candidate_count,
            );
            BoxCylinderTester::add_candidate_for_edge(
                &tangent_b, &tangent_bc_dir, &t_min_bc, &t_max_bc, &intersected_bc,
                &Vector::<i32>::splat(1), &use_cap_i, pair_count, candidates, &mut candidate_count,
            );
            BoxCylinderTester::add_candidate_for_edge(
                &tangent_c, &tangent_ca_dir, &t_min_ca, &t_max_ca, &intersected_ca,
                &Vector::<i32>::splat(2), &use_cap_i, pair_count, candidates, &mut candidate_count,
            );

            let use_cap_triangle_face = use_cap & !use_triangle_edge_case;
            let use_cap_triangle_face_i = use_cap_triangle_face.to_int();
            let mut maximum_contact_count_in_bundle = 6;
            if use_cap_triangle_face_i.simd_lt(zero_i).any() {
                maximum_contact_count_in_bundle = 10;
                let mut interior_on_cyl0 = Vector2Wide::default();
                let mut interior_on_cyl1 = Vector2Wide::default();
                let mut interior_on_cyl2 = Vector2Wide::default();
                let mut interior_on_cyl3 = Vector2Wide::default();
                BoxCylinderTester::generate_interior_points(
                    b, &local_normal, &closest_on_b,
                    &mut interior_on_cyl0, &mut interior_on_cyl1,
                    &mut interior_on_cyl2, &mut interior_on_cyl3,
                );
                let inverse_denominator = Vector::<f32>::splat(-1.0) / face_normal_a_dot_normal;
                let y_offset = local_triangle_center.y - cap_center_by;
                let x_offset0 = local_triangle_center.x - interior_on_cyl0.x;
                let z_offset0 = local_triangle_center.z - interior_on_cyl0.y;
                let x_offset1 = local_triangle_center.x - interior_on_cyl1.x;
                let z_offset1 = local_triangle_center.z - interior_on_cyl1.y;
                let x_offset2 = local_triangle_center.x - interior_on_cyl2.x;
                let z_offset2 = local_triangle_center.z - interior_on_cyl2.y;
                let x_offset3 = local_triangle_center.x - interior_on_cyl3.x;
                let z_offset3 = local_triangle_center.z - interior_on_cyl3.y;
                let t0 = (x_offset0 * local_normal.x + y_offset * local_normal.y + z_offset0 * local_normal.z)
                    * inverse_denominator;
                let t1 = (x_offset1 * local_normal.x + y_offset * local_normal.y + z_offset1 * local_normal.z)
                    * inverse_denominator;
                let t2 = (x_offset2 * local_normal.x + y_offset * local_normal.y + z_offset2 * local_normal.z)
                    * inverse_denominator;
                let t3 = (x_offset3 * local_normal.x + y_offset * local_normal.y + z_offset3 * local_normal.z)
                    * inverse_denominator;

                let mut tangent_local_normal = Vector2Wide::default();
                Vector3Wide::dot(&local_normal, &triangle_tangent_x, &mut tangent_local_normal.x);
                Vector3Wide::dot(&local_normal, &triangle_tangent_y, &mut tangent_local_normal.y);
                let y_on_tangent_x = y_offset * triangle_tangent_x.y;
                let y_on_tangent_y = y_offset * triangle_tangent_y.y;
                let interior0 = Vector2Wide {
                    x: tangent_local_normal.x * t0 - x_offset0 * triangle_tangent_x.x - y_on_tangent_x - z_offset0 * triangle_tangent_x.z,
                    y: tangent_local_normal.y * t0 - x_offset0 * triangle_tangent_y.x - y_on_tangent_y - z_offset0 * triangle_tangent_y.z,
                };
                let interior1 = Vector2Wide {
                    x: tangent_local_normal.x * t1 - x_offset1 * triangle_tangent_x.x - y_on_tangent_x - z_offset1 * triangle_tangent_x.z,
                    y: tangent_local_normal.y * t1 - x_offset1 * triangle_tangent_y.x - y_on_tangent_y - z_offset1 * triangle_tangent_y.z,
                };
                let interior2 = Vector2Wide {
                    x: tangent_local_normal.x * t2 - x_offset2 * triangle_tangent_x.x - y_on_tangent_x - z_offset2 * triangle_tangent_x.z,
                    y: tangent_local_normal.y * t2 - x_offset2 * triangle_tangent_y.x - y_on_tangent_y - z_offset2 * triangle_tangent_y.z,
                };
                let interior3 = Vector2Wide {
                    x: tangent_local_normal.x * t3 - x_offset3 * triangle_tangent_x.x - y_on_tangent_x - z_offset3 * triangle_tangent_x.z,
                    y: tangent_local_normal.y * t3 - x_offset3 * triangle_tangent_y.x - y_on_tangent_y - z_offset3 * triangle_tangent_y.z,
                };

                Self::try_add_interior_point(
                    &interior0, &Vector::<i32>::splat(8),
                    &tangent_a, &tangent_ab_dir, &tangent_b, &tangent_bc_dir, &tangent_c, &tangent_ca_dir,
                    &use_cap_triangle_face_i, candidates, &mut candidate_count, pair_count,
                );
                Self::try_add_interior_point(
                    &interior1, &Vector::<i32>::splat(9),
                    &tangent_a, &tangent_ab_dir, &tangent_b, &tangent_bc_dir, &tangent_c, &tangent_ca_dir,
                    &use_cap_triangle_face_i, candidates, &mut candidate_count, pair_count,
                );
                Self::try_add_interior_point(
                    &interior2, &Vector::<i32>::splat(10),
                    &tangent_a, &tangent_ab_dir, &tangent_b, &tangent_bc_dir, &tangent_c, &tangent_ca_dir,
                    &use_cap_triangle_face_i, candidates, &mut candidate_count, pair_count,
                );
                Self::try_add_interior_point(
                    &interior3, &Vector::<i32>::splat(11),
                    &tangent_a, &tangent_ab_dir, &tangent_b, &tangent_bc_dir, &tangent_c, &tangent_ca_dir,
                    &use_cap_triangle_face_i, candidates, &mut candidate_count, pair_count,
                );
            }

            let mut cap_normal = Vector3Wide::default();
            cap_normal.x = zero_f;
            cap_normal.y = local_normal.y.simd_lt(zero_f).select(one_f, Vector::<f32>::splat(-1.0));
            cap_normal.z = zero_f;
            let mut triangle_center_to_cap_center = Vector3Wide::default();
            triangle_center_to_cap_center.x = -local_triangle_center.x;
            triangle_center_to_cap_center.y = cap_center_by - local_triangle_center.y;
            triangle_center_to_cap_center.z = -local_triangle_center.z;
            let mut candidate0 = ManifoldCandidate::default();
            let mut candidate1 = ManifoldCandidate::default();
            let mut candidate2 = ManifoldCandidate::default();
            let mut candidate3 = ManifoldCandidate::default();
            ManifoldCandidateHelper::reduce(
                candidates,
                candidate_count,
                maximum_contact_count_in_bundle,
                &cap_normal,
                -cap_normal.y / local_normal.y,
                &triangle_center_to_cap_center,
                &triangle_tangent_x,
                &triangle_tangent_y,
                epsilon_scale,
                depth_threshold,
                pair_count,
                &mut candidate0,
                &mut candidate1,
                &mut candidate2,
                &mut candidate3,
                &mut manifold.contact0_exists,
                &mut manifold.contact1_exists,
                &mut manifold.contact2_exists,
                &mut manifold.contact3_exists,
            );

            local_offset_b0.x = triangle_tangent_x.x * candidate0.x + triangle_tangent_y.x * candidate0.y + local_triangle_center.x;
            local_offset_b0.y = triangle_tangent_x.y * candidate0.x + triangle_tangent_y.y * candidate0.y + local_triangle_center.y;
            local_offset_b0.z = triangle_tangent_x.z * candidate0.x + triangle_tangent_y.z * candidate0.y + local_triangle_center.z;
            local_offset_b1.x = triangle_tangent_x.x * candidate1.x + triangle_tangent_y.x * candidate1.y + local_triangle_center.x;
            local_offset_b1.y = triangle_tangent_x.y * candidate1.x + triangle_tangent_y.y * candidate1.y + local_triangle_center.y;
            local_offset_b1.z = triangle_tangent_x.z * candidate1.x + triangle_tangent_y.z * candidate1.y + local_triangle_center.z;
            local_offset_b2.x = triangle_tangent_x.x * candidate2.x + triangle_tangent_y.x * candidate2.y + local_triangle_center.x;
            local_offset_b2.y = triangle_tangent_x.y * candidate2.x + triangle_tangent_y.y * candidate2.y + local_triangle_center.y;
            local_offset_b2.z = triangle_tangent_x.z * candidate2.x + triangle_tangent_y.z * candidate2.y + local_triangle_center.z;
            local_offset_b3.x = triangle_tangent_x.x * candidate3.x + triangle_tangent_y.x * candidate3.y + local_triangle_center.x;
            local_offset_b3.y = triangle_tangent_x.y * candidate3.x + triangle_tangent_y.y * candidate3.y + local_triangle_center.y;
            local_offset_b3.z = triangle_tangent_x.z * candidate3.x + triangle_tangent_y.z * candidate3.y + local_triangle_center.z;

            manifold.feature_id0 = candidate0.feature_id;
            manifold.feature_id1 = candidate1.feature_id;
            manifold.feature_id2 = candidate2.feature_id;
            manifold.feature_id3 = candidate3.feature_id;
            manifold.depth0 = candidate0.depth;
            manifold.depth1 = candidate1.depth;
            manifold.depth2 = candidate2.depth;
            manifold.depth3 = candidate3.depth;
        } else {
            manifold.contact0_exists = zero_i;
            manifold.contact1_exists = zero_i;
            manifold.contact2_exists = zero_i;
            manifold.contact3_exists = zero_i;
        }

        let use_side = (!use_cap) & (!inactive_lanes.simd_lt(zero_i));
        let use_side_i = use_side.to_int();
        if use_side_i.simd_lt(zero_i).any() {
            let use_side_edge_case = use_side & use_triangle_edge_case;
            let use_side_edge_case_i = use_side_edge_case.to_int();
            let mut depth_t_min = Vector::<f32>::splat(0.0);
            let mut depth_t_max = Vector::<f32>::splat(0.0);
            let mut cylinder_t_min = Vector::<f32>::splat(0.0);
            let mut cylinder_t_max = Vector::<f32>::splat(0.0);

            // Identify dominant edge.
            let mut ab_edge_alignment = Vector::<f32>::splat(0.0);
            let mut bc_edge_alignment = Vector::<f32>::splat(0.0);
            let mut ca_edge_alignment = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&edge_plane_ab, &local_normal, &mut ab_edge_alignment);
            Vector3Wide::dot(&edge_plane_bc, &local_normal, &mut bc_edge_alignment);
            Vector3Wide::dot(&edge_plane_ca, &local_normal, &mut ca_edge_alignment);
            let max_alignment = ab_edge_alignment.simd_max(bc_edge_alignment.simd_max(ca_edge_alignment));
            let ab_is_dominant = max_alignment.simd_eq(ab_edge_alignment);
            let bc_is_dominant = max_alignment.simd_eq(bc_edge_alignment);
            let ab_is_dominant_i = ab_is_dominant.to_int();
            let bc_is_dominant_i = bc_is_dominant.to_int();

            let mut dominant_edge_start = Vector3Wide::conditional_select(&ab_is_dominant_i, &triangle_a_world, &triangle_c_world);
            dominant_edge_start = Vector3Wide::conditional_select(&bc_is_dominant_i, &triangle_b_world, &dominant_edge_start);
            let mut dominant_edge_offset = Vector3Wide::conditional_select(&ab_is_dominant_i, &triangle_ab, &triangle_ca);
            dominant_edge_offset = Vector3Wide::conditional_select(&bc_is_dominant_i, &triangle_bc, &dominant_edge_offset);

            // Parallelism check for the dominant edge.
            let dominant_edge_dot_horizontal_normal = dominant_edge_offset.z * local_normal.x - dominant_edge_offset.x * local_normal.z;
            let dominant_edge_dot_hn_sq = dominant_edge_dot_horizontal_normal * dominant_edge_dot_horizontal_normal;
            let dominant_edge_length_squared = dominant_edge_offset.length_squared();
            let horizontal_normal_length_squared = local_normal.x * local_normal.x + local_normal.z * local_normal.z;
            let interpolation_scale = dominant_edge_length_squared * horizontal_normal_length_squared;
            let lower_sin_sq = 0.01f32 * 0.01;
            let upper_sin_sq = 0.02f32 * 0.02;
            let interpolation_min = Vector::<f32>::splat(lower_sin_sq);
            let inverse_interpolation_span = Vector::<f32>::splat(1.0 / (upper_sin_sq - lower_sin_sq));
            let restrict_weight = ((dominant_edge_dot_hn_sq / interpolation_scale - interpolation_min)
                * inverse_interpolation_span)
                .simd_clamp(zero_f, one_f);

            if use_side_edge_case_i.simd_lt(zero_i).any() {
                // Edge-side case.
                let cyl_edge_to_dom_x = dominant_edge_start.x - closest_on_b.x;
                let cyl_edge_to_dom_z = dominant_edge_start.z - closest_on_b.z;
                let numerator = cyl_edge_to_dom_x * local_normal.z - cyl_edge_to_dom_z * local_normal.x;
                let edge_t = numerator / dominant_edge_dot_horizontal_normal;

                let inverse_edge_offset_length_squared = one_f / dominant_edge_length_squared;
                let t_center = -(cyl_edge_to_dom_x * dominant_edge_offset.x
                    + dominant_edge_start.y * dominant_edge_offset.y
                    + cyl_edge_to_dom_z * dominant_edge_offset.z)
                    * inverse_edge_offset_length_squared;
                let projected_extent_offset = b.half_length * dominant_edge_offset.y.abs() * inverse_edge_offset_length_squared;
                let cyl_t_min_unrestricted = t_center - projected_extent_offset;
                let cyl_t_max_unrestricted = t_center + projected_extent_offset;
                let regular_contribution = restrict_weight
                    * dominant_edge_dot_hn_sq
                        .simd_lt(interpolation_min)
                        .select(t_center, edge_t);
                let unrestrict_weight = one_f - restrict_weight;
                cylinder_t_min = regular_contribution + unrestrict_weight * cyl_t_min_unrestricted;
                cylinder_t_max = regular_contribution + unrestrict_weight * cyl_t_max_unrestricted;
                cylinder_t_min = cylinder_t_min.simd_max(zero_f).simd_min(one_f);
                cylinder_t_max = cylinder_t_max.simd_max(zero_f).simd_min(one_f);

                let inverse_depth_denominator = Vector::<f32>::splat(-1.0) / horizontal_normal_length_squared;
                let depth_base = (cyl_edge_to_dom_x * local_normal.x + cyl_edge_to_dom_z * local_normal.z) * inverse_depth_denominator;
                let t_depth_scale = (dominant_edge_offset.x * local_normal.x + dominant_edge_offset.z * local_normal.z) * inverse_depth_denominator;
                depth_t_min = depth_base + t_depth_scale * cylinder_t_min;
                depth_t_max = depth_base + t_depth_scale * cylinder_t_max;

                let min_offset = Vector3Wide::scale(&dominant_edge_offset, &cylinder_t_min);
                let max_offset = Vector3Wide::scale(&dominant_edge_offset, &cylinder_t_max);
                local_offset_b0.x = use_side_edge_case.select(dominant_edge_start.x + min_offset.x, local_offset_b0.x);
                local_offset_b0.y = use_side_edge_case.select(dominant_edge_start.y + min_offset.y, local_offset_b0.y);
                local_offset_b0.z = use_side_edge_case.select(dominant_edge_start.z + min_offset.z, local_offset_b0.z);
                local_offset_b1.x = use_side_edge_case.select(dominant_edge_start.x + max_offset.x, local_offset_b1.x);
                local_offset_b1.y = use_side_edge_case.select(dominant_edge_start.y + max_offset.y, local_offset_b1.y);
                local_offset_b1.z = use_side_edge_case.select(dominant_edge_start.z + max_offset.z, local_offset_b1.z);
            }

            let use_side_triangle_face = use_side & !use_triangle_edge_case;
            let use_side_triangle_face_i = use_side_triangle_face.to_int();
            if use_side_triangle_face_i.simd_lt(zero_i).any() {
                // Side-triangle face case.
                let inverse_denominator = one_f / face_normal_a_dot_normal;
                let xz_contribution =
                    (local_triangle_center.x - closest_on_b.x) * triangle_normal.x
                    + (local_triangle_center.z - closest_on_b.z) * triangle_normal.z;
                let t_min_to_triangle =
                    (xz_contribution + (local_triangle_center.y + b.half_length) * triangle_normal.y) * inverse_denominator;
                let t_max_to_triangle =
                    (xz_contribution + (local_triangle_center.y - b.half_length) * triangle_normal.y) * inverse_denominator;
                let mut min_on_triangle = Vector3Wide::default();
                min_on_triangle.x = t_min_to_triangle * local_normal.x + closest_on_b.x;
                min_on_triangle.y = t_min_to_triangle * local_normal.y - b.half_length;
                min_on_triangle.z = t_min_to_triangle * local_normal.z + closest_on_b.z;
                let mut max_on_triangle = Vector3Wide::default();
                max_on_triangle.x = t_max_to_triangle * local_normal.x + closest_on_b.x;
                max_on_triangle.y = t_max_to_triangle * local_normal.y + b.half_length;
                max_on_triangle.z = t_max_to_triangle * local_normal.z + closest_on_b.z;
                let mut min_to_max = Vector3Wide::default();
                Vector3Wide::subtract(&max_on_triangle, &min_on_triangle, &mut min_to_max);

                // Ray-triangle edge plane intersections.
                let numerator_ab = (triangle_a_world.x - min_on_triangle.x) * edge_plane_ab.x
                    + (triangle_a_world.y - min_on_triangle.y) * edge_plane_ab.y
                    + (triangle_a_world.z - min_on_triangle.z) * edge_plane_ab.z;
                let numerator_bc = (triangle_b_world.x - min_on_triangle.x) * edge_plane_bc.x
                    + (triangle_b_world.y - min_on_triangle.y) * edge_plane_bc.y
                    + (triangle_b_world.z - min_on_triangle.z) * edge_plane_bc.z;
                let numerator_ca = (triangle_c_world.x - min_on_triangle.x) * edge_plane_ca.x
                    + (triangle_c_world.y - min_on_triangle.y) * edge_plane_ca.y
                    + (triangle_c_world.z - min_on_triangle.z) * edge_plane_ca.z;
                let mut denominator_ab =
                    min_to_max.x * edge_plane_ab.x + min_to_max.y * edge_plane_ab.y + min_to_max.z * edge_plane_ab.z;
                let mut denominator_bc =
                    min_to_max.x * edge_plane_bc.x + min_to_max.y * edge_plane_bc.y + min_to_max.z * edge_plane_bc.z;
                let mut denominator_ca =
                    min_to_max.x * edge_plane_ca.x + min_to_max.y * edge_plane_ca.y + min_to_max.z * edge_plane_ca.z;
                let threshold = Vector::<f32>::splat(1e-30);
                let negative_threshold = -threshold;
                let exiting_ab = denominator_ab.simd_le(zero_f);
                let exiting_bc = denominator_bc.simd_le(zero_f);
                let exiting_ca = denominator_ca.simd_le(zero_f);
                denominator_ab = denominator_ab.abs().simd_lt(threshold).select(
                    exiting_ab.select(negative_threshold, threshold),
                    denominator_ab,
                );
                denominator_bc = denominator_bc.abs().simd_lt(threshold).select(
                    exiting_bc.select(negative_threshold, threshold),
                    denominator_bc,
                );
                denominator_ca = denominator_ca.abs().simd_lt(threshold).select(
                    exiting_ca.select(negative_threshold, threshold),
                    denominator_ca,
                );
                let edge_t_ab = numerator_ab / denominator_ab;
                let edge_t_bc = numerator_bc / denominator_bc;
                let edge_t_ca = numerator_ca / denominator_ca;

                let min_value = Vector::<f32>::splat(f32::MIN);
                let max_value = Vector::<f32>::splat(f32::MAX);
                let mut entry_ab = exiting_ab.select(min_value, edge_t_ab);
                let mut entry_bc = exiting_bc.select(min_value, edge_t_bc);
                let mut entry_ca = exiting_ca.select(min_value, edge_t_ca);
                let mut exit_ab = exiting_ab.select(edge_t_ab, max_value);
                let mut exit_bc = exiting_bc.select(edge_t_bc, max_value);
                let mut exit_ca = exiting_ca.select(edge_t_ca, max_value);

                let ca_is_dominant = (!ab_is_dominant) & (!bc_is_dominant);
                let ca_is_dominant_i = ca_is_dominant.to_int();
                let unrestrict_weight = one_f - restrict_weight;
                entry_ab = ab_is_dominant.select(entry_ab * restrict_weight, entry_ab);
                entry_bc = bc_is_dominant.select(entry_bc * restrict_weight, entry_bc);
                entry_ca = ca_is_dominant.select(entry_ca * restrict_weight, entry_ca);
                exit_ab = ab_is_dominant.select(exit_ab * restrict_weight + unrestrict_weight, exit_ab);
                exit_bc = bc_is_dominant.select(exit_bc * restrict_weight + unrestrict_weight, exit_bc);
                exit_ca = ca_is_dominant.select(exit_ca * restrict_weight + unrestrict_weight, exit_ca);

                let side_tri_cyl_t_min = entry_ab.simd_max(entry_bc.simd_max(entry_ca));
                let side_tri_cyl_t_max = exit_ab.simd_min(exit_bc.simd_min(exit_ca));

                // Vertex fallback for degenerate intervals.
                let use_vertex_fallback =
                    use_side_triangle_face & side_tri_cyl_t_max.simd_lt(side_tri_cyl_t_min);
                let use_vertex_fallback_i = use_vertex_fallback.to_int();
                let ab_contributed =
                    edge_t_ab.simd_eq(side_tri_cyl_t_min).to_int() | edge_t_ab.simd_eq(side_tri_cyl_t_max).to_int();
                let bc_contributed =
                    edge_t_bc.simd_eq(side_tri_cyl_t_min).to_int() | edge_t_bc.simd_eq(side_tri_cyl_t_max).to_int();
                let ca_contributed =
                    edge_t_ca.simd_eq(side_tri_cyl_t_min).to_int() | edge_t_ca.simd_eq(side_tri_cyl_t_max).to_int();
                let use_a = ca_contributed & ab_contributed;
                let use_b = ab_contributed & bc_contributed;
                let mut vertex_fallback = Vector3Wide::conditional_select(&use_a, &triangle_a_world, &triangle_c_world);
                vertex_fallback = Vector3Wide::conditional_select(&use_b, &triangle_b_world, &vertex_fallback);

                cylinder_t_min = use_side_triangle_face.select(
                    side_tri_cyl_t_min.simd_max(zero_f).simd_min(one_f),
                    cylinder_t_min,
                );
                cylinder_t_max = use_side_triangle_face.select(
                    side_tri_cyl_t_max.simd_max(zero_f).simd_min(one_f),
                    cylinder_t_max,
                );
                local_offset_b0.x = use_side_triangle_face.select(
                    min_on_triangle.x + min_to_max.x * cylinder_t_min,
                    local_offset_b0.x,
                );
                local_offset_b0.y = use_side_triangle_face.select(
                    min_on_triangle.y + min_to_max.y * cylinder_t_min,
                    local_offset_b0.y,
                );
                local_offset_b0.z = use_side_triangle_face.select(
                    min_on_triangle.z + min_to_max.z * cylinder_t_min,
                    local_offset_b0.z,
                );
                local_offset_b1.x = use_side_triangle_face.select(
                    min_on_triangle.x + min_to_max.x * cylinder_t_max,
                    local_offset_b1.x,
                );
                local_offset_b1.y = use_side_triangle_face.select(
                    min_on_triangle.y + min_to_max.y * cylinder_t_max,
                    local_offset_b1.y,
                );
                local_offset_b1.z = use_side_triangle_face.select(
                    min_on_triangle.z + min_to_max.z * cylinder_t_max,
                    local_offset_b1.z,
                );
                local_offset_b0 = Vector3Wide::conditional_select(
                    &use_vertex_fallback_i, &vertex_fallback, &local_offset_b0,
                );

                let inverse_depth_denom = one_f / (local_normal.x * local_normal.x + local_normal.z * local_normal.z);
                depth_t_min = use_side_triangle_face.select(
                    (local_normal.x * (closest_on_b.x - local_offset_b0.x) + local_normal.z * (closest_on_b.z - local_offset_b0.z)) * inverse_depth_denom,
                    depth_t_min,
                );
                depth_t_max = use_side_triangle_face.select(
                    (local_normal.x * (closest_on_b.x - local_offset_b1.x) + local_normal.z * (closest_on_b.z - local_offset_b1.z)) * inverse_depth_denom,
                    depth_t_max,
                );
            }
            manifold.feature_id0 = use_side_i.simd_ne(zero_i).select(zero_i, manifold.feature_id0);
            manifold.feature_id1 = use_side_i.simd_ne(zero_i).select(Vector::<i32>::splat(1), manifold.feature_id1);
            manifold.depth0 = use_side.select(depth_t_min, manifold.depth0);
            manifold.depth1 = use_side.select(depth_t_max, manifold.depth1);
            manifold.contact0_exists = use_side_i.simd_ne(zero_i).select(
                depth_t_min.simd_gt(depth_threshold).to_int(),
                manifold.contact0_exists,
            );
            manifold.contact1_exists = use_side_i.simd_ne(zero_i).select(
                depth_t_max.simd_gt(depth_threshold).to_int()
                    & cylinder_t_max.simd_gt(cylinder_t_min).to_int(),
                manifold.contact1_exists,
            );
            manifold.contact2_exists = use_side_i.simd_ne(zero_i).select(zero_i, manifold.contact2_exists);
            manifold.contact3_exists = use_side_i.simd_ne(zero_i).select(zero_i, manifold.contact3_exists);
        }

        Matrix3x3Wide::transform_without_overlap(&local_normal, &world_rb, &mut manifold.normal);

        let mut local_offset_a0 = Vector3Wide::default();
        let mut local_offset_a1 = Vector3Wide::default();
        let mut local_offset_a2 = Vector3Wide::default();
        let mut local_offset_a3 = Vector3Wide::default();
        Vector3Wide::add(&local_offset_b0, &local_offset_b, &mut local_offset_a0);
        Vector3Wide::add(&local_offset_b1, &local_offset_b, &mut local_offset_a1);
        Vector3Wide::add(&local_offset_b2, &local_offset_b, &mut local_offset_a2);
        Vector3Wide::add(&local_offset_b3, &local_offset_b, &mut local_offset_a3);
        Matrix3x3Wide::transform_without_overlap(&local_offset_a0, &world_rb, &mut manifold.offset_a0);
        Matrix3x3Wide::transform_without_overlap(&local_offset_a1, &world_rb, &mut manifold.offset_a1);
        Matrix3x3Wide::transform_without_overlap(&local_offset_a2, &world_rb, &mut manifold.offset_a2);
        Matrix3x3Wide::transform_without_overlap(&local_offset_a3, &world_rb, &mut manifold.offset_a3);

        // Mesh reductions use a face contact flag in the feature id.
        let face_collision_flag = face_normal_a_dot_normal
            .simd_lt(Vector::<f32>::splat(-mesh_reduction::MINIMUM_DOT_FOR_FACE_COLLISION))
            .select(
                Vector::<i32>::splat(mesh_reduction::FACE_COLLISION_FLAG),
                zero_i,
            );
        manifold.feature_id0 = manifold.feature_id0 + face_collision_flag;
    }
}
