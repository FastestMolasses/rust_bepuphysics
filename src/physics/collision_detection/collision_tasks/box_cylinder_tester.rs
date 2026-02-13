// Translated from BepuPhysics/CollisionDetection/CollisionTasks/BoxCylinderTester.cs

use crate::physics::collidables::box_shape::{BoxSupportFinder, BoxWide};
use crate::physics::collidables::cylinder::{CylinderSupportFinder, CylinderWide};
use crate::physics::collision_detection::collision_tasks::cylinder_pair_tester::CylinderPairTester;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidate, ManifoldCandidateHelper,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::mem::MaybeUninit;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for box vs cylinder collisions.
pub struct BoxCylinderTester;

impl BoxCylinderTester {
    pub const BATCH_SIZE: i32 = 16;

    #[inline(always)]
    pub(crate) fn intersect_line_circle(
        line_position: &Vector2Wide,
        line_direction: &Vector2Wide,
        radius: &Vector<f32>,
        t_min: &mut Vector<f32>,
        t_max: &mut Vector<f32>,
        intersected: &mut Vector<i32>,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let mut a_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_direction, line_direction, &mut a_val);
        a_val = a_val.simd_max(Vector::<f32>::splat(2e-38));
        let inverse_a = one_f / a_val;
        let mut b_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_position, line_direction, &mut b_val);
        let mut c_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_position, line_position, &mut c_val);
        let radius_squared = *radius * *radius;
        c_val = c_val - radius_squared;
        let d = b_val * b_val - a_val * c_val;
        *intersected = d.simd_ge(zero_f).to_int();
        let t_offset = StdFloat::sqrt(d.simd_max(zero_f)) * inverse_a;
        let t_base = -b_val * inverse_a;
        *t_min = t_base - t_offset;
        *t_max = t_base + t_offset;
    }

    #[inline(always)]
    pub(crate) unsafe fn add_candidate_for_edge(
        edge_start: &Vector2Wide,
        edge_offset: &Vector2Wide,
        t_min: &Vector<f32>,
        t_max: &Vector<f32>,
        intersected: &Vector<i32>,
        edge_id: &Vector<i32>,
        allow_feature_contacts: &Vector<i32>,
        pair_count: i32,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let _zero_i = Vector::<i32>::splat(0);
        let mut candidate = ManifoldCandidate::default();
        candidate.feature_id = *edge_id;
        candidate.x = edge_start.x + edge_offset.x * *t_min;
        candidate.y = edge_start.y + edge_offset.y * *t_min;
        let allow_contacts = *intersected & *allow_feature_contacts;
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            &candidate,
            &(allow_contacts & t_min.simd_lt(*t_max).to_int() & t_min.simd_gt(zero_f).to_int()),
            pair_count,
        );
        candidate.feature_id = *edge_id + Vector::<i32>::splat(4);
        candidate.x = edge_start.x + edge_offset.x * *t_max;
        candidate.y = edge_start.y + edge_offset.y * *t_max;
        ManifoldCandidateHelper::add_candidate(
            candidates,
            candidate_count,
            &candidate,
            &(allow_contacts & t_max.simd_gt(zero_f).to_int()),
            pair_count,
        );
    }

    #[inline(always)]
    pub(crate) fn generate_interior_points(
        cylinder: &CylinderWide,
        cylinder_local_normal: &Vector3Wide,
        local_closest_on_cylinder: &Vector3Wide,
        interior0: &mut Vector2Wide,
        interior1: &mut Vector2Wide,
        interior2: &mut Vector2Wide,
        interior3: &mut Vector2Wide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let interpolation_min = Vector::<f32>::splat(0.9999);
        let inverse_interpolation_span = Vector::<f32>::splat(1.0 / 0.00005);
        let parallel_weight = ((cylinder_local_normal.y.abs() - interpolation_min)
            * inverse_interpolation_span)
            .simd_clamp(zero_f, one_f);
        let deepest_weight = one_f - parallel_weight;
        let replace_x = local_closest_on_cylinder
            .x
            .abs()
            .simd_gt(local_closest_on_cylinder.z.abs());
        let replace0 = replace_x & local_closest_on_cylinder.x.simd_gt(zero_f);
        let replace1 = replace_x & local_closest_on_cylinder.x.simd_le(zero_f);
        let replace2 = !replace_x & local_closest_on_cylinder.z.simd_gt(zero_f);
        let replace3 = !replace_x & local_closest_on_cylinder.z.simd_le(zero_f);
        let scaled_radius = parallel_weight * cylinder.radius;
        interior0.x = replace0.select(
            deepest_weight * local_closest_on_cylinder.x + scaled_radius,
            cylinder.radius,
        );
        interior0.y = replace0.select(deepest_weight * local_closest_on_cylinder.z, zero_f);
        interior1.x = replace1.select(
            deepest_weight * local_closest_on_cylinder.x - scaled_radius,
            -cylinder.radius,
        );
        interior1.y = replace1.select(deepest_weight * local_closest_on_cylinder.z, zero_f);
        interior2.x = replace2.select(deepest_weight * local_closest_on_cylinder.x, zero_f);
        interior2.y = replace2.select(
            deepest_weight * local_closest_on_cylinder.z + scaled_radius,
            cylinder.radius,
        );
        interior3.x = replace3.select(deepest_weight * local_closest_on_cylinder.x, zero_f);
        interior3.y = replace3.select(
            deepest_weight * local_closest_on_cylinder.z - scaled_radius,
            -cylinder.radius,
        );
    }

    #[inline(always)]
    unsafe fn try_add_interior_point(
        point: &Vector2Wide,
        feature_id: &Vector<i32>,
        edge0010: &Vector2Wide,
        edge0010_plane_min: &Vector<f32>,
        edge0010_plane_max: &Vector<f32>,
        edge1011: &Vector2Wide,
        edge1011_plane_min: &Vector<f32>,
        edge1011_plane_max: &Vector<f32>,
        allow_contact: &Vector<i32>,
        candidates: *mut ManifoldCandidate,
        candidate_count: &mut Vector<i32>,
        pair_count: i32,
    ) {
        let edge0010_dot = point.x * edge0010.y - point.y * edge0010.x;
        let edge1011_dot = point.x * edge1011.y - point.y * edge1011.x;
        let contained = *allow_contact
            & (edge0010_dot.simd_ge(*edge0010_plane_min).to_int()
                & edge0010_dot.simd_le(*edge0010_plane_max).to_int())
            & (edge1011_dot.simd_ge(*edge1011_plane_min).to_int()
                & edge1011_dot.simd_le(*edge1011_plane_max).to_int());
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

    /// Tests box vs cylinder collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &BoxWide,
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
        let _neg_one = Vector::<i32>::splat(-1);
        let one_f = Vector::<f32>::splat(1.0);

        let mut world_ra = Matrix3x3Wide::default();
        let mut world_rb = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut world_ra);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut world_rb);
        let mut r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_ra, &world_rb, &mut r_a);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            &world_rb,
            &mut local_offset_b,
        );
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

        let mut length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&local_offset_a, &mut length);
        let mut local_normal = Vector3Wide::default();
        Vector3Wide::scale_to(&local_offset_a, &(one_f / length), &mut local_normal);
        let use_fallback = length.simd_lt(Vector::<f32>::splat(1e-10));
        local_normal.x = use_fallback.select(zero_f, local_normal.x);
        local_normal.y = use_fallback.select(one_f, local_normal.y);
        local_normal.z = use_fallback.select(zero_f, local_normal.z);

        let box_support_finder = BoxSupportFinder;
        let cylinder_support_finder = CylinderSupportFinder;

        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let depth_threshold = -*speculative_margin;
        let epsilon_scale = a
            .half_width
            .simd_max(a.half_height)
            .simd_max(a.half_length)
            .simd_min(b.half_length.simd_max(b.radius));

        let mut depth = Vector::<f32>::splat(0.0);
        let mut closest_on_b = Vector3Wide::default();
        let initial_normal = local_normal.clone();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &r_a,
            &cylinder_support_finder,
            &box_support_finder,
            &initial_normal,
            &inactive_lanes,
            &(epsilon_scale * Vector::<f32>::splat(1e-6)),
            &depth_threshold,
            &mut depth,
            &mut local_normal,
            &mut closest_on_b,
            25,
        );

        inactive_lanes = inactive_lanes | depth.simd_lt(depth_threshold).to_int();
        if inactive_lanes.simd_lt(zero_i).all() {
            *manifold = std::mem::zeroed();
            return;
        }

        // Identify the box face.
        let mut local_normal_in_a = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &local_normal,
            &r_a,
            &mut local_normal_in_a,
        );
        let abs_local_normal_in_a = local_normal_in_a.abs();
        let use_x = abs_local_normal_in_a.x.simd_gt(abs_local_normal_in_a.y)
            & abs_local_normal_in_a.x.simd_gt(abs_local_normal_in_a.z);
        let use_y = (!use_x) & abs_local_normal_in_a.y.simd_gt(abs_local_normal_in_a.z);
        let use_x_i = use_x.to_int();
        let use_y_i = use_y.to_int();
        let mut box_face_normal = Vector3Wide::conditional_select(&use_x_i, &r_a.x, &r_a.z);
        box_face_normal = Vector3Wide::conditional_select(&use_y_i, &r_a.y, &box_face_normal);
        let mut box_face_x = Vector3Wide::conditional_select(&use_x_i, &r_a.y, &r_a.x);
        box_face_x = Vector3Wide::conditional_select(&use_y_i, &r_a.z, &box_face_x);
        let mut box_face_y = Vector3Wide::conditional_select(&use_x_i, &r_a.z, &r_a.y);
        box_face_y = Vector3Wide::conditional_select(&use_y_i, &r_a.x, &box_face_y);
        let negate_face_mask = use_x.select(
            local_normal_in_a.x.simd_gt(zero_f).to_int(),
            use_y.select(
                local_normal_in_a.y.simd_gt(zero_f).to_int(),
                local_normal_in_a.z.simd_gt(zero_f).to_int(),
            ),
        );
        let negate_face = negate_face_mask;
        Vector3Wide::conditionally_negate(&negate_face, &mut box_face_normal);
        Vector3Wide::conditionally_negate(&negate_face, &mut box_face_x);
        Vector3Wide::conditionally_negate(&negate_face, &mut box_face_y);
        let box_face_half_width =
            use_x.select(a.half_height, use_y.select(a.half_length, a.half_width));
        let box_face_half_height =
            use_x.select(a.half_length, use_y.select(a.half_width, a.half_height));
        let box_face_normal_offset =
            use_x.select(a.half_width, use_y.select(a.half_height, a.half_length));
        let box_face_center_offset = Vector3Wide::scale(&box_face_normal, &box_face_normal_offset);
        let mut box_face_center = Vector3Wide::default();
        Vector3Wide::add(
            &box_face_center_offset,
            &local_offset_a,
            &mut box_face_center,
        );
        let box_face_x_offset = Vector3Wide::scale(&box_face_x, &box_face_half_width);
        let box_face_y_offset = Vector3Wide::scale(&box_face_y, &box_face_half_height);
        let mut v00 = Vector3Wide::default();
        Vector3Wide::subtract(&box_face_center, &box_face_x_offset, &mut v00);
        let v00_pre_y = v00.clone();
        Vector3Wide::subtract(&v00_pre_y, &box_face_y_offset, &mut v00);
        let mut v11 = Vector3Wide::default();
        Vector3Wide::add(&box_face_center, &box_face_x_offset, &mut v11);
        let v11_pre_y = v11.clone();
        Vector3Wide::add(&v11_pre_y, &box_face_y_offset, &mut v11);

        let cap_center_by = local_normal
            .y
            .simd_lt(zero_f)
            .select(-b.half_length, b.half_length);

        let use_cap = (!inactive_lanes.simd_lt(zero_i))
            & local_normal
                .y
                .abs()
                .simd_gt(Vector::<f32>::splat(0.70710678118));
        let use_cap_i = use_cap.to_int();

        let mut face_normal_dot_local_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &box_face_normal,
            &local_normal,
            &mut face_normal_dot_local_normal,
        );
        let inverse_face_normal_dot_local_normal = one_f / face_normal_dot_local_normal;

        if use_cap_i.simd_lt(zero_i).any() {
            // Cap-face manifold.
            let mut candidates_buf: [MaybeUninit<ManifoldCandidate>; 12] =
                MaybeUninit::uninit().assume_init();
            for c in candidates_buf.iter_mut() {
                c.write(ManifoldCandidate::default());
            }
            let candidates = candidates_buf[0].as_mut_ptr();
            let mut candidate_count = zero_i;

            let inverse_local_normal_y = one_f / local_normal.y;
            let mut v01 = Vector3Wide::default();
            Vector3Wide::subtract(&box_face_center, &box_face_x_offset, &mut v01);
            let v01_pre = v01.clone();
            Vector3Wide::add(&v01_pre, &box_face_y_offset, &mut v01);
            let mut v10 = Vector3Wide::default();
            Vector3Wide::add(&box_face_center, &box_face_x_offset, &mut v10);
            let v10_pre = v10.clone();
            Vector3Wide::subtract(&v10_pre, &box_face_y_offset, &mut v10);

            let mut p00 = Vector2Wide::default();
            let mut p01 = Vector2Wide::default();
            let mut p10 = Vector2Wide::default();
            let mut p11 = Vector2Wide::default();
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by,
                &inverse_local_normal_y,
                &local_normal,
                &v00,
                &mut p00,
            );
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by,
                &inverse_local_normal_y,
                &local_normal,
                &v01,
                &mut p01,
            );
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by,
                &inverse_local_normal_y,
                &local_normal,
                &v10,
                &mut p10,
            );
            CylinderPairTester::project_onto_cap_b(
                &cap_center_by,
                &inverse_local_normal_y,
                &local_normal,
                &v11,
                &mut p11,
            );

            let mut edge0010 = Vector2Wide::default();
            let mut edge1011 = Vector2Wide::default();
            let mut edge1101 = Vector2Wide::default();
            let mut edge0100 = Vector2Wide::default();
            Vector2Wide::subtract(&p10, &p00, &mut edge0010);
            Vector2Wide::subtract(&p11, &p10, &mut edge1011);
            Vector2Wide::subtract(&p01, &p11, &mut edge1101);
            Vector2Wide::subtract(&p00, &p01, &mut edge0100);

            let mut t_min0010 = zero_f;
            let mut t_max0010 = zero_f;
            let mut intersected0010 = zero_i;
            let mut t_min0100 = zero_f;
            let mut t_max0100 = zero_f;
            let mut intersected0100 = zero_i;
            let mut t_min1011 = zero_f;
            let mut t_max1011 = zero_f;
            let mut intersected1011 = zero_i;
            let mut t_min1101 = zero_f;
            let mut t_max1101 = zero_f;
            let mut intersected1101 = zero_i;
            Self::intersect_line_circle(
                &p00,
                &edge0010,
                &b.radius,
                &mut t_min0010,
                &mut t_max0010,
                &mut intersected0010,
            );
            Self::intersect_line_circle(
                &p01,
                &edge0100,
                &b.radius,
                &mut t_min0100,
                &mut t_max0100,
                &mut intersected0100,
            );
            Self::intersect_line_circle(
                &p10,
                &edge1011,
                &b.radius,
                &mut t_min1011,
                &mut t_max1011,
                &mut intersected1011,
            );
            Self::intersect_line_circle(
                &p11,
                &edge1101,
                &b.radius,
                &mut t_min1101,
                &mut t_max1101,
                &mut intersected1101,
            );

            t_min0010 = t_min0010.simd_max(zero_f).simd_min(one_f);
            t_max0010 = t_max0010.simd_max(zero_f).simd_min(one_f);
            t_min1101 = t_min1101.simd_max(zero_f).simd_min(one_f);
            t_max1101 = t_max1101.simd_max(zero_f).simd_min(one_f);
            t_min0100 = t_min0100.simd_max(zero_f).simd_min(one_f);
            t_max0100 = t_max0100.simd_max(zero_f).simd_min(one_f);
            t_min1011 = t_min1011.simd_max(zero_f).simd_min(one_f);
            t_max1011 = t_max1011.simd_max(zero_f).simd_min(one_f);

            Self::add_candidate_for_edge(
                &p00,
                &edge0010,
                &t_min0010,
                &t_max0010,
                &intersected0010,
                &zero_i,
                &use_cap_i,
                pair_count,
                candidates,
                &mut candidate_count,
            );
            Self::add_candidate_for_edge(
                &p01,
                &edge0100,
                &t_min0100,
                &t_max0100,
                &intersected0100,
                &Vector::<i32>::splat(1),
                &use_cap_i,
                pair_count,
                candidates,
                &mut candidate_count,
            );
            Self::add_candidate_for_edge(
                &p10,
                &edge1011,
                &t_min1011,
                &t_max1011,
                &intersected1011,
                &Vector::<i32>::splat(2),
                &use_cap_i,
                pair_count,
                candidates,
                &mut candidate_count,
            );
            Self::add_candidate_for_edge(
                &p11,
                &edge1101,
                &t_min1101,
                &t_max1101,
                &intersected1101,
                &Vector::<i32>::splat(3),
                &use_cap_i,
                pair_count,
                candidates,
                &mut candidate_count,
            );

            let mut interior0 = Vector2Wide::default();
            let mut interior1 = Vector2Wide::default();
            let mut interior2 = Vector2Wide::default();
            let mut interior3 = Vector2Wide::default();
            Self::generate_interior_points(
                b,
                &local_normal,
                &closest_on_b,
                &mut interior0,
                &mut interior1,
                &mut interior2,
                &mut interior3,
            );

            let edge0010_plane0 = p00.x * edge0010.y - p00.y * edge0010.x;
            let edge0010_plane1 = p01.x * edge0010.y - p01.y * edge0010.x;
            let edge1011_plane0 = p10.x * edge1011.y - p10.y * edge1011.x;
            let edge1011_plane1 = p00.x * edge1011.y - p00.y * edge1011.x;
            let edge0010_plane_min = edge0010_plane0.simd_min(edge0010_plane1);
            let edge0010_plane_max = edge0010_plane0.simd_max(edge0010_plane1);
            let edge1011_plane_min = edge1011_plane0.simd_min(edge1011_plane1);
            let edge1011_plane_max = edge1011_plane0.simd_max(edge1011_plane1);
            Self::try_add_interior_point(
                &interior0,
                &Vector::<i32>::splat(8),
                &edge0010,
                &edge0010_plane_min,
                &edge0010_plane_max,
                &edge1011,
                &edge1011_plane_min,
                &edge1011_plane_max,
                &use_cap_i,
                candidates,
                &mut candidate_count,
                pair_count,
            );
            Self::try_add_interior_point(
                &interior1,
                &Vector::<i32>::splat(9),
                &edge0010,
                &edge0010_plane_min,
                &edge0010_plane_max,
                &edge1011,
                &edge1011_plane_min,
                &edge1011_plane_max,
                &use_cap_i,
                candidates,
                &mut candidate_count,
                pair_count,
            );
            Self::try_add_interior_point(
                &interior2,
                &Vector::<i32>::splat(10),
                &edge0010,
                &edge0010_plane_min,
                &edge0010_plane_max,
                &edge1011,
                &edge1011_plane_min,
                &edge1011_plane_max,
                &use_cap_i,
                candidates,
                &mut candidate_count,
                pair_count,
            );
            Self::try_add_interior_point(
                &interior3,
                &Vector::<i32>::splat(11),
                &edge0010,
                &edge0010_plane_min,
                &edge0010_plane_max,
                &edge1011,
                &edge1011_plane_min,
                &edge1011_plane_max,
                &use_cap_i,
                candidates,
                &mut candidate_count,
                pair_count,
            );

            let mut cap_center_to_box_face_center = Vector3Wide::default();
            cap_center_to_box_face_center.x = box_face_center.x;
            cap_center_to_box_face_center.y = box_face_center.y - cap_center_by;
            cap_center_to_box_face_center.z = box_face_center.z;
            let mut tangent_bx = Vector3Wide::default();
            tangent_bx.x = one_f;
            tangent_bx.y = zero_f;
            tangent_bx.z = zero_f;
            let mut tangent_by = Vector3Wide::default();
            tangent_by.x = zero_f;
            tangent_by.y = zero_f;
            tangent_by.z = one_f;

            let mut candidate0 = ManifoldCandidate::default();
            let mut candidate1 = ManifoldCandidate::default();
            let mut candidate2 = ManifoldCandidate::default();
            let mut candidate3 = ManifoldCandidate::default();
            ManifoldCandidateHelper::reduce(
                candidates,
                candidate_count,
                12,
                &box_face_normal,
                inverse_face_normal_dot_local_normal,
                &cap_center_to_box_face_center,
                &tangent_bx,
                &tangent_by,
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

            let mut local_contact = Vector3Wide::default();
            local_contact.x = candidate0.x;
            local_contact.y = cap_center_by;
            local_contact.z = candidate0.y;
            let mut a_to_local_contact = Vector3Wide::default();
            Vector3Wide::add(&local_contact, &local_offset_b, &mut a_to_local_contact);
            Matrix3x3Wide::transform_without_overlap(
                &a_to_local_contact,
                &world_rb,
                &mut manifold.offset_a0,
            );
            local_contact.x = candidate1.x;
            local_contact.z = candidate1.y;
            Vector3Wide::add(&local_contact, &local_offset_b, &mut a_to_local_contact);
            Matrix3x3Wide::transform_without_overlap(
                &a_to_local_contact,
                &world_rb,
                &mut manifold.offset_a1,
            );
            local_contact.x = candidate2.x;
            local_contact.z = candidate2.y;
            Vector3Wide::add(&local_contact, &local_offset_b, &mut a_to_local_contact);
            Matrix3x3Wide::transform_without_overlap(
                &a_to_local_contact,
                &world_rb,
                &mut manifold.offset_a2,
            );
            local_contact.x = candidate3.x;
            local_contact.z = candidate3.y;
            Vector3Wide::add(&local_contact, &local_offset_b, &mut a_to_local_contact);
            Matrix3x3Wide::transform_without_overlap(
                &a_to_local_contact,
                &world_rb,
                &mut manifold.offset_a3,
            );
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
            // Side-face manifold.
            let mut edge_normal_x = Vector3Wide::default();
            Vector3Wide::cross(&box_face_x, &local_normal, &mut edge_normal_x);
            let mut edge_normal_y = Vector3Wide::default();
            Vector3Wide::cross(&box_face_y, &local_normal, &mut edge_normal_y);
            let negative_one = Vector::<f32>::splat(-1.0);
            let x_denominator = negative_one / edge_normal_x.y;
            let y_denominator = negative_one / edge_normal_y.y;
            let edge_normal_x_length_squared = edge_normal_x.length_squared();
            let edge_normal_y_length_squared = edge_normal_y.length_squared();
            let inverse_edge_normal_x_length_squared = one_f / edge_normal_x_length_squared;
            let inverse_edge_normal_y_length_squared = one_f / edge_normal_y_length_squared;

            let mut v00_to_side_line = Vector3Wide::default();
            v00_to_side_line.x = closest_on_b.x - v00.x;
            v00_to_side_line.y = -v00.y;
            v00_to_side_line.z = closest_on_b.z - v00.z;
            let mut v11_to_side_line = Vector3Wide::default();
            v11_to_side_line.x = closest_on_b.x - v11.x;
            v11_to_side_line.y = -v11.y;
            v11_to_side_line.z = closest_on_b.z - v11.z;

            let mut bottom_numerator = Vector::<f32>::splat(0.0);
            let mut left_numerator = Vector::<f32>::splat(0.0);
            let mut top_numerator = Vector::<f32>::splat(0.0);
            let mut right_numerator = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&edge_normal_x, &v00_to_side_line, &mut bottom_numerator);
            Vector3Wide::dot(&edge_normal_y, &v00_to_side_line, &mut left_numerator);
            Vector3Wide::dot(&edge_normal_x, &v11_to_side_line, &mut top_numerator);
            Vector3Wide::dot(&edge_normal_y, &v11_to_side_line, &mut right_numerator);
            let x_invalid = edge_normal_x.y.simd_eq(zero_f);
            let y_invalid = edge_normal_y.y.simd_eq(zero_f);
            let min_value = Vector::<f32>::splat(f32::MIN);
            let max_value = Vector::<f32>::splat(f32::MAX);
            let t_x0 = bottom_numerator * x_denominator;
            let t_x1 = top_numerator * x_denominator;
            let t_y0 = left_numerator * y_denominator;
            let t_y1 = right_numerator * y_denominator;

            let lower_threshold = 0.01f32 * 0.01;
            let upper_threshold = 0.02f32 * 0.02;
            let interpolation_min = Vector::<f32>::splat(upper_threshold);
            let inverse_interpolation_span =
                Vector::<f32>::splat(1.0 / (upper_threshold - lower_threshold));
            let unrestrict_weight_x = ((interpolation_min
                - edge_normal_x.y * edge_normal_x.y * inverse_edge_normal_x_length_squared)
                * inverse_interpolation_span)
                .simd_clamp(zero_f, one_f);
            let unrestrict_weight_y = ((interpolation_min
                - edge_normal_y.y * edge_normal_y.y * inverse_edge_normal_y_length_squared)
                * inverse_interpolation_span)
                .simd_clamp(zero_f, one_f);
            let regular_weight_x = one_f - unrestrict_weight_x;
            let regular_weight_y = one_f - unrestrict_weight_y;
            let negative_half_length = -b.half_length;
            let t_x_min = x_invalid.select(
                min_value,
                unrestrict_weight_x * negative_half_length + regular_weight_x * t_x0.simd_min(t_x1),
            );
            let t_x_max = x_invalid.select(
                max_value,
                unrestrict_weight_x * b.half_length + regular_weight_x * t_x0.simd_max(t_x1),
            );
            let t_y_min = y_invalid.select(
                min_value,
                unrestrict_weight_y * negative_half_length + regular_weight_y * t_y0.simd_min(t_y1),
            );
            let t_y_max = y_invalid.select(
                max_value,
                unrestrict_weight_y * b.half_length + regular_weight_y * t_y0.simd_max(t_y1),
            );
            let t_max = negative_half_length
                .simd_max(t_x_max.simd_min(t_y_max))
                .simd_min(b.half_length);
            let t_min = negative_half_length
                .simd_max(t_x_min.simd_max(t_y_min))
                .simd_min(b.half_length);

            let mut local_contact0 = Vector3Wide::default();
            local_contact0.x = closest_on_b.x;
            local_contact0.y = t_min;
            local_contact0.z = closest_on_b.z;
            let mut local_contact1 = Vector3Wide::default();
            local_contact1.x = closest_on_b.x;
            local_contact1.y = t_max;
            local_contact1.z = closest_on_b.z;

            let mut contact0_world = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &local_contact0,
                &world_rb,
                &mut contact0_world,
            );
            let mut contact1_world = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(
                &local_contact1,
                &world_rb,
                &mut contact1_world,
            );
            let contact0_tmp = contact0_world.clone();
            Vector3Wide::add(&contact0_tmp, offset_b, &mut contact0_world);
            let contact1_tmp = contact1_world.clone();
            Vector3Wide::add(&contact1_tmp, offset_b, &mut contact1_world);
            manifold.offset_a0 =
                Vector3Wide::conditional_select(&use_side_i, &contact0_world, &manifold.offset_a0);
            manifold.offset_a1 =
                Vector3Wide::conditional_select(&use_side_i, &contact1_world, &manifold.offset_a1);
            manifold.feature_id0 = use_side_i
                .simd_ne(zero_i)
                .select(zero_i, manifold.feature_id0);
            manifold.feature_id1 = use_side_i
                .simd_ne(zero_i)
                .select(Vector::<i32>::splat(1), manifold.feature_id1);

            let mut box_face_to_contact0 = Vector3Wide::default();
            let mut box_face_to_contact1 = Vector3Wide::default();
            Vector3Wide::subtract(&local_contact0, &box_face_center, &mut box_face_to_contact0);
            Vector3Wide::subtract(&local_contact1, &box_face_center, &mut box_face_to_contact1);
            let mut contact0_dot = Vector::<f32>::splat(0.0);
            let mut contact1_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&box_face_to_contact0, &box_face_normal, &mut contact0_dot);
            Vector3Wide::dot(&box_face_to_contact1, &box_face_normal, &mut contact1_dot);
            let depth0 = contact0_dot * inverse_face_normal_dot_local_normal;
            let depth1 = contact1_dot * inverse_face_normal_dot_local_normal;
            manifold.depth0 = use_side.select(depth0, manifold.depth0);
            manifold.depth1 = use_side.select(depth1, manifold.depth1);
            manifold.contact0_exists = use_side_i.simd_ne(zero_i).select(
                depth0.simd_ge(depth_threshold).to_int(),
                manifold.contact0_exists,
            );
            manifold.contact1_exists = use_side_i.simd_ne(zero_i).select(
                depth1.simd_ge(depth_threshold).to_int() & t_max.simd_gt(t_min).to_int(),
                manifold.contact1_exists,
            );
            manifold.contact2_exists = use_side_i
                .simd_ne(zero_i)
                .select(zero_i, manifold.contact2_exists);
            manifold.contact3_exists = use_side_i
                .simd_ne(zero_i)
                .select(zero_i, manifold.contact3_exists);
        }

        Matrix3x3Wide::transform_without_overlap(&local_normal, &world_rb, &mut manifold.normal);
    }
}
