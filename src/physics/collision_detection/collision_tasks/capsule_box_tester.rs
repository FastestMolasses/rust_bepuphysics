// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CapsuleBoxTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex2ContactManifoldWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for capsule vs box collisions.
pub struct CapsuleBoxTester;

impl CapsuleBoxTester {
    pub const BATCH_SIZE: i32 = 32;

    #[inline(always)]
    pub fn prepare(
        a: &CapsuleWide,
        b: &BoxWide,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        local_offset_a: &mut Vector3Wide,
        capsule_axis: &mut Vector3Wide,
        edge_centers: &mut Vector3Wide,
    ) {
        let to_local_b = QuaternionWide::conjugate(orientation_b);
        let mut transformed = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(offset_b, &to_local_b, &mut transformed);
        Vector3Wide::negate(&transformed, local_offset_a);
        let mut box_local_orientation_a = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            orientation_a,
            &to_local_b,
            &mut box_local_orientation_a,
        );
        *capsule_axis = QuaternionWide::transform_unit_y(box_local_orientation_a);

        // Get the closest point on the capsule segment to the box center.
        let mut dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(local_offset_a, capsule_axis, &mut dot);
        let clamped_dot = (-a.half_length).simd_max(a.half_length.simd_min(dot));
        let mut offset_to_capsule = Vector3Wide::default();
        Vector3Wide::scale_to(capsule_axis, &clamped_dot, &mut offset_to_capsule);
        let mut diff = Vector3Wide::default();
        Vector3Wide::subtract(local_offset_a, &offset_to_capsule, &mut diff);
        let zero = Vector::<f32>::splat(0.0);
        edge_centers.x = diff.x.simd_lt(zero).select(-b.half_width, b.half_width);
        edge_centers.y = diff.y.simd_lt(zero).select(-b.half_height, b.half_height);
        edge_centers.z = diff.z.simd_lt(zero).select(-b.half_length, b.half_length);
    }

    /// Tests a box edge (hardcoded Z direction, caller swizzles).
    #[inline(always)]
    pub fn test_box_edge(
        offset_ax: &Vector<f32>,
        offset_ay: &Vector<f32>,
        offset_az: &Vector<f32>,
        capsule_axis_x: &Vector<f32>,
        capsule_axis_y: &Vector<f32>,
        capsule_axis_z: &Vector<f32>,
        capsule_half_length: &Vector<f32>,
        box_edge_center_x: &Vector<f32>,
        box_edge_center_y: &Vector<f32>,
        box_half_width: &Vector<f32>,
        box_half_height: &Vector<f32>,
        box_half_length: &Vector<f32>,
        ta_min: &mut Vector<f32>,
        ta_max: &mut Vector<f32>,
        closest_point_on_a: &mut Vector3Wide,
        n_x: &mut Vector<f32>,
        n_y: &mut Vector<f32>,
        n_z: &mut Vector<f32>,
        ta: &mut Vector<f32>,
        epsilon: &mut Vector<f32>,
    ) {
        let ab_x = *box_edge_center_x - *offset_ax;
        let ab_y = *box_edge_center_y - *offset_ay;
        let da_offset_b =
            *capsule_axis_x * ab_x + *capsule_axis_y * ab_y - *capsule_axis_z * *offset_az;
        *ta = (da_offset_b + *offset_az * *capsule_axis_z)
            / Vector::<f32>::splat(1e-15)
                .simd_max(Vector::<f32>::splat(1.0) - *capsule_axis_z * *capsule_axis_z);
        let mut tb = *ta * *capsule_axis_z + *offset_az;

        let absdadb = capsule_axis_z.abs();
        let b_onto_a_offset = *box_half_length * absdadb;
        let a_onto_b_offset = *capsule_half_length * absdadb;
        *ta_min = (-*capsule_half_length)
            .simd_max((da_offset_b - b_onto_a_offset).simd_min(*capsule_half_length));
        *ta_max = capsule_half_length
            .simd_min((da_offset_b + b_onto_a_offset).simd_max(-*capsule_half_length));
        let b_min =
            (-*box_half_length).simd_max((*offset_az - a_onto_b_offset).simd_min(*box_half_length));
        let b_max =
            box_half_length.simd_min((*offset_az + a_onto_b_offset).simd_max(-*box_half_length));
        *ta = ta.simd_max(*ta_min).simd_min(*ta_max);
        tb = tb.simd_max(b_min).simd_min(b_max);

        closest_point_on_a.x = *ta * *capsule_axis_x + *offset_ax;
        closest_point_on_a.y = *ta * *capsule_axis_y + *offset_ay;
        closest_point_on_a.z = *ta * *capsule_axis_z + *offset_az;
        *n_x = closest_point_on_a.x - *box_edge_center_x;
        *n_y = closest_point_on_a.y - *box_edge_center_y;
        *n_z = closest_point_on_a.z - tb;
        let mut squared_length = *n_x * *n_x + *n_y * *n_y + *n_z * *n_z;
        let fallback_sq_len = *capsule_axis_y * *capsule_axis_y + *capsule_axis_x * *capsule_axis_x;
        *epsilon = Vector::<f32>::splat(1e-10);
        let use_fallback = squared_length.simd_lt(*epsilon);
        let use_second_fallback = use_fallback & fallback_sq_len.simd_lt(*epsilon);
        squared_length = use_second_fallback.select(
            Vector::<f32>::splat(1.0),
            use_fallback.select(fallback_sq_len, squared_length),
        );
        *n_x = use_second_fallback.select(
            Vector::<f32>::splat(1.0),
            use_fallback.select(-*capsule_axis_y, *n_x),
        );
        *n_y = use_second_fallback.select(
            Vector::<f32>::splat(0.0),
            use_fallback.select(*capsule_axis_x, *n_y),
        );
        *n_z = use_second_fallback.select(
            Vector::<f32>::splat(0.0),
            use_fallback.select(Vector::<f32>::splat(0.0), *n_z),
        );

        // Calibrate normal to point from B to A.
        let calibration_dot = *n_x * *offset_ax + *n_y * *offset_ay + *n_z * *offset_az;
        let should_negate = calibration_dot.simd_lt(Vector::<f32>::splat(0.0));
        *n_x = should_negate.select(-*n_x, *n_x);
        *n_y = should_negate.select(-*n_y, *n_y);
        *n_z = should_negate.select(-*n_z, *n_z);

        let inverse_length = Vector::<f32>::splat(1.0) / StdFloat::sqrt(squared_length);
        *n_x *= inverse_length;
        *n_y *= inverse_length;
        *n_z *= inverse_length;
    }

    #[inline(always)]
    fn test_and_refine_box_edge(
        offset_ax: &Vector<f32>,
        offset_ay: &Vector<f32>,
        offset_az: &Vector<f32>,
        capsule_axis_x: &Vector<f32>,
        capsule_axis_y: &Vector<f32>,
        capsule_axis_z: &Vector<f32>,
        capsule_half_length: &Vector<f32>,
        box_edge_center_x: &Vector<f32>,
        box_edge_center_y: &Vector<f32>,
        box_half_width: &Vector<f32>,
        box_half_height: &Vector<f32>,
        box_half_length: &Vector<f32>,
        ta: &mut Vector<f32>,
        depth: &mut Vector<f32>,
        n_x: &mut Vector<f32>,
        n_y: &mut Vector<f32>,
        n_z: &mut Vector<f32>,
    ) {
        let mut ta_min = Vector::<f32>::splat(0.0);
        let mut ta_max = Vector::<f32>::splat(0.0);
        let mut closest_point_on_a = Vector3Wide::default();
        let mut epsilon = Vector::<f32>::splat(0.0);
        Self::test_box_edge(
            offset_ax,
            offset_ay,
            offset_az,
            capsule_axis_x,
            capsule_axis_y,
            capsule_axis_z,
            capsule_half_length,
            box_edge_center_x,
            box_edge_center_y,
            box_half_width,
            box_half_height,
            box_half_length,
            &mut ta_min,
            &mut ta_max,
            &mut closest_point_on_a,
            n_x,
            n_y,
            n_z,
            ta,
            &mut epsilon,
        );

        let box_extreme = n_x.abs() * *box_half_width
            + n_y.abs() * *box_half_height
            + n_z.abs() * *box_half_length;
        let capsule_extreme =
            *n_x * closest_point_on_a.x + *n_y * closest_point_on_a.y + *n_z * closest_point_on_a.z;
        *depth = box_extreme - capsule_extreme;
    }

    #[inline(always)]
    fn test_box_face(
        offset_az: &Vector<f32>,
        capsule_axis_z: &Vector<f32>,
        capsule_half_length: &Vector<f32>,
        box_half_length: &Vector<f32>,
        depth: &mut Vector<f32>,
        normal_sign: &mut Vector<f32>,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        *normal_sign = offset_az
            .simd_gt(zero)
            .select(Vector::<f32>::splat(1.0), Vector::<f32>::splat(-1.0));
        *depth = *box_half_length + capsule_axis_z.abs() * *capsule_half_length
            - *normal_sign * *offset_az;
    }

    #[inline(always)]
    fn select_with_ta(
        depth: &mut Vector<f32>,
        ta: &mut Vector<f32>,
        nx: &mut Vector<f32>,
        ny: &mut Vector<f32>,
        nz: &mut Vector<f32>,
        depth_candidate: &Vector<f32>,
        ta_candidate: &Vector<f32>,
        nx_candidate: &Vector<f32>,
        ny_candidate: &Vector<f32>,
        nz_candidate: &Vector<f32>,
    ) {
        let use_candidate = depth_candidate.simd_lt(*depth);
        *ta = use_candidate.select(*ta_candidate, *ta);
        *depth = use_candidate.select(*depth_candidate, *depth);
        *nx = use_candidate.select(*nx_candidate, *nx);
        *ny = use_candidate.select(*ny_candidate, *ny);
        *nz = use_candidate.select(*nz_candidate, *nz);
    }

    #[inline(always)]
    fn select_no_ta(
        depth: &mut Vector<f32>,
        nx: &mut Vector<f32>,
        ny: &mut Vector<f32>,
        nz: &mut Vector<f32>,
        depth_candidate: &Vector<f32>,
        nx_candidate: &Vector<f32>,
        ny_candidate: &Vector<f32>,
        nz_candidate: &Vector<f32>,
    ) {
        let use_candidate = depth_candidate.simd_lt(*depth);
        *depth = use_candidate.select(*depth_candidate, *depth);
        *nx = use_candidate.select(*nx_candidate, *nx);
        *ny = use_candidate.select(*ny_candidate, *ny);
        *nz = use_candidate.select(*nz_candidate, *nz);
    }

    /// Tests capsule vs box collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &CapsuleWide,
        b: &BoxWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _pair_count: i32,
        manifold: &mut Convex2ContactManifoldWide,
    ) {
        let mut local_offset_a = Vector3Wide::default();
        let mut capsule_axis = Vector3Wide::default();
        let mut edge_centers = Vector3Wide::default();
        Self::prepare(
            a,
            b,
            offset_b,
            orientation_a,
            orientation_b,
            &mut local_offset_a,
            &mut capsule_axis,
            &mut edge_centers,
        );

        // Test edges with swizzle pattern
        // Swizzle XYZ -> YZX
        let (mut ta, mut depth, mut nx, mut ny, mut nz);
        {
            let (mut ta_e, mut d_e, mut ny_e, mut nz_e, mut nx_e) = (
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
            );
            Self::test_and_refine_box_edge(
                &local_offset_a.y,
                &local_offset_a.z,
                &local_offset_a.x,
                &capsule_axis.y,
                &capsule_axis.z,
                &capsule_axis.x,
                &a.half_length,
                &edge_centers.y,
                &edge_centers.z,
                &b.half_height,
                &b.half_length,
                &b.half_width,
                &mut ta_e,
                &mut d_e,
                &mut ny_e,
                &mut nz_e,
                &mut nx_e,
            );
            ta = ta_e;
            depth = d_e;
            nx = nx_e;
            ny = ny_e;
            nz = nz_e;
        }
        // Swizzle XYZ -> ZXY
        {
            let (mut ta_e, mut d_e, mut nz_e, mut nx_e, mut ny_e) = (
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
            );
            Self::test_and_refine_box_edge(
                &local_offset_a.z,
                &local_offset_a.x,
                &local_offset_a.y,
                &capsule_axis.z,
                &capsule_axis.x,
                &capsule_axis.y,
                &a.half_length,
                &edge_centers.z,
                &edge_centers.x,
                &b.half_length,
                &b.half_width,
                &b.half_height,
                &mut ta_e,
                &mut d_e,
                &mut nz_e,
                &mut nx_e,
                &mut ny_e,
            );
            Self::select_with_ta(
                &mut depth, &mut ta, &mut nx, &mut ny, &mut nz, &d_e, &ta_e, &nx_e, &ny_e, &nz_e,
            );
        }
        // Swizzle XYZ -> XYZ
        {
            let (mut ta_e, mut d_e, mut nx_e, mut ny_e, mut nz_e) = (
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
                Vector::<f32>::splat(0.0),
            );
            Self::test_and_refine_box_edge(
                &local_offset_a.x,
                &local_offset_a.y,
                &local_offset_a.z,
                &capsule_axis.x,
                &capsule_axis.y,
                &capsule_axis.z,
                &a.half_length,
                &edge_centers.x,
                &edge_centers.y,
                &b.half_width,
                &b.half_height,
                &b.half_length,
                &mut ta_e,
                &mut d_e,
                &mut nx_e,
                &mut ny_e,
                &mut nz_e,
            );
            Self::select_with_ta(
                &mut depth, &mut ta, &mut nx, &mut ny, &mut nz, &d_e, &ta_e, &nx_e, &ny_e, &nz_e,
            );
        }

        let zero = Vector::<f32>::splat(0.0);
        // Face X
        let (mut fx_depth, mut fxn) = (Vector::<f32>::splat(0.0), Vector::<f32>::splat(0.0));
        Self::test_box_face(
            &local_offset_a.x,
            &capsule_axis.x,
            &a.half_length,
            &b.half_width,
            &mut fx_depth,
            &mut fxn,
        );
        Self::select_no_ta(
            &mut depth, &mut nx, &mut ny, &mut nz, &fx_depth, &fxn, &zero, &zero,
        );
        // Face Y
        let (mut fy_depth, mut fyn) = (Vector::<f32>::splat(0.0), Vector::<f32>::splat(0.0));
        Self::test_box_face(
            &local_offset_a.y,
            &capsule_axis.y,
            &a.half_length,
            &b.half_height,
            &mut fy_depth,
            &mut fyn,
        );
        Self::select_no_ta(
            &mut depth, &mut nx, &mut ny, &mut nz, &fy_depth, &zero, &fyn, &zero,
        );
        // Face Z
        let (mut fz_depth, mut fzn) = (Vector::<f32>::splat(0.0), Vector::<f32>::splat(0.0));
        Self::test_box_face(
            &local_offset_a.z,
            &capsule_axis.z,
            &a.half_length,
            &b.half_length,
            &mut fz_depth,
            &mut fzn,
        );
        Self::select_no_ta(
            &mut depth, &mut nx, &mut ny, &mut nz, &fz_depth, &zero, &zero, &fzn,
        );

        // Choose representative box face and compute contact interval.
        let x_dot = nx * fxn;
        let y_dot = ny * fyn;
        let z_dot = nz * fzn;
        let use_x = x_dot.simd_gt(y_dot.simd_max(z_dot));
        let use_y = y_dot.simd_gt(z_dot) & !use_x;
        let use_z = !use_x & !use_y;

        let face_normal_dot_local_normal = use_x.select(x_dot, use_y.select(y_dot, z_dot));
        let inverse_fndln = Vector::<f32>::splat(1.0)
            / face_normal_dot_local_normal.simd_max(Vector::<f32>::splat(1e-15));
        let capsule_axis_dot_fn = use_x.select(
            capsule_axis.x * fxn,
            use_y.select(capsule_axis.y * fyn, capsule_axis.z * fzn),
        );
        let capsule_center_dot_fn = use_x.select(
            local_offset_a.x * fxn,
            use_y.select(local_offset_a.y * fyn, local_offset_a.z * fzn),
        );
        let face_plane_offset =
            use_x.select(b.half_width, use_y.select(b.half_height, b.half_length));
        let t_axis = capsule_axis_dot_fn * inverse_fndln;
        let t_center = (capsule_center_dot_fn - face_plane_offset) * inverse_fndln;

        let local_normal = Vector3Wide {
            x: nx,
            y: ny,
            z: nz,
        };
        let mut axis_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&local_normal, &t_axis, &mut axis_offset);
        let mut center_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&local_normal, &t_center, &mut center_offset);
        let mut unprojected_axis = Vector3Wide::default();
        Vector3Wide::subtract(&capsule_axis, &axis_offset, &mut unprojected_axis);
        let mut unprojected_center = Vector3Wide::default();
        Vector3Wide::subtract(&local_offset_a, &center_offset, &mut unprojected_center);

        let ts_axis_x = use_x.select(unprojected_axis.y, unprojected_axis.x);
        let ts_axis_y = use_z.select(unprojected_axis.y, unprojected_axis.z);
        let ts_center_x = use_x.select(unprojected_center.y, unprojected_center.x);
        let ts_center_y = use_z.select(unprojected_center.y, unprojected_center.z);

        let epsilon_scale = b
            .half_width
            .simd_max(b.half_height.simd_max(b.half_length))
            .simd_min(a.half_length.simd_max(a.radius));
        let epsilon = epsilon_scale * Vector::<f32>::splat(1e-3);
        let half_extent_x = epsilon + use_x.select(b.half_height, b.half_width);
        let half_extent_y = epsilon + use_z.select(b.half_height, b.half_length);

        let neg_one = Vector::<f32>::splat(-1.0);
        let inv_axis_x = neg_one / ts_axis_x;
        let inv_axis_y = neg_one / ts_axis_y;
        let tx0 = (ts_center_x - half_extent_x) * inv_axis_x;
        let tx1 = (ts_center_x + half_extent_x) * inv_axis_x;
        let ty0 = (ts_center_y - half_extent_y) * inv_axis_y;
        let ty1 = (ts_center_y + half_extent_y) * inv_axis_y;
        let mut min_x = tx0.simd_min(tx1);
        let mut max_x = tx0.simd_max(tx1);
        let mut min_y = ty0.simd_min(ty1);
        let mut max_y = ty0.simd_max(ty1);

        let use_fallback_x = ts_axis_x.abs().simd_lt(Vector::<f32>::splat(1e-15));
        let use_fallback_y = ts_axis_y.abs().simd_lt(Vector::<f32>::splat(1e-15));
        let center_contained_x = ts_center_x.abs().simd_le(half_extent_x);
        let center_contained_y = ts_center_y.abs().simd_le(half_extent_y);
        let large_neg = Vector::<f32>::splat(-f32::MAX);
        let large_pos = Vector::<f32>::splat(f32::MAX);
        min_x = use_fallback_x.select(center_contained_x.select(large_neg, large_pos), min_x);
        max_x = use_fallback_x.select(center_contained_x.select(large_pos, large_neg), max_x);
        min_y = use_fallback_y.select(center_contained_y.select(large_neg, large_pos), min_y);
        max_y = use_fallback_y.select(center_contained_y.select(large_pos, large_neg), max_y);

        let face_min = min_x.simd_max(min_y);
        let face_max = max_x.simd_min(max_y);
        let t_min_raw = a.half_length.simd_min(face_min).simd_max(-a.half_length);
        let t_max_raw = a.half_length.simd_min(face_max).simd_max(-a.half_length);
        let face_interval_exists = face_max.simd_ge(face_min);
        let t_min = face_interval_exists.select(t_min_raw.simd_min(ta), ta);
        let t_max = face_interval_exists.select(t_max_raw.simd_max(ta), ta);

        let separation_min = t_center + t_axis * t_min;
        let separation_max = t_center + t_axis * t_max;
        manifold.depth0 = a.radius - separation_min;
        manifold.depth1 = a.radius - separation_max;

        let mut local_a0 = Vector3Wide::default();
        Vector3Wide::scale_to(&capsule_axis, &t_min, &mut local_a0);
        let mut local_a1 = Vector3Wide::default();
        Vector3Wide::scale_to(&capsule_axis, &t_max, &mut local_a1);

        manifold.feature_id0 = Vector::<i32>::splat(0);
        manifold.feature_id1 = Vector::<i32>::splat(1);

        // Transform to world space.
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &orientation_matrix_b,
            &mut manifold.normal,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_a0,
            &orientation_matrix_b,
            &mut manifold.offset_a0,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_a1,
            &orientation_matrix_b,
            &mut manifold.offset_a1,
        );

        // Apply normal offset to contact positions.
        let neg_offset0 = manifold.depth0 * Vector::<f32>::splat(0.5) - a.radius;
        let neg_offset1 = manifold.depth1 * Vector::<f32>::splat(0.5) - a.radius;
        let mut push0 = Vector3Wide::default();
        Vector3Wide::scale_to(&manifold.normal, &neg_offset0, &mut push0);
        let mut push1 = Vector3Wide::default();
        Vector3Wide::scale_to(&manifold.normal, &neg_offset1, &mut push1);
        manifold.offset_a0 = manifold.offset_a0 + push0;
        manifold.offset_a1 = manifold.offset_a1 + push1;

        let min_accepted_depth = -*speculative_margin;
        manifold.contact0_exists = manifold.depth0.simd_ge(min_accepted_depth).to_int();
        manifold.contact1_exists = manifold.depth1.simd_ge(min_accepted_depth).to_int()
            & (t_max - t_min)
                .simd_gt(Vector::<f32>::splat(1e-7) * a.half_length)
                .to_int();
    }
}
