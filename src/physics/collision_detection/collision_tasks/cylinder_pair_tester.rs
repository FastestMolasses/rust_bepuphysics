// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CylinderPairTester.cs

use crate::physics::collidables::cylinder::{CylinderSupportFinder, CylinderWide};
use crate::physics::collision_detection::collision_tasks::capsule_cylinder_tester::CapsuleCylinderTester;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for cylinder vs cylinder collisions.
pub struct CylinderPairTester;

impl CylinderPairTester {
    pub const BATCH_SIZE: i32 = 16;

    #[inline(always)]
    fn project_onto_cap_a_2d(
        cap_center_by: &Vector<f32>,
        cap_center_a: &Vector3Wide,
        r_a: &Matrix3x3Wide,
        inverse_n_dot_ay: &Vector<f32>,
        local_normal: &Vector3Wide,
        point: &Vector2Wide,
        projected: &mut Vector2Wide,
    ) {
        let mut point3d = Vector3Wide::default();
        point3d.x = point.x;
        point3d.y = *cap_center_by;
        point3d.z = point.y;
        Self::project_onto_cap_a(cap_center_a, r_a, inverse_n_dot_ay, local_normal, &point3d, projected);
    }

    #[inline(always)]
    fn project_onto_cap_a(
        cap_center_a: &Vector3Wide,
        r_a: &Matrix3x3Wide,
        inverse_n_dot_ay: &Vector<f32>,
        local_normal: &Vector3Wide,
        point: &Vector3Wide,
        projected: &mut Vector2Wide,
    ) {
        let mut point_to_cap_center_a = Vector3Wide::default();
        Vector3Wide::subtract(cap_center_a, point, &mut point_to_cap_center_a);
        let mut t_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&point_to_cap_center_a, &r_a.y, &mut t_distance);
        let t_b_on_a = t_distance * *inverse_n_dot_ay;
        let mut projection_offset_b = Vector3Wide::default();
        Vector3Wide::scale_to(local_normal, &t_b_on_a, &mut projection_offset_b);
        let mut projected_point = Vector3Wide::default();
        Vector3Wide::add(point, &projection_offset_b, &mut projected_point);
        let mut cap_center_a_to_projected_point = Vector3Wide::default();
        Vector3Wide::subtract(&projected_point, cap_center_a, &mut cap_center_a_to_projected_point);
        Vector3Wide::dot(&cap_center_a_to_projected_point, &r_a.x, &mut projected.x);
        Vector3Wide::dot(&cap_center_a_to_projected_point, &r_a.z, &mut projected.y);
    }

    #[inline(always)]
    pub fn project_onto_cap_b(
        cap_center_by: &Vector<f32>,
        inverse_local_normal_y: &Vector<f32>,
        local_normal: &Vector3Wide,
        point: &Vector3Wide,
        projected: &mut Vector2Wide,
    ) {
        let t_a_on_b = (point.y - *cap_center_by) * *inverse_local_normal_y;
        projected.x = point.x - local_normal.x * t_a_on_b;
        projected.y = point.z - local_normal.z * t_a_on_b;
    }

    #[inline(always)]
    fn intersect_line_circle(
        line_position: &Vector2Wide,
        line_direction: &Vector2Wide,
        radius: &Vector<f32>,
        t_min: &mut Vector<f32>,
        t_max: &mut Vector<f32>,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let mut a_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_direction, line_direction, &mut a_val);
        let inverse_a = one_f / a_val;
        let mut b_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_position, line_direction, &mut b_val);
        let mut c_val = Vector::<f32>::splat(0.0);
        Vector2Wide::dot(line_position, line_position, &mut c_val);
        let radius_squared = *radius * *radius;
        c_val = c_val - radius_squared;
        let t_offset = StdFloat::sqrt((b_val * b_val - a_val * c_val).simd_max(zero_f)) * inverse_a;
        let t_base = -b_val * inverse_a;
        *t_min = t_base - t_offset;
        *t_max = t_base + t_offset;
        // If the projected line direction is zero, just compress the interval to tBase.
        let use_fallback = a_val.abs().simd_lt(Vector::<f32>::splat(1e-12));
        *t_min = use_fallback.select(t_base, *t_min);
        *t_max = use_fallback.select(t_base, *t_max);
    }

    #[inline(always)]
    fn from_cap_b_to_3d(
        contact: &Vector2Wide,
        cap_center_by: &Vector<f32>,
        local_contact_3d: &mut Vector3Wide,
    ) {
        local_contact_3d.x = contact.x;
        local_contact_3d.y = *cap_center_by;
        local_contact_3d.z = contact.y;
    }

    #[inline(always)]
    pub fn transform_contact(
        contact: &Vector3Wide,
        local_feature_position_a: &Vector3Wide,
        local_feature_normal_a: &Vector3Wide,
        inverse_feature_normal_a_dot_local_normal: Vector<f32>,
        local_offset_b: &Vector3Wide,
        orientation_b: &Matrix3x3Wide,
        negative_speculative_margin: Vector<f32>,
        a_to_contact: &mut Vector3Wide,
        depth: &mut Vector<f32>,
        contact_exists: &mut Vector<i32>,
    ) {
        let mut feature_offset = Vector3Wide::default();
        Vector3Wide::subtract(contact, local_feature_position_a, &mut feature_offset);
        let mut t_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&feature_offset, local_feature_normal_a, &mut t_distance);
        *depth = t_distance * inverse_feature_normal_a_dot_local_normal;
        let mut local_a_to_contact = Vector3Wide::default();
        Vector3Wide::add(contact, local_offset_b, &mut local_a_to_contact);
        Matrix3x3Wide::transform_without_overlap(&local_a_to_contact, orientation_b, a_to_contact);
        *contact_exists = *contact_exists & depth.simd_ge(negative_speculative_margin).to_int();
    }

    /// Tests cylinder vs cylinder collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &CylinderWide,
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
        // Work in b's local space.
        let mut r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_ra, &world_rb, &mut r_a);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &world_rb, &mut local_offset_b);
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
        let support_finder = CylinderSupportFinder;

        let mut inactive_lanes = BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let depth_threshold = -*speculative_margin;
        let epsilon_scale =
            a.half_length.simd_max(a.radius).simd_min(b.half_length.simd_max(b.radius));

        let mut depth = Vector::<f32>::splat(0.0);
        let mut closest_on_b = Vector3Wide::default();
        let initial_normal = local_normal.clone();
        DepthRefiner::find_minimum_depth_with_witness(
            b, a, &local_offset_a, &r_a,
            &support_finder, &support_finder,
            &initial_normal, &inactive_lanes,
            &(epsilon_scale * Vector::<f32>::splat(1e-6)),
            &depth_threshold,
            &mut depth, &mut local_normal, &mut closest_on_b, 25,
        );

        inactive_lanes = inactive_lanes | depth.simd_lt(depth_threshold).to_int();
        if inactive_lanes.simd_lt(zero_i).all() {
            *manifold = unsafe { std::mem::zeroed() };
            return;
        }

        // Determine dominant feature pairs along the collision normal.
        let mut n_dot_ay = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&r_a.y, &local_normal, &mut n_dot_ay);
        let inverse_n_dot_ay = one_f / n_dot_ay;
        let inverse_local_normal_y = one_f / local_normal.y;
        let cap_a_offset = n_dot_ay
            .simd_gt(zero_f)
            .select(-a.half_length, a.half_length);
        let mut cap_center_a = Vector3Wide::default();
        Vector3Wide::scale_to(&r_a.y, &cap_a_offset, &mut cap_center_a);
        let cap_center_a_tmp = cap_center_a.clone();
        Vector3Wide::add(&cap_center_a_tmp, &local_offset_a, &mut cap_center_a);
        let cap_center_by = local_normal.y
            .simd_lt(zero_f)
            .select(-b.half_length, b.half_length);

        let cap_threshold = Vector::<f32>::splat(0.70710678118);
        let use_cap_a = n_dot_ay.abs().simd_gt(cap_threshold);
        let use_cap_b = local_normal.y.abs().simd_gt(cap_threshold);
        let mut contact0 = Vector3Wide::default();
        let mut contact1 = Vector3Wide::default();
        let mut contact2 = Vector3Wide::default();
        let mut contact3 = Vector3Wide::default();
        manifold.contact0_exists = zero_i;
        manifold.contact1_exists = zero_i;
        manifold.contact2_exists = zero_i;
        manifold.contact3_exists = zero_i;
        let use_cap_cap = (use_cap_a & use_cap_b).to_int() & !inactive_lanes;

        // Extreme points along the contact normal.
        let mut b_to_a_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&local_normal, &(-depth), &mut b_to_a_offset);
        let mut extreme_a = Vector3Wide::default();
        Vector3Wide::add(&closest_on_b, &b_to_a_offset, &mut extreme_a);
        let mut extreme_a_horizontal_offset = Vector3Wide::default();
        Vector3Wide::subtract(&extreme_a, &local_offset_a, &mut extreme_a_horizontal_offset);
        let mut vertical_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&extreme_a_horizontal_offset, &r_a.y, &mut vertical_dot);
        let mut to_remove = Vector3Wide::default();
        Vector3Wide::scale_to(&r_a.y, &vertical_dot, &mut to_remove);
        let temp_horiz = extreme_a_horizontal_offset.clone();
        Vector3Wide::subtract(&temp_horiz, &to_remove, &mut extreme_a_horizontal_offset);

        let extreme_b = Vector2Wide {
            x: closest_on_b.x,
            y: closest_on_b.z,
        };

        let use_negative = n_dot_ay.simd_gt(zero_f);
        let mut cap_feature_normal_a = Vector3Wide::default();
        cap_feature_normal_a.x = use_negative.select(-r_a.y.x, r_a.y.x);
        cap_feature_normal_a.y = use_negative.select(-r_a.y.y, r_a.y.y);
        cap_feature_normal_a.z = use_negative.select(-r_a.y.z, r_a.y.z);
        let mut feature_normal_a = cap_feature_normal_a.clone();
        let mut feature_position_a = cap_center_a.clone();

        if use_cap_cap.simd_lt(zero_i).any() {
            // Cap-cap contact generation.
            let parallel_threshold = Vector::<f32>::splat(0.9999);
            let inverse_parallel_interpolation_span = Vector::<f32>::splat(1.0 / (0.99995 - 0.9999));
            let abs_a_dot = n_dot_ay.abs();
            let abs_b_dot = local_normal.y.abs();
            let a_cap_not_parallel = abs_a_dot.simd_lt(parallel_threshold);
            let b_cap_not_parallel = abs_b_dot.simd_lt(parallel_threshold);

            let mut cap_contact0 = extreme_b.clone();

            let both_not_parallel = (a_cap_not_parallel & b_cap_not_parallel).to_int();
            if ((!both_not_parallel) & !inactive_lanes).simd_lt(zero_i).any() {
                // At least one cap is parallel; build 4-contact manifold.
                let mut cap_center_a_on_b = Vector2Wide::default();
                Self::project_onto_cap_b(
                    &cap_center_by, &inverse_local_normal_y, &local_normal,
                    &cap_center_a, &mut cap_center_a_on_b,
                );
                let mut horizontal_offset_length = Vector::<f32>::splat(0.0);
                Vector2Wide::length(&cap_center_a_on_b, &mut horizontal_offset_length);
                let inverse_horiz_offset_length = one_f / horizontal_offset_length;
                let mut horizontal_offset_direction = Vector2Wide::default();
                Vector2Wide::scale(&cap_center_a_on_b, &inverse_horiz_offset_length, &mut horizontal_offset_direction);
                let use_both_parallel_fallback = horizontal_offset_length.simd_lt(Vector::<f32>::splat(1e-14));
                horizontal_offset_direction.x = use_both_parallel_fallback.select(one_f, horizontal_offset_direction.x);
                horizontal_offset_direction.y = use_both_parallel_fallback.select(zero_f, horizontal_offset_direction.y);
                let mut initial_line_start = Vector2Wide::default();
                Vector2Wide::scale(&horizontal_offset_direction, &b.radius, &mut initial_line_start);

                let mut contact1_line_endpoint = Vector2Wide::default();
                Vector2Wide::negate(&initial_line_start, &mut contact1_line_endpoint);

                let mut line_start_on_a = Vector2Wide::default();
                Self::project_onto_cap_a_2d(
                    &cap_center_by, &cap_center_a, &r_a,
                    &inverse_n_dot_ay, &local_normal,
                    &initial_line_start, &mut line_start_on_a,
                );
                let mut line_end_on_a = Vector2Wide::default();
                Self::project_onto_cap_a_2d(
                    &cap_center_by, &cap_center_a, &r_a,
                    &inverse_n_dot_ay, &local_normal,
                    &contact1_line_endpoint, &mut line_end_on_a,
                );
                let mut line_direction_on_a = Vector2Wide::default();
                Vector2Wide::subtract(&line_end_on_a, &line_start_on_a, &mut line_direction_on_a);
                let mut contact1_line_direction_on_b = Vector2Wide::default();
                Vector2Wide::subtract(&contact1_line_endpoint, &initial_line_start, &mut contact1_line_direction_on_b);
                let mut contact1_t_min_a = Vector::<f32>::splat(0.0);
                let mut contact1_t_max_a = Vector::<f32>::splat(0.0);
                Self::intersect_line_circle(
                    &line_start_on_a, &line_direction_on_a, &a.radius,
                    &mut contact1_t_min_a, &mut contact1_t_max_a,
                );
                let first_line_t_min = contact1_t_min_a.simd_max(zero_f);
                let first_line_t_max = contact1_t_max_a.simd_min(one_f);
                let mut scaled_dir = Vector2Wide::default();
                Vector2Wide::scale(&contact1_line_direction_on_b, &first_line_t_min, &mut scaled_dir);
                Vector2Wide::add(&initial_line_start, &scaled_dir, &mut cap_contact0);
                let mut cap_contact1 = Vector2Wide::default();
                Vector2Wide::scale(&contact1_line_direction_on_b, &first_line_t_max, &mut scaled_dir);
                Vector2Wide::add(&initial_line_start, &scaled_dir, &mut cap_contact1);

                let half_f = Vector::<f32>::splat(0.5);
                let circle_intersection_t = half_f
                    * (horizontal_offset_length
                        + (b.radius * b.radius - a.radius * a.radius) * inverse_horiz_offset_length);
                let second_line_start_t = horizontal_offset_length
                    .simd_min(circle_intersection_t.simd_max(zero_f));
                let mut second_line_start_on_b = Vector2Wide::default();
                Vector2Wide::scale(&horizontal_offset_direction, &second_line_start_t, &mut second_line_start_on_b);
                let second_line_direction_on_b = Vector2Wide {
                    x: horizontal_offset_direction.y,
                    y: -horizontal_offset_direction.x,
                };

                let mut second_line_end_on_b = Vector2Wide::default();
                Vector2Wide::add(&second_line_start_on_b, &second_line_direction_on_b, &mut second_line_end_on_b);
                let mut second_line_start_on_a = Vector2Wide::default();
                Self::project_onto_cap_a_2d(
                    &cap_center_by, &cap_center_a, &r_a,
                    &inverse_n_dot_ay, &local_normal,
                    &second_line_start_on_b, &mut second_line_start_on_a,
                );
                let mut second_line_end_on_a = Vector2Wide::default();
                Self::project_onto_cap_a_2d(
                    &cap_center_by, &cap_center_a, &r_a,
                    &inverse_n_dot_ay, &local_normal,
                    &second_line_end_on_b, &mut second_line_end_on_a,
                );
                let mut second_line_direction_on_a = Vector2Wide::default();
                Vector2Wide::subtract(&second_line_end_on_a, &second_line_start_on_a, &mut second_line_direction_on_a);

                let mut second_line_t_min_a = Vector::<f32>::splat(0.0);
                let mut second_line_t_max_a = Vector::<f32>::splat(0.0);
                Self::intersect_line_circle(
                    &second_line_start_on_a, &second_line_direction_on_a, &a.radius,
                    &mut second_line_t_min_a, &mut second_line_t_max_a,
                );
                let mut second_line_t_min_b = Vector::<f32>::splat(0.0);
                let mut second_line_t_max_b = Vector::<f32>::splat(0.0);
                Self::intersect_line_circle(
                    &second_line_start_on_b, &second_line_direction_on_b, &b.radius,
                    &mut second_line_t_min_b, &mut second_line_t_max_b,
                );
                let second_line_t_min = second_line_t_min_a.simd_max(second_line_t_min_b);
                let second_line_t_max = second_line_t_max_a.simd_min(second_line_t_max_b);

                let mut cap_contact2 = Vector2Wide::default();
                let mut cap_contact3 = Vector2Wide::default();
                Vector2Wide::scale(&second_line_direction_on_b, &second_line_t_min, &mut scaled_dir);
                Vector2Wide::add(&second_line_start_on_b, &scaled_dir, &mut cap_contact2);
                Vector2Wide::scale(&second_line_direction_on_b, &second_line_t_max, &mut scaled_dir);
                Vector2Wide::add(&second_line_start_on_b, &scaled_dir, &mut cap_contact3);

                // Blend in extreme point for non-parallel cases:
                let weight_a_parallel = ((abs_a_dot - parallel_threshold)
                    * inverse_parallel_interpolation_span)
                    .simd_clamp(zero_f, one_f);
                let weight_b_parallel = ((abs_b_dot - parallel_threshold)
                    * inverse_parallel_interpolation_span)
                    .simd_clamp(zero_f, one_f);
                let parallel_weight = weight_a_parallel * weight_b_parallel;
                let extreme_weight = one_f - parallel_weight;

                let mut manifold_center_to_extreme_b = Vector2Wide::default();
                Vector2Wide::subtract(&extreme_b, &second_line_start_on_b, &mut manifold_center_to_extreme_b);
                let replace_dot0 = horizontal_offset_direction.x * manifold_center_to_extreme_b.x
                    + horizontal_offset_direction.y * manifold_center_to_extreme_b.y;
                let replace_dot2 = second_line_direction_on_b.x * manifold_center_to_extreme_b.x
                    + second_line_direction_on_b.y * manifold_center_to_extreme_b.y;
                let replace_0_or_1 = replace_dot0.abs().simd_gt(replace_dot2.abs());
                let replace_0 = (both_not_parallel.simd_ne(zero_i))
                    | (replace_dot0.simd_gt(zero_f) & replace_0_or_1);
                let replace_1 = replace_dot0.simd_le(zero_f) & replace_0_or_1;
                let replace_2 = !replace_0_or_1 & replace_dot2.simd_lt(zero_f);
                let replace_3 = !replace_0_or_1 & replace_dot2.simd_ge(zero_f);

                cap_contact0.x = replace_0.select(
                    extreme_b.x * extreme_weight + cap_contact0.x * parallel_weight,
                    cap_contact0.x,
                );
                cap_contact0.y = replace_0.select(
                    extreme_b.y * extreme_weight + cap_contact0.y * parallel_weight,
                    cap_contact0.y,
                );
                cap_contact1.x = replace_1.select(
                    extreme_b.x * extreme_weight + cap_contact1.x * parallel_weight,
                    cap_contact1.x,
                );
                cap_contact1.y = replace_1.select(
                    extreme_b.y * extreme_weight + cap_contact1.y * parallel_weight,
                    cap_contact1.y,
                );
                cap_contact2.x = replace_2.select(
                    extreme_b.x * extreme_weight + cap_contact2.x * parallel_weight,
                    cap_contact2.x,
                );
                cap_contact2.y = replace_2.select(
                    extreme_b.y * extreme_weight + cap_contact2.y * parallel_weight,
                    cap_contact2.y,
                );
                cap_contact3.x = replace_3.select(
                    extreme_b.x * extreme_weight + cap_contact3.x * parallel_weight,
                    cap_contact3.x,
                );
                cap_contact3.y = replace_3.select(
                    extreme_b.y * extreme_weight + cap_contact3.y * parallel_weight,
                    cap_contact3.y,
                );

                Self::from_cap_b_to_3d(&cap_contact1, &cap_center_by, &mut contact1);
                Self::from_cap_b_to_3d(&cap_contact2, &cap_center_by, &mut contact2);
                Self::from_cap_b_to_3d(&cap_contact3, &cap_center_by, &mut contact3);
                manifold.contact1_exists =
                    (use_cap_cap & first_line_t_max.simd_gt(first_line_t_min).to_int()) & !both_not_parallel;
                manifold.contact2_exists = manifold.contact1_exists;
                manifold.contact3_exists = manifold.contact1_exists
                    & second_line_t_max.simd_gt(second_line_t_min).to_int();
            }
            Self::from_cap_b_to_3d(&cap_contact0, &cap_center_by, &mut contact0);
            manifold.contact0_exists = use_cap_cap;
        }

        let use_cap_side = ((use_cap_a & !use_cap_b) | (use_cap_b & !use_cap_a)).to_int() & !inactive_lanes;
        // Side normal computation (shared for cap-side and side-side).
        let mut ax = Vector::<f32>::splat(0.0);
        let mut az = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&r_a.x, &local_normal, &mut ax);
        Vector3Wide::dot(&r_a.z, &local_normal, &mut az);
        let horizontal_normal_length_a = StdFloat::sqrt(ax * ax + az * az);
        let inverse_horizontal_normal_length_a = one_f / horizontal_normal_length_a;
        let x_scale = ax * inverse_horizontal_normal_length_a;
        let z_scale = az * inverse_horizontal_normal_length_a;
        let side_feature_normal_ax = Vector3Wide::scale(&r_a.x, &x_scale);
        let side_feature_normal_az = Vector3Wide::scale(&r_a.z, &z_scale);
        let mut side_feature_normal_a = Vector3Wide::default();
        Vector3Wide::add(&side_feature_normal_ax, &side_feature_normal_az, &mut side_feature_normal_a);
        let mut side_center_a = Vector3Wide::default();
        side_center_a.x = extreme_a_horizontal_offset.x + local_offset_a.x;
        side_center_a.y = extreme_a_horizontal_offset.y + local_offset_a.y;
        side_center_a.z = extreme_a_horizontal_offset.z + local_offset_a.z;
        let mut side_center_b = Vector3Wide::default();
        side_center_b.x = extreme_b.x;
        side_center_b.y = zero_f;
        side_center_b.z = extreme_b.y;

        if use_cap_side.simd_lt(zero_i).any() {
            // Cap-side contact generation.
            let mut side_line_end_a = Vector3Wide::default();
            Vector3Wide::add(&side_center_a, &r_a.y, &mut side_line_end_a);
            let mut side_line_end_b = Vector3Wide::default();
            side_line_end_b.x = side_center_b.x;
            side_line_end_b.y = one_f;
            side_line_end_b.z = side_center_b.z;
            let mut projected_line_start_b_on_a = Vector2Wide::default();
            Self::project_onto_cap_a(
                &cap_center_a, &r_a, &inverse_n_dot_ay, &local_normal,
                &side_center_b, &mut projected_line_start_b_on_a,
            );
            let mut projected_line_start_a_on_b = Vector2Wide::default();
            Self::project_onto_cap_b(
                &cap_center_by, &inverse_local_normal_y, &local_normal,
                &side_center_a, &mut projected_line_start_a_on_b,
            );
            let mut projected_line_end_b_on_a = Vector2Wide::default();
            Self::project_onto_cap_a(
                &cap_center_a, &r_a, &inverse_n_dot_ay, &local_normal,
                &side_line_end_b, &mut projected_line_end_b_on_a,
            );
            let mut projected_line_end_a_on_b = Vector2Wide::default();
            Self::project_onto_cap_b(
                &cap_center_by, &inverse_local_normal_y, &local_normal,
                &side_line_end_a, &mut projected_line_end_a_on_b,
            );

            let use_cap_a_i = use_cap_a.to_int();
            let mut projected_line_start = Vector2Wide::default();
            Vector2Wide::conditional_select(
                &use_cap_a_i, &projected_line_start_b_on_a, &projected_line_start_a_on_b,
                &mut projected_line_start,
            );
            let mut projected_line_end = Vector2Wide::default();
            Vector2Wide::conditional_select(
                &use_cap_a_i, &projected_line_end_b_on_a, &projected_line_end_a_on_b,
                &mut projected_line_end,
            );
            let radius = use_cap_a.select(a.radius, b.radius);
            let side_half_length = use_cap_a.select(b.half_length, a.half_length);
            let mut projected_line_direction = Vector2Wide::default();
            Vector2Wide::subtract(&projected_line_end, &projected_line_start, &mut projected_line_direction);
            let mut t_min = Vector::<f32>::splat(0.0);
            let mut t_max = Vector::<f32>::splat(0.0);
            Self::intersect_line_circle(
                &projected_line_start, &projected_line_direction, &radius,
                &mut t_min, &mut t_max,
            );
            t_min = (-side_half_length).simd_max(t_min).simd_min(side_half_length);
            t_max = t_max.simd_min(side_half_length);

            // Build contacts for both cases.
            let mut contact0_for_cap_a = Vector3Wide::default();
            contact0_for_cap_a.x = side_center_b.x;
            contact0_for_cap_a.y = t_min;
            contact0_for_cap_a.z = side_center_b.z;
            let mut contact1_for_cap_a = Vector3Wide::default();
            contact1_for_cap_a.x = side_center_b.x;
            contact1_for_cap_a.y = t_max;
            contact1_for_cap_a.z = side_center_b.z;

            let mut contact0_for_cap_b = Vector3Wide::default();
            contact0_for_cap_b.x = projected_line_start.x + t_min * projected_line_direction.x;
            contact0_for_cap_b.y = cap_center_by;
            contact0_for_cap_b.z = projected_line_start.y + t_min * projected_line_direction.y;
            let mut contact1_for_cap_b = Vector3Wide::default();
            contact1_for_cap_b.x = projected_line_start.x + t_max * projected_line_direction.x;
            contact1_for_cap_b.y = cap_center_by;
            contact1_for_cap_b.z = projected_line_start.y + t_max * projected_line_direction.y;

            let cap_side_contact0 = Vector3Wide::conditional_select(
                &use_cap_a_i, &contact0_for_cap_a, &contact0_for_cap_b,
            );
            let cap_side_contact1 = Vector3Wide::conditional_select(
                &use_cap_a_i, &contact1_for_cap_a, &contact1_for_cap_b,
            );
            contact0 = Vector3Wide::conditional_select(&use_cap_side, &cap_side_contact0, &contact0);
            contact1 = Vector3Wide::conditional_select(&use_cap_side, &cap_side_contact1, &contact1);
            manifold.contact0_exists = use_cap_side.simd_ne(zero_i).select(neg_one, manifold.contact0_exists);
            manifold.contact1_exists = use_cap_side.simd_ne(zero_i).select(
                t_max.simd_gt(t_min).to_int(),
                manifold.contact1_exists,
            );

            let cap_side_feature_normal_a = Vector3Wide::conditional_select(
                &use_cap_a_i, &cap_feature_normal_a, &side_feature_normal_a,
            );
            feature_normal_a = Vector3Wide::conditional_select(
                &use_cap_side, &cap_side_feature_normal_a, &feature_normal_a,
            );

            let cap_side_feature_position_a = Vector3Wide::conditional_select(
                &use_cap_a_i, &cap_center_a, &side_center_a,
            );
            feature_position_a = Vector3Wide::conditional_select(
                &use_cap_side, &cap_side_feature_position_a, &feature_position_a,
            );
        }

        let use_side_side = ((!use_cap_a) & (!use_cap_b)).to_int() & !inactive_lanes;
        if use_side_side.simd_lt(zero_i).any() {
            // Side-side contact generation.
            let mut side_center_a_neg = Vector3Wide::default();
            Vector3Wide::negate(&side_center_a, &mut side_center_a_neg);
            let horizontal_normal_length_squared_b =
                local_normal.x * local_normal.x + local_normal.z * local_normal.z;
            let inverse_horizontal_normal_length_squared_b = one_f / horizontal_normal_length_squared_b;
            let mut contact_t_min = Vector::<f32>::splat(0.0);
            let mut contact_t_max = Vector::<f32>::splat(0.0);
            CapsuleCylinderTester::get_contact_interval_between_segments(
                &a.half_length, &b.half_length,
                &r_a.y, &local_normal,
                &inverse_horizontal_normal_length_squared_b,
                &side_center_a_neg,
                &mut contact_t_min, &mut contact_t_max,
            );

            contact0.x = use_side_side.simd_ne(zero_i).select(extreme_b.x, contact0.x);
            contact0.y = use_side_side.simd_ne(zero_i).select(contact_t_min, contact0.y);
            contact0.z = use_side_side.simd_ne(zero_i).select(extreme_b.y, contact0.z);
            contact1.x = use_side_side.simd_ne(zero_i).select(extreme_b.x, contact1.x);
            contact1.y = use_side_side.simd_ne(zero_i).select(contact_t_max, contact1.y);
            contact1.z = use_side_side.simd_ne(zero_i).select(extreme_b.y, contact1.z);
            manifold.contact0_exists = use_side_side.simd_ne(zero_i).select(neg_one, manifold.contact0_exists);
            manifold.contact1_exists = use_side_side.simd_ne(zero_i).select(
                contact_t_max.simd_gt(contact_t_min).to_int(),
                manifold.contact1_exists,
            );
            feature_normal_a = Vector3Wide::conditional_select(
                &use_side_side, &side_feature_normal_a, &feature_normal_a,
            );
            feature_position_a = Vector3Wide::conditional_select(
                &use_side_side, &side_center_a, &feature_position_a,
            );
        }

        let mut feature_normal_a_dot_local_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&feature_normal_a, &local_normal, &mut feature_normal_a_dot_local_normal);
        let inverse_feature_normal_a_dot_local_normal = one_f / feature_normal_a_dot_local_normal;
        let negative_speculative_margin = -*speculative_margin;
        Self::transform_contact(
            &contact0, &feature_position_a, &feature_normal_a,
            inverse_feature_normal_a_dot_local_normal,
            &local_offset_b, &world_rb, negative_speculative_margin,
            &mut manifold.offset_a0, &mut manifold.depth0, &mut manifold.contact0_exists,
        );
        Self::transform_contact(
            &contact1, &feature_position_a, &feature_normal_a,
            inverse_feature_normal_a_dot_local_normal,
            &local_offset_b, &world_rb, negative_speculative_margin,
            &mut manifold.offset_a1, &mut manifold.depth1, &mut manifold.contact1_exists,
        );
        Self::transform_contact(
            &contact2, &feature_position_a, &feature_normal_a,
            inverse_feature_normal_a_dot_local_normal,
            &local_offset_b, &world_rb, negative_speculative_margin,
            &mut manifold.offset_a2, &mut manifold.depth2, &mut manifold.contact2_exists,
        );
        Self::transform_contact(
            &contact3, &feature_position_a, &feature_normal_a,
            inverse_feature_normal_a_dot_local_normal,
            &local_offset_b, &world_rb, negative_speculative_margin,
            &mut manifold.offset_a3, &mut manifold.depth3, &mut manifold.contact3_exists,
        );
        Matrix3x3Wide::transform_without_overlap(&local_normal, &world_rb, &mut manifold.normal);
        manifold.feature_id0 = zero_i;
        manifold.feature_id1 = Vector::<i32>::splat(1);
        manifold.feature_id2 = Vector::<i32>::splat(2);
        manifold.feature_id3 = Vector::<i32>::splat(3);
    }
}
