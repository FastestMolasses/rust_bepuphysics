// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CapsuleCylinderTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::cylinder::CylinderWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex2ContactManifoldWide;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for capsule vs cylinder collisions.
pub struct CapsuleCylinderTester;

impl CapsuleCylinderTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Clamps a point on the capsule line to the cylinder bounds and projects back.
    #[inline(always)]
    fn bounce(
        line_origin: &Vector3Wide,
        line_direction: &Vector3Wide,
        t: &Vector<f32>,
        b: &CylinderWide,
        radius_squared: &Vector<f32>,
        p: &mut Vector3Wide,
        clamped: &mut Vector3Wide,
    ) {
        p.x = line_direction.x * *t + line_origin.x;
        p.y = line_direction.y * *t + line_origin.y;
        p.z = line_direction.z * *t + line_origin.z;
        let horizontal_dist_sq = p.x * p.x + p.z * p.z;
        let need_horizontal_clamp = horizontal_dist_sq.simd_gt(*radius_squared);
        let clamp_scale = b.radius / StdFloat::sqrt(horizontal_dist_sq);
        clamped.x = need_horizontal_clamp.select(clamp_scale * p.x, p.x);
        clamped.y = (-b.half_length).simd_max(b.half_length.simd_min(p.y));
        clamped.z = need_horizontal_clamp.select(clamp_scale * p.z, p.z);
    }

    /// Finds the closest point between a line segment and the cylinder surface using iterative projection.
    #[inline(always)]
    pub fn get_closest_point_between_line_segment_and_cylinder(
        line_origin: &Vector3Wide,
        line_direction: &Vector3Wide,
        half_length: &Vector<f32>,
        b: &CylinderWide,
        inactive_lanes: &Vector<i32>,
        t: &mut Vector<f32>,
        offset_from_cylinder_to_line_segment: &mut Vector3Wide,
    ) {
        let min = -*half_length;
        let max = *half_length;
        *t = Vector::<f32>::splat(0.0);
        let radius_squared = b.radius * b.radius;
        let mut origin_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(line_direction, line_origin, &mut origin_dot);
        let epsilon = *half_length * Vector::<f32>::splat(1e-7);
        let mut lane_deactivated = *inactive_lanes;
        let mut min_bound = min;
        let mut max_bound = max;
        let mut p = Vector3Wide::default();
        let mut clamped = Vector3Wide::default();
        for _i in 0..12 {
            Self::bounce(line_origin, line_direction, t, b, &radius_squared, &mut p, &mut clamped);
            let mut conservative_new_t = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&clamped, line_direction, &mut conservative_new_t);
            conservative_new_t = min.simd_max(max.simd_min(conservative_new_t - origin_dot));
            let change = conservative_new_t - *t;
            let lane_should_deactivate = change.abs().simd_lt(epsilon).to_int();
            lane_deactivated = lane_deactivated | lane_should_deactivate;
            if (lane_deactivated).simd_lt(Vector::<i32>::splat(0)).all() {
                break;
            }
            let moved_up = change.simd_gt(Vector::<f32>::splat(0.0));
            min_bound = moved_up.select(conservative_new_t, min_bound);
            max_bound = moved_up.select(max_bound, conservative_new_t);
            let new_t = Vector::<f32>::splat(0.5) * (min_bound + max_bound);
            *t = lane_deactivated.simd_lt(Vector::<i32>::splat(0)).select(*t, new_t);
        }
        Self::bounce(line_origin, line_direction, t, b, &radius_squared, &mut p, &mut clamped);
        Vector3Wide::subtract(&p, &clamped, offset_from_cylinder_to_line_segment);
    }

    /// Computes closest points between two line segments, exploiting cylinder local space where db = (0,1,0).
    #[inline(always)]
    pub fn get_closest_points_between_segments(
        da: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        a_half_length: &Vector<f32>,
        b_half_length: &Vector<f32>,
        ta: &mut Vector<f32>,
        ta_min: &mut Vector<f32>,
        ta_max: &mut Vector<f32>,
        tb: &mut Vector<f32>,
        tb_min: &mut Vector<f32>,
        tb_max: &mut Vector<f32>,
    ) {
        let mut da_offset_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(da, local_offset_b, &mut da_offset_b);
        let db_offset_b = local_offset_b.y;
        let dadb = da.y;
        *ta = (da_offset_b - db_offset_b * dadb) / (Vector::<f32>::splat(1.0) - dadb * dadb).simd_max(Vector::<f32>::splat(1e-15));
        *tb = *ta * dadb - db_offset_b;

        let absdadb = dadb.abs();
        let b_onto_a_offset = *b_half_length * absdadb;
        let a_onto_b_offset = *a_half_length * absdadb;
        *ta_min = (-*a_half_length).simd_max((*a_half_length).simd_min(da_offset_b - b_onto_a_offset));
        *ta_max = (*a_half_length).simd_min((-*a_half_length).simd_max(da_offset_b + b_onto_a_offset));
        *tb_min = (-*b_half_length).simd_max((*b_half_length).simd_min(-a_onto_b_offset - db_offset_b));
        *tb_max = (*b_half_length).simd_min((-*b_half_length).simd_max(a_onto_b_offset - db_offset_b));
        *ta = (*ta).simd_max(*ta_min).simd_min(*ta_max);
        *tb = (*tb).simd_max(*tb_min).simd_min(*tb_max);
    }

    /// Computes the contact interval between the capsule segment and a cylinder side segment.
    #[inline(always)]
    pub fn get_contact_interval_between_segments(
        a_half_length: &Vector<f32>,
        b_half_length: &Vector<f32>,
        axis_a: &Vector3Wide,
        local_normal: &Vector3Wide,
        inverse_horizontal_normal_length_squared_b: &Vector<f32>,
        offset_b: &Vector3Wide,
        contact_t_min: &mut Vector<f32>,
        contact_t_max: &mut Vector<f32>,
    ) {
        let mut ta = Vector::<f32>::splat(0.0);
        let mut ta_min = Vector::<f32>::splat(0.0);
        let mut ta_max = Vector::<f32>::splat(0.0);
        let mut tb = Vector::<f32>::splat(0.0);
        let mut tb_min = Vector::<f32>::splat(0.0);
        let mut tb_max = Vector::<f32>::splat(0.0);
        Self::get_closest_points_between_segments(axis_a, offset_b, a_half_length, b_half_length, &mut ta, &mut ta_min, &mut ta_max, &mut tb, &mut tb_min, &mut tb_max);

        // Rate coplanarity based on the angle between the capsule axis and the plane defined by the side and contact normal.
        // Since db is (0,1,0), db x normal is just (normal.Z, 0, -normal.X).
        let dot_val = axis_a.x * local_normal.z - axis_a.z * local_normal.x;
        let squared_angle = dot_val * dot_val * *inverse_horizontal_normal_length_squared_b;

        const LOWER_THRESHOLD: f32 = 0.02 * 0.02;
        const UPPER_THRESHOLD: f32 = 0.15 * 0.15;
        let interval_weight = Vector::<f32>::splat(0.0).simd_max(
            Vector::<f32>::splat(1.0).simd_min(
                (Vector::<f32>::splat(UPPER_THRESHOLD) - squared_angle)
                    * Vector::<f32>::splat(1.0 / (UPPER_THRESHOLD - LOWER_THRESHOLD)),
            ),
        );
        let weighted_tb = tb - tb * interval_weight;
        *contact_t_min = interval_weight * tb_min + weighted_tb;
        *contact_t_max = interval_weight * tb_max + weighted_tb;
    }

    /// Tests capsule vs cylinder collision (two orientations).
    #[inline(always)]
    pub fn test(
        a: &CapsuleWide,
        b: &CylinderWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex2ContactManifoldWide,
    ) {
        let mut world_r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut world_r_a);
        let mut world_r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut world_r_b);
        // Work in cylinder's local space.
        let mut r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(&world_r_a, &world_r_b, &mut r_a);
        let capsule_axis = r_a.y;
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &world_r_b, &mut local_offset_b);
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

        let inactive_lanes = BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut t = Vector::<f32>::splat(0.0);
        let mut local_normal = Vector3Wide::default();
        Self::get_closest_point_between_line_segment_and_cylinder(&local_offset_a, &capsule_axis, &a.half_length, b, &inactive_lanes, &mut t, &mut local_normal);
        let mut dist_from_cyl_to_seg_sq = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&local_normal, &mut dist_from_cyl_to_seg_sq);
        let internal_line_segment_intersected = dist_from_cyl_to_seg_sq.simd_lt(Vector::<f32>::splat(1e-12));
        let dist_from_cyl_to_seg = StdFloat::sqrt(dist_from_cyl_to_seg_sq);
        local_normal = Vector3Wide::scale(&local_normal, &(Vector::<f32>::splat(1.0) / dist_from_cyl_to_seg));
        let mut depth = internal_line_segment_intersected.select(Vector::<f32>::splat(f32::MAX), -dist_from_cyl_to_seg);
        let negative_margin = -*speculative_margin;
        let mut inactive_lanes = (depth + a.radius).simd_lt(negative_margin).to_int() | inactive_lanes;

        if (internal_line_segment_intersected.to_int() & !inactive_lanes).simd_lt(Vector::<i32>::splat(0)).any() {
            // At least one lane is deeply intersecting; examine other normals.
            let mut capsule_axis_dot_y = Vector::<f32>::splat(0.0);
            let _ = capsule_axis.y; // db = (0,1,0)
            let endpoint_vs_cap_depth = b.half_length + (capsule_axis.y * a.half_length).abs() - local_offset_a.y.abs();
            let use_endpoint_cap_depth = internal_line_segment_intersected.to_int() & endpoint_vs_cap_depth.simd_lt(depth).to_int();
            depth = use_endpoint_cap_depth.simd_lt(Vector::<i32>::splat(0)).select(endpoint_vs_cap_depth, depth);
            local_normal.x = use_endpoint_cap_depth.simd_lt(Vector::<i32>::splat(0)).select(Vector::<f32>::splat(0.0), local_normal.x);
            local_normal.y = use_endpoint_cap_depth.simd_lt(Vector::<i32>::splat(0)).select(
                local_offset_a.y.simd_gt(Vector::<f32>::splat(0.0)).select(Vector::<f32>::splat(1.0), Vector::<f32>::splat(-1.0)),
                local_normal.y,
            );
            local_normal.z = use_endpoint_cap_depth.simd_lt(Vector::<i32>::splat(0)).select(Vector::<f32>::splat(0.0), local_normal.z);

            let mut ta = Vector::<f32>::splat(0.0);
            let mut ta_min = Vector::<f32>::splat(0.0);
            let mut ta_max = Vector::<f32>::splat(0.0);
            let mut tb = Vector::<f32>::splat(0.0);
            let mut tb_min = Vector::<f32>::splat(0.0);
            let mut tb_max = Vector::<f32>::splat(0.0);
            Self::get_closest_points_between_segments(&capsule_axis, &local_offset_b, &a.half_length, &b.half_length, &mut ta, &mut ta_min, &mut ta_max, &mut tb, &mut tb_min, &mut tb_max);

            // offset = da * ta - (db * tb + offsetB); db = (0,1,0)
            let mut closest_a = Vector3Wide::default();
            Vector3Wide::scale_to(&capsule_axis, &ta, &mut closest_a);
            let mut offset = Vector3Wide::default();
            Vector3Wide::subtract(&closest_a, &local_offset_b, &mut offset);
            offset.y = offset.y - tb;

            let distance = offset.length();
            let inverse_distance = Vector::<f32>::splat(1.0) / distance;
            let mut internal_edge_normal = Vector3Wide::scale(&offset, &inverse_distance);
            let use_fallback = distance.simd_lt(Vector::<f32>::splat(1e-7));
            internal_edge_normal.x = use_fallback.select(Vector::<f32>::splat(1.0), internal_edge_normal.x);
            internal_edge_normal.y = use_fallback.select(Vector::<f32>::splat(0.0), internal_edge_normal.y);
            internal_edge_normal.z = use_fallback.select(Vector::<f32>::splat(0.0), internal_edge_normal.z);

            let mut center_separation_along_normal = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_offset_a, &internal_edge_normal, &mut center_separation_along_normal);
            let cylinder_contribution = (b.half_length * internal_edge_normal.y).abs()
                + b.radius * StdFloat::sqrt((Vector::<f32>::splat(1.0) - internal_edge_normal.y * internal_edge_normal.y).simd_max(Vector::<f32>::splat(0.0)));
            let mut capsule_axis_dot_normal = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&capsule_axis, &internal_edge_normal, &mut capsule_axis_dot_normal);
            let capsule_contribution = capsule_axis_dot_normal.abs() * a.half_length;
            let internal_edge_depth = cylinder_contribution + capsule_contribution - center_separation_along_normal;

            let use_internal_edge_depth = internal_line_segment_intersected.to_int() & internal_edge_depth.simd_lt(depth).to_int();
            depth = use_internal_edge_depth.simd_lt(Vector::<i32>::splat(0)).select(internal_edge_depth, depth);
            local_normal = Vector3Wide::conditional_select(&use_internal_edge_depth, &internal_edge_normal, &local_normal);
        }

        // All of the above excluded capsule radius. Include it now.
        depth = depth + a.radius;
        inactive_lanes = depth.simd_lt(negative_margin).to_int() | inactive_lanes;
        if inactive_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
            *manifold = unsafe { std::mem::zeroed() };
            return;
        }

        // Determine which contacts to produce: cap or side.
        let use_cap_contacts = (!inactive_lanes) & local_normal.y.abs().simd_gt(Vector::<f32>::splat(0.70710678118)).to_int();

        // First, assume non-cap (side) contacts.
        let inverse_horizontal_normal_length_squared = Vector::<f32>::splat(1.0) / (local_normal.x * local_normal.x + local_normal.z * local_normal.z);
        let scale_val = b.radius * StdFloat::sqrt(inverse_horizontal_normal_length_squared);
        let cylinder_segment_offset_x = local_normal.x * scale_val;
        let cylinder_segment_offset_z = local_normal.z * scale_val;
        let a_to_side_segment_center = Vector3Wide {
            x: local_offset_b.x + cylinder_segment_offset_x,
            y: local_offset_b.y,
            z: local_offset_b.z + cylinder_segment_offset_z,
        };
        let mut contact_t_min = Vector::<f32>::splat(0.0);
        let mut contact_t_max = Vector::<f32>::splat(0.0);
        Self::get_contact_interval_between_segments(&a.half_length, &b.half_length, &capsule_axis, &local_normal, &inverse_horizontal_normal_length_squared, &a_to_side_segment_center, &mut contact_t_min, &mut contact_t_max);

        let mut contact0 = Vector3Wide {
            x: cylinder_segment_offset_x,
            y: contact_t_min,
            z: cylinder_segment_offset_z,
        };
        let mut contact1 = Vector3Wide {
            x: cylinder_segment_offset_x,
            y: contact_t_max,
            z: cylinder_segment_offset_z,
        };

        let mut contact_count = (contact_t_max - contact_t_min).abs().simd_lt(b.half_length * Vector::<f32>::splat(1e-5)).select(Vector::<i32>::splat(1), Vector::<i32>::splat(2));

        if use_cap_contacts.simd_lt(Vector::<i32>::splat(0)).any() {
            // At least one lane requires cap contacts.
            let cap_height = local_normal.y.simd_gt(Vector::<f32>::splat(0.0)).select(b.half_length, -b.half_length);
            let inverse_normal_y = Vector::<f32>::splat(1.0) / local_normal.y;
            let mut endpoint_offset = Vector3Wide::default();
            Vector3Wide::scale_to(&capsule_axis, &a.half_length, &mut endpoint_offset);
            let positive = Vector3Wide {
                x: local_offset_a.x + endpoint_offset.x,
                y: local_offset_a.y + endpoint_offset.y - cap_height,
                z: local_offset_a.z + endpoint_offset.z,
            };
            let negative = Vector3Wide {
                x: local_offset_a.x - endpoint_offset.x,
                y: local_offset_a.y - endpoint_offset.y - cap_height,
                z: local_offset_a.z - endpoint_offset.z,
            };
            let t_negative = negative.y * inverse_normal_y;
            let t_positive = positive.y * inverse_normal_y;
            let projected_negative = Vector2Wide {
                x: negative.x - local_normal.x * t_negative,
                y: negative.z - local_normal.z * t_negative,
            };
            let projected_positive = Vector2Wide {
                x: positive.x - local_normal.x * t_positive,
                y: positive.z - local_normal.z * t_positive,
            };
            // Intersect the projected line segment with the cap circle at origin with radius b.radius.
            let mut projected_offset = Vector2Wide::default();
            Vector2Wide::subtract(&projected_positive, &projected_negative, &mut projected_offset);
            let mut coefficient_c = Vector::<f32>::splat(0.0);
            Vector2Wide::dot(&projected_negative, &projected_negative, &mut coefficient_c);
            coefficient_c = coefficient_c - b.radius * b.radius;
            let mut coefficient_b = Vector::<f32>::splat(0.0);
            Vector2Wide::dot(&projected_negative, &projected_offset, &mut coefficient_b);
            let mut coefficient_a = Vector::<f32>::splat(0.0);
            Vector2Wide::dot(&projected_offset, &projected_offset, &mut coefficient_a);
            let inverse_a = Vector::<f32>::splat(1.0) / coefficient_a;
            let t_offset_val = StdFloat::sqrt((coefficient_b * coefficient_b - coefficient_a * coefficient_c).simd_max(Vector::<f32>::splat(0.0))) * inverse_a;
            let t_base = -coefficient_b * inverse_a;
            let mut t_min_cap = (t_base - t_offset_val).simd_max(Vector::<f32>::splat(0.0)).simd_min(Vector::<f32>::splat(1.0));
            let mut t_max_cap = (t_base + t_offset_val).simd_max(Vector::<f32>::splat(0.0)).simd_min(Vector::<f32>::splat(1.0));
            let use_fallback_cap = coefficient_a.abs().simd_lt(Vector::<f32>::splat(1e-12));
            t_min_cap = use_fallback_cap.select(Vector::<f32>::splat(0.0), t_min_cap);
            t_max_cap = use_fallback_cap.select(Vector::<f32>::splat(0.0), t_max_cap);
            let cap_contact0 = Vector3Wide {
                x: t_min_cap * projected_offset.x + projected_negative.x,
                y: cap_height,
                z: t_min_cap * projected_offset.y + projected_negative.y,
            };
            let cap_contact1 = Vector3Wide {
                x: t_max_cap * projected_offset.x + projected_negative.x,
                y: cap_height,
                z: t_max_cap * projected_offset.y + projected_negative.y,
            };
            let cap_contact_count = (t_max_cap - t_min_cap).simd_gt(Vector::<f32>::splat(1e-5)).select(Vector::<i32>::splat(2), Vector::<i32>::splat(1));
            contact_count = use_cap_contacts.simd_lt(Vector::<i32>::splat(0)).select(cap_contact_count, contact_count);
            contact0 = Vector3Wide::conditional_select(&use_cap_contacts, &cap_contact0, &contact0);
            contact1 = Vector3Wide::conditional_select(&use_cap_contacts, &cap_contact1, &contact1);
        }

        // Per-contact depth. Project contact on B along normal to face of A.
        let mut capsule_tangent = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&local_normal, &capsule_axis, &mut capsule_tangent) };
        let mut face_normal_a = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&capsule_tangent, &capsule_axis, &mut face_normal_a) };
        let mut face_normal_a_dot_local_normal = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&face_normal_a, &local_normal, &mut face_normal_a_dot_local_normal);
        let inverse_face_normal_a_dot_local_normal = Vector::<f32>::splat(1.0) / face_normal_a_dot_local_normal;
        let offset0 = contact0 + local_offset_b;
        let offset1 = contact1 + local_offset_b;
        let mut t0 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset0, &face_normal_a, &mut t0);
        let mut t1 = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&offset1, &face_normal_a, &mut t1);
        t0 = t0 * inverse_face_normal_a_dot_local_normal;
        t1 = t1 * inverse_face_normal_a_dot_local_normal;
        manifold.depth0 = a.radius + t0;
        manifold.depth1 = a.radius + t1;

        let collapse = face_normal_a_dot_local_normal.abs().simd_lt(Vector::<f32>::splat(1e-7));
        manifold.depth0 = collapse.select(depth, manifold.depth0);
        let contact0_valid = manifold.depth0.simd_ge(negative_margin);
        let contact1_valid = contact_count.simd_eq(Vector::<i32>::splat(2)).to_int() & (!collapse.to_int()) & manifold.depth1.simd_ge(negative_margin).to_int();
        manifold.contact0_exists = (!inactive_lanes) & contact0_valid.to_int();
        manifold.contact1_exists = (!inactive_lanes) & contact1_valid;

        // Transform to world space.
        Matrix3x3Wide::transform_without_overlap(&local_normal, &world_r_b, &mut manifold.normal);
        Matrix3x3Wide::transform_without_overlap(&contact0, &world_r_b, &mut manifold.offset_a0);
        Matrix3x3Wide::transform_without_overlap(&contact1, &world_r_b, &mut manifold.offset_a1);
        manifold.offset_a0 = manifold.offset_a0 + *offset_b;
        manifold.offset_a1 = manifold.offset_a1 + *offset_b;

        manifold.feature_id0 = Vector::<i32>::splat(0);
        manifold.feature_id1 = Vector::<i32>::splat(1);
    }
}
