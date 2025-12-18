// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CapsuleConvexHullTester.cs

use crate::physics::collidables::capsule::{CapsuleSupportFinder, CapsuleWide};
use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collision_detection::collision_tasks::convex_hull_test_helper::ConvexHullTestHelper;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex2ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::physics::helpers::Helpers;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::prelude::*;

pub struct CapsuleConvexHullTester;

impl CapsuleConvexHullTester {
    pub const BATCH_SIZE: i32 = 16;

    #[inline(always)]
    pub unsafe fn test(
        a: &CapsuleWide,
        b: &ConvexHullWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex2ContactManifoldWide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let zero_i = Vector::<i32>::splat(0);

        let mut capsule_orientation = Matrix3x3Wide::default();
        let mut hull_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut capsule_orientation);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut hull_orientation);
        let mut hull_local_capsule_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(
            &capsule_orientation,
            &hull_orientation,
            &mut hull_local_capsule_orientation,
        );
        let local_capsule_axis = hull_local_capsule_orientation.y.clone();

        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            &hull_orientation,
            &mut local_offset_b,
        );
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);
        let mut center_distance = zero_f;
        Vector3Wide::length_into(&local_offset_a, &mut center_distance);
        let mut initial_normal = Vector3Wide::default();
        Vector3Wide::scale_to(&local_offset_a, &(one_f / center_distance), &mut initial_normal);
        let use_initial_fallback = center_distance.simd_lt(Vector::<f32>::splat(1e-8));
        initial_normal.x = use_initial_fallback.select(zero_f, initial_normal.x);
        initial_normal.y = use_initial_fallback.select(one_f, initial_normal.y);
        initial_normal.z = use_initial_fallback.select(zero_f, initial_normal.z);

        let hull_support_finder = ConvexHullSupportFinder;
        let capsule_support_finder = CapsuleSupportFinder;
        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut hull_epsilon_scale = zero_f;
        b.estimate_epsilon_scale(&inactive_lanes, &mut hull_epsilon_scale);
        let epsilon_scale = a.radius.simd_min(hull_epsilon_scale);
        let depth_threshold = -*speculative_margin;

        let mut depth = zero_f;
        let mut local_normal = Vector3Wide::default();
        let mut closest_on_hull = Vector3Wide::default();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &hull_local_capsule_orientation,
            &hull_support_finder,
            &capsule_support_finder,
            &initial_normal,
            &inactive_lanes,
            &(Vector::<f32>::splat(1e-5) * epsilon_scale),
            &depth_threshold,
            &mut depth,
            &mut local_normal,
            &mut closest_on_hull,
            25,
        );

        inactive_lanes = inactive_lanes | depth.simd_lt(depth_threshold).to_int();
        if inactive_lanes.simd_lt(zero_i).all() {
            *manifold = std::mem::zeroed();
            return;
        }

        // Clip the capsule axis against the representative face of the hull.
        let mut slot_offset_indices = Vector::<i32>::splat(0);
        Helpers::fill_vector_with_lane_indices(&mut slot_offset_indices);

        let mut face_normal_bundle = Vector3Wide::default();
        let bounding_plane_epsilon = Vector::<f32>::splat(1e-3) * epsilon_scale;

        let mut latest_entry_numerator_bundle = Vector::<f32>::splat(f32::MAX);
        let mut latest_entry_denominator_bundle = Vector::<f32>::splat(-1.0);
        let mut earliest_exit_numerator_bundle = Vector::<f32>::splat(f32::MAX);
        let mut earliest_exit_denominator_bundle = one_f;

        for slot_index in 0..(pair_count as usize) {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let hull = &b.hulls[slot_index];
            let mut slot_face_normal = Vec3::ZERO;
            let mut slot_local_normal = Vec3::ZERO;
            let mut best_face_index = 0i32;
            ConvexHullTestHelper::pick_representative_face(
                hull,
                slot_index,
                &local_normal,
                &closest_on_hull,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_face_normal,
                &mut slot_local_normal,
                &mut best_face_index,
            );
            Vector3Wide::write_slot(slot_face_normal, slot_index, &mut face_normal_bundle);

            // Get face vertex indices.
            let mut face_start = 0usize;
            let mut face_count = 0usize;
            hull.get_vertex_indices_for_face(
                best_face_index as usize,
                &mut face_start,
                &mut face_count,
            );

            // Read capsule axis and offset for this slot.
            let mut slot_capsule_axis = Vec3::ZERO;
            let mut slot_local_offset_a = Vec3::ZERO;
            Vector3Wide::read_slot(&local_capsule_axis, slot_index, &mut slot_capsule_axis);
            Vector3Wide::read_slot(&local_offset_a, slot_index, &mut slot_local_offset_a);

            // Read last vertex.
            let last_vi = hull.face_vertex_indices[face_start + face_count - 1];
            let mut previous_vertex = Vec3::ZERO;
            hull.get_point_by_vertex_index(&last_vi, &mut previous_vertex);

            let mut latest_entry_numerator = f32::MAX;
            let mut latest_entry_denominator = -1.0f32;
            let mut earliest_exit_numerator = f32::MAX;
            let mut earliest_exit_denominator = 1.0f32;

            for i in 0..face_count {
                let vi = hull.face_vertex_indices[face_start + i];
                let mut vertex = Vec3::ZERO;
                hull.get_point_by_vertex_index(&vi, &mut vertex);

                let edge_offset = vertex - previous_vertex;
                let edge_plane_normal = edge_offset.cross(slot_local_normal);

                let capsule_to_edge = previous_vertex - slot_local_offset_a;
                let mut numerator = capsule_to_edge.dot(edge_plane_normal);
                let denominator = edge_plane_normal.dot(slot_capsule_axis);
                previous_vertex = vertex;

                let edge_plane_normal_length_squared = edge_plane_normal.length_squared();
                let denominator_squared = denominator * denominator;

                const MIN: f32 = 1e-5;
                const MAX: f32 = 3e-4;
                const INVERSE_SPAN: f32 = 1.0 / (MAX - MIN);
                if denominator_squared > MIN * edge_plane_normal_length_squared {
                    if denominator_squared < MAX * edge_plane_normal_length_squared {
                        let mut restrict_weight =
                            (denominator_squared / edge_plane_normal_length_squared - MIN)
                                * INVERSE_SPAN;
                        if restrict_weight < 0.0 {
                            restrict_weight = 0.0;
                        } else if restrict_weight > 1.0 {
                            restrict_weight = 1.0;
                        }
                        let mut unrestricted_numerator =
                            a.half_length.as_array()[slot_index] * denominator;
                        if denominator < 0.0 {
                            unrestricted_numerator = -unrestricted_numerator;
                        }
                        numerator = restrict_weight * numerator
                            + (1.0 - restrict_weight) * unrestricted_numerator;
                    }
                    if denominator < 0.0 {
                        if numerator * latest_entry_denominator
                            > latest_entry_numerator * denominator
                        {
                            latest_entry_numerator = numerator;
                            latest_entry_denominator = denominator;
                        }
                    } else {
                        if numerator * earliest_exit_denominator
                            < earliest_exit_numerator * denominator
                        {
                            earliest_exit_numerator = numerator;
                            earliest_exit_denominator = denominator;
                        }
                    }
                }
            }

            latest_entry_numerator_bundle.as_mut_array()[slot_index] = latest_entry_numerator;
            latest_entry_denominator_bundle.as_mut_array()[slot_index] = latest_entry_denominator;
            earliest_exit_numerator_bundle.as_mut_array()[slot_index] = earliest_exit_numerator;
            earliest_exit_denominator_bundle.as_mut_array()[slot_index] = earliest_exit_denominator;
        }

        let t_entry = latest_entry_numerator_bundle / latest_entry_denominator_bundle;
        let t_exit = earliest_exit_numerator_bundle / earliest_exit_denominator_bundle;
        let negated_half_length = -a.half_length;
        let t_entry = t_entry.simd_max(negated_half_length).simd_min(a.half_length);
        let t_exit = t_exit.simd_max(negated_half_length).simd_min(a.half_length);

        let local_offset0 = Vector3Wide::scale(&local_capsule_axis, &t_entry);
        let local_offset1 = Vector3Wide::scale(&local_capsule_axis, &t_exit);

        // Compute depth per contact.
        let mut a_to_point_on_hull_face = Vector3Wide::default();
        Vector3Wide::add(&local_offset_b, &closest_on_hull, &mut a_to_point_on_hull_face);

        let mut depth_denominator = zero_f;
        Vector3Wide::dot(&face_normal_bundle, &local_normal, &mut depth_denominator);
        let inverse_depth_denominator = one_f / depth_denominator;
        let mut contact0_to_hull_face = Vector3Wide::default();
        let mut contact1_to_hull_face = Vector3Wide::default();
        Vector3Wide::subtract(
            &a_to_point_on_hull_face,
            &local_offset0,
            &mut contact0_to_hull_face,
        );
        Vector3Wide::subtract(
            &a_to_point_on_hull_face,
            &local_offset1,
            &mut contact1_to_hull_face,
        );
        let mut depth_numerator0 = zero_f;
        let mut depth_numerator1 = zero_f;
        Vector3Wide::dot(
            &contact0_to_hull_face,
            &face_normal_bundle,
            &mut depth_numerator0,
        );
        Vector3Wide::dot(
            &contact1_to_hull_face,
            &face_normal_bundle,
            &mut depth_numerator1,
        );
        let unexpanded_depth0 = depth_numerator0 * inverse_depth_denominator;
        let unexpanded_depth1 = depth_numerator1 * inverse_depth_denominator;
        manifold.depth0 = a.radius + unexpanded_depth0;
        manifold.depth1 = a.radius + unexpanded_depth1;
        manifold.feature_id0 = zero_i;
        manifold.feature_id1 = Vector::<i32>::splat(1);
        // AndNot(a, b) = a & !b
        manifold.contact0_exists =
            manifold.depth0.simd_ge(depth_threshold).to_int() & !inactive_lanes;
        manifold.contact1_exists = (t_exit - t_entry)
            .simd_gt(a.half_length * Vector::<f32>::splat(1e-3))
            .to_int()
            & manifold.depth1.simd_ge(depth_threshold).to_int()
            & !inactive_lanes;

        Matrix3x3Wide::transform_without_overlap(
            &local_offset0,
            &hull_orientation,
            &mut manifold.offset_a0,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_offset1,
            &hull_orientation,
            &mut manifold.offset_a1,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &hull_orientation,
            &mut manifold.normal,
        );
        // Push contacts to hull surface.
        let contact_offset0 = Vector3Wide::scale(&manifold.normal, &unexpanded_depth0);
        let contact_offset1 = Vector3Wide::scale(&manifold.normal, &unexpanded_depth1);
        let offset_a0 = manifold.offset_a0.clone();
        Vector3Wide::add(&offset_a0, &contact_offset0, &mut manifold.offset_a0);
        let offset_a1 = manifold.offset_a1.clone();
        Vector3Wide::add(&offset_a1, &contact_offset1, &mut manifold.offset_a1);
    }
}
