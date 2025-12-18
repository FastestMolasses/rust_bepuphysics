// Translated from BepuPhysics/CollisionDetection/CollisionTasks/BoxConvexHullTester.cs

use crate::physics::collidables::box_shape::{BoxSupportFinder, BoxWide};
use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collision_detection::collision_tasks::convex_hull_test_helper::ConvexHullTestHelper;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidateHelper, ManifoldCandidateScalar,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::physics::helpers::Helpers;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::prelude::*;

/// Pair tester for box vs convex hull collisions.
pub struct BoxConvexHullTester;

impl BoxConvexHullTester {
    pub const BATCH_SIZE: i32 = 16;

    /// Tests box vs convex hull collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &BoxWide,
        b: &ConvexHullWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let one_f = Vector::<f32>::splat(1.0);
        let zero_i = Vector::<i32>::splat(0);

        let mut box_orientation = Matrix3x3Wide::default();
        let mut hull_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut box_orientation);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut hull_orientation);
        let mut hull_local_box_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(
            &box_orientation,
            &hull_orientation,
            &mut hull_local_box_orientation,
        );

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
        let box_support_finder = BoxSupportFinder;
        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut hull_epsilon_scale = Vector::<f32>::default();
        b.estimate_epsilon_scale(&inactive_lanes, &mut hull_epsilon_scale);
        let epsilon_scale = a
            .half_width
            .simd_max(a.half_height.simd_max(a.half_length))
            .simd_min(hull_epsilon_scale);
        let depth_threshold = -*speculative_margin;

        let mut depth = Vector::<f32>::default();
        let mut local_normal = Vector3Wide::default();
        let mut closest_on_hull = Vector3Wide::default();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &hull_local_box_orientation,
            &hull_support_finder,
            &box_support_finder,
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
        // Clear all contact exists states up front.
        manifold.contact0_exists = zero_i;
        manifold.contact1_exists = zero_i;
        manifold.contact2_exists = zero_i;
        manifold.contact3_exists = zero_i;
        if inactive_lanes.simd_lt(zero_i).all() {
            return;
        }

        // Identify the box face.
        let mut local_normal_in_a = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &local_normal,
            &hull_local_box_orientation,
            &mut local_normal_in_a,
        );
        let abs_local_normal_in_a = local_normal_in_a.abs();
        let use_x = abs_local_normal_in_a.x.simd_gt(abs_local_normal_in_a.y)
            & abs_local_normal_in_a.x.simd_gt(abs_local_normal_in_a.z);
        let use_y = (!use_x) & abs_local_normal_in_a.y.simd_gt(abs_local_normal_in_a.z);
        let use_x_i = use_x.to_int();
        let use_y_i = use_y.to_int();

        let mut box_face_normal = Vector3Wide::conditional_select(
            &use_x_i,
            &hull_local_box_orientation.x,
            &hull_local_box_orientation.z,
        );
        box_face_normal = Vector3Wide::conditional_select(
            &use_y_i,
            &hull_local_box_orientation.y,
            &box_face_normal,
        );
        let mut box_face_x = Vector3Wide::conditional_select(
            &use_x_i,
            &hull_local_box_orientation.y,
            &hull_local_box_orientation.x,
        );
        box_face_x =
            Vector3Wide::conditional_select(&use_y_i, &hull_local_box_orientation.z, &box_face_x);
        let mut box_face_y = Vector3Wide::conditional_select(
            &use_x_i,
            &hull_local_box_orientation.z,
            &hull_local_box_orientation.y,
        );
        box_face_y =
            Vector3Wide::conditional_select(&use_y_i, &hull_local_box_orientation.x, &box_face_y);

        let negate_face = use_x.select(
            local_normal_in_a.x.simd_gt(zero_f).to_int(),
            use_y.select(
                local_normal_in_a.y.simd_gt(zero_f).to_int(),
                local_normal_in_a.z.simd_gt(zero_f).to_int(),
            ),
        );
        Vector3Wide::conditionally_negate(&negate_face, &mut box_face_normal);
        // Winding is important; flip the face bases if necessary.
        let not_negate_face = !negate_face;
        Vector3Wide::conditionally_negate(&not_negate_face, &mut box_face_x);

        let box_face_half_width = use_x.select(a.half_height, use_y.select(a.half_length, a.half_width));
        let box_face_half_height = use_x.select(a.half_length, use_y.select(a.half_width, a.half_height));
        let box_face_normal_offset = use_x.select(a.half_width, use_y.select(a.half_height, a.half_length));

        let box_face_center_offset = Vector3Wide::scale(&box_face_normal, &box_face_normal_offset);
        let mut box_face_center = Vector3Wide::default();
        Vector3Wide::add(&box_face_center_offset, &local_offset_a, &mut box_face_center);

        let box_face_x_offset = Vector3Wide::scale(&box_face_x, &box_face_half_width);
        let box_face_y_offset = Vector3Wide::scale(&box_face_y, &box_face_half_height);

        let mut v0 = Vector3Wide::default();
        let mut v1 = Vector3Wide::default();
        Vector3Wide::subtract(&box_face_center, &box_face_x_offset, &mut v0);
        Vector3Wide::add(&box_face_center, &box_face_x_offset, &mut v1);

        let mut v00 = Vector3Wide::default();
        let mut v01 = Vector3Wide::default();
        let mut v10 = Vector3Wide::default();
        let mut v11 = Vector3Wide::default();
        Vector3Wide::subtract(&v0, &box_face_y_offset, &mut v00);
        Vector3Wide::add(&v0, &box_face_y_offset, &mut v01);
        Vector3Wide::subtract(&v1, &box_face_y_offset, &mut v10);
        Vector3Wide::add(&v1, &box_face_y_offset, &mut v11);

        let mut slot_offset_indices = Vector::<i32>::splat(0);
        Helpers::fill_vector_with_lane_indices(&mut slot_offset_indices);
        let bounding_plane_epsilon = Vector::<f32>::splat(1e-3) * epsilon_scale;

        let mut slot_hull_face_normals = [Vec3::ZERO; 16];
        let mut slot_local_normals = [Vec3::ZERO; 16];
        let mut hull_face_starts = [0usize; 16];
        let mut hull_face_counts = [0usize; 16];
        let mut hull_face_normal = Vector3Wide::default();
        let mut maximum_face_vertex_count = 0usize;

        for slot_index in 0..pair_count as usize {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let hull = &b.hulls[slot_index];
            let mut best_face_index = 0i32;
            ConvexHullTestHelper::pick_representative_face(
                hull,
                slot_index,
                &local_normal,
                &closest_on_hull,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_hull_face_normals[slot_index],
                &mut slot_local_normals[slot_index],
                &mut best_face_index,
            );
            Vector3Wide::write_slot(
                slot_hull_face_normals[slot_index],
                slot_index,
                &mut hull_face_normal,
            );
            hull.get_vertex_indices_for_face(
                best_face_index as usize,
                &mut hull_face_starts[slot_index],
                &mut hull_face_counts[slot_index],
            );
            if hull_face_counts[slot_index] > maximum_face_vertex_count {
                maximum_face_vertex_count = hull_face_counts[slot_index];
            }
        }

        // To find the contact manifold, we clip box edges against the hull face per-slot.
        // There can be no more than 8 contacts from edge intersections, but more from hull faces with many vertices.
        let maximum_contact_count = 8usize.max(maximum_face_vertex_count);
        let mut candidates_buf = [ManifoldCandidateScalar::default(); 128];

        for slot_index in 0..pair_count as usize {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let hull = &b.hulls[slot_index];
            let slot_face_normal = slot_hull_face_normals[slot_index];
            let slot_local_normal = slot_local_normals[slot_index];

            // Get box face vertex positions for this slot via pointer offset.
            let v00_slot = GatherScatter::get_offset_instance(&v00, slot_index);
            let v10_slot = GatherScatter::get_offset_instance(&v10, slot_index);
            let v11_slot = GatherScatter::get_offset_instance(&v11, slot_index);
            let v01_slot = GatherScatter::get_offset_instance(&v01, slot_index);
            let slot_face_x = GatherScatter::get_offset_instance(&box_face_x, slot_index);
            let slot_face_y = GatherScatter::get_offset_instance(&box_face_y, slot_index);

            // 4 box edges: X is 00->10, Y is 10->11, Z is 11->01, W is 01->00
            let box_edge_start_x =
                [v00_slot.x[0], v10_slot.x[0], v11_slot.x[0], v01_slot.x[0]];
            let box_edge_start_y =
                [v00_slot.y[0], v10_slot.y[0], v11_slot.y[0], v01_slot.y[0]];
            let box_edge_start_z =
                [v00_slot.z[0], v10_slot.z[0], v11_slot.z[0], v01_slot.z[0]];
            let edge_direction_x = [
                slot_face_x.x[0],
                slot_face_y.x[0],
                -slot_face_x.x[0],
                -slot_face_y.x[0],
            ];
            let edge_direction_y = [
                slot_face_x.y[0],
                slot_face_y.y[0],
                -slot_face_x.y[0],
                -slot_face_y.y[0],
            ];
            let edge_direction_z = [
                slot_face_x.z[0],
                slot_face_y.z[0],
                -slot_face_x.z[0],
                -slot_face_y.z[0],
            ];

            let slot_local_normal_x4 = [slot_local_normal.x; 4];
            let slot_local_normal_y4 = [slot_local_normal.y; 4];
            let slot_local_normal_z4 = [slot_local_normal.z; 4];

            // edgePlaneNormal = edgeDirection x localNormal
            let mut edge_plane_normal_x = [0.0f32; 4];
            let mut edge_plane_normal_y = [0.0f32; 4];
            let mut edge_plane_normal_z = [0.0f32; 4];
            for k in 0..4 {
                edge_plane_normal_x[k] = edge_direction_y[k] * slot_local_normal_z4[k]
                    - edge_direction_z[k] * slot_local_normal_y4[k];
                edge_plane_normal_y[k] = edge_direction_z[k] * slot_local_normal_x4[k]
                    - edge_direction_x[k] * slot_local_normal_z4[k];
                edge_plane_normal_z[k] = edge_direction_x[k] * slot_local_normal_y4[k]
                    - edge_direction_y[k] * slot_local_normal_x4[k];
            }

            let face_start = hull_face_starts[slot_index];
            let face_count = hull_face_counts[slot_index];

            // Get previous vertex (last vertex of face).
            let previous_index_init = hull.face_vertex_indices[face_start + face_count - 1];
            let mut hull_face_origin = Vec3::ZERO;
            Vector3Wide::read_slot(
                &hull.points[previous_index_init.bundle_index as usize],
                previous_index_init.inner_index as usize,
                &mut hull_face_origin,
            );
            let mut previous_vertex = hull_face_origin;
            let mut previous_index = previous_index_init;
            let mut candidate_count = 0usize;

            let mut hull_face_x = Vec3::ZERO;
            let mut hull_face_y = Vec3::ZERO;
            Helpers::build_orthonormal_basis_scalar(
                slot_face_normal,
                &mut hull_face_x,
                &mut hull_face_y,
            );

            let mut maximum_vertex_containment_dots = [0.0f32; 4];

            for i in 0..face_count {
                let index = hull.face_vertex_indices[face_start + i];
                let mut vertex = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull.points[index.bundle_index as usize],
                    index.inner_index as usize,
                    &mut vertex,
                );

                let hull_edge_offset = vertex - previous_vertex;
                let hull_edge_start_x4 = [previous_vertex.x; 4];
                let hull_edge_start_y4 = [previous_vertex.y; 4];
                let hull_edge_start_z4 = [previous_vertex.z; 4];
                let hull_edge_offset_x4 = [hull_edge_offset.x; 4];
                let hull_edge_offset_y4 = [hull_edge_offset.y; 4];
                let hull_edge_offset_z4 = [hull_edge_offset.z; 4];

                // Containment test: hull edge plane normal = hullEdgeOffset x slotLocalNormal
                let hull_edge_plane_normal = hull_edge_offset.cross(slot_local_normal);
                let hull_edge_plane_normal_x4 = [hull_edge_plane_normal.x; 4];
                let hull_edge_plane_normal_y4 = [hull_edge_plane_normal.y; 4];
                let hull_edge_plane_normal_z4 = [hull_edge_plane_normal.z; 4];

                let mut hull_edge_start_to_box_edge_x = [0.0f32; 4];
                let mut hull_edge_start_to_box_edge_y = [0.0f32; 4];
                let mut hull_edge_start_to_box_edge_z = [0.0f32; 4];
                let mut box_vertex_containment_dots = [0.0f32; 4];
                let mut numerator = [0.0f32; 4];
                let mut denominator = [0.0f32; 4];
                let mut edge_intersections = [0.0f32; 4];
                for k in 0..4 {
                    hull_edge_start_to_box_edge_x[k] =
                        box_edge_start_x[k] - hull_edge_start_x4[k];
                    hull_edge_start_to_box_edge_y[k] =
                        box_edge_start_y[k] - hull_edge_start_y4[k];
                    hull_edge_start_to_box_edge_z[k] =
                        box_edge_start_z[k] - hull_edge_start_z4[k];
                    box_vertex_containment_dots[k] = hull_edge_plane_normal_x4[k]
                        * hull_edge_start_to_box_edge_x[k]
                        + hull_edge_plane_normal_y4[k] * hull_edge_start_to_box_edge_y[k]
                        + hull_edge_plane_normal_z4[k] * hull_edge_start_to_box_edge_z[k];
                    if box_vertex_containment_dots[k] > maximum_vertex_containment_dots[k] {
                        maximum_vertex_containment_dots[k] = box_vertex_containment_dots[k];
                    }
                    numerator[k] = hull_edge_start_to_box_edge_x[k] * edge_plane_normal_x[k]
                        + hull_edge_start_to_box_edge_y[k] * edge_plane_normal_y[k]
                        + hull_edge_start_to_box_edge_z[k] * edge_plane_normal_z[k];
                    denominator[k] = edge_plane_normal_x[k] * hull_edge_offset_x4[k]
                        + edge_plane_normal_y[k] * hull_edge_offset_y4[k]
                        + edge_plane_normal_z[k] * hull_edge_offset_z4[k];
                    edge_intersections[k] = numerator[k] / denominator[k];
                }

                // Compute latest entry / earliest exit across 4 box edges.
                let mut latest_entry;
                let mut earliest_exit;
                if denominator[0] < 0.0 {
                    latest_entry = edge_intersections[0];
                    earliest_exit = f32::MAX;
                } else if denominator[0] > 0.0 {
                    latest_entry = f32::MIN;
                    earliest_exit = edge_intersections[0];
                } else if numerator[0] < 0.0 {
                    earliest_exit = f32::MIN;
                    latest_entry = f32::MAX;
                } else {
                    latest_entry = f32::MIN;
                    earliest_exit = f32::MAX;
                }
                // Edge Y
                if denominator[1] < 0.0 {
                    if edge_intersections[1] > latest_entry {
                        latest_entry = edge_intersections[1];
                    }
                } else if denominator[1] > 0.0 {
                    if edge_intersections[1] < earliest_exit {
                        earliest_exit = edge_intersections[1];
                    }
                } else if numerator[1] < 0.0 {
                    earliest_exit = f32::MIN;
                    latest_entry = f32::MAX;
                }
                // Edge Z
                if denominator[2] < 0.0 {
                    if edge_intersections[2] > latest_entry {
                        latest_entry = edge_intersections[2];
                    }
                } else if denominator[2] > 0.0 {
                    if edge_intersections[2] < earliest_exit {
                        earliest_exit = edge_intersections[2];
                    }
                } else if numerator[2] < 0.0 {
                    earliest_exit = f32::MIN;
                    latest_entry = f32::MAX;
                }
                // Edge W
                if denominator[3] < 0.0 {
                    if edge_intersections[3] > latest_entry {
                        latest_entry = edge_intersections[3];
                    }
                } else if denominator[3] > 0.0 {
                    if edge_intersections[3] < earliest_exit {
                        earliest_exit = edge_intersections[3];
                    }
                } else if numerator[3] < 0.0 {
                    earliest_exit = f32::MIN;
                    latest_entry = f32::MAX;
                }

                // We now have a convex hull edge interval. Add contacts for it.
                if latest_entry < 0.0 {
                    latest_entry = 0.0;
                }
                if earliest_exit > 1.0 {
                    earliest_exit = 1.0;
                }
                let start_id = ((previous_index.bundle_index as usize)
                    << BundleIndexing::vector_shift())
                    + previous_index.inner_index as usize;
                let end_id = ((index.bundle_index as usize) << BundleIndexing::vector_shift())
                    + index.inner_index as usize;
                let base_feature_id = ((start_id ^ end_id) << 8) as i32;

                if earliest_exit >= latest_entry && candidate_count < maximum_contact_count {
                    // Create max contact.
                    let point =
                        hull_edge_offset * earliest_exit + previous_vertex - hull_face_origin;
                    let c = &mut candidates_buf[candidate_count];
                    c.x = point.dot(hull_face_x);
                    c.y = point.dot(hull_face_y);
                    c.feature_id = base_feature_id + end_id as i32;
                    candidate_count += 1;
                }
                if latest_entry < earliest_exit
                    && latest_entry > 0.0
                    && candidate_count < maximum_contact_count
                {
                    // Create min contact.
                    let point =
                        hull_edge_offset * latest_entry + previous_vertex - hull_face_origin;
                    let c = &mut candidates_buf[candidate_count];
                    c.x = point.dot(hull_face_x);
                    c.y = point.dot(hull_face_y);
                    c.feature_id = base_feature_id + start_id as i32;
                    candidate_count += 1;
                }

                previous_index = index;
                previous_vertex = vertex;
            }

            if candidate_count < maximum_contact_count {
                // Try adding box vertex contacts. Project each vertex onto the hull face.
                // t = dot(boxVertex - hullFaceVertex, hullFacePlaneNormal) / dot(hullFacePlaneNormal, localNormal)
                let hull_face_origin_x4 = [hull_face_origin.x; 4];
                let hull_face_origin_y4 = [hull_face_origin.y; 4];
                let hull_face_origin_z4 = [hull_face_origin.z; 4];
                let hull_face_normal_x4 = [slot_face_normal.x; 4];
                let hull_face_normal_y4 = [slot_face_normal.y; 4];
                let hull_face_normal_z4 = [slot_face_normal.z; 4];

                let mut closest_to_box_x = [0.0f32; 4];
                let mut closest_to_box_y = [0.0f32; 4];
                let mut closest_to_box_z = [0.0f32; 4];
                let mut vertex_proj_num = [0.0f32; 4];
                for k in 0..4 {
                    closest_to_box_x[k] = box_edge_start_x[k] - hull_face_origin_x4[k];
                    closest_to_box_y[k] = box_edge_start_y[k] - hull_face_origin_y4[k];
                    closest_to_box_z[k] = box_edge_start_z[k] - hull_face_origin_z4[k];
                    vertex_proj_num[k] = closest_to_box_x[k] * hull_face_normal_x4[k]
                        + closest_to_box_y[k] * hull_face_normal_y4[k]
                        + closest_to_box_z[k] * hull_face_normal_z4[k];
                }
                let vertex_proj_denom = slot_face_normal.dot(slot_local_normal);

                let mut projected_tangent_x = [0.0f32; 4];
                let mut projected_tangent_y = [0.0f32; 4];
                for k in 0..4 {
                    let t = vertex_proj_num[k] / vertex_proj_denom;
                    // Normal points from B to A.
                    let px = closest_to_box_x[k] - t * slot_local_normal.x;
                    let py = closest_to_box_y[k] - t * slot_local_normal.y;
                    let pz = closest_to_box_z[k] - t * slot_local_normal.z;
                    projected_tangent_x[k] =
                        px * hull_face_x.x + py * hull_face_x.y + pz * hull_face_x.z;
                    projected_tangent_y[k] =
                        px * hull_face_y.x + py * hull_face_y.y + pz * hull_face_y.z;
                }

                // Check each of 4 box vertices for containment inside hull face.
                if maximum_vertex_containment_dots[0] <= 0.0 {
                    let c = &mut candidates_buf[candidate_count];
                    c.x = projected_tangent_x[0];
                    c.y = projected_tangent_y[0];
                    c.feature_id = 0;
                    candidate_count += 1;
                }
                if candidate_count < maximum_contact_count
                    && maximum_vertex_containment_dots[1] <= 0.0
                {
                    let c = &mut candidates_buf[candidate_count];
                    c.x = projected_tangent_x[1];
                    c.y = projected_tangent_y[1];
                    c.feature_id = 1;
                    candidate_count += 1;
                }
                if candidate_count < maximum_contact_count
                    && maximum_vertex_containment_dots[2] <= 0.0
                {
                    let c = &mut candidates_buf[candidate_count];
                    c.x = projected_tangent_x[2];
                    c.y = projected_tangent_y[2];
                    c.feature_id = 2;
                    candidate_count += 1;
                }
                if candidate_count < maximum_contact_count
                    && maximum_vertex_containment_dots[3] <= 0.0
                {
                    let c = &mut candidates_buf[candidate_count];
                    c.x = projected_tangent_x[3];
                    c.y = projected_tangent_y[3];
                    c.feature_id = 3;
                    candidate_count += 1;
                }
            }

            // Reduce to 4 contacts.
            let mut slot_box_face_center = Vec3::ZERO;
            let mut slot_box_face_normal = Vec3::ZERO;
            let mut slot_offset_b = Vec3::ZERO;
            Vector3Wide::read_slot(&box_face_center, slot_index, &mut slot_box_face_center);
            Vector3Wide::read_slot(&box_face_normal, slot_index, &mut slot_box_face_normal);
            Vector3Wide::read_slot(offset_b, slot_index, &mut slot_offset_b);
            let mut slot_hull_orientation = Matrix3x3::default();
            Matrix3x3Wide::read_slot(&hull_orientation, slot_index, &mut slot_hull_orientation);
            ManifoldCandidateHelper::reduce_scalar(
                candidates_buf.as_mut_ptr(),
                candidate_count as i32,
                slot_box_face_normal,
                1.0 / slot_box_face_normal.dot(slot_local_normal),
                slot_box_face_center,
                hull_face_origin,
                hull_face_x,
                hull_face_y,
                epsilon_scale.as_array()[slot_index],
                depth_threshold.as_array()[slot_index],
                &slot_hull_orientation,
                slot_offset_b,
                slot_index,
                manifold,
            );
        }

        // The reduction does not assign the normal. Fill it in.
        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &hull_orientation,
            &mut manifold.normal,
        );
    }
}
