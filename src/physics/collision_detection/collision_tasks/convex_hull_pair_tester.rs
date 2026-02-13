// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexHullPairTester.cs

use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collision_detection::collision_tasks::convex_hull_test_helper::ConvexHullTestHelper;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidateHelper, ManifoldCandidateScalar,
};
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::physics::helpers::Helpers;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::prelude::*;

/// Cached edge data for face A's edges, used during edge clipping.
#[derive(Clone, Copy)]
struct CachedEdge {
    vertex: Vec3,
    edge_plane_normal: Vec3,
    maximum_containment_dot: f32,
}

impl Default for CachedEdge {
    fn default() -> Self {
        Self {
            vertex: Vec3::ZERO,
            edge_plane_normal: Vec3::ZERO,
            maximum_containment_dot: f32::MIN,
        }
    }
}

/// Pair tester for convex hull vs convex hull collisions.
pub struct ConvexHullPairTester;

impl ConvexHullPairTester {
    pub const BATCH_SIZE: i32 = 16;

    /// Tests convex hull vs convex hull collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &ConvexHullWide,
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

        let mut r_a = Matrix3x3Wide::default();
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut r_a);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let mut b_local_orientation_a = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(
            &r_a,
            &r_b,
            &mut b_local_orientation_a,
        );

        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &r_b, &mut local_offset_b);
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);
        let mut center_distance = zero_f;
        Vector3Wide::length_into(&local_offset_a, &mut center_distance);
        let mut initial_normal = Vector3Wide::default();
        Vector3Wide::scale_to(
            &local_offset_a,
            &(one_f / center_distance),
            &mut initial_normal,
        );
        let use_initial_fallback = center_distance.simd_lt(Vector::<f32>::splat(1e-8));
        initial_normal.x = use_initial_fallback.select(zero_f, initial_normal.x);
        initial_normal.y = use_initial_fallback.select(one_f, initial_normal.y);
        initial_normal.z = use_initial_fallback.select(zero_f, initial_normal.z);

        let hull_support_finder = ConvexHullSupportFinder;
        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut a_epsilon_scale = zero_f;
        let mut b_epsilon_scale = zero_f;
        a.estimate_epsilon_scale(&inactive_lanes, &mut a_epsilon_scale);
        b.estimate_epsilon_scale(&inactive_lanes, &mut b_epsilon_scale);
        let epsilon_scale = a_epsilon_scale.simd_min(b_epsilon_scale);
        let depth_threshold = -*speculative_margin;

        let mut depth = zero_f;
        let mut local_normal = Vector3Wide::default();
        let mut closest_on_b = Vector3Wide::default();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &b_local_orientation_a,
            &hull_support_finder,
            &hull_support_finder,
            &initial_normal,
            &inactive_lanes,
            &(Vector::<f32>::splat(1e-5) * epsilon_scale),
            &depth_threshold,
            &mut depth,
            &mut local_normal,
            &mut closest_on_b,
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

        // Compute local normal in A's frame and closest on A.
        let mut local_normal_in_a = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &local_normal,
            &b_local_orientation_a,
            &mut local_normal_in_a,
        );
        let mut negated_local_normal_in_a = Vector3Wide::default();
        Vector3Wide::negate(&local_normal_in_a, &mut negated_local_normal_in_a);
        let mut negated_offset_to_closest_on_a = Vector3Wide::default();
        Vector3Wide::scale_to(&local_normal, &depth, &mut negated_offset_to_closest_on_a);
        let mut closest_on_a = Vector3Wide::default();
        Vector3Wide::subtract(
            &closest_on_b,
            &negated_offset_to_closest_on_a,
            &mut closest_on_a,
        );
        let mut a_to_closest_on_a = Vector3Wide::default();
        Vector3Wide::subtract(&closest_on_a, &local_offset_a, &mut a_to_closest_on_a);
        let mut closest_on_a_in_a = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &a_to_closest_on_a,
            &b_local_orientation_a,
            &mut closest_on_a_in_a,
        );

        let mut slot_offset_indices = zero_i;
        Helpers::fill_vector_with_lane_indices(&mut slot_offset_indices);
        let bounding_plane_epsilon = Vector::<f32>::splat(1e-3) * epsilon_scale;

        for slot_index in 0..pair_count as usize {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let a_slot = &a.hulls[slot_index];
            let b_slot = &b.hulls[slot_index];

            // Pick representative face on A (in A-local space).
            let mut slot_face_normal_a_in_a = Vec3::ZERO;
            let mut slot_local_normal_tmp = Vec3::ZERO;
            let mut best_face_index_a = 0i32;
            ConvexHullTestHelper::pick_representative_face(
                a_slot,
                slot_index,
                &negated_local_normal_in_a,
                &closest_on_a_in_a,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_face_normal_a_in_a,
                &mut slot_local_normal_tmp,
                &mut best_face_index_a,
            );
            // Transform face normal A from A-local to B-local.
            let mut slot_b_local_orientation_a = Matrix3x3::default();
            Matrix3x3Wide::read_slot(
                &b_local_orientation_a,
                slot_index,
                &mut slot_b_local_orientation_a,
            );
            let mut slot_face_normal_a = Vec3::ZERO;
            Matrix3x3::transform(
                &slot_face_normal_a_in_a,
                &slot_b_local_orientation_a,
                &mut slot_face_normal_a,
            );
            let mut slot_local_offset_a = Vec3::ZERO;
            Vector3Wide::read_slot(&local_offset_a, slot_index, &mut slot_local_offset_a);

            // Pick representative face on B (in B-local space).
            let mut slot_face_normal_b = Vec3::ZERO;
            let mut slot_local_normal = Vec3::ZERO;
            let mut best_face_index_b = 0i32;
            ConvexHullTestHelper::pick_representative_face(
                b_slot,
                slot_index,
                &local_normal,
                &closest_on_b,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_face_normal_b,
                &mut slot_local_normal,
                &mut best_face_index_b,
            );
            let mut b_face_x = Vec3::ZERO;
            let mut b_face_y = Vec3::ZERO;
            Helpers::build_orthonormal_basis_scalar(
                slot_face_normal_b,
                &mut b_face_x,
                &mut b_face_y,
            );

            // Get face vertex indices.
            let mut face_start_a = 0usize;
            let mut face_count_a = 0usize;
            let mut face_start_b = 0usize;
            let mut face_count_b = 0usize;
            a_slot.get_vertex_indices_for_face(
                best_face_index_a as usize,
                &mut face_start_a,
                &mut face_count_a,
            );
            b_slot.get_vertex_indices_for_face(
                best_face_index_b as usize,
                &mut face_start_b,
                &mut face_count_b,
            );

            // Create cached edge data for A.
            // Transform A's vertices into B-local space.
            let mut cached_edges = [CachedEdge::default(); 64];
            let previous_index_a = a_slot.face_vertex_indices[face_start_a + face_count_a - 1];
            let mut previous_vertex_a = Vec3::ZERO;
            Vector3Wide::read_slot(
                &a_slot.points[previous_index_a.bundle_index as usize],
                previous_index_a.inner_index as usize,
                &mut previous_vertex_a,
            );
            Matrix3x3::transform(
                &previous_vertex_a.clone(),
                &slot_b_local_orientation_a,
                &mut previous_vertex_a,
            );
            previous_vertex_a += slot_local_offset_a;

            for i in 0..face_count_a {
                let edge = &mut cached_edges[i];
                edge.maximum_containment_dot = f32::MIN;
                let index_a = a_slot.face_vertex_indices[face_start_a + i];
                Vector3Wide::read_slot(
                    &a_slot.points[index_a.bundle_index as usize],
                    index_a.inner_index as usize,
                    &mut edge.vertex,
                );
                let vtmp = edge.vertex;
                Matrix3x3::transform(&vtmp, &slot_b_local_orientation_a, &mut edge.vertex);
                edge.vertex += slot_local_offset_a;
                // Note flipped cross order; local normal points from B to A.
                edge.edge_plane_normal = slot_local_normal.cross(edge.vertex - previous_vertex_a);
                previous_vertex_a = edge.vertex;
            }

            // Clip B's face edges against A's face, and test A's vertices against B's face.
            let maximum_candidate_count = face_count_a
                .max(face_count_b)
                .max((face_count_a * 2).min(face_count_b * 2));
            let mut candidates_buf = [ManifoldCandidateScalar::default(); 128];
            let mut candidate_count = 0usize;

            let previous_index_b_init = b_slot.face_vertex_indices[face_start_b + face_count_b - 1];
            let mut b_face_origin = Vec3::ZERO;
            Vector3Wide::read_slot(
                &b_slot.points[previous_index_b_init.bundle_index as usize],
                previous_index_b_init.inner_index as usize,
                &mut b_face_origin,
            );
            let mut previous_vertex_b = b_face_origin;
            let mut previous_index_b = previous_index_b_init;

            for face_vertex_index_b in 0..face_count_b {
                let index_b = b_slot.face_vertex_indices[face_start_b + face_vertex_index_b];
                let mut vertex_b = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &b_slot.points[index_b.bundle_index as usize],
                    index_b.inner_index as usize,
                    &mut vertex_b,
                );

                let edge_offset_b = vertex_b - previous_vertex_b;
                let edge_plane_normal_b = edge_offset_b.cross(slot_local_normal);

                let mut latest_entry = f32::MIN;
                let mut earliest_exit = f32::MAX;

                for face_vertex_index_a in 0..face_count_a {
                    let edge_a = &mut cached_edges[face_vertex_index_a];

                    // Check containment of A vertex in this B edge.
                    let edge_b_to_edge_a = edge_a.vertex - previous_vertex_b;
                    let containment_dot = edge_b_to_edge_a.dot(edge_plane_normal_b);
                    if edge_a.maximum_containment_dot < containment_dot {
                        edge_a.maximum_containment_dot = containment_dot;
                    }

                    // t = dot(pointOnEdgeA - pointOnEdgeB, edgePlaneNormalA)
                    //   / dot(edgePlaneNormalA, edgeOffsetB)
                    let numerator = edge_b_to_edge_a.dot(edge_a.edge_plane_normal);
                    let denominator = edge_a.edge_plane_normal.dot(edge_offset_b);

                    if denominator < 0.0 {
                        // Note compare flip for denominator sign.
                        if numerator < latest_entry * denominator {
                            latest_entry = numerator / denominator;
                        }
                    } else if denominator > 0.0 {
                        if numerator < earliest_exit * denominator {
                            earliest_exit = numerator / denominator;
                        }
                    } else if numerator < 0.0 {
                        // B edge is parallel and outside edge A.
                        earliest_exit = f32::MIN;
                        latest_entry = f32::MAX;
                    }
                }

                // Bounds on B's edge. Denominator signs are opposed; comparison flipped.
                if latest_entry <= earliest_exit {
                    // This edge of B was actually contained in A's face.
                    latest_entry = if latest_entry < 0.0 {
                        0.0
                    } else {
                        latest_entry
                    };
                    earliest_exit = if earliest_exit > 1.0 {
                        1.0
                    } else {
                        earliest_exit
                    };

                    let start_id = ((previous_index_b.bundle_index as usize)
                        << BundleIndexing::vector_shift())
                        + previous_index_b.inner_index as usize;
                    let end_id = ((index_b.bundle_index as usize)
                        << BundleIndexing::vector_shift())
                        + index_b.inner_index as usize;
                    let base_feature_id = ((start_id ^ end_id) << 8) as i32;

                    if earliest_exit >= latest_entry && candidate_count < maximum_candidate_count {
                        // Create max contact.
                        let point =
                            edge_offset_b * earliest_exit + previous_vertex_b - b_face_origin;
                        let c = &mut candidates_buf[candidate_count];
                        c.x = point.dot(b_face_x);
                        c.y = point.dot(b_face_y);
                        c.feature_id = base_feature_id + end_id as i32;
                        candidate_count += 1;
                    }
                    if latest_entry < earliest_exit
                        && latest_entry > 0.0
                        && candidate_count < maximum_candidate_count
                    {
                        // Create min contact.
                        let point =
                            edge_offset_b * latest_entry + previous_vertex_b - b_face_origin;
                        let c = &mut candidates_buf[candidate_count];
                        c.x = point.dot(b_face_x);
                        c.y = point.dot(b_face_y);
                        c.feature_id = base_feature_id + start_id as i32;
                        candidate_count += 1;
                    }
                }

                previous_index_b = index_b;
                previous_vertex_b = vertex_b;
            }

            // Check for A vertices contained in B's face.
            let inverse_local_normal_a_dot_face_normal_b =
                1.0 / slot_local_normal.dot(slot_face_normal_b);
            for i in 0..face_count_a {
                if candidate_count >= maximum_candidate_count {
                    break;
                }
                let edge = &cached_edges[i];
                if edge.maximum_containment_dot <= 0.0 {
                    // This vertex was contained by all B edge plane normals.
                    // Project it onto B's surface.
                    let b_face_to_vertex_a = edge.vertex - b_face_origin;
                    let distance = b_face_to_vertex_a.dot(slot_face_normal_b)
                        * inverse_local_normal_a_dot_face_normal_b;
                    let b_face_to_projected = b_face_to_vertex_a - slot_local_normal * distance;

                    let c = &mut candidates_buf[candidate_count];
                    c.x = b_face_x.dot(b_face_to_projected);
                    c.y = b_face_y.dot(b_face_to_projected);
                    c.feature_id = i as i32;
                    candidate_count += 1;
                }
            }

            // Reduce to 4 contacts.
            let mut slot_orientation_b = Matrix3x3::default();
            Matrix3x3Wide::read_slot(&r_b, slot_index, &mut slot_orientation_b);
            let mut slot_offset_b = Vec3::ZERO;
            Vector3Wide::read_slot(offset_b, slot_index, &mut slot_offset_b);
            ManifoldCandidateHelper::reduce_scalar(
                candidates_buf.as_mut_ptr(),
                candidate_count as i32,
                slot_face_normal_a,
                1.0 / slot_face_normal_a.dot(slot_local_normal),
                cached_edges[0].vertex,
                b_face_origin,
                b_face_x,
                b_face_y,
                epsilon_scale.as_array()[slot_index],
                depth_threshold.as_array()[slot_index],
                &slot_orientation_b,
                slot_offset_b,
                slot_index,
                manifold,
            );
        }

        // The reduction does not assign the normal. Fill it in.
        Matrix3x3Wide::transform_without_overlap(&local_normal, &r_b, &mut manifold.normal);
    }
}
