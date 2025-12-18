// Translated from BepuPhysics/CollisionDetection/CollisionTasks/TriangleConvexHullTester.cs

use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::collision_tasks::convex_hull_test_helper::ConvexHullTestHelper;
use crate::physics::collision_detection::collision_tasks::manifold_candidate_helper::{
    ManifoldCandidateHelper, ManifoldCandidateScalar,
};
use crate::physics::collision_detection::collision_tasks::triangle_cylinder_tester::PretransformedTriangleSupportFinder;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use crate::physics::collision_detection::depth_refiner::DepthRefiner;
use crate::physics::collision_detection::mesh_reduction;
use crate::physics::collision_detection::support_finder::ISupportFinder;
use crate::physics::helpers::Helpers;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::prelude::*;

/// Pair tester for triangle vs convex hull collisions.
pub struct TriangleConvexHullTester;

impl TriangleConvexHullTester {
    pub const BATCH_SIZE: i32 = 16;

    /// Tests triangle vs convex hull collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &TriangleWide,
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

        let mut triangle_orientation = Matrix3x3Wide::default();
        let mut hull_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut triangle_orientation);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut hull_orientation);
        let mut hull_local_triangle_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(
            &triangle_orientation,
            &hull_orientation,
            &mut hull_local_triangle_orientation,
        );

        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            &hull_orientation,
            &mut local_offset_b,
        );
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

        // Pretransform triangle vertices into hull-local space and recenter at centroid.
        let mut triangle = TriangleWide::default();
        Matrix3x3Wide::transform_without_overlap(
            &a.a,
            &hull_local_triangle_orientation,
            &mut triangle.a,
        );
        Matrix3x3Wide::transform_without_overlap(
            &a.b,
            &hull_local_triangle_orientation,
            &mut triangle.b,
        );
        Matrix3x3Wide::transform_without_overlap(
            &a.c,
            &hull_local_triangle_orientation,
            &mut triangle.c,
        );
        let mut centroid = Vector3Wide::default();
        Vector3Wide::add(&triangle.a, &triangle.b, &mut centroid);
        let centroid_tmp = centroid.clone();
        Vector3Wide::add(&triangle.c, &centroid_tmp, &mut centroid);
        let centroid_unscaled = centroid.clone();
        Vector3Wide::scale_to(
            &centroid_unscaled,
            &Vector::<f32>::splat(1.0 / 3.0),
            &mut centroid,
        );
        let ta = triangle.a.clone();
        Vector3Wide::subtract(&ta, &centroid, &mut triangle.a);
        let tb = triangle.b.clone();
        Vector3Wide::subtract(&tb, &centroid, &mut triangle.b);
        let tc = triangle.c.clone();
        Vector3Wide::subtract(&tc, &centroid, &mut triangle.c);
        let mut local_triangle_center = Vector3Wide::default();
        Vector3Wide::subtract(&centroid, &local_offset_b, &mut local_triangle_center);

        // Compute triangle edges.
        let mut triangle_ab = Vector3Wide::default();
        let mut triangle_bc = Vector3Wide::default();
        let mut triangle_ca = Vector3Wide::default();
        Vector3Wide::subtract(&triangle.b, &triangle.a, &mut triangle_ab);
        Vector3Wide::subtract(&triangle.c, &triangle.b, &mut triangle_bc);
        Vector3Wide::subtract(&triangle.a, &triangle.c, &mut triangle_ca);

        // Cache B-local triangle vertices.
        let mut triangle_a = Vector3Wide::default();
        let mut triangle_b_v = Vector3Wide::default();
        let mut triangle_c = Vector3Wide::default();
        Vector3Wide::add(&triangle.a, &local_triangle_center, &mut triangle_a);
        Vector3Wide::add(&triangle.b, &local_triangle_center, &mut triangle_b_v);
        Vector3Wide::add(&triangle.c, &local_triangle_center, &mut triangle_c);

        // Triangle normal.
        let mut triangle_normal = Vector3Wide::default();
        Vector3Wide::cross(&triangle_ab, &triangle_ca, &mut triangle_normal);
        let mut triangle_normal_length = zero_f;
        Vector3Wide::length_into(&triangle_normal, &mut triangle_normal_length);
        let tn_tmp = triangle_normal.clone();
        Vector3Wide::scale_to(
            &tn_tmp,
            &(one_f / triangle_normal_length),
            &mut triangle_normal,
        );

        // Check if the hull's position is within the triangle and below the triangle plane.
        let mut hull_to_triangle_center_dot = zero_f;
        Vector3Wide::dot(
            &triangle_normal,
            &local_triangle_center,
            &mut hull_to_triangle_center_dot,
        );
        let hull_below_plane = hull_to_triangle_center_dot.simd_ge(zero_f);
        let mut edge_plane_ab = Vector3Wide::default();
        let mut edge_plane_bc = Vector3Wide::default();
        let mut edge_plane_ca = Vector3Wide::default();
        Vector3Wide::cross(&triangle_ab, &triangle_normal, &mut edge_plane_ab);
        Vector3Wide::cross(&triangle_bc, &triangle_normal, &mut edge_plane_bc);
        Vector3Wide::cross(&triangle_ca, &triangle_normal, &mut edge_plane_ca);
        let mut ab_plane_test = zero_f;
        let mut bc_plane_test = zero_f;
        let mut ca_plane_test = zero_f;
        Vector3Wide::dot(&edge_plane_ab, &triangle_a, &mut ab_plane_test);
        Vector3Wide::dot(&edge_plane_bc, &triangle_b_v, &mut bc_plane_test);
        Vector3Wide::dot(&edge_plane_ca, &triangle_c, &mut ca_plane_test);
        let hull_inside_triangle_edge_planes = ab_plane_test.simd_le(zero_f).to_int()
            & bc_plane_test.simd_le(zero_f).to_int()
            & ca_plane_test.simd_le(zero_f).to_int();
        let hull_inside_and_below_triangle =
            hull_below_plane.to_int() & hull_inside_triangle_edge_planes;

        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut triangle_epsilon_scale = zero_f;
        let mut nondegenerate_mask = zero_i;
        TriangleWide::compute_nondegenerate_triangle_mask_from_edges(
            &triangle_ab,
            &triangle_ca,
            &triangle_normal_length,
            &mut triangle_epsilon_scale,
            &mut nondegenerate_mask,
        );
        let mut hull_epsilon_scale = zero_f;
        b.estimate_epsilon_scale(&inactive_lanes, &mut hull_epsilon_scale);
        let epsilon_scale = triangle_epsilon_scale.simd_min(hull_epsilon_scale);
        // Degenerate triangles don't contribute contacts.
        inactive_lanes = inactive_lanes | !nondegenerate_mask;
        inactive_lanes = inactive_lanes | hull_inside_and_below_triangle;
        // Clear all contact exists states up front.
        manifold.contact0_exists = zero_i;
        manifold.contact1_exists = zero_i;
        manifold.contact2_exists = zero_i;
        manifold.contact3_exists = zero_i;
        if inactive_lanes.simd_lt(zero_i).all() {
            return;
        }

        // Initial normal from triangle center with fallback.
        let mut center_distance = zero_f;
        Vector3Wide::length_into(&local_triangle_center, &mut center_distance);
        let mut initial_normal = Vector3Wide::default();
        Vector3Wide::scale_to(
            &local_triangle_center,
            &(one_f / center_distance),
            &mut initial_normal,
        );
        let use_initial_fallback = center_distance.simd_lt(Vector::<f32>::splat(1e-10));
        initial_normal.x = use_initial_fallback.select(zero_f, initial_normal.x);
        initial_normal.y = use_initial_fallback.select(one_f, initial_normal.y);
        initial_normal.z = use_initial_fallback.select(zero_f, initial_normal.z);

        // Face normal prepass: check if the hull's extreme point along negated triangle normal
        // lies inside the triangle. If so, no depth refinement needed.
        let hull_support_finder = ConvexHullSupportFinder;
        let triangle_support_finder = PretransformedTriangleSupportFinder;
        let mut negated_triangle_normal = Vector3Wide::default();
        Vector3Wide::negate(&triangle_normal, &mut negated_triangle_normal);
        let mut hull_support_along_negated_tn = Vector3Wide::default();
        <ConvexHullSupportFinder as ISupportFinder<ConvexHullWide>>::compute_local_support(
            b,
            &negated_triangle_normal,
            &inactive_lanes,
            &mut hull_support_along_negated_tn,
        );
        let mut support_along_negated_tn = Vector3Wide::default();
        Vector3Wide::subtract(
            &hull_support_along_negated_tn,
            &local_triangle_center,
            &mut support_along_negated_tn,
        );
        let mut triangle_face_depth = zero_f;
        Vector3Wide::dot(
            &support_along_negated_tn,
            &negated_triangle_normal,
            &mut triangle_face_depth,
        );
        let mut closest_to_a = Vector3Wide::default();
        let mut closest_to_b_v = Vector3Wide::default();
        let mut closest_to_c = Vector3Wide::default();
        Vector3Wide::subtract(
            &triangle_a,
            &hull_support_along_negated_tn,
            &mut closest_to_a,
        );
        Vector3Wide::subtract(
            &triangle_b_v,
            &hull_support_along_negated_tn,
            &mut closest_to_b_v,
        );
        Vector3Wide::subtract(
            &triangle_c,
            &hull_support_along_negated_tn,
            &mut closest_to_c,
        );
        let mut extreme_ab_test = zero_f;
        let mut extreme_bc_test = zero_f;
        let mut extreme_ca_test = zero_f;
        Vector3Wide::dot(&edge_plane_ab, &closest_to_a, &mut extreme_ab_test);
        Vector3Wide::dot(&edge_plane_bc, &closest_to_b_v, &mut extreme_bc_test);
        Vector3Wide::dot(&edge_plane_ca, &closest_to_c, &mut extreme_ca_test);
        // Triangle normal is minimal if hull center is inside + above triangle AND extreme point
        // is inside triangle edges.
        let triangle_normal_is_minimal =
            (hull_inside_triangle_edge_planes & (!hull_below_plane.to_int()))
                & extreme_ab_test.simd_le(zero_f).to_int()
                & extreme_bc_test.simd_le(zero_f).to_int()
                & extreme_ca_test.simd_le(zero_f).to_int();

        let depth_threshold = -*speculative_margin;
        let skip_depth_refine = triangle_normal_is_minimal | inactive_lanes;
        let local_normal;
        let closest_on_hull;
        let depth;

        if skip_depth_refine.simd_eq(zero_i).any() {
            let mut refined_depth = zero_f;
            let mut refined_normal = Vector3Wide::default();
            let mut refined_closest_on_hull = Vector3Wide::default();
            DepthRefiner::find_minimum_depth_with_witness(
                b,
                &triangle,
                &local_triangle_center,
                &hull_local_triangle_orientation,
                &hull_support_finder,
                &triangle_support_finder,
                &initial_normal,
                &skip_depth_refine,
                &(Vector::<f32>::splat(1e-4) * epsilon_scale),
                &depth_threshold,
                &mut refined_depth,
                &mut refined_normal,
                &mut refined_closest_on_hull,
                25,
            );
            closest_on_hull = Vector3Wide::conditional_select(
                &skip_depth_refine,
                &hull_support_along_negated_tn,
                &refined_closest_on_hull,
            );
            local_normal = Vector3Wide::conditional_select(
                &skip_depth_refine,
                &negated_triangle_normal,
                &refined_normal,
            );
            depth = skip_depth_refine
                .simd_lt(zero_i)
                .select(triangle_face_depth, refined_depth);
        } else {
            // No depth refine ran; the extreme point prepass did everything needed.
            local_normal = negated_triangle_normal.clone();
            closest_on_hull = hull_support_along_negated_tn.clone();
            depth = triangle_face_depth;
        }

        // Backface rejection + depth threshold.
        let mut triangle_normal_dot_local_normal = zero_f;
        Vector3Wide::dot(
            &triangle_normal,
            &local_normal,
            &mut triangle_normal_dot_local_normal,
        );
        inactive_lanes = inactive_lanes
            | triangle_normal_dot_local_normal
                .simd_gt(Vector::<f32>::splat(
                    -TriangleWide::BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD,
                ))
                .to_int()
            | depth.simd_lt(depth_threshold).to_int();
        if inactive_lanes.simd_lt(zero_i).all() {
            return;
        }

        // Pick representative face per slot.
        let mut slot_offset_indices = zero_i;
        Helpers::fill_vector_with_lane_indices(&mut slot_offset_indices);
        let bounding_plane_epsilon = Vector::<f32>::splat(1e-3) * epsilon_scale;
        let mut slot_hull_face_normals = [Vec3::ZERO; 16];
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
            let mut slot_local_normal_tmp = Vec3::ZERO;
            ConvexHullTestHelper::pick_representative_face(
                hull,
                slot_index,
                &local_normal,
                &closest_on_hull,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_hull_face_normals[slot_index],
                &mut slot_local_normal_tmp,
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

        // Project triangle vertices onto hull face plane.
        let mut hull_to_a = Vector3Wide::default();
        let mut hull_to_b = Vector3Wide::default();
        let mut hull_to_c = Vector3Wide::default();
        Vector3Wide::subtract(&triangle_a, &closest_on_hull, &mut hull_to_a);
        Vector3Wide::subtract(&triangle_b_v, &closest_on_hull, &mut hull_to_b);
        Vector3Wide::subtract(&triangle_c, &closest_on_hull, &mut hull_to_c);
        let mut numerator_a_to_hull_face = zero_f;
        let mut numerator_b_to_hull_face = zero_f;
        let mut numerator_c_to_hull_face = zero_f;
        Vector3Wide::dot(&hull_to_a, &hull_face_normal, &mut numerator_a_to_hull_face);
        Vector3Wide::dot(&hull_to_b, &hull_face_normal, &mut numerator_b_to_hull_face);
        Vector3Wide::dot(&hull_to_c, &hull_face_normal, &mut numerator_c_to_hull_face);
        let mut denominator_to_hull_face = zero_f;
        Vector3Wide::dot(
            &local_normal,
            &hull_face_normal,
            &mut denominator_to_hull_face,
        );
        let inverse_denominator_to_hull_face = one_f / denominator_to_hull_face;
        let t_a_to_hull_face = numerator_a_to_hull_face * inverse_denominator_to_hull_face;
        let t_b_to_hull_face = numerator_b_to_hull_face * inverse_denominator_to_hull_face;
        let t_c_to_hull_face = numerator_c_to_hull_face * inverse_denominator_to_hull_face;
        let a_on_hull = Vector3Wide {
            x: triangle_a.x - local_normal.x * t_a_to_hull_face,
            y: triangle_a.y - local_normal.y * t_a_to_hull_face,
            z: triangle_a.z - local_normal.z * t_a_to_hull_face,
        };
        let b_on_hull = Vector3Wide {
            x: triangle_b_v.x - local_normal.x * t_b_to_hull_face,
            y: triangle_b_v.y - local_normal.y * t_b_to_hull_face,
            z: triangle_b_v.z - local_normal.z * t_b_to_hull_face,
        };
        let c_on_hull = Vector3Wide {
            x: triangle_c.x - local_normal.x * t_c_to_hull_face,
            y: triangle_c.y - local_normal.y * t_c_to_hull_face,
            z: triangle_c.z - local_normal.z * t_c_to_hull_face,
        };

        // Projected triangle edges on hull.
        let mut ab_on_hull = Vector3Wide::default();
        let mut bc_on_hull = Vector3Wide::default();
        let mut ca_on_hull = Vector3Wide::default();
        Vector3Wide::subtract(&b_on_hull, &a_on_hull, &mut ab_on_hull);
        Vector3Wide::subtract(&c_on_hull, &b_on_hull, &mut bc_on_hull);
        Vector3Wide::subtract(&a_on_hull, &c_on_hull, &mut ca_on_hull);

        // Triangle tangent basis for surface coordinates.
        let mut triangle_tangent_x = Vector3Wide::default();
        Vector3Wide::normalize_to(&triangle_ab, &mut triangle_tangent_x);
        let mut triangle_tangent_y = Vector3Wide::default();
        Vector3Wide::cross(
            &triangle_tangent_x,
            &triangle_normal,
            &mut triangle_tangent_y,
        );

        // Edge planes for projected triangle on hull face.
        let mut ab_edge_plane_on_hull = Vector3Wide::default();
        let mut bc_edge_plane_on_hull = Vector3Wide::default();
        let mut ca_edge_plane_on_hull = Vector3Wide::default();
        Vector3Wide::cross(&ab_on_hull, &hull_face_normal, &mut ab_edge_plane_on_hull);
        Vector3Wide::cross(&bc_on_hull, &hull_face_normal, &mut bc_edge_plane_on_hull);
        Vector3Wide::cross(&ca_on_hull, &hull_face_normal, &mut ca_edge_plane_on_hull);

        let inverse_triangle_normal_dot_local_normal = one_f / triangle_normal_dot_local_normal;

        // Maximum contacts: 6 from edge intersections + hull face vertex count.
        let maximum_contact_count = 6usize.max(maximum_face_vertex_count);
        let mut candidates_buf = [ManifoldCandidateScalar::default(); 128];

        // Per-slot scalar loop: clip triangle edges against hull face edges.
        for slot_index in 0..pair_count as usize {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let hull = &b.hulls[slot_index];
            let slot_face_normal = slot_hull_face_normals[slot_index];
            let mut slot_local_normal = Vec3::ZERO;
            Vector3Wide::read_slot(&local_normal, slot_index, &mut slot_local_normal);
            let face_start = hull_face_starts[slot_index];
            let face_count = hull_face_counts[slot_index];

            let mut slot_triangle_a = Vec3::ZERO;
            let mut slot_triangle_b = Vec3::ZERO;
            let mut slot_triangle_c = Vec3::ZERO;
            let mut slot_triangle_normal = Vec3::ZERO;
            Vector3Wide::read_slot(&triangle_a, slot_index, &mut slot_triangle_a);
            Vector3Wide::read_slot(&triangle_b_v, slot_index, &mut slot_triangle_b);
            Vector3Wide::read_slot(&triangle_c, slot_index, &mut slot_triangle_c);
            Vector3Wide::read_slot(&triangle_normal, slot_index, &mut slot_triangle_normal);
            let slot_inverse_tn_dot_ln =
                inverse_triangle_normal_dot_local_normal.as_array()[slot_index];

            let mut slot_a_on_hull = Vec3::ZERO;
            let mut slot_b_on_hull = Vec3::ZERO;
            let mut slot_c_on_hull = Vec3::ZERO;
            Vector3Wide::read_slot(&a_on_hull, slot_index, &mut slot_a_on_hull);
            Vector3Wide::read_slot(&b_on_hull, slot_index, &mut slot_b_on_hull);
            Vector3Wide::read_slot(&c_on_hull, slot_index, &mut slot_c_on_hull);

            let mut slot_triangle_tangent_x = Vec3::ZERO;
            let mut slot_triangle_tangent_y = Vec3::ZERO;
            Vector3Wide::read_slot(&triangle_tangent_x, slot_index, &mut slot_triangle_tangent_x);
            Vector3Wide::read_slot(&triangle_tangent_y, slot_index, &mut slot_triangle_tangent_y);

            let mut slot_ab_edge_plane = Vec3::ZERO;
            let mut slot_bc_edge_plane = Vec3::ZERO;
            let mut slot_ca_edge_plane = Vec3::ZERO;
            Vector3Wide::read_slot(
                &ab_edge_plane_on_hull,
                slot_index,
                &mut slot_ab_edge_plane,
            );
            Vector3Wide::read_slot(
                &bc_edge_plane_on_hull,
                slot_index,
                &mut slot_bc_edge_plane,
            );
            Vector3Wide::read_slot(
                &ca_edge_plane_on_hull,
                slot_index,
                &mut slot_ca_edge_plane,
            );

            // Get last vertex of face as initial previous.
            let previous_index_init = hull.face_vertex_indices[face_start + face_count - 1];
            let mut hull_face_origin = Vec3::ZERO;
            Vector3Wide::read_slot(
                &hull.points[previous_index_init.bundle_index as usize],
                previous_index_init.inner_index as usize,
                &mut hull_face_origin,
            );
            let mut previous_vertex = hull_face_origin;
            let mut candidate_count = 0usize;

            let mut latest_entry_ab = f32::MIN;
            let mut earliest_exit_ab = f32::MAX;
            let mut latest_entry_bc = f32::MIN;
            let mut earliest_exit_bc = f32::MAX;
            let mut latest_entry_ca = f32::MIN;
            let mut earliest_exit_ca = f32::MAX;

            let slot_ab_on_hull = slot_b_on_hull - slot_a_on_hull;
            let slot_bc_on_hull = slot_c_on_hull - slot_b_on_hull;
            let slot_ca_on_hull = slot_a_on_hull - slot_c_on_hull;
            let slot_triangle_ab = slot_triangle_b - slot_triangle_a;
            let slot_triangle_bc = slot_triangle_c - slot_triangle_b;
            let slot_triangle_ca = slot_triangle_a - slot_triangle_c;

            for i in 0..face_count {
                let index = hull.face_vertex_indices[face_start + i];
                let mut vertex = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull.points[index.bundle_index as usize],
                    index.inner_index as usize,
                    &mut vertex,
                );

                let hull_edge_offset = vertex - previous_vertex;
                previous_vertex = vertex;

                // Containment test: check if hull vertex is inside projected triangle edge planes.
                let ap = vertex - slot_a_on_hull;
                let bp = vertex - slot_b_on_hull;
                // Use strict inequality since projected edge planes can be zero for degenerate
                // projections.
                let vertex_contained = ap.dot(slot_ab_edge_plane) < 0.0
                    && bp.dot(slot_bc_edge_plane) < 0.0
                    && ap.dot(slot_ca_edge_plane) < 0.0;
                if vertex_contained && candidate_count < maximum_contact_count {
                    // Project the hull vertex down to the triangle's surface.
                    let projection_t = (vertex - slot_triangle_a).dot(slot_triangle_normal)
                        * slot_inverse_tn_dot_ln;
                    let projected_vertex = vertex - slot_local_normal * projection_t;
                    // Use triangle.A as the surface basis origin.
                    let to_vertex = projected_vertex - slot_triangle_a;
                    let c = &mut candidates_buf[candidate_count];
                    c.x = to_vertex.dot(slot_triangle_tangent_x);
                    c.y = to_vertex.dot(slot_triangle_tangent_y);
                    // Vertex contacts occupy the feature indices after the edge slots.
                    c.feature_id = (6 + i) as i32;
                    candidate_count += 1;
                }

                // Intersect the three triangle edges against the hull edge.
                let hull_edge_plane_normal = hull_edge_offset.cross(slot_local_normal);

                // AB edge.
                let ab_numerator = ap.dot(hull_edge_plane_normal);
                let ab_denominator = hull_edge_plane_normal.dot(slot_ab_on_hull);
                if ab_denominator < 0.0 {
                    if latest_entry_ab * ab_denominator > ab_numerator {
                        latest_entry_ab = ab_numerator / ab_denominator;
                    }
                } else if ab_denominator > 0.0 {
                    if earliest_exit_ab * ab_denominator > ab_numerator {
                        earliest_exit_ab = ab_numerator / ab_denominator;
                    }
                } else if ab_denominator == 0.0 {
                    if ab_numerator < 0.0 {
                        earliest_exit_ab = f32::MIN;
                        latest_entry_ab = f32::MAX;
                    }
                }

                // BC edge.
                let bc_numerator = bp.dot(hull_edge_plane_normal);
                let bc_denominator = hull_edge_plane_normal.dot(slot_bc_on_hull);
                if bc_denominator < 0.0 {
                    if latest_entry_bc * bc_denominator > bc_numerator {
                        latest_entry_bc = bc_numerator / bc_denominator;
                    }
                } else if bc_denominator > 0.0 {
                    if earliest_exit_bc * bc_denominator > bc_numerator {
                        earliest_exit_bc = bc_numerator / bc_denominator;
                    }
                } else if bc_denominator == 0.0 {
                    if bc_numerator < 0.0 {
                        earliest_exit_bc = f32::MIN;
                        latest_entry_bc = f32::MAX;
                    }
                }

                // CA edge.
                let ca_numerator = (vertex - slot_c_on_hull).dot(hull_edge_plane_normal);
                let ca_denominator = hull_edge_plane_normal.dot(slot_ca_on_hull);
                if ca_denominator < 0.0 {
                    if latest_entry_ca * ca_denominator > ca_numerator {
                        latest_entry_ca = ca_numerator / ca_denominator;
                    }
                } else if ca_denominator > 0.0 {
                    if earliest_exit_ca * ca_denominator > ca_numerator {
                        earliest_exit_ca = ca_numerator / ca_denominator;
                    }
                } else if ca_denominator == 0.0 {
                    if ca_numerator < 0.0 {
                        earliest_exit_ca = f32::MIN;
                        latest_entry_ca = f32::MAX;
                    }
                }
            }

            // Clamp intervals to [0, 1].
            latest_entry_ab = latest_entry_ab.max(0.0);
            latest_entry_bc = latest_entry_bc.max(0.0);
            latest_entry_ca = latest_entry_ca.max(0.0);
            earliest_exit_ab = earliest_exit_ab.min(1.0);
            earliest_exit_bc = earliest_exit_bc.min(1.0);
            earliest_exit_ca = earliest_exit_ca.min(1.0);

            // AB edge contacts. Note triangle A is origin for surface basis.
            if earliest_exit_ab >= latest_entry_ab && candidate_count < maximum_contact_count {
                let point = slot_triangle_ab * earliest_exit_ab;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 0;
                candidate_count += 1;
            }
            if latest_entry_ab < earliest_exit_ab
                && latest_entry_ab > 0.0
                && candidate_count < maximum_contact_count
            {
                let point = slot_triangle_ab * latest_entry_ab;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 1;
                candidate_count += 1;
            }

            // BC edge contacts.
            if earliest_exit_bc >= latest_entry_bc && candidate_count < maximum_contact_count {
                let point = slot_triangle_bc * earliest_exit_bc + slot_triangle_ab;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 2;
                candidate_count += 1;
            }
            if latest_entry_bc < earliest_exit_bc
                && latest_entry_bc > 0.0
                && candidate_count < maximum_contact_count
            {
                let point = slot_triangle_bc * latest_entry_bc + slot_triangle_ab;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 3;
                candidate_count += 1;
            }

            // CA edge contacts.
            if earliest_exit_ca >= latest_entry_ca && candidate_count < maximum_contact_count {
                let point = slot_triangle_ca * earliest_exit_ca - slot_triangle_ca;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 4;
                candidate_count += 1;
            }
            if latest_entry_ca < earliest_exit_ca
                && latest_entry_ca > 0.0
                && candidate_count < maximum_contact_count
            {
                let point = slot_triangle_ca * latest_entry_ca - slot_triangle_ca;
                let c = &mut candidates_buf[candidate_count];
                c.x = point.dot(slot_triangle_tangent_x);
                c.y = point.dot(slot_triangle_tangent_y);
                c.feature_id = 5;
                candidate_count += 1;
            }

            // Reduce to 4 contacts.
            let mut slot_offset_b = Vec3::ZERO;
            Vector3Wide::read_slot(offset_b, slot_index, &mut slot_offset_b);
            let mut slot_hull_orientation = Matrix3x3::default();
            Matrix3x3Wide::read_slot(&hull_orientation, slot_index, &mut slot_hull_orientation);
            ManifoldCandidateHelper::reduce_scalar(
                candidates_buf.as_mut_ptr(),
                candidate_count as i32,
                slot_face_normal,
                -1.0 / slot_face_normal.dot(slot_local_normal),
                previous_vertex,
                slot_triangle_a,
                slot_triangle_tangent_x,
                slot_triangle_tangent_y,
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

        // Mesh reduction: face collision flag in feature id.
        let face_collision_flag = triangle_normal_dot_local_normal
            .simd_lt(Vector::<f32>::splat(
                -mesh_reduction::MINIMUM_DOT_FOR_FACE_COLLISION,
            ))
            .select(
                Vector::<i32>::splat(mesh_reduction::FACE_COLLISION_FLAG),
                zero_i,
            );
        manifold.feature_id0 += face_collision_flag;
    }
}
