// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CylinderConvexHullTester.cs

use crate::physics::collidables::convex_hull::{ConvexHullSupportFinder, ConvexHullWide};
use crate::physics::collidables::cylinder::{CylinderSupportFinder, CylinderWide};
use crate::physics::collision_detection::collision_tasks::box_cylinder_tester::BoxCylinderTester;
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
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::{Vec2, Vec3};
use std::simd::prelude::*;

/// Pair tester for cylinder vs convex hull collisions.
pub struct CylinderConvexHullTester;

impl CylinderConvexHullTester {
    pub const BATCH_SIZE: i32 = 16;

    /// Projects a point onto a cylinder cap's surface for 2D clipping.
    #[inline(always)]
    fn project_onto_cap(
        cap_center: Vec3,
        cylinder_orientation: &Matrix3x3,
        inverse_local_normal_dot_ay: f32,
        local_normal: Vec3,
        point: Vec3,
        projected: &mut Vec2,
    ) {
        let point_to_cap_center = cap_center - point;
        let t = point_to_cap_center.dot(cylinder_orientation.y) * inverse_local_normal_dot_ay;
        let projection_offset_b = local_normal * t;
        let projected_point = point - projection_offset_b;
        let cap_center_to_projected = projected_point - cap_center;
        *projected = Vec2::new(
            cap_center_to_projected.dot(cylinder_orientation.x),
            cap_center_to_projected.dot(cylinder_orientation.z),
        );
    }

    /// Intersects a 2D line segment with a circle. Returns whether there's an intersection.
    #[inline(always)]
    fn intersect_line_circle(
        line_position: Vec2,
        line_direction: Vec2,
        radius: f32,
        t_min: &mut f32,
        t_max: &mut f32,
    ) -> bool {
        let a = line_direction.dot(line_direction);
        let inverse_a = 1.0 / a;
        let b = line_position.dot(line_direction);
        let c = line_position.dot(line_position) - radius * radius;
        let d = b * b - a * c;
        if d < 0.0 {
            *t_min = 0.0;
            *t_max = 0.0;
            return false;
        }
        let t_offset = d.sqrt() * inverse_a;
        let t_base = -b * inverse_a;
        if a < 1e-12 && a > -1e-12 {
            *t_min = t_base;
            *t_max = t_base;
        } else {
            *t_min = t_base - t_offset;
            *t_max = t_base + t_offset;
        }
        if *t_min < 0.0 {
            *t_min = 0.0;
        }
        if *t_max > 1.0 {
            *t_max = 1.0;
        }
        true
    }

    /// Inserts a contact from the cylinder side edge path.
    #[inline(always)]
    unsafe fn insert_contact(
        slot_side_edge_center: Vec3,
        slot_cylinder_edge_axis: Vec3,
        t: f32,
        hull_face_origin: Vec3,
        slot_hull_face_normal: Vec3,
        inverse_depth_denominator: f32,
        slot_hull_orientation: &Matrix3x3,
        slot_offset_b: Vec3,
        feature_id: i32,
        contact_offset_a_wide: &mut Vector3Wide,
        contact_depth_wide: &mut Vector<f32>,
        contact_feature_id_wide: &mut Vector<i32>,
        contact_exists_wide: &mut Vector<i32>,
    ) {
        let local_point = slot_side_edge_center + slot_cylinder_edge_axis * t;
        let contact_depth =
            (hull_face_origin - local_point).dot(slot_hull_face_normal) * inverse_depth_denominator;
        let mut contact_offset_a = Vec3::ZERO;
        Matrix3x3::transform(&local_point, slot_hull_orientation, &mut contact_offset_a);
        contact_offset_a += slot_offset_b;
        Vector3Wide::write_first(contact_offset_a, contact_offset_a_wide);
        *GatherScatter::get_first_mut(contact_depth_wide) = contact_depth;
        *GatherScatter::get_first_mut(contact_feature_id_wide) = feature_id;
        *GatherScatter::get_first_mut(contact_exists_wide) = -1;
    }

    /// Tests cylinder vs convex hull collision (two orientations).
    #[inline(always)]
    pub unsafe fn test(
        a: &CylinderWide,
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

        let mut cylinder_orientation = Matrix3x3Wide::default();
        let mut hull_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut cylinder_orientation);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut hull_orientation);
        let mut hull_local_cylinder_orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::multiply_by_transpose_without_overlap(
            &cylinder_orientation,
            &hull_orientation,
            &mut hull_local_cylinder_orientation,
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
        let cylinder_support_finder = CylinderSupportFinder;
        let mut inactive_lanes =
            BundleIndexing::create_trailing_mask_for_count_in_bundle(pair_count as usize);
        let mut hull_epsilon_scale = zero_f;
        b.estimate_epsilon_scale(&inactive_lanes, &mut hull_epsilon_scale);
        let epsilon_scale = a
            .half_length
            .simd_max(a.radius)
            .simd_min(hull_epsilon_scale);
        let depth_threshold = -*speculative_margin;

        let mut depth = Vector::<f32>::default();
        let mut local_normal = Vector3Wide::default();
        let mut closest_on_hull = Vector3Wide::default();
        DepthRefiner::find_minimum_depth_with_witness(
            b,
            a,
            &local_offset_a,
            &hull_local_cylinder_orientation,
            &hull_support_finder,
            &cylinder_support_finder,
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
        manifold.contact0_exists = zero_i;
        manifold.contact1_exists = zero_i;
        manifold.contact2_exists = zero_i;
        manifold.contact3_exists = zero_i;
        if inactive_lanes.simd_lt(zero_i).all() {
            return;
        }

        // Identify the cylinder feature.
        let closest_on_cylinder_offset = Vector3Wide::scale(&local_normal, &depth);
        let mut closest_on_cylinder = Vector3Wide::default();
        Vector3Wide::subtract(
            &closest_on_hull,
            &closest_on_cylinder_offset,
            &mut closest_on_cylinder,
        );
        let mut local_normal_in_a = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &local_normal,
            &hull_local_cylinder_orientation,
            &mut local_normal_in_a,
        );
        let inverse_local_normal_dot_cap_normal = one_f / local_normal_in_a.y;
        let use_cap = local_normal_in_a
            .y
            .abs()
            .simd_gt(Vector::<f32>::splat(0.70710678118))
            .to_int();

        let mut cap_center = Vector3Wide::default();
        let mut interior0 = Vector2Wide::default();
        let mut interior1 = Vector2Wide::default();
        let mut interior2 = Vector2Wide::default();
        let mut interior3 = Vector2Wide::default();
        if (use_cap & !inactive_lanes).simd_lt(zero_i).any() {
            let use_bottom = local_normal_in_a.y.simd_gt(zero_f);
            let cap_offset = use_bottom.select(-a.half_length, a.half_length);
            cap_center = Vector3Wide::scale(&hull_local_cylinder_orientation.y, &cap_offset);
            Vector3Wide::add(&cap_center.clone(), &local_offset_a, &mut cap_center);

            let mut hull_local_cylinder_to_closest = Vector3Wide::default();
            Vector3Wide::subtract(
                &closest_on_cylinder,
                &local_offset_a,
                &mut hull_local_cylinder_to_closest,
            );
            let mut cylinder_local_cylinder_to_closest = Vector3Wide::default();
            Matrix3x3Wide::transform_by_transposed_without_overlap(
                &hull_local_cylinder_to_closest,
                &hull_local_cylinder_orientation,
                &mut cylinder_local_cylinder_to_closest,
            );
            BoxCylinderTester::generate_interior_points(
                a,
                &local_normal_in_a,
                &cylinder_local_cylinder_to_closest,
                &mut interior0,
                &mut interior1,
                &mut interior2,
                &mut interior3,
            );
        }

        let mut cylinder_side_edge_center = Vector3Wide::default();
        if (use_cap | inactive_lanes).simd_eq(zero_i).any() {
            let mut cylinder_to_closest = Vector3Wide::default();
            Vector3Wide::subtract(
                &closest_on_cylinder,
                &local_offset_a,
                &mut cylinder_to_closest,
            );
            let mut cylinder_local_closest_y = zero_f;
            Vector3Wide::dot(
                &cylinder_to_closest,
                &hull_local_cylinder_orientation.y,
                &mut cylinder_local_closest_y,
            );
            let edge_center_to_closest =
                Vector3Wide::scale(&hull_local_cylinder_orientation.y, &cylinder_local_closest_y);
            Vector3Wide::subtract(
                &closest_on_cylinder,
                &edge_center_to_closest,
                &mut cylinder_side_edge_center,
            );
        }

        let mut maximum_candidate_count = 4usize;
        for slot_index in 0..pair_count as usize {
            let slot_max = b.hulls[slot_index].face_vertex_indices.len() as usize * 2;
            if slot_max > maximum_candidate_count {
                maximum_candidate_count = slot_max;
            }
        }
        let mut candidates_buf = [ManifoldCandidateScalar::default(); 128];

        let mut slot_offset_indices = Vector::<i32>::splat(0);
        Helpers::fill_vector_with_lane_indices(&mut slot_offset_indices);
        let bounding_plane_epsilon = Vector::<f32>::splat(1e-3) * epsilon_scale;

        for slot_index in 0..pair_count as usize {
            if inactive_lanes.as_array()[slot_index] < 0 {
                continue;
            }
            let hull = &b.hulls[slot_index];
            let mut slot_hull_face_normal = Vec3::ZERO;
            let mut slot_local_normal = Vec3::ZERO;
            let mut best_face_index = 0i32;
            ConvexHullTestHelper::pick_representative_face(
                hull,
                slot_index,
                &local_normal,
                &closest_on_hull,
                &slot_offset_indices,
                &bounding_plane_epsilon,
                &mut slot_hull_face_normal,
                &mut slot_local_normal,
                &mut best_face_index,
            );
            let mut face_start = 0usize;
            let mut face_count = 0usize;
            hull.get_vertex_indices_for_face(
                best_face_index as usize,
                &mut face_start,
                &mut face_count,
            );

            if use_cap.as_array()[slot_index] < 0 {
                // Cap path: clip hull edges against cylinder cap circle.
                let mut candidate_count = 0usize;
                let mut slot_cap_center = Vec3::ZERO;
                Vector3Wide::read_slot(&cap_center, slot_index, &mut slot_cap_center);
                let slot_inverse_local_normal_dot_cap_normal =
                    inverse_local_normal_dot_cap_normal.as_array()[slot_index];

                let interior0_slot =
                    GatherScatter::get_offset_instance(&interior0, slot_index);
                let interior1_slot =
                    GatherScatter::get_offset_instance(&interior1, slot_index);
                let interior2_slot =
                    GatherScatter::get_offset_instance(&interior2, slot_index);
                let interior3_slot =
                    GatherScatter::get_offset_instance(&interior3, slot_index);
                let interior_points_x = [
                    interior0_slot.x[0],
                    interior1_slot.x[0],
                    interior2_slot.x[0],
                    interior3_slot.x[0],
                ];
                let interior_points_y = [
                    interior0_slot.y[0],
                    interior1_slot.y[0],
                    interior2_slot.y[0],
                    interior3_slot.y[0],
                ];
                let slot_radius = a.radius.as_array()[slot_index];
                let mut slot_cylinder_orientation = Matrix3x3::default();
                Matrix3x3Wide::read_slot(
                    &hull_local_cylinder_orientation,
                    slot_index,
                    &mut slot_cylinder_orientation,
                );

                let previous_index_init =
                    hull.face_vertex_indices[face_start + face_count - 1];
                let mut hull_face_origin = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull.points[previous_index_init.bundle_index as usize],
                    previous_index_init.inner_index as usize,
                    &mut hull_face_origin,
                );
                let mut previous_vertex_2d = Vec2::ZERO;
                Self::project_onto_cap(
                    slot_cap_center,
                    &slot_cylinder_orientation,
                    slot_inverse_local_normal_dot_cap_normal,
                    slot_local_normal,
                    hull_face_origin,
                    &mut previous_vertex_2d,
                );
                let mut maximum_interior_containment_dots = [0.0f32; 4];
                let mut previous_index = previous_index_init;

                for i in 0..face_count {
                    let index = hull.face_vertex_indices[face_start + i];
                    let mut hull_vertex = Vec3::ZERO;
                    Vector3Wide::read_slot(
                        &hull.points[index.bundle_index as usize],
                        index.inner_index as usize,
                        &mut hull_vertex,
                    );
                    let mut vertex_2d = Vec2::ZERO;
                    Self::project_onto_cap(
                        slot_cap_center,
                        &slot_cylinder_orientation,
                        slot_inverse_local_normal_dot_cap_normal,
                        slot_local_normal,
                        hull_vertex,
                        &mut vertex_2d,
                    );

                    let hull_edge_offset = vertex_2d - previous_vertex_2d;
                    // Interior point containment test (perp dot product).
                    for k in 0..4 {
                        let mut dot = (interior_points_x[k] - previous_vertex_2d.x)
                            * hull_edge_offset.y
                            - (interior_points_y[k] - previous_vertex_2d.y)
                                * hull_edge_offset.x;
                        // If bottom cap, flip containment sign.
                        if slot_inverse_local_normal_dot_cap_normal > 0.0 {
                            dot = -dot;
                        }
                        if dot > maximum_interior_containment_dots[k] {
                            maximum_interior_containment_dots[k] = dot;
                        }
                    }

                    // Test hull edge against cap circle.
                    let mut t_min = 0.0f32;
                    let mut t_max = 0.0f32;
                    if Self::intersect_line_circle(
                        previous_vertex_2d,
                        hull_edge_offset,
                        slot_radius,
                        &mut t_min,
                        &mut t_max,
                    ) {
                        let start_id = ((previous_index.bundle_index as usize)
                            << BundleIndexing::vector_shift())
                            + previous_index.inner_index as usize;
                        let end_id = ((index.bundle_index as usize)
                            << BundleIndexing::vector_shift())
                            + index.inner_index as usize;
                        let base_feature_id = ((start_id ^ end_id) << 8) as i32;

                        if t_max >= t_min && candidate_count < maximum_candidate_count {
                            let point_2d = hull_edge_offset * t_max + previous_vertex_2d;
                            let c = &mut candidates_buf[candidate_count];
                            c.x = point_2d.x;
                            c.y = point_2d.y;
                            c.feature_id = base_feature_id + end_id as i32;
                            candidate_count += 1;
                        }
                        if t_min < t_max
                            && t_min > 0.0
                            && candidate_count < maximum_candidate_count
                        {
                            let point_2d = hull_edge_offset * t_min + previous_vertex_2d;
                            let c = &mut candidates_buf[candidate_count];
                            c.x = point_2d.x;
                            c.y = point_2d.y;
                            c.feature_id = base_feature_id + start_id as i32;
                            candidate_count += 1;
                        }
                    }

                    previous_index = index;
                    previous_vertex_2d = vertex_2d;
                }

                // Try adding cylinder 'vertex' interior contacts.
                if candidate_count < maximum_candidate_count {
                    if maximum_interior_containment_dots[0] <= 0.0 {
                        let c = &mut candidates_buf[candidate_count];
                        c.x = interior_points_x[0];
                        c.y = interior_points_y[0];
                        c.feature_id = 0;
                        candidate_count += 1;
                    }
                    if candidate_count < maximum_candidate_count
                        && maximum_interior_containment_dots[1] <= 0.0
                    {
                        let c = &mut candidates_buf[candidate_count];
                        c.x = interior_points_x[1];
                        c.y = interior_points_y[1];
                        c.feature_id = 1;
                        candidate_count += 1;
                    }
                    if candidate_count < maximum_candidate_count
                        && maximum_interior_containment_dots[2] <= 0.0
                    {
                        let c = &mut candidates_buf[candidate_count];
                        c.x = interior_points_x[2];
                        c.y = interior_points_y[2];
                        c.feature_id = 2;
                        candidate_count += 1;
                    }
                    if candidate_count < maximum_candidate_count
                        && maximum_interior_containment_dots[3] <= 0.0
                    {
                        let c = &mut candidates_buf[candidate_count];
                        c.x = interior_points_x[3];
                        c.y = interior_points_y[3];
                        c.feature_id = 3;
                        candidate_count += 1;
                    }
                }

                // Reduce contacts on cylinder cap surface, then push back onto hull.
                let mut slot_offset_b = Vec3::ZERO;
                Vector3Wide::read_slot(offset_b, slot_index, &mut slot_offset_b);
                let mut slot_cylinder_face_x = Vec3::ZERO;
                let mut slot_cylinder_face_y = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull_local_cylinder_orientation.x,
                    slot_index,
                    &mut slot_cylinder_face_x,
                );
                Vector3Wide::read_slot(
                    &hull_local_cylinder_orientation.z,
                    slot_index,
                    &mut slot_cylinder_face_y,
                );
                let mut slot_hull_orientation = Matrix3x3::default();
                Matrix3x3Wide::read_slot(&hull_orientation, slot_index, &mut slot_hull_orientation);
                // Note: parameters flipped — working on cap, pushed back onto hull in postpass.
                ManifoldCandidateHelper::reduce_scalar(
                    candidates_buf.as_mut_ptr(),
                    candidate_count as i32,
                    slot_hull_face_normal,
                    -1.0 / slot_local_normal.dot(slot_hull_face_normal),
                    hull_face_origin,
                    slot_cap_center,
                    slot_cylinder_face_x,
                    slot_cylinder_face_y,
                    epsilon_scale.as_array()[slot_index],
                    depth_threshold.as_array()[slot_index],
                    &slot_hull_orientation,
                    slot_offset_b,
                    slot_index,
                    manifold,
                );
            } else {
                // Side edge path: clip cylinder side edge against hull face edges.
                let mut slot_cylinder_edge_axis = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull_local_cylinder_orientation.y,
                    slot_index,
                    &mut slot_cylinder_edge_axis,
                );
                let mut slot_side_edge_center = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &cylinder_side_edge_center,
                    slot_index,
                    &mut slot_side_edge_center,
                );

                let previous_index_init =
                    hull.face_vertex_indices[face_start + face_count - 1];
                let mut hull_face_origin = Vec3::ZERO;
                Vector3Wide::read_slot(
                    &hull.points[previous_index_init.bundle_index as usize],
                    previous_index_init.inner_index as usize,
                    &mut hull_face_origin,
                );
                let mut previous_vertex = hull_face_origin;
                let mut latest_entry_numerator = f32::MAX;
                let mut latest_entry_denominator = -1.0f32;
                let mut earliest_exit_numerator = f32::MAX;
                let mut earliest_exit_denominator = 1.0f32;

                for i in 0..face_count {
                    let index = hull.face_vertex_indices[face_start + i];
                    let mut vertex = Vec3::ZERO;
                    Vector3Wide::read_slot(
                        &hull.points[index.bundle_index as usize],
                        index.inner_index as usize,
                        &mut vertex,
                    );

                    let edge_offset = vertex - previous_vertex;
                    let edge_plane_normal = edge_offset.cross(slot_local_normal);
                    let cylinder_side_to_hull_edge_start =
                        previous_vertex - slot_side_edge_center;
                    let numerator =
                        cylinder_side_to_hull_edge_start.dot(edge_plane_normal);
                    let denominator =
                        edge_plane_normal.dot(slot_cylinder_edge_axis);
                    previous_vertex = vertex;

                    let edge_plane_normal_length_squared =
                        edge_plane_normal.length_squared();
                    let denominator_squared = denominator * denominator;

                    const MIN: f32 = 1e-5;
                    const MAX: f32 = 3e-4;
                    const INVERSE_SPAN: f32 = 1.0 / (MAX - MIN);

                    if denominator_squared > MIN * edge_plane_normal_length_squared {
                        let mut num = numerator;
                        if denominator_squared
                            < MAX * edge_plane_normal_length_squared
                        {
                            let mut restrict_weight = (denominator_squared
                                / edge_plane_normal_length_squared
                                - MIN)
                                * INVERSE_SPAN;
                            if restrict_weight < 0.0 {
                                restrict_weight = 0.0;
                            } else if restrict_weight > 1.0 {
                                restrict_weight = 1.0;
                            }
                            let slot_half_length =
                                a.half_length.as_array()[slot_index];
                            let mut unrestricted_numerator =
                                slot_half_length * denominator;
                            if denominator < 0.0 {
                                unrestricted_numerator = -unrestricted_numerator;
                            }
                            num = restrict_weight * num
                                + (1.0 - restrict_weight) * unrestricted_numerator;
                        }
                        if denominator < 0.0 {
                            if num * latest_entry_denominator
                                > latest_entry_numerator * denominator
                            {
                                latest_entry_numerator = num;
                                latest_entry_denominator = denominator;
                            }
                        } else {
                            if num * earliest_exit_denominator
                                < earliest_exit_numerator * denominator
                            {
                                earliest_exit_numerator = num;
                                earliest_exit_denominator = denominator;
                            }
                        }
                    }
                }

                let slot_side_edge_half_length =
                    a.half_length.as_array()[slot_index];
                let mut latest_entry =
                    latest_entry_numerator / latest_entry_denominator;
                let mut earliest_exit =
                    earliest_exit_numerator / earliest_exit_denominator;
                let inverse_depth_denominator =
                    1.0 / slot_hull_face_normal.dot(slot_local_normal);
                let negated_edge_length = -slot_side_edge_half_length;
                if latest_entry < negated_edge_length {
                    latest_entry = negated_edge_length;
                }
                if latest_entry > slot_side_edge_half_length {
                    latest_entry = slot_side_edge_half_length;
                }
                if earliest_exit < negated_edge_length {
                    earliest_exit = negated_edge_length;
                }
                if earliest_exit > slot_side_edge_half_length {
                    earliest_exit = slot_side_edge_half_length;
                }

                let mut slot_hull_orientation = Matrix3x3::default();
                Matrix3x3Wide::read_slot(
                    &hull_orientation,
                    slot_index,
                    &mut slot_hull_orientation,
                );
                let mut slot_offset_b = Vec3::ZERO;
                Vector3Wide::read_slot(offset_b, slot_index, &mut slot_offset_b);
                // Use pointer arithmetic for per-slot manifold access (Convex4ContactManifoldWide is not Copy).
                let manifold_ptr = manifold as *mut Convex4ContactManifoldWide;
                let slot_manifold = &mut *manifold_ptr.cast::<u8>().add(slot_index * std::mem::size_of::<f32>()).cast::<Convex4ContactManifoldWide>();

                Self::insert_contact(
                    slot_side_edge_center,
                    slot_cylinder_edge_axis,
                    earliest_exit,
                    hull_face_origin,
                    slot_hull_face_normal,
                    inverse_depth_denominator,
                    &slot_hull_orientation,
                    slot_offset_b,
                    0,
                    &mut slot_manifold.offset_a0,
                    &mut slot_manifold.depth0,
                    &mut slot_manifold.feature_id0,
                    &mut slot_manifold.contact0_exists,
                );
                if earliest_exit - latest_entry
                    > slot_side_edge_half_length * 1e-3
                {
                    Self::insert_contact(
                        slot_side_edge_center,
                        slot_cylinder_edge_axis,
                        latest_entry,
                        hull_face_origin,
                        slot_hull_face_normal,
                        inverse_depth_denominator,
                        &slot_hull_orientation,
                        slot_offset_b,
                        1,
                        &mut slot_manifold.offset_a1,
                        &mut slot_manifold.depth1,
                        &mut slot_manifold.feature_id1,
                        &mut slot_manifold.contact1_exists,
                    );
                } else {
                    slot_manifold.contact1_exists.as_mut_array()[0] = 0;
                }
                slot_manifold.contact2_exists.as_mut_array()[0] = 0;
                slot_manifold.contact3_exists.as_mut_array()[0] = 0;
            }
        }

        // Push the manifold onto the hull surface and fill in the normal.
        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &hull_orientation,
            &mut manifold.normal,
        );
        let offset0 = Vector3Wide::scale(&manifold.normal, &manifold.depth0);
        let offset1 = Vector3Wide::scale(&manifold.normal, &manifold.depth1);
        let offset2 = Vector3Wide::scale(&manifold.normal, &manifold.depth2);
        let offset3 = Vector3Wide::scale(&manifold.normal, &manifold.depth3);
        Vector3Wide::add(&manifold.offset_a0.clone(), &offset0, &mut manifold.offset_a0);
        Vector3Wide::add(&manifold.offset_a1.clone(), &offset1, &mut manifold.offset_a1);
        Vector3Wide::add(&manifold.offset_a2.clone(), &offset2, &mut manifold.offset_a2);
        Vector3Wide::add(&manifold.offset_a3.clone(), &offset3, &mut manifold.offset_a3);
    }
}
