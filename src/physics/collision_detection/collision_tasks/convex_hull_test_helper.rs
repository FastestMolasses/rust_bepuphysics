// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexHullTestHelper.cs

use crate::physics::collidables::convex_hull::ConvexHull;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::prelude::*;

/// Helper methods for convex hull face picking during collision testing.
pub struct ConvexHullTestHelper;

impl ConvexHullTestHelper {
    /// Picks the representative face from the set of faces touching the best sampled support point.
    /// The representative face is chosen based on:
    /// 1) the closest hull point is on the face's plane and
    /// 2) of all faces touching the hull point, the face's normal is most aligned with the contact normal.
    #[inline]
    pub fn pick_representative_face(
        hull: &ConvexHull,
        slot_index: usize,
        local_normal: &Vector3Wide,
        closest_on_hull: &Vector3Wide,
        slot_offset_indices: &Vector<i32>,
        bounding_plane_epsilon: &Vector<f32>,
        slot_face_normal: &mut Vec3,
        slot_local_normal: &mut Vec3,
        best_face_index: &mut i32,
    ) {
        Vector3Wide::read_slot(local_normal, slot_index, slot_local_normal);
        let slot_local_normal_bundle = Vector3Wide::broadcast(*slot_local_normal);
        let slot_closest_on_hull = Vector3Wide::rebroadcast(closest_on_hull, slot_index);

        let slot_bounding_plane_epsilon = bounding_plane_epsilon[slot_index];
        let slot_bounding_plane_epsilon_bundle = Vector::<f32>::splat(slot_bounding_plane_epsilon);
        let negated_slot_bounding_plane_epsilon_bundle = -slot_bounding_plane_epsilon_bundle;

        // Start with the first bounding plane
        let first_plane = &hull.bounding_planes[0usize];
        let mut best_face_dot_bundle = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &first_plane.normal,
            &slot_local_normal_bundle,
            &mut best_face_dot_bundle,
        );
        let mut closest_on_hull_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &first_plane.normal,
            &slot_closest_on_hull,
            &mut closest_on_hull_dot,
        );
        let mut best_plane_error_bundle = (closest_on_hull_dot - first_plane.offset).abs();
        let mut best_indices = *slot_offset_indices;

        let vector_shift = BundleIndexing::vector_shift();
        for i in 1..hull.bounding_planes.len() as usize {
            let slot_indices =
                Vector::<i32>::splat((i << vector_shift) as i32) + *slot_offset_indices;
            let bounding_plane = &hull.bounding_planes[i];
            let mut dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&bounding_plane.normal, &slot_local_normal_bundle, &mut dot);
            let mut candidate_dot_hull = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(
                &bounding_plane.normal,
                &slot_closest_on_hull,
                &mut candidate_dot_hull,
            );
            let candidate_error = (candidate_dot_hull - bounding_plane.offset).abs();
            let error_improvement = best_plane_error_bundle - candidate_error;
            // If the plane error improvement is significant, use it.
            // If the plane error improvement is small, then only use the candidate if it has a better aligned normal.
            let use_candidate = error_improvement.simd_ge(slot_bounding_plane_epsilon_bundle)
                | (error_improvement.simd_gt(negated_slot_bounding_plane_epsilon_bundle)
                    & dot.simd_gt(best_face_dot_bundle));
            let use_candidate_i = use_candidate.to_int();
            best_face_dot_bundle = use_candidate.select(dot, best_face_dot_bundle);
            best_plane_error_bundle =
                use_candidate.select(candidate_error, best_plane_error_bundle);
            best_indices = use_candidate_i
                .simd_ne(Vector::<i32>::splat(0))
                .select(slot_indices, best_indices);
        }

        // Horizontal reduction across SIMD lanes
        let mut best_face_dot = best_face_dot_bundle[0];
        let mut best_plane_error = best_plane_error_bundle[0];
        *best_face_index = best_indices[0];
        let negated_slot_bounding_plane_epsilon = -slot_bounding_plane_epsilon;

        for i in 1..Vector::<f32>::LEN {
            let dot = best_face_dot_bundle[i];
            let error = best_plane_error_bundle[i];
            let improvement = best_plane_error - error;
            if improvement >= slot_bounding_plane_epsilon
                || (improvement >= negated_slot_bounding_plane_epsilon && dot > best_face_dot)
            {
                best_face_dot = dot;
                best_plane_error = error;
                *best_face_index = best_indices[i];
            }
        }

        debug_assert!(
            *best_face_index >= 0
                && (*best_face_index as usize) < hull.face_to_vertex_indices_start.len() as usize
        );
        let mut face_bundle_index = 0usize;
        let mut face_inner_index = 0usize;
        BundleIndexing::get_bundle_indices(
            *best_face_index as usize,
            &mut face_bundle_index,
            &mut face_inner_index,
        );
        Vector3Wide::read_slot(
            &hull.bounding_planes[face_bundle_index].normal,
            face_inner_index,
            slot_face_normal,
        );
    }
}
