// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ManifoldCandidateHelper.cs

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex4ContactManifoldWide;
use std::simd::prelude::*;
use glam::Vec3;

/// A scalar manifold candidate (2D position on face + feature ID).
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ManifoldCandidateScalar {
    pub x: f32,
    pub y: f32,
    pub feature_id: i32,
}

/// A SIMD-wide manifold candidate.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ManifoldCandidate {
    pub x: Vector<f32>,
    pub y: Vector<f32>,
    pub depth: Vector<f32>,
    pub feature_id: Vector<i32>,
}

impl Default for ManifoldCandidate {
    fn default() -> Self {
        Self {
            x: Vector::<f32>::splat(0.0),
            y: Vector::<f32>::splat(0.0),
            depth: Vector::<f32>::splat(0.0),
            feature_id: Vector::<i32>::splat(0),
        }
    }
}

pub struct ManifoldCandidateHelper;

impl ManifoldCandidateHelper {
    /// Adds a candidate contact to the SIMD-wide candidate list.
    ///
    /// Incrementally maintaining a list is a poor fit for SIMD—each pair has its own count,
    /// so scatter is emulated with scalar operations per lane.
    #[inline(always)]
    pub unsafe fn add_candidate(
        candidates: *mut ManifoldCandidate,
        count: &mut Vector<i32>,
        candidate: &ManifoldCandidate,
        new_contact_exists: &Vector<i32>,
        pair_count: i32,
    ) {
        for i in 0..pair_count as usize {
            if new_contact_exists[i] < 0 {
                let target_index = count[i] as usize;
                let target = &mut *candidates.add(target_index);
                target.x.as_mut_array()[i] = candidate.x[i];
                target.y.as_mut_array()[i] = candidate.y[i];
                target.feature_id.as_mut_array()[i] = candidate.feature_id[i];
            }
        }
        let ones = Vector::<i32>::splat(1);
        *count = new_contact_exists.simd_lt(Vector::<i32>::splat(0))
            .select(*count + ones, *count);
    }

    /// Adds a candidate contact with depth included.
    #[inline(always)]
    pub unsafe fn add_candidate_with_depth(
        candidates: *mut ManifoldCandidate,
        count: &mut Vector<i32>,
        candidate: &ManifoldCandidate,
        new_contact_exists: &Vector<i32>,
        pair_count: i32,
    ) {
        for i in 0..pair_count as usize {
            if new_contact_exists[i] < 0 {
                let target_index = count[i] as usize;
                let target = &mut *candidates.add(target_index);
                target.x.as_mut_array()[i] = candidate.x[i];
                target.y.as_mut_array()[i] = candidate.y[i];
                target.depth.as_mut_array()[i] = candidate.depth[i];
                target.feature_id.as_mut_array()[i] = candidate.feature_id[i];
            }
        }
        let ones = Vector::<i32>::splat(1);
        *count = new_contact_exists.simd_lt(Vector::<i32>::splat(0))
            .select(*count + ones, *count);
    }

    #[inline(always)]
    fn conditional_select(
        use_a: &Vector<i32>,
        a: &ManifoldCandidate,
        b: &ManifoldCandidate,
    ) -> ManifoldCandidate {
        let mask_f = use_a.simd_lt(Vector::<i32>::splat(0));
        let mask_i = use_a.simd_lt(Vector::<i32>::splat(0));
        ManifoldCandidate {
            x: mask_f.select(a.x, b.x),
            y: mask_f.select(a.y, b.y),
            depth: mask_f.select(a.depth, b.depth),
            feature_id: mask_i.select(a.feature_id, b.feature_id),
        }
    }

    #[inline(always)]
    unsafe fn candidate_exists(
        candidate: &ManifoldCandidate,
        minimum_depth: &Vector<f32>,
        raw_contact_count: &Vector<i32>,
        i: i32,
        exists: &mut Vector<i32>,
    ) {
        let i_vec = Vector::<i32>::splat(i);
        let depth_ok = candidate.depth.simd_gt(*minimum_depth);
        let index_ok = i_vec.simd_lt(*raw_contact_count);
        let combined = depth_ok & index_ok;
        *exists = combined.select(Vector::<i32>::splat(-1), Vector::<i32>::splat(0));
    }

    /// Reduces a set of manifold candidates to at most 4 contacts.
    ///
    /// Strategy:
    /// 1. Find the deepest contact (+ extremity bias)
    /// 2. Find the most distant contact from the first
    /// 3. Find the contacts with the largest positive and negative signed area with the first edge
    pub unsafe fn reduce(
        candidates: *mut ManifoldCandidate,
        raw_contact_count: Vector<i32>,
        max_candidate_count: i32,
        face_normal_a: &Vector3Wide,
        inverse_face_normal_dot_normal: Vector<f32>,
        face_center_b_to_face_center_a: &Vector3Wide,
        tangent_bx: &Vector3Wide,
        tangent_by: &Vector3Wide,
        epsilon_scale: Vector<f32>,
        minimum_depth: Vector<f32>,
        pair_count: i32,
        contact0: &mut ManifoldCandidate,
        contact1: &mut ManifoldCandidate,
        contact2: &mut ManifoldCandidate,
        contact3: &mut ManifoldCandidate,
        contact0_exists: &mut Vector<i32>,
        contact1_exists: &mut Vector<i32>,
        contact2_exists: &mut Vector<i32>,
        contact3_exists: &mut Vector<i32>,
    ) {
        let mut max_count = max_candidate_count;
        let mut masked_contact_count = raw_contact_count;

        // Mask out any contacts generated on pairs which don't actually exist.
        for i in pair_count as usize..std::mem::size_of::<Vector<i32>>() / 4 {
            masked_contact_count.as_mut_array()[i] = 0;
        }
        for i in (0..=max_count).rev() {
            if (masked_contact_count.simd_eq(Vector::<i32>::splat(i))).any() {
                max_count = i;
                break;
            }
        }

        // Compute depths for reduction
        let dot_axis = Vector3Wide::scale(face_normal_a, &inverse_face_normal_dot_normal);
        let mut negative_base_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(face_center_b_to_face_center_a, &dot_axis, &mut negative_base_dot);
        let mut x_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(tangent_bx, &dot_axis, &mut x_dot);
        let mut y_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(tangent_by, &dot_axis, &mut y_dot);

        for i in 0..max_count as usize {
            let candidate = &mut *candidates.add(i);
            candidate.depth = candidate.x * x_dot + candidate.y * y_dot - negative_base_dot;
        }

        // See if we can compress the count due to depth-rejected candidates.
        for i in (0..max_count as usize).rev() {
            let candidate = &*candidates.add(i);
            let mut contact_exists = Vector::<i32>::splat(0);
            Self::candidate_exists(candidate, &minimum_depth, &masked_contact_count, i as i32, &mut contact_exists);
            if (contact_exists.simd_eq(Vector::<i32>::splat(-i32::MAX))).any() {
                max_count = i as i32 + 1;
                break;
            }
        }

        Self::internal_reduce(
            candidates,
            max_count,
            &epsilon_scale,
            &minimum_depth,
            &masked_contact_count,
            contact0,
            contact1,
            contact2,
            contact3,
            contact0_exists,
            contact1_exists,
            contact2_exists,
            contact3_exists,
        );
    }

    /// Reduces candidates without computing depths first (depths already set on candidates).
    /// Used by triangle-triangle tester where depths are computed inline during clipping.
    pub unsafe fn reduce_without_computing_depths(
        candidates: *mut ManifoldCandidate,
        raw_contact_count: Vector<i32>,
        max_candidate_count: i32,
        epsilon_scale: Vector<f32>,
        minimum_depth: Vector<f32>,
        pair_count: i32,
        contact0: &mut ManifoldCandidate,
        contact1: &mut ManifoldCandidate,
        contact2: &mut ManifoldCandidate,
        contact3: &mut ManifoldCandidate,
        contact0_exists: &mut Vector<i32>,
        contact1_exists: &mut Vector<i32>,
        contact2_exists: &mut Vector<i32>,
        contact3_exists: &mut Vector<i32>,
    ) {
        let mut max_count = max_candidate_count;
        let mut masked_contact_count = raw_contact_count;

        // Mask out any contacts generated on pairs which don't actually exist.
        for i in pair_count as usize..std::mem::size_of::<Vector<i32>>() / 4 {
            masked_contact_count.as_mut_array()[i] = 0;
        }
        for i in (0..=max_count).rev() {
            if (masked_contact_count.simd_eq(Vector::<i32>::splat(i))).any() {
                max_count = i;
                break;
            }
        }

        Self::internal_reduce(
            candidates,
            max_count,
            &epsilon_scale,
            &minimum_depth,
            &masked_contact_count,
            contact0,
            contact1,
            contact2,
            contact3,
            contact0_exists,
            contact1_exists,
            contact2_exists,
            contact3_exists,
        );
    }

    /// Internal reduction: picks up to 4 contacts from candidates using deepest + spread heuristic.
    unsafe fn internal_reduce(
        candidates: *mut ManifoldCandidate,
        max_candidate_count: i32,
        epsilon_scale: &Vector<f32>,
        minimum_depth: &Vector<f32>,
        masked_contact_count: &Vector<i32>,
        contact0: &mut ManifoldCandidate,
        contact1: &mut ManifoldCandidate,
        contact2: &mut ManifoldCandidate,
        contact3: &mut ManifoldCandidate,
        contact0_exists: &mut Vector<i32>,
        contact1_exists: &mut Vector<i32>,
        contact2_exists: &mut Vector<i32>,
        contact3_exists: &mut Vector<i32>,
    ) {
        if max_candidate_count == 0 {
            *contact0_exists = Vector::<i32>::splat(0);
            *contact1_exists = Vector::<i32>::splat(0);
            *contact2_exists = Vector::<i32>::splat(0);
            *contact3_exists = Vector::<i32>::splat(0);
            return;
        }

        // 1. Find deepest contact with extremity bias → contact0
        let mut best_score = Vector::<f32>::splat(-f32::MAX);
        let extremity_scale = *epsilon_scale * Vector::<f32>::splat(1e-2);
        for i in 0..max_candidate_count {
            let candidate = &*candidates.add(i as usize);
            let mut exists = Vector::<i32>::splat(0);
            Self::candidate_exists(candidate, minimum_depth, masked_contact_count, i, &mut exists);
            let extremity = (candidate.x * Vector::<f32>::splat(0.7946897654)
                + candidate.y * Vector::<f32>::splat(0.60701579614))
                .abs();
            let candidate_score = candidate.depth
                + candidate
                    .depth
                    .simd_ge(Vector::<f32>::splat(0.0))
                    .select(extremity * extremity_scale, Vector::<f32>::splat(0.0));
            let candidate_is_best = (exists.simd_lt(Vector::<i32>::splat(0)))
                & (candidate_score.simd_gt(best_score));
            *contact0 = Self::conditional_select(
                &candidate_is_best.to_int(),
                candidate,
                contact0,
            );
            best_score = candidate_is_best.select(candidate_score, best_score);
        }
        *contact0_exists = best_score
            .simd_gt(Vector::<f32>::splat(-f32::MAX))
            .to_int();

        // 2. Find most distant from contact0 → contact1
        let mut max_distance_squared = Vector::<f32>::splat(0.0);
        for i in 0..max_candidate_count {
            let candidate = &*candidates.add(i as usize);
            let offset_x = candidate.x - contact0.x;
            let offset_y = candidate.y - contact0.y;
            let distance_squared = offset_x * offset_x + offset_y * offset_y;
            let mut exists = Vector::<i32>::splat(0);
            Self::candidate_exists(candidate, minimum_depth, masked_contact_count, i, &mut exists);
            let candidate_is_most_distant = (distance_squared.simd_gt(max_distance_squared))
                & (exists.simd_lt(Vector::<i32>::splat(0)));
            max_distance_squared =
                candidate_is_most_distant.select(distance_squared, max_distance_squared);
            *contact1 = Self::conditional_select(
                &candidate_is_most_distant.to_int(),
                candidate,
                contact1,
            );
        }
        *contact1_exists = max_distance_squared
            .simd_gt(*epsilon_scale * *epsilon_scale * Vector::<f32>::splat(1e-6))
            .to_int();

        // 3. Find contacts with largest positive and negative signed area → contact2, contact3
        let edge_offset_x = contact1.x - contact0.x;
        let edge_offset_y = contact1.y - contact0.y;
        let mut min_signed_area = Vector::<f32>::splat(0.0);
        let mut max_signed_area = Vector::<f32>::splat(0.0);
        for i in 0..max_candidate_count {
            let candidate = &*candidates.add(i as usize);
            let candidate_offset_x = candidate.x - contact0.x;
            let candidate_offset_y = candidate.y - contact0.y;
            let mut signed_area =
                candidate_offset_x * edge_offset_y - candidate_offset_y * edge_offset_x;
            // Penalize speculative contacts
            signed_area = candidate
                .depth
                .simd_lt(Vector::<f32>::splat(0.0))
                .select(
                    Vector::<f32>::splat(0.25) * signed_area,
                    signed_area,
                );
            let mut exists = Vector::<i32>::splat(0);
            Self::candidate_exists(candidate, minimum_depth, masked_contact_count, i, &mut exists);
            let is_min_area = (signed_area.simd_lt(min_signed_area))
                & (exists.simd_lt(Vector::<i32>::splat(0)));
            min_signed_area = is_min_area.select(signed_area, min_signed_area);
            *contact2 =
                Self::conditional_select(&is_min_area.to_int(), candidate, contact2);
            let is_max_area = (signed_area.simd_gt(max_signed_area))
                & (exists.simd_lt(Vector::<i32>::splat(0)));
            max_signed_area = is_max_area.select(signed_area, max_signed_area);
            *contact3 =
                Self::conditional_select(&is_max_area.to_int(), candidate, contact3);
        }
        let epsilon =
            max_distance_squared * max_distance_squared * Vector::<f32>::splat(1e-6);
        *contact2_exists = (min_signed_area * min_signed_area)
            .simd_gt(epsilon)
            .to_int();
        *contact3_exists = (max_signed_area * max_signed_area)
            .simd_gt(epsilon)
            .to_int();
    }

    #[inline(always)]
    unsafe fn place_candidate_in_slot(
        candidate: &ManifoldCandidateScalar,
        contact_index: usize,
        face_center_b: Vec3,
        face_bx: Vec3,
        face_by: Vec3,
        depth: f32,
        orientation_b: &Matrix3x3,
        offset_b: Vec3,
        slot_index: usize,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        let local_position = face_bx * candidate.x + face_by * candidate.y + face_center_b;
        let mut position = Vec3::ZERO;
        Matrix3x3::transform(&local_position, orientation_b, &mut position);
        position += offset_b;
        let offset_a = match contact_index {
            0 => &mut manifold.offset_a0,
            1 => &mut manifold.offset_a1,
            2 => &mut manifold.offset_a2,
            3 => &mut manifold.offset_a3,
            _ => unreachable!(),
        };
        Vector3Wide::write_slot(position, slot_index, offset_a);
        let depth_field = match contact_index {
            0 => &mut manifold.depth0,
            1 => &mut manifold.depth1,
            2 => &mut manifold.depth2,
            3 => &mut manifold.depth3,
            _ => unreachable!(),
        };
        depth_field.as_mut_array()[slot_index] = depth;
        let feature_id_field = match contact_index {
            0 => &mut manifold.feature_id0,
            1 => &mut manifold.feature_id1,
            2 => &mut manifold.feature_id2,
            3 => &mut manifold.feature_id3,
            _ => unreachable!(),
        };
        feature_id_field.as_mut_array()[slot_index] = candidate.feature_id;
        let exists_field = match contact_index {
            0 => &mut manifold.contact0_exists,
            1 => &mut manifold.contact1_exists,
            2 => &mut manifold.contact2_exists,
            3 => &mut manifold.contact3_exists,
            _ => unreachable!(),
        };
        exists_field.as_mut_array()[slot_index] = -1;
    }

    #[inline(always)]
    unsafe fn remove_candidate_at(
        candidates: *mut ManifoldCandidateScalar,
        depths: *mut f32,
        removal_index: usize,
        candidate_count: &mut i32,
    ) {
        let last_index = *candidate_count as usize - 1;
        if removal_index < last_index {
            *candidates.add(removal_index) = *candidates.add(last_index);
            *depths.add(removal_index) = *depths.add(last_index);
        }
        *candidate_count -= 1;
    }

    /// Scalar per-slot contact reduction for convex hull testers.
    /// Selects up to 4 contacts from a list of candidates using depth + extremity heuristics.
    /// Does NOT assign the contact normal — caller must set `manifold.normal` after.
    pub unsafe fn reduce_scalar(
        candidates: *mut ManifoldCandidateScalar,
        mut candidate_count: i32,
        face_normal_a: Vec3,
        inverse_face_normal_a_dot_local_normal: f32,
        face_center_a: Vec3,
        face_center_b: Vec3,
        tangent_bx: Vec3,
        tangent_by: Vec3,
        epsilon_scale: f32,
        minimum_depth: f32,
        rotation_to_world: &Matrix3x3,
        world_offset_b: Vec3,
        slot_index: usize,
        manifold: &mut Convex4ContactManifoldWide,
    ) {
        if candidate_count == 0 {
            return;
        }

        // Calculate depths and prune below threshold.
        let dot_axis = face_normal_a * inverse_face_normal_a_dot_local_normal;
        let face_center_a_to_face_center_b = face_center_b - face_center_a;
        let base_dot = face_center_a_to_face_center_b.dot(dot_axis);
        let x_dot = tangent_bx.dot(dot_axis);
        let y_dot = tangent_by.dot(dot_axis);
        // Use a fixed-size buffer for depths - convex hulls have bounded face vertex counts.
        let mut candidate_depths_buf = [0.0f32; 128];
        let candidate_depths = candidate_depths_buf.as_mut_ptr();
        for i in (0..candidate_count as usize).rev() {
            let c = &*candidates.add(i);
            let d = base_dot + c.x * x_dot + c.y * y_dot;
            *candidate_depths.add(i) = d;
            if d < minimum_depth {
                Self::remove_candidate_at(candidates, candidate_depths, i, &mut candidate_count);
            }
        }
        if candidate_count <= 4 {
            for i in 0..candidate_count as usize {
                Self::place_candidate_in_slot(
                    &*candidates.add(i), i, face_center_b, tangent_bx, tangent_by,
                    *candidate_depths.add(i), rotation_to_world, world_offset_b, slot_index, manifold,
                );
            }
            return;
        }

        // Find deepest contact (contact 0).
        let mut best_score0 = f32::MIN;
        let mut best_index0 = 0usize;
        let extremity_scale = epsilon_scale * 1e-2;
        let extremity_x = 0.7946897654 * extremity_scale;
        let extremity_y = 0.60701579614 * extremity_scale;
        for i in 0..candidate_count as usize {
            let c = &*candidates.add(i);
            let d = *candidate_depths.add(i);
            let mut candidate_score = d;
            if d >= 0.0 {
                let mut extremity = c.x * extremity_x + c.y * extremity_y;
                if extremity < 0.0 {
                    extremity = -extremity;
                }
                candidate_score += extremity;
            }
            if candidate_score > best_score0 {
                best_score0 = candidate_score;
                best_index0 = i;
            }
        }
        let candidate0 = *candidates.add(best_index0);
        let depth0 = *candidate_depths.add(best_index0);
        Self::place_candidate_in_slot(
            &candidate0, 0, face_center_b, tangent_bx, tangent_by,
            depth0, rotation_to_world, world_offset_b, slot_index, manifold,
        );
        Self::remove_candidate_at(candidates, candidate_depths, best_index0, &mut candidate_count);

        // Find most distant contact (contact 1).
        let mut maximum_distance_squared = -1.0f32;
        let mut best_index1 = 0usize;
        for i in 0..candidate_count as usize {
            let c = &*candidates.add(i);
            let offset_x = c.x - candidate0.x;
            let offset_y = c.y - candidate0.y;
            let distance_squared = offset_x * offset_x + offset_y * offset_y;
            if distance_squared > maximum_distance_squared {
                maximum_distance_squared = distance_squared;
                best_index1 = i;
            }
        }
        if maximum_distance_squared < 1e-6 * epsilon_scale * epsilon_scale {
            return;
        }
        let candidate1 = *candidates.add(best_index1);
        let depth1 = *candidate_depths.add(best_index1);
        Self::place_candidate_in_slot(
            &candidate1, 1, face_center_b, tangent_bx, tangent_by,
            depth1, rotation_to_world, world_offset_b, slot_index, manifold,
        );
        Self::remove_candidate_at(candidates, candidate_depths, best_index1, &mut candidate_count);

        // Find two more contacts with largest signed areas (contacts 2 & 3).
        let edge_offset_x = candidate1.x - candidate0.x;
        let edge_offset_y = candidate1.y - candidate0.y;
        let mut min_signed_area = 0.0f32;
        let mut max_signed_area = 0.0f32;
        let mut best_index2 = 0usize;
        let mut best_index3 = 0usize;
        for i in 0..candidate_count as usize {
            let c = &*candidates.add(i);
            let candidate_offset_x = c.x - candidate0.x;
            let candidate_offset_y = c.y - candidate0.y;
            let mut signed_area =
                candidate_offset_x * edge_offset_y - candidate_offset_y * edge_offset_x;
            if *candidate_depths.add(i) < 0.0 {
                signed_area *= 0.25;
            }
            if signed_area < min_signed_area {
                min_signed_area = signed_area;
                best_index2 = i;
            }
            if signed_area > max_signed_area {
                max_signed_area = signed_area;
                best_index3 = i;
            }
        }

        let area_epsilon = maximum_distance_squared * maximum_distance_squared * 1e-6;
        if min_signed_area * min_signed_area > area_epsilon {
            Self::place_candidate_in_slot(
                &*candidates.add(best_index2), 2, face_center_b, tangent_bx, tangent_by,
                *candidate_depths.add(best_index2), rotation_to_world, world_offset_b, slot_index, manifold,
            );
        }
        if max_signed_area * max_signed_area > area_epsilon {
            Self::place_candidate_in_slot(
                &*candidates.add(best_index3), 3, face_center_b, tangent_bx, tangent_by,
                *candidate_depths.add(best_index3), rotation_to_world, world_offset_b, slot_index, manifold,
            );
        }
    }
}
