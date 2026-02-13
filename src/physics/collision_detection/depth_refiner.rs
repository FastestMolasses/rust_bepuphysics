// Translated from BepuPhysics/CollisionDetection/DepthRefiner.cs
// This is a T4 auto-generated file in the original C# source.
// Contains two parallel implementations: without witness and with witness tracking.

use crate::physics::collision_detection::support_finder::ISupportFinder;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

// ============================================================================
// Without-witness variant
// ============================================================================

/// A vertex in the depth refiner simplex.
#[derive(Clone)]
pub struct Vertex {
    pub support: Vector3Wide,
    pub exists: Vector<i32>,
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            support: Vector3Wide::default(),
            exists: Vector::<i32>::splat(0),
        }
    }
}

/// The simplex used by the depth refiner to track support points.
#[derive(Clone, Default)]
pub struct Simplex {
    pub a: Vertex,
    pub b: Vertex,
    pub c: Vertex,
}

// ============================================================================
// With-witness variant
// ============================================================================

/// A vertex in the depth refiner simplex that also tracks the witness point on shape A.
#[derive(Clone)]
pub struct VertexWithWitness {
    pub support: Vector3Wide,
    pub support_on_a: Vector3Wide,
    pub weight: Vector<f32>,
    pub exists: Vector<i32>,
}

impl Default for VertexWithWitness {
    fn default() -> Self {
        Self {
            support: Vector3Wide::default(),
            support_on_a: Vector3Wide::default(),
            weight: Vector::<f32>::splat(0.0),
            exists: Vector::<i32>::splat(0),
        }
    }
}

/// The simplex used by the depth refiner with witness point tracking.
#[derive(Clone, Default)]
pub struct SimplexWithWitness {
    pub a: VertexWithWitness,
    pub b: VertexWithWitness,
    pub c: VertexWithWitness,
    pub weight_denominator: Vector<f32>,
}

/// Incrementally refines a sample direction to approach a local minimum depth between two convex bodies.
///
/// The DepthRefiner implements a Tootbird search: an incremental algorithm that takes steps towards the Tootbird.
/// The Tootbird is the origin projected on the support plane of the best (lowest depth) support direction observed so far.
/// This uses a simplex that updates with rules similar to a simplified version of GJK.
/// The Tootbird is definitionally not inside the Minkowski sum.
pub struct DepthRefiner;

impl DepthRefiner {
    // ========================================================================
    // FindSupport (without witness)
    // ========================================================================

    /// Computes the support point for the Minkowski difference (A - B) in the given direction.
    #[inline(always)]
    pub fn find_support<TShapeWideA, TShapeWideB, TSupportFinderA, TSupportFinderB>(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        // support(N, A) - support(-N, B)
        let mut support_on_a = Vector3Wide::default();
        TSupportFinderA::compute_local_support(a, direction, terminated_lanes, &mut support_on_a);

        let mut negated_direction = Vector3Wide::default();
        Vector3Wide::negate(direction, &mut negated_direction);

        let mut extreme_b = Vector3Wide::default();
        TSupportFinderB::compute_support(
            b,
            local_orientation_b,
            &negated_direction,
            terminated_lanes,
            &mut extreme_b,
        );
        let mut offset_extreme_b = Vector3Wide::default();
        Vector3Wide::add(&extreme_b, local_offset_b, &mut offset_extreme_b);

        Vector3Wide::subtract(&support_on_a, &offset_extreme_b, support);
    }

    // ========================================================================
    // FindSupport (with witness)
    // ========================================================================

    /// Computes the support point for the Minkowski difference (A - B) and also returns the support on A.
    #[inline(always)]
    pub fn find_support_with_witness<TShapeWideA, TShapeWideB, TSupportFinderA, TSupportFinderB>(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
        support_on_a: &mut Vector3Wide,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        // support(N, A) - support(-N, B)
        TSupportFinderA::compute_local_support(a, direction, terminated_lanes, support_on_a);

        let mut negated_direction = Vector3Wide::default();
        Vector3Wide::negate(direction, &mut negated_direction);

        let mut extreme_b = Vector3Wide::default();
        TSupportFinderB::compute_support(
            b,
            local_orientation_b,
            &negated_direction,
            terminated_lanes,
            &mut extreme_b,
        );
        let mut offset_extreme_b = Vector3Wide::default();
        Vector3Wide::add(&extreme_b, local_offset_b, &mut offset_extreme_b);

        Vector3Wide::subtract(support_on_a, &offset_extreme_b, support);
    }

    // ========================================================================
    // Simplex slot management (without witness)
    // ========================================================================

    /// Fills an empty simplex slot with the given support point.
    #[inline(always)]
    fn fill_slot(vertex: &mut Vertex, support: &Vector3Wide, terminated_lanes: &Vector<i32>) {
        // Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
        // and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
        let dont_fill_slot = vertex.exists | *terminated_lanes;
        vertex.support = Vector3Wide::conditional_select(&dont_fill_slot, &vertex.support, support);
        let neg_one = Vector::<i32>::splat(-1);
        vertex.exists = dont_fill_slot
            .simd_eq(Vector::<i32>::splat(0))
            .select(neg_one, vertex.exists);
    }

    /// Force-fills a simplex slot.
    #[inline(always)]
    fn force_fill_slot(should_fill: &Vector<i32>, vertex: &mut Vertex, support: &Vector3Wide) {
        vertex.exists = vertex.exists | *should_fill;
        vertex.support = Vector3Wide::conditional_select(should_fill, support, &vertex.support);
    }

    // ========================================================================
    // Simplex slot management (with witness)
    // ========================================================================

    /// Fills an empty witness simplex slot with the given support and witness points.
    #[inline(always)]
    fn fill_slot_witness(
        vertex: &mut VertexWithWitness,
        support: &Vector3Wide,
        support_on_a: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
    ) {
        let dont_fill_slot = vertex.exists | *terminated_lanes;
        vertex.support = Vector3Wide::conditional_select(&dont_fill_slot, &vertex.support, support);
        vertex.support_on_a =
            Vector3Wide::conditional_select(&dont_fill_slot, &vertex.support_on_a, support_on_a);
        let neg_one = Vector::<i32>::splat(-1);
        vertex.exists = dont_fill_slot
            .simd_eq(Vector::<i32>::splat(0))
            .select(neg_one, vertex.exists);
    }

    /// Force-fills a witness simplex slot.
    #[inline(always)]
    fn force_fill_slot_witness(
        should_fill: &Vector<i32>,
        vertex: &mut VertexWithWitness,
        support: &Vector3Wide,
        support_on_a: &Vector3Wide,
    ) {
        vertex.exists = vertex.exists | *should_fill;
        vertex.support = Vector3Wide::conditional_select(should_fill, support, &vertex.support);
        vertex.support_on_a =
            Vector3Wide::conditional_select(should_fill, support_on_a, &vertex.support_on_a);
    }

    // ========================================================================
    // Create (without witness)
    // ========================================================================

    /// Creates a simplex from an initial normal and support point.
    #[inline(always)]
    pub fn create(normal: &Vector3Wide, support: &Vector3Wide, simplex: &mut Simplex) {
        // While only one slot is actually full, GetNextNormal expects every slot to have some kind of data-
        // for those slots which are not yet filled, it should be duplicates of other data.
        // (The sub-triangle case is treated the same as the degenerate case.)
        simplex.a.support = support.clone();
        simplex.b.support = support.clone();
        simplex.c.support = support.clone();
        simplex.a.exists = Vector::<i32>::splat(-1);
        simplex.b.exists = Vector::<i32>::splat(0);
        simplex.c.exists = Vector::<i32>::splat(0);
        let _ = normal; // Normal is used implicitly via the caller's context.
    }

    // ========================================================================
    // Create (with witness)
    // ========================================================================

    /// Creates a witness simplex from an initial normal, support, and support-on-A.
    #[inline(always)]
    pub fn create_with_witness(
        normal: &Vector3Wide,
        support: &Vector3Wide,
        support_on_a: &Vector3Wide,
        simplex: &mut SimplexWithWitness,
    ) {
        simplex.a.support = support.clone();
        simplex.b.support = support.clone();
        simplex.c.support = support.clone();
        simplex.a.support_on_a = support_on_a.clone();
        simplex.b.support_on_a = support_on_a.clone();
        simplex.c.support_on_a = support_on_a.clone();
        simplex.a.exists = Vector::<i32>::splat(-1);
        simplex.b.exists = Vector::<i32>::splat(0);
        simplex.c.exists = Vector::<i32>::splat(0);
        let _ = normal;
    }

    // ========================================================================
    // GetNextNormal (without witness)
    // ========================================================================

    /// Computes the next search normal from the simplex state.
    /// When `has_new_support` is true, the provided support is integrated into the simplex.
    /// When false, empty slots are filled with duplicates of existing data.
    fn get_next_normal(
        simplex: &mut Simplex,
        support: &Vector3Wide,
        terminated_lanes: &mut Vector<i32>,
        best_normal: &Vector3Wide,
        best_depth: &Vector<f32>,
        convergence_threshold: &Vector<f32>,
        has_new_support: bool,
        next_normal: &mut Vector3Wide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let zero_i = Vector::<i32>::splat(0);
        let neg_one = Vector::<i32>::splat(-1);

        // The search target is the closest point to the origin on the so-far-best bounding plane (the tootbird).
        let clamped_depth = best_depth.simd_max(zero_f);
        let mut search_target = Vector3Wide::default();
        Vector3Wide::scale_to(best_normal, &clamped_depth, &mut search_target);
        let termination_epsilon = best_depth
            .simd_lt(zero_f)
            .select(*convergence_threshold - *best_depth, *convergence_threshold);
        let termination_epsilon_squared = termination_epsilon * termination_epsilon;

        if has_new_support {
            let simplex_full =
                (simplex.a.exists & (simplex.b.exists & simplex.c.exists)) & !*terminated_lanes;
            // Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
            Self::fill_slot(&mut simplex.a, support, terminated_lanes);
            Self::fill_slot(&mut simplex.b, support, terminated_lanes);
            Self::fill_slot(&mut simplex.c, support, terminated_lanes);

            if simplex_full.simd_lt(zero_i).any() {
                // At least one active lane has a full simplex and an incoming new sample.
                // Choose the subtriangle based on the edge plane tests of AD, BD, and CD, where D is the new support point.
                let mut ab_early = Vector3Wide::default();
                let mut ca_early = Vector3Wide::default();
                Vector3Wide::subtract(&simplex.b.support, &simplex.a.support, &mut ab_early);
                Vector3Wide::subtract(&simplex.a.support, &simplex.c.support, &mut ca_early);
                let mut ad = Vector3Wide::default();
                let mut bd = Vector3Wide::default();
                let mut cd = Vector3Wide::default();
                Vector3Wide::subtract(support, &simplex.a.support, &mut ad);
                Vector3Wide::subtract(support, &simplex.b.support, &mut bd);
                Vector3Wide::subtract(support, &simplex.c.support, &mut cd);
                let mut triangle_normal_early = Vector3Wide::default();
                unsafe {
                    Vector3Wide::cross_without_overlap(
                        &ab_early,
                        &ca_early,
                        &mut triangle_normal_early,
                    );
                }
                // (ad x n) * (d - searchTarget) = (n x (d - searchTarget)) * ad
                let mut target_to_support = Vector3Wide::default();
                Vector3Wide::subtract(support, &search_target, &mut target_to_support);
                let mut nx_offset = Vector3Wide::default();
                unsafe {
                    Vector3Wide::cross_without_overlap(
                        &triangle_normal_early,
                        &target_to_support,
                        &mut nx_offset,
                    );
                }
                let mut ad_plane_test = Vector::<f32>::splat(0.0);
                let mut bd_plane_test = Vector::<f32>::splat(0.0);
                let mut cd_plane_test = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&nx_offset, &ad, &mut ad_plane_test);
                Vector3Wide::dot(&nx_offset, &bd, &mut bd_plane_test);
                Vector3Wide::dot(&nx_offset, &cd, &mut cd_plane_test);

                let use_abd = ad_plane_test.simd_ge(zero_f) & bd_plane_test.simd_lt(zero_f);
                let use_bcd = bd_plane_test.simd_ge(zero_f) & cd_plane_test.simd_lt(zero_f);
                let use_cad = cd_plane_test.simd_ge(zero_f) & ad_plane_test.simd_lt(zero_f);

                // Fallback: if none of the subtriangles contain the best normal, use ABD.
                let none_selected = !(use_abd | use_bcd | use_cad);
                let use_abd = none_selected.select(neg_one, use_abd.to_int());
                let use_bcd = use_bcd.to_int();
                let use_cad = use_cad.to_int();

                Self::force_fill_slot(&(use_bcd & simplex_full), &mut simplex.a, support);
                Self::force_fill_slot(&(use_cad & simplex_full), &mut simplex.b, support);
                Self::force_fill_slot(&(use_abd & simplex_full), &mut simplex.c, support);
            }
        } else {
            let a_support = simplex.a.support.clone();
            Self::fill_slot(&mut simplex.a, &a_support, terminated_lanes);
            Self::fill_slot(&mut simplex.b, &a_support, terminated_lanes);
            Self::fill_slot(&mut simplex.c, &a_support, terminated_lanes);
        }

        let mut ab = Vector3Wide::default();
        let mut ca = Vector3Wide::default();
        let mut bc = Vector3Wide::default();
        Vector3Wide::subtract(&simplex.b.support, &simplex.a.support, &mut ab);
        Vector3Wide::subtract(&simplex.a.support, &simplex.c.support, &mut ca);
        Vector3Wide::subtract(&simplex.c.support, &simplex.b.support, &mut bc);
        let mut triangle_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ab, &ca, &mut triangle_normal);
        }
        let mut triangle_normal_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&triangle_normal, &mut triangle_normal_length_squared);

        // Compute the plane sign tests (barycentric weights, unscaled).
        let mut target_to_a = Vector3Wide::default();
        let mut target_to_c = Vector3Wide::default();
        Vector3Wide::subtract(&simplex.a.support, &search_target, &mut target_to_a);
        Vector3Wide::subtract(&simplex.c.support, &search_target, &mut target_to_c);
        let mut abxta = Vector3Wide::default();
        let mut caxtc = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ab, &target_to_a, &mut abxta);
            Vector3Wide::cross_without_overlap(&ca, &target_to_c, &mut caxtc);
        }
        let mut ab_plane_test = Vector::<f32>::splat(0.0);
        let mut ca_plane_test = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&abxta, &triangle_normal, &mut ab_plane_test);
        Vector3Wide::dot(&caxtc, &triangle_normal, &mut ca_plane_test);
        let bc_plane_test = triangle_normal_length_squared - ca_plane_test - ab_plane_test;
        let outside_ab = ab_plane_test.simd_lt(zero_f);
        let outside_bc = bc_plane_test.simd_lt(zero_f);
        let outside_ca = ca_plane_test.simd_lt(zero_f);

        let mut ab_length_squared = Vector::<f32>::splat(0.0);
        let mut bc_length_squared = Vector::<f32>::splat(0.0);
        let mut ca_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ab, &mut ab_length_squared);
        Vector3Wide::length_squared_to(&bc, &mut bc_length_squared);
        Vector3Wide::length_squared_to(&ca, &mut ca_length_squared);
        let longest_edge_length_squared = ab_length_squared
            .simd_max(bc_length_squared)
            .simd_max(ca_length_squared);
        let simplex_degenerate = triangle_normal_length_squared
            .simd_le(longest_edge_length_squared * Vector::<f32>::splat(1e-10));
        let degeneracy_epsilon = Vector::<f32>::splat(1e-14);
        let simplex_is_a_vertex = longest_edge_length_squared.simd_lt(degeneracy_epsilon);
        let simplex_is_an_edge = simplex_degenerate & !simplex_is_a_vertex;

        let mut calibration_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle_normal, best_normal, &mut calibration_dot);
        Vector3Wide::conditionally_negate(
            &calibration_dot.simd_lt(zero_f).to_int(),
            &mut triangle_normal,
        );

        let target_outside_triangle_edges = outside_ab | outside_bc | outside_ca;

        // Compute the direction from the origin to the closest point on the triangle.
        // If the simplex is degenerate and just a vertex, pick the first simplex entry as representative.
        let mut triangle_to_target = Vector3Wide::default();
        Vector3Wide::negate(&target_to_a, &mut triangle_to_target);

        let one_i = Vector::<i32>::splat(1);
        let two_i = Vector::<i32>::splat(2);
        let four_i = Vector::<i32>::splat(4);
        let mut relevant_features = one_i;

        // If this is a vertex case and the sample is right on top of the target, immediately quit.
        let mut target_to_a_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&target_to_a, &mut target_to_a_length_squared);
        *terminated_lanes = *terminated_lanes
            | (simplex_is_a_vertex.to_int()
                & target_to_a_length_squared
                    .simd_lt(termination_epsilon_squared)
                    .to_int());

        let use_edge =
            (target_outside_triangle_edges | simplex_is_an_edge).to_int() & !*terminated_lanes;
        if use_edge.simd_lt(zero_i).any() {
            // Choose the edge that is closest to the search target.
            let one_f = Vector::<f32>::splat(1.0);
            let inverse_ab_length_squared = one_f / ab_length_squared;
            let inverse_bc_length_squared = one_f / bc_length_squared;
            let inverse_ca_length_squared = one_f / ca_length_squared;
            let mut target_to_b = Vector3Wide::default();
            Vector3Wide::subtract(&simplex.b.support, &search_target, &mut target_to_b);
            let mut oa_dot_ab = Vector::<f32>::splat(0.0);
            let mut ob_dot_bc = Vector::<f32>::splat(0.0);
            let mut oc_dot_ca = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&target_to_a, &ab, &mut oa_dot_ab);
            Vector3Wide::dot(&target_to_b, &bc, &mut ob_dot_bc);
            Vector3Wide::dot(&target_to_c, &ca, &mut oc_dot_ca);
            let ab_scaled_t = zero_f.simd_max(ab_length_squared.simd_min(-oa_dot_ab));
            let bc_scaled_t = zero_f.simd_max(bc_length_squared.simd_min(-ob_dot_bc));
            let ca_scaled_t = zero_f.simd_max(ca_length_squared.simd_min(-oc_dot_ca));
            let ab_t = ab_scaled_t * inverse_ab_length_squared;
            let bc_t = bc_scaled_t * inverse_bc_length_squared;
            let ca_t = ca_scaled_t * inverse_ca_length_squared;
            let ab_scaled_edge_offset = Vector3Wide::scale(&ab, &ab_t);
            let bc_scaled_edge_offset = Vector3Wide::scale(&bc, &bc_t);
            let ca_scaled_edge_offset = Vector3Wide::scale(&ca, &ca_t);
            let mut ab_closest_offset = Vector3Wide::default();
            let mut bc_closest_offset = Vector3Wide::default();
            let mut ca_closest_offset = Vector3Wide::default();
            Vector3Wide::add(&target_to_a, &ab_scaled_edge_offset, &mut ab_closest_offset);
            Vector3Wide::add(&target_to_b, &bc_scaled_edge_offset, &mut bc_closest_offset);
            Vector3Wide::add(&target_to_c, &ca_scaled_edge_offset, &mut ca_closest_offset);
            let ab_distance_squared = Vector3Wide::length_squared_of(&ab_closest_offset);
            let bc_distance_squared = Vector3Wide::length_squared_of(&bc_closest_offset);
            let ca_distance_squared = Vector3Wide::length_squared_of(&ca_closest_offset);

            let bc_degenerate = bc_length_squared.simd_eq(zero_f);
            let ca_degenerate = ca_length_squared.simd_eq(zero_f);
            let ab_closer_than_bc =
                bc_degenerate | ab_distance_squared.simd_lt(bc_distance_squared);
            let ab_closer_than_ca =
                ca_degenerate | ab_distance_squared.simd_lt(ca_distance_squared);
            let bc_closer_than_ca =
                ca_degenerate | bc_distance_squared.simd_lt(ca_distance_squared);

            let use_ab = (ab_closer_than_bc & ab_closer_than_ca).to_int();
            let use_bc = bc_closer_than_ca.to_int() & !use_ab;

            let best_distance_squared = use_ab.simd_ne(zero_i).select(
                ab_distance_squared,
                use_bc
                    .simd_ne(zero_i)
                    .select(bc_distance_squared, ca_distance_squared),
            );

            // If the search target is on the edge, we can immediately quit.
            *terminated_lanes = *terminated_lanes
                | (use_edge
                    & best_distance_squared
                        .simd_le(termination_epsilon_squared)
                        .to_int());

            // NOTE: Wrapping this in a condition under the assumption that we just terminated is a little iffy. Measure.
            if (use_edge & !*terminated_lanes).simd_lt(zero_i).any() {
                let t = use_ab
                    .simd_ne(zero_i)
                    .select(ab_t, use_bc.simd_ne(zero_i).select(bc_t, ca_t));
                let mut edge_offset = Vector3Wide::conditional_select(&use_ab, &ab, &ca);
                let mut edge_start =
                    Vector3Wide::conditional_select(&use_ab, &target_to_a, &target_to_c);
                edge_offset = Vector3Wide::conditional_select(&use_bc, &bc, &edge_offset);
                edge_start = Vector3Wide::conditional_select(&use_bc, &target_to_b, &edge_start);

                let scaled_offset = Vector3Wide::scale(&edge_offset, &(-t));
                let mut triangle_to_target_candidate = Vector3Wide::default();
                Vector3Wide::subtract(
                    &scaled_offset,
                    &edge_start,
                    &mut triangle_to_target_candidate,
                );

                let origin_nearest_start = t.simd_eq(zero_f);
                let origin_nearest_end = t.simd_eq(one_f);
                let three_i = Vector::<i32>::splat(3);
                let five_i = Vector::<i32>::splat(5);
                let six_i = Vector::<i32>::splat(6);
                let feature_for_ab =
                    origin_nearest_start.select(one_i, origin_nearest_end.select(two_i, three_i));
                let feature_for_bc =
                    origin_nearest_start.select(two_i, origin_nearest_end.select(four_i, six_i));
                let feature_for_ca =
                    origin_nearest_start.select(four_i, origin_nearest_end.select(one_i, five_i));
                let edge_features = use_ab.simd_ne(zero_i).select(
                    feature_for_ab,
                    use_bc
                        .simd_ne(zero_i)
                        .select(feature_for_bc, feature_for_ca),
                );
                relevant_features = use_edge
                    .simd_ne(zero_i)
                    .select(edge_features, relevant_features);
                triangle_to_target = Vector3Wide::conditional_select(
                    &use_edge,
                    &triangle_to_target_candidate,
                    &triangle_to_target,
                );
            }
        }

        // We've examined the vertex and edge case, now check the triangle face case.
        let target_contained_in_edge_planes = (!target_outside_triangle_edges.to_int())
            & (!simplex_degenerate.to_int())
            & !*terminated_lanes;
        if target_contained_in_edge_planes.simd_lt(zero_i).any() {
            // At least one lane needs a face test.
            // dot(n, searchTarget - a)^2 / ||n||^2
            let mut target_to_a_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&target_to_a, &triangle_normal, &mut target_to_a_dot);
            let target_on_triangle_surface = (target_to_a_dot * target_to_a_dot)
                .simd_lt(termination_epsilon_squared * triangle_normal_length_squared);
            *terminated_lanes = *terminated_lanes
                | (target_contained_in_edge_planes & target_on_triangle_surface.to_int());
            triangle_to_target = Vector3Wide::conditional_select(
                &target_contained_in_edge_planes,
                &triangle_normal,
                &triangle_to_target,
            );
            let seven_i = Vector::<i32>::splat(7);
            relevant_features = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(seven_i, relevant_features);
        }

        simplex.a.exists = (relevant_features & one_i).simd_gt(zero_i).to_int();
        simplex.b.exists = (relevant_features & two_i).simd_gt(zero_i).to_int();
        simplex.c.exists = (relevant_features & four_i).simd_gt(zero_i).to_int();

        if terminated_lanes.simd_eq(zero_i).any() {
            // In fairly rare cases near penetrating convergence, it's possible for the triangle->target offset
            // to point nearly 90 degrees away from the previous best. Use the offset to tilt the normal.
            let four_f = Vector::<f32>::splat(4.0);
            let push_offset = Vector3Wide::scale(&triangle_to_target, &four_f);
            let mut push_normal_candidate = Vector3Wide::default();
            Vector3Wide::add(&search_target, &push_offset, &mut push_normal_candidate);
            let use_direct = best_depth.simd_le(zero_f).to_int() | target_contained_in_edge_planes;
            triangle_to_target = Vector3Wide::conditional_select(
                &use_direct,
                &triangle_to_target,
                &push_normal_candidate,
            );

            // No active lanes can have a zero length targetToTriangle, so we can normalize safely.
            let length_squared = Vector3Wide::length_squared_of(&triangle_to_target);
            let inv_length = Vector::<f32>::splat(1.0) / StdFloat::sqrt(length_squared);
            Vector3Wide::scale_to(&triangle_to_target, &inv_length, next_normal);
        }
    }

    // ========================================================================
    // GetNextNormal (with witness)
    // ========================================================================

    /// Computes the next search normal from the witness simplex state.
    fn get_next_normal_witness(
        simplex: &mut SimplexWithWitness,
        support: &Vector3Wide,
        support_on_a: &Vector3Wide,
        terminated_lanes: &mut Vector<i32>,
        best_normal: &Vector3Wide,
        best_depth: &Vector<f32>,
        convergence_threshold: &Vector<f32>,
        has_new_support: bool,
        next_normal: &mut Vector3Wide,
    ) {
        let zero_f = Vector::<f32>::splat(0.0);
        let zero_i = Vector::<i32>::splat(0);
        let neg_one = Vector::<i32>::splat(-1);
        let one_f = Vector::<f32>::splat(1.0);

        let clamped_depth = best_depth.simd_max(zero_f);
        let mut search_target = Vector3Wide::default();
        Vector3Wide::scale_to(best_normal, &clamped_depth, &mut search_target);
        let termination_epsilon = best_depth
            .simd_lt(zero_f)
            .select(*convergence_threshold - *best_depth, *convergence_threshold);
        let termination_epsilon_squared = termination_epsilon * termination_epsilon;

        if has_new_support {
            let simplex_full =
                (simplex.a.exists & (simplex.b.exists & simplex.c.exists)) & !*terminated_lanes;
            Self::fill_slot_witness(&mut simplex.a, support, support_on_a, terminated_lanes);
            Self::fill_slot_witness(&mut simplex.b, support, support_on_a, terminated_lanes);
            Self::fill_slot_witness(&mut simplex.c, support, support_on_a, terminated_lanes);

            if simplex_full.simd_lt(zero_i).any() {
                let mut ab_early = Vector3Wide::default();
                let mut ca_early = Vector3Wide::default();
                Vector3Wide::subtract(&simplex.b.support, &simplex.a.support, &mut ab_early);
                Vector3Wide::subtract(&simplex.a.support, &simplex.c.support, &mut ca_early);
                let mut ad = Vector3Wide::default();
                let mut bd = Vector3Wide::default();
                let mut cd = Vector3Wide::default();
                Vector3Wide::subtract(support, &simplex.a.support, &mut ad);
                Vector3Wide::subtract(support, &simplex.b.support, &mut bd);
                Vector3Wide::subtract(support, &simplex.c.support, &mut cd);
                let mut triangle_normal_early = Vector3Wide::default();
                unsafe {
                    Vector3Wide::cross_without_overlap(
                        &ab_early,
                        &ca_early,
                        &mut triangle_normal_early,
                    );
                }
                let mut target_to_support = Vector3Wide::default();
                Vector3Wide::subtract(support, &search_target, &mut target_to_support);
                let mut nx_offset = Vector3Wide::default();
                unsafe {
                    Vector3Wide::cross_without_overlap(
                        &triangle_normal_early,
                        &target_to_support,
                        &mut nx_offset,
                    );
                }
                let mut ad_plane_test = Vector::<f32>::splat(0.0);
                let mut bd_plane_test = Vector::<f32>::splat(0.0);
                let mut cd_plane_test = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&nx_offset, &ad, &mut ad_plane_test);
                Vector3Wide::dot(&nx_offset, &bd, &mut bd_plane_test);
                Vector3Wide::dot(&nx_offset, &cd, &mut cd_plane_test);

                let use_abd = ad_plane_test.simd_ge(zero_f) & bd_plane_test.simd_lt(zero_f);
                let use_bcd = bd_plane_test.simd_ge(zero_f) & cd_plane_test.simd_lt(zero_f);
                let use_cad = cd_plane_test.simd_ge(zero_f) & ad_plane_test.simd_lt(zero_f);

                let none_selected = !(use_abd | use_bcd | use_cad);
                let use_abd = none_selected.select(neg_one, use_abd.to_int());
                let use_bcd = use_bcd.to_int();
                let use_cad = use_cad.to_int();

                Self::force_fill_slot_witness(
                    &(use_bcd & simplex_full),
                    &mut simplex.a,
                    support,
                    support_on_a,
                );
                Self::force_fill_slot_witness(
                    &(use_cad & simplex_full),
                    &mut simplex.b,
                    support,
                    support_on_a,
                );
                Self::force_fill_slot_witness(
                    &(use_abd & simplex_full),
                    &mut simplex.c,
                    support,
                    support_on_a,
                );
            }
        } else {
            let a_support = simplex.a.support.clone();
            let a_support_on_a = simplex.a.support_on_a.clone();
            Self::fill_slot_witness(
                &mut simplex.a,
                &a_support,
                &a_support_on_a,
                terminated_lanes,
            );
            Self::fill_slot_witness(
                &mut simplex.b,
                &a_support,
                &a_support_on_a,
                terminated_lanes,
            );
            Self::fill_slot_witness(
                &mut simplex.c,
                &a_support,
                &a_support_on_a,
                terminated_lanes,
            );
        }

        let mut ab = Vector3Wide::default();
        let mut ca = Vector3Wide::default();
        let mut bc = Vector3Wide::default();
        Vector3Wide::subtract(&simplex.b.support, &simplex.a.support, &mut ab);
        Vector3Wide::subtract(&simplex.a.support, &simplex.c.support, &mut ca);
        Vector3Wide::subtract(&simplex.c.support, &simplex.b.support, &mut bc);
        let mut triangle_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ab, &ca, &mut triangle_normal);
        }
        let mut triangle_normal_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&triangle_normal, &mut triangle_normal_length_squared);

        let mut target_to_a = Vector3Wide::default();
        let mut target_to_c = Vector3Wide::default();
        Vector3Wide::subtract(&simplex.a.support, &search_target, &mut target_to_a);
        Vector3Wide::subtract(&simplex.c.support, &search_target, &mut target_to_c);
        let mut abxta = Vector3Wide::default();
        let mut caxtc = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ab, &target_to_a, &mut abxta);
            Vector3Wide::cross_without_overlap(&ca, &target_to_c, &mut caxtc);
        }
        let mut ab_plane_test = Vector::<f32>::splat(0.0);
        let mut ca_plane_test = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&abxta, &triangle_normal, &mut ab_plane_test);
        Vector3Wide::dot(&caxtc, &triangle_normal, &mut ca_plane_test);
        let bc_plane_test = triangle_normal_length_squared - ca_plane_test - ab_plane_test;
        let outside_ab = ab_plane_test.simd_lt(zero_f);
        let outside_bc = bc_plane_test.simd_lt(zero_f);
        let outside_ca = ca_plane_test.simd_lt(zero_f);

        let mut ab_length_squared = Vector::<f32>::splat(0.0);
        let mut bc_length_squared = Vector::<f32>::splat(0.0);
        let mut ca_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&ab, &mut ab_length_squared);
        Vector3Wide::length_squared_to(&bc, &mut bc_length_squared);
        Vector3Wide::length_squared_to(&ca, &mut ca_length_squared);
        let longest_edge_length_squared = ab_length_squared
            .simd_max(bc_length_squared)
            .simd_max(ca_length_squared);
        let simplex_degenerate = triangle_normal_length_squared
            .simd_le(longest_edge_length_squared * Vector::<f32>::splat(1e-10));
        let degeneracy_epsilon = Vector::<f32>::splat(1e-14);
        let simplex_is_a_vertex = longest_edge_length_squared.simd_lt(degeneracy_epsilon);
        let simplex_is_an_edge = simplex_degenerate & !simplex_is_a_vertex;

        let mut calibration_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&triangle_normal, best_normal, &mut calibration_dot);
        Vector3Wide::conditionally_negate(
            &calibration_dot.simd_lt(zero_f).to_int(),
            &mut triangle_normal,
        );

        let target_outside_triangle_edges = outside_ab | outside_bc | outside_ca;

        let mut triangle_to_target = Vector3Wide::default();
        Vector3Wide::negate(&target_to_a, &mut triangle_to_target);

        let one_i = Vector::<i32>::splat(1);
        let two_i = Vector::<i32>::splat(2);
        let four_i = Vector::<i32>::splat(4);
        let mut relevant_features = one_i;
        simplex.a.weight = terminated_lanes
            .simd_ne(zero_i)
            .select(simplex.a.weight, one_f);
        simplex.b.weight = terminated_lanes
            .simd_ne(zero_i)
            .select(simplex.b.weight, zero_f);
        simplex.c.weight = terminated_lanes
            .simd_ne(zero_i)
            .select(simplex.c.weight, zero_f);
        simplex.weight_denominator = terminated_lanes
            .simd_ne(zero_i)
            .select(simplex.weight_denominator, one_f);

        let mut target_to_a_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&target_to_a, &mut target_to_a_length_squared);
        *terminated_lanes = *terminated_lanes
            | (simplex_is_a_vertex.to_int()
                & target_to_a_length_squared
                    .simd_lt(termination_epsilon_squared)
                    .to_int());

        let use_edge =
            (target_outside_triangle_edges | simplex_is_an_edge).to_int() & !*terminated_lanes;
        if use_edge.simd_lt(zero_i).any() {
            let inverse_ab_length_squared = one_f / ab_length_squared;
            let inverse_bc_length_squared = one_f / bc_length_squared;
            let inverse_ca_length_squared = one_f / ca_length_squared;
            let mut target_to_b = Vector3Wide::default();
            Vector3Wide::subtract(&simplex.b.support, &search_target, &mut target_to_b);
            let mut oa_dot_ab = Vector::<f32>::splat(0.0);
            let mut ob_dot_bc = Vector::<f32>::splat(0.0);
            let mut oc_dot_ca = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&target_to_a, &ab, &mut oa_dot_ab);
            Vector3Wide::dot(&target_to_b, &bc, &mut ob_dot_bc);
            Vector3Wide::dot(&target_to_c, &ca, &mut oc_dot_ca);
            let ab_scaled_t = zero_f.simd_max(ab_length_squared.simd_min(-oa_dot_ab));
            let bc_scaled_t = zero_f.simd_max(bc_length_squared.simd_min(-ob_dot_bc));
            let ca_scaled_t = zero_f.simd_max(ca_length_squared.simd_min(-oc_dot_ca));
            let ab_t = ab_scaled_t * inverse_ab_length_squared;
            let bc_t = bc_scaled_t * inverse_bc_length_squared;
            let ca_t = ca_scaled_t * inverse_ca_length_squared;
            let ab_scaled_edge_offset = Vector3Wide::scale(&ab, &ab_t);
            let bc_scaled_edge_offset = Vector3Wide::scale(&bc, &bc_t);
            let ca_scaled_edge_offset = Vector3Wide::scale(&ca, &ca_t);
            let mut ab_closest_offset = Vector3Wide::default();
            let mut bc_closest_offset = Vector3Wide::default();
            let mut ca_closest_offset = Vector3Wide::default();
            Vector3Wide::add(&target_to_a, &ab_scaled_edge_offset, &mut ab_closest_offset);
            Vector3Wide::add(&target_to_b, &bc_scaled_edge_offset, &mut bc_closest_offset);
            Vector3Wide::add(&target_to_c, &ca_scaled_edge_offset, &mut ca_closest_offset);
            let ab_distance_squared = Vector3Wide::length_squared_of(&ab_closest_offset);
            let bc_distance_squared = Vector3Wide::length_squared_of(&bc_closest_offset);
            let ca_distance_squared = Vector3Wide::length_squared_of(&ca_closest_offset);

            let bc_degenerate = bc_length_squared.simd_eq(zero_f);
            let ca_degenerate = ca_length_squared.simd_eq(zero_f);
            let ab_closer_than_bc =
                bc_degenerate | ab_distance_squared.simd_lt(bc_distance_squared);
            let ab_closer_than_ca =
                ca_degenerate | ab_distance_squared.simd_lt(ca_distance_squared);
            let bc_closer_than_ca =
                ca_degenerate | bc_distance_squared.simd_lt(ca_distance_squared);

            let use_ab = (ab_closer_than_bc & ab_closer_than_ca).to_int();
            let use_bc = bc_closer_than_ca.to_int() & !use_ab;

            let best_distance_squared = use_ab.simd_ne(zero_i).select(
                ab_distance_squared,
                use_bc
                    .simd_ne(zero_i)
                    .select(bc_distance_squared, ca_distance_squared),
            );

            *terminated_lanes = *terminated_lanes
                | (use_edge
                    & best_distance_squared
                        .simd_le(termination_epsilon_squared)
                        .to_int());

            // Note: in the witness variant, the C# doesn't have the conditional guard around this block.
            {
                let t = use_ab
                    .simd_ne(zero_i)
                    .select(ab_t, use_bc.simd_ne(zero_i).select(bc_t, ca_t));
                let mut edge_offset = Vector3Wide::conditional_select(&use_ab, &ab, &ca);
                let mut edge_start =
                    Vector3Wide::conditional_select(&use_ab, &target_to_a, &target_to_c);
                edge_offset = Vector3Wide::conditional_select(&use_bc, &bc, &edge_offset);
                edge_start = Vector3Wide::conditional_select(&use_bc, &target_to_b, &edge_start);

                let scaled_offset = Vector3Wide::scale(&edge_offset, &(-t));
                let mut triangle_to_target_candidate = Vector3Wide::default();
                Vector3Wide::subtract(
                    &scaled_offset,
                    &edge_start,
                    &mut triangle_to_target_candidate,
                );

                let origin_nearest_start = t.simd_eq(zero_f);
                let origin_nearest_end = t.simd_eq(one_f);
                let three_i = Vector::<i32>::splat(3);
                let five_i = Vector::<i32>::splat(5);
                let six_i = Vector::<i32>::splat(6);
                let feature_for_ab =
                    origin_nearest_start.select(one_i, origin_nearest_end.select(two_i, three_i));
                let feature_for_bc =
                    origin_nearest_start.select(two_i, origin_nearest_end.select(four_i, six_i));
                let feature_for_ca =
                    origin_nearest_start.select(four_i, origin_nearest_end.select(one_i, five_i));
                let edge_features = use_ab.simd_ne(zero_i).select(
                    feature_for_ab,
                    use_bc
                        .simd_ne(zero_i)
                        .select(feature_for_bc, feature_for_ca),
                );
                relevant_features = use_edge
                    .simd_ne(zero_i)
                    .select(edge_features, relevant_features);
                triangle_to_target = Vector3Wide::conditional_select(
                    &use_edge,
                    &triangle_to_target_candidate,
                    &triangle_to_target,
                );

                // Weight tracking for witness computation.
                let weight_edge_start = one_f - t;
                simplex.a.weight = use_edge.simd_ne(zero_i).select(
                    use_ab
                        .simd_ne(zero_i)
                        .select(weight_edge_start, use_bc.simd_ne(zero_i).select(zero_f, t)),
                    simplex.a.weight,
                );
                simplex.b.weight = use_edge.simd_ne(zero_i).select(
                    use_ab
                        .simd_ne(zero_i)
                        .select(t, use_bc.simd_ne(zero_i).select(weight_edge_start, zero_f)),
                    simplex.b.weight,
                );
                simplex.c.weight = use_edge.simd_ne(zero_i).select(
                    use_ab
                        .simd_ne(zero_i)
                        .select(zero_f, use_bc.simd_ne(zero_i).select(t, weight_edge_start)),
                    simplex.c.weight,
                );
                // Weight denominator is still just one, as it is in the vertex case.
            }
        }

        let target_contained_in_edge_planes = (!target_outside_triangle_edges.to_int())
            & (!simplex_degenerate.to_int())
            & !*terminated_lanes;
        if target_contained_in_edge_planes.simd_lt(zero_i).any() {
            let mut target_to_a_dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&target_to_a, &triangle_normal, &mut target_to_a_dot);
            let target_on_triangle_surface = (target_to_a_dot * target_to_a_dot)
                .simd_lt(termination_epsilon_squared * triangle_normal_length_squared);
            *terminated_lanes = *terminated_lanes
                | (target_contained_in_edge_planes & target_on_triangle_surface.to_int());
            triangle_to_target = Vector3Wide::conditional_select(
                &target_contained_in_edge_planes,
                &triangle_normal,
                &triangle_to_target,
            );
            let seven_i = Vector::<i32>::splat(7);
            relevant_features = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(seven_i, relevant_features);

            // Weight tracking for witness: use barycentric coordinates from plane tests.
            simplex.a.weight = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(bc_plane_test, simplex.a.weight);
            simplex.b.weight = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(ca_plane_test, simplex.b.weight);
            simplex.c.weight = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(ab_plane_test, simplex.c.weight);
            simplex.weight_denominator = target_contained_in_edge_planes
                .simd_ne(zero_i)
                .select(triangle_normal_length_squared, simplex.weight_denominator);
        }

        simplex.a.exists = (relevant_features & one_i).simd_gt(zero_i).to_int();
        simplex.b.exists = (relevant_features & two_i).simd_gt(zero_i).to_int();
        simplex.c.exists = (relevant_features & four_i).simd_gt(zero_i).to_int();

        if terminated_lanes.simd_eq(zero_i).any() {
            let four_f = Vector::<f32>::splat(4.0);
            let push_offset = Vector3Wide::scale(&triangle_to_target, &four_f);
            let mut push_normal_candidate = Vector3Wide::default();
            Vector3Wide::add(&search_target, &push_offset, &mut push_normal_candidate);
            let use_direct = best_depth.simd_le(zero_f).to_int() | target_contained_in_edge_planes;
            triangle_to_target = Vector3Wide::conditional_select(
                &use_direct,
                &triangle_to_target,
                &push_normal_candidate,
            );

            let length_squared = Vector3Wide::length_squared_of(&triangle_to_target);
            let inv_length = one_f / StdFloat::sqrt(length_squared);
            Vector3Wide::scale_to(&triangle_to_target, &inv_length, next_normal);
        }
    }

    // ========================================================================
    // FindMinimumDepth (without witness) — convenience overload
    // ========================================================================

    /// Finds the minimum depth between two convex shapes along the initial normal direction.
    /// This is the convenience overload that creates the initial simplex internally.
    #[inline(always)]
    pub fn find_minimum_depth<TShapeWideA, TShapeWideB, TSupportFinderA, TSupportFinderB>(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        initial_normal: &Vector3Wide,
        inactive_lanes: &Vector<i32>,
        search_epsilon: &Vector<f32>,
        minimum_depth_threshold: &Vector<f32>,
        depth: &mut Vector<f32>,
        refined_normal: &mut Vector3Wide,
        maximum_iterations: i32,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        let mut initial_support = Vector3Wide::default();
        Self::find_support(
            a,
            b,
            local_offset_b,
            local_orientation_b,
            support_finder_a,
            support_finder_b,
            initial_normal,
            inactive_lanes,
            &mut initial_support,
        );
        let mut initial_depth = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&initial_support, initial_normal, &mut initial_depth);
        let mut simplex = Simplex::default();
        Self::create(initial_normal, &initial_support, &mut simplex);
        Self::find_minimum_depth_with_simplex(
            a,
            b,
            local_offset_b,
            local_orientation_b,
            support_finder_a,
            support_finder_b,
            &mut simplex,
            initial_normal,
            &initial_depth,
            inactive_lanes,
            search_epsilon,
            minimum_depth_threshold,
            depth,
            refined_normal,
            maximum_iterations,
        );
    }

    // ========================================================================
    // FindMinimumDepth (without witness) — with existing simplex
    // ========================================================================

    /// Finds the minimum depth using an existing simplex state.
    pub fn find_minimum_depth_with_simplex<
        TShapeWideA,
        TShapeWideB,
        TSupportFinderA,
        TSupportFinderB,
    >(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        simplex: &mut Simplex,
        initial_normal: &Vector3Wide,
        initial_depth: &Vector<f32>,
        inactive_lanes: &Vector<i32>,
        convergence_threshold: &Vector<f32>,
        minimum_depth_threshold: &Vector<f32>,
        refined_depth: &mut Vector<f32>,
        refined_normal: &mut Vector3Wide,
        maximum_iterations: i32,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        let mut depth_threshold = *minimum_depth_threshold;
        if TSupportFinderA::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderA::get_margin(a, &mut margin);
            depth_threshold = depth_threshold - margin;
        }
        if TSupportFinderB::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderB::get_margin(b, &mut margin);
            depth_threshold = depth_threshold - margin;
        }
        let depth_below_threshold = initial_depth.simd_lt(depth_threshold);
        let mut terminated_lanes = depth_below_threshold.to_int() | *inactive_lanes;

        *refined_normal = initial_normal.clone();
        *refined_depth = *initial_depth;
        if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
            return;
        }

        let dummy = Vector3Wide::default();
        let mut normal = Vector3Wide::default();
        Self::get_next_normal(
            simplex,
            &dummy,
            &mut terminated_lanes,
            refined_normal,
            refined_depth,
            convergence_threshold,
            false,
            &mut normal,
        );

        for _i in 0..maximum_iterations {
            if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
                break;
            }
            let mut support = Vector3Wide::default();
            Self::find_support(
                a,
                b,
                local_offset_b,
                local_orientation_b,
                support_finder_a,
                support_finder_b,
                &normal,
                &terminated_lanes,
                &mut support,
            );
            let mut depth = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&support, &normal, &mut depth);

            let use_new_depth = depth.simd_lt(*refined_depth).to_int() & !terminated_lanes;
            *refined_depth = use_new_depth
                .simd_ne(Vector::<i32>::splat(0))
                .select(depth, *refined_depth);
            *refined_normal =
                Vector3Wide::conditional_select(&use_new_depth, &normal, refined_normal);
            terminated_lanes = terminated_lanes | refined_depth.simd_le(depth_threshold).to_int();
            if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
                break;
            }

            Self::get_next_normal(
                simplex,
                &support,
                &mut terminated_lanes,
                refined_normal,
                refined_depth,
                convergence_threshold,
                true,
                &mut normal,
            );
        }

        if TSupportFinderA::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderA::get_margin(a, &mut margin);
            *refined_depth = *refined_depth + margin;
        }
        if TSupportFinderB::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderB::get_margin(b, &mut margin);
            *refined_depth = *refined_depth + margin;
        }
    }

    // ========================================================================
    // FindMinimumDepth (with witness) — convenience overload
    // ========================================================================

    /// Finds the minimum depth and computes the witness point on shape A.
    #[inline(always)]
    pub fn find_minimum_depth_with_witness<
        TShapeWideA,
        TShapeWideB,
        TSupportFinderA,
        TSupportFinderB,
    >(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        initial_normal: &Vector3Wide,
        inactive_lanes: &Vector<i32>,
        search_epsilon: &Vector<f32>,
        minimum_depth_threshold: &Vector<f32>,
        depth: &mut Vector<f32>,
        refined_normal: &mut Vector3Wide,
        witness_on_a: &mut Vector3Wide,
        maximum_iterations: i32,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        let mut initial_support = Vector3Wide::default();
        let mut initial_support_on_a = Vector3Wide::default();
        Self::find_support_with_witness(
            a,
            b,
            local_offset_b,
            local_orientation_b,
            support_finder_a,
            support_finder_b,
            initial_normal,
            inactive_lanes,
            &mut initial_support,
            &mut initial_support_on_a,
        );
        let mut initial_depth = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&initial_support, initial_normal, &mut initial_depth);
        let mut simplex = SimplexWithWitness::default();
        Self::create_with_witness(
            initial_normal,
            &initial_support,
            &initial_support_on_a,
            &mut simplex,
        );
        Self::find_minimum_depth_with_witness_and_simplex(
            a,
            b,
            local_offset_b,
            local_orientation_b,
            support_finder_a,
            support_finder_b,
            &mut simplex,
            initial_normal,
            &initial_depth,
            inactive_lanes,
            search_epsilon,
            minimum_depth_threshold,
            depth,
            refined_normal,
            witness_on_a,
            maximum_iterations,
        );
    }

    // ========================================================================
    // FindMinimumDepth (with witness) — with existing simplex
    // ========================================================================

    /// Finds the minimum depth using an existing witness simplex, and computes the witness point on A.
    pub fn find_minimum_depth_with_witness_and_simplex<
        TShapeWideA,
        TShapeWideB,
        TSupportFinderA,
        TSupportFinderB,
    >(
        a: &TShapeWideA,
        b: &TShapeWideB,
        local_offset_b: &Vector3Wide,
        local_orientation_b: &Matrix3x3Wide,
        support_finder_a: &TSupportFinderA,
        support_finder_b: &TSupportFinderB,
        simplex: &mut SimplexWithWitness,
        initial_normal: &Vector3Wide,
        initial_depth: &Vector<f32>,
        inactive_lanes: &Vector<i32>,
        convergence_threshold: &Vector<f32>,
        minimum_depth_threshold: &Vector<f32>,
        refined_depth: &mut Vector<f32>,
        refined_normal: &mut Vector3Wide,
        witness_on_a: &mut Vector3Wide,
        maximum_iterations: i32,
    ) where
        TSupportFinderA: ISupportFinder<TShapeWideA>,
        TSupportFinderB: ISupportFinder<TShapeWideB>,
    {
        let zero_f = Vector::<f32>::splat(0.0);

        let mut depth_threshold = *minimum_depth_threshold;
        if TSupportFinderA::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderA::get_margin(a, &mut margin);
            depth_threshold = depth_threshold - margin;
        }
        if TSupportFinderB::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderB::get_margin(b, &mut margin);
            depth_threshold = depth_threshold - margin;
        }
        let depth_below_threshold = initial_depth.simd_lt(depth_threshold);
        let mut terminated_lanes = depth_below_threshold.to_int() | *inactive_lanes;

        *refined_normal = initial_normal.clone();
        *refined_depth = *initial_depth;
        if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
            *witness_on_a = Vector3Wide::default();
            return;
        }

        let dummy = Vector3Wide::default();
        let mut normal = Vector3Wide::default();
        Self::get_next_normal_witness(
            simplex,
            &dummy,
            &dummy,
            &mut terminated_lanes,
            refined_normal,
            refined_depth,
            convergence_threshold,
            false,
            &mut normal,
        );

        for _i in 0..maximum_iterations {
            if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
                break;
            }
            let mut support = Vector3Wide::default();
            let mut support_on_a = Vector3Wide::default();
            Self::find_support_with_witness(
                a,
                b,
                local_offset_b,
                local_orientation_b,
                support_finder_a,
                support_finder_b,
                &normal,
                &terminated_lanes,
                &mut support,
                &mut support_on_a,
            );
            let mut depth = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&support, &normal, &mut depth);

            let use_new_depth = depth.simd_lt(*refined_depth).to_int() & !terminated_lanes;
            *refined_depth = use_new_depth
                .simd_ne(Vector::<i32>::splat(0))
                .select(depth, *refined_depth);
            *refined_normal =
                Vector3Wide::conditional_select(&use_new_depth, &normal, refined_normal);
            terminated_lanes = terminated_lanes | refined_depth.simd_le(depth_threshold).to_int();
            if terminated_lanes.simd_lt(Vector::<i32>::splat(0)).all() {
                break;
            }

            Self::get_next_normal_witness(
                simplex,
                &support,
                &support_on_a,
                &mut terminated_lanes,
                refined_normal,
                refined_depth,
                convergence_threshold,
                true,
                &mut normal,
            );
        }

        if TSupportFinderA::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderA::get_margin(a, &mut margin);
            *refined_depth = *refined_depth + margin;
        }
        if TSupportFinderB::has_margin() {
            let mut margin = Vector::<f32>::splat(0.0);
            TSupportFinderB::get_margin(b, &mut margin);
            *refined_depth = *refined_depth + margin;
        }

        // For simplexes terminating in a triangle state, we deferred the division for converting
        // plane tests to barycentric coordinates until now.
        let one_f = Vector::<f32>::splat(1.0);
        let inverse_denominator = one_f / simplex.weight_denominator;
        let weighted_a = Vector3Wide::scale(
            &simplex.a.support_on_a,
            &(simplex.a.weight * inverse_denominator),
        );
        let weighted_b = Vector3Wide::scale(
            &simplex.b.support_on_a,
            &(simplex.b.weight * inverse_denominator),
        );
        let weighted_c = Vector3Wide::scale(
            &simplex.c.support_on_a,
            &(simplex.c.weight * inverse_denominator),
        );
        Vector3Wide::add(&weighted_a, &weighted_b, witness_on_a);
        let mut temp = Vector3Wide::default();
        Vector3Wide::add(&weighted_c, witness_on_a, &mut temp);
        *witness_on_a = temp;

        if TSupportFinderA::has_margin() {
            let mut margin = zero_f;
            TSupportFinderA::get_margin(a, &mut margin);
            let witness_offset = Vector3Wide::scale(refined_normal, &margin);
            let mut result = Vector3Wide::default();
            Vector3Wide::add(&witness_offset, witness_on_a, &mut result);
            *witness_on_a = result;
        }
    }
}
