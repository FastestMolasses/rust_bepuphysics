// Translated from BepuPhysics/CollisionDetection/SweepTasks/SphereTriangleDistanceTester.cs

use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collidables::triangle::TriangleWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::Mask;

#[derive(Default)]
pub struct SphereTriangleDistanceTester;

impl IPairDistanceTester<SphereWide, TriangleWide> for SphereTriangleDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &SphereWide,
        b: &TriangleWide,
        offset_b: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        // Note that we're borrowing a lot here from the SphereTriangleCollisionTask. Could share more if you find yourself needing to change things dramatically.
        // Main difficulty in fully sharing is that sweep tests do not honor one sidedness, so some of the conditions change.

        // Work in the local space of the triangle, since it's quicker to transform the sphere position than the vertices of the triangle.
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &r_b, &mut local_offset_b);

        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(&b.b, &b.a, &mut ab);
        let mut ac = Vector3Wide::default();
        Vector3Wide::subtract(&b.c, &b.a, &mut ac);
        // localOffsetA = -localOffsetB, so pa = triangle.A + localOffsetB.
        let mut pa = Vector3Wide::default();
        Vector3Wide::add(&b.a, &local_offset_b, &mut pa);
        let mut local_triangle_normal = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ab, &ac, &mut local_triangle_normal);
        }
        let mut pa_n = Vector::<f32>::default();
        Vector3Wide::dot(&local_triangle_normal, &pa, &mut pa_n);

        // EdgeAB plane test: (pa x ab) * (ab x ac) >= 0
        // EdgeAC plane test: (ac x pa) * (ab x ac) >= 0
        // Note that these are scaled versions of the barycentric coordinates.
        // To normalize them such that the weights of a point within the triangle equal 1, we just need to divide by dot(ab x ac, ab x ac).
        // In other words, to test the third edge plane, we can ensure that the unnormalized weights are both positive and sum to a value less than dot(ab x ac, ab x ac).
        // If a point is outside of an edge plane, we know that it's not in the face region or any other edge region. It could, however, be in an adjacent vertex region.
        // Vertex cases can be handled by clamping an edge case.
        // Further, note that for any query location, it is sufficient to only test one edge even if the point is outside two edge planes. If it's outside two edge planes,
        // that just means it's going to be on the shared vertex, so a clamped edge test captures the correct closest point.
        // So, at each edge, if the point is outside the plane, cache the edge. The last edge registering an outside result will be tested.
        let mut paxab = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&pa, &ab, &mut paxab);
        }
        let mut acxpa = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&ac, &pa, &mut acxpa);
        }
        let mut edge_plane_test_ab = Vector::<f32>::default();
        Vector3Wide::dot(&paxab, &local_triangle_normal, &mut edge_plane_test_ab);
        let mut edge_plane_test_ac = Vector::<f32>::default();
        Vector3Wide::dot(&acxpa, &local_triangle_normal, &mut edge_plane_test_ac);
        let mut triangle_normal_length_squared = Vector::<f32>::default();
        Vector3Wide::dot(
            &local_triangle_normal,
            &local_triangle_normal,
            &mut triangle_normal_length_squared,
        );
        let edge_plane_test_bc =
            triangle_normal_length_squared - edge_plane_test_ab - edge_plane_test_ac;

        let zero_f = Vector::<f32>::splat(0.0);
        let outside_ab = edge_plane_test_ab.simd_lt(zero_f).to_int();
        let outside_ac = edge_plane_test_ac.simd_lt(zero_f).to_int();
        let outside_bc = edge_plane_test_bc.simd_lt(zero_f).to_int();

        let outside_any_edge = outside_ab | (outside_ac | outside_bc);
        let mut local_closest_on_triangle = Vector3Wide::default();
        let outside_any = Mask::from_int(outside_any_edge);

        if outside_any.any() {
            // At least one lane detected a point outside of the triangle. Choose one edge which is outside as the representative.
            let mut edge_direction = Vector3Wide::conditional_select(&outside_ac, &ac, &ab);
            let mut bc = Vector3Wide::default();
            Vector3Wide::subtract(&b.c, &b.b, &mut bc);
            edge_direction = Vector3Wide::conditional_select(&outside_bc, &bc, &edge_direction);
            let edge_start = Vector3Wide::conditional_select(&outside_bc, &b.b, &b.a);

            let mut negative_edge_start_to_p = Vector3Wide::default();
            Vector3Wide::add(&local_offset_b, &edge_start, &mut negative_edge_start_to_p);
            // This does some partially redundant work if the edge is AB or AC, but given that we didn't have bcbc or bcpb, it's fine.
            let mut negative_offset_dot_edge = Vector::<f32>::default();
            Vector3Wide::dot(
                &negative_edge_start_to_p,
                &edge_direction,
                &mut negative_offset_dot_edge,
            );
            let mut edge_dot_edge = Vector::<f32>::default();
            Vector3Wide::dot(&edge_direction, &edge_direction, &mut edge_dot_edge);
            let edge_scale = zero_f.simd_max(
                Vector::<f32>::splat(1.0).simd_min(-negative_offset_dot_edge / edge_dot_edge),
            );
            let mut point_on_edge = Vector3Wide::default();
            Vector3Wide::scale_to(&edge_direction, &edge_scale, &mut point_on_edge);
            point_on_edge = point_on_edge + edge_start;

            local_closest_on_triangle = Vector3Wide::conditional_select(
                &outside_any_edge,
                &point_on_edge,
                &local_closest_on_triangle,
            );
        }
        if (!outside_any).any() {
            // p + N * (pa * N) / ||N||^2 = N * (pa * N) / ||N||^2 - (-p)
            let n_scale = pa_n / triangle_normal_length_squared;
            let mut offset_to_plane = Vector3Wide::default();
            Vector3Wide::scale_to(&local_triangle_normal, &n_scale, &mut offset_to_plane);
            let mut point_on_face = Vector3Wide::default();
            Vector3Wide::subtract(&offset_to_plane, &local_offset_b, &mut point_on_face);

            local_closest_on_triangle = Vector3Wide::conditional_select(
                &outside_any_edge,
                &local_closest_on_triangle,
                &point_on_face,
            );
        }

        // normal = normalize(localOffsetA - localClosestOnTriangle)
        //        = (localOffsetB + localClosestOnTriangle) / (-||localOffsetB + localClosestOnTriangle||)
        let mut local_normal = Vector3Wide::default();
        Vector3Wide::add(
            &local_offset_b,
            &local_closest_on_triangle,
            &mut local_normal,
        );
        let local_normal_length = local_normal.length();
        local_normal = Vector3Wide::scale(
            &local_normal,
            &(Vector::<f32>::splat(-1.0) / local_normal_length),
        );
        Matrix3x3Wide::transform_without_overlap(&local_normal, &r_b, normal);
        Vector3Wide::scale_to(&*normal, &(-a.radius), closest_a);
        *distance = local_normal_length - a.radius;
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
    }
}
