// Translated from BepuPhysics/CollisionDetection/SweepTasks/CapsuleBoxDistanceTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

#[derive(Default)]
pub struct CapsuleBoxDistanceTester;

#[inline(always)]
fn test_box_edge(
    offset_ax: &Vector<f32>,
    offset_ay: &Vector<f32>,
    offset_az: &Vector<f32>,
    capsule_axis_x: &Vector<f32>,
    capsule_axis_y: &Vector<f32>,
    _capsule_axis_z: &Vector<f32>,
    box_half_width: &Vector<f32>,
    box_half_height: &Vector<f32>,
    box_half_length: &Vector<f32>,
    depth: &mut Vector<f32>,
    nx: &mut Vector<f32>,
    ny: &mut Vector<f32>,
    nz: &mut Vector<f32>,
) {
    // Assume edge Z is being tested. Input will be swizzled to match.
    let length = (*capsule_axis_x * *capsule_axis_x + *capsule_axis_y * *capsule_axis_y).sqrt();
    let inverse_length = Vector::<f32>::splat(1.0) / length;
    let use_fallback_normal = length.simd_lt(Vector::<f32>::splat(1e-7));
    *nx = use_fallback_normal.select(Vector::<f32>::splat(1.0), -*capsule_axis_y * inverse_length);
    *ny = use_fallback_normal.select(Vector::<f32>::splat(0.0), *capsule_axis_x * inverse_length);
    *nz = Vector::<f32>::splat(0.0);

    // The normal is perpendicular to the capsule axis, so the capsule's extent is 0 centered on offsetA * N.
    *depth = nx.abs() * *box_half_width + ny.abs() * *box_half_height + nz.abs() * *box_half_length
        - (*offset_ax * *nx + *offset_ay * *ny + *offset_az * *nz).abs();
}

#[inline(always)]
fn get_edge_closest_point(
    normal: &mut Vector3Wide,
    edge_direction_index: &Vector<i32>,
    b: &BoxWide,
    offset_a: &Vector3Wide,
    capsule_axis: &Vector3Wide,
    capsule_half_length: &Vector<f32>,
    closest_point_from_edge: &mut Vector3Wide,
) {
    let mut calibration_dot = Vector::<f32>::default();
    Vector3Wide::dot(normal, offset_a, &mut calibration_dot);
    let flip_normal = calibration_dot.simd_lt(Vector::<f32>::splat(0.0));
    normal.x = flip_normal.select(-normal.x, normal.x);
    normal.y = flip_normal.select(-normal.y, normal.y);
    normal.z = flip_normal.select(-normal.z, normal.z);

    let zero = Vector::<f32>::splat(0.0);
    let is_x_zero = normal.x.simd_eq(zero);
    let is_y_zero = normal.y.simd_eq(zero);
    let is_z_zero = normal.z.simd_eq(zero);
    let x_pos = normal.x.simd_gt(zero);
    let y_pos = normal.y.simd_gt(zero);
    let z_pos = normal.z.simd_gt(zero);

    // Note that this can result in a closest point in the middle of the box when the capsule is parallel to a face. That's fine.
    let box_edge_center = Vector3Wide {
        x: is_x_zero.select(zero, x_pos.select(b.half_width, -b.half_width)),
        y: is_y_zero.select(zero, y_pos.select(b.half_height, -b.half_height)),
        z: is_z_zero.select(zero, z_pos.select(b.half_length, -b.half_length)),
    };

    let use_edge_x = edge_direction_index.simd_eq(Simd::splat(0));
    let use_edge_y = edge_direction_index.simd_eq(Simd::splat(1));
    let box_edge_direction = Vector3Wide {
        x: use_edge_x.select(Vector::<f32>::splat(1.0), zero),
        y: use_edge_y.select(Vector::<f32>::splat(1.0), zero),
        z: (!use_edge_x & !use_edge_y).select(Vector::<f32>::splat(1.0), zero),
    };

    // From CapsulePairCollisionTask, point of closest approach:
    let mut ab = Vector3Wide::default();
    Vector3Wide::subtract(&box_edge_center, offset_a, &mut ab);
    let mut abda = Vector::<f32>::default();
    Vector3Wide::dot(&ab, capsule_axis, &mut abda);
    let mut abdb = Vector::<f32>::default();
    Vector3Wide::dot(&ab, &box_edge_direction, &mut abdb);
    let mut dadb = Vector::<f32>::default();
    Vector3Wide::dot(capsule_axis, &box_edge_direction, &mut dadb);

    // Note division by zero guard.
    let mut ta = (abda - abdb * dadb)
        / Vector::<f32>::splat(1e-15).simd_max(Vector::<f32>::splat(1.0) - dadb * dadb);

    // In some cases, ta won't be in a useful location. Need to constrain it to the projection of the box edge onto the capsule edge.
    let use_edge_x_f = edge_direction_index.simd_eq(Simd::splat(0));
    let use_edge_y_f = edge_direction_index.simd_eq(Simd::splat(1));
    let b_half_extent = use_edge_x_f.select(
        b.half_width,
        use_edge_y_f.select(b.half_height, b.half_length),
    );
    let b_onto_a_offset = b_half_extent * dadb.abs();
    let ta_min =
        (-*capsule_half_length).simd_max((*capsule_half_length).simd_min(abda - b_onto_a_offset));
    let ta_max =
        (*capsule_half_length).simd_min((-*capsule_half_length).simd_max(abda + b_onto_a_offset));
    ta = ta.simd_max(ta_min).simd_min(ta_max);

    let mut offset_along_capsule = Vector3Wide::default();
    Vector3Wide::scale_to(capsule_axis, &ta, &mut offset_along_capsule);
    Vector3Wide::add(offset_a, &offset_along_capsule, closest_point_from_edge);
}

#[inline(always)]
fn test_endpoint_normal(
    offset_a: &Vector3Wide,
    capsule_axis: &Vector3Wide,
    capsule_half_length: &Vector<f32>,
    endpoint: &Vector3Wide,
    b: &BoxWide,
    depth: &mut Vector<f32>,
    normal: &mut Vector3Wide,
) {
    let clamped = Vector3Wide {
        x: endpoint.x.simd_max(-b.half_width).simd_min(b.half_width),
        y: endpoint.y.simd_max(-b.half_height).simd_min(b.half_height),
        z: endpoint.z.simd_max(-b.half_length).simd_min(b.half_length),
    };
    Vector3Wide::subtract(endpoint, &clamped, normal);

    let length = normal.length();
    let inverse_length = Vector::<f32>::splat(1.0) / length;
    *normal = Vector3Wide::scale(normal, &inverse_length);

    let mut ba_n = Vector::<f32>::default();
    Vector3Wide::dot(offset_a, normal, &mut ba_n);
    let mut da_n = Vector::<f32>::default();
    Vector3Wide::dot(capsule_axis, normal, &mut da_n);
    *depth = normal.x.abs() * b.half_width
        + normal.y.abs() * b.half_height
        + normal.z.abs() * b.half_length
        + (da_n * *capsule_half_length).abs()
        - ba_n.abs();
    // If the endpoint doesn't generate a valid normal due to containment, ignore the depth result.
    *depth = length
        .simd_gt(Vector::<f32>::splat(1e-10))
        .select(*depth, Vector::<f32>::splat(f32::MAX));
}

#[inline(always)]
fn test_vertex_axis(
    b: &BoxWide,
    offset_a: &Vector3Wide,
    capsule_axis: &Vector3Wide,
    capsule_half_length: &Vector<f32>,
    depth: &mut Vector<f32>,
    normal: &mut Vector3Wide,
    closest_a: &mut Vector3Wide,
) {
    // closest point on axis to origin = offsetA - (offsetA * capsuleAxis) * capsuleAxis
    let mut dot = Vector::<f32>::default();
    Vector3Wide::dot(offset_a, capsule_axis, &mut dot);
    let clamped_dot = dot
        .simd_max(-*capsule_half_length)
        .simd_min(*capsule_half_length);
    let mut axis_offset = Vector3Wide::default();
    Vector3Wide::scale_to(capsule_axis, &clamped_dot, &mut axis_offset);
    let mut closest_on_axis = Vector3Wide::default();
    Vector3Wide::subtract(offset_a, &axis_offset, &mut closest_on_axis);

    let zero = Vector::<f32>::splat(0.0);
    let vertex = Vector3Wide {
        x: closest_on_axis
            .x
            .simd_lt(zero)
            .select(-b.half_width, b.half_width),
        y: closest_on_axis
            .y
            .simd_lt(zero)
            .select(-b.half_height, b.half_height),
        z: closest_on_axis
            .z
            .simd_lt(zero)
            .select(-b.half_length, b.half_length),
    };

    // closest point on axis to vertex: ((vertex - offsetA) * capsuleAxis) * capsuleAxis + offsetA - vertex
    let mut capsule_center_to_vertex = Vector3Wide::default();
    Vector3Wide::subtract(&vertex, offset_a, &mut capsule_center_to_vertex);
    let mut vertex_dot = Vector::<f32>::default();
    Vector3Wide::dot(&capsule_center_to_vertex, capsule_axis, &mut vertex_dot);
    let mut vertex_axis_offset = Vector3Wide::default();
    Vector3Wide::scale_to(capsule_axis, &vertex_dot, &mut vertex_axis_offset);
    Vector3Wide::add(&vertex_axis_offset, offset_a, closest_a);
    let mut vertex_to_closest_on_capsule = Vector3Wide::default();
    Vector3Wide::subtract(closest_a, &vertex, &mut vertex_to_closest_on_capsule);

    let length = vertex_to_closest_on_capsule.length();
    let inverse_length = Vector::<f32>::splat(1.0) / length;
    Vector3Wide::scale_to(&vertex_to_closest_on_capsule, &inverse_length, normal);
    // The normal is perpendicular to the capsule axis by construction, so no need to include the capsule length extent.
    *depth = normal.x.abs() * b.half_width
        + normal.y.abs() * b.half_height
        + normal.z.abs() * b.half_length
        - (offset_a.x * normal.x + offset_a.y * normal.y + offset_a.z * normal.z).abs();
    // Ignore degenerate cases. Worst outcome is that it reports intersection, which is pretty reasonable.
    *depth = length
        .simd_lt(Vector::<f32>::splat(1e-10))
        .select(Vector::<f32>::splat(f32::MAX), *depth);
}

#[inline(always)]
fn select_for_edge(
    edge_depth: &mut Vector<f32>,
    edge_local_normal: &mut Vector3Wide,
    edge_direction_index: &mut Vector<i32>,
    edge_depth_candidate: &Vector<f32>,
    edge_local_normal_candidate: &Vector3Wide,
    edge_direction_index_candidate: &Vector<i32>,
) {
    let use_candidate = edge_depth_candidate.simd_lt(*edge_depth).to_int();
    *edge_depth = edge_depth.simd_min(*edge_depth_candidate);
    *edge_local_normal = Vector3Wide::conditional_select(
        &use_candidate,
        edge_local_normal_candidate,
        edge_local_normal,
    );
    *edge_direction_index = Mask::from_int(use_candidate)
        .select(*edge_direction_index_candidate, *edge_direction_index);
}

#[inline(always)]
fn select3(
    depth: &mut Vector<f32>,
    local_normal: &mut Vector3Wide,
    local_closest: &mut Vector3Wide,
    depth_candidate: &Vector<f32>,
    local_normal_candidate: &Vector3Wide,
    local_closest_candidate: &Vector3Wide,
) {
    let use_candidate = depth_candidate.simd_lt(*depth).to_int();
    *depth = depth.simd_min(*depth_candidate);
    *local_normal =
        Vector3Wide::conditional_select(&use_candidate, local_normal_candidate, local_normal);
    *local_closest =
        Vector3Wide::conditional_select(&use_candidate, local_closest_candidate, local_closest);
}

#[inline(always)]
fn select2(
    depth: &mut Vector<f32>,
    local_normal: &mut Vector3Wide,
    depth_candidate: &Vector<f32>,
    local_normal_candidate: &Vector3Wide,
) {
    let use_candidate = depth_candidate.simd_lt(*depth).to_int();
    *depth = depth.simd_min(*depth_candidate);
    *local_normal =
        Vector3Wide::conditional_select(&use_candidate, local_normal_candidate, local_normal);
}

impl IPairDistanceTester<CapsuleWide, BoxWide> for CapsuleBoxDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &CapsuleWide,
        b: &BoxWide,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        // Bring the capsule into the box's local space.
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let capsule_axis = QuaternionWide::transform_unit_y(*orientation_a);
        let mut local_capsule_axis = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &capsule_axis,
            &r_b,
            &mut local_capsule_axis,
        );
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &r_b, &mut local_offset_b);
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

        let mut endpoint_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&local_capsule_axis, &a.half_length, &mut endpoint_offset);
        let mut endpoint0 = Vector3Wide::default();
        Vector3Wide::subtract(&local_offset_a, &endpoint_offset, &mut endpoint0);
        let mut depth = Vector::<f32>::default();
        let mut local_normal = Vector3Wide::default();
        test_endpoint_normal(
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &endpoint0,
            b,
            &mut depth,
            &mut local_normal,
        );
        let mut endpoint1 = Vector3Wide::default();
        Vector3Wide::add(&local_offset_a, &endpoint_offset, &mut endpoint1);
        let mut depth_candidate = Vector::<f32>::default();
        let mut local_normal_candidate = Vector3Wide::default();
        test_endpoint_normal(
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &endpoint1,
            b,
            &mut depth_candidate,
            &mut local_normal_candidate,
        );
        select2(
            &mut depth,
            &mut local_normal,
            &depth_candidate,
            &local_normal_candidate,
        );
        // Note that we did not yet pick a closest point for endpoint cases. That's because each case only generates a normal and interval test, not a minimal distance test.
        // The choice of which endpoint is actually closer is deferred until now.
        let mut endpoint_choice_dot = Vector::<f32>::default();
        Vector3Wide::dot(&local_capsule_axis, &local_normal, &mut endpoint_choice_dot);
        let mut local_closest = Vector3Wide::conditional_select(
            &endpoint_choice_dot
                .simd_lt(Vector::<f32>::splat(0.0))
                .to_int(),
            &endpoint1,
            &endpoint0,
        );

        // Swizzle XYZ -> YZX
        let mut edge_depth = Vector::<f32>::default();
        let mut edge_local_normal = Vector3Wide::default();
        test_box_edge(
            &local_offset_a.y,
            &local_offset_a.z,
            &local_offset_a.x,
            &local_capsule_axis.y,
            &local_capsule_axis.z,
            &local_capsule_axis.x,
            &b.half_height,
            &b.half_length,
            &b.half_width,
            &mut edge_depth,
            &mut edge_local_normal.y,
            &mut edge_local_normal.z,
            &mut edge_local_normal.x,
        );
        let mut edge_direction_index = Simd::splat(0i32);
        // Swizzle XYZ -> ZXY
        let mut edge_depth_candidate = Vector::<f32>::default();
        let mut edge_local_normal_candidate = Vector3Wide::default();
        test_box_edge(
            &local_offset_a.z,
            &local_offset_a.x,
            &local_offset_a.y,
            &local_capsule_axis.z,
            &local_capsule_axis.x,
            &local_capsule_axis.y,
            &b.half_length,
            &b.half_width,
            &b.half_height,
            &mut edge_depth_candidate,
            &mut edge_local_normal_candidate.z,
            &mut edge_local_normal_candidate.x,
            &mut edge_local_normal_candidate.y,
        );
        select_for_edge(
            &mut edge_depth,
            &mut edge_local_normal,
            &mut edge_direction_index,
            &edge_depth_candidate,
            &edge_local_normal_candidate,
            &Simd::splat(1i32),
        );
        // Swizzle XYZ -> XYZ
        test_box_edge(
            &local_offset_a.x,
            &local_offset_a.y,
            &local_offset_a.z,
            &local_capsule_axis.x,
            &local_capsule_axis.y,
            &local_capsule_axis.z,
            &b.half_width,
            &b.half_height,
            &b.half_length,
            &mut edge_depth_candidate,
            &mut edge_local_normal_candidate.x,
            &mut edge_local_normal_candidate.y,
            &mut edge_local_normal_candidate.z,
        );
        select_for_edge(
            &mut edge_depth,
            &mut edge_local_normal,
            &mut edge_direction_index,
            &edge_depth_candidate,
            &edge_local_normal_candidate,
            &Simd::splat(2i32),
        );

        // We can skip the edge finalization if they aren't ever used.
        if edge_depth.simd_lt(depth).any() {
            let mut edge_local_closest = Vector3Wide::default();
            get_edge_closest_point(
                &mut edge_local_normal,
                &edge_direction_index,
                b,
                &local_offset_a,
                &local_capsule_axis,
                &a.half_length,
                &mut edge_local_closest,
            );
            select3(
                &mut depth,
                &mut local_normal,
                &mut local_closest,
                &edge_depth,
                &edge_local_normal,
                &edge_local_closest,
            );
        }

        let mut vertex_depth = Vector::<f32>::default();
        let mut vertex_normal = Vector3Wide::default();
        let mut vertex_closest = Vector3Wide::default();
        test_vertex_axis(
            b,
            &local_offset_a,
            &local_capsule_axis,
            &a.half_length,
            &mut vertex_depth,
            &mut vertex_normal,
            &mut vertex_closest,
        );
        select3(
            &mut depth,
            &mut local_normal,
            &mut local_closest,
            &vertex_depth,
            &vertex_normal,
            &vertex_closest,
        );

        // Transform normal and closest point back into world space.
        Matrix3x3Wide::transform_without_overlap(&local_normal, &r_b, normal);
        Matrix3x3Wide::transform_without_overlap(&local_closest, &r_b, closest_a);
        let mut temp = Vector3Wide::default();
        Vector3Wide::add(&*closest_a, offset_b, &mut temp);
        *closest_a = temp;
        let mut closest_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&*normal, &a.radius, &mut closest_offset);
        closest_a.x -= closest_offset.x;
        closest_a.y -= closest_offset.y;
        closest_a.z -= closest_offset.z;
        *distance = -depth - a.radius;
        *intersected = distance.simd_lt(Vector::<f32>::splat(0.0)).to_int();
    }
}
