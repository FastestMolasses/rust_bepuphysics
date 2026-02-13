// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SphereCylinderTester.cs

use crate::physics::collidables::cylinder::CylinderWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

/// Pair tester for sphere vs cylinder collisions.
pub struct SphereCylinderTester;

impl SphereCylinderTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Computes the closest point on the cylinder to the sphere center.
    #[inline(always)]
    pub fn compute_sphere_to_closest(
        b: &CylinderWide,
        offset_b: &Vector3Wide,
        orientation_matrix_b: &Matrix3x3Wide,
        cylinder_local_offset_a: &mut Vector3Wide,
        horizontal_offset_length: &mut Vector<f32>,
        inverse_horizontal_offset_length: &mut Vector<f32>,
        sphere_to_closest_local_b: &mut Vector3Wide,
        sphere_to_closest: &mut Vector3Wide,
    ) {
        // Clamp the sphere position to the cylinder's volume.
        let mut cylinder_local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            orientation_matrix_b,
            &mut cylinder_local_offset_b,
        );
        Vector3Wide::negate(&cylinder_local_offset_b, cylinder_local_offset_a);
        *horizontal_offset_length = StdFloat::sqrt(
            cylinder_local_offset_a.x * cylinder_local_offset_a.x
                + cylinder_local_offset_a.z * cylinder_local_offset_a.z,
        );
        *inverse_horizontal_offset_length = Vector::<f32>::splat(1.0) / *horizontal_offset_length;
        let horizontal_clamp_multiplier = b.radius * *inverse_horizontal_offset_length;
        let horizontal_clamp_required = horizontal_offset_length.simd_gt(b.radius);

        let clamped_x = horizontal_clamp_required.select(
            cylinder_local_offset_a.x * horizontal_clamp_multiplier,
            cylinder_local_offset_a.x,
        );
        let clamped_y = b
            .half_length
            .simd_min((-b.half_length).simd_max(cylinder_local_offset_a.y));
        let clamped_z = horizontal_clamp_required.select(
            cylinder_local_offset_a.z * horizontal_clamp_multiplier,
            cylinder_local_offset_a.z,
        );

        let clamped = Vector3Wide {
            x: clamped_x,
            y: clamped_y,
            z: clamped_z,
        };
        Vector3Wide::add(
            &clamped,
            &cylinder_local_offset_b,
            sphere_to_closest_local_b,
        );
        Matrix3x3Wide::transform_without_overlap(
            sphere_to_closest_local_b,
            orientation_matrix_b,
            sphere_to_closest,
        );
    }

    /// Tests sphere vs cylinder collision (one orientation for the cylinder).
    #[inline(always)]
    pub fn test(
        a: &SphereWide,
        b: &CylinderWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        _pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        let mut cylinder_local_offset_a = Vector3Wide::default();
        let mut horizontal_offset_length = Vector::<f32>::splat(0.0);
        let mut inverse_horizontal_offset_length = Vector::<f32>::splat(0.0);
        let mut sphere_to_contact_local_b = Vector3Wide::default();
        Self::compute_sphere_to_closest(
            b,
            offset_b,
            &orientation_matrix_b,
            &mut cylinder_local_offset_a,
            &mut horizontal_offset_length,
            &mut inverse_horizontal_offset_length,
            &mut sphere_to_contact_local_b,
            &mut manifold.offset_a,
        );

        // If the sphere center is inside the cylinder, compute the fastest way out.
        let abs_y = cylinder_local_offset_a.y.abs();
        let depth_y = b.half_length - abs_y;
        let horizontal_depth = b.radius - horizontal_offset_length;
        let use_depth_y = depth_y.simd_le(horizontal_depth);
        let use_top_cap_normal = cylinder_local_offset_a.y.simd_gt(Vector::<f32>::splat(0.0));

        let use_horizontal_fallback =
            horizontal_offset_length.simd_le(b.radius * Vector::<f32>::splat(1e-5));
        let zero = Vector::<f32>::splat(0.0);
        let one = Vector::<f32>::splat(1.0);
        let neg_one = Vector::<f32>::splat(-1.0);

        let local_internal_normal = Vector3Wide {
            x: use_depth_y.select(
                zero,
                use_horizontal_fallback.select(
                    one,
                    cylinder_local_offset_a.x * inverse_horizontal_offset_length,
                ),
            ),
            y: use_depth_y.select(use_top_cap_normal.select(one, neg_one), zero),
            z: use_depth_y.select(
                zero,
                use_horizontal_fallback.select(
                    zero,
                    cylinder_local_offset_a.z * inverse_horizontal_offset_length,
                ),
            ),
        };

        let mut contact_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&sphere_to_contact_local_b, &mut contact_distance);
        // Normal points from B to A by convention (negate).
        let neg_inv_distance = Vector::<f32>::splat(-1.0) / contact_distance;
        let mut local_external_normal = Vector3Wide::default();
        Vector3Wide::scale_to(
            &sphere_to_contact_local_b,
            &neg_inv_distance,
            &mut local_external_normal,
        );

        // Can't rely on the external normal if the sphere is too close to the surface.
        let use_internal = contact_distance.simd_lt(Vector::<f32>::splat(1e-7));
        let local_normal = Vector3Wide::conditional_select(
            &use_internal.to_int(),
            &local_internal_normal,
            &local_external_normal,
        );

        Matrix3x3Wide::transform_without_overlap(
            &local_normal,
            &orientation_matrix_b,
            &mut manifold.normal,
        );

        manifold.feature_id = Vector::<i32>::splat(0);
        manifold.depth = use_internal.select(
            use_depth_y.select(depth_y, horizontal_depth),
            -contact_distance,
        ) + a.radius;
        manifold.contact_exists = manifold.depth.simd_ge(-*speculative_margin).to_int();
    }
}
