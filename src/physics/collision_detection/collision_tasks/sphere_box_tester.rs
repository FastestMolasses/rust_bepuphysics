// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SphereBoxTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for sphere vs box collisions.
pub struct SphereBoxTester;

impl SphereBoxTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests sphere vs box collision (one orientation for the box).
    #[inline(always)]
    pub fn test(
        a: &SphereWide,
        b: &BoxWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        _pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        // Clamp the position of the sphere to the box.
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(offset_b, &orientation_matrix_b, &mut local_offset_b);

        let clamped_x = local_offset_b.x.simd_max(-b.half_width).simd_min(b.half_width);
        let clamped_y = local_offset_b.y.simd_max(-b.half_height).simd_min(b.half_height);
        let clamped_z = local_offset_b.z.simd_max(-b.half_length).simd_min(b.half_length);
        let clamped_local_offset_b = Vector3Wide {
            x: clamped_x,
            y: clamped_y,
            z: clamped_z,
        };

        // Implicit negation to make the normal point from B to A.
        let mut outside_normal = Vector3Wide::default();
        Vector3Wide::subtract(&clamped_local_offset_b, &local_offset_b, &mut outside_normal);
        let mut distance = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&outside_normal, &mut distance);
        let inverse_distance = Vector::<f32>::splat(1.0) / distance;
        outside_normal = Vector3Wide::scale(&outside_normal, &inverse_distance);
        let outside_depth = a.radius - distance;

        // If the sphere center is inside the box, choose the shortest local axis to exit.
        let depth_x = b.half_width - local_offset_b.x.abs();
        let depth_y = b.half_height - local_offset_b.y.abs();
        let depth_z = b.half_length - local_offset_b.z.abs();
        let inside_depth = depth_x.simd_min(depth_y.simd_min(depth_z));
        let use_x = inside_depth.simd_eq(depth_x);
        let use_y = inside_depth.simd_eq(depth_y) & !use_x;
        let use_z = !(use_x | use_y);

        let one = Vector::<f32>::splat(1.0);
        let neg_one = Vector::<f32>::splat(-1.0);
        let zero = Vector::<f32>::splat(0.0);
        let inside_normal = Vector3Wide {
            x: use_x.select(
                local_offset_b.x.simd_lt(zero).select(one, neg_one),
                zero,
            ),
            y: use_y.select(
                local_offset_b.y.simd_lt(zero).select(one, neg_one),
                zero,
            ),
            z: use_z.select(
                local_offset_b.z.simd_lt(zero).select(one, neg_one),
                zero,
            ),
        };

        let inside_depth_total = inside_depth + a.radius;
        let use_inside = distance.simd_eq(Vector::<f32>::splat(0.0));
        let local_normal = Vector3Wide::conditional_select(&use_inside.to_int(), &inside_normal, &outside_normal);
        Matrix3x3Wide::transform_without_overlap(&local_normal, &orientation_matrix_b, &mut manifold.normal);
        manifold.depth = use_inside.select(inside_depth_total, outside_depth);
        manifold.feature_id = Vector::<i32>::splat(0);

        // Contact position from the normal and depth.
        let negative_offset_from_sphere = manifold.depth * Vector::<f32>::splat(0.5) - a.radius;
        Vector3Wide::scale_to(&manifold.normal, &negative_offset_from_sphere, &mut manifold.offset_a);
        manifold.contact_exists = manifold.depth.simd_gt(-*speculative_margin).to_int();
    }
}
