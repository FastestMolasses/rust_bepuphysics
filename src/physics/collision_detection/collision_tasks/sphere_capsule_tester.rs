// Translated from BepuPhysics/CollisionDetection/CollisionTasks/SphereCapsuleTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::convex_contact_manifold_wide::Convex1ContactManifoldWide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

/// Pair tester for sphere vs capsule collisions.
pub struct SphereCapsuleTester;

impl SphereCapsuleTester {
    pub const BATCH_SIZE: i32 = 32;

    /// Tests sphere vs capsule collision (one orientation for the capsule).
    #[inline(always)]
    pub fn test(
        a: &SphereWide,
        b: &CapsuleWide,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        _pair_count: i32,
        manifold: &mut Convex1ContactManifoldWide,
    ) {
        // The contact for a sphere-capsule pair is based on the closest point of the sphere center
        // to the capsule internal line segment.
        let mut x = Vector3Wide::default();
        let mut y = Vector3Wide::default();
        QuaternionWide::transform_unit_xy(orientation_b, &mut x, &mut y);
        let mut t = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&y, offset_b, &mut t);
        t = b.half_length.simd_min((-b.half_length).simd_max(-t));
        let mut capsule_local_closest = Vector3Wide::default();
        Vector3Wide::scale_to(&y, &t, &mut capsule_local_closest);

        let mut sphere_to_internal = Vector3Wide::default();
        Vector3Wide::add(offset_b, &capsule_local_closest, &mut sphere_to_internal);
        let mut internal_distance = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&sphere_to_internal, &mut internal_distance);
        // Normal points from B to A by convention. Sphere is A, capsule is B, so negate.
        let inverse_distance = Vector::<f32>::splat(-1.0) / internal_distance;
        Vector3Wide::scale_to(&sphere_to_internal, &inverse_distance, &mut manifold.normal);
        let normal_is_valid = internal_distance.simd_gt(Vector::<f32>::splat(0.0));
        // If the center of the sphere is on the internal line segment, choose a direction
        // on the plane defined by the capsule's up vector.
        manifold.normal = Vector3Wide::conditional_select(&normal_is_valid.to_int(), &manifold.normal, &x);
        manifold.depth = a.radius + b.radius - internal_distance;
        manifold.feature_id = Vector::<i32>::splat(0);

        // Contact position from the normal and depth.
        let negative_offset_from_sphere = manifold.depth * Vector::<f32>::splat(0.5) - a.radius;
        Vector3Wide::scale_to(&manifold.normal, &negative_offset_from_sphere, &mut manifold.offset_a);
        manifold.contact_exists = manifold.depth.simd_gt(-*speculative_margin).to_int();
    }
}
