// Translated from BepuPhysics/CollisionDetection/SweepTasks/SphereCapsuleDistanceTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

#[derive(Default)]
pub struct SphereCapsuleDistanceTester;

impl IPairDistanceTester<SphereWide, CapsuleWide> for SphereCapsuleDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &SphereWide,
        b: &CapsuleWide,
        offset_b: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        let mut _x = Vector3Wide::default();
        let mut y = Vector3Wide::default();
        QuaternionWide::transform_unit_xy(orientation_b, &mut _x, &mut y);
        let mut t = Vector::<f32>::default();
        Vector3Wide::dot(&y, offset_b, &mut t);
        t = b.half_length.simd_min((-b.half_length).simd_max(-t));
        let mut capsule_local_closest_point_on_line_segment = Vector3Wide::default();
        Vector3Wide::scale_to(&y, &t, &mut capsule_local_closest_point_on_line_segment);
        let mut sphere_to_internal_segment = Vector3Wide::default();
        Vector3Wide::add(
            offset_b,
            &capsule_local_closest_point_on_line_segment,
            &mut sphere_to_internal_segment,
        );
        let internal_distance = sphere_to_internal_segment.length();
        let inverse_distance = Vector::<f32>::splat(-1.0) / internal_distance;
        Vector3Wide::scale_to(&sphere_to_internal_segment, &inverse_distance, normal);
        let surface_offset = -a.radius;
        Vector3Wide::scale_to(&*normal, &surface_offset, closest_a);
        *distance = internal_distance - a.radius - b.radius;
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
    }
}
