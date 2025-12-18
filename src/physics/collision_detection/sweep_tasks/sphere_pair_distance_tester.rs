// Translated from BepuPhysics/CollisionDetection/SweepTasks/SpherePairDistanceTester.cs

use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

#[derive(Default)]
pub struct SpherePairDistanceTester;

impl IPairDistanceTester<SphereWide, SphereWide> for SpherePairDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &SphereWide,
        b: &SphereWide,
        offset_b: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        _orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        let center_distance = offset_b.length();
        let inverse_distance = Vector::<f32>::splat(-1.0) / center_distance;
        Vector3Wide::scale_to(offset_b, &inverse_distance, normal);
        *distance = center_distance - a.radius - b.radius;
        let negative_radius_a = -a.radius;
        Vector3Wide::scale_to(&*normal, &negative_radius_a, closest_a);
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
    }
}
