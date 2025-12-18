// Translated from BepuPhysics/CollisionDetection/SweepTasks/SphereCylinderDistanceTester.cs

use crate::physics::collidables::cylinder::CylinderWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::collision_tasks::sphere_cylinder_tester::SphereCylinderTester;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

#[derive(Default)]
pub struct SphereCylinderDistanceTester;

impl IPairDistanceTester<SphereWide, CylinderWide> for SphereCylinderDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &SphereWide,
        b: &CylinderWide,
        offset_b: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        let mut _cylinder_local_offset_a = Vector3Wide::default();
        let mut _horizontal_offset_length = Vector::<f32>::default();
        let mut _inverse_horizontal_offset_length = Vector::<f32>::default();
        let mut _sphere_to_closest_local_b = Vector3Wide::default();
        SphereCylinderTester::compute_sphere_to_closest(
            b,
            offset_b,
            &orientation_matrix_b,
            &mut _cylinder_local_offset_a,
            &mut _horizontal_offset_length,
            &mut _inverse_horizontal_offset_length,
            &mut _sphere_to_closest_local_b,
            closest_a,
        );

        let contact_distance_from_sphere_center = closest_a.length();
        // Note negation; normal points from B to A by convention.
        let inv_scale = Vector::<f32>::splat(-1.0) / contact_distance_from_sphere_center;
        Vector3Wide::scale_to(&*closest_a, &inv_scale, normal);
        *distance = contact_distance_from_sphere_center - a.radius;
        *intersected = distance.simd_lt(Vector::<f32>::splat(0.0)).to_int();
    }
}
