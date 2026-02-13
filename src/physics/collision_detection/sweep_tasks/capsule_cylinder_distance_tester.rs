// Translated from BepuPhysics/CollisionDetection/SweepTasks/CapsuleCylinderDistanceTester.cs

// This is actually a little *slower* than using GJK, which is interesting. Playing with termination epsilons might change things. Keeping it here for example purposes.
// Using GJK in the contact generating tester is an option, but it's a bit awkward and the performance difference doesn't seem large enough to worry about in context.

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collidables::cylinder::CylinderWide;
use crate::physics::collision_detection::collision_tasks::capsule_cylinder_tester::CapsuleCylinderTester;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

pub struct CapsuleCylinderDistanceTester;

impl IPairDistanceTester<CapsuleWide, CylinderWide> for CapsuleCylinderDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &CapsuleWide,
        b: &CylinderWide,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        let inverse_orientation_b = QuaternionWide::conjugate(orientation_b);
        let mut local_orientation_a = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(
            orientation_a,
            &inverse_orientation_b,
            &mut local_orientation_a,
        );
        let capsule_axis = QuaternionWide::transform_unit_y(local_orientation_a);
        let mut local_offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            offset_b,
            &inverse_orientation_b,
            &mut local_offset_b,
        );
        let mut local_offset_a = Vector3Wide::default();
        Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

        let mut t = Vector::<f32>::default();
        let mut offset_from_cylinder_to_line_segment = Vector3Wide::default();
        CapsuleCylinderTester::get_closest_point_between_line_segment_and_cylinder(
            &local_offset_a,
            &capsule_axis,
            &a.half_length,
            b,
            &Vector::<i32>::splat(0),
            &mut t,
            &mut offset_from_cylinder_to_line_segment,
        );

        Vector3Wide::length_into(&offset_from_cylinder_to_line_segment, distance);
        let inv_dist = Vector::<f32>::splat(1.0) / *distance;
        let mut local_normal = Vector3Wide::default();
        Vector3Wide::scale_to(
            &offset_from_cylinder_to_line_segment,
            &inv_dist,
            &mut local_normal,
        );
        *distance = *distance - a.radius;
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
        QuaternionWide::transform_without_overlap(&local_normal, orientation_b, normal);
        let mut local_closest_a = Vector3Wide::default();
        Vector3Wide::scale_to(&capsule_axis, &t, &mut local_closest_a);
        let mut normal_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&local_normal, &a.radius, &mut normal_offset);
        local_closest_a = local_closest_a - normal_offset;
        local_closest_a = local_closest_a + local_offset_a;
        QuaternionWide::transform_without_overlap(&local_closest_a, orientation_b, closest_a);
        *closest_a = *closest_a + *offset_b;
    }
}
