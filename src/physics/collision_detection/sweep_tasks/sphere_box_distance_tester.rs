// Translated from BepuPhysics/CollisionDetection/SweepTasks/SphereBoxDistanceTester.cs

use crate::physics::collidables::box_shape::BoxWide;
use crate::physics::collidables::sphere::SphereWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

#[derive(Default)]
pub struct SphereBoxDistanceTester;

impl IPairDistanceTester<SphereWide, BoxWide> for SphereBoxDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &SphereWide,
        b: &BoxWide,
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
        let mut local_offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            offset_b,
            &orientation_matrix_b,
            &mut local_offset_b,
        );
        let clamped_local_offset_b = Vector3Wide {
            x: local_offset_b
                .x
                .simd_max(-b.half_width)
                .simd_min(b.half_width),
            y: local_offset_b
                .y
                .simd_max(-b.half_height)
                .simd_min(b.half_height),
            z: local_offset_b
                .z
                .simd_max(-b.half_length)
                .simd_min(b.half_length),
        };
        let mut local_normal = Vector3Wide::default();
        Vector3Wide::subtract(&clamped_local_offset_b, &local_offset_b, &mut local_normal);
        let inner_distance = local_normal.length();
        let inverse_distance = Vector::<f32>::splat(1.0) / inner_distance;
        local_normal = Vector3Wide::scale(&local_normal, &inverse_distance);
        Matrix3x3Wide::transform_without_overlap(&local_normal, &orientation_matrix_b, normal);
        let negative_radius = -a.radius;
        Vector3Wide::scale_to(&*normal, &negative_radius, closest_a);
        *distance = inner_distance - a.radius;
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
    }
}
