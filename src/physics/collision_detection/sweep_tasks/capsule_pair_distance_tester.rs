// Translated from BepuPhysics/CollisionDetection/SweepTasks/CapsulePairDistanceTester.cs

use crate::physics::collidables::capsule::CapsuleWide;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

#[derive(Default)]
pub struct CapsulePairDistanceTester;

impl IPairDistanceTester<CapsuleWide, CapsuleWide> for CapsulePairDistanceTester {
    #[inline(always)]
    fn test(
        &self,
        a: &CapsuleWide,
        b: &CapsuleWide,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        _inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        // Compute the closest points between the two line segments. No clamping to begin with.
        // We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
        // Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
        // ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))
        let da = QuaternionWide::transform_unit_y(*orientation_a);
        let db = QuaternionWide::transform_unit_y(*orientation_b);
        let mut da_offset_b = Vector::<f32>::default();
        Vector3Wide::dot(&da, offset_b, &mut da_offset_b);
        let mut db_offset_b = Vector::<f32>::default();
        Vector3Wide::dot(&db, offset_b, &mut db_offset_b);
        let mut dadb = Vector::<f32>::default();
        Vector3Wide::dot(&da, &db, &mut dadb);
        // Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
        let mut ta = (da_offset_b - db_offset_b * dadb)
            / Vector::<f32>::splat(1e-15).simd_max(Vector::<f32>::splat(1.0) - dadb * dadb);
        // tb = ta * (da * db) - db * (b - a)
        let mut tb = ta * dadb - db_offset_b;

        // We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
        // That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
        // The projected intervals are:
        // B onto A: +-BHalfLength * (da * db) + da * offsetB
        // A onto B: +-AHalfLength * (da * db) - db * offsetB
        let absdadb = dadb.abs();
        let b_onto_a_offset = b.half_length * absdadb;
        let a_onto_b_offset = a.half_length * absdadb;
        let a_min = (-a.half_length).simd_max((da_offset_b - b_onto_a_offset).simd_min(a.half_length));
        let a_max = a.half_length.simd_min((-a.half_length).simd_max(da_offset_b + b_onto_a_offset));
        let b_min = (-b.half_length).simd_max((-a_onto_b_offset - db_offset_b).simd_min(b.half_length));
        let b_max = b.half_length.simd_min((-b.half_length).simd_max(a_onto_b_offset - db_offset_b));
        ta = ta.simd_max(a_min).simd_min(a_max);
        tb = tb.simd_max(b_min).simd_min(b_max);

        Vector3Wide::scale_to(&da, &ta, closest_a);
        let mut closest_b = Vector3Wide::default();
        Vector3Wide::scale_to(&db, &tb, &mut closest_b);
        closest_b = closest_b + *offset_b;

        Vector3Wide::subtract(&*closest_a, &closest_b, normal);
        Vector3Wide::length_into(&*normal, distance);
        let inverse_distance = Vector::<f32>::splat(1.0) / *distance;
        *normal = Vector3Wide::scale(&*normal, &inverse_distance);
        let mut a_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&*normal, &a.radius, &mut a_offset);
        *closest_a = *closest_a - a_offset;
        *distance = *distance - a.radius - b.radius;
        *intersected = distance.simd_le(Vector::<f32>::splat(0.0)).to_int();
    }
}
