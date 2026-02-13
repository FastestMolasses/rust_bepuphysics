// Translated from BepuPhysics/CollisionDetection/SweepTasks/

use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

pub mod capsule_box_distance_tester;
pub mod capsule_cylinder_distance_tester;
pub mod capsule_pair_distance_tester;
pub mod compound_homogeneous_compound_sweep_task;
pub mod compound_pair_sweep_overlap_finder;
pub mod compound_pair_sweep_overlaps;
pub mod compound_pair_sweep_task;
pub mod convex_compound_sweep_overlap_finder;
pub mod convex_compound_sweep_task;
pub mod convex_homogeneous_compound_sweep_task;
pub mod convex_pair_sweep_task;
pub mod gjk_distance_tester;
pub mod sphere_box_distance_tester;
pub mod sphere_capsule_distance_tester;
pub mod sphere_cylinder_distance_tester;
pub mod sphere_pair_distance_tester;
pub mod sphere_triangle_distance_tester;

/// Defines a tester that computes the distance between two shapes.
pub trait IPairDistanceTester<TShapeWideA, TShapeWideB> {
    fn test(
        &self,
        a: &TShapeWideA,
        b: &TShapeWideB,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    );
}
