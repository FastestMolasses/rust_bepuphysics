// Translated from BepuPhysics/CollisionDetection/CollisionTasks/MeshPairOverlapFinder.cs

use std::marker::PhantomData;

use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shape::{IHomogeneousCompoundShape, IShapeWide};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::triangle::{Triangle, TriangleWide};
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

use super::compound_pair_collision_task::ICompoundPairOverlapFinder;
use super::compound_pair_overlaps::{
    ChildOverlapsCollection, CompoundPairOverlaps, OverlapQueryForPair,
};

/// Finds overlapping triangles between two mesh shapes.
///
/// For each triangle in mesh A, computes its bounding box in mesh B's local space
/// using SIMD-wide operations, then queries mesh B's structure for overlaps.
pub struct MeshPairOverlapFinder<TMeshA, TMeshB> {
    _marker: PhantomData<(TMeshA, TMeshB)>,
}

impl<
        TMeshA: IHomogeneousCompoundShape<Triangle, TriangleWide>,
        TMeshB: IHomogeneousCompoundShape<Triangle, TriangleWide> + IBoundsQueryableCompound,
    > ICompoundPairOverlapFinder for MeshPairOverlapFinder<TMeshA, TMeshB>
{
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut CompoundPairOverlaps,
    ) {
        let lanes = Vector::<f32>::LEN as i32;

        // Count total triangles in mesh A across all pairs.
        let mut total_compound_child_count = 0i32;
        for i in 0..pair_count {
            let mesh_a = &*(pairs[i].a as *const TMeshA);
            total_compound_child_count += mesh_a.child_count();
        }

        *overlaps = CompoundPairOverlaps::new(pool, pair_count, total_compound_child_count);

        // Set up query pairs: each triangle in mesh A needs to be tested against mesh B.
        let mut next_subpair_index = 0i32;
        for i in 0..pair_count {
            let pair = &pairs[i];
            let mesh_a = &*(pair.a as *const TMeshA);
            let child_count = mesh_a.child_count();
            overlaps.create_pair_overlaps(child_count);
            for j in 0..child_count {
                let subpair_index = next_subpair_index;
                next_subpair_index += 1;
                overlaps.get_overlaps_for_pair(subpair_index).child_index = j;
                overlaps.pair_queries[subpair_index].container = pair.b;
            }
        }

        // Process triangles in SIMD-wide batches to compute local bounding boxes.
        let mut triangles = TriangleWide::default();
        next_subpair_index = 0;
        for i in 0..pair_count {
            let pair = &pairs[i];
            let offset_b = Vector3Wide::broadcast(pair.offset_b);
            let mut orientation_a = QuaternionWide::default();
            QuaternionWide::broadcast(pair.orientation_a, &mut orientation_a);
            let mut orientation_b = QuaternionWide::default();
            QuaternionWide::broadcast(pair.orientation_b, &mut orientation_b);
            let relative_linear_velocity_a =
                Vector3Wide::broadcast(pair.relative_linear_velocity_a);
            let angular_velocity_a = Vector3Wide::broadcast(pair.angular_velocity_a);
            let angular_velocity_b = Vector3Wide::broadcast(pair.angular_velocity_b);
            let maximum_allowed_expansion = Vector::<f32>::splat(pair.maximum_expansion);

            let to_local_b = QuaternionWide::conjugate(&orientation_b);
            let mut local_orientation_a = QuaternionWide::default();
            QuaternionWide::concatenate_without_overlap(
                &orientation_a,
                &to_local_b,
                &mut local_orientation_a,
            );
            let mut local_offset_b = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(&offset_b, &to_local_b, &mut local_offset_b);
            let mut local_offset_a = Vector3Wide::default();
            Vector3Wide::negate(&local_offset_b, &mut local_offset_a);

            let mesh_a = &*(pair.a as *const TMeshA);
            let child_count = mesh_a.child_count();
            let mut j = 0i32;
            while j < child_count {
                let mut count = child_count - j;
                if count > lanes {
                    count = lanes;
                }
                for inner_index in 0..count {
                    mesh_a.get_local_child_wide(
                        j + inner_index,
                        GatherScatter::get_offset_instance_mut(
                            &mut triangles,
                            inner_index as usize,
                        ),
                    );
                }

                let mut maximum_radius = Vector::<f32>::default();
                let mut maximum_angular_expansion = Vector::<f32>::default();
                let mut min = Vector3Wide::default();
                let mut max = Vector3Wide::default();
                IShapeWide::<Triangle>::get_bounds(
                    &triangles,
                    &mut local_orientation_a,
                    count,
                    &mut maximum_radius,
                    &mut maximum_angular_expansion,
                    &mut min,
                    &mut max,
                );

                let mut local_relative_linear_velocity_a = Vector3Wide::default();
                QuaternionWide::transform_without_overlap(
                    &relative_linear_velocity_a,
                    &to_local_b,
                    &mut local_relative_linear_velocity_a,
                );
                let mut radius_a = Vector::<f32>::default();
                Vector3Wide::length_into(&local_offset_a, &mut radius_a);
                BoundingBoxHelpers::expand_local_bounding_boxes(
                    &mut min,
                    &mut max,
                    radius_a,
                    &local_offset_a,
                    &local_relative_linear_velocity_a,
                    &angular_velocity_a,
                    &angular_velocity_b,
                    dt,
                    maximum_radius,
                    maximum_angular_expansion,
                    maximum_allowed_expansion,
                );

                for inner_index in 0..count {
                    let pair_to_test = &mut overlaps.pair_queries[next_subpair_index];
                    next_subpair_index += 1;
                    Vector3Wide::read_slot(&min, inner_index as usize, &mut pair_to_test.min);
                    Vector3Wide::read_slot(&max, inner_index as usize, &mut pair_to_test.max);
                }

                j += lanes;
            }
        }

        // Query mesh B for overlaps using the computed query bounds.
        debug_assert!(total_compound_child_count > 0);
        let mesh_b = &*(overlaps.pair_queries[0i32].container as *const TMeshB);
        let pair_queries_ptr = &overlaps.pair_queries as *const Buffer<OverlapQueryForPair>;
        IBoundsQueryableCompound::find_local_overlaps::<
            CompoundPairOverlaps,
            ChildOverlapsCollection,
        >(mesh_b, &*pair_queries_ptr, pool, shapes, overlaps);
    }
}
