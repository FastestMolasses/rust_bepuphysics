// Translated from BepuPhysics/CollisionDetection/CollisionTasks/CompoundPairOverlapFinder.cs

use std::marker::PhantomData;

use crate::physics::body_properties::RigidPoseWide;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::compound::CompoundChild;
use crate::physics::collidables::shape::ICompoundShape;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_batcher::BoundsTestedPair;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;

use super::compound_pair_collision_task::ICompoundPairOverlapFinder;
use super::compound_pair_overlaps::{ChildOverlapsCollection, CompoundPairOverlaps, OverlapQueryForPair};
use super::convex_compound_overlap_finder::IBoundsQueryableCompound;

/// Internal data linking a subpair to its source pair and child.
#[repr(C)]
struct SubpairData {
    pair: *const BoundsTestedPair,
    child: *const CompoundChild,
}

/// Finds overlapping children between two compound shapes.
///
/// Iterates over compound A's children, computes each child's bounding box
/// in compound B's local space using SIMD-wide operations, then queries
/// compound B's structure for overlaps.
pub struct CompoundPairOverlapFinder<TCompoundA, TCompoundB> {
    _marker: PhantomData<(TCompoundA, TCompoundB)>,
}

impl<TCompoundA: ICompoundShape, TCompoundB: IBoundsQueryableCompound> ICompoundPairOverlapFinder
    for CompoundPairOverlapFinder<TCompoundA, TCompoundB>
{
    unsafe fn find_local_overlaps(
        pairs: &Buffer<BoundsTestedPair>,
        pair_count: i32,
        pool: &mut BufferPool,
        shapes: &Shapes,
        dt: f32,
        overlaps: &mut CompoundPairOverlaps,
    ) {
        // Count total children across all pairs.
        let mut total_compound_child_count = 0i32;
        for i in 0..pair_count {
            let compound_a = &*(pairs[i].a as *const TCompoundA);
            total_compound_child_count += compound_a.child_count();
        }

        *overlaps = CompoundPairOverlaps::new(pool, pair_count, total_compound_child_count);

        // Allocate subpair tracking data.
        let mut subpair_data: Buffer<SubpairData> = pool.take(total_compound_child_count);

        let mut next_subpair_index = 0i32;
        for i in 0..pair_count {
            let pair = &pairs[i];
            let compound_a = &*(pair.a as *const TCompoundA);
            let child_count = compound_a.child_count();
            overlaps.create_pair_overlaps(child_count);

            for j in 0..child_count {
                let subpair_index = next_subpair_index;
                next_subpair_index += 1;

                overlaps.get_overlaps_for_pair(subpair_index).child_index = j;
                overlaps.pair_queries[subpair_index].container = pair.b;

                let subpair = &mut subpair_data[subpair_index];
                subpair.pair = pair as *const BoundsTestedPair;
                subpair.child = compound_a.get_child(j) as *const CompoundChild;
            }
        }

        let lanes = Vector::<f32>::LEN;

        // Process children in SIMD-wide batches, computing local bounding boxes.
        let mut offset_b = Vector3Wide::default();
        let mut orientation_a = QuaternionWide::default();
        let mut orientation_b = QuaternionWide::default();
        let mut relative_linear_velocity_a = Vector3Wide::default();
        let mut angular_velocity_a = Vector3Wide::default();
        let mut angular_velocity_b = Vector3Wide::default();
        let mut maximum_allowed_expansion = Vector::<f32>::default();
        let mut maximum_radius = Vector::<f32>::default();
        let mut maximum_angular_expansion = Vector::<f32>::default();
        let mut local_poses_a = RigidPoseWide::default();
        let mut mins = Vector3Wide::default();
        let mut maxes = Vector3Wide::default();

        let mut i = 0i32;
        while i < total_compound_child_count {
            let mut count = total_compound_child_count - i;
            if count > lanes as i32 {
                count = lanes as i32;
            }

            // Gather data from each subpair into wide registers.
            for j in 0..count as usize {
                let subpair_index = i as usize + j;
                let subpair = &subpair_data[subpair_index];
                let pair = &*subpair.pair;
                let child = &*subpair.child;

                Vector3Wide::write_first(pair.offset_b, GatherScatter::get_offset_instance_mut(&mut offset_b, j));
                QuaternionWide::write_first(pair.orientation_a, GatherScatter::get_offset_instance_mut(&mut orientation_a, j));
                QuaternionWide::write_first(pair.orientation_b, GatherScatter::get_offset_instance_mut(&mut orientation_b, j));
                Vector3Wide::write_first(pair.relative_linear_velocity_a, GatherScatter::get_offset_instance_mut(&mut relative_linear_velocity_a, j));
                Vector3Wide::write_first(pair.angular_velocity_a, GatherScatter::get_offset_instance_mut(&mut angular_velocity_a, j));
                Vector3Wide::write_first(pair.angular_velocity_b, GatherScatter::get_offset_instance_mut(&mut angular_velocity_b, j));
                *(&mut maximum_allowed_expansion as *mut Vector<f32> as *mut f32).add(j) = pair.maximum_expansion;

                RigidPoseWide::write_first(child.as_pose(), GatherScatter::get_offset_instance_mut(&mut local_poses_a, j));
            }

            // Transform child poses from compound A space to compound B's local space.
            let to_local_b = QuaternionWide::conjugate(&orientation_b);
            let mut local_orientations_a = QuaternionWide::default();
            QuaternionWide::concatenate_without_overlap(&orientation_a, &to_local_b, &mut local_orientations_a);
            let mut local_child_orientations_a = QuaternionWide::default();
            QuaternionWide::concatenate_without_overlap(&local_poses_a.orientation, &local_orientations_a, &mut local_child_orientations_a);
            let mut local_offset_a = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(&local_poses_a.position, &local_orientations_a, &mut local_offset_a);
            let mut local_offset_b = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(&offset_b, &to_local_b, &mut local_offset_b);
            let mut local_positions_a = Vector3Wide::default();
            Vector3Wide::subtract(&local_offset_a, &local_offset_b, &mut local_positions_a);

            // Compute child shape bounds in compound B's local space.
            for j in 0..count as usize {
                let shape_index = (*subpair_data[i as usize + j].child).shape_index;
                let mut local_child_orientation_a = glam::Quat::IDENTITY;
                QuaternionWide::read_slot(
                    GatherScatter::get_offset_instance(&local_child_orientations_a, j),
                    0,
                    &mut local_child_orientation_a,
                );
                let batch = shapes.get_batch(shape_index.type_id() as usize).expect("Shape batch must exist");
                let mut min = glam::Vec3::ZERO;
                let mut max = glam::Vec3::ZERO;
                let mr = &mut maximum_radius as *mut Vector<f32> as *mut f32;
                let mae = &mut maximum_angular_expansion as *mut Vector<f32> as *mut f32;
                batch.compute_bounds_with_angular_data(
                    shape_index.index() as usize,
                    local_child_orientation_a,
                    &mut *mr.add(j),
                    &mut *mae.add(j),
                    &mut min,
                    &mut max,
                );
                Vector3Wide::write_first(min, GatherScatter::get_offset_instance_mut(&mut mins, j));
                Vector3Wide::write_first(max, GatherScatter::get_offset_instance_mut(&mut maxes, j));
            }

            // Expand bounds for velocity, angular motion, and speculative margin.
            let mut local_relative_linear_velocity_a = Vector3Wide::default();
            QuaternionWide::transform_without_overlap(&relative_linear_velocity_a, &to_local_b, &mut local_relative_linear_velocity_a);
            let mut radius_a = Vector::<f32>::default();
            Vector3Wide::length_into(&local_offset_a, &mut radius_a);
            BoundingBoxHelpers::expand_local_bounding_boxes(
                &mut mins, &mut maxes,
                radius_a, &local_positions_a,
                &local_relative_linear_velocity_a, &angular_velocity_a, &angular_velocity_b,
                dt,
                maximum_radius, maximum_angular_expansion, maximum_allowed_expansion,
            );

            // Store results in query bounds for downstream overlap testing.
            for j in 0..count as usize {
                let pair_to_test = &mut overlaps.pair_queries[i as usize + j];
                Vector3Wide::read_slot(&mins, j, &mut pair_to_test.min);
                Vector3Wide::read_slot(&maxes, j, &mut pair_to_test.max);
            }

            i += lanes as i32;
        }

        // Use compound B's acceleration structure to find overlapping children.
        debug_assert!(total_compound_child_count > 0);
        let pair_queries = overlaps.pair_queries;
        let compound_b = &*(pair_queries[0usize].container as *const TCompoundB);
        compound_b.find_local_overlaps::<CompoundPairOverlaps, ChildOverlapsCollection>(
            &pair_queries, pool, shapes, overlaps,
        );

        pool.return_buffer(&mut subpair_data);
    }
}
