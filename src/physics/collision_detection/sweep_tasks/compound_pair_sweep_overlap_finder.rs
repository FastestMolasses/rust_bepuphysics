// Translated from BepuPhysics/CollisionDetection/SweepTasks/CompoundPairSweepOverlapFinder.cs

use crate::physics::body_properties::BodyVelocity;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::compound::CompoundChild;
use crate::physics::collidables::shape::ICompoundShape;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::ChildOverlapsCollection;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::physics::collision_detection::sweep_tasks::compound_pair_sweep_overlaps::CompoundPairSweepOverlaps;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};
use std::marker::PhantomData;

/// Abstraction for finding sweep overlaps between two compound shapes.
pub trait ICompoundPairSweepOverlapFinder<
    TCompoundA: ICompoundShape,
    TCompoundB: IBoundsQueryableCompound,
>
{
    unsafe fn find_overlaps(
        compound_a: &TCompoundA,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        compound_b: &TCompoundB,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        shapes: &Shapes,
        pool: &mut BufferPool,
        overlaps: &mut CompoundPairSweepOverlaps,
    );
}

pub struct CompoundPairSweepOverlapFinder<
    TCompoundA: ICompoundShape,
    TCompoundB: IBoundsQueryableCompound,
> {
    _phantom: PhantomData<(TCompoundA, TCompoundB)>,
}

impl<TCompoundA: ICompoundShape, TCompoundB: IBoundsQueryableCompound>
    ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB>
    for CompoundPairSweepOverlapFinder<TCompoundA, TCompoundB>
{
    unsafe fn find_overlaps(
        compound_a: &TCompoundA,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        compound_b: &TCompoundB,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        shapes: &Shapes,
        pool: &mut BufferPool,
        overlaps: &mut CompoundPairSweepOverlaps,
    ) {
        *overlaps = CompoundPairSweepOverlaps::new(pool, compound_a.child_count());
        for i in 0..compound_a.child_count() {
            let child: &CompoundChild = compound_a.get_child(i);
            let mut sweep = Vec3::ZERO;
            let mut min = Vec3::ZERO;
            let mut max = Vec3::ZERO;
            BoundingBoxHelpers::get_local_bounding_box_for_sweep_child(
                child.shape_index,
                shapes,
                child.as_pose(),
                orientation_a,
                velocity_a,
                offset_b,
                orientation_b,
                velocity_b,
                maximum_t,
                &mut sweep,
                &mut min,
                &mut max,
            );
            let child_overlaps = overlaps.get_overlaps_for_child(i);
            child_overlaps.child_index = i;
            compound_b.find_local_overlaps_sweep(
                min,
                max,
                sweep,
                maximum_t,
                pool,
                shapes,
                child_overlaps as *mut ChildOverlapsCollection as *mut u8,
            );
        }
    }
}
