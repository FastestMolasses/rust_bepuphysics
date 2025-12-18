// Translated from BepuPhysics/CollisionDetection/SweepTasks/ConvexCompoundSweepOverlapFinder.cs

use crate::physics::body_properties::BodyVelocity;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::shape::IConvexShape;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::ChildOverlapsCollection;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};
use std::marker::PhantomData;

/// Abstraction for finding sweep overlaps between a convex shape and a compound.
pub trait IConvexCompoundSweepOverlapFinder<TShapeA: IConvexShape, TCompoundB: IBoundsQueryableCompound> {
    unsafe fn find_overlaps(
        shape_a: &TShapeA,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        compound_b: &TCompoundB,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        shapes: &Shapes,
        pool: &mut BufferPool,
        overlaps: &mut ChildOverlapsCollection,
    );
}

pub struct ConvexCompoundSweepOverlapFinder<TShapeA: IConvexShape, TCompoundB: IBoundsQueryableCompound> {
    _phantom: PhantomData<(TShapeA, TCompoundB)>,
}

impl<TShapeA: IConvexShape, TCompoundB: IBoundsQueryableCompound>
    IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB>
    for ConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB>
{
    unsafe fn find_overlaps(
        shape_a: &TShapeA,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        compound_b: &TCompoundB,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        shapes: &Shapes,
        pool: &mut BufferPool,
        overlaps: &mut ChildOverlapsCollection,
    ) {
        let mut sweep = Vec3::ZERO;
        let mut min = Vec3::ZERO;
        let mut max = Vec3::ZERO;
        BoundingBoxHelpers::get_local_bounding_box_for_sweep_shape(
            shape_a,
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

        *overlaps = ChildOverlapsCollection::default();
        compound_b.find_local_overlaps_sweep(
            min,
            max,
            sweep,
            maximum_t,
            pool,
            shapes,
            overlaps as *mut ChildOverlapsCollection as *mut u8,
        );
    }
}
