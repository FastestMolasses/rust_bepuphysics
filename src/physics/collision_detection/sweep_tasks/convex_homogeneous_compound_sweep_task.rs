// Translated from BepuPhysics/CollisionDetection/SweepTasks/ConvexHomogeneousCompoundSweepTask.cs

use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::shape::{IConvexShape, IHomogeneousCompoundShape, IShapeWide};
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::physics::collision_detection::sweep_task_registry::{
    ISweepFilter, SweepTask, SweepTaskRegistry,
};
use crate::physics::collision_detection::sweep_tasks::convex_compound_sweep_overlap_finder::IConvexCompoundSweepOverlapFinder;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};
use std::marker::PhantomData;

pub struct ConvexHomogeneousCompoundSweepTask<
    TConvex,
    TConvexWide,
    TCompound,
    TChildType,
    TChildTypeWide,
    TOverlapFinder,
> {
    shape_type_index_a: i32,
    shape_type_index_b: i32,
    _phantom: PhantomData<(
        TConvex,
        TConvexWide,
        TCompound,
        TChildType,
        TChildTypeWide,
        TOverlapFinder,
    )>,
}

impl<
        TConvex: IConvexShape + 'static,
        TConvexWide: IShapeWide<TConvex> + 'static,
        TCompound: IHomogeneousCompoundShape<TChildType, TChildTypeWide> + IBoundsQueryableCompound + 'static,
        TChildType: IConvexShape + 'static,
        TChildTypeWide: IShapeWide<TChildType> + 'static,
        TOverlapFinder: IConvexCompoundSweepOverlapFinder<TConvex, TCompound> + 'static,
    >
    ConvexHomogeneousCompoundSweepTask<
        TConvex,
        TConvexWide,
        TCompound,
        TChildType,
        TChildTypeWide,
        TOverlapFinder,
    >
{
    pub fn new() -> Self {
        Self {
            shape_type_index_a: TConvex::type_id(),
            shape_type_index_b: TCompound::type_id(),
            _phantom: PhantomData,
        }
    }
}

impl<
        TConvex: IConvexShape + 'static,
        TConvexWide: IShapeWide<TConvex> + 'static,
        TCompound: IHomogeneousCompoundShape<TChildType, TChildTypeWide> + IBoundsQueryableCompound + 'static,
        TChildType: IConvexShape + 'static,
        TChildTypeWide: IShapeWide<TChildType> + 'static,
        TOverlapFinder: IConvexCompoundSweepOverlapFinder<TConvex, TCompound> + 'static,
    > SweepTask
    for ConvexHomogeneousCompoundSweepTask<
        TConvex,
        TConvexWide,
        TCompound,
        TChildType,
        TChildTypeWide,
        TOverlapFinder,
    >
{
    fn shape_type_index_a(&self) -> i32 {
        self.shape_type_index_a
    }
    fn shape_type_index_b(&self) -> i32 {
        self.shape_type_index_b
    }

    unsafe fn preordered_type_sweep(
        &self,
        _shape_data_a: *const u8,
        _local_pose_a: &RigidPose,
        _orientation_a: Quat,
        _velocity_a: &BodyVelocity,
        _shape_data_b: *const u8,
        _local_pose_b: &RigidPose,
        _offset_b: Vec3,
        _orientation_b: Quat,
        _velocity_b: &BodyVelocity,
        _maximum_t: f32,
        _minimum_progression: f32,
        _convergence_threshold: f32,
        _maximum_iteration_count: i32,
        _t0: &mut f32,
        _t1: &mut f32,
        _hit_location: &mut Vec3,
        _hit_normal: &mut Vec3,
    ) -> bool {
        panic!("Compounds can never be nested; this should never be called.");
    }

    unsafe fn preordered_type_sweep_filtered(
        &self,
        shape_data_a: *const u8,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        flip_required: bool,
        filter: *mut u8,
        shapes: *mut Shapes,
        sweep_tasks: *mut SweepTaskRegistry,
        pool: *mut BufferPool,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool {
        let compound = &*(shape_data_b as *const TCompound);
        *t0 = f32::MAX;
        *t1 = f32::MAX;
        *hit_location = Vec3::ZERO;
        *hit_normal = Vec3::ZERO;
        let task = (&*sweep_tasks).get_task(self.shape_type_index_a, TChildType::type_id());
        if let Some(task) = task {
            let convex = &*(shape_data_a as *const TConvex);
            let mut overlaps = Default::default();
            TOverlapFinder::find_overlaps(
                convex,
                orientation_a,
                velocity_a,
                compound,
                offset_b,
                orientation_b,
                velocity_b,
                maximum_t,
                &*shapes,
                &mut *pool,
                &mut overlaps,
            );
            let filter_ref = &**(filter as *const *const dyn ISweepFilter);
            for i in 0..overlaps.count {
                let child_index = overlaps.overlaps[i as usize];
                let allow = if flip_required {
                    filter_ref.allow_test(0, child_index)
                } else {
                    filter_ref.allow_test(child_index, 0)
                };
                if allow {
                    let mut child_shape = std::mem::MaybeUninit::<TChildType>::uninit();
                    let mut child_pose = RigidPose::IDENTITY;
                    compound.get_posed_local_child(
                        child_index,
                        &mut *child_shape.as_mut_ptr(),
                        &mut child_pose,
                    );
                    let identity_pose = RigidPose::IDENTITY;
                    let mut t0_candidate = 0.0f32;
                    let mut t1_candidate = 0.0f32;
                    let mut hit_location_candidate = Vec3::ZERO;
                    let mut hit_normal_candidate = Vec3::ZERO;
                    if task.sweep(
                        shape_data_a,
                        self.shape_type_index_a,
                        &identity_pose,
                        orientation_a,
                        velocity_a,
                        child_shape.as_ptr() as *const u8,
                        TChildType::type_id(),
                        &child_pose,
                        offset_b,
                        orientation_b,
                        velocity_b,
                        maximum_t,
                        minimum_progression,
                        convergence_threshold,
                        maximum_iteration_count,
                        &mut t0_candidate,
                        &mut t1_candidate,
                        &mut hit_location_candidate,
                        &mut hit_normal_candidate,
                    ) {
                        if t1_candidate < *t1 {
                            *t0 = t0_candidate;
                            *t1 = t1_candidate;
                            *hit_location = hit_location_candidate;
                            *hit_normal = hit_normal_candidate;
                        }
                    }
                }
            }
            overlaps.dispose(&mut *pool);
        }
        *t1 < f32::MAX
    }
}
