// Translated from BepuPhysics/CollisionDetection/SweepTasks/CompoundPairSweepTask.cs

use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::shape::ICompoundShape;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collision_detection::sweep_task_registry::{ISweepFilter, SweepTask, SweepTaskRegistry};
use crate::physics::collision_detection::sweep_tasks::compound_pair_sweep_overlap_finder::ICompoundPairSweepOverlapFinder;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};
use std::marker::PhantomData;

pub struct CompoundPairSweepTask<TCompoundA, TCompoundB, TOverlapFinder> {
    shape_type_index_a: i32,
    shape_type_index_b: i32,
    _phantom: PhantomData<(TCompoundA, TCompoundB, TOverlapFinder)>,
}

impl<
        TCompoundA: ICompoundShape + 'static,
        TCompoundB: ICompoundShape + IBoundsQueryableCompound + 'static,
        TOverlapFinder: ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB> + 'static,
    > CompoundPairSweepTask<TCompoundA, TCompoundB, TOverlapFinder>
{
    pub fn new() -> Self {
        Self {
            shape_type_index_a: TCompoundA::type_id(),
            shape_type_index_b: TCompoundB::type_id(),
            _phantom: PhantomData,
        }
    }
}

impl<
        TCompoundA: ICompoundShape + 'static,
        TCompoundB: ICompoundShape + IBoundsQueryableCompound + 'static,
        TOverlapFinder: ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB> + 'static,
    > SweepTask
    for CompoundPairSweepTask<TCompoundA, TCompoundB, TOverlapFinder>
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
        panic!("Compounds cannot be nested; this should never be called.");
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
        let a = &*(shape_data_a as *const TCompoundA);
        let b = &*(shape_data_b as *const TCompoundB);
        *t0 = f32::MAX;
        *t1 = f32::MAX;
        *hit_location = Vec3::ZERO;
        *hit_normal = Vec3::ZERO;
        let mut overlaps = Default::default();
        TOverlapFinder::find_overlaps(
            a, orientation_a, velocity_a,
            b, offset_b, orientation_b, velocity_b,
            maximum_t, &*shapes, &mut *pool, &mut overlaps,
        );
        let filter_ref = &**(filter as *const *const dyn ISweepFilter);
        for i in 0..overlaps.child_count {
            let child_overlaps = overlaps.get_overlaps_for_child(i);
            let child_a = a.get_child(child_overlaps.child_index);
            let child_type_a = child_a.shape_index.type_id();
            let batch_a = (&*shapes).get_batch(child_type_a as usize).unwrap();
            let (child_shape_data_a, _) = batch_a.get_shape_data(child_a.shape_index.index() as usize);
            for j in 0..child_overlaps.count {
                let child_index_b = child_overlaps.overlaps[j as usize];
                let child_b = b.get_child(child_index_b);
                let child_type_b = child_b.shape_index.type_id();
                let batch_b = (&*shapes).get_batch(child_type_b as usize).unwrap();
                let (child_shape_data_b, _) = batch_b.get_shape_data(child_b.shape_index.index() as usize);
                let allow = if flip_required {
                    filter_ref.allow_test(child_index_b, child_overlaps.child_index)
                } else {
                    filter_ref.allow_test(child_overlaps.child_index, child_index_b)
                };
                if allow {
                    if let Some(task) = (&*sweep_tasks).get_task(child_type_a, child_type_b) {
                        let mut t0_candidate = 0.0f32;
                        let mut t1_candidate = 0.0f32;
                        let mut hit_location_candidate = Vec3::ZERO;
                        let mut hit_normal_candidate = Vec3::ZERO;
                        if task.sweep(
                            child_shape_data_a, child_type_a, child_a.as_pose(), orientation_a, velocity_a,
                            child_shape_data_b, child_type_b, child_b.as_pose(), offset_b, orientation_b, velocity_b,
                            maximum_t, minimum_progression, convergence_threshold, maximum_iteration_count,
                            &mut t0_candidate, &mut t1_candidate, &mut hit_location_candidate, &mut hit_normal_candidate,
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
            }
        }
        overlaps.dispose(&mut *pool);
        *t1 < f32::MAX
    }
}
