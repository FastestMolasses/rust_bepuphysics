// Translated from BepuPhysics/CollisionDetection/SweepTaskRegistry.cs

use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::shapes::Shapes;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};

/// Filter for swept tests between children of compound shapes.
pub trait ISweepFilter {
    /// Checks whether a swept test should be performed for children of swept shapes.
    fn allow_test(&self, child_a: i32, child_b: i32) -> bool;
}

/// Parent type of tasks which handle sweep tests between shape pairs.
pub trait SweepTask {
    /// Gets the first shape type index associated with the task.
    fn shape_type_index_a(&self) -> i32;
    /// Gets the second shape type index associated with the task.
    fn shape_type_index_b(&self) -> i32;

    /// Performs a preordered type sweep with local poses (for shapes with local offsets).
    unsafe fn preordered_type_sweep(
        &self,
        shape_data_a: *const u8,
        local_pose_a: &RigidPose,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        local_pose_b: &RigidPose,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool;

    /// Performs a sweep with local poses, handling type reordering.
    unsafe fn sweep(
        &self,
        shape_data_a: *const u8,
        shape_type_a: i32,
        local_pose_a: &RigidPose,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        shape_type_b: i32,
        local_pose_b: &RigidPose,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool {
        debug_assert!(
            (shape_type_a == self.shape_type_index_a() && shape_type_b == self.shape_type_index_b())
                || (shape_type_a == self.shape_type_index_b()
                    && shape_type_b == self.shape_type_index_a()),
            "Sweep type requirements not met."
        );
        if shape_type_a == self.shape_type_index_a() {
            self.preordered_type_sweep(
                shape_data_a,
                local_pose_a,
                orientation_a,
                velocity_a,
                shape_data_b,
                local_pose_b,
                offset_b,
                orientation_b,
                velocity_b,
                maximum_t,
                minimum_progression,
                convergence_threshold,
                maximum_iteration_count,
                t0,
                t1,
                hit_location,
                hit_normal,
            )
        } else {
            let intersected = self.preordered_type_sweep(
                shape_data_b,
                local_pose_b,
                orientation_b,
                velocity_b,
                shape_data_a,
                local_pose_a,
                -offset_b,
                orientation_a,
                velocity_a,
                maximum_t,
                minimum_progression,
                convergence_threshold,
                maximum_iteration_count,
                t0,
                t1,
                hit_location,
                hit_normal,
            );
            // Normals are calibrated to point from B to A by convention; retain that convention if the parameters were reversed.
            *hit_normal = -*hit_normal;
            *hit_location = *hit_location + offset_b;
            intersected
        }
    }

    /// Performs a preordered type sweep with compound filter support.
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
    ) -> bool;

    /// Performs a sweep with compound filter support, handling type reordering.
    unsafe fn sweep_filtered(
        &self,
        shape_data_a: *const u8,
        shape_type_a: i32,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        shape_type_b: i32,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        filter: *mut u8,
        shapes: *mut Shapes,
        sweep_tasks: *mut SweepTaskRegistry,
        pool: *mut BufferPool,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool {
        debug_assert!(
            (shape_type_a == self.shape_type_index_a() && shape_type_b == self.shape_type_index_b())
                || (shape_type_a == self.shape_type_index_b()
                    && shape_type_b == self.shape_type_index_a()),
            "Types must match expected types."
        );
        let flip_required = shape_type_b == self.shape_type_index_a();
        if flip_required {
            let hit = self.preordered_type_sweep_filtered(
                shape_data_b,
                orientation_b,
                velocity_b,
                shape_data_a,
                -offset_b,
                orientation_a,
                velocity_a,
                maximum_t,
                minimum_progression,
                convergence_threshold,
                maximum_iteration_count,
                flip_required,
                filter,
                shapes,
                sweep_tasks,
                pool,
                t0,
                t1,
                hit_location,
                hit_normal,
            );
            *hit_normal = -*hit_normal;
            *hit_location = *hit_location + offset_b;
            hit
        } else {
            self.preordered_type_sweep_filtered(
                shape_data_a,
                orientation_a,
                velocity_a,
                shape_data_b,
                offset_b,
                orientation_b,
                velocity_b,
                maximum_t,
                minimum_progression,
                convergence_threshold,
                maximum_iteration_count,
                flip_required,
                filter,
                shapes,
                sweep_tasks,
                pool,
                t0,
                t1,
                hit_location,
                hit_normal,
            )
        }
    }
}

/// Registry of sweep tasks used to handle CCD for various shape pair types.
pub struct SweepTaskRegistry {
    top_level_matrix: Vec<Vec<i32>>,
    pub(crate) tasks: Vec<Box<dyn SweepTask>>,
    count: usize,
}

impl SweepTaskRegistry {
    /// Creates a new sweep task registry.
    pub fn new(initial_shape_count: usize) -> Self {
        let mut registry = Self {
            top_level_matrix: Vec::new(),
            tasks: Vec::new(),
            count: 0,
        };
        registry.resize_matrix(initial_shape_count);
        registry
    }

    fn resize_matrix(&mut self, new_size: usize) {
        let old_size = self.top_level_matrix.len();
        self.top_level_matrix.resize_with(new_size, Vec::new);
        for i in 0..new_size {
            self.top_level_matrix[i].resize(new_size, -1);
        }
    }

    /// Registers a sweep task. Returns the index of the task.
    pub fn register(&mut self, task: Box<dyn SweepTask>) -> usize {
        let index = self.count;

        let new_count = self.count + 1;
        if self.tasks.len() < new_count {
            self.tasks.reserve(new_count - self.tasks.len());
        }
        self.tasks.push(task);
        self.count = new_count;

        let a = self.tasks[index].shape_type_index_a();
        let b = self.tasks[index].shape_type_index_b();
        let highest_shape_index = if a > b { a } else { b };
        if highest_shape_index as usize >= self.top_level_matrix.len() {
            self.resize_matrix(highest_shape_index as usize + 1);
        }
        self.top_level_matrix[a as usize][b as usize] = index as i32;
        self.top_level_matrix[b as usize][a as usize] = index as i32;

        index
    }

    /// Gets the task for the given shape type pair, or None if no task is registered.
    #[inline(always)]
    pub fn get_task(&self, top_level_type_a: i32, top_level_type_b: i32) -> Option<&dyn SweepTask> {
        if top_level_type_a as usize >= self.top_level_matrix.len() {
            return None;
        }
        if top_level_type_b as usize >= self.top_level_matrix[top_level_type_a as usize].len() {
            return None;
        }
        let task_index = self.top_level_matrix[top_level_type_a as usize][top_level_type_b as usize];
        if task_index < 0 {
            return None;
        }
        Some(&*self.tasks[task_index as usize])
    }
}
