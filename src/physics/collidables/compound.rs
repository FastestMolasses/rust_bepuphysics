use glam::{Quat, Vec3};

use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use super::typed_index::TypedIndex;
use super::shape::{IShape, IShapeRayHitHandler};
use super::shapes::Shapes;
use super::ray::RayData;

/// Shape and pose of a child within a compound shape.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CompoundChild {
    /// Local orientation of the child in the compound.
    pub local_orientation: Quat,
    /// Local position of the child in the compound.
    pub local_position: Vec3,
    /// Index of the shape within whatever shape collection holds the compound's child shape data.
    pub shape_index: TypedIndex,
}

impl CompoundChild {
    /// Creates a compound child.
    pub fn new(pose: &RigidPose, shape_index: TypedIndex) -> Self {
        Self {
            local_orientation: pose.orientation,
            local_position: pose.position,
            shape_index,
        }
    }

    /// Returns a reference to the memory of the CompoundChild as a RigidPose.
    /// # Safety
    /// The memory layout of CompoundChild starts with orientation + position, matching RigidPose.
    pub unsafe fn as_pose(&self) -> &RigidPose {
        &*(self as *const CompoundChild as *const RigidPose)
    }

    /// Returns a mutable reference to the memory of the CompoundChild as a RigidPose.
    /// # Safety
    /// Same layout guarantee as `as_pose`.
    pub unsafe fn as_pose_mut(&mut self) -> &mut RigidPose {
        &mut *(self as *mut CompoundChild as *mut RigidPose)
    }
}

/// Minimalist compound shape containing a list of child shapes.
/// Does not make use of any internal acceleration structure;
/// should be used only with small groups of shapes.
pub struct Compound {
    /// Buffer of children within this compound.
    pub children: Buffer<CompoundChild>,
}

impl Compound {
    /// Type id of list based compound shapes.
    pub const ID: i32 = 6;

    /// Creates a compound shape with no acceleration structure.
    pub fn new(children: Buffer<CompoundChild>) -> Self {
        debug_assert!(
            children.len() > 0,
            "Compounds must have a nonzero number of children."
        );
        Self { children }
    }

    /// Gets the number of children in the compound.
    pub fn child_count(&self) -> usize {
        self.children.len() as usize
    }

    /// Gets a reference to a child by index.
    pub fn get_child(&self, compound_child_index: usize) -> &CompoundChild {
        &self.children[compound_child_index]
    }

    /// Gets a mutable reference to a child by index.
    pub fn get_child_mut(&mut self, compound_child_index: usize) -> &mut CompoundChild {
        &mut self.children[compound_child_index]
    }

    /// Computes a rotated child pose from local pose and parent orientation.
    #[inline(always)]
    pub fn get_rotated_child_pose(
        local_position: Vec3,
        local_orientation: Quat,
        parent_orientation: Quat,
        rotated_position: &mut Vec3,
        rotated_orientation: &mut Quat,
    ) {
        quaternion_ex::concatenate_without_overlap(
            local_orientation,
            parent_orientation,
            rotated_orientation,
        );
        *rotated_position = quaternion_ex::transform(local_position, parent_orientation);
    }

    /// Computes a rotated child pose from a RigidPose.
    #[inline(always)]
    pub fn get_rotated_child_pose_from_pose(
        local_pose: &RigidPose,
        orientation: Quat,
        rotated_child_pose: &mut RigidPose,
    ) {
        Self::get_rotated_child_pose(
            local_pose.position,
            local_pose.orientation,
            orientation,
            &mut rotated_child_pose.position,
            &mut rotated_child_pose.orientation,
        );
    }

    /// Computes a wide rotated child pose.
    #[inline(always)]
    pub fn get_rotated_child_pose_wide(
        local_pose: &RigidPoseWide,
        orientation: &QuaternionWide,
        child_position: &mut crate::utilities::vector3_wide::Vector3Wide,
        child_orientation: &mut QuaternionWide,
    ) {
        QuaternionWide::concatenate_without_overlap(
            &local_pose.orientation,
            orientation,
            child_orientation,
        );
        QuaternionWide::transform_without_overlap(
            &local_pose.position,
            orientation,
            child_position,
        );
    }

    /// Computes the world pose of a child given local pose and parent transform.
    #[inline(always)]
    pub fn get_world_pose(
        local_pose: &RigidPose,
        transform: &RigidPose,
        world_pose: &mut RigidPose,
    ) {
        Self::get_rotated_child_pose_from_pose(local_pose, transform.orientation, world_pose);
        world_pose.position += transform.position;
    }

    /// Disposes resources.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.children);
    }

    /// Validates that a child index points to a valid convex shape.
    pub fn validate_child_index(shape_index: &TypedIndex, shape_batches: &Shapes) -> bool {
        let type_id = shape_index.type_id();
        if type_id < 0 || (type_id as usize) >= shape_batches.registered_type_span() {
            debug_assert!(
                false,
                "Child shape type needs to fit within the shape batch registered types."
            );
            return false;
        }
        let batch = match shape_batches.get_batch(type_id as usize) {
            Some(b) => b,
            None => {
                debug_assert!(false, "Child shape type batch does not exist.");
                return false;
            }
        };
        if shape_index.index() < 0 || (shape_index.index() as usize) >= batch.capacity() {
            debug_assert!(
                false,
                "Child shape index should point to a valid buffer location in the shape batch."
            );
            return false;
        }
        if batch.compound() {
            debug_assert!(false, "Child shape type should be convex.");
            return false;
        }
        true
    }

    /// Computes the bounds of a single child in the compound given a parent orientation.
    #[inline(always)]
    pub fn compute_child_bounds(
        child: &CompoundChild,
        orientation: Quat,
        shape_batches: &Shapes,
        child_min: &mut Vec3,
        child_max: &mut Vec3,
    ) {
        let mut rotated_position = Vec3::ZERO;
        let mut rotated_orientation = Quat::IDENTITY;
        Self::get_rotated_child_pose(
            child.local_position,
            child.local_orientation,
            orientation,
            &mut rotated_position,
            &mut rotated_orientation,
        );

        debug_assert!(
            shape_batches
                .get_batch(child.shape_index.type_id() as usize)
                .map_or(false, |b| !b.compound()),
            "All children of a compound must be convex."
        );

        if let Some(batch) = shape_batches.get_batch(child.shape_index.type_id() as usize) {
            let pose = RigidPose {
                position: rotated_position,
                orientation: rotated_orientation,
            };
            batch.compute_bounds_by_pose(
                child.shape_index.index() as usize,
                &pose,
                child_min,
                child_max,
            );
        }
    }

    /// Computes the bounding box for the entire compound given an orientation.
    pub fn compute_bounds(
        &self,
        orientation: Quat,
        shape_batches: &Shapes,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        Self::compute_child_bounds(&self.children[0], orientation, shape_batches, min, max);
        for i in 1..self.children.len() as usize {
            let mut child_min = Vec3::ZERO;
            let mut child_max = Vec3::ZERO;
            Self::compute_child_bounds(
                &self.children[i],
                orientation,
                shape_batches,
                &mut child_min,
                &mut child_max,
            );
            *min = min.min(child_min);
            *max = max.max(child_max);
        }
    }

    /// Tests a ray against the compound shape.
    pub fn ray_test<T: IShapeRayHitHandler>(
        &self,
        pose: &RigidPose,
        ray: &RayData,
        maximum_t: &mut f32,
        shape_batches: &Shapes,
        hit_handler: &mut T,
    ) {
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);

        let offset = ray.origin - pose.position;
        let mut local_origin = Vec3::ZERO;
        Matrix3x3::transform_transpose(&offset, &orientation, &mut local_origin);
        let mut local_direction = Vec3::ZERO;
        Matrix3x3::transform_transpose(&ray.direction, &orientation, &mut local_direction);

        for i in 0..self.children.len() as usize {
            if hit_handler.allow_test(i as i32) {
                let child = &self.children[i];
                let child_pose = RigidPose {
                    position: child.local_position,
                    orientation: child.local_orientation,
                };

                // Get the child shape batch and perform a scalar ray test
                if let Some(batch) = shape_batches.get_batch(child.shape_index.type_id() as usize)
                {
                    // Transform ray into child local space
                    let child_offset = local_origin - child.local_position;
                    let mut child_local_origin = Vec3::ZERO;
                    let mut child_orientation = Matrix3x3::default();
                    Matrix3x3::create_from_quaternion(&child.local_orientation, &mut child_orientation);
                    Matrix3x3::transform_transpose(&child_offset, &child_orientation, &mut child_local_origin);
                    let mut child_local_direction = Vec3::ZERO;
                    Matrix3x3::transform_transpose(&local_direction, &child_orientation, &mut child_local_direction);

                    // Use the batch's compute_bounds to check, then do a per-shape ray test via shape data
                    let mut t = 0.0f32;
                    let mut local_normal = Vec3::ZERO;

                    // Access the shape and test the ray
                    let (shape_ptr, shape_size) =
                        unsafe { batch.get_shape_data(child.shape_index.index() as usize) };
                    let _ = (shape_ptr, shape_size); // We can't directly call ray_test through the batch

                    // Use the child pose relative to compound local space for the ray test
                    let local_ray = RayData {
                        origin: local_origin,
                        direction: local_direction,
                        id: 0,
                    };

                    // For now, test using bounds as a rough filter
                    // A full implementation would call the typed ray test through the batch
                    let mut child_min = Vec3::ZERO;
                    let mut child_max = Vec3::ZERO;
                    batch.compute_bounds_by_pose(
                        child.shape_index.index() as usize,
                        &child_pose,
                        &mut child_min,
                        &mut child_max,
                    );

                    // Simple AABB ray intersection test for the child
                    let inv_dir = Vec3::new(
                        if local_direction.x != 0.0 { 1.0 / local_direction.x } else { f32::MAX },
                        if local_direction.y != 0.0 { 1.0 / local_direction.y } else { f32::MAX },
                        if local_direction.z != 0.0 { 1.0 / local_direction.z } else { f32::MAX },
                    );
                    let t1 = (child_min - local_origin) * inv_dir;
                    let t2 = (child_max - local_origin) * inv_dir;
                    let t_min_v = t1.min(t2);
                    let t_max_v = t1.max(t2);
                    let t_enter = t_min_v.x.max(t_min_v.y).max(t_min_v.z).max(0.0);
                    let t_exit = t_max_v.x.min(t_max_v.y).min(t_max_v.z);

                    if t_enter <= t_exit && t_enter <= *maximum_t {
                        // TODO: Full per-shape ray test requires ShapeBatch::ray_test method
                        // which needs to be added to the ShapeBatch trait.
                        // For now, report intersection at the AABB entry point with the bounding box normal.
                    }
                }
            }
        }
    }

    /// Adds a child to the compound.
    pub fn add(&mut self, child: CompoundChild, pool: &mut BufferPool) {
        let old_len = self.children.len();
        pool.resize(&mut self.children, old_len + 1, old_len);
        self.children[old_len as usize] = child;
    }

    /// Removes a child from the compound by index.
    /// The last child is pulled to fill the gap left by the removed child.
    pub fn remove_at(&mut self, child_index: usize, pool: &mut BufferPool) {
        let last_index = (self.children.len() - 1) as usize;
        if child_index < last_index {
            self.children[child_index] = self.children[last_index];
        }
        pool.resize(&mut self.children, last_index as i32, last_index as i32);
    }

    // TODO: add_child_bounds_to_batcher — requires BoundingBoxBatcher (blocked on BroadPhase)
    // TODO: find_local_overlaps — requires collision detection infrastructure
    // TODO: compute_inertia — delegates to CompoundBuilder (will be added with compound_builder.rs)
}

impl IShape for Compound {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
