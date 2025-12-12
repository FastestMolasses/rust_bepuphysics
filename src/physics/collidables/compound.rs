use glam::{Quat, Vec3};

use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use crate::physics::body_properties::{RigidPose, RigidPoseWide};
use super::typed_index::TypedIndex;
use super::shape::IShape;

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

    // TODO: The following methods depend on Shapes, BoundingBoxBatcher, and collision detection infrastructure:
    // - validate_child_index
    // - compute_child_bounds
    // - compute_bounds
    // - add_child_bounds_to_batcher
    // - ray_test
    // - find_local_overlaps
    // - compute_inertia
    // - add / remove_at
}

impl IShape for Compound {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
