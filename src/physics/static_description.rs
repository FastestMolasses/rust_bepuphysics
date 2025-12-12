use glam::{Quat, Vec3};

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::collidable::ContinuousDetection;
use crate::physics::collidables::typed_index::TypedIndex;

/// Describes the properties of a static object. When added to a simulation, static objects can
/// collide but have no velocity and will not move in response to forces.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct StaticDescription {
    /// Position and orientation of the static.
    pub pose: RigidPose,
    /// Shape of the static.
    pub shape: TypedIndex,
    /// Continuous collision detection settings for the static.
    pub continuity: ContinuousDetection,
}

impl StaticDescription {
    /// Builds a new static description.
    #[inline(always)]
    pub fn new(pose: RigidPose, shape: TypedIndex, continuity: ContinuousDetection) -> Self {
        Self {
            pose,
            shape,
            continuity,
        }
    }

    /// Builds a new static description with `Discrete` continuity.
    #[inline(always)]
    pub fn with_discrete(pose: RigidPose, shape: TypedIndex) -> Self {
        Self {
            pose,
            shape,
            continuity: ContinuousDetection::discrete(),
        }
    }

    /// Builds a new static description from position, orientation, and shape.
    #[inline(always)]
    pub fn from_pose_parts(
        position: Vec3,
        orientation: Quat,
        shape: TypedIndex,
        continuity: ContinuousDetection,
    ) -> Self {
        Self {
            pose: RigidPose::new(position, orientation),
            shape,
            continuity,
        }
    }

    /// Builds a new static description from position, orientation, and shape with `Discrete` continuity.
    #[inline(always)]
    pub fn from_pose_parts_discrete(
        position: Vec3,
        orientation: Quat,
        shape: TypedIndex,
    ) -> Self {
        Self {
            pose: RigidPose::new(position, orientation),
            shape,
            continuity: ContinuousDetection::discrete(),
        }
    }
}
