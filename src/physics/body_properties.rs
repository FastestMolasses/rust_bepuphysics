use glam::{Quat, Vec3};
use std::fmt;

use crate::utilities::quaternion_ex;
use crate::utilities::symmetric3x3::Symmetric3x3;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;

/// Represents a rigid transformation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RigidPose {
    /// Orientation of the pose.
    pub orientation: Quat,
    /// Position of the pose.
    pub position: Vec3,
}

// Ensure layout matches C# [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 1)]
const _: () = {
    assert!(std::mem::size_of::<RigidPose>() == 32);
};

impl Default for RigidPose {
    #[inline(always)]
    fn default() -> Self {
        Self {
            orientation: Quat::IDENTITY,
            position: Vec3::ZERO,
        }
    }
}

impl RigidPose {
    /// Returns a pose with a position at (0,0,0) and identity orientation.
    pub const IDENTITY: Self = Self {
        orientation: Quat::IDENTITY,
        position: Vec3::ZERO,
    };

    /// Creates a rigid pose with the given position and orientation.
    #[inline(always)]
    pub fn new(position: Vec3, orientation: Quat) -> Self {
        Self {
            position,
            orientation,
        }
    }

    /// Creates a rigid pose with the given position and identity orientation.
    #[inline(always)]
    pub fn from_position(position: Vec3) -> Self {
        Self {
            position,
            orientation: Quat::IDENTITY,
        }
    }

    /// Transforms a vector by the rigid pose: v * pose.Orientation + pose.Position.
    #[inline(always)]
    pub fn transform(v: Vec3, pose: &RigidPose, result: &mut Vec3) {
        let mut rotated = Vec3::ZERO;
        quaternion_ex::transform_without_overlap(v, pose.orientation, &mut rotated);
        *result = rotated + pose.position;
    }

    /// Transforms a vector by the inverse of a rigid pose: (v - pose.Position) * pose.Orientation^-1.
    #[inline(always)]
    pub fn transform_by_inverse(v: Vec3, pose: &RigidPose, result: &mut Vec3) {
        let translated = v - pose.position;
        let conjugate = quaternion_ex::conjugate(pose.orientation);
        quaternion_ex::transform_without_overlap(translated, conjugate, result);
    }

    /// Inverts the rigid transformation of the pose.
    #[inline(always)]
    pub fn invert(pose: &RigidPose, inverse: &mut RigidPose) {
        inverse.orientation = quaternion_ex::conjugate(pose.orientation);
        inverse.position = quaternion_ex::transform(-pose.position, inverse.orientation);
    }

    /// Concatenates one rigid transform with another. The resulting transform is equivalent
    /// to performing transform a followed by transform b.
    #[inline(always)]
    pub fn multiply_without_overlap(a: &RigidPose, b: &RigidPose, result: &mut RigidPose) {
        quaternion_ex::concatenate_without_overlap(a.orientation, b.orientation, &mut result.orientation);
        let rotated_translation_a = quaternion_ex::transform(a.position, b.orientation);
        result.position = rotated_translation_a + b.position;
    }
}

impl From<Vec3> for RigidPose {
    fn from(position: Vec3) -> Self {
        Self::from_position(position)
    }
}

impl From<Quat> for RigidPose {
    fn from(orientation: Quat) -> Self {
        Self {
            position: Vec3::ZERO,
            orientation,
        }
    }
}

impl From<(Vec3, Quat)> for RigidPose {
    fn from((position, orientation): (Vec3, Quat)) -> Self {
        Self::new(position, orientation)
    }
}

impl fmt::Display for RigidPose {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {}", self.position, self.orientation)
    }
}

/// Linear and angular velocity for a body.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyVelocity {
    /// Linear velocity associated with the body.
    pub linear: Vec3,
    _pad0: f32,
    /// Angular velocity associated with the body.
    pub angular: Vec3,
    _pad1: f32,
}

// Ensure layout matches C# [StructLayout(LayoutKind.Explicit, Size = 32)]
// with Linear at offset 0 and Angular at offset 16
const _: () = {
    assert!(std::mem::size_of::<BodyVelocity>() == 32);
    assert!(std::mem::offset_of!(BodyVelocity, linear) == 0);
    assert!(std::mem::offset_of!(BodyVelocity, angular) == 16);
};

impl Default for BodyVelocity {
    fn default() -> Self {
        Self {
            linear: Vec3::ZERO,
            _pad0: 0.0,
            angular: Vec3::ZERO,
            _pad1: 0.0,
        }
    }
}

impl BodyVelocity {
    /// Creates a new set of body velocities. Angular velocity is set to zero.
    #[inline(always)]
    pub fn from_linear(linear: Vec3) -> Self {
        Self {
            linear,
            _pad0: 0.0,
            angular: Vec3::ZERO,
            _pad1: 0.0,
        }
    }

    /// Creates a new set of body velocities.
    #[inline(always)]
    pub fn new(linear: Vec3, angular: Vec3) -> Self {
        Self {
            linear,
            _pad0: 0.0,
            angular,
            _pad1: 0.0,
        }
    }
}

impl From<Vec3> for BodyVelocity {
    fn from(linear: Vec3) -> Self {
        Self::from_linear(linear)
    }
}

impl fmt::Display for BodyVelocity {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {}", self.linear, self.angular)
    }
}

/// Stores the inertia for a body.
///
/// This representation stores the inverse mass and inverse inertia tensor.
/// Most of the high frequency use cases in the engine naturally use the inverse.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyInertia {
    /// Inverse of the body's inertia tensor.
    pub inverse_inertia_tensor: Symmetric3x3,
    /// Inverse of the body's mass.
    pub inverse_mass: f32,
    /// Padding to match C# [StructLayout(Size = 32)].
    _padding: f32,
}

// Ensure layout matches C# [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 4)]
const _: () = {
    assert!(std::mem::size_of::<BodyInertia>() == 32);
};

impl fmt::Display for BodyInertia {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {:?}", self.inverse_mass, self.inverse_inertia_tensor)
    }
}

/// Stores the local and world views of a body's inertia, packed together for efficient access.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyInertias {
    /// Local inertia of the body.
    pub local: BodyInertia,
    /// Transformed world inertia of the body.
    pub world: BodyInertia,
}

/// Describes the pose and velocity of a body.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MotionState {
    /// Pose of the body.
    pub pose: RigidPose,
    /// Linear and angular velocity of the body.
    pub velocity: BodyVelocity,
}

// Ensure layout matches C# [StructLayout(LayoutKind.Sequential, Size = 64, Pack = 1)]
const _: () = {
    assert!(std::mem::size_of::<MotionState>() == 64);
};

impl Default for MotionState {
    fn default() -> Self {
        Self {
            pose: RigidPose::default(),
            velocity: BodyVelocity::default(),
        }
    }
}

/// Stores all body information needed by the solver together.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyDynamics {
    /// Pose and velocity information for the body.
    pub motion: MotionState,
    /// Inertia information for the body.
    pub inertia: BodyInertias,
}

// --- Wide types ---

#[derive(Clone, Copy, Default)]
pub struct RigidPoseWide {
    pub position: Vector3Wide,
    pub orientation: QuaternionWide,
}

impl RigidPoseWide {
    #[inline(always)]
    pub fn broadcast(pose: &RigidPose, poses: &mut RigidPoseWide) {
        Vector3Wide::broadcast_to(pose.position, &mut poses.position);
        QuaternionWide::broadcast(pose.orientation, &mut poses.orientation);
    }

    #[inline(always)]
    pub fn write_first(pose: &RigidPose, poses: &mut RigidPoseWide) {
        Vector3Wide::write_first(pose.position, &mut poses.position);
        QuaternionWide::write_first(pose.orientation, &mut poses.orientation);
    }

    #[inline(always)]
    pub fn read_first(poses: &RigidPoseWide, pose: &mut RigidPose) {
        Vector3Wide::read_first(&poses.position, &mut pose.position);
        QuaternionWide::read_first(&poses.orientation, &mut pose.orientation);
    }
}

#[derive(Clone, Copy, Default)]
pub struct BodyVelocityWide {
    pub linear: Vector3Wide,
    pub angular: Vector3Wide,
}

#[derive(Clone, Copy, Default)]
pub struct BodyInertiaWide {
    pub inverse_inertia_tensor: Symmetric3x3Wide,
    pub inverse_mass: Vector<f32>,
}

/// Describes how a body sleeps, and its current state with respect to sleeping.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyActivity {
    /// Threshold of squared velocity under which the body is allowed to go to sleep.
    pub sleep_threshold: f32,
    /// The number of time steps that the body must be under the sleep threshold before becoming a sleep candidate.
    pub minimum_timesteps_under_threshold: u8,
    /// If the body is awake, this is the number of time steps that the body has had a velocity below the sleep threshold.
    pub timesteps_under_threshold_count: u8,
    /// True if this body is a candidate for being slept. If all the bodies that it is connected to by constraints are also candidates, this body may go to sleep.
    pub sleep_candidate: bool,
}

impl Default for BodyActivity {
    fn default() -> Self {
        Self {
            sleep_threshold: 0.0,
            minimum_timesteps_under_threshold: 0,
            timesteps_under_threshold_count: 0,
            sleep_candidate: false,
        }
    }
}
