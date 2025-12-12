use crate::physics::body_properties::{BodyInertia, BodyVelocity, RigidPose};
use crate::physics::collidables::collidable_description::CollidableDescription;
use crate::physics::collidables::shape::IConvexShape;

/// Describes the thresholds for a body going to sleep.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyActivityDescription {
    /// Threshold of squared velocity under which the body is allowed to go to sleep.
    /// This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).
    pub sleep_threshold: f32,
    /// The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
    /// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.
    pub minimum_timestep_count_under_threshold: u8,
}

impl BodyActivityDescription {
    /// Creates a body activity description.
    #[inline(always)]
    pub fn new(sleep_threshold: f32, minimum_timestep_count_under_threshold: u8) -> Self {
        Self {
            sleep_threshold,
            minimum_timestep_count_under_threshold,
        }
    }

    /// Creates a body activity description with a default `minimum_timestep_count_under_threshold` of 32.
    #[inline(always)]
    pub fn with_default_threshold(sleep_threshold: f32) -> Self {
        Self {
            sleep_threshold,
            minimum_timestep_count_under_threshold: 32,
        }
    }
}

impl From<f32> for BodyActivityDescription {
    fn from(sleep_threshold: f32) -> Self {
        Self::with_default_threshold(sleep_threshold)
    }
}

/// Describes a body's state.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BodyDescription {
    /// Position and orientation of the body.
    pub pose: RigidPose,
    /// Linear and angular velocity of the body.
    pub velocity: BodyVelocity,
    /// Mass and inertia tensor of the body.
    pub local_inertia: BodyInertia,
    /// Shape and collision detection settings for the body.
    pub collidable: CollidableDescription,
    /// Sleeping settings for the body.
    pub activity: BodyActivityDescription,
}

impl BodyDescription {
    /// Computes a decent default speculative margin for a shape based on its minimum and maximum radii.
    #[inline(always)]
    pub fn get_default_speculative_margin<TShape: IConvexShape>(shape: &TShape) -> f32 {
        let mut maximum_radius = 0.0f32;
        let mut maximum_angular_expansion = 0.0f32;
        shape.compute_angular_expansion_data(&mut maximum_radius, &mut maximum_angular_expansion);
        let minimum_radius = maximum_radius - maximum_angular_expansion;
        0.1 * (maximum_radius * minimum_radius).sqrt()
    }

    /// Computes a decent default activity description for a shape.
    #[inline(always)]
    pub fn get_default_activity<TShape: IConvexShape>(shape: &TShape) -> BodyActivityDescription {
        let mut maximum_radius = 0.0f32;
        let mut maximum_angular_expansion = 0.0f32;
        shape.compute_angular_expansion_data(&mut maximum_radius, &mut maximum_angular_expansion);
        let minimum_radius = maximum_radius - maximum_angular_expansion;
        BodyActivityDescription {
            minimum_timestep_count_under_threshold: 32,
            sleep_threshold: minimum_radius * minimum_radius * 0.1,
        }
    }

    /// Creates a dynamic body description.
    #[inline(always)]
    pub fn create_dynamic(
        pose: RigidPose,
        velocity: BodyVelocity,
        inertia: BodyInertia,
        collidable: CollidableDescription,
        activity: BodyActivityDescription,
    ) -> Self {
        Self {
            pose,
            velocity,
            local_inertia: inertia,
            activity,
            collidable,
        }
    }

    /// Creates a dynamic body description with zero initial velocity.
    #[inline(always)]
    pub fn create_dynamic_no_velocity(
        pose: RigidPose,
        inertia: BodyInertia,
        collidable: CollidableDescription,
        activity: BodyActivityDescription,
    ) -> Self {
        Self {
            pose,
            velocity: BodyVelocity::default(),
            local_inertia: inertia,
            activity,
            collidable,
        }
    }

    /// Creates a kinematic body description.
    #[inline(always)]
    pub fn create_kinematic(
        pose: RigidPose,
        velocity: BodyVelocity,
        collidable: CollidableDescription,
        activity: BodyActivityDescription,
    ) -> Self {
        Self {
            pose,
            velocity,
            local_inertia: BodyInertia::default(),
            activity,
            collidable,
        }
    }

    /// Creates a kinematic body description with zero initial velocity.
    #[inline(always)]
    pub fn create_kinematic_no_velocity(
        pose: RigidPose,
        collidable: CollidableDescription,
        activity: BodyActivityDescription,
    ) -> Self {
        Self {
            pose,
            velocity: BodyVelocity::default(),
            local_inertia: BodyInertia::default(),
            activity,
            collidable,
        }
    }
}
