//! Joint / constraint components for the Bevy-BepuPhysics integration.
//!
//! Spawn a joint as its own entity. The plugin reads it and registers the
//! corresponding Bepu constraint.

use bevy::prelude::*;

use crate::physics::constraints::spring_settings::SpringSettings;

// ---------------------------------------------------------------------------
// Ball-socket joint
// ---------------------------------------------------------------------------

/// A ball-and-socket joint connecting two bodies at local anchor points.
///
/// Spawn as its own entity:
/// ```ignore
/// commands.spawn(BallSocketJoint::new(entity_a, entity_b));
/// ```
#[derive(Component, Debug, Clone)]
pub struct BallSocketJoint {
    /// First body entity.
    pub entity_a: Entity,
    /// Second body entity.
    pub entity_b: Entity,
    /// Anchor point in the local space of body A.
    pub local_offset_a: Vec3,
    /// Anchor point in the local space of body B.
    pub local_offset_b: Vec3,
    /// Spring settings for the constraint.
    pub spring: SpringSettings,
}

impl BallSocketJoint {
    /// Creates a ball-socket joint between two entities at their origins.
    pub fn new(entity_a: Entity, entity_b: Entity) -> Self {
        Self {
            entity_a,
            entity_b,
            local_offset_a: Vec3::ZERO,
            local_offset_b: Vec3::ZERO,
            spring: SpringSettings::new(30.0, 1.0),
        }
    }

    /// Sets the local anchor on body A.
    pub fn with_local_offset_a(mut self, offset: Vec3) -> Self {
        self.local_offset_a = offset;
        self
    }

    /// Sets the local anchor on body B.
    pub fn with_local_offset_b(mut self, offset: Vec3) -> Self {
        self.local_offset_b = offset;
        self
    }

    /// Sets the spring parameters.
    pub fn with_spring(mut self, frequency: f32, damping_ratio: f32) -> Self {
        self.spring = SpringSettings::new(frequency, damping_ratio);
        self
    }
}

// ---------------------------------------------------------------------------
// Weld joint
// ---------------------------------------------------------------------------

/// A weld constraint that locks two bodies' relative pose.
#[derive(Component, Debug, Clone)]
pub struct WeldJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_offset_a: Vec3,
    pub local_offset_b: Vec3,
    pub spring: SpringSettings,
}

impl WeldJoint {
    pub fn new(entity_a: Entity, entity_b: Entity) -> Self {
        Self {
            entity_a,
            entity_b,
            local_offset_a: Vec3::ZERO,
            local_offset_b: Vec3::ZERO,
            spring: SpringSettings::new(30.0, 1.0),
        }
    }

    pub fn with_local_offset_a(mut self, offset: Vec3) -> Self {
        self.local_offset_a = offset;
        self
    }

    pub fn with_local_offset_b(mut self, offset: Vec3) -> Self {
        self.local_offset_b = offset;
        self
    }

    pub fn with_spring(mut self, frequency: f32, damping_ratio: f32) -> Self {
        self.spring = SpringSettings::new(frequency, damping_ratio);
        self
    }
}

// ---------------------------------------------------------------------------
// Distance joint
// ---------------------------------------------------------------------------

/// A distance constraint that maintains a target distance between two anchors.
#[derive(Component, Debug, Clone)]
pub struct DistanceJoint {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub local_offset_a: Vec3,
    pub local_offset_b: Vec3,
    /// Target distance to maintain.
    pub target_distance: f32,
    pub spring: SpringSettings,
}

impl DistanceJoint {
    pub fn new(entity_a: Entity, entity_b: Entity, target_distance: f32) -> Self {
        Self {
            entity_a,
            entity_b,
            local_offset_a: Vec3::ZERO,
            local_offset_b: Vec3::ZERO,
            target_distance,
            spring: SpringSettings::new(30.0, 1.0),
        }
    }

    pub fn with_local_offset_a(mut self, offset: Vec3) -> Self {
        self.local_offset_a = offset;
        self
    }

    pub fn with_local_offset_b(mut self, offset: Vec3) -> Self {
        self.local_offset_b = offset;
        self
    }

    pub fn with_spring(mut self, frequency: f32, damping_ratio: f32) -> Self {
        self.spring = SpringSettings::new(frequency, damping_ratio);
        self
    }
}
