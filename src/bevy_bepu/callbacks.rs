//! Default implementations of BepuPhysics callback traits.
//!
//! These read configuration from Bevy resources and per-entity components,
//! removing the need for users to implement the raw SIMD-level callbacks.

use std::simd::prelude::*;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::contact_manifold::{ConvexContactManifold, IContactManifold};
use crate::physics::collision_detection::narrow_phase_callbacks::{
    INarrowPhaseCallbacks, PairMaterialProperties,
};
use crate::physics::collision_detection::pair_cache::CollidablePair;
use crate::physics::constraints::spring_settings::SpringSettings;
use crate::physics::pose_integration::{AngularIntegrationMode, IPoseIntegratorCallbacks};
use crate::physics::simulation::Simulation;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

// ---------------------------------------------------------------------------
// Pose integrator callbacks (gravity + damping)
// ---------------------------------------------------------------------------

/// Default pose-integration callbacks that apply global gravity and damping.
///
/// Before each step the plugin copies the current [`Gravity`](super::resources::Gravity)
/// and damping values from resources into this struct.
pub struct DefaultPoseCallbacks {
    /// Gravity vector (copied from Gravity resource each frame).
    pub gravity: glam::Vec3,
    /// Fraction of linear velocity lost per second.
    pub linear_damping: f32,
    /// Fraction of angular velocity lost per second.
    pub angular_damping: f32,
    // Pre-computed per-dt values (set in prepare_for_integration).
    linear_damping_dt: f32,
    angular_damping_dt: f32,
}

impl DefaultPoseCallbacks {
    pub fn new(gravity: glam::Vec3, linear_damping: f32, angular_damping: f32) -> Self {
        Self {
            gravity,
            linear_damping,
            angular_damping,
            linear_damping_dt: 1.0,
            angular_damping_dt: 1.0,
        }
    }
}

impl IPoseIntegratorCallbacks for DefaultPoseCallbacks {
    fn angular_integration_mode(&self) -> AngularIntegrationMode {
        AngularIntegrationMode::Nonconserving
    }

    fn allow_substeps_for_unconstrained_bodies(&self) -> bool {
        false
    }

    fn integrate_velocity_for_kinematics(&self) -> bool {
        false
    }

    unsafe fn initialize(&mut self, _simulation: *mut u8) {}

    fn prepare_for_integration(&mut self, dt: f32) {
        self.linear_damping_dt = (1.0 - self.linear_damping).clamp(0.0, 1.0).powf(dt);
        self.angular_damping_dt = (1.0 - self.angular_damping).clamp(0.0, 1.0).powf(dt);
    }

    fn integrate_velocity(
        &self,
        _body_indices: Vector<i32>,
        _position: Vector3Wide,
        _orientation: QuaternionWide,
        _local_inertia: BodyInertiaWide,
        _integration_mask: Vector<i32>,
        _worker_index: i32,
        dt: Vector<f32>,
        velocity: &mut BodyVelocityWide,
    ) {
        let gx = Vector::<f32>::splat(self.gravity.x);
        let gy = Vector::<f32>::splat(self.gravity.y);
        let gz = Vector::<f32>::splat(self.gravity.z);
        let lin_damp = Vector::<f32>::splat(self.linear_damping_dt);
        let ang_damp = Vector::<f32>::splat(self.angular_damping_dt);

        velocity.linear.x = (velocity.linear.x + gx * dt) * lin_damp;
        velocity.linear.y = (velocity.linear.y + gy * dt) * lin_damp;
        velocity.linear.z = (velocity.linear.z + gz * dt) * lin_damp;
        velocity.angular.x = velocity.angular.x * ang_damp;
        velocity.angular.y = velocity.angular.y * ang_damp;
        velocity.angular.z = velocity.angular.z * ang_damp;
    }
}

// ---------------------------------------------------------------------------
// Narrow phase callbacks (friction + restitution)
// ---------------------------------------------------------------------------

/// Default narrow-phase callbacks that configure contact springs, friction,
/// and restitution from the global [`BepuConfig`](super::resources::BepuConfig) values.
pub struct DefaultNarrowPhaseCallbacks {
    /// Friction coefficient for all pairs.
    pub friction: f32,
    /// Maximum recovery velocity.
    pub max_recovery_velocity: f32,
    /// Contact spring settings.
    pub spring: SpringSettings,
}

impl DefaultNarrowPhaseCallbacks {
    pub fn new(friction: f32, max_recovery_velocity: f32, spring_frequency: f32, spring_damping_ratio: f32) -> Self {
        Self {
            friction,
            max_recovery_velocity,
            spring: SpringSettings::new(spring_frequency, spring_damping_ratio),
        }
    }
}

impl INarrowPhaseCallbacks for DefaultNarrowPhaseCallbacks {
    fn initialize(&mut self, _simulation: *mut Simulation) {}

    fn allow_contact_generation(
        &self,
        _worker_index: i32,
        _a: CollidableReference,
        _b: CollidableReference,
        _speculative_margin: &mut f32,
    ) -> bool {
        true
    }

    fn configure_contact_manifold<TManifold: IContactManifold>(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _manifold: &mut TManifold,
        pair_material: &mut PairMaterialProperties,
    ) -> bool {
        pair_material.friction_coefficient = self.friction;
        pair_material.maximum_recovery_velocity = self.max_recovery_velocity;
        pair_material.spring_settings = self.spring;
        true
    }

    fn allow_contact_generation_for_children(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
    ) -> bool {
        true
    }

    fn configure_child_contact_manifold(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
        _manifold: &mut ConvexContactManifold,
    ) -> bool {
        true
    }

    fn dispose(&mut self) {}
}
