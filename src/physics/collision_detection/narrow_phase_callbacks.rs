// Translated from BepuPhysics/CollisionDetection/INarrowPhaseCallbacks.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::constraints::spring_settings::SpringSettings;
use super::contact_manifold::{ConvexContactManifold, IContactManifold};
use super::pair_cache::CollidablePair;

/// Material properties governing the interaction between colliding bodies.
/// Used by the narrow phase to create constraints of the appropriate configuration.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PairMaterialProperties {
    /// Coefficient of friction to apply for the constraint.
    /// Maximum friction force will be equal to the normal force times the friction coefficient.
    pub friction_coefficient: f32,
    /// Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration.
    /// Clamps the velocity goal created from the spring settings.
    pub maximum_recovery_velocity: f32,
    /// Defines the constraint's penetration recovery spring properties.
    pub spring_settings: SpringSettings,
}

impl PairMaterialProperties {
    /// Constructs a pair's material properties.
    #[inline(always)]
    pub fn new(
        friction_coefficient: f32,
        maximum_recovery_velocity: f32,
        spring_settings: SpringSettings,
    ) -> Self {
        Self {
            friction_coefficient,
            maximum_recovery_velocity,
            spring_settings,
        }
    }
}

impl Default for PairMaterialProperties {
    fn default() -> Self {
        Self {
            friction_coefficient: 0.0,
            maximum_recovery_velocity: 0.0,
            spring_settings: SpringSettings::default(),
        }
    }
}

/// Defines handlers for narrow phase events.
pub trait INarrowPhaseCallbacks {
    /// Performs any required initialization logic after the Simulation instance has been constructed.
    fn initialize(&mut self, simulation: *mut crate::physics::simulation::Simulation);

    /// Chooses whether to allow contact generation to proceed for two overlapping collidables.
    fn allow_contact_generation(
        &self,
        worker_index: i32,
        a: CollidableReference,
        b: CollidableReference,
        speculative_margin: &mut f32,
    ) -> bool;

    /// Provides a notification that a manifold has been created for a pair.
    /// Offers an opportunity to change the manifold's details.
    /// Returns true if a constraint should be created for the manifold, false otherwise.
    fn configure_contact_manifold<TManifold: IContactManifold>(
        &self,
        worker_index: i32,
        pair: CollidablePair,
        manifold: &mut TManifold,
        pair_material: &mut PairMaterialProperties,
    ) -> bool;

    /// Chooses whether to allow contact generation to proceed for the children of two overlapping collidables
    /// in a compound-including pair.
    fn allow_contact_generation_for_children(
        &self,
        worker_index: i32,
        pair: CollidablePair,
        child_index_a: i32,
        child_index_b: i32,
    ) -> bool;

    /// Provides a notification that a manifold has been created between the children of two collidables
    /// in a compound-including pair.
    /// Returns true if this manifold should be considered for the parent pair's contact manifold generation, false otherwise.
    fn configure_child_contact_manifold(
        &self,
        worker_index: i32,
        pair: CollidablePair,
        child_index_a: i32,
        child_index_b: i32,
        manifold: &mut ConvexContactManifold,
    ) -> bool;

    /// Releases any resources held by the callbacks.
    fn dispose(&mut self);
}
