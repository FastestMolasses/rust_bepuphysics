//! The main plugin group that users add to their Bevy `App`.

use bevy::prelude::*;

use super::components::*;
use super::joints::*;
use super::resources::*;
use super::systems;

// ---------------------------------------------------------------------------
// System sets
// ---------------------------------------------------------------------------

/// Ordering sets for the BepuPhysics systems within `FixedPostUpdate`.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum BepuSet {
    /// Prepare phase: add/remove bodies, sync user changes to simulation.
    Prepare,
    /// The simulation timestep.
    Step,
    /// Post-step: write simulation results back to ECS.
    Writeback,
}

// ---------------------------------------------------------------------------
// Plugin group
// ---------------------------------------------------------------------------

/// Add BepuPhysics to your Bevy app.
///
/// # Example
/// ```ignore
/// App::new()
///     .add_plugins((DefaultPlugins, BepuPhysicsPlugin::default()))
///     .insert_resource(Gravity(Vec3::new(0.0, -20.0, 0.0)))
///     .add_systems(Startup, setup)
///     .run();
/// ```
pub struct BepuPhysicsPlugin;

impl Plugin for BepuPhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Insert default resources if the user hasn't already.
        app.init_resource::<BepuConfig>();
        app.init_resource::<Gravity>();

        // Register types for reflection.
        app.register_type::<RigidBody>();
        app.register_type::<BepuCollider>();
        app.register_type::<Mass>();
        app.register_type::<LinearVelocity>();
        app.register_type::<AngularVelocity>();
        app.register_type::<Friction>();
        app.register_type::<Restitution>();
        app.register_type::<LinearDamping>();
        app.register_type::<AngularDamping>();
        app.register_type::<Gravity>();
        app.register_type::<BepuConfig>();

        // Configure system set ordering.
        app.configure_sets(
            FixedPostUpdate,
            (BepuSet::Prepare, BepuSet::Step, BepuSet::Writeback).chain(),
        );

        // Startup: create the simulation.
        app.add_systems(Startup, systems::initialize_simulation);

        // Fixed-rate systems.
        app.add_systems(
            FixedPostUpdate,
            (
                systems::add_new_bodies,
                systems::remove_despawned_bodies,
                systems::sync_velocities_to_bepu,
                systems::sync_transforms_to_bepu,
                systems::update_callback_data,
            )
                .chain()
                .in_set(BepuSet::Prepare),
        );

        app.add_systems(
            FixedPostUpdate,
            systems::step_simulation.in_set(BepuSet::Step),
        );

        app.add_systems(
            FixedPostUpdate,
            (
                systems::sync_bepu_to_transforms,
                systems::sync_bepu_to_velocities,
            )
                .in_set(BepuSet::Writeback),
        );
    }
}

/// Plugin group containing the full BepuPhysics integration.
///
/// Equivalent to `.add_plugins(BepuPhysicsPlugin)` but structured as a
/// [`PluginGroup`] for forward-compatibility with future sub-plugins
/// (joints, spatial queries, debug rendering).
pub struct BepuPhysicsPlugins;

impl PluginGroup for BepuPhysicsPlugins {
    fn build(self) -> bevy::app::PluginGroupBuilder {
        bevy::app::PluginGroupBuilder::start::<Self>().add(BepuPhysicsPlugin)
    }
}
