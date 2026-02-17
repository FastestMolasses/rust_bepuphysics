//! Bevy integration for the BepuPhysics engine.
//!
//! This module provides an ergonomic ECS-based API for the low-level
//! BepuPhysics simulation. Users spawn entities with [`RigidBody`],
//! [`BepuCollider`], and [`Transform`] components; the plugin handles all
//! simulation management, transform synchronization, and dispatch.
//!
//! # Quick start
//! ```ignore
//! use bevy::prelude::*;
//! use rust_bepuphysics::bevy_bepu::prelude::*;
//!
//! fn main() {
//!     App::new()
//!         .add_plugins((DefaultPlugins, BepuPhysicsPlugin))
//!         .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
//!         .add_systems(Startup, setup)
//!         .run();
//! }
//!
//! fn setup(mut commands: Commands) {
//!     // Static floor
//!     commands.spawn((
//!         RigidBody::Static,
//!         BepuCollider::cuboid(100.0, 1.0, 100.0),
//!         Transform::from_xyz(0.0, -0.5, 0.0),
//!     ));
//!     // Dynamic sphere
//!     commands.spawn((
//!         RigidBody::Dynamic,
//!         BepuCollider::sphere(0.5),
//!         Mass(1.0),
//!         Transform::from_xyz(0.0, 5.0, 0.0),
//!     ));
//! }
//! ```

pub mod callbacks;
pub mod components;
pub mod joints;
pub mod plugin;
pub mod resources;
pub mod spatial_query;
pub mod systems;

/// Convenience re-exports for typical usage.
///
/// ```ignore
/// use rust_bepuphysics::bevy_bepu::prelude::*;
/// ```
pub mod prelude {
    pub use super::components::{
        AngularDamping, AngularVelocity, BepuBodyHandle, BepuCollider, BepuStaticHandle,
        Friction, LinearDamping, LinearVelocity, Mass, Restitution, RigidBody,
    };
    pub use super::joints::{BallSocketJoint, DistanceJoint, WeldJoint};
    pub use super::plugin::{BepuPhysicsPlugin, BepuPhysicsPlugins, BepuSet};
    pub use super::resources::{BepuConfig, BepuSimulation, Gravity};
    pub use super::spatial_query::{BepuSpatialQuery, RayHit};
}
