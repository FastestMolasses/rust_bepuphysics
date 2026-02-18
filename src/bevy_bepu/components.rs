//! ECS components for the Bevy-BepuPhysics integration.
//!
//! Users spawn entities with these components; the plugin synchronizes them
//! with the underlying BepuPhysics simulation automatically.

use bevy::prelude::*;
use glam::Vec3;

use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::{BodyHandle, ConstraintHandle, StaticHandle};

// ---------------------------------------------------------------------------
// Rigid body
// ---------------------------------------------------------------------------

/// The type of rigid body. Spawn this on an entity alongside a [`BepuCollider`]
/// and a [`Transform`] to create a physics body.
///
/// # Example
/// ```ignore
/// commands.spawn((
///     RigidBody::Dynamic,
///     BepuCollider::sphere(0.5),
///     Mass(1.0),
///     Transform::from_xyz(0.0, 5.0, 0.0),
/// ));
/// ```
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Default, Reflect)]
pub enum RigidBody {
    /// A body affected by forces, gravity, and collisions.
    #[default]
    Dynamic,
    /// A body that does not move. Other bodies collide with it but it is never
    /// displaced. Internally registered as a Bepu *static*.
    Static,
    /// A body whose motion is controlled entirely by the user (via velocity or
    /// transform changes). Has infinite mass from the solver's perspective.
    Kinematic,
}

// ---------------------------------------------------------------------------
// Collider shapes
// ---------------------------------------------------------------------------

/// Describes the collision shape attached to an entity.
///
/// The plugin reads this when the entity is first added and registers the
/// corresponding Bepu shape. Changing it after creation is not yet supported
/// (despawn + respawn the entity instead).
#[derive(Component, Debug, Clone, Reflect)]
pub enum BepuCollider {
    /// A sphere with the given radius.
    Sphere { radius: f32 },
    /// A box with the given **full extents** (width, height, depth).
    Box { width: f32, height: f32, depth: f32 },
    /// A capsule aligned along the local Y axis.
    Capsule { radius: f32, length: f32 },
    /// A cylinder aligned along the local Y axis.
    Cylinder { radius: f32, length: f32 },
}

impl BepuCollider {
    /// Creates a sphere collider.
    #[inline]
    pub fn sphere(radius: f32) -> Self {
        Self::Sphere { radius }
    }

    /// Creates a box collider from **full extents** (not half-extents).
    #[inline]
    pub fn cuboid(width: f32, height: f32, depth: f32) -> Self {
        Self::Box {
            width,
            height,
            depth,
        }
    }

    /// Creates a capsule collider (Y-axis aligned).
    #[inline]
    pub fn capsule(radius: f32, length: f32) -> Self {
        Self::Capsule { radius, length }
    }

    /// Creates a cylinder collider (Y-axis aligned).
    #[inline]
    pub fn cylinder(radius: f32, length: f32) -> Self {
        Self::Cylinder { radius, length }
    }
}

// ---------------------------------------------------------------------------
// Mass
// ---------------------------------------------------------------------------

/// The mass of a dynamic body in kilograms. Ignored for statics and kinematics.
///
/// If not present, defaults to `1.0`.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Self {
        Self(1.0)
    }
}

// ---------------------------------------------------------------------------
// Velocities
// ---------------------------------------------------------------------------

/// Linear velocity of the body in world-space units per second.
///
/// Write to this component to set the velocity before the next step.
/// After each step the plugin writes back the simulated velocity.
#[derive(Component, Debug, Clone, Copy, Default, Reflect)]
pub struct LinearVelocity(pub Vec3);

/// Angular velocity of the body in radians per second (axis × speed).
///
/// Write to this component to set the angular velocity before the next step.
/// After each step the plugin writes back the simulated angular velocity.
#[derive(Component, Debug, Clone, Copy, Default, Reflect)]
pub struct AngularVelocity(pub Vec3);

// ---------------------------------------------------------------------------
// Material properties (per-entity overrides)
// ---------------------------------------------------------------------------

/// Per-entity friction coefficient override.
///
/// If absent the global default from [`BepuConfig`](super::resources::BepuConfig) is used.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct Friction(pub f32);

/// Per-entity restitution (bounciness) coefficient override.
///
/// If absent the global default from [`BepuConfig`](super::resources::BepuConfig) is used.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct Restitution(pub f32);

// ---------------------------------------------------------------------------
// Damping (per-entity overrides)
// ---------------------------------------------------------------------------

/// Per-entity linear damping override (fraction of velocity lost per second, e.g. 0.03 = 3%).
///
/// If absent the global default from [`BepuConfig`](super::resources::BepuConfig) is used.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct LinearDamping(pub f32);

/// Per-entity angular damping override (fraction of angular velocity lost per second).
///
/// If absent the global default from [`BepuConfig`](super::resources::BepuConfig) is used.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct AngularDamping(pub f32);

// ---------------------------------------------------------------------------
// Internal marker components (inserted by the plugin, not by users)
// ---------------------------------------------------------------------------

/// Inserted by the plugin after a dynamic/kinematic body is added to the Bepu simulation.
/// Stores the internal [`BodyHandle`].
#[derive(Component, Debug, Clone, Copy)]
pub struct BepuBodyHandle(pub BodyHandle);

/// Inserted by the plugin after a static body is added to the Bepu simulation.
/// Stores the internal [`StaticHandle`].
#[derive(Component, Debug, Clone, Copy)]
pub struct BepuStaticHandle(pub StaticHandle);

/// Tracks the shape index allocated in the Bepu `Shapes` collection.
#[derive(Component, Debug, Clone, Copy)]
pub struct BepuShapeIndex(pub TypedIndex);

/// Inserted by the plugin after a constraint/joint is added to the solver.
#[derive(Component, Debug, Clone, Copy)]
pub struct BepuConstraintHandle(pub ConstraintHandle);
