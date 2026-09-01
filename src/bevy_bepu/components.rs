//! ECS components for the Bevy-BepuPhysics integration.
//!
//! Users spawn entities with these components; the plugin synchronizes them
//! with the underlying BepuPhysics simulation automatically.

use bevy::ecs::lifecycle::HookContext;
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;
use glam::Vec3;

use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::{BodyHandle, ConstraintHandle, StaticHandle};
use crate::utilities::symmetric3x3::Symmetric3x3;

use super::resources::BepuRemovalQueue;

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
/// After each step the plugin writes back the simulated velocity — but only when the value actually
/// differs, so a settled body does not trip change detection every tick.
#[derive(Component, Debug, Clone, Copy, Default, PartialEq, Reflect)]
pub struct LinearVelocity(pub Vec3);

/// Angular velocity of the body in radians per second (axis × speed).
///
/// Write to this component to set the angular velocity before the next step.
/// After each step the plugin writes back the simulated angular velocity — but only when the value
/// actually differs, so a settled body does not trip change detection every tick.
#[derive(Component, Debug, Clone, Copy, Default, PartialEq, Reflect)]
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
// Inertia / collidable tuning
// ---------------------------------------------------------------------------

/// Prevents the body from rotating about the selected axes. Mass (linear response) is unaffected.
///
/// Locking works by zeroing the corresponding rows and columns of the body's **local** inverse
/// inertia tensor, which is what the solver and pose integrator consult for every angular response.
/// A zeroed axis has infinite inertia about it, so no torque, contact, or joint can produce
/// rotation there. Directly written [`AngularVelocity`] is projected through the same mask, so a
/// locked axis stays locked no matter how the spin was introduced.
///
/// ```ignore
/// // A character or any prop that must stay upright but may still turn to face things.
/// commands.spawn((RigidBody::Dynamic, collider, LockedRotation::UPRIGHT));
///
/// // Any combination.
/// commands.spawn((RigidBody::Dynamic, collider, LockedRotation::X | LockedRotation::Y));
/// ```
///
/// # Which frame the axes are in
///
/// The mask names axes of the body's **local** frame, and the local tensor is rotated into world
/// space each step. That distinction only ever matters for a mask that locks exactly one axis:
///
/// - [`ALL`](Self::ALL) — no rotation at all, so local and world are trivially the same thing.
/// - **Two axes** (e.g. [`UPRIGHT`](Self::UPRIGHT)) — the body can only spin about the single free
///   axis, and spinning about an axis cannot move that axis, so the free axis is pinned in world
///   space too. This is exact: `LockedRotation::UPRIGHT` on a body spawned upright keeps its local
///   Y aligned with world Y forever, while yaw stays free.
/// - **One axis** — the body is still free to rotate about the other two, which carries the locked
///   axis around with it. The lock therefore follows the body rather than the world. That is the
///   right behaviour for "this wheel may not spin about its own axle", and the wrong one if you
///   wanted a world-space constraint; use a joint for the latter.
///
/// Only meaningful on [`RigidBody::Dynamic`]; statics and kinematics already have zero inverse
/// inertia.
///
/// # Locking a rotation a joint also constrains
///
/// An angular constraint builds its effective mass from nothing but the two bodies' inverse inertia
/// tensors, and Bepu inverts that matrix without a singularity guard, matching the C# original.
/// Locking every axis a joint is trying to drive on *both* of its bodies therefore leaves that
/// matrix singular and the solve produces infinities. Bepu has the same hazard for an angular
/// constraint between two kinematics; the mask just makes it reachable in more configurations. Leave
/// at least one of the two bodies free about the axes its angular joints constrain.
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Hash, Reflect)]
pub struct LockedRotation(u8);

impl LockedRotation {
    /// Locks nothing; equivalent to leaving the component off.
    pub const NONE: Self = Self(0);
    /// Locks rotation about the body's local X axis.
    pub const X: Self = Self(1 << 0);
    /// Locks rotation about the body's local Y axis.
    pub const Y: Self = Self(1 << 1);
    /// Locks rotation about the body's local Z axis.
    pub const Z: Self = Self(1 << 2);
    /// Locks every axis, so no torque can rotate the body at all.
    pub const ALL: Self = Self(0b111);
    /// Locks pitch and roll, leaving yaw about Y free.
    pub const UPRIGHT: Self = Self(0b101);

    /// Builds a mask from three flags.
    #[inline]
    pub const fn new(x: bool, y: bool, z: bool) -> Self {
        Self((x as u8) | ((y as u8) << 1) | ((z as u8) << 2))
    }

    /// Whether rotation about the local X axis is locked.
    #[inline]
    pub const fn x(self) -> bool {
        self.0 & Self::X.0 != 0
    }

    /// Whether rotation about the local Y axis is locked.
    #[inline]
    pub const fn y(self) -> bool {
        self.0 & Self::Y.0 != 0
    }

    /// Whether rotation about the local Z axis is locked.
    #[inline]
    pub const fn z(self) -> bool {
        self.0 & Self::Z.0 != 0
    }

    /// Whether this mask locks no axes at all.
    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Whether this mask locks every axis.
    #[inline]
    pub const fn is_all(self) -> bool {
        self.0 == Self::ALL.0
    }

    /// Applies the mask to a local inverse inertia tensor, in place.
    pub fn apply_to_inverse_inertia(self, tensor: &mut Symmetric3x3) {
        if self.x() {
            tensor.xx = 0.0;
            tensor.yx = 0.0;
            tensor.zx = 0.0;
        }
        if self.y() {
            tensor.yy = 0.0;
            tensor.yx = 0.0;
            tensor.zy = 0.0;
        }
        if self.z() {
            tensor.zz = 0.0;
            tensor.zx = 0.0;
            tensor.zy = 0.0;
        }
    }

    /// Zeroes the locked components of an angular velocity, in the body's local frame.
    ///
    /// `orientation` is the body's world orientation; the velocity is rotated into local space,
    /// masked, and rotated back. [`ALL`](Self::ALL) short-circuits to zero, which is both faster and
    /// exact regardless of orientation.
    pub fn apply_to_angular_velocity(self, orientation: glam::Quat, angular: Vec3) -> Vec3 {
        if self.is_empty() {
            return angular;
        }
        if self.is_all() {
            return Vec3::ZERO;
        }
        let mut local = orientation.conjugate() * angular;
        if self.x() {
            local.x = 0.0;
        }
        if self.y() {
            local.y = 0.0;
        }
        if self.z() {
            local.z = 0.0;
        }
        orientation * local
    }
}

impl Default for LockedRotation {
    fn default() -> Self {
        Self::ALL
    }
}

impl std::ops::BitOr for LockedRotation {
    type Output = Self;
    #[inline]
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for LockedRotation {
    #[inline]
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Overrides the speculative contact margin bounds of a body's collidable.
///
/// The speculative margin is how far ahead of the surface the narrow phase is willing to create
/// contacts. A non-zero *minimum* makes contacts exist slightly before the surfaces actually
/// touch.
///
/// If absent, Bepu's defaults are used (minimum 0, maximum `f32::MAX`).
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct SpeculativeMargin {
    /// Lower bound on the speculative margin.
    pub minimum: f32,
    /// Upper bound on the speculative margin.
    pub maximum: f32,
}

impl SpeculativeMargin {
    /// Creates a margin with the given minimum and an unbounded maximum.
    #[inline]
    pub fn at_least(minimum: f32) -> Self {
        Self {
            minimum,
            maximum: f32::MAX,
        }
    }
}

impl Default for SpeculativeMargin {
    fn default() -> Self {
        Self {
            minimum: 0.0,
            maximum: f32::MAX,
        }
    }
}

/// Overrides the squared-velocity threshold below which a body may go to sleep.
///
/// A negative value prevents the body from ever sleeping.
#[derive(Component, Debug, Clone, Copy, Reflect)]
pub struct SleepThreshold(pub f32);

impl SleepThreshold {
    /// A body that never sleeps.
    pub const NEVER: Self = Self(-1.0);
}

// ---------------------------------------------------------------------------
// Query layers
// ---------------------------------------------------------------------------

/// Bitmask identifying which query layers this body or static belongs to.
///
/// Spatial queries carry a mask of their own
/// (see [`QueryFilter`](super::spatial_query::QueryFilter)); a collidable is considered only if
/// `collidable_layers & query_mask != 0`.
///
/// **Entities without this component belong to layer 0** ([`QueryLayers::DEFAULT`]), not to every
/// layer. That is what makes a mask useful: `QueryFilter::default().layers(ENEMIES)` returns only
/// things you explicitly put on the `ENEMIES` layer. The default *query* mask is still
/// `u32::MAX`, so an unfiltered query sees everything.
///
/// This is a *query* filter only — it does not affect collision response.
///
/// ```ignore
/// const WORLD: u32 = 1 << 0;   // same bit as the default, so unlabeled geometry is included
/// const ENEMY: u32 = 1 << 1;
///
/// commands.spawn((RigidBody::Dynamic, BepuCollider::capsule(0.3, 1.0), QueryLayers(ENEMY)));
/// let filter = QueryFilter::default().layers(ENEMY);
/// ```
#[derive(Component, Debug, Clone, Copy, PartialEq, Eq, Reflect)]
pub struct QueryLayers(pub u32);

impl QueryLayers {
    /// What a collidable with no `QueryLayers` component is treated as: layer 0.
    pub const DEFAULT: Self = Self(1);
    /// Belongs to every layer, so it is found by any non-empty query mask.
    pub const ALL: Self = Self(u32::MAX);
    /// Belongs to no layer; never returned by a query.
    pub const NONE: Self = Self(0);

    /// Creates a mask containing a single layer index (0..32).
    #[inline]
    pub const fn layer(index: u32) -> Self {
        Self(1 << index)
    }
}

impl Default for QueryLayers {
    fn default() -> Self {
        Self::DEFAULT
    }
}

// ---------------------------------------------------------------------------
// Internal marker components (inserted by the plugin, not by users)
// ---------------------------------------------------------------------------
//
// Each of these carries an `on_remove` (or `on_discard`) hook that enqueues its teardown into
// [`BepuRemovalQueue`]. Hooks rather than `RemovedComponents` because the latter is a
// per-render-frame double-buffered queue and the physics schedule is `FixedPostUpdate`, which skips
// entire frames whenever the frame rate exceeds the fixed rate; see [`BepuRemovalQueue`] for the
// full explanation. Hooks fire synchronously at the moment the component leaves the entity, so no
// removal can expire unobserved no matter how many frames pass before the next tick.
//
// Do not remove these components by hand. They are the plugin's record of what it allocated; taking
// one off an entity tears down the corresponding simulation object.

/// Inserted by the plugin after a dynamic/kinematic body is added to the Bepu simulation.
/// Stores the internal [`BodyHandle`].
///
/// Removing it (or despawning the entity) removes the body from the simulation on the next
/// [`BepuSet::Prepare`](super::plugin::BepuSet::Prepare).
///
/// Removing it *without* despawning the entity is the supported way to rebuild a body: the entity
/// no longer matches any handle component, so `add_new_bodies` picks it up again on the next tick
/// and creates a fresh body from the current components. The rebuilt body gets a **freshly
/// allocated** [`BodyHandle`] — sometimes the old value, recycled, and sometimes not — so never
/// cache one across a rebuild.
#[derive(Component, Debug, Clone, Copy)]
#[component(on_remove = on_remove_body_handle)]
pub struct BepuBodyHandle(pub BodyHandle);

fn on_remove_body_handle(mut world: DeferredWorld, ctx: HookContext) {
    let Some(handle) = world.get::<BepuBodyHandle>(ctx.entity).map(|h| h.0) else {
        return;
    };
    if let Some(mut queue) = world.get_resource_mut::<BepuRemovalQueue>() {
        queue.bodies.push((ctx.entity, handle));
    }
}

/// Inserted by the plugin after a static body is added to the Bepu simulation.
/// Stores the internal [`StaticHandle`].
///
/// Removing it (or despawning the entity) removes the static from the simulation on the next
/// [`BepuSet::Prepare`](super::plugin::BepuSet::Prepare).
#[derive(Component, Debug, Clone, Copy)]
#[component(on_remove = on_remove_static_handle)]
pub struct BepuStaticHandle(pub StaticHandle);

fn on_remove_static_handle(mut world: DeferredWorld, ctx: HookContext) {
    let Some(handle) = world.get::<BepuStaticHandle>(ctx.entity).map(|h| h.0) else {
        return;
    };
    if let Some(mut queue) = world.get_resource_mut::<BepuRemovalQueue>() {
        queue.statics.push((ctx.entity, handle));
    }
}

/// Tracks the shape index allocated in the Bepu `Shapes` collection.
///
/// Inserted by the plugin alongside [`BepuBodyHandle`] / [`BepuStaticHandle`]. Shapes are
/// reference-counted and shared between identical colliders, so this index is not unique to the
/// entity.
///
/// The invariant is: **the value currently stored in this component holds exactly one reference to
/// its shape.** Whenever that value stops being the current one — the component is removed, the
/// entity is despawned, *or* the component is overwritten — the reference is dropped, and the shape
/// is freed once the last one goes away. Exactly once each way: one acquire, one release.
///
/// # Do not touch this component
///
/// The plugin owns it. Removing or overwriting it by hand drops the reference **while the body or
/// static still points at that shape slot**; if it was the last reference, the `Shapes` entry is
/// disposed under a live collidable and the next timestep's bounds update reads freed memory.
///
/// It is deliberately **not** re-exported from [`prelude`](super::prelude) for that reason. To
/// rebuild a body, remove [`BepuBodyHandle`] / [`BepuStaticHandle`] instead — that path is
/// supported, and it re-acquires the shape correctly.
#[derive(Component, Debug, Clone, Copy)]
// `on_discard`, deliberately, and *not* `on_remove`.
//
// `add_new_bodies` does not filter on `Added`; its predicate is "this entity has no handle
// component". An entity whose `BepuBodyHandle` is removed — the rebuild path the docs on those
// components advertise — therefore gets a fresh body and a fresh `acquire_shape` while its old
// `BepuShapeIndex` is still sitting on it, so the insert is an *overwrite*. Bevy fires `on_discard`
// for an overwrite and `on_remove` only for an actual removal, so a release hook on `on_remove`
// silently loses the overwritten reference and the `Shapes` entry leaks with nothing left able to
// name it.
//
// `on_discard` covers all three cases and covers each exactly once: bevy_ecs runs it whenever the
// value is about to be dropped — replaced in place by an insert, removed, or dropped with the
// entity — and in the latter two immediately *before* `on_remove`. Adding `on_remove` as well would
// therefore not be belt and braces, it would be a double release: both hooks fire on every removal,
// the reference count would be decremented twice for one acquire, and the shape would be freed out
// from under its remaining users. So there is one hook, and it is this one.
//
// `on_discard` runs before the new value is written, so `world.get` below reads the outgoing index.
#[component(on_discard = on_discard_shape_index)]
pub struct BepuShapeIndex(pub TypedIndex);

fn on_discard_shape_index(mut world: DeferredWorld, ctx: HookContext) {
    let Some(index) = world.get::<BepuShapeIndex>(ctx.entity).map(|s| s.0) else {
        return;
    };
    if let Some(mut queue) = world.get_resource_mut::<BepuRemovalQueue>() {
        queue.shapes.push(index);
    }
}

/// Inserted by the plugin after a constraint/joint is added to the solver.
#[derive(Component, Debug, Clone, Copy)]
pub struct BepuConstraintHandle(pub ConstraintHandle);
