//! On-demand spatial queries: ray casts and shape sweeps against the physics world.
//!
//! [`BepuSpatialQuery`] is a [`SystemParam`], so a system just asks for it:
//!
//! ```ignore
//! fn shoot(mut spatial: BepuSpatialQuery, player: Single<Entity, With<Player>>) {
//!     let filter = QueryFilter::default().exclude(&[*player]);
//!     if let Some(hit) = spatial.ray_cast(origin, direction, 100.0, filter) {
//!         info!("hit {:?} at t={} (child {})", hit.entity, hit.t, hit.child_index);
//!     }
//! }
//! ```
//!
//! # Two things that bite people
//!
//! **Self-exclusion.** A ray fired from inside a body hits that body first, every time. Every query
//! here takes a [`QueryFilter`]; use [`QueryFilter::exclude`] for the shooter. Exclusion resolves
//! collidables by branching on mobility *before* comparing handles, because a `BodyHandle` and a
//! `StaticHandle` with the same raw integer are unrelated objects — comparing raw handle values
//! alone silently never matches for one of the two kinds.
//!
//! **Sweeps that start already touching.** Bepu reports a sweep that begins in contact through a
//! separate callback with no `t`, no hit location, and no normal, because none of those are
//! defined. That case surfaces here as [`SweepResult::StartPenetrating`], never as a miss and never
//! as a hit with a zeroed normal. Match on it explicitly.

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;

use super::resources::BepuSimulation;

use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::box_shape::Box as PhysicsBox;
use crate::physics::collidables::capsule::Capsule;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::cylinder::Cylinder;
use crate::physics::collidables::sphere::Sphere;
use crate::physics::collision_detection::ray_batchers::RayData;
use crate::physics::simulation::{IRayHitHandler, ISweepHitHandler};
use crate::utilities::memory::buffer_pool::BufferPool;


/// A ray cast hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// The entity that was hit, if the collidable belongs to one the plugin created.
    pub entity: Option<Entity>,
    /// Parametric distance along the ray: the hit point is `origin + direction * t`. If
    /// `direction` is normalized this is the distance in metres.
    pub t: f32,
    /// World-space hit point, `origin + direction * t`.
    pub point: Vec3,
    /// Surface normal at the hit point.
    pub normal: Vec3,
    /// Index of the child shape that was hit, for compounds and meshes. `0` for convex shapes.
    ///
    /// Bepu reports this on every hit; it is what tells you *which triangle* of a mesh or which
    /// piece of a compound you actually hit.
    pub child_index: i32,
    /// Whether the hit collidable is a movable body rather than a static.
    pub is_dynamic: bool,
}

/// The outcome of a shape sweep.
///
/// The `StartPenetrating` variant is the important one: Bepu discovers overlap at `t = 0` through a
/// different callback than a normal hit, one that carries no time, point, or normal. Collapsing it
/// into either `Miss` or a zero-normal `Hit` produces bugs that only appear when geometry is
/// already touching — which is exactly when a camera collision probe cares most.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepResult {
    /// Nothing was hit within the sweep distance.
    Miss,
    /// The swept shape was already intersecting something at the start of the sweep.
    StartPenetrating {
        /// The entity already being overlapped, if known.
        entity: Option<Entity>,
        /// Whether the overlapped collidable is a movable body rather than a static.
        is_dynamic: bool,
    },
    /// A clean hit part-way along the sweep.
    Hit(SweepHit),
}

impl SweepResult {
    /// The hit, if this is a clean [`SweepResult::Hit`]. `StartPenetrating` yields `None` — if you
    /// use this, handle penetration separately first.
    #[inline]
    pub fn hit(self) -> Option<SweepHit> {
        match self {
            SweepResult::Hit(hit) => Some(hit),
            _ => None,
        }
    }

    /// Whether anything was found at all, penetrating or not.
    #[inline]
    pub fn is_blocked(self) -> bool {
        !matches!(self, SweepResult::Miss)
    }
}

/// A clean shape sweep hit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepHit {
    /// The entity that was hit, if the collidable belongs to one the plugin created.
    pub entity: Option<Entity>,
    /// Time of impact along the sweep, in the same units as the sweep's `max_t`.
    pub t: f32,
    /// World-space contact point.
    pub point: Vec3,
    /// World-space contact normal.
    pub normal: Vec3,
    /// Whether the hit collidable is a movable body rather than a static.
    pub is_dynamic: bool,
}

/// The shapes a sweep can be performed with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QueryShape {
    /// A sphere of the given radius.
    Sphere { radius: f32 },
    /// A box with the given full extents.
    Box { width: f32, height: f32, depth: f32 },
    /// A capsule aligned along the local Y axis.
    Capsule { radius: f32, length: f32 },
    /// A cylinder aligned along the local Y axis.
    Cylinder { radius: f32, length: f32 },
}

impl QueryShape {
    /// A sphere.
    #[inline]
    pub fn sphere(radius: f32) -> Self {
        Self::Sphere { radius }
    }
    /// A box, from full extents.
    #[inline]
    pub fn cuboid(width: f32, height: f32, depth: f32) -> Self {
        Self::Box {
            width,
            height,
            depth,
        }
    }
    /// A Y-aligned capsule.
    #[inline]
    pub fn capsule(radius: f32, length: f32) -> Self {
        Self::Capsule { radius, length }
    }
    /// A Y-aligned cylinder.
    #[inline]
    pub fn cylinder(radius: f32, length: f32) -> Self {
        Self::Cylinder { radius, length }
    }
}

impl From<&super::components::BepuCollider> for QueryShape {
    fn from(collider: &super::components::BepuCollider) -> Self {
        use super::components::BepuCollider as C;
        match *collider {
            C::Sphere { radius } => Self::Sphere { radius },
            C::Box {
                width,
                height,
                depth,
            } => Self::Box {
                width,
                height,
                depth,
            },
            C::Capsule { radius, length } => Self::Capsule { radius, length },
            C::Cylinder { radius, length } => Self::Cylinder { radius, length },
        }
    }
}

// ---------------------------------------------------------------------------
// Filtering
// ---------------------------------------------------------------------------

/// Which collidables a query is allowed to hit.
///
/// The default hits everything. Build it up fluently:
///
/// ```ignore
/// // Everything except the shooter, and only things on the "world" and "enemy" layers.
/// let filter = QueryFilter::default()
///     .exclude(&[shooter])
///     .layers(WORLD | ENEMY);
///
/// // Level geometry only.
/// let filter = QueryFilter::default().statics_only();
/// ```
///
/// The excluded-entity list is borrowed rather than owned so that per-frame queries do not
/// allocate. For a single entity, `&[entity]` is fine.
#[derive(Debug, Clone, Copy)]
pub struct QueryFilter<'a> {
    /// Entities that are never hit. Typically the querying entity itself.
    pub excluded: &'a [Entity],
    /// Layer mask. A collidable is considered only if its
    /// [`QueryLayers`](super::components::QueryLayers) mask shares a bit with this one. Entities
    /// with no `QueryLayers` component belong to layer 0.
    pub layers: u32,
    /// Whether static collidables can be hit.
    pub hit_statics: bool,
    /// Whether dynamic bodies can be hit.
    pub hit_dynamics: bool,
    /// Whether kinematic bodies can be hit.
    pub hit_kinematics: bool,
}

impl Default for QueryFilter<'_> {
    fn default() -> Self {
        Self {
            excluded: &[],
            layers: u32::MAX,
            hit_statics: true,
            hit_dynamics: true,
            hit_kinematics: true,
        }
    }
}

impl<'a> QueryFilter<'a> {
    /// Never hit these entities. Use this for the entity firing the query — otherwise the very
    /// first thing every query hits is the querying entity's own collider.
    #[inline]
    pub fn exclude(mut self, entities: &'a [Entity]) -> Self {
        self.excluded = entities;
        self
    }

    /// Only consider collidables whose [`QueryLayers`](super::components::QueryLayers) mask
    /// overlaps `mask`.
    #[inline]
    pub fn layers(mut self, mask: u32) -> Self {
        self.layers = mask;
        self
    }

    /// Only hit statics (level geometry).
    #[inline]
    pub fn statics_only(mut self) -> Self {
        self.hit_statics = true;
        self.hit_dynamics = false;
        self.hit_kinematics = false;
        self
    }

    /// Only hit bodies (dynamic and kinematic), never statics.
    #[inline]
    pub fn bodies_only(mut self) -> Self {
        self.hit_statics = false;
        self.hit_dynamics = true;
        self.hit_kinematics = true;
        self
    }

    /// Whether this collidable passes the filter.
    fn allows(&self, sim: &BepuSimulation, collidable: CollidableReference) -> bool {
        // Mobility first. The mobility bits are the only thing that says whether the packed handle
        // means a body or a static, so every decision below has to branch on it before it can
        // legally look at the handle.
        let mobility = collidable.mobility();
        let allowed_kind = match mobility {
            CollidableMobility::Dynamic => self.hit_dynamics,
            CollidableMobility::Kinematic => self.hit_kinematics,
            CollidableMobility::Static => self.hit_statics,
        };
        if !allowed_kind {
            return false;
        }

        if self.layers != u32::MAX && (sim.layers_for_collidable(collidable) & self.layers) == 0 {
            return false;
        }

        if !self.excluded.is_empty() {
            if let Some(entity) = sim.entity_for_collidable(collidable) {
                if self.excluded.contains(&entity) {
                    return false;
                }
            }
        }

        true
    }
}

/// Per-system scratch memory for query traversals.
///
/// Broad phase traversal needs a little stack space, and Bepu takes it from a `BufferPool`. Handing
/// each query system its own pool (via `Local`) is what lets [`BepuSpatialQuery`] borrow the
/// simulation *shared* — several query systems can then run in parallel instead of queueing behind
/// a single `ResMut`.
pub struct QueryScratch {
    pool: Box<BufferPool>,
}

impl Default for QueryScratch {
    fn default() -> Self {
        Self {
            // Small blocks: query traversals allocate a stack of node indices, not bulk storage.
            pool: Box::new(BufferPool::new(16384, 8)),
        }
    }
}

impl Drop for QueryScratch {
    fn drop(&mut self) {
        self.pool.clear();
    }
}

// SAFETY: the pool is only ever touched through `&mut QueryScratch`, which Bevy guarantees is
// exclusive to one system at a time, and it owns all of its allocations.
unsafe impl Send for QueryScratch {}
unsafe impl Sync for QueryScratch {}

/// System parameter for ray casts and shape sweeps against the BepuPhysics simulation.
///
/// Takes the simulation *shared* (`Res`, not `ResMut`), so multiple query systems can run
/// concurrently. Scratch memory comes from a `Local`, which is private to each system.
///
/// Queries read simulation state, so they must not run while the simulation is stepping. Schedule
/// them outside [`BepuSet::Step`](super::plugin::BepuSet) — anywhere in `Update`, or in
/// `FixedPostUpdate` under `BepuSet::Writeback`.
#[derive(SystemParam)]
pub struct BepuSpatialQuery<'w, 's> {
    sim: Res<'w, BepuSimulation>,
    scratch: Local<'s, QueryScratch>,
}

impl BepuSpatialQuery<'_, '_> {
    /// Casts a ray and returns the closest hit, if any.
    ///
    /// `direction` need not be normalized; `t` is measured in units of `direction`, so a normalized
    /// direction gives `t` in metres and `max_t` in metres.
    pub fn ray_cast(
        &mut self,
        origin: Vec3,
        direction: Vec3,
        max_t: f32,
        filter: QueryFilter<'_>,
    ) -> Option<RayHit> {
        struct ClosestRayHandler<'a> {
            sim: &'a BepuSimulation,
            filter: QueryFilter<'a>,
            best: Option<(f32, glam::Vec3, CollidableReference, i32)>,
        }

        impl IRayHitHandler for ClosestRayHandler<'_> {
            fn allow_test_collidable(&self, collidable: &CollidableReference) -> bool {
                self.filter.allows(self.sim, *collidable)
            }
            fn allow_test_child(
                &self,
                _collidable: &CollidableReference,
                _child_index: i32,
            ) -> bool {
                true
            }
            fn on_ray_hit(
                &mut self,
                _ray: &RayData,
                maximum_t: &mut f32,
                t: f32,
                normal: glam::Vec3,
                collidable: &CollidableReference,
                child_index: i32,
            ) {
                if self.best.is_none_or(|(best_t, ..)| t < best_t) {
                    self.best = Some((t, normal, *collidable, child_index));
                    // Narrow the search to closer hits only.
                    *maximum_t = t;
                }
            }
        }

        let origin_g = glam::Vec3::new(origin.x, origin.y, origin.z);
        let direction_g = glam::Vec3::new(direction.x, direction.y, direction.z);

        let mut handler = ClosestRayHandler {
            sim: &self.sim,
            filter,
            best: None,
        };

        unsafe {
            self.sim.simulation.ray_cast_with_pool(
                origin_g,
                direction_g,
                max_t,
                &mut handler,
                0,
                &mut *self.scratch.pool,
            );
        }

        let (t, normal, collidable, child_index) = handler.best?;
        let point = origin + direction * t;
        Some(RayHit {
            entity: self.sim.entity_for_collidable(collidable),
            t,
            point,
            normal: Vec3::new(normal.x, normal.y, normal.z),
            child_index,
            is_dynamic: collidable.mobility() != CollidableMobility::Static,
        })
    }

    /// Casts a ray and reports every hit to `on_hit`, in no particular order.
    ///
    /// Unlike [`BepuSpatialQuery::ray_cast`] this does not narrow the search as it goes, so it is
    /// more expensive; use it when you need to see through things (bullet penetration, visibility
    /// against a list of candidates).
    pub fn ray_cast_all(
        &mut self,
        origin: Vec3,
        direction: Vec3,
        max_t: f32,
        filter: QueryFilter<'_>,
        mut on_hit: impl FnMut(RayHit),
    ) {
        struct AllRayHandler<'a, F: FnMut(RayHit)> {
            sim: &'a BepuSimulation,
            filter: QueryFilter<'a>,
            origin: Vec3,
            direction: Vec3,
            on_hit: F,
        }

        impl<F: FnMut(RayHit)> IRayHitHandler for AllRayHandler<'_, F> {
            fn allow_test_collidable(&self, collidable: &CollidableReference) -> bool {
                self.filter.allows(self.sim, *collidable)
            }
            fn allow_test_child(
                &self,
                _collidable: &CollidableReference,
                _child_index: i32,
            ) -> bool {
                true
            }
            fn on_ray_hit(
                &mut self,
                _ray: &RayData,
                _maximum_t: &mut f32,
                t: f32,
                normal: glam::Vec3,
                collidable: &CollidableReference,
                child_index: i32,
            ) {
                let hit = RayHit {
                    entity: self.sim.entity_for_collidable(*collidable),
                    t,
                    point: self.origin + self.direction * t,
                    normal: Vec3::new(normal.x, normal.y, normal.z),
                    child_index,
                    is_dynamic: collidable.mobility() != CollidableMobility::Static,
                };
                (self.on_hit)(hit);
            }
        }

        let origin_g = glam::Vec3::new(origin.x, origin.y, origin.z);
        let direction_g = glam::Vec3::new(direction.x, direction.y, direction.z);

        let mut handler = AllRayHandler {
            sim: &self.sim,
            filter,
            origin,
            direction,
            on_hit: &mut on_hit,
        };

        unsafe {
            self.sim.simulation.ray_cast_with_pool(
                origin_g,
                direction_g,
                max_t,
                &mut handler,
                0,
                &mut *self.scratch.pool,
            );
        }
    }

    /// Sweeps a convex shape through the world and returns the first thing it runs into.
    ///
    /// `position` / `rotation` place the shape at the start of the sweep, and it travels along
    /// `direction` for at most `max_t` units of `direction`. Simulation objects are treated as
    /// stationary for the duration of the sweep.
    ///
    /// Read the [`SweepResult`] docs before using this: a sweep that starts already touching
    /// something returns [`SweepResult::StartPenetrating`], which carries no time or normal.
    pub fn sweep(
        &mut self,
        shape: QueryShape,
        position: Vec3,
        rotation: Quat,
        direction: Vec3,
        max_t: f32,
        filter: QueryFilter<'_>,
    ) -> SweepResult {
        let pose = RigidPose::new(
            glam::Vec3::new(position.x, position.y, position.z),
            glam::Quat::from_xyzw(rotation.x, rotation.y, rotation.z, rotation.w),
        );
        let velocity = BodyVelocity::new(
            glam::Vec3::new(direction.x, direction.y, direction.z),
            glam::Vec3::ZERO,
        );

        let mut handler = ClosestSweepHandler {
            sim: &self.sim,
            filter,
            best: None,
            penetrating: None,
        };

        // The pool is used for both the broad phase traversal and the sweep tasks' scratch space.
        let pool: *mut BufferPool = &mut *self.scratch.pool;
        unsafe {
            match shape {
                QueryShape::Sphere { radius } => self.sim.simulation.sweep_shape(
                    &Sphere::new(radius),
                    &pose,
                    &velocity,
                    max_t,
                    pool,
                    &mut handler,
                ),
                QueryShape::Box {
                    width,
                    height,
                    depth,
                } => self.sim.simulation.sweep_shape(
                    &PhysicsBox::new(width, height, depth),
                    &pose,
                    &velocity,
                    max_t,
                    pool,
                    &mut handler,
                ),
                QueryShape::Capsule { radius, length } => self.sim.simulation.sweep_shape(
                    &Capsule::new(radius, length),
                    &pose,
                    &velocity,
                    max_t,
                    pool,
                    &mut handler,
                ),
                QueryShape::Cylinder { radius, length } => self.sim.simulation.sweep_shape(
                    &Cylinder::new(radius, length),
                    &pose,
                    &velocity,
                    max_t,
                    pool,
                    &mut handler,
                ),
            }
        }

        // Penetration wins over any later hit: if the shape starts inside something, no time of
        // impact along the sweep is meaningful, and silently returning the farther clean hit is
        // exactly the kind of bug this API is trying to make impossible.
        if let Some(collidable) = handler.penetrating {
            return SweepResult::StartPenetrating {
                entity: self.sim.entity_for_collidable(collidable),
                is_dynamic: collidable.mobility() != CollidableMobility::Static,
            };
        }
        match handler.best {
            None => SweepResult::Miss,
            Some((t, point, normal, collidable)) => SweepResult::Hit(SweepHit {
                entity: self.sim.entity_for_collidable(collidable),
                t,
                point: Vec3::new(point.x, point.y, point.z),
                normal: Vec3::new(normal.x, normal.y, normal.z),
                is_dynamic: collidable.mobility() != CollidableMobility::Static,
            }),
        }
    }

    /// Convenience wrapper around [`BepuSpatialQuery::sweep`] for an unrotated shape.
    pub fn sweep_shape(
        &mut self,
        shape: QueryShape,
        position: Vec3,
        direction: Vec3,
        max_t: f32,
        filter: QueryFilter<'_>,
    ) -> SweepResult {
        self.sweep(shape, position, Quat::IDENTITY, direction, max_t, filter)
    }

    /// The underlying simulation resource, for code that needs the raw API.
    #[inline]
    pub fn simulation(&self) -> &BepuSimulation {
        &self.sim
    }
}

struct ClosestSweepHandler<'a> {
    sim: &'a BepuSimulation,
    filter: QueryFilter<'a>,
    best: Option<(f32, glam::Vec3, glam::Vec3, CollidableReference)>,
    penetrating: Option<CollidableReference>,
}

impl ISweepHitHandler for ClosestSweepHandler<'_> {
    fn allow_test(&self, collidable: &CollidableReference) -> bool {
        self.filter.allows(self.sim, *collidable)
    }

    fn allow_test_child(&self, _collidable: &CollidableReference, _child_index: i32) -> bool {
        true
    }

    fn on_hit(
        &mut self,
        maximum_t: &mut f32,
        t: f32,
        hit_location: glam::Vec3,
        hit_normal: glam::Vec3,
        collidable: &CollidableReference,
    ) {
        if self.best.is_none_or(|(best_t, ..)| t < best_t) {
            self.best = Some((t, hit_location, hit_normal, *collidable));
            *maximum_t = t;
        }
    }

    fn on_hit_at_zero_t(&mut self, maximum_t: &mut f32, collidable: &CollidableReference) {
        // No t, no location, no normal: the shape is already intersecting this collidable. Record
        // it as its own outcome rather than pretending it is a hit at the origin with a zero
        // normal, which is what a naive implementation does and what makes penetration bugs
        // intermittent and impossible to reason about.
        self.penetrating = Some(*collidable);
        *maximum_t = 0.0;
    }
}
