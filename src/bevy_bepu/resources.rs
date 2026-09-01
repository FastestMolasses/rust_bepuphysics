//! Bevy resources for the BepuPhysics integration.

use bevy::prelude::*;
use std::collections::HashMap;

use crate::physics::bodies::Bodies;
use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::{BodyHandle, StaticHandle};
use crate::physics::simulation::Simulation;
use crate::physics::statics::Statics;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::ThreadDispatcher;

use super::components::BepuCollider;
use super::shape_registry::ShapeRegistry;

// ---------------------------------------------------------------------------
// Gravity
// ---------------------------------------------------------------------------

/// Global gravity vector applied to all dynamic bodies.
///
/// Default: `Vec3::new(0.0, -9.81, 0.0)`
#[derive(Resource, Debug, Clone, Copy, Reflect)]
pub struct Gravity(pub Vec3);

impl Default for Gravity {
    fn default() -> Self {
        Self(Vec3::new(0.0, -9.81, 0.0))
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Global physics configuration. Insert this resource **before** the plugin
/// initializes (i.e., before `Startup`) to override defaults.
#[derive(Resource, Debug, Clone, Reflect)]
pub struct BepuConfig {
    /// Number of velocity iterations per solve. Default: 4.
    pub velocity_iterations: u32,
    /// Number of substeps per timestep. Default: 1.
    pub substep_count: u32,
    /// Default friction coefficient for pairs where neither entity has a
    /// [`Friction`](super::components::Friction) component. Default: 0.5.
    pub default_friction: f32,
    /// Default restitution for pairs. Default: 0.0 (no bounce).
    pub default_restitution: f32,
    /// Maximum recovery velocity for contact springs. Default: 2.0.
    pub max_recovery_velocity: f32,
    /// Contact spring frequency in Hz. Default: 30.0.
    pub spring_frequency: f32,
    /// Contact spring damping ratio. Default: 1.0 (critically damped).
    pub spring_damping_ratio: f32,
    /// Global linear damping — fraction of velocity lost per second. Default: 0.03.
    pub linear_damping: f32,
    /// Global angular damping — fraction of angular velocity lost per second. Default: 0.03.
    pub angular_damping: f32,
    /// Number of worker threads. `None` = auto-detect (number of logical CPUs).
    pub thread_count: Option<u32>,
    /// Whether to use deterministic simulation. Default: false.
    pub deterministic: bool,
}

impl Default for BepuConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 4,
            substep_count: 1,
            default_friction: 0.5,
            default_restitution: 0.0,
            max_recovery_velocity: 2.0,
            spring_frequency: 30.0,
            spring_damping_ratio: 1.0,
            linear_damping: 0.03,
            angular_damping: 0.03,
            thread_count: None,
            deterministic: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Deferred removals
// ---------------------------------------------------------------------------

/// Teardown work produced by component removal hooks, drained once per fixed tick.
#[derive(Resource, Debug, Default)]
pub struct BepuRemovalQueue {
    /// Bodies whose [`BepuBodyHandle`](super::components::BepuBodyHandle) was removed.
    pub(crate) bodies: Vec<(Entity, BodyHandle)>,
    /// Statics whose [`BepuStaticHandle`](super::components::BepuStaticHandle) was removed.
    pub(crate) statics: Vec<(Entity, StaticHandle)>,
    /// Shape references released by removal of
    /// [`BepuShapeIndex`](super::components::BepuShapeIndex).
    pub(crate) shapes: Vec<TypedIndex>,
}

impl BepuRemovalQueue {
    /// Whether anything is waiting to be torn down.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty() && self.statics.is_empty() && self.shapes.is_empty()
    }
}

// ---------------------------------------------------------------------------
// BepuSimulation — internal resource wrapping the raw simulation
// ---------------------------------------------------------------------------

/// Internal resource holding the BepuPhysics simulation and bookkeeping.
///
/// This is **not** exported to users directly. All access goes through
/// components and system params.
#[derive(Resource)]
pub struct BepuSimulation {
    /// The simulation itself.
    pub(crate) simulation: Box<Simulation>,
    /// The memory pool.
    pub(crate) buffer_pool: Box<BufferPool>,
    /// Thread dispatcher for multithreaded simulation.
    pub(crate) dispatcher: ThreadDispatcher,
    /// Entity ↔ BodyHandle mapping.
    pub(crate) entity_to_body: HashMap<Entity, BodyHandle>,
    pub(crate) body_to_entity: HashMap<BodyHandle, Entity>,
    /// Entity ↔ StaticHandle mapping.
    pub(crate) entity_to_static: HashMap<Entity, StaticHandle>,
    pub(crate) static_to_entity: HashMap<StaticHandle, Entity>,
    /// Query layer masks. A missing entry means "belongs to every layer".
    pub(crate) body_layers: HashMap<BodyHandle, u32>,
    pub(crate) static_layers: HashMap<StaticHandle, u32>,
    /// Reference-counted sharing of `Shapes` entries, so identical colliders cost one shape and a
    /// despawned collider's shape is actually freed. See
    /// [`ShapeRegistry`](super::shape_registry::ShapeRegistry).
    pub(crate) shape_registry: ShapeRegistry,
    /// Snapshot of the config used to create the simulation.
    pub(crate) config: BepuConfig,
}

// SAFETY: The Simulation's raw pointers are only accessed from Bevy systems
// that have exclusive (`&mut`) access. ThreadDispatcher handles its own internal
// synchronization. All accesses to Simulation go through the single
// BepuSimulation resource which Bevy's scheduler protects with its standard
// single-writer / multiple-reader guarantees.
unsafe impl Send for BepuSimulation {}
unsafe impl Sync for BepuSimulation {}

impl Drop for BepuSimulation {
    fn drop(&mut self) {
        unsafe {
            self.simulation.dispose();
            self.buffer_pool.clear();
        }
    }
}

impl BepuSimulation {
    /// The entity owning the given body handle, if it is still alive.
    #[inline]
    pub fn entity_for_body(&self, handle: BodyHandle) -> Option<Entity> {
        self.body_to_entity.get(&handle).copied()
    }

    /// The entity owning the given static handle, if it is still alive.
    #[inline]
    pub fn entity_for_static(&self, handle: StaticHandle) -> Option<Entity> {
        self.static_to_entity.get(&handle).copied()
    }

    /// The body handle for an entity, if it has one.
    #[inline]
    pub fn body_handle(&self, entity: Entity) -> Option<BodyHandle> {
        self.entity_to_body.get(&entity).copied()
    }

    /// The static handle for an entity, if it has one.
    #[inline]
    pub fn static_handle(&self, entity: Entity) -> Option<StaticHandle> {
        self.entity_to_static.get(&entity).copied()
    }

    /// Resolves a collidable reference (as reported by queries and contacts) back to an entity.
    ///
    /// Branches on mobility first: a `BodyHandle` and a `StaticHandle` with the same raw value are
    /// unrelated, so comparing raw handle values without checking mobility silently mismatches.
    #[inline]
    pub fn entity_for_collidable(
        &self,
        collidable: crate::physics::collidables::collidable_reference::CollidableReference,
    ) -> Option<Entity> {
        use crate::physics::collidables::collidable_reference::CollidableMobility;
        match collidable.mobility() {
            CollidableMobility::Dynamic | CollidableMobility::Kinematic => {
                self.entity_for_body(BodyHandle(collidable.raw_handle_value()))
            }
            CollidableMobility::Static => {
                self.entity_for_static(StaticHandle(collidable.raw_handle_value()))
            }
        }
    }

    /// The query layer mask of a collidable. Collidables with no
    /// [`QueryLayers`](super::components::QueryLayers) component belong to layer 0
    /// ([`QueryLayers::DEFAULT`](super::components::QueryLayers::DEFAULT)).
    #[inline]
    pub fn layers_for_collidable(
        &self,
        collidable: crate::physics::collidables::collidable_reference::CollidableReference,
    ) -> u32 {
        use crate::physics::collidables::collidable_reference::CollidableMobility;
        let default = super::components::QueryLayers::DEFAULT.0;
        match collidable.mobility() {
            CollidableMobility::Dynamic | CollidableMobility::Kinematic => self
                .body_layers
                .get(&BodyHandle(collidable.raw_handle_value()))
                .copied()
                .unwrap_or(default),
            CollidableMobility::Static => self
                .static_layers
                .get(&StaticHandle(collidable.raw_handle_value()))
                .copied()
                .unwrap_or(default),
        }
    }

    /// Number of dynamic and kinematic bodies the plugin currently has registered.
    ///
    /// This counts the plugin's own bookkeeping, which by construction matches the number of live
    /// Bepu bodies the plugin created. Use [`BepuSimulation::simulation_body_count`] for the
    /// simulation's own count, including any bodies added through the raw API.
    #[inline]
    pub fn body_count(&self) -> usize {
        self.entity_to_body.len()
    }

    /// Number of statics the plugin currently has registered.
    #[inline]
    pub fn static_count(&self) -> usize {
        self.entity_to_static.len()
    }

    /// Number of bodies the Bepu simulation itself holds, awake or asleep.
    #[inline]
    pub fn simulation_body_count(&self) -> usize {
        unsafe { self.bodies().count_bodies().max(0) as usize }
    }

    /// Number of distinct shapes the plugin currently keeps alive in the `Shapes` collection.
    ///
    /// Identical colliders share one shape, so this is the number of *distinct* collider
    /// descriptions in use, not the number of colliders.
    #[inline]
    pub fn shape_count(&self) -> usize {
        self.shape_registry.len()
    }

    /// How many live colliders share the shape backing `collider`. Zero if no entity uses it.
    #[inline]
    pub fn shape_reference_count(&self, collider: &BepuCollider) -> u32 {
        self.shape_registry.reference_count(collider)
    }

    /// Installs a user filter consulted by
    /// [`DefaultNarrowPhaseCallbacks`](super::callbacks::DefaultNarrowPhaseCallbacks) before contacts
    /// are generated for a pair. Returning `false` suppresses the pair entirely — no contacts and
    /// no solver work.
    ///
    /// The filter runs on narrow phase worker threads, concurrently, once per candidate pair per
    /// step, so it must be cheap and must not touch the simulation. Resolve collidables to entities
    /// with [`BepuSimulation::entity_for_collidable`] beforehand and capture the result if you need
    /// ECS data.
    ///
    /// Pass `None` to clear it. The built-in dynamic-involvement rule still applies first: a pair
    /// with no dynamic collidable is rejected before the filter is consulted.
    ///
    /// ```ignore
    /// fn setup(mut sim: ResMut<BepuSimulation>) {
    ///     sim.set_contact_filter(Some(Box::new(|a, b| {
    ///         a.mobility() != CollidableMobility::Kinematic
    ///             || b.mobility() != CollidableMobility::Kinematic
    ///     })));
    /// }
    /// ```
    pub fn set_contact_filter(&mut self, filter: Option<super::callbacks::ContactFilter>) {
        // Same assumption `update_callback_data` makes: `initialize_simulation` always builds the
        // simulation with `DefaultNarrowPhaseCallbacks`.
        unsafe {
            let narrow_phase = self.simulation.narrow_phase
                as *mut crate::physics::collision_detection::narrow_phase::NarrowPhaseGeneric<
                    super::callbacks::DefaultNarrowPhaseCallbacks,
                >;
            (*narrow_phase).callbacks.filter = filter;
        }
    }

    /// Resolves a collidable pair the way the narrow phase sees it, for use inside a contact filter
    /// installed with [`BepuSimulation::set_contact_filter`].
    #[inline]
    pub fn entities_for_collidables(
        &self,
        a: CollidableReference,
        b: CollidableReference,
    ) -> (Option<Entity>, Option<Entity>) {
        (self.entity_for_collidable(a), self.entity_for_collidable(b))
    }

    /// Raw access to the underlying simulation.
    ///
    /// # Safety
    /// The caller must respect Bepu's threading rules: no query or mutation may overlap a timestep,
    /// and mutations require exclusive access to this resource.
    #[inline]
    pub unsafe fn raw(&self) -> &Simulation {
        &self.simulation
    }

    /// Raw access to the underlying simulation.
    ///
    /// # Safety
    /// See [`BepuSimulation::raw`].
    #[inline]
    pub unsafe fn raw_mut(&mut self) -> &mut Simulation {
        &mut self.simulation
    }

    /// Get a reference to the bodies collection.
    ///
    /// # Safety
    /// Caller must ensure no other mutable reference to bodies exists.
    #[inline]
    pub(crate) unsafe fn bodies(&self) -> &Bodies {
        &*self.simulation.bodies
    }

    /// Get a mutable reference to the bodies collection.
    ///
    /// # Safety
    /// Caller must ensure exclusive access.
    #[inline]
    pub(crate) unsafe fn bodies_mut(&mut self) -> &mut Bodies {
        &mut *self.simulation.bodies
    }

    /// Get a reference to the statics collection.
    ///
    /// # Safety
    /// Caller must ensure no other mutable reference exists.
    #[inline]
    pub(crate) unsafe fn statics(&self) -> &Statics {
        &*self.simulation.statics
    }

    /// Get a mutable reference to the statics collection.
    ///
    /// # Safety
    /// Caller must ensure exclusive access.
    #[inline]
    pub(crate) unsafe fn statics_mut(&mut self) -> &mut Statics {
        &mut *self.simulation.statics
    }

    /// Takes a reference to a shared shape for `collider`, adding it to the `Shapes` collection only
    /// if no equivalent shape exists yet. Balance with [`BepuSimulation::release_shape`].
    pub(crate) fn acquire_shape(&mut self, collider: &BepuCollider) -> TypedIndex {
        // Split borrow: the registry lives next to the raw pointer it is about to mutate through.
        let shapes = unsafe { &mut *(self.simulation.shapes as *mut Shapes) };
        self.shape_registry.acquire(shapes, collider)
    }

    /// Drops a reference taken by [`BepuSimulation::acquire_shape`], freeing the shape on the last
    /// release.
    pub(crate) fn release_shape(&mut self, index: TypedIndex) {
        let shapes = unsafe { &mut *(self.simulation.shapes as *mut Shapes) };
        let pool: &mut BufferPool = &mut self.buffer_pool;
        self.shape_registry.release(shapes, pool, index);
    }
}
