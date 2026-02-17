//! Bevy resources for the BepuPhysics integration.

use bevy::prelude::*;
use std::collections::HashMap;

use crate::physics::bodies::Bodies;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::handles::{BodyHandle, StaticHandle};
use crate::physics::simulation::Simulation;
use crate::physics::statics::Statics;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::ThreadDispatcher;

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

    /// Get a mutable reference to the shapes collection.
    ///
    /// # Safety
    /// Caller must ensure exclusive access.
    #[inline]
    pub(crate) unsafe fn shapes_mut(&mut self) -> &mut Shapes {
        &mut *(self.simulation.shapes as *mut Shapes)
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
}
