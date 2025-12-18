// Translated from BepuPhysics/Simulation.cs and BepuPhysics/Simulation_Queries.cs

use crate::physics::batch_compressor::BatchCompressor;
use crate::physics::bodies::Bodies;
use crate::physics::island_awakener::IslandAwakener;
use crate::physics::island_sleeper::IslandSleeper;
use crate::physics::pose_integration::IPoseIntegratorCallbacks;
use crate::physics::pose_integrator::{IPoseIntegrator, PoseIntegrator};
use crate::physics::simulation_allocation_sizes::SimulationAllocationSizes;
use crate::physics::simulation_profiler::SimulationProfiler;
use crate::physics::solve_description::SolveDescription;
use crate::physics::solver::Solver;
use crate::physics::statics::Statics;
use crate::physics::timestepper::ITimestepper;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

// Forward declarations for types not yet fully translated
pub struct Shapes {
    _opaque: [u8; 0],
}

pub struct BroadPhase {
    _opaque: [u8; 0],
}

pub struct CollidableOverlapFinder {
    _opaque: [u8; 0],
}

pub struct NarrowPhase {
    _opaque: [u8; 0],
}

pub struct ConstraintRemover {
    _opaque: [u8; 0],
}

/// Orchestrates the bookkeeping and execution of a full dynamic simulation.
pub struct Simulation {
    /// The system responsible for awakening bodies within the simulation.
    pub awakener: *mut IslandAwakener,
    /// The system responsible for putting bodies to sleep within the simulation.
    pub sleeper: *mut IslandSleeper,
    /// The collection of bodies in the simulation.
    pub bodies: *mut Bodies,
    /// The collection of statics within the simulation.
    pub statics: *mut Statics,
    /// The collection of shapes used by the simulation.
    pub shapes: *mut Shapes,
    /// The batch compressor used to compact constraints into fewer solver batches.
    pub solver_batch_compressor: *mut BatchCompressor,
    /// The solver used to solve constraints within the simulation.
    pub solver: *mut Solver,
    /// The integrator used to update velocities and poses within the simulation.
    pub pose_integrator: *mut dyn IPoseIntegrator,
    /// The broad phase used by the simulation.
    pub broad_phase: *mut BroadPhase,
    /// The system used to find overlapping pairs within the simulation.
    pub broad_phase_overlap_finder: *mut CollidableOverlapFinder,
    /// The system used to identify contacts and update contact constraint data.
    pub narrow_phase: *mut NarrowPhase,

    profiler: SimulationProfiler,

    // Helpers shared across at least two stages.
    constraint_remover: *mut ConstraintRemover,

    /// The main memory pool used to fill persistent structures and main thread ephemeral resources.
    pub buffer_pool: *mut BufferPool,

    /// The timestepper used to update the simulation state.
    pub timestepper: Box<dyn ITimestepper>,

    /// Whether to use a deterministic time step when using multithreading.
    /// When set to true, additional time is spent sorting constraint additions and transfers.
    pub deterministic: bool,
}

impl Simulation {
    /// Performs one timestep of the given length.
    pub fn timestep(&mut self, dt: f32, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        assert!(dt > 0.0, "Timestep duration must be positive.");
        self.profiler.clear();
        // profiler.start(self);
        unsafe {
            let sim_ptr = self as *mut Simulation as *mut u8;
            let td_ptr = match thread_dispatcher {
                Some(td) => td as *const dyn IThreadDispatcher as *mut u8,
                None => std::ptr::null_mut(),
            };
            (*self.timestepper).timestep(sim_ptr, dt, td_ptr);
        }
        // profiler.end(self);
    }

    /// Executes the sleep stage.
    pub unsafe fn sleep(&mut self, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        // profiler.start(sleeper);
        (&mut *self.sleeper).update(thread_dispatcher, self.deterministic);
        // profiler.end(sleeper);
    }

    /// Predicts bounding boxes of active bodies by speculatively integrating velocity.
    /// Does not actually modify body velocities. Updates deactivation candidacy.
    pub unsafe fn predict_bounding_boxes(
        &mut self,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        // profiler.start(pose_integrator);
        (&mut *self.pose_integrator).predict_bounding_boxes(dt, &mut *self.buffer_pool, thread_dispatcher);
        // profiler.end(pose_integrator);
    }

    /// Updates the broad phase, finds potentially colliding pairs, and executes the narrow phase.
    pub unsafe fn collision_detection(
        &mut self,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        // Cast opaque pointers to their actual types.
        type RealBroadPhase = crate::physics::collision_detection::broad_phase::BroadPhase;
        type RealOverlapFinder =
            crate::physics::collision_detection::collidable_overlap_finder::CollidableOverlapFinder;
        type RealNarrowPhase = crate::physics::collision_detection::narrow_phase::NarrowPhase;

        // profiler.start(broad_phase);
        let broad_phase = &mut *(self.broad_phase as *mut RealBroadPhase);
        broad_phase.update2(None, self.deterministic); // MT dispatcher TODO
        // profiler.end(broad_phase);

        // profiler.start(broad_phase_overlap_finder);
        let overlap_finder = &mut *(self.broad_phase_overlap_finder as *mut RealOverlapFinder);
        overlap_finder.dispatch_overlaps(dt, thread_dispatcher);
        // profiler.end(broad_phase_overlap_finder);

        // profiler.start(narrow_phase);
        // NarrowPhaseGeneric<T>::flush_with_preflush() is the full C# Flush,
        // but Simulation doesn't know TCallbacks. Call base NarrowPhase::flush()
        // for constraint removal pipeline. Preflush/postflush need type-erased dispatch (TODO).
        let narrow_phase = &mut *(self.narrow_phase as *mut RealNarrowPhase);
        narrow_phase.flush(thread_dispatcher);
        // profiler.end(narrow_phase);
    }

    /// Solves all constraints, performs substepped integration for constrained bodies,
    /// and integrates unconstrained bodies.
    pub unsafe fn solve(
        &mut self,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        let solver = &mut *self.solver;
        // profiler.start(solver);
        let constrained_body_set = solver.prepare_constraint_integration_responsibilities(thread_dispatcher);
        solver.solve(dt, thread_dispatcher);
        // profiler.end(solver);

        // profiler.start(pose_integrator);
        let substep_count = solver.substep_count();
        (&mut *self.pose_integrator).integrate_after_substepping(
            &constrained_body_set,
            dt,
            substep_count,
            thread_dispatcher,
        );
        // profiler.end(pose_integrator);

        solver.dispose_constraint_integration_responsibilities();
    }

    /// Incrementally improves body and constraint storage for better performance.
    pub unsafe fn incrementally_optimize_data_structures(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        // profiler.start(solver_batch_compressor);
        (&mut *self.solver_batch_compressor).compress(
            &mut *self.buffer_pool,
            thread_dispatcher,
            thread_dispatcher.is_some() && self.deterministic,
        );
        // profiler.end(solver_batch_compressor);
    }

    /// Clears the simulation of every object, only returning memory to the pool
    /// that would be returned by sequential removes.
    pub unsafe fn clear(&mut self) {
        (&mut *self.solver).clear();
        (&mut *self.bodies).clear();
        (&mut *self.statics).clear();
        // shapes.clear();
        // broad_phase.clear();
        // narrow_phase.clear();
        (&mut *self.sleeper).clear();
    }

    /// Increases the allocation size of any buffers too small to hold the allocation target.
    pub unsafe fn ensure_capacity(&mut self, target: &SimulationAllocationSizes) {
        let solver = &mut *self.solver;
        solver.ensure_solver_capacities(target.bodies, target.constraints);
        let min = target.constraints_per_type_batch.max(solver.minimum_capacity_per_type_batch());
        solver.set_minimum_capacity_per_type_batch(min);
        solver.ensure_type_batch_capacities();

        // narrow_phase.pair_cache.ensure_constraint_to_pair_mapping_capacity(solver, target.constraints);

        let bodies = &mut *self.bodies;
        bodies.ensure_capacity(target.bodies);
        bodies.minimum_constraint_capacity_per_body = target.constraint_count_per_body_estimate;
        bodies.ensure_constraint_list_capacities();

        (&mut *self.sleeper).ensure_sets_capacity(target.islands + 1);
        (&mut *self.statics).ensure_capacity(target.statics);
        // shapes.ensure_batch_capacities(target.shapes_per_type);
        // broad_phase.ensure_capacity(target.bodies, target.bodies + target.statics);
    }

    /// Increases/decreases allocation sizes to match the target.
    pub unsafe fn resize(&mut self, target: &SimulationAllocationSizes) {
        let solver = &mut *self.solver;
        solver.resize_solver_capacities(target.bodies, target.constraints);
        solver.set_minimum_capacity_per_type_batch(target.constraints_per_type_batch);
        solver.resize_type_batch_capacities();

        let bodies = &mut *self.bodies;
        bodies.resize(target.bodies);
        bodies.minimum_constraint_capacity_per_body = target.constraint_count_per_body_estimate;
        bodies.resize_constraint_list_capacities();

        (&mut *self.sleeper).resize_sets_capacity(target.islands + 1);
        (&mut *self.statics).resize(target.statics);
    }

    /// Clears the simulation and returns all pooled memory to the buffer pool.
    /// Leaves the simulation in an unusable state.
    pub unsafe fn dispose(&mut self) {
        self.clear();
        (&mut *self.sleeper).dispose();
        (&mut *self.solver).dispose();
        // broad_phase.dispose();
        // narrow_phase.dispose();
        (&mut *self.bodies).dispose();
        (&mut *self.statics).dispose();
        // shapes.dispose();
    }
}

unsafe impl Send for Simulation {}
unsafe impl Sync for Simulation {}

// --- Query types (from Simulation_Queries.cs) ---

use glam::Vec3;

/// Data about a ray being cast.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RayData {
    pub origin: Vec3,
    pub direction: Vec3,
    pub id: i32,
}

/// Defines a type capable of filtering ray test candidates on individual children of shapes.
pub trait IShapeRayHitHandler {
    /// Checks whether a child of a collidable should be tested against a ray.
    fn allow_test(&self, child_index: i32) -> bool;

    /// Called when a ray impact has been found.
    fn on_ray_hit(
        &mut self,
        ray: &RayData,
        maximum_t: &mut f32,
        t: f32,
        normal: Vec3,
        child_index: i32,
    );
}

// TODO: forward declaration
pub struct CollidableReference;

/// Defines a type capable of filtering ray test candidates and handling ray hit results.
pub trait IRayHitHandler {
    /// Checks whether a collidable should be tested against a ray.
    fn allow_test_collidable(&self, collidable: &CollidableReference) -> bool;

    /// Checks whether a child of a collidable should be tested against a ray.
    fn allow_test_child(&self, collidable: &CollidableReference, child_index: i32) -> bool;

    /// Called when a ray impact has been found.
    fn on_ray_hit(
        &mut self,
        ray: &RayData,
        maximum_t: &mut f32,
        t: f32,
        normal: Vec3,
        collidable: &CollidableReference,
        child_index: i32,
    );
}

/// Defines a type capable of filtering sweep candidates and handling sweep results.
pub trait ISweepHitHandler {
    /// Checks whether to run a detailed sweep test against a target collidable.
    fn allow_test(&self, collidable: &CollidableReference) -> bool;

    /// Checks whether to run a detailed sweep against a collidable's child.
    fn allow_test_child(&self, collidable: &CollidableReference, child_index: i32) -> bool;

    /// Called when a sweep test detects a hit with nonzero T value.
    fn on_hit(
        &mut self,
        maximum_t: &mut f32,
        t: f32,
        hit_location: Vec3,
        hit_normal: Vec3,
        collidable: &CollidableReference,
    );

    /// Called when a sweep test detects a hit at T = 0.
    fn on_hit_at_zero_t(&mut self, maximum_t: &mut f32, collidable: &CollidableReference);
}

impl Simulation {
    /// Intersects a ray against the simulation.
    pub unsafe fn ray_cast<H: IRayHitHandler>(
        &self,
        _origin: Vec3,
        _direction: Vec3,
        _maximum_t: f32,
        _hit_handler: &mut H,
        _id: i32,
    ) {
        // TODO: Requires BroadPhase.ray_cast and Shapes integration
        // Build a RayHitDispatcher that wraps the hit_handler,
        // calls broad_phase.ray_cast, then for each candidate calls shapes[type].ray_test.
    }

    /// Sweeps a shape against the simulation.
    pub unsafe fn sweep<H: ISweepHitHandler>(
        &self,
        // shape, pose, velocity, maximum_t, pool, hit_handler, ...
        _hit_handler: &mut H,
    ) {
        // TODO: Requires BroadPhase.sweep, NarrowPhase.sweep_task_registry, and Shapes integration
    }
}
