// Translated from BepuPhysics/Simulation.cs and BepuPhysics/Simulation_Queries.cs

use crate::physics::batch_compressor::BatchCompressor;
use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::shape::IConvexShape;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::island_awakener::IslandAwakener;
use crate::physics::island_sleeper::IslandSleeper;
use crate::physics::pose_integrator::IPoseIntegrator;
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
    /// Constructs a simulation supporting dynamic movement and constraints with the specified callbacks.
    ///
    /// # Safety
    /// The buffer pool must remain valid for the lifetime of the simulation.
    /// All callback types must be valid for the lifetime of the simulation.
    pub unsafe fn create<TNarrowPhaseCallbacks, TPoseIntegratorCallbacks>(
        buffer_pool: *mut BufferPool,
        narrow_phase_callbacks: TNarrowPhaseCallbacks,
        pose_integrator_callbacks: TPoseIntegratorCallbacks,
        solve_description: SolveDescription,
        timestepper: Option<Box<dyn ITimestepper>>,
        initial_allocation_sizes: Option<SimulationAllocationSizes>,
        shapes: Option<*mut crate::physics::collidables::shapes::Shapes>,
    ) -> Box<Simulation>
    where
        TNarrowPhaseCallbacks:
            crate::physics::collision_detection::narrow_phase_callbacks::INarrowPhaseCallbacks
                + 'static,
        TPoseIntegratorCallbacks:
            crate::physics::pose_integration::IPoseIntegratorCallbacks + 'static,
    {
        use crate::physics::batch_compressor::BatchCompressor;
        use crate::physics::collidables::shapes::Shapes as RealShapes;
        use crate::physics::collision_detection::broad_phase::BroadPhase as RealBroadPhase;
        use crate::physics::collision_detection::collidable_overlap_finder::CollidableOverlapFinder as RealOverlapFinder;
        use crate::physics::collision_detection::constraint_remover::ConstraintRemover as RealConstraintRemover;
        use crate::physics::collision_detection::narrow_phase::NarrowPhaseGeneric;
        use crate::physics::default_timestepper::DefaultTimestepper;
        use crate::physics::default_types::DefaultTypes;
        use crate::physics::island_awakener::IslandAwakener;
        use crate::physics::island_sleeper::IslandSleeper;
        use crate::physics::pose_integrator::PoseIntegrator;

        let sizes = initial_allocation_sizes.unwrap_or(SimulationAllocationSizes {
            bodies: 4096,
            statics: 4096,
            shapes_per_type: 128,
            constraint_count_per_body_estimate: 8,
            constraints: 16384,
            constraints_per_type_batch: 256,
            islands: 16,
        });

        let pool = &mut *buffer_pool;

        // Create shapes collection.
        let shapes_ptr = if let Some(s) = shapes {
            s
        } else {
            Box::into_raw(Box::new(RealShapes::new(
                pool,
                sizes.shapes_per_type as usize,
            )))
        };

        // Create broad phase.
        let broad_phase = Box::into_raw(Box::new(RealBroadPhase::new(
            buffer_pool,
            sizes.bodies,
            sizes.bodies + sizes.statics,
        )));

        // Create bodies.
        let bodies = Box::into_raw(Box::new(Bodies::new(
            buffer_pool,
            shapes_ptr as *mut crate::physics::collidables::shapes::Shapes,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            sizes.bodies,
            sizes.islands,
            sizes.constraint_count_per_body_estimate,
        )));

        // Create statics.
        let statics = Box::into_raw(Box::new(Statics::new(
            buffer_pool,
            shapes_ptr as *mut crate::physics::collidables::shapes::Shapes,
            bodies,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            sizes.statics,
        )));

        // Create pose integrator.
        let pose_integrator = Box::into_raw(Box::new(PoseIntegrator::new(
            bodies,
            shapes_ptr as *mut crate::physics::collidables::shapes::Shapes,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            pose_integrator_callbacks,
        )));

        // Create solver.
        let solver = Box::into_raw(Box::new(Solver::new(
            bodies,
            buffer_pool,
            solve_description,
            sizes.constraints,
            sizes.islands,
            sizes.constraints_per_type_batch,
        )));
        (*solver).pose_integrator =
            Some(pose_integrator as *mut dyn crate::physics::pose_integrator::IPoseIntegrator);

        // Create constraint remover.
        let constraint_remover = RealConstraintRemover::with_defaults(buffer_pool, bodies, solver);

        // Create island sleeper.
        let sleeper = Box::into_raw(Box::new(IslandSleeper::new(
            bodies,
            solver,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            // The constraint_remover is moved into NarrowPhase; sleeper needs its own pointer.
            // For now, use a null pointer; it will be wired later.
            std::ptr::null_mut(),
            buffer_pool,
        )));

        // Create island awakener.
        let awakener = Box::into_raw(Box::new(IslandAwakener::new(
            bodies,
            statics,
            solver,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            sleeper,
            std::ptr::null_mut(), // pair_cache, wired later
            buffer_pool,
        )));

        // Wire deferred pointers.
        (*statics).awakener = awakener;
        (*solver).awakener = awakener;
        (*bodies).initialize(solver, awakener, sleeper);

        // Create batch compressor.
        let batch_compressor =
            Box::into_raw(Box::new(BatchCompressor::with_defaults(solver, bodies)));

        // Create timestepper.
        let timestepper: Box<dyn ITimestepper> =
            timestepper.unwrap_or_else(|| Box::new(DefaultTimestepper::default()));

        // Create collision task registries.
        let collision_task_registry = Box::into_raw(Box::new(
            DefaultTypes::create_default_collision_task_registry(),
        ));
        let sweep_task_registry =
            Box::into_raw(Box::new(DefaultTypes::create_default_sweep_task_registry()));

        // Create narrow phase.
        let narrow_phase = Box::into_raw(Box::new(NarrowPhaseGeneric::new(
            buffer_pool,
            bodies,
            statics,
            solver,
            shapes_ptr as *mut crate::physics::collidables::shapes::Shapes,
            collision_task_registry,
            sweep_task_registry,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
            constraint_remover,
            narrow_phase_callbacks,
            sizes.islands + 1,
            1024,
            512,
        )));

        // Register default constraint types.
        DefaultTypes::register_defaults(&mut *solver, &mut (*narrow_phase).base);

        // Wire pair cache into sleeper, awakener, solver.
        let pair_cache_ptr = &mut (*narrow_phase).base.pair_cache
            as *mut crate::physics::collision_detection::pair_cache::PairCache;
        (*sleeper).pair_cache = pair_cache_ptr;
        (*awakener).pair_cache = pair_cache_ptr;
        (*solver).pair_cache = pair_cache_ptr;

        // Wire constraint_remover into sleeper (the NarrowPhase owns the ConstraintRemover so we use its address).
        (*sleeper).constraint_remover =
            &mut (*narrow_phase).base.constraint_remover as *mut RealConstraintRemover;

        // Create broad phase overlap finder.
        let overlap_finder = Box::into_raw(Box::new(RealOverlapFinder::new(
            narrow_phase,
            broad_phase as *mut crate::physics::collision_detection::broad_phase::BroadPhase,
        )));

        // Build the simulation struct.
        let simulation = Box::new(Simulation {
            awakener,
            sleeper,
            bodies,
            statics,
            shapes: shapes_ptr as *mut Shapes,
            solver_batch_compressor: batch_compressor,
            solver,
            pose_integrator: pose_integrator
                as *mut dyn crate::physics::pose_integrator::IPoseIntegrator,
            broad_phase: broad_phase as *mut BroadPhase,
            broad_phase_overlap_finder: overlap_finder as *mut CollidableOverlapFinder,
            narrow_phase: narrow_phase as *mut NarrowPhase,
            profiler: SimulationProfiler::new(16),
            constraint_remover: &mut (*narrow_phase).base.constraint_remover
                as *mut RealConstraintRemover
                as *mut ConstraintRemover,
            buffer_pool: buffer_pool,
            timestepper,
            deterministic: false,
        });

        // Initialize callbacks (deferred until after all simulation bits are constructed).
        // C#: poseIntegrator.Callbacks.Initialize(simulation);
        //     narrowPhase.Callbacks.Initialize(simulation);
        let sim_ptr = &*simulation as *const Simulation as *mut Simulation;
        (*pose_integrator).callbacks.initialize(sim_ptr as *mut u8);
        (*narrow_phase).callbacks.initialize(sim_ptr);

        simulation
    }

    /// Performs one timestep of the given length.
    pub fn timestep(&mut self, dt: f32, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        assert!(dt > 0.0, "Timestep duration must be positive.");
        self.profiler.clear();
        self.profiler.start("simulation");
        unsafe {
            let self_ptr = self as *mut Simulation;
            (*self_ptr)
                .timestepper
                .timestep(self_ptr, dt, thread_dispatcher);
        }
        self.profiler.end("simulation");
    }

    /// Executes the sleep stage.
    pub unsafe fn sleep(&mut self, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        self.profiler.start("sleeper");
        (&mut *self.sleeper).update(thread_dispatcher, self.deterministic);
        self.profiler.end("sleeper");
    }

    /// Predicts bounding boxes of active bodies by speculatively integrating velocity.
    /// Does not actually modify body velocities. Updates deactivation candidacy.
    pub unsafe fn predict_bounding_boxes(
        &mut self,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        self.profiler.start("pose_integrator");
        (&mut *self.pose_integrator).predict_bounding_boxes(
            dt,
            &mut *self.buffer_pool,
            thread_dispatcher,
        );
        self.profiler.end("pose_integrator");
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

        self.profiler.start("broad_phase");
        let broad_phase = &mut *(self.broad_phase as *mut RealBroadPhase);
        // C# BroadPhase.Update2 uses default deterministic=false; don't pass self.deterministic here.
        broad_phase.update2(thread_dispatcher, false);
        self.profiler.end("broad_phase");

        self.profiler.start("broad_phase_overlap_finder");
        let overlap_finder = &mut *(self.broad_phase_overlap_finder as *mut RealOverlapFinder);
        overlap_finder.dispatch_overlaps(dt, thread_dispatcher);
        self.profiler.end("broad_phase_overlap_finder");

        self.profiler.start("narrow_phase");
        // NarrowPhaseGeneric<T>::flush_with_preflush() is the full C# Flush,
        // but Simulation doesn't know TCallbacks. Call base NarrowPhase::flush()
        // for constraint removal pipeline. Preflush/postflush need type-erased dispatch (TODO).
        let narrow_phase = &mut *(self.narrow_phase as *mut RealNarrowPhase);
        let deterministic = thread_dispatcher.is_some() && self.deterministic;
        narrow_phase.flush(thread_dispatcher, deterministic);
        self.profiler.end("narrow_phase");
    }

    /// Solves all constraints, performs substepped integration for constrained bodies,
    /// and integrates unconstrained bodies.
    pub unsafe fn solve(&mut self, dt: f32, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        let solver = &mut *self.solver;
        // Wire the pose integrator into the solver for kinematic integration during substepping.
        solver.pose_integrator = Some(self.pose_integrator);
        self.profiler.start("solver");
        let constrained_body_set =
            solver.prepare_constraint_integration_responsibilities(thread_dispatcher);
        solver.solve(dt, thread_dispatcher);
        self.profiler.end("solver");

        self.profiler.start("pose_integrator");
        let substep_count = solver.substep_count();
        (&mut *self.pose_integrator).integrate_after_substepping(
            &constrained_body_set,
            dt,
            substep_count,
            thread_dispatcher,
        );
        self.profiler.end("pose_integrator");

        solver.dispose_constraint_integration_responsibilities();
    }

    /// Incrementally improves body and constraint storage for better performance.
    pub unsafe fn incrementally_optimize_data_structures(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        self.profiler.start("solver_batch_compressor");
        (&mut *self.solver_batch_compressor).compress(
            &mut *self.buffer_pool,
            thread_dispatcher,
            thread_dispatcher.is_some() && self.deterministic,
        );
        self.profiler.end("solver_batch_compressor");
    }

    /// Clears the simulation of every object, only returning memory to the pool
    /// that would be returned by sequential removes.
    pub unsafe fn clear(&mut self) {
        (&mut *self.solver).clear();
        (&mut *self.bodies).clear();
        (&mut *self.statics).clear();
        {
            use crate::physics::collidables::shapes::Shapes as RealShapes;
            (&mut *(self.shapes as *mut RealShapes)).clear();
        }
        {
            use crate::physics::collision_detection::broad_phase::BroadPhase as RealBroadPhase;
            (&mut *(self.broad_phase as *mut RealBroadPhase)).clear();
        }
        {
            use crate::physics::collision_detection::narrow_phase::NarrowPhase as RealNarrowPhase;
            (&mut *(self.narrow_phase as *mut RealNarrowPhase)).clear();
        }
        (&mut *self.sleeper).clear();
    }

    /// Increases the allocation size of any buffers too small to hold the allocation target.
    pub unsafe fn ensure_capacity(&mut self, target: &SimulationAllocationSizes) {
        let solver = &mut *self.solver;
        solver.ensure_solver_capacities(target.bodies, target.constraints);
        let min = target
            .constraints_per_type_batch
            .max(solver.minimum_capacity_per_type_batch());
        solver.set_minimum_capacity_per_type_batch(min);
        solver.ensure_type_batch_capacities();

        {
            use crate::physics::collision_detection::narrow_phase::NarrowPhase as RealNarrowPhase;
            (&mut *(self.narrow_phase as *mut RealNarrowPhase))
                .pair_cache
                .ensure_constraint_to_pair_mapping_capacity(solver, target.constraints);
        }

        let bodies = &mut *self.bodies;
        bodies.ensure_capacity(target.bodies);
        bodies.minimum_constraint_capacity_per_body = target.constraint_count_per_body_estimate;
        bodies.ensure_constraint_list_capacities();

        (&mut *self.sleeper).ensure_sets_capacity(target.islands + 1);
        (&mut *self.statics).ensure_capacity(target.statics);
        {
            use crate::physics::collidables::shapes::Shapes as RealShapes;
            (&mut *(self.shapes as *mut RealShapes))
                .ensure_batch_capacities(target.shapes_per_type as usize);
        }
        {
            use crate::physics::collision_detection::broad_phase::BroadPhase as RealBroadPhase;
            (&mut *(self.broad_phase as *mut RealBroadPhase))
                .ensure_capacity(target.bodies, target.bodies + target.statics);
        }
    }

    /// Increases/decreases allocation sizes to match the target.
    pub unsafe fn resize(&mut self, target: &SimulationAllocationSizes) {
        let solver = &mut *self.solver;
        solver.resize_solver_capacities(target.bodies, target.constraints);
        solver.set_minimum_capacity_per_type_batch(target.constraints_per_type_batch);
        solver.resize_type_batch_capacities();

        {
            use crate::physics::collision_detection::narrow_phase::NarrowPhase as RealNarrowPhase;
            (&mut *(self.narrow_phase as *mut RealNarrowPhase))
                .pair_cache
                .resize_constraint_to_pair_mapping_capacity(solver, target.constraints);
        }

        let bodies = &mut *self.bodies;
        bodies.resize(target.bodies);
        bodies.minimum_constraint_capacity_per_body = target.constraint_count_per_body_estimate;
        bodies.resize_constraint_list_capacities();

        (&mut *self.sleeper).resize_sets_capacity(target.islands + 1);
        (&mut *self.statics).resize(target.statics);
        {
            use crate::physics::collidables::shapes::Shapes as RealShapes;
            (&mut *(self.shapes as *mut RealShapes))
                .resize_batches(target.shapes_per_type as usize);
        }
        {
            use crate::physics::collision_detection::broad_phase::BroadPhase as RealBroadPhase;
            (&mut *(self.broad_phase as *mut RealBroadPhase))
                .resize(target.bodies, target.bodies + target.statics);
        }
    }

    /// Clears the simulation and returns all pooled memory to the buffer pool.
    /// Leaves the simulation in an unusable state.
    pub unsafe fn dispose(&mut self) {
        self.clear();
        (&mut *self.sleeper).dispose();
        (&mut *self.solver).dispose();
        {
            use crate::physics::collision_detection::broad_phase::BroadPhase as RealBroadPhase;
            (&mut *(self.broad_phase as *mut RealBroadPhase)).dispose();
        }
        {
            use crate::physics::collision_detection::narrow_phase::NarrowPhase as RealNarrowPhase;
            (&mut *(self.narrow_phase as *mut RealNarrowPhase)).dispose();
        }
        (&mut *self.bodies).dispose();
        (&mut *self.statics).dispose();
        {
            use crate::physics::collidables::shapes::Shapes as RealShapes;
            (&mut *(self.shapes as *mut RealShapes)).dispose();
        }
    }
}

unsafe impl Send for Simulation {}
unsafe impl Sync for Simulation {}

// --- Query types (from Simulation_Queries.cs) ---

use glam::Vec3;

// Re-export types from ray_batchers for public API.
pub use crate::physics::collidables::collidable_reference::CollidableReference;
pub use crate::physics::collision_detection::ray_batchers::{IShapeRayHitHandler, RayData};

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
    /// Gets the pose and shape for a collidable reference.
    ///
    /// # Safety
    /// The collidable reference must be valid (i.e. the handle must exist in bodies or statics).
    #[inline(always)]
    pub unsafe fn get_pose_and_shape(
        &self,
        reference: CollidableReference,
    ) -> (*const RigidPose, TypedIndex) {
        use crate::physics::collidables::collidable_reference::CollidableMobility;
        if reference.mobility() == CollidableMobility::Static {
            let statics = &*self.statics;
            let index = *statics.handle_to_index.get(reference.static_handle().0);
            let s = statics.statics_buffer.get(index);
            (&s.pose as *const RigidPose, s.shape)
        } else {
            let bodies = &*self.bodies;
            let location = bodies.handle_to_location.get(reference.body_handle().0);
            let set = bodies.sets.get(location.set_index);
            let dynamics = set.dynamics_state.get(location.index);
            (
                &dynamics.motion.pose as *const RigidPose,
                set.collidables.get(location.index).shape,
            )
        }
    }

    /// Intersects a ray against the simulation.
    pub unsafe fn ray_cast<H: IRayHitHandler>(
        &self,
        origin: Vec3,
        direction: Vec3,
        maximum_t: f32,
        hit_handler: &mut H,
        id: i32,
    ) {
        use crate::physics::collision_detection::ray_batchers::{
            IBroadPhaseRayTester, IShapeRayHitHandler, RayData,
        };

        // ShapeRayHitHandler wraps the user's IRayHitHandler to implement IShapeRayHitHandler.
        struct ShapeRayHitHandler<'a, H: IRayHitHandler> {
            hit_handler: &'a mut H,
            collidable: CollidableReference,
        }

        impl<H: IRayHitHandler> IShapeRayHitHandler for ShapeRayHitHandler<'_, H> {
            #[inline(always)]
            fn allow_test(&self, child_index: i32) -> bool {
                self.hit_handler
                    .allow_test_child(&self.collidable, child_index)
            }

            #[inline(always)]
            fn on_ray_hit(
                &mut self,
                ray: &RayData,
                maximum_t: &mut f32,
                t: f32,
                normal: Vec3,
                child_index: i32,
            ) {
                self.hit_handler.on_ray_hit(
                    ray,
                    maximum_t,
                    t,
                    normal,
                    &self.collidable,
                    child_index,
                );
            }
        }

        // RayHitDispatcher implements IBroadPhaseRayTester for the broad phase traversal.
        struct RayHitDispatcher<'a, H: IRayHitHandler> {
            simulation: &'a Simulation,
            shape_hit_handler: ShapeRayHitHandler<'a, H>,
        }

        impl<H: IRayHitHandler> IBroadPhaseRayTester for RayHitDispatcher<'_, H> {
            #[inline(always)]
            unsafe fn ray_test(
                &mut self,
                collidable: CollidableReference,
                ray_data: *const RayData,
                maximum_t: *mut f32,
            ) {
                if self
                    .shape_hit_handler
                    .hit_handler
                    .allow_test_collidable(&collidable)
                {
                    self.shape_hit_handler.collidable = collidable;
                    let (pose, shape) = self.simulation.get_pose_and_shape(collidable);
                    type RealShapes = crate::physics::collidables::shapes::Shapes;
                    let shapes = &*(self.simulation.shapes as *const RealShapes);
                    if let Some(batch) = shapes.get_batch(shape.type_id() as usize) {
                        batch.ray_test(
                            shape.index() as usize,
                            &*pose,
                            &*ray_data,
                            &mut *maximum_t,
                            &mut self.shape_hit_handler,
                        );
                    }
                }
            }
        }

        let mut dispatcher = RayHitDispatcher {
            simulation: self,
            shape_hit_handler: ShapeRayHitHandler {
                hit_handler,
                collidable: CollidableReference::from_raw(
                    crate::physics::collidables::collidable_reference::CollidableMobility::Static,
                    0,
                ),
            },
        };

        type RealBroadPhase = crate::physics::collision_detection::broad_phase::BroadPhase;
        let broad_phase = &*(self.broad_phase as *const RealBroadPhase);
        broad_phase.ray_cast(origin, direction, maximum_t, &mut dispatcher, id);
    }

    /// Sweeps a shape against the simulation.
    /// Simulation objects are treated as stationary during the sweep.
    ///
    /// # Safety
    /// The shape data pointer must be valid for the given shape type.
    /// The pool must be valid. All broad phase and narrow phase pointers must be valid.
    pub unsafe fn sweep<H: ISweepHitHandler>(
        &self,
        shape_data: *const u8,
        shape_type: i32,
        pose: &RigidPose,
        velocity: &BodyVelocity,
        maximum_t: f32,
        pool: *mut BufferPool,
        hit_handler: &mut H,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        min: Vec3,
        max: Vec3,
    ) {
        use crate::physics::collision_detection::ray_batchers::IBroadPhaseSweepTester;
        use crate::physics::collision_detection::sweep_task_registry::{
            ISweepFilter, SweepTaskRegistry,
        };

        // SweepFilter delegates child-level filtering to the user's hit handler.
        struct SweepFilter<'a, H: ISweepHitHandler> {
            hit_handler: &'a H,
            collidable_being_tested: CollidableReference,
        }

        impl<H: ISweepHitHandler> ISweepFilter for SweepFilter<'_, H> {
            #[inline(always)]
            fn allow_test(&self, _child_a: i32, child_b: i32) -> bool {
                // Simulation sweep does not permit nonconvex sweep shapes, so childA is irrelevant.
                self.hit_handler
                    .allow_test_child(&self.collidable_being_tested, child_b)
            }
        }

        // SweepHitDispatcher implements IBroadPhaseSweepTester for the broad phase traversal.
        struct SweepHitDispatcher<'a, H: ISweepHitHandler> {
            simulation: &'a Simulation,
            pool: *mut BufferPool,
            shape_data: *const u8,
            shape_type: i32,
            pose: RigidPose,
            velocity: BodyVelocity,
            hit_handler: &'a mut H,
            collidable_being_tested: CollidableReference,
            minimum_progression: f32,
            convergence_threshold: f32,
            maximum_iteration_count: i32,
        }

        impl<H: ISweepHitHandler> IBroadPhaseSweepTester for SweepHitDispatcher<'_, H> {
            #[inline(always)]
            fn test(&mut self, collidable: CollidableReference, maximum_t: &mut f32) {
                if self.hit_handler.allow_test(&collidable) {
                    let (target_pose, target_shape) =
                        unsafe { self.simulation.get_pose_and_shape(collidable) };
                    type RealShapes = crate::physics::collidables::shapes::Shapes;
                    let shapes = unsafe { &*(self.simulation.shapes as *const RealShapes) };
                    let (target_shape_data, _) = unsafe {
                        shapes
                            .get_batch(target_shape.type_id() as usize)
                            .unwrap()
                            .get_shape_data(target_shape.index() as usize)
                    };

                    self.collidable_being_tested = collidable;

                    type RealNarrowPhase =
                        crate::physics::collision_detection::narrow_phase::NarrowPhase;
                    let narrow_phase =
                        unsafe { &*(self.simulation.narrow_phase as *const RealNarrowPhase) };
                    let sweep_task_registry = unsafe { &*narrow_phase.sweep_task_registry };

                    if let Some(task) =
                        sweep_task_registry.get_task(self.shape_type, target_shape.type_id())
                    {
                        let zero_velocity = BodyVelocity::default();
                        let mut t0 = 0.0f32;
                        let mut t1 = 0.0f32;
                        let mut hit_location = Vec3::ZERO;
                        let mut hit_normal = Vec3::ZERO;

                        let mut filter = SweepFilter {
                            hit_handler: &*self.hit_handler,
                            collidable_being_tested: collidable,
                        };

                        let result = unsafe {
                            task.sweep_filtered(
                                self.shape_data,
                                self.shape_type,
                                self.pose.orientation,
                                &self.velocity,
                                target_shape_data,
                                target_shape.type_id(),
                                unsafe { (*target_pose).position } - self.pose.position,
                                unsafe { (*target_pose).orientation },
                                &zero_velocity,
                                *maximum_t,
                                self.minimum_progression,
                                self.convergence_threshold,
                                self.maximum_iteration_count,
                                &mut filter as *mut SweepFilter<'_, H> as *mut u8,
                                self.simulation.shapes
                                    as *mut crate::physics::collidables::shapes::Shapes,
                                narrow_phase.sweep_task_registry as *mut SweepTaskRegistry,
                                self.pool,
                                &mut t0,
                                &mut t1,
                                &mut hit_location,
                                &mut hit_normal,
                            )
                        };

                        if result {
                            if t1 > 0.0 {
                                hit_location += self.pose.position;
                                self.hit_handler.on_hit(
                                    maximum_t,
                                    t1,
                                    hit_location,
                                    hit_normal,
                                    &collidable,
                                );
                            } else {
                                self.hit_handler.on_hit_at_zero_t(maximum_t, &collidable);
                            }
                        }
                    }
                }
            }
        }

        let mut dispatcher = SweepHitDispatcher {
            simulation: self,
            pool,
            shape_data,
            shape_type,
            pose: *pose,
            velocity: *velocity,
            hit_handler,
            collidable_being_tested: CollidableReference::from_raw(
                crate::physics::collidables::collidable_reference::CollidableMobility::Static,
                0,
            ),
            minimum_progression,
            convergence_threshold,
            maximum_iteration_count,
        };

        type RealBroadPhase = crate::physics::collision_detection::broad_phase::BroadPhase;
        let broad_phase = &*(self.broad_phase as *const RealBroadPhase);
        broad_phase.sweep_minmax(min, max, velocity.linear, maximum_t, &mut dispatcher);
    }

    /// Sweeps a convex shape against the simulation with automatically estimated termination conditions.
    /// Simulation objects are treated as stationary during the sweep.
    ///
    /// # Safety
    /// The shape must be a valid convex shape. All simulation pointers must be valid.
    pub unsafe fn sweep_shape<TShape: IConvexShape, H: ISweepHitHandler>(
        &self,
        shape: &TShape,
        pose: &RigidPose,
        velocity: &BodyVelocity,
        maximum_t: f32,
        pool: *mut BufferPool,
        hit_handler: &mut H,
    ) {
        // Estimate reasonable termination conditions from shape size.
        let mut maximum_radius = 0.0f32;
        let mut maximum_angular_expansion = 0.0f32;
        shape.compute_angular_expansion_data(&mut maximum_radius, &mut maximum_angular_expansion);
        let minimum_radius = maximum_radius - maximum_angular_expansion;
        let size_estimate = minimum_radius.max(maximum_radius * 0.25);
        let minimum_progression_distance = 0.1 * size_estimate;
        let convergence_threshold_distance = 1e-5 * size_estimate;
        let tangent_velocity =
            (velocity.angular.length() * maximum_radius).min(maximum_angular_expansion / maximum_t);
        let inverse_velocity = 1.0 / (velocity.linear.length() + tangent_velocity);
        let minimum_progression_t = minimum_progression_distance * inverse_velocity;
        let convergence_threshold_t = convergence_threshold_distance * inverse_velocity;
        let maximum_iteration_count = 25;

        // Compute bounding box.
        let mut min = Vec3::ZERO;
        let mut max = Vec3::ZERO;
        shape.compute_bounds(pose.orientation, &mut min, &mut max);
        let angular_expansion = Vec3::splat(
            crate::physics::bounding_box_helpers::BoundingBoxHelpers::get_angular_bounds_expansion(
                velocity.angular.length(),
                maximum_t,
                maximum_radius,
                maximum_angular_expansion,
            ),
        );
        min = min - angular_expansion + pose.position;
        max = max + angular_expansion + pose.position;

        self.sweep(
            shape as *const TShape as *const u8,
            TShape::type_id(),
            pose,
            velocity,
            maximum_t,
            pool,
            hit_handler,
            minimum_progression_t,
            convergence_threshold_t,
            maximum_iteration_count,
            min,
            max,
        );
    }
}
