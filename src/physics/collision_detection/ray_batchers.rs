// Translated from BepuPhysics/CollisionDetection/RayBatchers.cs

use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::simulation::Simulation;
use crate::physics::trees::ray_batcher::{IBatchedRayLeafTester, RayBatcher};
use crate::physics::trees::tree_ray_cast::IRayLeafTester;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::span_helper;
use glam::Vec3;

/// Ray data passed to leaf testers; re-exported so the broad phase and tree layers share one 28-byte definition.
pub use crate::physics::trees::ray_batcher::RayData;
/// A source of batched ray data, re-exported from the tree-level ray batching module.
pub use crate::physics::trees::ray_batcher::RaySource;
pub use crate::physics::trees::ray_batcher::RaySourceTrait;

/// Callback for testing a ray against a specific collidable.
pub trait IBroadPhaseRayTester {
    /// Tests a ray against a collidable.
    unsafe fn ray_test(
        &mut self,
        collidable: CollidableReference,
        ray_data: *const RayData,
        maximum_t: *mut f32,
    );
}

/// Extended ray tester that can also handle batched ray sources.
pub trait IBroadPhaseBatchedRayTester: IBroadPhaseRayTester {
    /// Tests a batch of rays against the collidable.
    fn ray_test_batched(
        &mut self,
        collidable: CollidableReference,
        rays: &mut RaySource,
        pool: &mut BufferPool,
    );
}

/// Callback for handling ray hits against the simulation.
pub trait IRayHitHandler {
    /// Whether to allow testing against the given collidable.
    fn allow_test(&self, collidable: CollidableReference) -> bool;

    /// Whether to allow testing against a specific child of a collidable.
    /// Only called by shape types that can have more than one child (compounds, meshes).
    fn allow_test_child(&self, collidable: CollidableReference, child_index: i32) -> bool;

    /// Called when a ray intersects a collidable.
    fn on_ray_hit(
        &mut self,
        ray: &RayData,
        maximum_t: &mut f32,
        t: f32,
        normal: Vec3,
        collidable: CollidableReference,
        child_index: i32,
    );
}

/// Callback for shape-level ray hit handling.
pub trait IShapeRayHitHandler {
    /// Whether to allow testing against child shapes.
    fn allow_test(&self, child_index: i32) -> bool;

    /// Called when a ray hit is detected.
    fn on_ray_hit(
        &mut self,
        ray: &RayData,
        maximum_t: &mut f32,
        t: f32,
        normal: Vec3,
        child_index: i32,
    );
}

/// Callback for broad phase sweep tests.
pub trait IBroadPhaseSweepTester {
    /// Tests a sweep against a collidable.
    fn test(&mut self, collidable: CollidableReference, maximum_t: &mut f32);
}

/// Adapts a borrowed `TRayTester` plus a leaf buffer into the leaf testers `RayBatcher::test_rays` expects.
struct LeafTester<'a, TRayTester: IBroadPhaseBatchedRayTester> {
    ray_tester: &'a mut TRayTester,
    leaves: Buffer<CollidableReference>,
}

impl<'a, TRayTester: IBroadPhaseBatchedRayTester> IRayLeafTester for LeafTester<'a, TRayTester> {
    #[inline(always)]
    unsafe fn test_leaf(
        &mut self,
        leaf_index: i32,
        ray_data: *mut RayData,
        maximum_t: *mut f32,
        _pool: &mut BufferPool,
    ) {
        let collidable = *self.leaves.get(leaf_index);
        self.ray_tester.ray_test(collidable, ray_data, maximum_t);
    }
}

impl<'a, TRayTester: IBroadPhaseBatchedRayTester> IBatchedRayLeafTester
    for LeafTester<'a, TRayTester>
{
    #[inline(always)]
    fn ray_test(&mut self, leaf_index: i32, rays: &mut RaySource, pool: &mut BufferPool) {
        let collidable = *self.leaves.get(leaf_index);
        self.ray_tester.ray_test_batched(collidable, rays, pool);
    }
}

/// Helps test the broad phase's active and static trees with a custom leaf tester.
pub struct BroadPhaseRayBatcher<TRayTester: IBroadPhaseBatchedRayTester> {
    broad_phase: *const BroadPhase,
    batcher: RayBatcher,
    ray_tester: TRayTester,
    active_leaves: Buffer<CollidableReference>,
    static_leaves: Buffer<CollidableReference>,
}

impl<TRayTester: IBroadPhaseBatchedRayTester> BroadPhaseRayBatcher<TRayTester> {
    /// Constructs a ray batcher for the broad phase and initializes its backing resources.
    ///
    /// # Arguments
    /// * `pool` - Pool to pull resources from.
    /// * `broad_phase` - Broad phase to be tested.
    /// * `ray_tester` - Ray tester used to test leaves found by the broad phase tree traversals.
    /// * `batcher_ray_capacity` - Maximum number of rays to execute in each traversal.
    ///   This should typically be chosen as the highest value which avoids spilling data out of L2 cache.
    pub fn new(
        pool: *mut BufferPool,
        broad_phase: *const BroadPhase,
        ray_tester: TRayTester,
        batcher_ray_capacity: i32,
    ) -> Self {
        let bp = unsafe { &*broad_phase };
        let max_leaf_count = bp.static_tree.leaf_count.max(bp.active_tree.leaf_count);
        let preallocated_tree_depth =
            8i32.max(2 * span_helper::get_containing_power_of_2(max_leaf_count));
        Self {
            broad_phase,
            batcher: RayBatcher::new(pool, batcher_ray_capacity, preallocated_tree_depth),
            ray_tester,
            active_leaves: bp.active_leaves,
            static_leaves: bp.static_leaves,
        }
    }

    /// Adds a ray to the batcher to test against the broad phase trees.
    /// If the underlying ray batcher hits its maximum capacity, all the accumulated rays will be
    /// tested against the broad phase trees and the accumulator will be reset.
    #[inline(always)]
    pub fn add(&mut self, origin: Vec3, direction: Vec3, maximum_t: f32, id: i32) {
        if self.batcher.add(origin, direction, maximum_t, id) {
            // TODO: Note that this order implies we test against the active tree before the static tree.
            // This should be revisited - there are many simulations in which testing the static tree first
            // would be better because of more conservative maximumT values. Not immediately clear which
            // case is dominant.
            unsafe {
                let broad_phase = &*self.broad_phase;
                let mut active_tester = LeafTester {
                    ray_tester: &mut self.ray_tester,
                    leaves: self.active_leaves,
                };
                self.batcher
                    .test_rays(&broad_phase.active_tree, &mut active_tester);
                let mut static_tester = LeafTester {
                    ray_tester: &mut self.ray_tester,
                    leaves: self.static_leaves,
                };
                self.batcher
                    .test_rays(&broad_phase.static_tree, &mut static_tester);
            }
            self.batcher.reset_rays();
        }
    }

    /// Tests any accumulated rays against the broad phase trees and then resets the batcher.
    pub fn flush(&mut self) {
        if self.batcher.ray_count() > 0 {
            // TODO: Similar to Add - order matters here for performance. Need testing to determine which
            // order tends to be better.
            unsafe {
                let broad_phase = &*self.broad_phase;
                let mut active_tester = LeafTester {
                    ray_tester: &mut self.ray_tester,
                    leaves: self.active_leaves,
                };
                self.batcher
                    .test_rays(&broad_phase.active_tree, &mut active_tester);
                let mut static_tester = LeafTester {
                    ray_tester: &mut self.ray_tester,
                    leaves: self.static_leaves,
                };
                self.batcher
                    .test_rays(&broad_phase.static_tree, &mut static_tester);
            }
            self.batcher.reset_rays();
        }
    }

    /// Disposes the underlying batcher resources.
    pub fn dispose(&mut self) {
        self.batcher.dispose();
    }
}

/// Adapts a user's `IRayHitHandler` into the `IShapeRayHitHandler` expected by `ShapeBatch::ray_test`.
struct ShapeHitHandler<TRayHitHandler: IRayHitHandler> {
    hit_handler: TRayHitHandler,
    reference: CollidableReference,
}

impl<TRayHitHandler: IRayHitHandler> IShapeRayHitHandler for ShapeHitHandler<TRayHitHandler> {
    #[inline(always)]
    fn allow_test(&self, child_index: i32) -> bool {
        self.hit_handler.allow_test_child(self.reference, child_index)
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
        self.hit_handler
            .on_ray_hit(ray, maximum_t, t, normal, self.reference, child_index);
    }
}

/// Looks up a collidable's pose and shape and forwards into the shape's ray test.
/// Holds the caller's `pool` because the scalar `ray_test` path has no pool parameter.
struct SimulationRayDispatcher<TRayHitHandler: IRayHitHandler> {
    simulation: *const Simulation,
    pool: *mut BufferPool,
    hit_handler: ShapeHitHandler<TRayHitHandler>,
}

impl<TRayHitHandler: IRayHitHandler> IBroadPhaseRayTester for SimulationRayDispatcher<TRayHitHandler> {
    #[inline(always)]
    unsafe fn ray_test(
        &mut self,
        collidable: CollidableReference,
        ray_data: *const RayData,
        maximum_t: *mut f32,
    ) {
        if self.hit_handler.hit_handler.allow_test(collidable) {
            let simulation = &*self.simulation;
            let (pose, shape) = simulation.get_pose_and_shape(collidable);
            self.hit_handler.reference = collidable;
            type RealShapes = crate::physics::collidables::shapes::Shapes;
            let shapes = &*(simulation.shapes as *const RealShapes);
            if let Some(batch) = shapes.get_batch(shape.type_id() as usize) {
                batch.ray_test(
                    shape.index() as usize,
                    &*pose,
                    &*ray_data,
                    &mut *maximum_t,
                    &mut *self.pool,
                    &mut self.hit_handler,
                );
            }
        }
    }
}

impl<TRayHitHandler: IRayHitHandler> IBroadPhaseBatchedRayTester
    for SimulationRayDispatcher<TRayHitHandler>
{
    #[inline(always)]
    fn ray_test_batched(
        &mut self,
        collidable: CollidableReference,
        rays: &mut RaySource,
        pool: &mut BufferPool,
    ) {
        if self.hit_handler.hit_handler.allow_test(collidable) {
            unsafe {
                let simulation = &*self.simulation;
                let (pose, shape) = simulation.get_pose_and_shape(collidable);
                self.hit_handler.reference = collidable;
                type RealShapes = crate::physics::collidables::shapes::Shapes;
                let shapes = &*(simulation.shapes as *const RealShapes);
                if let Some(batch) = shapes.get_batch(shape.type_id() as usize) {
                    batch.ray_test_batched(
                        shape.index() as usize,
                        &*pose,
                        rays,
                        pool,
                        &mut self.hit_handler,
                    );
                }
            }
        }
    }
}

/// Tests batches of rays against the simulation.
pub struct SimulationRayBatcher<TRayHitHandler: IRayHitHandler> {
    batcher: BroadPhaseRayBatcher<SimulationRayDispatcher<TRayHitHandler>>,
}

impl<TRayHitHandler: IRayHitHandler> SimulationRayBatcher<TRayHitHandler> {
    /// Constructs a batcher for testing rays against a simulation.
    ///
    /// # Arguments
    /// * `batcher_ray_capacity` - Maximum number of rays to execute in each traversal.
    pub fn new(
        pool: *mut BufferPool,
        simulation: *const Simulation,
        hit_handler: TRayHitHandler,
        batcher_ray_capacity: i32,
    ) -> Self {
        let dispatcher = SimulationRayDispatcher {
            simulation,
            pool,
            hit_handler: ShapeHitHandler {
                hit_handler,
                reference: CollidableReference::from_raw(CollidableMobility::Static, 0),
            },
        };
        unsafe {
            type RealBroadPhase = crate::physics::collision_detection::broad_phase::BroadPhase;
            let broad_phase = (*simulation).broad_phase as *const RealBroadPhase;
            Self {
                batcher: BroadPhaseRayBatcher::new(
                    pool,
                    broad_phase,
                    dispatcher,
                    batcher_ray_capacity,
                ),
            }
        }
    }

    /// Adds a ray to the batcher to test against the simulation.
    /// If the underlying ray batcher hits its maximum capacity, all the accumulated rays will be
    /// tested against the simulation and the accumulator will be reset.
    #[inline(always)]
    pub fn add(&mut self, origin: Vec3, direction: Vec3, maximum_t: f32, id: i32) {
        self.batcher.add(origin, direction, maximum_t, id);
    }

    /// Tests any accumulated rays against the broad phase trees and then resets the batcher.
    #[inline(always)]
    pub fn flush(&mut self) {
        self.batcher.flush();
    }

    /// Disposes the underlying batcher resources.
    pub fn dispose(&mut self) {
        self.batcher.dispose();
    }
}
