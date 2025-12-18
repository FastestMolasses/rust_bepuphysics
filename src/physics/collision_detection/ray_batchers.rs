// Translated from BepuPhysics/CollisionDetection/RayBatchers.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

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
    fn ray_test_batched(&mut self, collidable: CollidableReference, rays: &mut RaySource);
}

/// Ray data passed to leaf testers.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RayData {
    pub origin: Vec3,
    pub id: i32,
    pub direction: Vec3,
    _padding: f32,
}

impl Default for RayData {
    fn default() -> Self {
        Self {
            origin: Vec3::ZERO,
            id: 0,
            direction: Vec3::ZERO,
            _padding: 0.0,
        }
    }
}

/// A source of ray data for batched testing.
pub struct RaySource {
    // TODO: implement once the ray batcher infrastructure is fully wired
    _placeholder: i32,
}

/// Callback for handling ray hits against the simulation.
pub trait IRayHitHandler {
    /// Whether to allow testing against the given collidable.
    fn allow_test(&self, collidable: CollidableReference) -> bool;

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

/// Helps test the broad phase's active and static trees with a custom leaf tester.
pub struct BroadPhaseRayBatcher {
    broad_phase: *const BroadPhase,
    pool: *mut BufferPool,
    // TODO: Add RayBatcher once tree ray batching is implemented
}

impl BroadPhaseRayBatcher {
    pub fn new(pool: *mut BufferPool, broad_phase: *const BroadPhase) -> Self {
        Self { broad_phase, pool }
    }

    /// Adds a ray to the batcher. If the batch is full, rays are automatically tested.
    pub fn add(&mut self, _origin: &Vec3, _direction: &Vec3, _maximum_t: f32, _id: i32) {
        // TODO: Batch rays and test against trees when batch is full
    }

    /// Tests any accumulated rays against the broad phase trees and resets.
    pub fn flush(&mut self) {
        // TODO: Flush remaining rays
    }

    /// Disposes the batcher resources.
    pub fn dispose(&mut self) {
        // TODO: Return ray buffer to pool
    }
}
