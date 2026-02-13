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
/// Only used by the SIMD-batched ray path (not yet implemented).
/// The per-ray path in BroadPhaseRayBatcher dispatches rays immediately.
pub struct RaySource {
    _placeholder: i32,
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

/// Helps test the broad phase's active and static trees with a custom leaf tester.
/// Currently uses per-ray traversal via BroadPhase::ray_cast.
/// A future optimization would use the batched RayBatcher for SIMD ray streaming.
pub struct BroadPhaseRayBatcher<'a, TRayTester: IBroadPhaseRayTester> {
    broad_phase: *const BroadPhase,
    pool: *mut BufferPool,
    ray_tester: &'a mut TRayTester,
}

impl<'a, TRayTester: IBroadPhaseRayTester> BroadPhaseRayBatcher<'a, TRayTester> {
    pub fn new(
        pool: *mut BufferPool,
        broad_phase: *const BroadPhase,
        ray_tester: &'a mut TRayTester,
    ) -> Self {
        Self {
            broad_phase,
            pool,
            ray_tester,
        }
    }

    /// Adds a ray to the batcher. Tests immediately via per-ray traversal.
    pub fn add(&mut self, origin: &Vec3, direction: &Vec3, maximum_t: f32, id: i32) {
        unsafe {
            (&*self.broad_phase).ray_cast(*origin, *direction, maximum_t, self.ray_tester, id);
        }
    }

    /// Tests any accumulated rays against the broad phase trees and resets.
    /// In the per-ray implementation, rays are tested immediately in add(), so this is a no-op.
    pub fn flush(&mut self) {
        // Per-ray implementation: rays are dispatched immediately in add().
    }

    /// Disposes the batcher resources.
    pub fn dispose(&mut self) {
        // Per-ray implementation: no backing buffers to dispose.
    }
}
