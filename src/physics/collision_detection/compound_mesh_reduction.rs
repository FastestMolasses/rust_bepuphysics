// Translated from BepuPhysics/CollisionDetection/CompoundMeshReduction.cs

use crate::physics::collision_detection::collision_batcher_continuations::{
    ICollisionTestContinuation, PairContinuation,
};
use crate::physics::collision_detection::contact_manifold::ConvexContactManifold;
use crate::physics::collision_detection::mesh_reduction::{BoundingBox, MeshReduction, Triangle};
use crate::physics::collision_detection::nonconvex_reduction::{FlushResult, NonconvexReduction};
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Quat;

/// Handles mesh boundary smoothing for compound-mesh collisions.
/// Extends the mesh reduction concept to handle multiple child regions from compound shapes.
#[repr(C)]
pub struct CompoundMeshReduction {
    pub region_count: i32,
    pub child_manifold_regions: Buffer<(i32, i32)>, // (Start, Count) pairs
    pub query_bounds: Buffer<BoundingBox>,
    pub triangles: Buffer<Triangle>,
    /// Whether the pair was flipped (mesh is considered shape B).
    pub requires_flip: bool,
    /// Orientation of the mesh for transforming contacts.
    pub mesh_orientation: Quat,
    /// Nested nonconvex reduction for final manifold assembly.
    pub inner: NonconvexReduction,
    /// Pointer to the mesh. TODO: Make flexible for different mesh types.
    pub mesh: *mut u8,
}

impl ICollisionTestContinuation for CompoundMeshReduction {
    fn create(&mut self, child_manifold_count: i32, pool: &mut BufferPool) {
        self.inner.create(child_manifold_count, pool);
    }
}

impl Default for CompoundMeshReduction {
    fn default() -> Self {
        Self {
            region_count: 0,
            child_manifold_regions: Buffer::default(),
            query_bounds: Buffer::default(),
            triangles: Buffer::default(),
            requires_flip: false,
            mesh_orientation: Quat::IDENTITY,
            inner: NonconvexReduction::default(),
            mesh: std::ptr::null_mut(),
        }
    }
}

impl Copy for CompoundMeshReduction {}
impl Clone for CompoundMeshReduction {
    fn clone(&self) -> Self {
        *self
    }
}

impl CompoundMeshReduction {
    /// Records a completed child manifold.
    #[inline(always)]
    pub unsafe fn on_child_completed(
        &mut self,
        report: &PairContinuation,
        manifold: &ConvexContactManifold,
    ) {
        self.inner.on_child_completed(report, manifold);
    }

    /// Records an untested child as having zero contacts.
    #[inline(always)]
    pub unsafe fn on_untested_child_completed(&mut self, report: &PairContinuation) {
        self.inner.on_untested_child_completed(report);
    }

    /// Tries to flush the compound-mesh reduction if all children are completed.
    pub unsafe fn try_flush(&mut self, pair_id: i32, pool: &mut BufferPool) -> Option<FlushResult> {
        debug_assert!(self.inner.child_count > 0);
        if self.inner.completed_child_count == self.inner.child_count {
            let mut mesh_orientation = Matrix3x3::default();
            Matrix3x3::create_from_quaternion(&self.mesh_orientation, &mut mesh_orientation);
            let mut mesh_inverse_orientation = Matrix3x3::default();
            Matrix3x3::transpose_ref(&mesh_orientation, &mut mesh_inverse_orientation);

            for i in 0..self.region_count {
                let region = *self.child_manifold_regions.get(i);
                if region.1 > 0 {
                    MeshReduction::reduce_manifolds(
                        &self.triangles,
                        &mut self.inner.children,
                        region.0,
                        region.1,
                        self.requires_flip,
                        self.query_bounds.get(i),
                        &mesh_orientation,
                        &mesh_inverse_orientation,
                        self.mesh,
                        pool,
                    );
                }
            }

            // Clean up continuation resources.
            pool.return_buffer(&mut self.triangles);
            pool.return_buffer(&mut self.child_manifold_regions);
            pool.return_buffer(&mut self.query_bounds);
            // Now flush the inner nonconvex reduction to produce the final manifold.
            return Some(self.inner.flush(pair_id, pool));
        }
        None
    }
}
