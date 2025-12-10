//! Ray batching system for efficient BVH traversal.
//!
//! This module provides types for batching multiple rays together for more efficient
//! broad-phase traversal through the tree structure.

use glam::Vec3;
use crate::utilities::memory::{buffer::Buffer, buffer_pool::BufferPool};

/// Raw ray data containing origin, direction, and an identifier.
#[repr(C)]
pub struct RayData {
    pub origin: Vec3,
    pub id: i32,
    pub direction: Vec3,
}

/// Ray representation designed for quicker intersection against axis aligned bounding boxes.
#[repr(C)]
pub struct TreeRay {
    pub origin_over_direction: Vec3,
    pub maximum_t: f32,
    pub inverse_direction: Vec3,
}

impl TreeRay {
    /// Creates a TreeRay from an origin, direction, and maximum t value.
    #[inline(always)]
    pub fn create_from(origin: Vec3, direction: Vec3, maximum_t: f32, tree_ray: &mut TreeRay) {
        // Note that this division has two odd properties:
        // 1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value.
        //    This is a hack, but it works reasonably well. The idea is that any interval computed using such an inverse
        //    would be enormous. Those values will not be exactly accurate, but they will never appear as a result
        //    because a parallel ray will never actually intersect the surface. The resulting intervals are practical
        //    approximations of the 'true' infinite intervals.
        // 2) To compensate for the clamp and abs, we reintroduce the sign in the numerator.
        tree_ray.inverse_direction = Vec3::new(
            if direction.x < 0.0 { -1.0 } else { 1.0 },
            if direction.y < 0.0 { -1.0 } else { 1.0 },
            if direction.z < 0.0 { -1.0 } else { 1.0 },
        ) / Vec3::max(Vec3::splat(1e-15), Vec3::abs(direction));
        tree_ray.maximum_t = maximum_t;
        tree_ray.origin_over_direction = origin * tree_ray.inverse_direction;
    }

    /// Creates both a RayData and TreeRay from ray parameters.
    #[inline(always)]
    pub fn create_from_ray(
        origin: Vec3,
        direction: Vec3,
        maximum_t: f32,
        id: i32,
        ray: &mut RayData,
        tree_ray: &mut TreeRay,
    ) {
        ray.origin = origin;
        ray.direction = direction;
        ray.id = id;
        Self::create_from(origin, direction, maximum_t, tree_ray);
    }
}

/// Trait for types that can provide ray data.
pub trait RaySourceTrait {
    /// Gets the number of rays in the batch.
    fn ray_count(&self) -> i32;
    /// Gets a reference to the data for a ray.
    fn get_ray(&self, ray_index: i32) -> &RayData;
    /// Gets pointers to the data for a ray (origin/direction and maximum_t).
    unsafe fn get_ray_ptrs(&self, ray_index: i32) -> (*mut RayData, *mut f32);
}

/// A source of rays with indirection through ray pointers.
#[repr(C)]
pub struct RaySource {
    tree_rays: *mut TreeRay,
    rays: *mut RayData,
    ray_pointers: *mut u16,
    ray_count: i32,
}

impl RaySourceTrait for RaySource {
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    #[inline(always)]
    fn get_ray(&self, ray_index: i32) -> &RayData {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The ray index must be within 0 and RayCount - 1."
        );
        unsafe {
            let remapped_index = *self.ray_pointers.offset(ray_index as isize);
            &*self.rays.offset(remapped_index as isize)
        }
    }

    #[inline(always)]
    unsafe fn get_ray_ptrs(&self, ray_index: i32) -> (*mut RayData, *mut f32) {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The ray index must be within 0 and RayCount - 1."
        );
        let remapped_index = *self.ray_pointers.offset(ray_index as isize);
        let ray = self.rays.offset(remapped_index as isize);
        let maximum_t = &mut (*self.tree_rays.offset(remapped_index as isize)).maximum_t as *mut f32;
        (ray, maximum_t)
    }
}

/// Entry on the traversal stack.
#[repr(C)]
struct StackEntry {
    node_index: i32,
    ray_count: u16,
    ray_stack: u8,
    depth: u8,
}

/// Trait for tree ray sources used during traversal.
trait TreeRaySourceTrait {
    fn ray_count(&self) -> i32;
    fn get(&self, ray_index: i32) -> i32;
}

/// Root ray source that returns indices directly.
#[repr(C)]
struct RootRaySource {
    ray_count: i32,
}

impl RootRaySource {
    #[inline(always)]
    pub fn new(ray_count: i32) -> Self {
        Self { ray_count }
    }
}

impl TreeRaySourceTrait for RootRaySource {
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    #[inline(always)]
    fn get(&self, ray_index: i32) -> i32 {
        ray_index
    }
}

/// Tree ray source that uses indirection through pointers.
#[repr(C)]
struct TreeRaySource {
    ray_pointers: *mut u16,
    ray_count: i32,
}

impl TreeRaySource {
    #[inline(always)]
    pub fn new(ray_pointers: *mut u16, ray_count: i32) -> Self {
        Self {
            ray_pointers,
            ray_count,
        }
    }
}

impl TreeRaySourceTrait for TreeRaySource {
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    #[inline(always)]
    fn get(&self, ray_index: i32) -> i32 {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The requested ray index must be within the source's region."
        );
        unsafe { *self.ray_pointers.offset(ray_index as isize) as i32 }
    }
}

/// Batches rays for efficient tree traversal.
///
/// This is a stub implementation - the full ray batching logic will be implemented later.
pub struct RayBatcher {
    stack_pointer_a0: i32,
    stack_pointer_a1: i32,
    stack_pointer_b: i32,
    stack_pointer: i32,
    stack: Buffer<StackEntry>,
    batch_ray_count: i32,
    batch_rays: Buffer<TreeRay>,
    batch_original_rays: Buffer<RayData>,
    ray_capacity: i32,
    preallocated_tree_depth: i32,
    fallback_stack: Buffer<i32>,
}

impl RayBatcher {
    /// Creates a new RayBatcher.
    ///
    /// # Arguments
    /// * `pool` - Buffer pool to allocate from.
    /// * `ray_capacity` - Maximum number of rays that can be batched.
    /// * `tree_depth_for_preallocation` - Depth of tree to preallocate stack space for.
    #[inline(always)]
    pub fn new(pool: &mut BufferPool, ray_capacity: i32, tree_depth_for_preallocation: i32) -> Self {
        // Calculate stack size based on tree depth and ray capacity
        let stack_size = ray_capacity * 4; // Conservative estimate
        
        Self {
            stack_pointer_a0: 0,
            stack_pointer_a1: 0,
            stack_pointer_b: 0,
            stack_pointer: 0,
            stack: pool.take(stack_size),
            batch_ray_count: 0,
            batch_rays: pool.take(ray_capacity),
            batch_original_rays: pool.take(ray_capacity),
            ray_capacity,
            preallocated_tree_depth: tree_depth_for_preallocation,
            fallback_stack: pool.take(ray_capacity),
        }
    }

    /// Gets the maximum ray capacity of this batcher.
    #[inline(always)]
    pub fn ray_capacity(&self) -> i32 {
        self.ray_capacity
    }

    /// Gets the current number of rays in the batch.
    #[inline(always)]
    pub fn ray_count(&self) -> i32 {
        self.batch_ray_count
    }

    /// Resets the ray batch, clearing all rays.
    #[inline(always)]
    pub fn reset_rays(&mut self) {
        self.batch_ray_count = 0;
        self.stack_pointer_a0 = 0;
        self.stack_pointer_a1 = 0;
        self.stack_pointer_b = 0;
        self.stack_pointer = 0;
    }

    /// Disposes the batcher, returning buffers to the pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.stack);
        pool.return_buffer(&mut self.batch_rays);
        pool.return_buffer(&mut self.batch_original_rays);
        pool.return_buffer(&mut self.fallback_stack);
    }

    // TODO: Implement the following methods:
    // - add() - Add a ray to the batch
    // - test_rays() - Test all rays in the batch against the tree
    // - test_node() - Test rays against a specific node
}
