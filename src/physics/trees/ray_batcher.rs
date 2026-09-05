//! Ray batching system for efficient BVH traversal.
//!
//! This module provides types for batching multiple rays together for more efficient
//! broad-phase traversal through the tree structure.

use super::node::Node;
use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
use super::tree_ray_cast::IRayLeafTester;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::{buffer::Buffer, buffer_pool::BufferPool};
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Raw ray data containing origin, direction, and an identifier.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RayData {
    pub origin: Vec3,
    pub id: i32,
    pub direction: Vec3,
}

impl Default for RayData {
    fn default() -> Self {
        Self {
            origin: Vec3::ZERO,
            id: 0,
            direction: Vec3::ZERO,
        }
    }
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
        let maximum_t =
            &mut (*self.tree_rays.offset(remapped_index as isize)).maximum_t as *mut f32;
        (ray, maximum_t)
    }
}

/// Extends `IRayLeafTester` with the batched-ray leaf test used during wide tree traversal.
pub trait IBatchedRayLeafTester: IRayLeafTester {
    /// Tests a batch of rays (routed through the same leaf) at once.
    fn ray_test(&mut self, leaf_index: i32, rays: &mut RaySource, pool: &mut BufferPool);
}

/// Entry on the traversal stack.
#[repr(C)]
#[derive(Clone, Copy, Default)]
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

/// Ray representation designed for quicker intersection against axis aligned bounding boxes,
/// bundled into SIMD lanes.
#[derive(Clone, Copy, Default)]
struct TreeRayWide {
    origin_over_direction: Vector3Wide,
    maximum_t: Vector<f32>,
    inverse_direction: Vector3Wide,
}

impl TreeRayWide {
    /// Scatters a single scalar ray into one lane of a wide ray bundle.
    #[inline(always)]
    fn gather_into_slot(ray: &TreeRay, wide: &mut TreeRayWide, lane: usize) {
        Vector3Wide::write_slot(
            ray.origin_over_direction,
            lane,
            &mut wide.origin_over_direction,
        );
        unsafe {
            *GatherScatter::get_mut(&mut wide.maximum_t, lane) = ray.maximum_t;
        }
        Vector3Wide::write_slot(ray.inverse_direction, lane, &mut wide.inverse_direction);
    }
}

/// A tree node broadcast into SIMD lanes for wide ray intersection.
#[derive(Clone, Copy, Default)]
struct NodeWide {
    min_a: Vector3Wide,
    max_a: Vector3Wide,
    min_b: Vector3Wide,
    max_b: Vector3Wide,
}

#[inline(always)]
fn broadcast_node(node: &Node) -> NodeWide {
    NodeWide {
        min_a: Vector3Wide::broadcast(node.a.min),
        max_a: Vector3Wide::broadcast(node.a.max),
        min_b: Vector3Wide::broadcast(node.b.min),
        max_b: Vector3Wide::broadcast(node.b.max),
    }
}

#[inline(always)]
fn intersect(
    ray: &TreeRayWide,
    min: &Vector3Wide,
    max: &Vector3Wide,
) -> (Vector<f32>, Vector<i32>) {
    let t_x0 = min.x * ray.inverse_direction.x - ray.origin_over_direction.x;
    let t_x1 = max.x * ray.inverse_direction.x - ray.origin_over_direction.x;
    let t_min_x = t_x0.simd_min(t_x1);
    let t_max_x = t_x0.simd_max(t_x1);

    let t_y0 = min.y * ray.inverse_direction.y - ray.origin_over_direction.y;
    let t_y1 = max.y * ray.inverse_direction.y - ray.origin_over_direction.y;
    let t_min_y = t_y0.simd_min(t_y1);
    let t_max_y = t_y0.simd_max(t_y1);

    let t_z0 = min.z * ray.inverse_direction.z - ray.origin_over_direction.z;
    let t_z1 = max.z * ray.inverse_direction.z - ray.origin_over_direction.z;
    let t_min_z = t_z0.simd_min(t_z1);
    let t_max_z = t_z0.simd_max(t_z1);

    let t_min = Vector::<f32>::splat(0.0)
        .simd_max(t_min_x)
        .simd_max(t_min_y.simd_max(t_min_z));
    let t_max = ray
        .maximum_t
        .simd_min(t_max_x)
        .simd_min(t_max_y.simd_min(t_max_z));
    let intersected = t_min.simd_le(t_max).to_simd();
    (t_min, intersected)
}

/// Per-ray fallback traversal; duplicates the private `Tree::ray_cast_node`. Keep in sync with it.
unsafe fn tree_ray_cast_fallback<TLeafTester: IRayLeafTester>(
    tree: &Tree,
    mut node_index: i32,
    tree_ray: *mut TreeRay,
    ray_data: *mut RayData,
    mut stack: Buffer<i32>,
    pool: &mut BufferPool,
    leaf_tester: &mut TLeafTester,
) {
    debug_assert!(
        (node_index >= 0 && node_index < tree.node_count)
            || (Tree::encode(node_index) >= 0 && Tree::encode(node_index) < tree.leaf_count)
    );
    debug_assert!(
        tree.leaf_count >= 2,
        "This implementation assumes all nodes are filled."
    );

    let mut stack_end: i32 = 0;
    loop {
        if node_index < 0 {
            // This is actually a leaf node.
            let leaf_index = Tree::encode(node_index);
            leaf_tester.test_leaf(leaf_index, ray_data, &mut (*tree_ray).maximum_t, pool);
            // Leaves have no children; pull from the stack.
            if stack_end == 0 {
                break;
            }
            stack_end -= 1;
            node_index = *stack.get(stack_end);
        } else {
            let node = tree.nodes.get(node_index);
            let mut t_a = 0.0f32;
            let a_intersected = Tree::intersects_ray(node.a.min, node.a.max, tree_ray, &mut t_a);
            let mut t_b = 0.0f32;
            let b_intersected = Tree::intersects_ray(node.b.min, node.b.max, tree_ray, &mut t_b);

            if a_intersected {
                if b_intersected {
                    // Visit the earlier AABB intersection first.
                    if stack_end == stack.len() {
                        if stack.len() == TRAVERSAL_STACK_CAPACITY as i32 {
                            // First allocation is on the stack.
                            let mut new_stack: Buffer<i32> =
                                pool.take_at_least(TRAVERSAL_STACK_CAPACITY as i32 * 2);
                            stack.copy_to(0, &mut new_stack, 0, TRAVERSAL_STACK_CAPACITY as i32);
                            stack = new_stack;
                        } else {
                            pool.resize(&mut stack, stack_end * 2, stack_end);
                        }
                    }
                    if t_a < t_b {
                        node_index = node.a.index;
                        *stack.get_mut(stack_end) = node.b.index;
                        stack_end += 1;
                    } else {
                        node_index = node.b.index;
                        *stack.get_mut(stack_end) = node.a.index;
                        stack_end += 1;
                    }
                } else {
                    node_index = node.a.index;
                }
            } else if b_intersected {
                node_index = node.b.index;
            } else {
                // No intersection. Pull from stack.
                if stack_end == 0 {
                    break;
                }
                stack_end -= 1;
                node_index = *stack.get(stack_end);
            }
        }
    }
    if stack.len() > TRAVERSAL_STACK_CAPACITY as i32 {
        // We rented a larger stack at some point. Return it.
        pool.return_buffer(&mut stack);
    }
}

/// Reusable structure for testing large numbers of rays against trees.
///
/// If you want a deeper explanation about this implementation, check out the Dynamic Ray
/// Stream Traversal paper by Barringer and Akenine-Moller. It's basically the same.
pub struct RayBatcher {
    stack_pointer_a0: i32,
    stack_pointer_b: i32,
    stack_pointer_a1: i32,
    ray_indices_a0: Buffer<u16>,
    ray_indices_b: Buffer<u16>,
    ray_indices_a1: Buffer<u16>,
    stack_pointer: i32,
    stack: Buffer<StackEntry>,
    pool: *mut BufferPool,
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
    /// * `ray_capacity` - Maximum number of rays to execute in each traversal. This should
    ///   typically be chosen as the highest value which avoids spilling data out of L2 cache.
    /// * `tree_depth_for_preallocation` - Tree depth to preallocate ray stack space for. If a
    ///   traversal finds nodes deeper than this, a dynamic resize will be triggered.
    pub fn new(pool: *mut BufferPool, ray_capacity: i32, tree_depth_for_preallocation: i32) -> Self {
        let mut batcher = Self {
            stack_pointer_a0: 0,
            stack_pointer_b: 0,
            stack_pointer_a1: 0,
            ray_indices_a0: Buffer::default(),
            ray_indices_b: Buffer::default(),
            ray_indices_a1: Buffer::default(),
            stack_pointer: 0,
            stack: Buffer::default(),
            pool,
            batch_ray_count: 0,
            batch_rays: Buffer::default(),
            batch_original_rays: Buffer::default(),
            ray_capacity: 0,
            preallocated_tree_depth: 0,
            fallback_stack: Buffer::default(),
        };
        let pool_ref = unsafe { &mut *pool };
        batcher.batch_rays = pool_ref.take_at_least(ray_capacity);
        batcher.batch_original_rays = pool_ref.take_at_least(ray_capacity);
        debug_assert!(
            ray_capacity <= u16::MAX as i32,
            "The number of rays per traversal must be less than {}.",
            u16::MAX
        );

        // Note that this assumes the tree has a fixed maximum depth. Not a great idea in the long term.
        batcher.fallback_stack = pool_ref.take_at_least(TRAVERSAL_STACK_CAPACITY as i32);
        batcher.resize_ray_stacks(ray_capacity, tree_depth_for_preallocation);

        batcher.stack_pointer = 0;
        batcher.stack_pointer_a0 = 0;
        batcher.stack_pointer_b = 0;
        batcher.stack_pointer_a1 = 0;
        batcher
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

    /// Disposes all the resources backing the ray batcher.
    pub fn dispose(&mut self) {
        unsafe {
            let pool = &mut *self.pool;
            pool.return_buffer(&mut self.ray_indices_a0);
            pool.return_buffer(&mut self.ray_indices_b);
            pool.return_buffer(&mut self.ray_indices_a1);
            pool.return_buffer(&mut self.stack);
            // Easier to catch bugs if the references get cleared.
            pool.return_buffer(&mut self.fallback_stack);
            pool.return_buffer(&mut self.batch_original_rays);
            pool.return_buffer(&mut self.batch_rays);
        }
        self.stack_pointer_a0 = 0;
        self.stack_pointer_b = 0;
        self.stack_pointer_a1 = 0;
        self.stack_pointer = 0;
        self.pool = std::ptr::null_mut();
        self.batch_ray_count = 0;
        self.ray_capacity = 0;
        self.preallocated_tree_depth = 0;
    }

    fn resize_ray_stacks(&mut self, ray_capacity: i32, tree_depth_for_preallocation: i32) {
        self.ray_capacity = ray_capacity;
        self.preallocated_tree_depth = tree_depth_for_preallocation;
        // The number of ray pointers on the stack is limited in the worst case to all rays per level of the tree.
        let preallocated_ray_pointer_count = ray_capacity * tree_depth_for_preallocation;
        let pool = unsafe { &mut *self.pool };
        pool.resize_to_at_least(
            &mut self.ray_indices_a0,
            preallocated_ray_pointer_count,
            self.stack_pointer_a0,
        );
        pool.resize_to_at_least(
            &mut self.ray_indices_b,
            preallocated_ray_pointer_count,
            self.stack_pointer_b,
        );
        pool.resize_to_at_least(
            &mut self.ray_indices_a1,
            preallocated_ray_pointer_count,
            self.stack_pointer_a1,
        );
        // The number of stack entries is limited by the number of node entries (tree node count * 3) and the
        // number of ray entries. (Can't have more entries on the stack than total ray pointers, after all.)
        pool.resize_to_at_least(
            &mut self.stack,
            preallocated_ray_pointer_count.min(3 << tree_depth_for_preallocation.min(16)),
            self.stack_pointer,
        );
    }

    unsafe fn test_node<TRaySource: TreeRaySourceTrait>(
        &mut self,
        node: &Node,
        depth: u8,
        ray_source: &TRaySource,
    ) {
        let a0_start = self.stack_pointer_a0;
        let b_start = self.stack_pointer_b;
        let a1_start = self.stack_pointer_a1;
        let wide_node = broadcast_node(node);
        let mut ray_bundle = TreeRayWide::default();
        debug_assert!(
            depth < 255,
            "We represent the depth as a byte under the assumption that there won't be any \
             absurdly degenerate tree with extreme depth."
        );
        let new_depth = depth + 1;

        let vector_count = Vector::<f32>::LEN as i32;
        let mut bundle_start_index = 0;
        while bundle_start_index < ray_source.ray_count() {
            let mut count = ray_source.ray_count() - bundle_start_index;
            if count > vector_count {
                count = vector_count;
            }

            for inner_index in 0..count {
                let ray_index = ray_source.get(bundle_start_index + inner_index);
                TreeRayWide::gather_into_slot(
                    self.batch_rays.get(ray_index),
                    &mut ray_bundle,
                    inner_index as usize,
                );
            }
            let (t_a, a_intersected) = intersect(&ray_bundle, &wide_node.min_a, &wide_node.max_a);
            let (t_b, b_intersected) = intersect(&ray_bundle, &wide_node.min_b, &wide_node.max_b);
            // There are three potential stack regions that a single ray can go based on when and if each node
            // child is intersected:
            // ray_indices_a0: A first or A only
            // ray_indices_b:  B is intersected at all
            // ray_indices_a1: A after B
            let a_first = t_a.simd_le(t_b).to_simd();
            for inner_index in 0..count {
                let ray_pointer_index = ray_source.get(bundle_start_index + inner_index) as u16;
                let slot = inner_index as usize;
                let b_slot_intersected = b_intersected[slot] < 0;
                if a_intersected[slot] < 0 {
                    if b_slot_intersected {
                        if a_first[slot] < 0 {
                            *self.ray_indices_a0.get_mut(self.stack_pointer_a0) = ray_pointer_index;
                            self.stack_pointer_a0 += 1;
                            *self.ray_indices_b.get_mut(self.stack_pointer_b) = ray_pointer_index;
                            self.stack_pointer_b += 1;
                        } else {
                            *self.ray_indices_b.get_mut(self.stack_pointer_b) = ray_pointer_index;
                            self.stack_pointer_b += 1;
                            *self.ray_indices_a1.get_mut(self.stack_pointer_a1) = ray_pointer_index;
                            self.stack_pointer_a1 += 1;
                        }
                    } else {
                        *self.ray_indices_a0.get_mut(self.stack_pointer_a0) = ray_pointer_index;
                        self.stack_pointer_a0 += 1;
                    }
                } else if b_slot_intersected {
                    *self.ray_indices_b.get_mut(self.stack_pointer_b) = ray_pointer_index;
                    self.stack_pointer_b += 1;
                }
            }
            bundle_start_index += vector_count;
        }

        let a1_count = self.stack_pointer_a1 - a1_start;
        if a1_count > 0 {
            let idx = self.stack_pointer;
            self.stack_pointer += 1;
            let entry = self.stack.get_mut(idx);
            entry.node_index = node.a.index;
            entry.ray_count = a1_count as u16;
            entry.ray_stack = 2;
            entry.depth = new_depth;
        }
        let b_count = self.stack_pointer_b - b_start;
        if b_count > 0 {
            let idx = self.stack_pointer;
            self.stack_pointer += 1;
            let entry = self.stack.get_mut(idx);
            entry.node_index = node.b.index;
            entry.ray_count = b_count as u16;
            entry.ray_stack = 1;
            entry.depth = new_depth;
        }
        let a0_count = self.stack_pointer_a0 - a0_start;
        if a0_count > 0 {
            let idx = self.stack_pointer;
            self.stack_pointer += 1;
            let entry = self.stack.get_mut(idx);
            entry.node_index = node.a.index;
            entry.ray_count = a0_count as u16;
            entry.ray_stack = 0;
            entry.depth = new_depth;
        }
    }

    /// Tests any batched rays against the given tree.
    ///
    /// # Arguments
    /// * `tree` - Tree to test the accumulated rays against.
    pub unsafe fn test_rays<TLeafTester: IBatchedRayLeafTester>(
        &mut self,
        tree: &Tree,
        leaf_tester: &mut TLeafTester,
    ) {
        debug_assert!(
            self.stack_pointer_a0 == 0
                && self.stack_pointer_b == 0
                && self.stack_pointer_a1 == 0
                && self.stack_pointer == 0,
            "At the beginning of the traversal, there should exist no entries on the traversal stack."
        );

        if tree.leaf_count == 0 {
            return;
        }

        // The traversal begins by assuming an implicit stack entry for the root node containing all ray
        // pointers from 0 to rayCount-1.
        if tree.leaf_count >= 2 {
            let ray_source = RootRaySource::new(self.batch_ray_count);
            let node = tree.nodes.get(0);
            self.test_node(node, 0, &ray_source);
        } else {
            debug_assert!(tree.leaf_count == 1);
            // Only one child in the tree. Handle it as a special case.
            let a0_start = self.stack_pointer_a0;
            let node = tree.nodes.get(0);
            let node_wide = broadcast_node(node);
            let vector_count = Vector::<f32>::LEN as i32;
            let mut ray_bundle = TreeRayWide::default();
            let mut bundle_start_index = 0;
            while bundle_start_index < self.batch_ray_count {
                let mut count = self.batch_ray_count - bundle_start_index;
                if count > vector_count {
                    count = vector_count;
                }
                for inner_index in 0..count {
                    let idx = bundle_start_index + inner_index;
                    TreeRayWide::gather_into_slot(
                        self.batch_rays.get(idx),
                        &mut ray_bundle,
                        inner_index as usize,
                    );
                }
                let (_t_a, a_intersected) =
                    intersect(&ray_bundle, &node_wide.min_a, &node_wide.max_a);
                for inner_index in 0..count {
                    // TODO: Examine codegen. Bounds checks MIGHT be elided, but if they aren't, we can work around them.
                    if a_intersected[inner_index as usize] < 0 {
                        *self.ray_indices_a0.get_mut(self.stack_pointer_a0) =
                            (bundle_start_index + inner_index) as u16;
                        self.stack_pointer_a0 += 1;
                    }
                }
                bundle_start_index += vector_count;
            }
            let a0_count = self.stack_pointer_a0 - a0_start;
            if a0_count > 0 {
                let idx = self.stack_pointer;
                self.stack_pointer += 1;
                let entry = self.stack.get_mut(idx);
                entry.node_index = node.a.index;
                entry.ray_count = a0_count as u16;
                entry.ray_stack = 0;
                entry.depth = 1;
            }
        }
        while self.stack_pointer > 0 {
            // Move the ray stack pointer back to the start of the popped region. The test will read from the
            // region and potentially push additional elements. The pushes are guaranteed to never go beyond the
            // region that was popped - a node cannot be traversed by more rays than its parent. Further, the reads
            // are guaranteed to complete before being overwritten. Each bundle is popped and processed before any
            // dangerous pushes can occur.
            if self.stack.get(self.stack_pointer - 1).depth as i32 + 1 > self.preallocated_tree_depth {
                // We were not aggressive enough in preallocating for the ray stacks, apparently. Resize them aggressively.
                // This must happen before popping: the resize copies the stacks up to their current pointers and frees the old buffers.
                self.resize_ray_stacks(self.ray_capacity, (self.preallocated_tree_depth * 2).max(1));
            }
            self.stack_pointer -= 1;
            let entry = *self.stack.get(self.stack_pointer);
            let ray_stack_start_ptr: *mut u16 = match entry.ray_stack {
                0 => {
                    self.stack_pointer_a0 -= entry.ray_count as i32;
                    self.ray_indices_a0.get_mut_ptr(self.stack_pointer_a0)
                }
                1 => {
                    self.stack_pointer_b -= entry.ray_count as i32;
                    self.ray_indices_b.get_mut_ptr(self.stack_pointer_b)
                }
                _ => {
                    self.stack_pointer_a1 -= entry.ray_count as i32;
                    self.ray_indices_a1.get_mut_ptr(self.stack_pointer_a1)
                }
            };
            if entry.ray_count >= 3 {
                // There are enough rays that we can justify continuing this vectorized approach.
                if entry.node_index >= 0 {
                    let ray_stack_source =
                        TreeRaySource::new(ray_stack_start_ptr, entry.ray_count as i32);
                    let node = tree.nodes.get(entry.node_index);
                    self.test_node(node, entry.depth, &ray_stack_source);
                } else {
                    // This is a leaf node.
                    let mut ray_stack_source = RaySource {
                        tree_rays: self.batch_rays.as_mut_ptr(),
                        rays: self.batch_original_rays.as_mut_ptr(),
                        ray_pointers: ray_stack_start_ptr,
                        ray_count: entry.ray_count as i32,
                    };
                    leaf_tester.ray_test(
                        Tree::encode(entry.node_index),
                        &mut ray_stack_source,
                        &mut *self.pool,
                    );
                }
            } else {
                // Not enough rays remain to justify group tests. Fall back to a per-ray traversal.
                for i in 0..entry.ray_count as i32 {
                    let ray_index = *ray_stack_start_ptr.offset(i as isize) as i32;
                    let tree_ray_ptr = self.batch_rays.get_mut_ptr(ray_index);
                    let ray_data_ptr = self.batch_original_rays.get_mut_ptr(ray_index);
                    tree_ray_cast_fallback(
                        tree,
                        entry.node_index,
                        tree_ray_ptr,
                        ray_data_ptr,
                        self.fallback_stack,
                        &mut *self.pool,
                        leaf_tester,
                    );
                }
            }
        }
        debug_assert!(
            self.stack_pointer_a0 == 0
                && self.stack_pointer_b == 0
                && self.stack_pointer_a1 == 0
                && self.stack_pointer == 0,
            "By the end of the traversal, there should exist no entries on the traversal stack."
        );
    }

    /// Adds a ray to the batcher.
    ///
    /// # Arguments
    /// * `origin` - Origin of the ray to test against the tree.
    /// * `direction` - Direction of the ray to test against the tree.
    /// * `maximum_t` - Maximum distance that the ray will travel in units of the ray's length.
    /// * `id` - Identifier value for the ray. Leaf tests will have access to the id.
    ///
    /// Returns true if the batcher has reached maximum ray capacity and needs to be reset in
    /// order to continue adding rays.
    #[inline(always)]
    pub fn add(&mut self, origin: Vec3, direction: Vec3, maximum_t: f32, id: i32) -> bool {
        debug_assert!(
            self.batch_ray_count >= 0 && self.batch_ray_count < self.ray_capacity,
            "The accumulated rays must not exceed the maximum count per traversal; make sure \
             reset_rays was called following a call to add that returned true."
        );
        let ray_index = self.batch_ray_count;
        self.batch_ray_count += 1;
        TreeRay::create_from_ray(
            origin,
            direction,
            maximum_t,
            id,
            self.batch_original_rays.get_mut(ray_index),
            self.batch_rays.get_mut(ray_index),
        );
        self.batch_ray_count == self.ray_capacity
    }

    /// Resets the accumulated ray count to zero.
    #[inline(always)]
    pub fn reset_rays(&mut self) {
        self.batch_ray_count = 0;
    }
}
