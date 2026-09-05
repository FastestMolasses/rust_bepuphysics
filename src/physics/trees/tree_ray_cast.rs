// Translated from BepuPhysics/Trees/Tree_RayCast.cs

use super::ray_batcher::{RayData, TreeRay};
use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

/// Trait for testing ray intersections against leaves in the tree.
pub trait IRayLeafTester {
    /// Tests a leaf for intersection with a ray.
    unsafe fn test_leaf(
        &mut self,
        leaf_index: i32,
        ray_data: *mut RayData,
        maximum_t: *mut f32,
        pool: &mut BufferPool,
    );
}

impl Tree {
    /// Tests a ray against an AABB defined by min/max.
    /// Returns true if the ray intersects, and writes the entry t into `t`.
    #[inline(always)]
    pub unsafe fn intersects_ray(min: Vec3, max: Vec3, ray: *const TreeRay, t: &mut f32) -> bool {
        let t0 = min * (*ray).inverse_direction - (*ray).origin_over_direction;
        let t1 = max * (*ray).inverse_direction - (*ray).origin_over_direction;
        let t_exit = t0.max(t1);
        let t_entry = t0.min(t1);
        // Note the use of broadcast and SIMD min/max here. This is much faster than using branches to compute
        // minimum elements, since the branches get mispredicted extremely frequently. Also note 4-wide
        // operations; they're actually faster than using narrower vectors due to some unnecessary codegen.
        #[cfg(target_arch = "x86_64")]
        {
            use std::arch::x86_64::*;
            let earliest_exit_v = _mm_min_ps(
                _mm_min_ps(_mm_set1_ps((*ray).maximum_t), _mm_set1_ps(t_exit.x)),
                _mm_min_ps(_mm_set1_ps(t_exit.y), _mm_set1_ps(t_exit.z)),
            );
            let earliest_exit = _mm_cvtss_f32(earliest_exit_v);
            let t_v = _mm_max_ps(
                _mm_max_ps(_mm_set1_ps(t_entry.x), _mm_setzero_ps()),
                _mm_max_ps(_mm_set1_ps(t_entry.y), _mm_set1_ps(t_entry.z)),
            );
            *t = _mm_cvtss_f32(t_v);
            *t <= earliest_exit
        }
        #[cfg(target_arch = "aarch64")]
        {
            use std::arch::aarch64::*;
            let earliest_exit_v = vminq_f32(
                vminq_f32(vdupq_n_f32((*ray).maximum_t), vdupq_n_f32(t_exit.x)),
                vminq_f32(vdupq_n_f32(t_exit.y), vdupq_n_f32(t_exit.z)),
            );
            let earliest_exit = vgetq_lane_f32(earliest_exit_v, 0);
            let t_v = vmaxq_f32(
                vmaxq_f32(vdupq_n_f32(t_entry.x), vdupq_n_f32(0.0)),
                vmaxq_f32(vdupq_n_f32(t_entry.y), vdupq_n_f32(t_entry.z)),
            );
            *t = vgetq_lane_f32(t_v, 0);
            *t <= earliest_exit
        }
        #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
        {
            use std::simd::num::SimdFloat;
            use std::simd::Simd;
            let earliest_exit_v = Simd::<f32, 4>::splat((*ray).maximum_t)
                .simd_min(Simd::<f32, 4>::splat(t_exit.x))
                .simd_min(Simd::<f32, 4>::splat(t_exit.y).simd_min(Simd::<f32, 4>::splat(t_exit.z)));
            let earliest_exit = earliest_exit_v[0];
            let t_v = Simd::<f32, 4>::splat(t_entry.x)
                .simd_max(Simd::<f32, 4>::splat(0.0))
                .simd_max(Simd::<f32, 4>::splat(t_entry.y).simd_max(Simd::<f32, 4>::splat(t_entry.z)));
            *t = t_v[0];
            *t <= earliest_exit
        }
    }

    unsafe fn ray_cast_node<TLeafTester: IRayLeafTester>(
        &self,
        mut node_index: i32,
        tree_ray: *mut TreeRay,
        ray_data: *mut RayData,
        mut stack: Buffer<i32>,
        pool: &mut BufferPool,
        leaf_tester: &mut TLeafTester,
    ) {
        debug_assert!(
            (node_index >= 0 && node_index < self.node_count)
                || (Self::encode(node_index) >= 0 && Self::encode(node_index) < self.leaf_count)
        );
        debug_assert!(
            self.leaf_count >= 2 || node_index < 0,
            "The node traversal assumes all nodes are filled; a single-leaf tree can only be entered through its encoded leaf index."
        );

        let mut stack_end: i32 = 0;
        loop {
            if node_index < 0 {
                // This is actually a leaf node.
                let leaf_index = Self::encode(node_index);
                leaf_tester.test_leaf(leaf_index, ray_data, &mut (*tree_ray).maximum_t, pool);
                // Leaves have no children; pull from the stack.
                if stack_end == 0 {
                    break;
                }
                stack_end -= 1;
                node_index = *stack.get(stack_end);
            } else {
                let node = self.nodes.get(node_index);
                let mut t_a = 0.0f32;
                let a_intersected =
                    Self::intersects_ray(node.a.min, node.a.max, tree_ray, &mut t_a);
                let mut t_b = 0.0f32;
                let b_intersected =
                    Self::intersects_ray(node.b.min, node.b.max, tree_ray, &mut t_b);

                if a_intersected {
                    if b_intersected {
                        // Visit the earlier AABB intersection first.
                        if stack_end == stack.len() {
                            if stack.len() == TRAVERSAL_STACK_CAPACITY as i32 {
                                // First allocation is on the stack.
                                let mut new_stack: Buffer<i32> =
                                    pool.take_at_least(TRAVERSAL_STACK_CAPACITY as i32 * 2);
                                stack.copy_to(
                                    0,
                                    &mut new_stack,
                                    0,
                                    TRAVERSAL_STACK_CAPACITY as i32,
                                );
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

    pub(crate) unsafe fn ray_cast_internal<TLeafTester: IRayLeafTester>(
        &self,
        tree_ray: *mut TreeRay,
        ray_data: *mut RayData,
        pool: &mut BufferPool,
        leaf_tester: &mut TLeafTester,
    ) {
        if self.leaf_count == 0 {
            return;
        }

        if self.leaf_count == 1 {
            // If the first node isn't filled, we have to use a special case.
            let mut t_a = 0.0f32;
            if Self::intersects_ray(
                self.nodes.get(0).a.min,
                self.nodes.get(0).a.max,
                tree_ray,
                &mut t_a,
            ) {
                leaf_tester.test_leaf(0, ray_data, &mut (*tree_ray).maximum_t, pool);
            }
        } else {
            let mut stack_memory = std::mem::MaybeUninit::<[i32; TRAVERSAL_STACK_CAPACITY]>::uninit();
            let stack = Buffer::new(
                stack_memory.as_mut_ptr() as *mut i32,
                TRAVERSAL_STACK_CAPACITY as i32,
                -1,
            );
            self.ray_cast_node(0, tree_ray, ray_data, stack, pool, leaf_tester);
        }
    }

    /// Tests a ray against the tree and invokes the leaf tester for each leaf node that the ray intersects.
    ///
    /// # Arguments
    /// * `origin` - The origin point of the ray.
    /// * `direction` - The direction of the ray.
    /// * `maximum_t` - The maximum parametric distance along the ray to test. May be modified by the leaf tester.
    /// * `pool` - Buffer pool used for temporary allocations if the tree is pathologically deep; stack memory is used preferentially.
    /// * `leaf_tester` - A reference to the tester that processes the indices of intersecting leaves.
    /// * `id` - An optional identifier for the ray.
    pub unsafe fn ray_cast<TLeafTester: IRayLeafTester>(
        &self,
        origin: Vec3,
        direction: Vec3,
        maximum_t: &mut f32,
        pool: &mut BufferPool,
        leaf_tester: &mut TLeafTester,
        id: i32,
    ) {
        let mut ray_data = std::mem::MaybeUninit::<RayData>::uninit();
        let mut tree_ray = std::mem::MaybeUninit::<TreeRay>::uninit();
        TreeRay::create_from_ray(
            origin,
            direction,
            *maximum_t,
            id,
            &mut *ray_data.as_mut_ptr(),
            &mut *tree_ray.as_mut_ptr(),
        );
        self.ray_cast_internal(
            tree_ray.as_mut_ptr(),
            ray_data.as_mut_ptr(),
            pool,
            leaf_tester,
        );
        // The maximumT could have been mutated by the leaf tester. Propagate that change.
        *maximum_t = (*tree_ray.as_ptr()).maximum_t;
    }
}
