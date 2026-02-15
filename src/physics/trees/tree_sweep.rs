// Translated from BepuPhysics/Trees/Tree_Sweep.cs

use super::ray_batcher::TreeRay;
use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

/// Trait for testing sweep intersections against leaves in the tree.
pub trait ISweepLeafTester {
    fn test_leaf(&mut self, leaf_index: i32, maximum_t: &mut f32);
}

impl Tree {
    unsafe fn sweep_node<TLeafTester: ISweepLeafTester>(
        &self,
        mut node_index: i32,
        expansion: Vec3,
        _origin: Vec3,
        _direction: Vec3,
        tree_ray: *mut TreeRay,
        mut stack: Buffer<i32>,
        pool: &mut BufferPool,
        leaf_tester: &mut TLeafTester,
    ) {
        debug_assert!(
            (node_index >= 0 && node_index < self.node_count)
                || (Self::encode(node_index) >= 0 && Self::encode(node_index) < self.leaf_count)
        );
        debug_assert!(
            self.leaf_count >= 2,
            "This implementation assumes all nodes are filled."
        );

        let mut stack_end: i32 = 0;
        loop {
            if node_index < 0 {
                let leaf_index = Self::encode(node_index);
                leaf_tester.test_leaf(leaf_index, &mut (*tree_ray).maximum_t);
                if stack_end == 0 {
                    break;
                }
                stack_end -= 1;
                node_index = *stack.get(stack_end);
            } else {
                let node = self.nodes.get(node_index);
                let min_a = node.a.min - expansion;
                let max_a = node.a.max + expansion;
                let mut t_a = 0.0f32;
                let a_intersected = Self::intersects_ray(min_a, max_a, tree_ray, &mut t_a);
                let min_b = node.b.min - expansion;
                let max_b = node.b.max + expansion;
                let mut t_b = 0.0f32;
                let b_intersected = Self::intersects_ray(min_b, max_b, tree_ray, &mut t_b);

                if a_intersected {
                    if b_intersected {
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

    pub(crate) unsafe fn sweep_internal<TLeafTester: ISweepLeafTester>(
        &self,
        expansion: Vec3,
        origin: Vec3,
        direction: Vec3,
        tree_ray: *mut TreeRay,
        pool: &mut BufferPool,
        sweep_tester: &mut TLeafTester,
    ) {
        if self.leaf_count == 0 {
            return;
        }

        if self.leaf_count == 1 {
            let min = self.nodes.get(0).a.min - expansion;
            let max = self.nodes.get(0).a.max + expansion;
            let mut t_a = 0.0f32;
            if Self::intersects_ray(min, max, tree_ray, &mut t_a) {
                sweep_tester.test_leaf(0, &mut (*tree_ray).maximum_t);
            }
        } else {
            let mut stack_memory = [0i32; TRAVERSAL_STACK_CAPACITY];
            let stack = Buffer::new(
                stack_memory.as_mut_ptr(),
                TRAVERSAL_STACK_CAPACITY as i32,
                -1,
            );
            self.sweep_node(
                0,
                expansion,
                origin,
                direction,
                tree_ray,
                stack,
                pool,
                sweep_tester,
            );
        }
    }

    /// Converts a bounding box to a centroid + half-extent representation.
    #[inline(always)]
    pub fn convert_box_to_centroid_with_extent(
        min: Vec3,
        max: Vec3,
        origin: &mut Vec3,
        expansion: &mut Vec3,
    ) {
        let half_min = 0.5 * min;
        let half_max = 0.5 * max;
        *expansion = half_max - half_min;
        *origin = half_max + half_min;
    }

    /// Performs a sweep test of an axis-aligned bounding box against the tree and invokes the sweep_tester for each intersecting leaf.
    ///
    /// # Arguments
    /// * `min` - The minimum corner of the axis-aligned bounding box to sweep.
    /// * `max` - The maximum corner of the axis-aligned bounding box to sweep.
    /// * `direction` - The direction of the sweep.
    /// * `maximum_t` - The maximum parametric distance along the sweep direction to test.
    /// * `pool` - Buffer pool used for temporary allocations if the tree is pathologically deep; stack memory is used preferentially.
    /// * `sweep_tester` - A reference to the tester that processes the indices of intersecting leaves.
    pub unsafe fn sweep<TLeafTester: ISweepLeafTester>(
        &self,
        min: Vec3,
        max: Vec3,
        direction: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        sweep_tester: &mut TLeafTester,
    ) {
        let mut origin = Vec3::ZERO;
        let mut expansion = Vec3::ZERO;
        Self::convert_box_to_centroid_with_extent(min, max, &mut origin, &mut expansion);
        let mut tree_ray = std::mem::MaybeUninit::<TreeRay>::uninit();
        TreeRay::create_from(
            origin,
            direction,
            maximum_t,
            tree_ray.as_mut_ptr().as_mut().unwrap(),
        );
        self.sweep_internal(
            expansion,
            origin,
            direction,
            tree_ray.as_mut_ptr(),
            pool,
            sweep_tester,
        );
    }

    /// Performs a sweep test of a bounding box against the tree and invokes the sweep_tester for each intersecting leaf.
    #[inline(always)]
    pub unsafe fn sweep_bbox<TLeafTester: ISweepLeafTester>(
        &self,
        bounding_box: &crate::utilities::bounding_box::BoundingBox,
        direction: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        sweep_tester: &mut TLeafTester,
    ) {
        self.sweep(bounding_box.min, bounding_box.max, direction, maximum_t, pool, sweep_tester);
    }
}
