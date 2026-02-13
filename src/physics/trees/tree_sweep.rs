// Translated from BepuPhysics/Trees/Tree_Sweep.cs

use super::ray_batcher::TreeRay;
use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
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
        stack: *mut i32,
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
                    return;
                }
                stack_end -= 1;
                node_index = *stack.add(stack_end as usize);
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
                        debug_assert!(
                            (stack_end as usize) < TRAVERSAL_STACK_CAPACITY - 1,
                            "Fixed size stack overflow."
                        );
                        if t_a < t_b {
                            node_index = node.a.index;
                            *stack.add(stack_end as usize) = node.b.index;
                            stack_end += 1;
                        } else {
                            node_index = node.b.index;
                            *stack.add(stack_end as usize) = node.a.index;
                            stack_end += 1;
                        }
                    } else {
                        node_index = node.a.index;
                    }
                } else if b_intersected {
                    node_index = node.b.index;
                } else {
                    if stack_end == 0 {
                        return;
                    }
                    stack_end -= 1;
                    node_index = *stack.add(stack_end as usize);
                }
            }
        }
    }

    pub(crate) unsafe fn sweep_internal<TLeafTester: ISweepLeafTester>(
        &self,
        expansion: Vec3,
        origin: Vec3,
        direction: Vec3,
        tree_ray: *mut TreeRay,
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
            let mut stack = [0i32; TRAVERSAL_STACK_CAPACITY];
            self.sweep_node(
                0,
                expansion,
                origin,
                direction,
                tree_ray,
                stack.as_mut_ptr(),
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

    /// Performs a swept bounding box test against the tree.
    pub unsafe fn sweep<TLeafTester: ISweepLeafTester>(
        &self,
        min: Vec3,
        max: Vec3,
        direction: Vec3,
        maximum_t: f32,
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
            sweep_tester,
        );
    }
}
