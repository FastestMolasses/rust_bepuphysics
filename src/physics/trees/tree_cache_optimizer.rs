// Translated from BepuPhysics/Trees/Tree_CacheOptimizer.cs

use super::leaf::Leaf;
use super::node::Node;
use super::tree::Tree;

impl Tree {
    fn swap_nodes(&mut self, index_a: i32, index_b: i32) {
        unsafe {
            let nodes_ptr = self.nodes.as_ptr() as *mut Node;
            let a = &mut *nodes_ptr.add(index_a as usize);
            let b = &mut *nodes_ptr.add(index_b as usize);

            std::mem::swap(a, b);

            let meta_ptr = self.metanodes.as_ptr() as *mut super::node::Metanode;
            let meta_a = &mut *meta_ptr.add(index_a as usize);
            let meta_b = &mut *meta_ptr.add(index_b as usize);
            std::mem::swap(meta_a, meta_b);

            if meta_a.parent == index_a {
                // The original B's parent was A. That parent has moved.
                meta_a.parent = index_b;
            } else if meta_b.parent == index_b {
                // The original A's parent was B. That parent has moved.
                meta_b.parent = index_a;
            }

            let meta_a_copy = *meta_a;
            let meta_b_copy = *meta_b;

            Self::node_child_mut(
                &mut *nodes_ptr.add(meta_a_copy.parent as usize),
                meta_a_copy.index_in_parent,
            )
            .index = index_a;
            Self::node_child_mut(
                &mut *nodes_ptr.add(meta_b_copy.parent as usize),
                meta_b_copy.index_in_parent,
            )
            .index = index_b;

            // Update the parent pointers of the children of node A.
            let a = &*nodes_ptr.add(index_a as usize);
            for i in 0..2i32 {
                let child = Self::node_child(a, i);
                if child.index >= 0 {
                    self.metanodes.get_mut(child.index).parent = index_a;
                } else {
                    let leaf_index = Self::encode(child.index);
                    *self.leaves.get_mut(leaf_index) = Leaf::new(index_a, i);
                }
            }

            // Update the parent pointers of the children of node B.
            let b = &*nodes_ptr.add(index_b as usize);
            for i in 0..2i32 {
                let child = Self::node_child(b, i);
                if child.index >= 0 {
                    self.metanodes.get_mut(child.index).parent = index_b;
                } else {
                    let leaf_index = Self::encode(child.index);
                    *self.leaves.get_mut(leaf_index) = Leaf::new(index_b, i);
                }
            }
        }
    }

    /// Computes the index where the given node would be located if the tree were in
    /// depth first traversal order.
    pub fn compute_cache_optimal_location(&self, node_index: i32) -> i32 {
        // Walk back up to the root. Count all nodes to the left of the current node.
        let mut left_node_count = 0;
        let mut chained_node_index = node_index;
        loop {
            let metanode = self.metanodes.get(chained_node_index);
            let parent = metanode.parent;
            if parent < 0 {
                break;
            }
            chained_node_index = parent;
            left_node_count += 1;
            if metanode.index_in_parent == 1 {
                left_node_count += self.nodes.get(parent).a.leaf_count - 1;
            }
        }
        left_node_count
    }

    fn cache_optimize_recursive(&mut self, node_index: i32, next_index: &mut i32) {
        let node = *self.nodes.get(node_index);
        for i in 0..2i32 {
            let child = unsafe { Self::node_child(&node, i) };
            if child.index >= 0 {
                debug_assert!(
                    *next_index >= 0 && *next_index < self.node_count,
                    "Swap target should be within the node set."
                );
                if child.index != *next_index {
                    self.swap_nodes(child.index, *next_index);
                }
                debug_assert!(child.index != *next_index);
                *next_index += 1;
                self.cache_optimize_recursive(child.index, next_index);
            }
        }
    }

    /// Begins a cache optimization at the given node and proceeds all the way to the bottom of the tree.
    /// Requires that the targeted node is already at the global optimum position.
    pub fn cache_optimize(&mut self, node_index: i32) {
        if self.leaf_count <= 2 {
            return;
        }
        let mut target_index = node_index + 1;
        self.cache_optimize_recursive(node_index, &mut target_index);
    }

    fn cache_optimized_limited_subtree_internal(
        &mut self,
        source_node_index: i32,
        target_node_index: i32,
        mut node_optimization_count: i32,
    ) {
        if source_node_index != target_node_index {
            self.swap_nodes(target_node_index, source_node_index);
        }
        node_optimization_count -= 1;
        if node_optimization_count == 0 {
            return;
        }
        let node = *self.nodes.get(target_node_index);
        let lower_node_count = node.a.leaf_count.min(node.b.leaf_count) - 1;
        let lower_target_node_count = lower_node_count.min((node_optimization_count + 1) / 2);
        let higher_node_count = node_optimization_count - lower_target_node_count;
        let a_is_smaller = node.a.leaf_count < node.b.leaf_count;
        let node_optimization_count_a = if a_is_smaller {
            lower_target_node_count
        } else {
            higher_node_count
        };
        let node_optimization_count_b = if a_is_smaller {
            higher_node_count
        } else {
            lower_target_node_count
        };
        if node_optimization_count_a > 0 {
            self.cache_optimized_limited_subtree_internal(
                node.a.index,
                target_node_index + 1,
                node_optimization_count_a,
            );
        }
        if node_optimization_count_b > 0 {
            self.cache_optimized_limited_subtree_internal(
                node.b.index,
                target_node_index + node.a.leaf_count,
                node_optimization_count_b,
            );
        }
    }

    /// Starts a cache optimization process at the target node index and the
    /// `node_optimization_count` closest nodes in the tree.
    pub fn cache_optimize_limited_subtree(
        &mut self,
        node_index: i32,
        node_optimization_count: i32,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let target_node_index = self.compute_cache_optimal_location(node_index);
        let original_node = *self.nodes.get(node_index);
        let effective_count =
            (original_node.a.leaf_count + original_node.b.leaf_count - 1).min(node_optimization_count);
        self.cache_optimized_limited_subtree_internal(node_index, target_node_index, effective_count);
    }

    /// Puts all nodes starting from the given node index into depth first traversal order.
    /// Returns the number of nodes optimized.
    pub fn cache_optimize_region(
        &mut self,
        starting_node_index: i32,
        target_count: i32,
    ) -> i32 {
        if self.leaf_count <= 2 {
            return 0;
        }
        let target_node_index = self.compute_cache_optimal_location(starting_node_index);
        if starting_node_index != target_node_index {
            self.swap_nodes(target_node_index, starting_node_index);
        }

        let start_node = *self.nodes.get(target_node_index);
        // Note minus 2: visiting the parent of the last node is sufficient to put the last node into position.
        let node_count =
            (start_node.a.leaf_count + start_node.b.leaf_count - 2).min(target_count);
        for i in 0..node_count {
            let parent_index = target_node_index + i;
            let node = *self.nodes.get(parent_index);
            let target_location_a = parent_index + 1;
            let target_location_b = parent_index + node.a.leaf_count;
            if node.a.index >= 0 && node.a.index != target_location_a {
                self.swap_nodes(node.a.index, target_location_a);
            }
            if node.b.index >= 0 && node.b.index != target_location_b {
                self.swap_nodes(node.b.index, target_location_b);
            }
        }
        node_count
    }
}
