// Translated from BepuPhysics/Trees/Tree_Diagnostics.cs

use super::node::Node;
use super::tree::Tree;
use glam::Vec3;

impl Tree {
    /// Measures the SAH cost metric of the tree. Lower is better.
    pub fn measure_cost_metric(&self) -> f32 {
        let root_node = self.nodes.get(0);

        let mut merged_min = Vec3::splat(f32::MAX);
        let mut merged_max = Vec3::splat(f32::MIN);

        let child_count = self.leaf_count.min(2);
        for i in 0..child_count {
            let child = unsafe { Self::node_child(root_node, i) };
            merged_min = merged_min.min(child.min);
            merged_max = merged_max.max(child.max);
        }
        let root_metric = Self::compute_bounds_metric_vecs(&merged_min, &merged_max);

        const _LEAF_COST: f32 = 1.0;
        const _INTERNAL_NODE_COST: f32 = 1.0;

        if self.leaf_count > 2 {
            let mut total_cost = 0.0f32;
            for i in 0..self.node_count {
                let node = self.nodes.get(i);
                for child_index in 0..2i32 {
                    let child = unsafe { Self::node_child(node, child_index) };
                    // Both leaf and internal node cost is 1, so just add the metric.
                    total_cost += Self::compute_bounds_metric_vecs(&child.min, &child.max);
                }
            }
            total_cost / root_metric
        } else {
            0.0
        }
    }

    fn validate_node(
        &self,
        node_index: i32,
        expected_parent_index: i32,
        expected_index_in_parent: i32,
        expected_min: &Vec3,
        expected_max: &Vec3,
    ) -> i32 {
        let _node = self.nodes.get(node_index);
        let metanode = self.metanodes.get(node_index);

        assert_eq!(
            metanode.parent, expected_parent_index,
            "Bad parent index on node {node_index}"
        );
        assert_eq!(
            metanode.index_in_parent, expected_index_in_parent,
            "Bad index in parent on node {node_index}"
        );
        assert_eq!(
            unsafe { metanode.cost_or_flag.refine_flag }, 0,
            "Nonzero refine flag on node {node_index}"
        );

        let mut found_leaf_count = 0i32;
        let bad_min_value = Vec3::splat(f32::MAX);
        let bad_max_value = Vec3::splat(f32::MIN);
        let mut merged_min = bad_min_value;
        let mut merged_max = bad_max_value;
        let child_count = self.leaf_count.min(2);

        for i in 0..child_count {
            let child = unsafe { Self::node_child(self.nodes.get(node_index), i) };
            assert!(
                child.min != bad_min_value && child.max != bad_max_value,
                "Node {node_index} child {i} has a bad bounding box."
            );
            merged_min = merged_min.min(child.min);
            merged_max = merged_max.max(child.max);

            if child.index >= 0 {
                assert!(
                    child.index < self.node_count,
                    "Implied existence of node {} is outside of count {}.",
                    child.index,
                    self.node_count,
                );
                let child_found_leaf_count = self.validate_node(
                    child.index,
                    node_index,
                    i,
                    &child.min,
                    &child.max,
                );
                assert_eq!(
                    child_found_leaf_count, child.leaf_count,
                    "Bad leaf count for child {i} of node {node_index}."
                );
                found_leaf_count += child_found_leaf_count;
            } else {
                found_leaf_count += 1;
                assert_eq!(
                    child.leaf_count, 1,
                    "Bad leaf count on node {node_index} child {i}, it's a leaf but leaf_count is {}.",
                    child.leaf_count
                );
                let leaf_index = Self::encode(child.index);
                assert!(
                    leaf_index >= 0 && leaf_index < self.leaf_count,
                    "Bad node-contained leaf index."
                );
                let leaf = self.leaves.get(leaf_index);
                assert!(
                    leaf.node_index() == node_index && leaf.child_index() == i,
                    "Mismatch between node-held leaf pointer and leaf's pointers."
                );
            }
        }

        if found_leaf_count == 0 && (self.leaf_count > 0 || expected_parent_index >= 0) {
            panic!("Bad leaf count.");
        }

        let metric = Self::compute_bounds_metric_vecs(&merged_min, &merged_max);
        if found_leaf_count > 0 && (metric.is_nan() || metric.is_infinite()) {
            panic!(
                "Bad bounds: {metric} SAH. {merged_min:?}, {merged_max:?}."
            );
        }

        if expected_parent_index >= 0
            && (merged_min != *expected_min || merged_max != *expected_max)
        {
            panic!(
                "{node_index} bounds {merged_min:?}, {merged_max:?}, expected ({expected_min:?}, {expected_max:?})."
            );
        }

        found_leaf_count
    }

    fn validate_leaf_node_indices(&self) {
        for i in 0..self.leaf_count {
            let leaf = self.leaves.get(i);
            assert!(
                leaf.node_index() >= 0,
                "Leaf {i} has negative node index: {}.",
                leaf.node_index()
            );
            assert!(
                leaf.node_index() < self.node_count,
                "Leaf {i} points to a node outside the node set, {} >= {}.",
                leaf.node_index(),
                self.node_count
            );
        }
    }

    fn validate_leaves(&self) {
        self.validate_leaf_node_indices();
        for i in 0..self.leaf_count {
            let leaf = self.leaves.get(i);
            let child =
                unsafe { Self::node_child(self.nodes.get(leaf.node_index()), leaf.child_index()) };
            assert_eq!(
                Self::encode(child.index),
                i,
                "Leaf {i} data does not agree with node about parenthood."
            );
        }
    }

    /// Validates the tree structure, panicking on any inconsistency.
    pub fn validate(&self) {
        assert!(
            self.node_count >= 0,
            "Invalid negative node count of {}",
            self.node_count
        );
        assert!(
            self.node_count <= self.nodes.len(),
            "Invalid node count of {}, larger than nodes array length {}.",
            self.node_count,
            self.nodes.len()
        );
        if self.leaf_count > 0 {
            let root_meta = self.metanodes.get(0);
            assert!(
                root_meta.parent == -1 && root_meta.index_in_parent == -1,
                "Invalid parent pointers on root."
            );
        }
        let valid = (self.node_count == 1 && self.leaf_count < 2)
            || (self.node_count == self.leaf_count - 1 && self.leaf_count >= 2);
        assert!(
            valid,
            "Invalid node count versus leaf count."
        );

        let stand_in_min = Vec3::ZERO;
        let stand_in_max = Vec3::ZERO;
        let found_leaf_count =
            self.validate_node(0, -1, -1, &stand_in_min, &stand_in_max);
        assert_eq!(
            found_leaf_count, self.leaf_count,
            "{found_leaf_count} leaves found in tree, expected {}.",
            self.leaf_count
        );

        self.validate_leaves();
    }

    fn compute_maximum_depth_recursive(&self, node: &Node, current_depth: i32) -> i32 {
        let mut maximum = current_depth;
        let next_depth = current_depth + 1;
        let child_count = self.leaf_count.min(2);
        for i in 0..child_count {
            let child = unsafe { Self::node_child(node, i) };
            if child.index >= 0 {
                let candidate =
                    self.compute_maximum_depth_recursive(self.nodes.get(child.index), next_depth);
                if candidate > maximum {
                    maximum = candidate;
                }
            }
        }
        maximum
    }

    /// Computes the maximum depth of the tree.
    pub fn compute_maximum_depth(&self) -> i32 {
        self.compute_maximum_depth_recursive(self.nodes.get(0), 0)
    }

    fn measure_cache_quality_recursive(
        &self,
        node_index: i32,
        found_nodes: &mut i32,
        node_score: &mut f32,
        scorable_node_count: &mut i32,
    ) {
        let mut correctly_positioned_immediate_children = 0;
        let mut immediate_internal_children = 0;
        let mut expected_child_index = node_index + 1;
        let child_count = self.leaf_count.min(2);

        for i in 0..child_count {
            let child = unsafe { Self::node_child(self.nodes.get(node_index), i) };
            if child.index >= 0 {
                immediate_internal_children += 1;
                if child.index == expected_child_index {
                    correctly_positioned_immediate_children += 1;
                }
                let mut child_found_nodes = 0i32;
                let mut child_node_score = 0.0f32;
                let mut child_scorable_nodes = 0i32;
                self.measure_cache_quality_recursive(
                    child.index,
                    &mut child_found_nodes,
                    &mut child_node_score,
                    &mut child_scorable_nodes,
                );
                *found_nodes += child_found_nodes;
                expected_child_index += child_found_nodes;
                *node_score += child_node_score;
                *scorable_node_count += child_scorable_nodes;
            }
        }

        *found_nodes += 1;
        if immediate_internal_children > 0 {
            *node_score +=
                correctly_positioned_immediate_children as f32 / immediate_internal_children as f32;
            *scorable_node_count += 1;
        }
    }

    /// Measures the cache quality of the tree. 1.0 means perfectly cache-optimal.
    pub fn measure_cache_quality(&self) -> f32 {
        let mut found_nodes = 0;
        let mut node_score = 0.0f32;
        let mut scorable_node_count = 0;
        self.measure_cache_quality_recursive(0, &mut found_nodes, &mut node_score, &mut scorable_node_count);
        if scorable_node_count > 0 {
            node_score / scorable_node_count as f32
        } else {
            1.0
        }
    }

    /// Measures the cache quality of the subtree rooted at the given node index.
    pub fn measure_cache_quality_at(&self, node_index: i32) -> f32 {
        assert!(
            node_index >= 0 && node_index < self.node_count,
            "Measurement target index must be nonnegative and less than node count."
        );
        let mut found_nodes = 0;
        let mut node_score = 0.0f32;
        let mut scorable_node_count = 0;
        self.measure_cache_quality_recursive(
            node_index,
            &mut found_nodes,
            &mut node_score,
            &mut scorable_node_count,
        );
        if scorable_node_count > 0 {
            node_score / scorable_node_count as f32
        } else {
            1.0
        }
    }
}
