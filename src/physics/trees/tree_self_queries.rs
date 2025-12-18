// Translated from BepuPhysics/Trees/Tree_SelfQueries.cs (core recursive overlap testing)

use super::node::{Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;

/// Overlap callback for tree overlap queries.
pub trait IOverlapHandler {
    /// Handles an overlap between leaves.
    fn handle(&mut self, index_a: i32, index_b: i32);
}

/// Overlap callback for tree overlap queries. Used in multithreaded contexts.
pub trait IThreadedOverlapHandler {
    /// Handles an overlap between leaves.
    fn handle(&mut self, index_a: i32, index_b: i32, worker_index: i32, managed_context: *mut u8);
}

impl Tree {
    fn dispatch_test_for_leaf<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        results: &mut TOverlapHandler,
    ) {
        if node_index < 0 {
            results.handle(leaf_index, Self::encode(node_index));
        } else {
            self.test_leaf_against_node(leaf_index, leaf_child, node_index, results);
        }
    }

    pub(crate) fn test_leaf_against_node<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let node = self.nodes.get(node_index);
            let b_index = node.b.index;
            let a_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.a);
            let b_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.b);
            if a_intersects {
                self.dispatch_test_for_leaf(leaf_index, leaf_child, node.a.index, results);
            }
            if b_intersects {
                self.dispatch_test_for_leaf(leaf_index, leaf_child, b_index, results);
            }
        }
    }

    #[inline(always)]
    fn dispatch_test_for_nodes<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &NodeChild,
        b: &NodeChild,
        results: &mut TOverlapHandler,
    ) {
        if a.index >= 0 {
            if b.index >= 0 {
                self.get_overlaps_between_different_nodes(
                    self.nodes.get(a.index),
                    self.nodes.get(b.index),
                    results,
                );
            } else {
                // leaf B versus node A.
                self.test_leaf_against_node(Self::encode(b.index), b, a.index, results);
            }
        } else if b.index >= 0 {
            // leaf A versus node B.
            self.test_leaf_against_node(Self::encode(a.index), a, b.index, results);
        } else {
            // Two leaves.
            results.handle(Self::encode(a.index), Self::encode(b.index));
        }
    }

    pub(crate) fn get_overlaps_between_different_nodes<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &Node,
        b: &Node,
        results: &mut TOverlapHandler,
    ) {
        // There are no shared children, so test them all.
        unsafe {
            let aa_intersects = BoundingBox::intersects_unsafe(&a.a, &b.a);
            let ab_intersects = BoundingBox::intersects_unsafe(&a.a, &b.b);
            let ba_intersects = BoundingBox::intersects_unsafe(&a.b, &b.a);
            let bb_intersects = BoundingBox::intersects_unsafe(&a.b, &b.b);

            if aa_intersects {
                self.dispatch_test_for_nodes(&a.a, &b.a, results);
            }
            if ab_intersects {
                self.dispatch_test_for_nodes(&a.a, &b.b, results);
            }
            if ba_intersects {
                self.dispatch_test_for_nodes(&a.b, &b.a, results);
            }
            if bb_intersects {
                self.dispatch_test_for_nodes(&a.b, &b.b, results);
            }
        }
    }

    pub(crate) fn get_overlaps_in_node<TOverlapHandler: IOverlapHandler>(
        &self,
        node: &Node,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let ab = BoundingBox::intersects_unsafe(&node.a, &node.b);
            if node.a.index >= 0 {
                self.get_overlaps_in_node(self.nodes.get(node.a.index), results);
            }
            if node.b.index >= 0 {
                self.get_overlaps_in_node(self.nodes.get(node.b.index), results);
            }
            // Test all different nodes.
            if ab {
                self.dispatch_test_for_nodes(&node.a, &node.b, results);
            }
        }
    }

    /// Gets pairs of leaf indices with bounding boxes which overlap within this tree.
    pub fn get_self_overlaps<TOverlapHandler: IOverlapHandler>(
        &self,
        results: &mut TOverlapHandler,
    ) {
        // If there are less than two leaves, there can't be any overlap.
        if self.leaf_count < 2 {
            return;
        }
        self.get_overlaps_in_node(self.nodes.get(0), results);
    }
}
