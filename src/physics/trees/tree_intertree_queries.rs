// Translated from BepuPhysics/Trees/Tree_IntertreeQueries.cs

use super::node::{Node, NodeChild};
use super::tree::Tree;
use super::tree_self_queries::IOverlapHandler;
use crate::utilities::bounding_box::BoundingBox;

impl Tree {
    fn dispatch_test_for_node_against_leaf<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        results: &mut TOverlapHandler,
    ) {
        if node_index < 0 {
            results.handle(Self::encode(node_index), leaf_index);
        } else {
            self.test_node_against_leaf(node_index, leaf_index, leaf_child, results);
        }
    }

    pub(crate) fn test_node_against_leaf<TOverlapHandler: IOverlapHandler>(
        &self,
        node_index: i32,
        leaf_index: i32,
        leaf_child: &NodeChild,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let node = self.nodes.get(node_index);
            let b_index = node.b.index;
            let a_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.a);
            let b_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.b);
            if a_intersects {
                self.dispatch_test_for_node_against_leaf(
                    leaf_index,
                    leaf_child,
                    node.a.index,
                    results,
                );
            }
            if b_intersects {
                self.dispatch_test_for_node_against_leaf(
                    leaf_index,
                    leaf_child,
                    b_index,
                    results,
                );
            }
        }
    }

    fn dispatch_test_for_leaf_against_node_intertree<TOverlapHandler: IOverlapHandler>(
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        tree_b: &Tree,
        results: &mut TOverlapHandler,
    ) {
        if node_index < 0 {
            results.handle(leaf_index, Self::encode(node_index));
        } else {
            Self::test_leaf_against_node_intertree(
                leaf_index, leaf_child, node_index, tree_b, results,
            );
        }
    }

    pub(crate) fn test_leaf_against_node_intertree<TOverlapHandler: IOverlapHandler>(
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        tree_b: &Tree,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let node = tree_b.nodes.get(node_index);
            let b_index = node.b.index;
            let a_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.a);
            let b_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.b);
            if a_intersects {
                Self::dispatch_test_for_leaf_against_node_intertree(
                    leaf_index,
                    leaf_child,
                    node.a.index,
                    tree_b,
                    results,
                );
            }
            if b_intersects {
                Self::dispatch_test_for_leaf_against_node_intertree(
                    leaf_index,
                    leaf_child,
                    b_index,
                    tree_b,
                    results,
                );
            }
        }
    }

    #[inline(always)]
    fn dispatch_test_for_nodes_intertree<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &NodeChild,
        b: &NodeChild,
        tree_b: &Tree,
        results: &mut TOverlapHandler,
    ) {
        if a.index >= 0 {
            if b.index >= 0 {
                self.get_overlaps_between_different_nodes_intertree(
                    self.nodes.get(a.index),
                    tree_b.nodes.get(b.index),
                    tree_b,
                    results,
                );
            } else {
                // leaf B versus node A. treeB nodes always should be in the second slot.
                self.test_node_against_leaf(a.index, Self::encode(b.index), b, results);
            }
        } else if b.index >= 0 {
            // leaf A versus node B. treeB nodes always should be in the second slot.
            Self::test_leaf_against_node_intertree(
                Self::encode(a.index),
                a,
                b.index,
                tree_b,
                results,
            );
        } else {
            // Two leaves.
            results.handle(Self::encode(a.index), Self::encode(b.index));
        }
    }

    pub(crate) fn get_overlaps_between_different_nodes_intertree<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &Node,
        b: &Node,
        tree_b: &Tree,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let aa_intersects = BoundingBox::intersects_unsafe(&a.a, &b.a);
            let ab_intersects = BoundingBox::intersects_unsafe(&a.a, &b.b);
            let ba_intersects = BoundingBox::intersects_unsafe(&a.b, &b.a);
            let bb_intersects = BoundingBox::intersects_unsafe(&a.b, &b.b);

            if aa_intersects {
                self.dispatch_test_for_nodes_intertree(&a.a, &b.a, tree_b, results);
            }
            if ab_intersects {
                self.dispatch_test_for_nodes_intertree(&a.a, &b.b, tree_b, results);
            }
            if ba_intersects {
                self.dispatch_test_for_nodes_intertree(&a.b, &b.a, tree_b, results);
            }
            if bb_intersects {
                self.dispatch_test_for_nodes_intertree(&a.b, &b.b, tree_b, results);
            }
        }
    }

    /// Gets pairs of leaf indices with bounding boxes which overlap between two trees.
    pub fn get_overlaps_with_tree<TOverlapHandler: IOverlapHandler>(
        &self,
        tree_b: &Tree,
        overlap_handler: &mut TOverlapHandler,
    ) {
        if self.leaf_count == 0 || tree_b.leaf_count == 0 {
            return;
        }
        unsafe {
            if self.leaf_count == 1 && tree_b.leaf_count >= 2 {
                // Tree A is degenerate; needs a special case.
                let a = self.nodes.get(0);
                let b = tree_b.nodes.get(0);
                let aa_intersects = BoundingBox::intersects_unsafe(&a.a, &b.a);
                let ab_intersects = BoundingBox::intersects_unsafe(&a.a, &b.b);
                if aa_intersects {
                    self.dispatch_test_for_nodes_intertree(
                        &a.a,
                        &b.a,
                        tree_b,
                        overlap_handler,
                    );
                }
                if ab_intersects {
                    self.dispatch_test_for_nodes_intertree(
                        &a.a,
                        &b.b,
                        tree_b,
                        overlap_handler,
                    );
                }
                return;
            }
            if self.leaf_count >= 2 && tree_b.leaf_count == 1 {
                // Tree B is degenerate; needs a special case.
                let a = self.nodes.get(0);
                let b = tree_b.nodes.get(0);
                let aa_intersects = BoundingBox::intersects_unsafe(&a.a, &b.a);
                let ba_intersects = BoundingBox::intersects_unsafe(&a.b, &b.a);
                if aa_intersects {
                    self.dispatch_test_for_nodes_intertree(
                        &a.a,
                        &b.a,
                        tree_b,
                        overlap_handler,
                    );
                }
                if ba_intersects {
                    self.dispatch_test_for_nodes_intertree(
                        &a.b,
                        &b.a,
                        tree_b,
                        overlap_handler,
                    );
                }
                return;
            }
            if self.leaf_count == 1 && tree_b.leaf_count == 1 {
                // Both degenerate.
                if BoundingBox::intersects_unsafe(&self.nodes.get(0).a, &tree_b.nodes.get(0).a) {
                    self.dispatch_test_for_nodes_intertree(
                        &self.nodes.get(0).a,
                        &tree_b.nodes.get(0).a,
                        tree_b,
                        overlap_handler,
                    );
                }
                return;
            }
        }
        // Both trees have complete nodes; use the general case.
        self.get_overlaps_between_different_nodes_intertree(
            self.nodes.get(0),
            tree_b.nodes.get(0),
            tree_b,
            overlap_handler,
        );
    }
}
