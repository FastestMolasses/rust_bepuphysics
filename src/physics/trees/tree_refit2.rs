// Translated from BepuPhysics/Trees/Tree_Refit2.cs

use super::leaf::Leaf;
use super::node::{Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use std::mem::MaybeUninit;

impl Tree {
    fn refit2_recursive(&self, child_in_parent: &mut NodeChild) {
        debug_assert!(self.leaf_count >= 2);
        unsafe {
            let node = &mut *(self.nodes.as_ptr() as *mut Node).add(child_in_parent.index as usize);
            if node.a.index >= 0 {
                self.refit2_recursive(&mut node.a);
            }
            if node.b.index >= 0 {
                self.refit2_recursive(&mut node.b);
            }
            let mut merged = MaybeUninit::new(*child_in_parent);
            BoundingBox::create_merged_unsafe_with_preservation(&node.a, &node.b, &mut merged);
            *child_in_parent = merged.assume_init();
        }
    }

    /// Updates the bounding boxes of all internal nodes in the tree.
    pub fn refit2(&self) {
        // No point in refitting a tree with no internal nodes!
        if self.leaf_count <= 2 {
            return;
        }
        let mut stub = NodeChild::default();
        // The root node's children are at index 0. We use a stub for the root's parent child.
        unsafe {
            let node = &mut *(self.nodes.as_ptr() as *mut Node);
            stub.index = 0;
            self.refit2_recursive(&mut stub);
        }
    }

    fn refit2_with_cache_optimization_recursive(
        source_node_index: i32,
        parent_index: i32,
        child_index_in_parent: i32,
        child_in_parent: &mut NodeChild,
        source_nodes: &Buffer<Node>,
        tree: &mut Tree,
    ) {
        debug_assert!(tree.leaf_count >= 2);

        unsafe {
            let source_node = &*source_nodes.get(source_node_index);
            let target_node_index = child_in_parent.index;
            let target_node =
                &mut *(tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
            let target_metanode = tree.metanodes.get_mut(target_node_index);
            target_metanode.parent = parent_index;
            target_metanode.index_in_parent = child_index_in_parent;

            let source_a = &source_node.a;
            let source_b = &source_node.b;
            let target_index_a = target_node_index + 1;
            let target_index_b = target_node_index + source_a.leaf_count;

            if source_a.index >= 0 {
                target_node.a.index = target_index_a;
                target_node.a.leaf_count = source_a.leaf_count;
                Self::refit2_with_cache_optimization_recursive(
                    source_a.index,
                    target_node_index,
                    0,
                    &mut target_node.a,
                    source_nodes,
                    tree,
                );
            } else {
                // It's a leaf; copy over the source verbatim.
                target_node.a = *source_a;
                *tree.leaves.get_mut(Self::encode(source_a.index)) =
                    Leaf::new(target_node_index, 0);
            }

            if source_b.index >= 0 {
                target_node.b.index = target_index_b;
                target_node.b.leaf_count = source_b.leaf_count;
                Self::refit2_with_cache_optimization_recursive(
                    source_b.index,
                    target_node_index,
                    1,
                    &mut target_node.b,
                    source_nodes,
                    tree,
                );
            } else {
                target_node.b = *source_b;
                *tree.leaves.get_mut(Self::encode(source_b.index)) =
                    Leaf::new(target_node_index, 1);
            }

            // Re-borrow target_node after recursive calls.
            let target_node =
                &mut *(tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
            let mut merged = MaybeUninit::new(*child_in_parent);
            BoundingBox::create_merged_unsafe_with_preservation(
                &target_node.a,
                &target_node.b,
                &mut merged,
            );
            *child_in_parent = merged.assume_init();
        }
    }

    /// Updates the bounding boxes of all internal nodes in the tree. The refit is based on the
    /// provided source_nodes, and the results are written into the tree's current nodes, metanodes,
    /// and leaves buffers. The nodes and metanodes will be in depth traversal order.
    /// The input source buffer is not modified.
    pub fn refit2_with_cache_optimization_from_source(&mut self, source_nodes: &Buffer<Node>) {
        // No point in refitting a tree with no internal nodes!
        if self.leaf_count <= 2 {
            return;
        }
        let mut stub = NodeChild::default();
        stub.index = 0;
        // We need to pass self mutably but also read from source_nodes.
        // Use a raw pointer to work around the borrow checker here.
        let source_nodes_copy = *source_nodes;
        Self::refit2_with_cache_optimization_recursive(
            0,
            -1,
            -1,
            &mut stub,
            &source_nodes_copy,
            self,
        );
    }

    /// Updates the bounding boxes of all internal nodes in the tree. Reallocates the nodes and
    /// metanodes and writes the refit tree into them in depth first traversal order.
    /// The tree instance is modified to point to the new nodes and metanodes.
    pub fn refit2_with_cache_optimization(
        &mut self,
        pool: &mut BufferPool,
        dispose_original_nodes: bool,
    ) {
        // No point in refitting a tree with no internal nodes!
        if self.leaf_count <= 2 {
            return;
        }
        let old_nodes = self.nodes;
        self.nodes = pool.take_at_least::<Node>(old_nodes.len());
        self.refit2_with_cache_optimization_from_source(&old_nodes);
        if dispose_original_nodes {
            let mut old = old_nodes;
            pool.return_buffer(&mut old);
        }
    }
}
