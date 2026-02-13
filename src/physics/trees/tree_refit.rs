// Translated from BepuPhysics/Trees/Tree_Refit.cs

use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use glam::Vec3;

impl Tree {
    /// Refits the bounding box of every parent of the node recursively to the root.
    pub fn refit_for_node_bounds_change(&self, node_index: i32) {
        // Note that no attempt is made to refit the root node.
        // The root node is the only node that can have a number of children less than 2.
        unsafe {
            let mut node = &*self.nodes.get(node_index);
            let mut metanode = &*self.metanodes.get(node_index);
            while metanode.parent >= 0 {
                // Compute the new bounding box for this node.
                let parent = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                    .add(metanode.parent as usize);
                let child_in_parent = Self::node_child_mut(parent, metanode.index_in_parent);
                BoundingBox::create_merged(
                    node.a.min,
                    node.a.max,
                    node.b.min,
                    node.b.max,
                    &mut child_in_parent.min,
                    &mut child_in_parent.max,
                );
                node = &*parent;
                metanode = &*self.metanodes.get(metanode.parent);
            }
        }
    }

    /// Recursive refit helper.
    fn refit_recursive(&self, node_index: i32, min: &mut Vec3, max: &mut Vec3) {
        debug_assert!(self.leaf_count >= 2);
        unsafe {
            let node =
                &mut *(self.nodes.as_ptr() as *mut super::node::Node).add(node_index as usize);
            if node.a.index >= 0 {
                self.refit_recursive(node.a.index, &mut node.a.min, &mut node.a.max);
            }
            if node.b.index >= 0 {
                self.refit_recursive(node.b.index, &mut node.b.min, &mut node.b.max);
            }
            BoundingBox::create_merged(node.a.min, node.a.max, node.b.min, node.b.max, min, max);
        }
    }

    /// Updates the bounding boxes of all internal nodes in the tree.
    pub fn refit(&self) {
        // No point in refitting a tree with no internal nodes!
        if self.leaf_count <= 2 {
            return;
        }
        let mut root_min = Vec3::ZERO;
        let mut root_max = Vec3::ZERO;
        self.refit_recursive(0, &mut root_min, &mut root_max);
    }
}
