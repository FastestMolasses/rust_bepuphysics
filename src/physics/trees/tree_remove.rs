// Translated from BepuPhysics/Trees/Tree_Remove.cs

use super::leaf::Leaf;
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;

impl Tree {
    fn remove_node_at(&mut self, node_index: i32) {
        // Note that this function is a cache scrambling influence.
        // The cache optimization routines will take care of it later.
        debug_assert!(node_index < self.node_count && node_index >= 0);
        self.node_count -= 1;
        // If the node wasn't the last node in the list, it will be replaced by the last node.
        if node_index < self.node_count {
            unsafe {
                let last = self.node_count;
                // Swap last node for removed node.
                let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                    .add(node_index as usize);
                *node = *self.nodes.get(last);
                let meta_ptr = self.metanodes.as_ptr() as *mut super::node::Metanode;
                let metanode = &mut *meta_ptr.add(node_index as usize);
                *metanode = *&*meta_ptr.add(last as usize);

                // Update the moved node's pointers:
                // its parent's child pointer should change
                Self::node_child_mut(
                    &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                        .add(metanode.parent as usize),
                    metanode.index_in_parent,
                )
                .index = node_index;

                // its children's parent pointers should change
                for i in 0..2 {
                    let child = Self::node_child(node, i);
                    if child.index >= 0 {
                        self.metanodes.get_mut(child.index).parent = node_index;
                    } else {
                        // It's a leaf node. It needs to have its pointers updated.
                        *self.leaves.get_mut(Self::encode(child.index)) =
                            Leaf::new(node_index, i);
                    }
                }
            }
        }
    }

    fn refit_for_removal(&self, node_index: i32) {
        // Note that no attempt is made to refit the root node.
        unsafe {
            let mut node = &*(self.nodes.as_ptr()).add(node_index as usize);
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
                child_in_parent.leaf_count -= 1;
                node = &*parent;
                metanode = &*self.metanodes.get(metanode.parent);
            }
        }
    }

    /// Removes a leaf at an index. If the index is not at the end of the leaf list,
    /// the last leaf is swapped into the removed location.
    ///
    /// Returns the former index of the leaf that was moved into the removed leaf's slot, if any.
    /// If `leaf_index` pointed at the last slot in the list, returns -1 since no leaf was moved.
    pub fn remove_at(&mut self, leaf_index: i32) -> i32 {
        assert!(
            leaf_index >= 0 && leaf_index < self.leaf_count,
            "Leaf index must be a valid index in the tree's leaf array."
        );

        // Cache the leaf being removed.
        let leaf = *self.leaves.get(leaf_index);
        // Delete the leaf from the leaves array.
        self.leaf_count -= 1;
        if leaf_index < self.leaf_count {
            // The removed leaf was not the last leaf, so move the last leaf into its slot.
            unsafe {
                let last_leaf = *self.leaves.get(self.leaf_count);
                *self.leaves.get_mut(leaf_index) = last_leaf;
                Self::node_child_mut(
                    &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                        .add(last_leaf.node_index() as usize),
                    last_leaf.child_index(),
                )
                .index = Self::encode(leaf_index);
            }
        }

        unsafe {
            let node_index = leaf.node_index();
            let metanode = *self.metanodes.get(node_index);

            // Remove the leaf from this node.
            let surviving_child_index_in_node = leaf.child_index() ^ 1;
            let surviving_child = *Self::node_child(
                self.nodes.get(node_index),
                surviving_child_index_in_node,
            );

            // Check to see if this node should collapse.
            if metanode.parent >= 0 {
                // This is a non-root internal node.
                // Since there are only two children, the node containing the removed leaf will collapse.

                // Move the other node into the slot that used to point to the collapsing internal node.
                let child_in_parent = Self::node_child_mut(
                    &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                        .add(metanode.parent as usize),
                    metanode.index_in_parent,
                );
                child_in_parent.min = surviving_child.min;
                child_in_parent.max = surviving_child.max;
                child_in_parent.index = surviving_child.index;
                child_in_parent.leaf_count = surviving_child.leaf_count;

                if surviving_child.index < 0 {
                    // It's a leaf. Update the leaf's reference in the leaves array.
                    let other_leaf_index = Self::encode(surviving_child.index);
                    *self.leaves.get_mut(other_leaf_index) =
                        Leaf::new(metanode.parent, metanode.index_in_parent);
                } else {
                    // It's an internal node. Update its parent node.
                    let surviving_meta = self.metanodes.get_mut(surviving_child.index);
                    surviving_meta.parent = metanode.parent;
                    surviving_meta.index_in_parent = metanode.index_in_parent;
                }

                // Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                self.refit_for_removal(metanode.parent);

                // Remove the now dead node.
                self.remove_node_at(node_index);
            } else {
                // This is the root. It cannot collapse, but if the other child is an internal node,
                // then it will overwrite the root node.
                debug_assert!(
                    node_index == 0,
                    "Only the root should have a negative parent."
                );
                if self.leaf_count > 0 {
                    if surviving_child.index >= 0 {
                        // The surviving child is an internal node and it should replace the root.
                        let pulled_node_index = surviving_child.index;
                        *self.nodes.get_mut(0) = *self.nodes.get(pulled_node_index);
                        let root_meta = self.metanodes.get_mut(0);
                        root_meta.parent = -1;
                        root_meta.index_in_parent = -1;

                        // Update the parent pointers of the children of the moved internal node.
                        for i in 0..2i32 {
                            let child = *Self::node_child(self.nodes.get(0), i);
                            if child.index >= 0 {
                                self.metanodes.get_mut(child.index).parent = 0;
                            } else {
                                *self.leaves.get_mut(Self::encode(child.index)) = Leaf::new(0, i);
                            }
                        }
                        self.remove_node_at(pulled_node_index);
                    } else {
                        // The surviving child is a leaf node.
                        if surviving_child_index_in_node > 0 {
                            // It needs to be moved to keep the lowest slot filled.
                            self.nodes.get_mut(0).a = self.nodes.get(0).b;
                            *self.leaves.get_mut(Self::encode(surviving_child.index)) =
                                Leaf::new(0, 0);
                        }
                    }
                }
                // No need to perform a RefitForRemoval here; it's the root.
            }
        }

        if leaf_index < self.leaf_count {
            self.leaf_count
        } else {
            -1
        }
    }
}
