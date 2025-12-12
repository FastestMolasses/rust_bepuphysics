// Translated from BepuPhysics/Trees/Tree_Add.cs

use super::leaf::Leaf;
use super::node::{CostOrFlag, Metanode, Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::{BoundingBox, BoundingBox4};
use crate::utilities::memory::buffer_pool::BufferPool;
use std::mem::MaybeUninit;

/// Marker types for controlling rotation behavior during insertion.
/// These correspond to C#'s unmanaged marker structs for generic specialization.
struct InsertShouldNotRotate;
struct InsertShouldRotateTopDown;
struct InsertShouldRotateBottomUp;

/// Trait to distinguish rotation strategies at compile time.
trait InsertionRotation {
    const ROTATE_TOP_DOWN: bool;
    const ROTATE_BOTTOM_UP: bool;
}

impl InsertionRotation for InsertShouldNotRotate {
    const ROTATE_TOP_DOWN: bool = false;
    const ROTATE_BOTTOM_UP: bool = false;
}

impl InsertionRotation for InsertShouldRotateTopDown {
    const ROTATE_TOP_DOWN: bool = true;
    const ROTATE_BOTTOM_UP: bool = false;
}

impl InsertionRotation for InsertShouldRotateBottomUp {
    const ROTATE_TOP_DOWN: bool = false;
    const ROTATE_BOTTOM_UP: bool = true;
}

impl Tree {
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    ///
    /// This performs no incremental refinement. Slightly cheaper than `add`, but tree quality
    /// depends on insertion order. Pathological orders can result in maximally imbalanced trees.
    pub fn add_without_refinement(&mut self, bounds: BoundingBox, pool: &mut BufferPool) -> i32 {
        self.add_internal::<InsertShouldNotRotate>(bounds, pool)
    }

    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    ///
    /// Performs incrementally refining tree rotations down along the insertion path.
    pub fn add(&mut self, bounds: BoundingBox, pool: &mut BufferPool) -> i32 {
        self.add_internal::<InsertShouldRotateTopDown>(bounds, pool)
    }

    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    ///
    /// Performs incrementally refining tree rotations up along the insertion path.
    /// Slightly better quality than `add`, but also slightly more expensive.
    pub fn add_with_bottom_up_refinement(
        &mut self,
        bounds: BoundingBox,
        pool: &mut BufferPool,
    ) -> i32 {
        self.add_internal::<InsertShouldRotateBottomUp>(bounds, pool)
    }

    fn add_internal<TShouldRotate: InsertionRotation>(
        &mut self,
        bounds: BoundingBox,
        pool: &mut BufferPool,
    ) -> i32 {
        // Ensure sufficient room. We don't want to deal with invalidated pointers mid-add.
        if self.leaves.len() == self.leaf_count {
            // Note that, while we add 1, the underlying pool will request the next higher power of 2 in bytes.
            self.resize(pool, self.leaf_count + 1);
        }

        if self.leaf_count < 2 {
            // The root is partial.
            unsafe {
                let nodes_ptr = self.nodes.as_ptr() as *mut Node;
                let leaf_child = Self::node_child_mut(
                    &mut *nodes_ptr,
                    self.leaf_count,
                );
                // Reinterpret BoundingBox as NodeChild (both are 32 bytes, same min/max layout).
                let bounds_as_child = &*(&bounds as *const BoundingBox as *const NodeChild);
                *leaf_child = *bounds_as_child;
                let leaf_count = self.leaf_count;
                let leaf_index = self.add_leaf(0, leaf_count);
                let leaf_child = Self::node_child_mut(
                    &mut *nodes_ptr,
                    leaf_count,
                );
                leaf_child.index = Self::encode(leaf_index);
                leaf_child.leaf_count = 1;
                return leaf_index;
            }
        }

        // The tree is complete; traverse to find the best place to insert the leaf.
        let mut node_index: i32 = 0;
        let bounds4 = unsafe { *(&bounds as *const BoundingBox as *const BoundingBox4) };
        let new_node_index = self.allocate_node();
        // We only ever insert into child A, and it's guaranteed to belong to a new node,
        // so we don't have to wait to add the leaf.
        let new_leaf_index = self.add_leaf(new_node_index, 0);

        loop {
            // Rotating from the top down produces a tree that's lower quality than rotating from the bottom up.
            // The advantage is that top down is a little faster.
            if TShouldRotate::ROTATE_TOP_DOWN {
                self.try_rotate_node(node_index);
            }

            unsafe {
                let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                    .add(node_index as usize);

                // Choose whichever child requires less bounds expansion.
                let mut merged_a = MaybeUninit::<BoundingBox4>::uninit();
                BoundingBox::create_merged_unsafe(&bounds4, &node.a, &mut merged_a);
                let mut merged_b = MaybeUninit::<BoundingBox4>::uninit();
                BoundingBox::create_merged_unsafe(&bounds4, &node.b, &mut merged_b);

                let merged_a = merged_a.assume_init();
                let merged_b = merged_b.assume_init();

                let bounds_increase_a =
                    Tree::compute_bounds_metric(&merged_a) - Tree::compute_bounds_metric(&node.a);
                let bounds_increase_b =
                    Tree::compute_bounds_metric(&merged_b) - Tree::compute_bounds_metric(&node.b);

                let use_a = if bounds_increase_a == bounds_increase_b {
                    node.a.leaf_count < node.b.leaf_count
                } else {
                    bounds_increase_a < bounds_increase_b
                };

                if use_a {
                    if node.a.leaf_count == 1 {
                        // The merge target is a leaf. We'll need a new internal node.
                        let new_node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                            .add(new_node_index as usize);
                        let new_metanode = self.metanodes.get_mut(new_node_index);
                        new_metanode.parent = node_index;
                        new_metanode.index_in_parent = 0;

                        // Create the new child for the inserted leaf.
                        new_node.a = *(&bounds as *const BoundingBox as *const NodeChild);
                        new_node.a.leaf_count = 1;
                        new_node.a.index = Self::encode(new_leaf_index);

                        // Move the leaf that used to be in the parent down into its new slot.
                        new_node.b = node.a;
                        // Update the moved leaf's location in the leaves.
                        *self.leaves.get_mut(Self::encode(node.a.index)) =
                            Leaf::new(new_node_index, 1);

                        // Update the parent's child reference to point to the new node.
                        node.a = *(&merged_a as *const BoundingBox4 as *const NodeChild);
                        node.a.leaf_count = 2;
                        node.a.index = new_node_index;
                        break;
                    } else {
                        // Just traversing into an internal node.
                        let index = node.a.index;
                        let leaf_count = node.a.leaf_count + 1;
                        node.a = *(&merged_a as *const BoundingBox4 as *const NodeChild);
                        node.a.index = index;
                        node.a.leaf_count = leaf_count;
                        node_index = index;
                    }
                } else {
                    if node.b.leaf_count == 1 {
                        let new_node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                            .add(new_node_index as usize);
                        let new_metanode = self.metanodes.get_mut(new_node_index);
                        new_metanode.parent = node_index;
                        new_metanode.index_in_parent = 1;

                        new_node.a = *(&bounds as *const BoundingBox as *const NodeChild);
                        new_node.a.leaf_count = 1;
                        new_node.a.index = Self::encode(new_leaf_index);

                        new_node.b = node.b;
                        *self.leaves.get_mut(Self::encode(node.b.index)) =
                            Leaf::new(new_node_index, 1);

                        node.b = *(&merged_b as *const BoundingBox4 as *const NodeChild);
                        node.b.leaf_count = 2;
                        node.b.index = new_node_index;
                        break;
                    } else {
                        let index = node.b.index;
                        let leaf_count = node.b.leaf_count + 1;
                        node.b = *(&merged_b as *const BoundingBox4 as *const NodeChild);
                        node.b.index = index;
                        node.b.leaf_count = leaf_count;
                        node_index = index;
                    }
                }
            }
        }

        if TShouldRotate::ROTATE_BOTTOM_UP {
            let mut parent_index = self.leaves.get(new_leaf_index).node_index();
            while parent_index >= 0 {
                self.try_rotate_node(parent_index);
                parent_index = self.metanodes.get(parent_index).parent;
            }
        }

        new_leaf_index
    }

    fn try_rotate_node(&mut self, rotation_root_index: i32) {
        unsafe {
            let root = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                .add(rotation_root_index as usize);

            let cost_a = Tree::compute_bounds_metric(&root.a);
            let cost_b = Tree::compute_bounds_metric(&root.b);
            let mut left_rotation_cost_change = 0.0f32;
            let mut left_uses_a = false;
            let mut right_rotation_cost_change = 0.0f32;
            let mut right_uses_a = false;

            if root.a.index >= 0 {
                // Try a right rotation.
                let a = &*(self.nodes.as_ptr()).add(root.a.index as usize);
                let mut merged_aa_b = MaybeUninit::<NodeChild>::uninit();
                BoundingBox::create_merged_unsafe(&a.a, &root.b, &mut merged_aa_b);
                let mut merged_ab_b = MaybeUninit::<NodeChild>::uninit();
                BoundingBox::create_merged_unsafe(&a.b, &root.b, &mut merged_ab_b);
                let cost_aab = Tree::compute_bounds_metric(&merged_aa_b.assume_init());
                let cost_abb = Tree::compute_bounds_metric(&merged_ab_b.assume_init());
                right_uses_a = cost_aab < cost_abb;
                right_rotation_cost_change = cost_aab.min(cost_abb) - cost_a;
            }

            if root.b.index >= 0 {
                // Try a left rotation.
                let b = &*(self.nodes.as_ptr()).add(root.b.index as usize);
                let mut merged_ba_b = MaybeUninit::<NodeChild>::uninit();
                BoundingBox::create_merged_unsafe(&root.a, &b.a, &mut merged_ba_b);
                let mut merged_bb_b = MaybeUninit::<NodeChild>::uninit();
                BoundingBox::create_merged_unsafe(&root.a, &b.b, &mut merged_bb_b);
                let cost_bab = Tree::compute_bounds_metric(&merged_ba_b.assume_init());
                let cost_bbb = Tree::compute_bounds_metric(&merged_bb_b.assume_init());
                left_uses_a = cost_bab < cost_bbb;
                left_rotation_cost_change = cost_bab.min(cost_bbb) - cost_b;
            }

            if left_rotation_cost_change.min(right_rotation_cost_change) < 0.0 {
                // A rotation is worth it.
                if left_rotation_cost_change < right_rotation_cost_change {
                    // Left rotation wins!
                    let node_index_to_replace = root.b.index;
                    let node_to_replace = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                        .add(node_index_to_replace as usize);
                    let child_to_shift_up = if left_uses_a {
                        node_to_replace.b
                    } else {
                        node_to_replace.a
                    };
                    let child_to_shift_left = if left_uses_a {
                        node_to_replace.a
                    } else {
                        node_to_replace.b
                    };
                    node_to_replace.a = root.a;
                    node_to_replace.b = child_to_shift_left;

                    let mut merged = MaybeUninit::<NodeChild>::uninit();
                    BoundingBox::create_merged_unsafe(
                        &node_to_replace.a,
                        &node_to_replace.b,
                        &mut merged,
                    );
                    root.a = merged.assume_init();
                    root.a.index = node_index_to_replace;
                    root.a.leaf_count =
                        node_to_replace.a.leaf_count + node_to_replace.b.leaf_count;
                    root.b = child_to_shift_up;

                    *self.metanodes.get_mut(node_index_to_replace) = Metanode {
                        parent: rotation_root_index,
                        index_in_parent: 0,
                        cost_or_flag: CostOrFlag { refine_flag: 0 },
                    };

                    if child_to_shift_up.index < 0 {
                        *self.leaves.get_mut(Self::encode(child_to_shift_up.index)) =
                            Leaf::new(rotation_root_index, 1);
                    } else {
                        *self.metanodes.get_mut(child_to_shift_up.index) = Metanode {
                            parent: rotation_root_index,
                            index_in_parent: 1,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }

                    if node_to_replace.a.index < 0 {
                        *self.leaves.get_mut(Self::encode(node_to_replace.a.index)) =
                            Leaf::new(node_index_to_replace, 0);
                    } else {
                        *self.metanodes.get_mut(node_to_replace.a.index) = Metanode {
                            parent: node_index_to_replace,
                            index_in_parent: 0,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }

                    if node_to_replace.b.index < 0 {
                        *self.leaves.get_mut(Self::encode(node_to_replace.b.index)) =
                            Leaf::new(node_index_to_replace, 1);
                    } else {
                        *self.metanodes.get_mut(node_to_replace.b.index) = Metanode {
                            parent: node_index_to_replace,
                            index_in_parent: 1,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }
                } else {
                    // Right rotation wins!
                    let node_index_to_replace = root.a.index;
                    let node_to_replace = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                        .add(node_index_to_replace as usize);
                    let child_to_shift_up = if right_uses_a {
                        node_to_replace.b
                    } else {
                        node_to_replace.a
                    };
                    let child_to_shift_right = if right_uses_a {
                        node_to_replace.a
                    } else {
                        node_to_replace.b
                    };
                    node_to_replace.a = child_to_shift_right;
                    node_to_replace.b = root.b;

                    let mut merged = MaybeUninit::<NodeChild>::uninit();
                    BoundingBox::create_merged_unsafe(
                        &node_to_replace.a,
                        &node_to_replace.b,
                        &mut merged,
                    );
                    root.b = merged.assume_init();
                    root.b.index = node_index_to_replace;
                    root.b.leaf_count =
                        node_to_replace.a.leaf_count + node_to_replace.b.leaf_count;
                    root.a = child_to_shift_up;

                    *self.metanodes.get_mut(node_index_to_replace) = Metanode {
                        parent: rotation_root_index,
                        index_in_parent: 1,
                        cost_or_flag: CostOrFlag { refine_flag: 0 },
                    };

                    if child_to_shift_up.index < 0 {
                        *self.leaves.get_mut(Self::encode(child_to_shift_up.index)) =
                            Leaf::new(rotation_root_index, 0);
                    } else {
                        *self.metanodes.get_mut(child_to_shift_up.index) = Metanode {
                            parent: rotation_root_index,
                            index_in_parent: 0,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }

                    if node_to_replace.a.index < 0 {
                        *self.leaves.get_mut(Self::encode(node_to_replace.a.index)) =
                            Leaf::new(node_index_to_replace, 0);
                    } else {
                        *self.metanodes.get_mut(node_to_replace.a.index) = Metanode {
                            parent: node_index_to_replace,
                            index_in_parent: 0,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }

                    if node_to_replace.b.index < 0 {
                        *self.leaves.get_mut(Self::encode(node_to_replace.b.index)) =
                            Leaf::new(node_index_to_replace, 1);
                    } else {
                        *self.metanodes.get_mut(node_to_replace.b.index) = Metanode {
                            parent: node_index_to_replace,
                            index_in_parent: 1,
                            cost_or_flag: CostOrFlag { refine_flag: 0 },
                        };
                    }
                }
            }
        }
    }
}
