// Translated from BepuPhysics/Trees/Tree_RefineCommon.cs

use super::node::{Metanode, Node};
use super::tree::Tree;
use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::collections::quicksort::Quicksort;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use std::cmp::Ordering;

struct I32Comparer;
impl RefComparer<i32> for I32Comparer {
    fn compare(&self, a: &i32, b: &i32) -> Ordering {
        a.cmp(b)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SubtreeHeapEntry {
    pub index: i32,
    pub cost: f32,
}

/// An unsafe max-heap of SubtreeHeapEntry used to iteratively collect the highest-cost subtrees
/// during refinement. Uses a raw pointer to a pre-allocated entry buffer.
pub struct SubtreeBinaryHeap {
    pub entries: *mut SubtreeHeapEntry,
    pub count: i32,
}

impl SubtreeBinaryHeap {
    pub unsafe fn new(entries: *mut SubtreeHeapEntry) -> Self {
        Self { entries, count: 0 }
    }

    /// Inserts the children of a node into the heap. Leaf children are immediately added to the
    /// subtrees list. Internal children are sifted into the max-heap by cost.
    pub unsafe fn insert(&mut self, node: &Node, subtrees: &mut QuickList<i32>) {
        // Process both children (A and B).
        let children = &node.a as *const super::node::NodeChild;
        for child_index in 0..2 {
            let child = &*children.add(child_index);
            if child.index >= 0 {
                let mut index = self.count;
                let cost =
                    Tree::compute_bounds_metric_vecs(&child.min, &child.max);
                self.count += 1;

                // Sift up.
                while index > 0 {
                    let parent_index = (index - 1) >> 1;
                    let parent = &*self.entries.add(parent_index as usize);
                    if parent.cost < cost {
                        // Pull the parent down.
                        *self.entries.add(index as usize) = *parent;
                        index = parent_index;
                    } else {
                        break;
                    }
                }
                let entry = &mut *self.entries.add(index as usize);
                entry.index = child.index;
                entry.cost = cost;
            } else {
                // Immediately add leaf nodes.
                subtrees.add_unsafely(child.index);
            }
        }
    }

    /// Pops the highest-cost entry from the heap.
    #[inline(always)]
    unsafe fn pop(&mut self) -> SubtreeHeapEntry {
        let entry = *self.entries;
        self.count -= 1;
        let cost = (*self.entries.add(self.count as usize)).cost;

        // Pull elements up to fill the gap.
        let mut index = 0i32;
        loop {
            let child_index_a = (index << 1) + 1;
            let child_index_b = (index << 1) + 2;
            if child_index_b < self.count {
                // Both children are available.
                let child_a = &*self.entries.add(child_index_a as usize);
                let child_b = &*self.entries.add(child_index_b as usize);
                if child_a.cost > child_b.cost {
                    if cost > child_a.cost {
                        break;
                    }
                    *self.entries.add(index as usize) =
                        *self.entries.add(child_index_a as usize);
                    index = child_index_a;
                } else {
                    if cost > child_b.cost {
                        break;
                    }
                    *self.entries.add(index as usize) =
                        *self.entries.add(child_index_b as usize);
                    index = child_index_b;
                }
            } else if child_index_a < self.count {
                // Only one child available.
                let child_a = &*self.entries.add(child_index_a as usize);
                if cost > child_a.cost {
                    break;
                }
                *self.entries.add(index as usize) =
                    *self.entries.add(child_index_a as usize);
                index = child_index_a;
            } else {
                // The children were beyond the heap.
                break;
            }
        }
        // Move the last entry into position.
        *self.entries.add(index as usize) = *self.entries.add(self.count as usize);
        entry
    }

    /// Repeatedly pops from the heap until an expandable node is found.
    /// Returns true if a node was found whose children can fit in the remaining space.
    /// Non-expandable nodes (either because of space or because they have refine flags set)
    /// are added directly to the subtrees list.
    #[inline(always)]
    pub unsafe fn try_pop(
        &mut self,
        metanodes: &mut Buffer<Metanode>,
        remaining_subtree_space: &mut i32,
        subtrees: &mut QuickList<i32>,
        index: &mut i32,
        cost: &mut f32,
    ) -> bool {
        while self.count > 0 {
            let entry = self.pop();
            // Only expand if children will fit and the node is not a refinement target.
            if *remaining_subtree_space >= 1
                && unsafe { metanodes.get(entry.index).cost_or_flag.refine_flag } == 0
            {
                *index = entry.index;
                *cost = entry.cost;
                *remaining_subtree_space -= 1;
                return true;
            } else {
                // Node's children did not fit, or it was a refinement target.
                subtrees.add_unsafely(entry.index);
            }
        }
        *index = -1;
        *cost = -1.0;
        false
    }
}

impl Tree {
    /// Collects subtrees iteratively by choosing the highest cost subtree repeatedly.
    /// This collects every child of a given node at once — the set of subtrees must not
    /// include only SOME of the children of a node.
    pub unsafe fn collect_subtrees(
        &mut self,
        node_index: i32,
        maximum_subtrees: i32,
        entries: *mut SubtreeHeapEntry,
        subtrees: &mut QuickList<i32>,
        internal_nodes: &mut QuickList<i32>,
        treelet_cost: &mut f32,
    ) {
        let node = &*self.nodes.get(node_index);
        debug_assert!(
            maximum_subtrees >= 2,
            "Can't only consider some of a node's children, but specified maximum_subtrees precludes the treelet root's children."
        );

        let mut priority_queue = SubtreeBinaryHeap::new(entries);
        priority_queue.insert(node, subtrees);

        // Note: treelet root is NOT added to internal nodes list.
        // Its cost is excluded from treelet_cost because the treelet root cannot change.
        *treelet_cost = 0.0;
        let mut remaining_subtree_space =
            maximum_subtrees - priority_queue.count - subtrees.count;
        let mut highest_index = 0i32;
        let mut highest_cost = 0.0f32;
        while priority_queue.try_pop(
            &mut self.metanodes,
            &mut remaining_subtree_space,
            subtrees,
            &mut highest_index,
            &mut highest_cost,
        ) {
            *treelet_cost += highest_cost;
            internal_nodes.add_unsafely(highest_index);

            // Add all children to the set of subtrees.
            let expanded_node = &*self.nodes.get(highest_index);
            priority_queue.insert(expanded_node, subtrees);
        }

        for i in 0..priority_queue.count {
            subtrees.add_unsafely((*priority_queue.entries.add(i as usize)).index);
        }

        // Sort internal nodes so the depth-first builder produces less cache-scrambled results.
        if internal_nodes.count > 0 {
            let comparer = I32Comparer;
            Quicksort::sort_keys(
                std::slice::from_raw_parts_mut(
                    &mut internal_nodes[0] as *mut i32,
                    internal_nodes.count as usize,
                ),
                0,
                internal_nodes.count - 1,
                &comparer,
            );
        }
    }

    /// Debug validation of staging nodes.
    #[cfg(debug_assertions)]
    pub unsafe fn validate_staging(
        &self,
        staging_nodes: *mut Node,
        subtree_node_pointers: &mut QuickList<i32>,
        treelet_parent: i32,
        treelet_index_in_parent: i32,
        pool: &mut BufferPool,
    ) {
        let mut collected_subtree_references =
            QuickList::<i32>::with_capacity(subtree_node_pointers.count, pool);
        let mut internal_references =
            QuickList::<i32>::with_capacity(subtree_node_pointers.count, pool);
        internal_references.add(0, pool);

        let mut found_subtrees = 0i32;
        let mut found_leaf_count = 0i32;
        self.validate_staging_recursive(
            staging_nodes,
            0,
            subtree_node_pointers,
            &mut collected_subtree_references,
            &mut internal_references,
            pool,
            &mut found_subtrees,
            &mut found_leaf_count,
        );

        assert!(
            treelet_parent >= -1 && treelet_parent < self.node_count,
            "Bad treelet parent."
        );
        assert!(
            treelet_index_in_parent >= -1
                && (treelet_parent < 0 || treelet_index_in_parent < 2),
            "Bad treelet index in parent."
        );
        if treelet_parent >= 0 {
            let parent_node = &*self.nodes.get(treelet_parent);
            let child = Self::node_child(parent_node, treelet_index_in_parent);
            assert_eq!(
                child.leaf_count, found_leaf_count,
                "Bad leaf count."
            );
        }
        assert_eq!(
            subtree_node_pointers.count, found_subtrees,
            "Bad subtree found count."
        );
        for i in 0..collected_subtree_references.count {
            assert!(
                subtree_node_pointers.contains(&collected_subtree_references[i])
                    && collected_subtree_references
                        .contains(&subtree_node_pointers[i]),
                "Bad subtree reference."
            );
        }
        collected_subtree_references.dispose(pool);
        internal_references.dispose(pool);
    }

    #[cfg(debug_assertions)]
    unsafe fn validate_staging_recursive(
        &self,
        staging_nodes: *mut Node,
        staging_node_index: i32,
        subtree_node_pointers: &mut QuickList<i32>,
        collected_subtree_references: &mut QuickList<i32>,
        internal_references: &mut QuickList<i32>,
        pool: &mut BufferPool,
        found_subtrees: &mut i32,
        found_leaf_count: &mut i32,
    ) {
        let staging_node = &*staging_nodes.add(staging_node_index as usize);
        let children = &staging_node.a as *const super::node::NodeChild;
        *found_subtrees = 0;
        *found_leaf_count = 0;
        for i in 0..2 {
            let child = &*children.add(i);
            if child.index >= 0 {
                assert!(
                    !internal_references.contains(&child.index),
                    "A child points to an internal node that was visited. Possible loop, or just general invalid."
                );
                internal_references.add(child.index, pool);
                let mut child_found_subtrees = 0i32;
                let mut child_found_leaf_count = 0i32;
                self.validate_staging_recursive(
                    staging_nodes,
                    child.index,
                    subtree_node_pointers,
                    collected_subtree_references,
                    internal_references,
                    pool,
                    &mut child_found_subtrees,
                    &mut child_found_leaf_count,
                );
                assert_eq!(
                    child_found_leaf_count, child.leaf_count,
                    "Bad leaf count."
                );
                *found_subtrees += child_found_subtrees;
                *found_leaf_count += child_found_leaf_count;
            } else {
                let subtree_node_pointer_index = Self::encode(child.index);
                let subtree_node_pointer =
                    subtree_node_pointers[subtree_node_pointer_index];
                if subtree_node_pointer >= 0 {
                    let node = &*self.nodes.get(subtree_node_pointer);
                    let mut total_leaf_count = 0i32;
                    let node_children = &node.a as *const super::node::NodeChild;
                    for child_index in 0..2 {
                        total_leaf_count +=
                            (*node_children.add(child_index)).leaf_count;
                    }
                    assert_eq!(
                        child.leaf_count, total_leaf_count,
                        "Bad leaf count."
                    );
                    *found_leaf_count += total_leaf_count;
                } else {
                    let _leaf_index = Self::encode(subtree_node_pointer);
                    assert_eq!(child.leaf_count, 1, "Bad leaf count.");
                    *found_leaf_count += 1;
                }
                *found_subtrees += 1;
                collected_subtree_references.add(subtree_node_pointer, pool);
            }
        }
    }
}
