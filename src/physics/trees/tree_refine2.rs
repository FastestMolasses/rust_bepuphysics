// Translated from BepuPhysics/Trees/Tree_Refine2.cs
// Note: Multithreaded paths are omitted for now; only single-threaded refinement is provided.

use super::leaf::Leaf;
use super::node::{Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use std::simd::Simd;
use std::simd::cmp::SimdPartialEq;

const FLAG_FOR_ROOT_REFINEMENT_SUBTREE: i32 = 1 << 30;

/// A max-heap entry for the priority-queue based root refinement collection.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct HeapEntry {
    pub index: i32,
    pub cost: f32,
}

/// A buffer-backed binary max-heap used for priority-queue driven root refinement.
pub(crate) struct BinaryHeap {
    pub entries: Buffer<HeapEntry>,
    pub count: i32,
}

impl BinaryHeap {
    pub fn new(entries: Buffer<HeapEntry>) -> Self {
        Self { entries, count: 0 }
    }

    pub fn with_capacity(capacity: i32, pool: &mut BufferPool) -> Self {
        Self::new(pool.take_at_least(capacity))
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.entries);
    }

    pub fn insert(&mut self, index_to_insert: i32, cost: f32) {
        let mut index = self.count;
        self.count += 1;
        // Sift up.
        while index > 0 {
            let parent_index = (index - 1) >> 1;
            let parent = *self.entries.get(parent_index);
            if parent.cost < cost {
                *self.entries.get_mut(index) = parent;
                index = parent_index;
            } else {
                break;
            }
        }
        let entry = self.entries.get_mut(index);
        entry.index = index_to_insert;
        entry.cost = cost;
    }

    pub fn pop(&mut self) -> HeapEntry {
        let entry = *self.entries.get(0);
        self.count -= 1;
        let cost = self.entries.get(self.count).cost;

        let mut index = 0i32;
        loop {
            let child_index_a = (index << 1) + 1;
            let child_index_b = (index << 1) + 2;
            if child_index_b < self.count {
                let child_a = *self.entries.get(child_index_a);
                let child_b = *self.entries.get(child_index_b);
                if child_a.cost > child_b.cost {
                    if cost > child_a.cost {
                        break;
                    }
                    *self.entries.get_mut(index) = *self.entries.get(child_index_a);
                    index = child_index_a;
                } else {
                    if cost > child_b.cost {
                        break;
                    }
                    *self.entries.get_mut(index) = *self.entries.get(child_index_b);
                    index = child_index_b;
                }
            } else if child_index_a < self.count {
                let child_a = *self.entries.get(child_index_a);
                if cost > child_a.cost {
                    break;
                }
                *self.entries.get_mut(index) = *self.entries.get(child_index_a);
                index = child_index_a;
            } else {
                break;
            }
        }
        *self.entries.get_mut(index) = *self.entries.get(self.count);
        entry
    }
}

impl Tree {
    fn reify_root_refinement_node_child(
        &mut self,
        index: &mut i32,
        refinement_node_indices: &QuickList<i32>,
        real_node_index: i32,
        child_index_in_parent: i32,
    ) {
        if *index < 0 {
            // Leaf child.
            unsafe {
                *self.leaves.get_mut(Self::encode(*index)) =
                    Leaf::new(real_node_index, child_index_in_parent);
            }
        } else {
            if (*index as u32) < FLAG_FOR_ROOT_REFINEMENT_SUBTREE as u32 {
                // Internal node part of the refinement; remap.
                *index = refinement_node_indices[*index];
            } else {
                // Internal node NOT part of refinement; strip the flag.
                *index &= !FLAG_FOR_ROOT_REFINEMENT_SUBTREE;
            }
            unsafe {
                let metanode = self.metanodes.get_mut(*index);
                metanode.parent = real_node_index;
                metanode.index_in_parent = child_index_in_parent;
            }
        }
    }

    fn reify_root_refinement_range(
        start_index: i32,
        end_index: i32,
        node_indices: &QuickList<i32>,
        refinement_nodes: &Buffer<Node>,
        tree: &mut Tree,
    ) {
        for i in start_index..end_index {
            let real_node_index = node_indices[i];
            let refined_node =
                unsafe { &mut *(refinement_nodes.as_ptr() as *mut Node).add(i as usize) };
            tree.reify_root_refinement_node_child(
                &mut refined_node.a.index,
                node_indices,
                real_node_index,
                0,
            );
            tree.reify_root_refinement_node_child(
                &mut refined_node.b.index,
                node_indices,
                real_node_index,
                1,
            );
            unsafe {
                *(tree.nodes.as_ptr() as *mut Node).add(real_node_index as usize) = *refined_node;
            }
        }
    }

    fn reify_root_refinement_st(
        &mut self,
        refinement_node_indices: &QuickList<i32>,
        refinement_nodes: &Buffer<Node>,
    ) {
        Self::reify_root_refinement_range(
            0,
            refinement_node_indices.count,
            refinement_node_indices,
            refinement_nodes,
            self,
        );
    }

    fn reify_subtree_refinement_node_child(
        &mut self,
        index: &mut i32,
        refinement_node_indices: &QuickList<i32>,
        real_node_index: i32,
        child_index_in_parent: i32,
    ) {
        if *index < 0 {
            // Leaf child.
            unsafe {
                *self.leaves.get_mut(Self::encode(*index)) =
                    Leaf::new(real_node_index, child_index_in_parent);
            }
        } else {
            // Internal node part of the refinement; remap.
            *index = refinement_node_indices[*index];
            unsafe {
                let metanode = self.metanodes.get_mut(*index);
                metanode.parent = real_node_index;
                metanode.index_in_parent = child_index_in_parent;
            }
        }
    }

    fn reify_subtree_refinement_range(
        start_index: i32,
        end_index: i32,
        node_indices: &QuickList<i32>,
        refinement_nodes: &Buffer<Node>,
        tree: &mut Tree,
    ) {
        for i in start_index..end_index {
            let real_node_index = node_indices[i];
            let refined_node =
                unsafe { &mut *(refinement_nodes.as_ptr() as *mut Node).add(i as usize) };
            tree.reify_subtree_refinement_node_child(
                &mut refined_node.a.index,
                node_indices,
                real_node_index,
                0,
            );
            tree.reify_subtree_refinement_node_child(
                &mut refined_node.b.index,
                node_indices,
                real_node_index,
                1,
            );
            unsafe {
                *(tree.nodes.as_ptr() as *mut Node).add(real_node_index as usize) = *refined_node;
            }
        }
    }

    fn reify_subtree_refinement_st(
        &mut self,
        refinement_node_indices: &QuickList<i32>,
        refinement_nodes: &Buffer<Node>,
    ) {
        Self::reify_subtree_refinement_range(
            0,
            refinement_node_indices.count,
            refinement_node_indices,
            refinement_nodes,
            self,
        );
    }

    fn find_subtree_refinement_targets_recursive(
        &self,
        node_index: i32,
        left_leaf_count: i32,
        subtree_refinement_size: i32,
        target_subtree_refinement_count: i32,
        start_index: &mut i32,
        end_index: i32,
        refinement_targets: &mut QuickList<i32>,
    ) {
        if *start_index >= end_index
            || refinement_targets.count == target_subtree_refinement_count
        {
            return;
        }

        let node = unsafe { &*self.nodes.get(node_index) };
        let midpoint = left_leaf_count + node.a.leaf_count;
        if *start_index < midpoint {
            // Go left!
            if node.a.leaf_count <= subtree_refinement_size {
                if node.a.leaf_count > 2 {
                    *refinement_targets.allocate_unsafely() = node.a.index;
                }
                *start_index += node.a.leaf_count;
            } else {
                self.find_subtree_refinement_targets_recursive(
                    node.a.index,
                    left_leaf_count,
                    subtree_refinement_size,
                    target_subtree_refinement_count,
                    start_index,
                    end_index,
                    refinement_targets,
                );
            }
        }

        if *start_index >= end_index
            || refinement_targets.count == target_subtree_refinement_count
        {
            return;
        }

        if *start_index >= midpoint {
            // Go right!
            if node.b.leaf_count <= subtree_refinement_size {
                if node.b.leaf_count > 2 {
                    *refinement_targets.allocate_unsafely() = node.b.index;
                }
                *start_index += node.b.leaf_count;
            } else {
                self.find_subtree_refinement_targets_recursive(
                    node.b.index,
                    left_leaf_count + node.a.leaf_count,
                    subtree_refinement_size,
                    target_subtree_refinement_count,
                    start_index,
                    end_index,
                    refinement_targets,
                );
            }
        }
    }

    fn find_subtree_refinement_targets(
        &self,
        subtree_refinement_size: i32,
        target_subtree_refinement_count: i32,
        start_index: &mut i32,
        refinement_targets: &mut QuickList<i32>,
    ) {
        if *start_index >= self.leaf_count || *start_index < 0 {
            *start_index = 0;
        }
        let initial_start = *start_index;
        self.find_subtree_refinement_targets_recursive(
            0,
            0,
            subtree_refinement_size,
            target_subtree_refinement_count,
            start_index,
            self.leaf_count,
            refinement_targets,
        );
        if *start_index >= self.leaf_count
            && refinement_targets.count < target_subtree_refinement_count
        {
            *start_index = 0;
            let remaining_leaves = initial_start;
            self.find_subtree_refinement_targets_recursive(
                0,
                0,
                subtree_refinement_size,
                target_subtree_refinement_count,
                start_index,
                remaining_leaves,
                refinement_targets,
            );
        }
    }

    fn is_node_child_subtree_refinement_target(
        subtree_refinement_bundles: &Buffer<Simd<i32, 4>>,
        child: &NodeChild,
        parent_total_leaf_count: i32,
        subtree_refinement_size: i32,
    ) -> bool {
        if child.leaf_count <= subtree_refinement_size
            && parent_total_leaf_count > subtree_refinement_size
        {
            let search = Simd::<i32, 4>::splat(child.index);
            for i in 0..subtree_refinement_bundles.len() {
                let bundle = *subtree_refinement_bundles.get(i);
                if search.simd_eq(bundle).any() {
                    return true;
                }
            }
        }
        false
    }

    fn try_push_child_for_root_refinement(
        subtree_refinement_size: i32,
        subtree_refinement_root_bundles: &Buffer<Simd<i32, 4>>,
        node_total_leaf_count: i32,
        subtree_budget: i32,
        child: &NodeChild,
        stack: &mut QuickList<(i32, i32)>,
        root_refinement_subtrees: &mut QuickList<NodeChild>,
    ) {
        debug_assert!(subtree_budget >= 0);
        if subtree_budget == 1 {
            let allocated = root_refinement_subtrees.allocate_unsafely();
            *allocated = *child;
            allocated.index |= FLAG_FOR_ROOT_REFINEMENT_SUBTREE;
        } else if child.index < 0 {
            // Leaf node
            *root_refinement_subtrees.allocate_unsafely() = *child;
        } else {
            if Self::is_node_child_subtree_refinement_target(
                subtree_refinement_root_bundles,
                child,
                node_total_leaf_count,
                subtree_refinement_size,
            ) {
                let allocated = root_refinement_subtrees.allocate_unsafely();
                *allocated = *child;
                debug_assert!(
                    allocated.index < FLAG_FOR_ROOT_REFINEMENT_SUBTREE,
                    "Index flag overflow."
                );
                allocated.index |= FLAG_FOR_ROOT_REFINEMENT_SUBTREE;
            } else {
                *stack.allocate_unsafely() = (child.index, subtree_budget);
            }
        }
    }

    fn collect_subtrees_for_root_refinement(
        &self,
        root_refinement_size: i32,
        subtree_refinement_size: i32,
        pool: &mut BufferPool,
        subtree_refinement_targets: &QuickList<i32>,
        root_refinement_node_indices: &mut QuickList<i32>,
        root_refinement_subtrees: &mut QuickList<NodeChild>,
    ) {
        let mut root_stack =
            QuickList::<(i32, i32)>::with_capacity(root_refinement_size, pool);
        *root_stack.allocate_unsafely() = (0, root_refinement_size);

        let bundle_count =
            BundleIndexing::get_bundle_count(subtree_refinement_targets.count as usize);
        let subtree_refinement_target_bundles = Buffer::<Simd<i32, 4>>::new(
            subtree_refinement_targets.span.as_ptr() as *mut Simd<i32, 4>,
            bundle_count as i32,
            -1,
        );

        while let Some(node_to_visit) = root_stack.try_pop() {
            *root_refinement_node_indices.allocate_unsafely() = node_to_visit.0;
            let node = unsafe { &*self.nodes.get(node_to_visit.0) };
            let node_total_leaf_count = node.a.leaf_count + node.b.leaf_count;
            debug_assert!(node_to_visit.1 <= node_total_leaf_count);
            let lower_subtree_budget = ((node_to_visit.1 + 1) / 2)
                .min(node.a.leaf_count.min(node.b.leaf_count));
            let higher_subtree_budget = node_to_visit.1 - lower_subtree_budget;
            let use_smaller_for_a = lower_subtree_budget == node.a.leaf_count;
            let a_subtree_budget = if use_smaller_for_a {
                lower_subtree_budget
            } else {
                higher_subtree_budget
            };
            let b_subtree_budget = if use_smaller_for_a {
                higher_subtree_budget
            } else {
                lower_subtree_budget
            };

            Self::try_push_child_for_root_refinement(
                subtree_refinement_size,
                &subtree_refinement_target_bundles,
                node_total_leaf_count,
                b_subtree_budget,
                &node.b,
                &mut root_stack,
                root_refinement_subtrees,
            );
            Self::try_push_child_for_root_refinement(
                subtree_refinement_size,
                &subtree_refinement_target_bundles,
                node_total_leaf_count,
                a_subtree_budget,
                &node.a,
                &mut root_stack,
                root_refinement_subtrees,
            );
        }
        root_stack.dispose(pool);
    }

    fn try_push_child_for_root_refinement2(
        &self,
        subtree_refinement_size: i32,
        node_total_leaf_count: i32,
        subtree_refinement_root_bundles: &Buffer<Simd<i32, 4>>,
        child: &NodeChild,
        heap: &mut BinaryHeap,
        root_refinement_subtrees: &mut QuickList<NodeChild>,
    ) {
        if child.index < 0 {
            // Leaf node.
            *root_refinement_subtrees.allocate_unsafely() = *child;
        } else {
            if Self::is_node_child_subtree_refinement_target(
                subtree_refinement_root_bundles,
                child,
                node_total_leaf_count,
                subtree_refinement_size,
            ) {
                let allocated = root_refinement_subtrees.allocate_unsafely();
                *allocated = *child;
                debug_assert!(allocated.index < FLAG_FOR_ROOT_REFINEMENT_SUBTREE);
                allocated.index |= FLAG_FOR_ROOT_REFINEMENT_SUBTREE;
            } else {
                let metric = Self::compute_bounds_metric_vecs(&child.min, &child.max);
                heap.insert(child.index, metric * child.leaf_count as f32);
            }
        }
    }

    fn collect_subtrees_for_root_refinement_with_priority_queue(
        &self,
        root_refinement_size: i32,
        subtree_refinement_size: i32,
        pool: &mut BufferPool,
        subtree_refinement_targets: &QuickList<i32>,
        root_refinement_node_indices: &mut QuickList<i32>,
        root_refinement_subtrees: &mut QuickList<NodeChild>,
    ) {
        let mut heap = BinaryHeap::with_capacity(root_refinement_size, pool);
        heap.insert(0, 0.0);

        let bundle_count =
            BundleIndexing::get_bundle_count(subtree_refinement_targets.count as usize);
        let subtree_refinement_target_bundles = Buffer::<Simd<i32, 4>>::new(
            subtree_refinement_targets.span.as_ptr() as *mut Simd<i32, 4>,
            bundle_count as i32,
            -1,
        );

        while heap.count > 0
            && heap.count + root_refinement_subtrees.count < root_refinement_size
        {
            let entry = heap.pop();
            *root_refinement_node_indices.allocate_unsafely() = entry.index;
            let node = unsafe { &*self.nodes.get(entry.index) };
            let node_total_leaf_count = node.a.leaf_count + node.b.leaf_count;

            self.try_push_child_for_root_refinement2(
                subtree_refinement_size,
                node_total_leaf_count,
                &subtree_refinement_target_bundles,
                &node.b,
                &mut heap,
                root_refinement_subtrees,
            );
            self.try_push_child_for_root_refinement2(
                subtree_refinement_size,
                node_total_leaf_count,
                &subtree_refinement_target_bundles,
                &node.a,
                &mut heap,
                root_refinement_subtrees,
            );
        }

        // Add remaining heap entries as subtrees.
        for i in 0..heap.count {
            let entry = *heap.entries.get(i);
            unsafe {
                let metanode = &*self.metanodes.get(entry.index);
                debug_assert!(metanode.parent >= 0);
                let parent = &*self.nodes.get(metanode.parent);
                let child_in_parent = Self::node_child(parent, metanode.index_in_parent);
                let allocated = root_refinement_subtrees.allocate_unsafely();
                debug_assert!(child_in_parent.index >= 0);
                *allocated = *child_in_parent;
                allocated.index |= FLAG_FOR_ROOT_REFINEMENT_SUBTREE;
            }
        }
        pool.return_buffer(&mut heap.entries);
    }

    fn collect_subtrees_for_subtree_refinement(
        &self,
        refinement_target_root_node_index: i32,
        subtree_stack_buffer: &Buffer<i32>,
        subtree_refinement_node_indices: &mut QuickList<i32>,
        subtree_refinement_leaves: &mut QuickList<NodeChild>,
    ) {
        debug_assert!(
            subtree_refinement_leaves.count == 0 && subtree_refinement_node_indices.count == 0
        );
        let mut subtree_stack = QuickList::<i32>::new(*subtree_stack_buffer);
        *subtree_stack.allocate_unsafely() = refinement_target_root_node_index;
        while let Some(node_to_visit) = subtree_stack.try_pop() {
            let node = unsafe { &*self.nodes.get(node_to_visit) };
            *subtree_refinement_node_indices.allocate_unsafely() = node_to_visit;
            if node.b.index >= 0 {
                *subtree_stack.allocate_unsafely() = node.b.index;
            } else {
                *subtree_refinement_leaves.allocate_unsafely() = node.b;
            }
            if node.a.index >= 0 {
                *subtree_stack.allocate_unsafely() = node.a.index;
            } else {
                *subtree_refinement_leaves.allocate_unsafely() = node.a;
            }
        }
    }

    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// Single-threaded version.
    ///
    /// * `root_refinement_size` - Size of the refinement near the root. Non-positive skips root refinement.
    /// * `subtree_refinement_start_index` - Index for distributing subtree refinements across frames.
    /// * `subtree_refinement_count` - Number of subtree refinements to execute.
    /// * `subtree_refinement_size` - Target size of subtree refinements.
    /// * `pool` - Pool used for ephemeral allocations.
    /// * `use_priority_queue` - Whether to use priority queue for root refinement collection.
    pub fn refine2(
        &mut self,
        root_refinement_size: i32,
        subtree_refinement_start_index: &mut i32,
        subtree_refinement_count: i32,
        subtree_refinement_size: i32,
        pool: &mut BufferPool,
        use_priority_queue: bool,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let root_refinement_size = root_refinement_size.min(self.leaf_count);
        let subtree_refinement_size = subtree_refinement_size.min(self.leaf_count);

        // Pad out for vectorized containment test.
        let subtree_refinement_capacity =
            BundleIndexing::get_bundle_count(subtree_refinement_count as usize) as i32 * 4; // Vector<int>.Count = 4 for Simd<i32,4>

        let mut subtree_refinement_targets =
            QuickList::<i32>::with_capacity(subtree_refinement_capacity, pool);
        self.find_subtree_refinement_targets(
            subtree_refinement_size,
            subtree_refinement_count,
            subtree_refinement_start_index,
            &mut subtree_refinement_targets,
        );
        // Fill trailing slots with -1 to avoid false matches.
        for i in subtree_refinement_targets.count..subtree_refinement_capacity {
            if i < subtree_refinement_targets.span.len() {
                *subtree_refinement_targets.span.get_mut(i) = -1;
            }
        }

        let refinement_nodes_allocation: Buffer<Node> =
            pool.take_at_least(root_refinement_size.max(subtree_refinement_size));

        if root_refinement_size > 0 {
            let mut root_refinement_subtrees =
                QuickList::<NodeChild>::with_capacity(root_refinement_size, pool);
            let mut root_refinement_node_indices =
                QuickList::<i32>::with_capacity(root_refinement_size, pool);

            if use_priority_queue {
                self.collect_subtrees_for_root_refinement_with_priority_queue(
                    root_refinement_size,
                    subtree_refinement_size,
                    pool,
                    &subtree_refinement_targets,
                    &mut root_refinement_node_indices,
                    &mut root_refinement_subtrees,
                );
            } else {
                self.collect_subtrees_for_root_refinement(
                    root_refinement_size,
                    subtree_refinement_size,
                    pool,
                    &subtree_refinement_targets,
                    &mut root_refinement_node_indices,
                    &mut root_refinement_subtrees,
                );
            }

            debug_assert_eq!(
                root_refinement_node_indices.count,
                root_refinement_subtrees.count - 1
            );

            let root_refinement_nodes = Buffer::<Node>::new(
                refinement_nodes_allocation.as_ptr() as *mut Node,
                root_refinement_node_indices.count,
                -1,
            );
            // Run the binned builder over the collected subtrees.
            unsafe {
                Self::binned_build_static(
                    root_refinement_subtrees.span,
                    root_refinement_nodes,
                    Buffer::default(),
                    Buffer::default(),
                    Some(pool),
                    16,
                    64,
                    1.0 / 16.0,
                    64,
                );
            }
            self.reify_root_refinement_st(&root_refinement_node_indices, &root_refinement_nodes);
            root_refinement_subtrees.dispose(pool);
            root_refinement_node_indices.dispose(pool);
        }

        let mut subtree_refinement_node_indices =
            QuickList::<i32>::with_capacity(subtree_refinement_size, pool);
        let mut subtree_refinement_leaves =
            QuickList::<NodeChild>::with_capacity(subtree_refinement_size, pool);
        let subtree_stack_buffer: Buffer<i32> = pool.take_at_least(subtree_refinement_size);

        for i in 0..subtree_refinement_targets.count {
            self.collect_subtrees_for_subtree_refinement(
                subtree_refinement_targets[i],
                &subtree_stack_buffer,
                &mut subtree_refinement_node_indices,
                &mut subtree_refinement_leaves,
            );

            let refinement_nodes = Buffer::<Node>::new(
                refinement_nodes_allocation.as_ptr() as *mut Node,
                subtree_refinement_node_indices.count,
                -1,
            );
            unsafe {
                Self::binned_build_static(
                    subtree_refinement_leaves.span,
                    refinement_nodes,
                    Buffer::default(),
                    Buffer::default(),
                    Some(pool),
                    16,
                    64,
                    1.0 / 16.0,
                    64,
                );
            }
            self.reify_subtree_refinement_st(
                &subtree_refinement_node_indices,
                &refinement_nodes,
            );

            subtree_refinement_node_indices.count = 0;
            subtree_refinement_leaves.count = 0;
        }

        subtree_refinement_node_indices.dispose(pool);
        subtree_refinement_leaves.dispose(pool);
        subtree_refinement_targets.dispose(pool);
        let mut subtree_stack_buffer = subtree_stack_buffer;
        pool.return_buffer(&mut subtree_stack_buffer);
        let mut refinement_nodes_allocation = refinement_nodes_allocation;
        pool.return_buffer(&mut refinement_nodes_allocation);
    }
}
