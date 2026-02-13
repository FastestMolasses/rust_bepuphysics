// Translated from BepuPhysics/Trees/Tree_MultithreadedRefitRefine.cs

use super::node::{Metanode, Node, NodeChild};
use super::tree::Tree;
use super::tree_binned_refine::BinnedResources;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use glam::Vec3;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

/// Caches input and output for the multithreaded execution of a tree's refit and refinement operations.
pub struct RefitAndRefineMultithreadedContext {
    tree: *mut Tree,

    refit_node_index: UnsafeCell<i32>,
    pub refit_nodes: QuickList<i32>,
    refit_cost_change: UnsafeCell<f32>,

    refinement_leaf_count_threshold: i32,
    refinement_candidates: Buffer<QuickList<i32>>,

    refine_index: UnsafeCell<i32>,
    pub refinement_targets: QuickList<i32>,
    pub maximum_subtrees: i32,

    thread_count: i32,
    worker_pools_ptr: *const crate::utilities::memory::worker_buffer_pools::WorkerBufferPools,
}

unsafe impl Send for RefitAndRefineMultithreadedContext {}
unsafe impl Sync for RefitAndRefineMultithreadedContext {}

impl RefitAndRefineMultithreadedContext {
    pub fn new() -> Self {
        Self {
            tree: std::ptr::null_mut(),
            refit_node_index: UnsafeCell::new(-1),
            refit_nodes: QuickList::default(),
            refit_cost_change: UnsafeCell::new(0.0),
            refinement_leaf_count_threshold: 0,
            refinement_candidates: Buffer::default(),
            refine_index: UnsafeCell::new(-1),
            refinement_targets: QuickList::default(),
            maximum_subtrees: 0,
            thread_count: 0,
            worker_pools_ptr: std::ptr::null(),
        }
    }

    pub unsafe fn create_refit_and_mark_jobs(
        &mut self,
        tree: &mut Tree,
        pool: &mut BufferPool,
        thread_dispatcher: &dyn IThreadDispatcher,
    ) {
        if tree.leaf_count <= 2 {
            self.refit_nodes = QuickList::default();
            return;
        }
        self.thread_count = thread_dispatcher.thread_count();
        self.worker_pools_ptr = thread_dispatcher.worker_pools() as *const _;
        self.tree = tree as *mut Tree;

        self.refinement_candidates = pool.take_at_least::<QuickList<i32>>(self.thread_count);
        let (
            maximum_subtrees,
            estimated_refinement_candidate_count,
            refinement_leaf_count_threshold,
        ) = tree.get_refit_and_mark_tuning();
        self.maximum_subtrees = maximum_subtrees;
        self.refinement_leaf_count_threshold = refinement_leaf_count_threshold;
        self.refit_nodes = QuickList::with_capacity(maximum_subtrees, pool);

        for i in 0..self.thread_count {
            let worker_pool = thread_dispatcher.worker_pool_ptr(i);
            *self.refinement_candidates.get_mut(i) =
                QuickList::with_capacity(estimated_refinement_candidate_count, &mut *worker_pool);
        }

        let mut multithreading_leaf_count_threshold = tree.leaf_count / (self.thread_count * 2);
        if multithreading_leaf_count_threshold < refinement_leaf_count_threshold {
            multithreading_leaf_count_threshold = refinement_leaf_count_threshold;
        }

        let worker_pool_0 = thread_dispatcher.worker_pool_ptr(0);
        self.collect_nodes_for_multithreaded_refit(
            0,
            multithreading_leaf_count_threshold,
            refinement_leaf_count_threshold,
            pool,
            &mut *worker_pool_0,
        );

        unsafe {
            *self.refit_node_index.get() = -1;
        }
    }

    pub unsafe fn create_refinement_jobs(
        &mut self,
        pool: &mut BufferPool,
        frame_index: i32,
        refine_aggressiveness_scale: f32,
    ) {
        let tree = &mut *self.tree;
        if tree.leaf_count <= 2 {
            self.refinement_targets = QuickList::default();
            return;
        }

        let mut refinement_candidates_count = 0;
        for i in 0..self.thread_count {
            refinement_candidates_count += self.refinement_candidates.get(i).count;
        }

        let (target_refinement_count, period, offset) = tree.get_refine_tuning(
            frame_index,
            refinement_candidates_count,
            refine_aggressiveness_scale,
            unsafe { *self.refit_cost_change.get() },
        );

        self.refinement_targets = QuickList::with_capacity(target_refinement_count, pool);

        let mut current_candidates_index = 0i32;
        let mut index = offset;
        for _ in 0..target_refinement_count - 1 {
            index += period;
            while index
                >= self
                    .refinement_candidates
                    .get(current_candidates_index)
                    .count
            {
                index -= self
                    .refinement_candidates
                    .get(current_candidates_index)
                    .count;
                current_candidates_index += 1;
                if current_candidates_index >= self.thread_count {
                    current_candidates_index -= self.thread_count;
                }
            }
            debug_assert!(
                index
                    < self
                        .refinement_candidates
                        .get(current_candidates_index)
                        .count
                    && index >= 0
            );
            let node_index = self.refinement_candidates.get(current_candidates_index)[index];
            debug_assert!(
                tree.metanodes.get(node_index).cost_or_flag.refine_flag == 0,
                "Refinement target search shouldn't run into the same node twice!"
            );
            self.refinement_targets.add_unsafely(node_index);
            (*((tree.metanodes.as_ptr() as *mut Metanode).add(node_index as usize)))
                .cost_or_flag
                .refine_flag = 1;
        }

        if tree.metanodes.get(0).cost_or_flag.refine_flag != 1 {
            self.refinement_targets.add_unsafely(0);
            (*((tree.metanodes.as_ptr() as *mut Metanode).add(0)))
                .cost_or_flag
                .refine_flag = 1;
        }

        unsafe {
            *self.refine_index.get() = -1;
        }
    }

    pub unsafe fn clean_up_for_refit_and_refine(&mut self, pool: &mut BufferPool) {
        let tree = &mut *self.tree;
        if tree.leaf_count <= 2 {
            return;
        }

        for i in 0..self.refinement_targets.count {
            tree.metanodes
                .get_mut(self.refinement_targets[i])
                .cost_or_flag
                .refine_flag = 0;
        }

        let worker_pools = &*self.worker_pools_ptr;
        for i in 0..self.thread_count {
            let worker_pool = worker_pools.get_pool_ptr(i as usize);
            self.refinement_candidates
                .get_mut(i)
                .dispose(&mut *worker_pool);
        }
        pool.return_buffer(&mut self.refinement_candidates);
        self.refit_nodes.dispose(pool);
        self.refinement_targets.dispose(pool);
        self.tree = std::ptr::null_mut();
        self.worker_pools_ptr = std::ptr::null();
    }

    pub unsafe fn refit_and_refine(
        &mut self,
        tree: &mut Tree,
        pool: &mut BufferPool,
        thread_dispatcher: &dyn IThreadDispatcher,
        frame_index: i32,
        refine_aggressiveness_scale: f32,
    ) {
        self.create_refit_and_mark_jobs(tree, pool, thread_dispatcher);
        let refit_count = self.refit_nodes.count;
        // Phase 1: Refit and mark.
        let ctx_ptr = self as *mut Self;
        thread_dispatcher.dispatch_workers(
            refit_and_mark_worker,
            refit_count,
            ctx_ptr as *mut (),
            None,
        );
        self.create_refinement_jobs(pool, frame_index, refine_aggressiveness_scale);
        let refine_count = self.refinement_targets.count;
        // Phase 2: Refine.
        thread_dispatcher.dispatch_workers(refine_worker, refine_count, ctx_ptr as *mut (), None);
        self.clean_up_for_refit_and_refine(pool);
    }

    unsafe fn collect_nodes_for_multithreaded_refit(
        &mut self,
        node_index: i32,
        multithreading_leaf_count_threshold: i32,
        refinement_leaf_count_threshold: i32,
        pool: &mut BufferPool,
        thread_pool: &mut BufferPool,
    ) {
        let tree = &*self.tree;
        let node = &*tree.nodes.get(node_index);
        debug_assert!(tree.metanodes.get(node_index).cost_or_flag.refine_flag == 0);
        debug_assert!(tree.leaf_count > 2);

        let children = &node.a as *const NodeChild;
        for i in 0..2 {
            let child = &*children.add(i);
            if child.index >= 0 {
                // Increment refine_flag to count pending child work.
                let metanode =
                    &mut *(tree.metanodes.as_ptr() as *mut Metanode).add((node_index) as usize);
                metanode.cost_or_flag.refine_flag += 1;

                if child.leaf_count <= multithreading_leaf_count_threshold {
                    if child.leaf_count <= refinement_leaf_count_threshold {
                        self.refinement_candidates
                            .get_mut(0)
                            .add(child.index, thread_pool);
                        self.refit_nodes.add(Tree::encode(child.index), pool);
                    } else {
                        self.refit_nodes.add(child.index, pool);
                    }
                } else {
                    self.collect_nodes_for_multithreaded_refit(
                        child.index,
                        multithreading_leaf_count_threshold,
                        refinement_leaf_count_threshold,
                        pool,
                        thread_pool,
                    );
                }
            }
        }
    }

    pub unsafe fn execute_refit_and_mark_job(
        &self,
        thread_pool: &mut BufferPool,
        worker_index: i32,
        refit_index: i32,
    ) {
        let tree = &*self.tree;
        let mut node_index = self.refit_nodes[refit_index];
        let should_use_mark;
        if node_index < 0 {
            node_index = Tree::encode(node_index);
            should_use_mark = false;
        } else {
            should_use_mark = true;
        }

        let node = &mut *(tree.nodes.as_ptr() as *mut Node).add(node_index as usize);
        let metanode = &mut *(tree.metanodes.as_ptr() as *mut Metanode).add((node_index) as usize);
        debug_assert!(
            metanode.parent >= 0,
            "The root should not be marked for refit."
        );

        let parent = &mut *(tree.nodes.as_ptr() as *mut Node).add(metanode.parent as usize);
        let child_in_parent = Tree::node_child_mut(parent, metanode.index_in_parent);

        let cost_change = if should_use_mark {
            tree.refit_and_mark(
                child_in_parent,
                self.refinement_leaf_count_threshold,
                &mut *((self.refinement_candidates.as_ptr() as *mut QuickList<i32>)
                    .add(worker_index as usize)),
                thread_pool,
            )
        } else {
            tree.refit_and_measure(child_in_parent)
        };
        metanode.cost_or_flag.local_cost_change = cost_change;

        // Walk up the tree, atomically decrementing refine_flag.
        let mut current_node = parent;
        let mut current_metanode =
            &mut *(tree.metanodes.as_ptr() as *mut Metanode).add((metanode.parent) as usize);
        loop {
            let refine_flag_ptr = &mut current_metanode.cost_or_flag.refine_flag as *mut i32;
            let prev = AtomicI32::from_ptr(refine_flag_ptr).fetch_sub(1, Ordering::AcqRel);
            if prev - 1 == 0 {
                // This thread is the last to finish for this node. Merge children bounds up.
                let children = &current_node.a as *const NodeChild;
                current_metanode.cost_or_flag.local_cost_change = 0.0;
                for j in 0..2 {
                    let child = &*children.add(j);
                    if child.index >= 0 {
                        let child_metanode = &mut *(tree.metanodes.as_ptr() as *mut Metanode)
                            .add((child.index) as usize);
                        current_metanode.cost_or_flag.local_cost_change +=
                            child_metanode.cost_or_flag.local_cost_change;
                        child_metanode.cost_or_flag.refine_flag = 0;
                    }
                }

                if current_metanode.parent < 0 {
                    // We reached the root.
                    let mut merged = BoundingBox {
                        min: Vec3::splat(f32::MAX),
                        _pad0: 0.0,
                        max: Vec3::splat(f32::MIN),
                        _pad1: 0.0,
                    };
                    for j in 0..2 {
                        let child = &*children.add(j);
                        BoundingBox::create_merged(
                            child.min,
                            child.max,
                            merged.min,
                            merged.max,
                            &mut merged.min,
                            &mut merged.max,
                        );
                    }
                    let postmetric = Tree::compute_bounds_metric_bb(&merged);
                    if postmetric > 1e-9 {
                        self.refit_cost_change_store(
                            current_metanode.cost_or_flag.local_cost_change / postmetric,
                        );
                    } else {
                        self.refit_cost_change_store(0.0);
                    }
                    current_metanode.cost_or_flag.refine_flag = 0;
                    break;
                } else {
                    let parent_node = &mut *(tree.nodes.as_ptr() as *mut Node)
                        .add(current_metanode.parent as usize);
                    let child_in_parent =
                        Tree::node_child_mut(parent_node, current_metanode.index_in_parent);
                    let premetric = Tree::compute_bounds_metric_vecs(
                        &child_in_parent.min,
                        &child_in_parent.max,
                    );
                    child_in_parent.min = Vec3::splat(f32::MAX);
                    child_in_parent.max = Vec3::splat(f32::MIN);
                    for j in 0..2 {
                        let child = &*children.add(j);
                        BoundingBox::create_merged(
                            child.min,
                            child.max,
                            child_in_parent.min,
                            child_in_parent.max,
                            &mut child_in_parent.min,
                            &mut child_in_parent.max,
                        );
                    }
                    let postmetric = Tree::compute_bounds_metric_vecs(
                        &child_in_parent.min,
                        &child_in_parent.max,
                    );
                    current_metanode.cost_or_flag.local_cost_change += postmetric - premetric;

                    current_node = parent_node;
                    current_metanode = &mut *(tree.metanodes.as_ptr() as *mut Metanode)
                        .add((current_metanode.parent) as usize);
                }
            } else {
                break;
            }
        }
    }

    /// Stores the refit cost change. Uses a raw pointer write since this is written
    /// by exactly one thread (the last one to reach the root).
    unsafe fn refit_cost_change_store(&self, value: f32) {
        *self.refit_cost_change.get() = value;
    }

    pub unsafe fn refit_and_mark_for_worker(
        &self,
        worker_index: i32,
        thread_dispatcher: &dyn IThreadDispatcher,
    ) {
        if self.refit_nodes.count == 0 {
            return;
        }
        let thread_pool = thread_dispatcher.worker_pool_ptr(worker_index);
        debug_assert!((*self.tree).leaf_count > 2);
        loop {
            let refit_index = unsafe {
                AtomicI32::from_ptr(self.refit_node_index.get()).fetch_add(1, Ordering::AcqRel)
            } + 1;
            if refit_index >= self.refit_nodes.count {
                break;
            }
            self.execute_refit_and_mark_job(&mut *thread_pool, worker_index, refit_index);
        }
    }

    pub unsafe fn execute_refine_job(
        &self,
        subtree_references: &mut QuickList<i32>,
        treelet_internal_nodes: &mut QuickList<i32>,
        resources: &mut BinnedResources,
        thread_pool: &mut BufferPool,
        refine_index: i32,
    ) {
        let tree = &mut *self.tree;
        tree.binned_refine(
            self.refinement_targets[refine_index],
            subtree_references,
            self.maximum_subtrees,
            treelet_internal_nodes,
            resources,
            thread_pool,
        );
        subtree_references.count = 0;
        treelet_internal_nodes.count = 0;
    }

    pub unsafe fn refine_for_worker(
        &self,
        worker_index: i32,
        thread_dispatcher: &dyn IThreadDispatcher,
    ) {
        if self.refinement_targets.count == 0 {
            return;
        }
        let thread_pool = thread_dispatcher.worker_pool_ptr(worker_index);

        let subtree_count_estimate = (self.maximum_subtrees as u32).next_power_of_two() as i32;
        let mut subtree_references =
            QuickList::<i32>::with_capacity(subtree_count_estimate, &mut *thread_pool);
        let mut treelet_internal_nodes =
            QuickList::<i32>::with_capacity(subtree_count_estimate, &mut *thread_pool);

        let tree = &mut *self.tree;
        let (mut buffer, mut resources) =
            tree.create_binned_resources(&mut *thread_pool, self.maximum_subtrees);

        loop {
            let refine_index = unsafe {
                AtomicI32::from_ptr(self.refine_index.get()).fetch_add(1, Ordering::AcqRel)
            } + 1;
            if refine_index >= self.refinement_targets.count {
                break;
            }
            self.execute_refine_job(
                &mut subtree_references,
                &mut treelet_internal_nodes,
                &mut resources,
                &mut *thread_pool,
                refine_index,
            );
        }

        subtree_references.dispose(&mut *thread_pool);
        treelet_internal_nodes.dispose(&mut *thread_pool);
        (&mut *thread_pool).return_buffer(&mut buffer);
    }
}

/// Worker function for refit-and-mark phase.
fn refit_and_mark_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
    unsafe {
        let ctx = &*(dispatcher.unmanaged_context() as *const RefitAndRefineMultithreadedContext);
        ctx.refit_and_mark_for_worker(worker_index, dispatcher);
    }
}

/// Worker function for refine phase.
fn refine_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
    unsafe {
        let ctx = &*(dispatcher.unmanaged_context() as *const RefitAndRefineMultithreadedContext);
        ctx.refine_for_worker(worker_index, dispatcher);
    }
}
