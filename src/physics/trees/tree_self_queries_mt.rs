// Translated from BepuPhysics/Trees/Tree_SelfQueries_MultithreadedIntertree.cs

use crate::physics::trees::node::{Node, NodeChild};
use crate::physics::trees::tree::Tree;
use crate::physics::trees::tree_self_queries::IOverlapHandler;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

#[repr(C)]
#[derive(Clone, Copy)]
struct Job {
    a: i32,
    b: i32,
}

/// Caches state for a multithreaded self-overlap test on a single tree.
pub struct MultithreadedSelfTest<TOverlapHandler: IOverlapHandler> {
    pub pool: *mut BufferPool,
    next_node_pair: UnsafeCell<i32>,
    leaf_threshold: i32,
    jobs: QuickList<Job>,
    pub tree: *const Tree,
    pub overlap_handlers: Vec<TOverlapHandler>,
}

unsafe impl<TOverlapHandler: IOverlapHandler> Send for MultithreadedSelfTest<TOverlapHandler> {}
unsafe impl<TOverlapHandler: IOverlapHandler> Sync for MultithreadedSelfTest<TOverlapHandler> {}

impl<TOverlapHandler: IOverlapHandler> MultithreadedSelfTest<TOverlapHandler> {
    pub fn new(pool: *mut BufferPool) -> Self {
        Self {
            pool,
            next_node_pair: UnsafeCell::new(-1),
            leaf_threshold: 0,
            jobs: QuickList::default(),
            tree: std::ptr::null(),
            overlap_handlers: Vec::new(),
        }
    }

    pub fn job_count(&self) -> i32 {
        self.jobs.count
    }

    /// Prepares the jobs associated with a self test. Must be called before a dispatch over `pair_test`.
    pub unsafe fn prepare_jobs(
        &mut self,
        tree: &Tree,
        overlap_handlers: Vec<TOverlapHandler>,
        thread_count: i32,
    ) {
        if tree.leaf_count < 2 {
            self.jobs = QuickList::default();
            return;
        }
        debug_assert!(overlap_handlers.len() >= thread_count as usize);
        let job_multiplier = 8.0f32;
        let target_job_count = (1.0f32).max(job_multiplier * thread_count as f32);
        self.leaf_threshold = (tree.leaf_count as f32 / target_job_count) as i32;
        self.jobs = QuickList::with_capacity((target_job_count * 2.0) as i32, &mut *self.pool);
        unsafe {
            *self.next_node_pair.get() = -1;
        }
        self.overlap_handlers = overlap_handlers;
        self.tree = tree as *const Tree;
        // Collect jobs.
        self.collect_jobs_in_node(0, tree.leaf_count);
    }

    /// Cleans up after a multithreaded self test.
    pub unsafe fn complete_self_test(&mut self) {
        if self.jobs.span.allocated() {
            self.jobs.dispose(&mut *self.pool);
        }
    }

    pub unsafe fn execute_job(&mut self, job_index: i32, worker_index: i32) {
        let tree = &*self.tree;
        let overlap = *self.jobs.get(job_index);
        if overlap.a >= 0 {
            if overlap.a == overlap.b {
                // Same node.
                tree.get_overlaps_in_node(
                    &*tree.nodes.get(overlap.a),
                    &mut self.overlap_handlers[worker_index as usize],
                );
            } else if overlap.b >= 0 {
                // Different nodes.
                tree.get_overlaps_between_different_nodes(
                    &*tree.nodes.get(overlap.a),
                    &*tree.nodes.get(overlap.b),
                    &mut self.overlap_handlers[worker_index as usize],
                );
            } else {
                // A is an internal node, B is a leaf.
                let leaf_index = Tree::encode(overlap.b);
                let leaf = &*tree.leaves.get(leaf_index);
                let node = &*tree.nodes.get(leaf.node_index());
                let child_owning_leaf = if leaf.child_index() == 0 {
                    &node.a
                } else {
                    &node.b
                };
                tree.test_leaf_against_node(
                    leaf_index,
                    child_owning_leaf,
                    overlap.a,
                    &mut self.overlap_handlers[worker_index as usize],
                );
            }
        } else {
            // A is a leaf, B is internal.
            let leaf_index = Tree::encode(overlap.a);
            let leaf = &*tree.leaves.get(leaf_index);
            let node = &*tree.nodes.get(leaf.node_index());
            let child_owning_leaf = if leaf.child_index() == 0 {
                &node.a
            } else {
                &node.b
            };
            tree.test_leaf_against_node(
                leaf_index,
                child_owning_leaf,
                overlap.b,
                &mut self.overlap_handlers[worker_index as usize],
            );
        }
    }

    /// Executes a single worker of the multithreaded self test.
    pub unsafe fn pair_test(&mut self, worker_index: i32) {
        debug_assert!(worker_index >= 0 && (worker_index as usize) < self.overlap_handlers.len());
        loop {
            let next_node_pair_index = unsafe {
                AtomicI32::from_ptr(self.next_node_pair.get()).fetch_add(1, Ordering::AcqRel)
            } + 1;
            if next_node_pair_index >= self.jobs.count {
                break;
            }
            self.execute_job(next_node_pair_index, worker_index);
        }
    }

    unsafe fn dispatch_test_for_leaf(
        &mut self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        node_leaf_count: i32,
        results: &mut TOverlapHandler,
    ) {
        if node_index < 0 {
            results.handle(leaf_index, Tree::encode(node_index));
        } else {
            if node_leaf_count <= self.leaf_threshold {
                self.jobs.add(
                    Job {
                        a: Tree::encode(leaf_index),
                        b: node_index,
                    },
                    &mut *self.pool,
                );
            } else {
                let tree = &*self.tree;
                tree.test_leaf_against_node(leaf_index, leaf_child, node_index, results);
            }
        }
    }

    unsafe fn dispatch_test_for_nodes(
        &mut self,
        a: &NodeChild,
        b: &NodeChild,
        results: &mut TOverlapHandler,
    ) {
        let tree = &*self.tree;
        if a.index >= 0 {
            if b.index >= 0 {
                if a.leaf_count + b.leaf_count <= self.leaf_threshold {
                    self.jobs.add(
                        Job {
                            a: a.index,
                            b: b.index,
                        },
                        &mut *self.pool,
                    );
                } else {
                    self.get_jobs_between_different_nodes(
                        &*tree.nodes.get(a.index),
                        &*tree.nodes.get(b.index),
                        results,
                    );
                }
            } else {
                // leaf B versus node A
                let leaf_index = Tree::encode(b.index);
                let leaf = &*tree.leaves.get(leaf_index);
                let lnode = &*tree.nodes.get(leaf.node_index());
                let child_owning = if leaf.child_index() == 0 {
                    &lnode.a
                } else {
                    &lnode.b
                };
                self.dispatch_test_for_leaf(
                    leaf_index,
                    child_owning,
                    a.index,
                    a.leaf_count,
                    results,
                );
            }
        } else if b.index >= 0 {
            // leaf A versus node B
            let leaf_index = Tree::encode(a.index);
            let leaf = &*tree.leaves.get(leaf_index);
            let lnode = &*tree.nodes.get(leaf.node_index());
            let child_owning = if leaf.child_index() == 0 {
                &lnode.a
            } else {
                &lnode.b
            };
            self.dispatch_test_for_leaf(leaf_index, child_owning, b.index, b.leaf_count, results);
        } else {
            // Two leaves.
            results.handle(Tree::encode(a.index), Tree::encode(b.index));
        }
    }

    unsafe fn get_jobs_between_different_nodes(
        &mut self,
        a: &Node,
        b: &Node,
        results: &mut TOverlapHandler,
    ) {
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

    unsafe fn collect_jobs_in_node(&mut self, node_index: i32, leaf_count: i32) {
        if leaf_count <= self.leaf_threshold {
            self.jobs.add(
                Job {
                    a: node_index,
                    b: node_index,
                },
                &mut *self.pool,
            );
            return;
        }

        let tree = &*self.tree;
        let node = &*tree.nodes.get(node_index);

        let ab = BoundingBox::intersects_unsafe(&node.a, &node.b);

        if node.a.index >= 0 {
            self.collect_jobs_in_node(node.a.index, node.a.leaf_count);
        }
        if node.b.index >= 0 {
            self.collect_jobs_in_node(node.b.index, node.b.leaf_count);
        }

        if ab {
            // Need to pass &mut TOverlapHandler but we only have &mut self which contains it.
            // Use the first handler for collection phase.
            let handler_ptr = &mut self.overlap_handlers[0] as *mut TOverlapHandler;
            self.dispatch_test_for_nodes(&node.a, &node.b, &mut *handler_ptr);
        }
    }
}
