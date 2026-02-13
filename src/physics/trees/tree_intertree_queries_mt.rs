// Translated from BepuPhysics/Trees/Tree_IntertreeQueries_Multithreaded.cs

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

/// Caches state for a multithreaded inter-tree overlap test.
pub struct MultithreadedIntertreeTest<TOverlapHandler: IOverlapHandler> {
    pub pool: *mut BufferPool,
    next_node_pair: UnsafeCell<i32>,
    leaf_threshold: i32,
    jobs: QuickList<Job>,
    pub tree_a: *const Tree,
    pub tree_b: *const Tree,
    pub overlap_handlers: Vec<TOverlapHandler>,
}

unsafe impl<TOverlapHandler: IOverlapHandler> Send for MultithreadedIntertreeTest<TOverlapHandler> {}
unsafe impl<TOverlapHandler: IOverlapHandler> Sync for MultithreadedIntertreeTest<TOverlapHandler> {}

impl<TOverlapHandler: IOverlapHandler> MultithreadedIntertreeTest<TOverlapHandler> {
    pub fn new(pool: *mut BufferPool) -> Self {
        Self {
            pool,
            next_node_pair: UnsafeCell::new(-1),
            leaf_threshold: 0,
            jobs: QuickList::default(),
            tree_a: std::ptr::null(),
            tree_b: std::ptr::null(),
            overlap_handlers: Vec::new(),
        }
    }

    pub fn job_count(&self) -> i32 {
        self.jobs.count
    }

    /// Prepares the jobs associated with an inter-tree test.
    pub unsafe fn prepare_jobs(
        &mut self,
        tree_a: &Tree,
        tree_b: &Tree,
        overlap_handlers: Vec<TOverlapHandler>,
        thread_count: i32,
    ) {
        if tree_a.leaf_count == 0 || tree_b.leaf_count == 0 {
            self.jobs = QuickList::default();
            return;
        }
        debug_assert!(overlap_handlers.len() >= thread_count as usize);
        let job_multiplier = 8.0f32;
        let target_job_count = (1.0f32).max(job_multiplier * thread_count as f32);
        self.leaf_threshold =
            ((tree_a.leaf_count + tree_b.leaf_count) as f32 / target_job_count) as i32;
        self.jobs = QuickList::with_capacity((target_job_count * 2.0) as i32, &mut *self.pool);
        unsafe {
            *self.next_node_pair.get() = -1;
        }
        self.overlap_handlers = overlap_handlers;
        self.tree_a = tree_a as *const Tree;
        self.tree_b = tree_b as *const Tree;

        // Collect jobs.
        if tree_a.leaf_count >= 2 && tree_b.leaf_count >= 2 {
            self.get_jobs_between_different_nodes(&*tree_a.nodes.get(0), &*tree_b.nodes.get(0));
        } else if tree_a.leaf_count == 1 && tree_b.leaf_count >= 2 {
            let a = &*tree_a.nodes.get(0);
            let b = &*tree_b.nodes.get(0);
            if BoundingBox::intersects_unsafe(&a.a, &b.a) {
                self.dispatch_test_for_nodes(&a.a, &b.a);
            }
            if BoundingBox::intersects_unsafe(&a.a, &b.b) {
                self.dispatch_test_for_nodes(&a.a, &b.b);
            }
        } else if tree_a.leaf_count >= 2 && tree_b.leaf_count == 1 {
            let a = &*tree_a.nodes.get(0);
            let b = &*tree_b.nodes.get(0);
            if BoundingBox::intersects_unsafe(&a.a, &b.a) {
                self.dispatch_test_for_nodes(&a.a, &b.a);
            }
            if BoundingBox::intersects_unsafe(&a.b, &b.a) {
                self.dispatch_test_for_nodes(&a.b, &b.a);
            }
        } else {
            debug_assert!(tree_a.leaf_count == 1 && tree_b.leaf_count == 1);
            if BoundingBox::intersects_unsafe(&(*tree_a.nodes.get(0)).a, &(*tree_b.nodes.get(0)).a)
            {
                self.dispatch_test_for_nodes(&(*tree_a.nodes.get(0)).a, &(*tree_b.nodes.get(0)).a);
            }
        }
    }

    /// Cleans up after a multithreaded inter-tree test.
    pub unsafe fn complete_test(&mut self) {
        if self.jobs.span.allocated() {
            self.jobs.dispose(&mut *self.pool);
        }
    }

    pub unsafe fn execute_job(&mut self, job_index: i32, worker_index: i32) {
        let tree_a = &*self.tree_a;
        let tree_b = &*self.tree_b;
        let overlap = *self.jobs.get(job_index);
        if overlap.a >= 0 {
            if overlap.b >= 0 {
                // Different internal nodes.
                tree_a.get_overlaps_between_different_nodes_intertree(
                    &*tree_a.nodes.get(overlap.a),
                    &*tree_b.nodes.get(overlap.b),
                    tree_b,
                    &mut self.overlap_handlers[worker_index as usize],
                );
            } else {
                // A is an internal node, B is a leaf.
                let leaf_index = Tree::encode(overlap.b);
                let leaf = &*tree_b.leaves.get(leaf_index);
                let node = &*tree_b.nodes.get(leaf.node_index());
                let child_owning_leaf = if leaf.child_index() == 0 {
                    &node.a
                } else {
                    &node.b
                };
                tree_a.test_node_against_leaf(
                    overlap.a,
                    leaf_index,
                    child_owning_leaf,
                    &mut self.overlap_handlers[worker_index as usize],
                );
            }
        } else {
            // A is a leaf, B is internal.
            let leaf_index = Tree::encode(overlap.a);
            let leaf = &*tree_a.leaves.get(leaf_index);
            let node = &*tree_a.nodes.get(leaf.node_index());
            let child_owning_leaf = if leaf.child_index() == 0 {
                &node.a
            } else {
                &node.b
            };
            Tree::test_leaf_against_node_intertree(
                leaf_index,
                child_owning_leaf,
                overlap.b,
                tree_b,
                &mut self.overlap_handlers[worker_index as usize],
            );
        }
    }

    /// Executes a single worker of the multithreaded inter-tree test.
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
        node_owner_is_a: bool,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        node_leaf_count: i32,
    ) {
        if node_index < 0 {
            // Maintain the order of trees. Leaves from tree A should always be the first parameter.
            if node_owner_is_a {
                self.overlap_handlers[0].handle(Tree::encode(node_index), leaf_index);
            } else {
                self.overlap_handlers[0].handle(leaf_index, Tree::encode(node_index));
            }
        } else {
            if node_leaf_count <= self.leaf_threshold {
                if node_owner_is_a {
                    self.jobs.add(
                        Job {
                            b: Tree::encode(leaf_index),
                            a: node_index,
                        },
                        &mut *self.pool,
                    );
                } else {
                    self.jobs.add(
                        Job {
                            a: Tree::encode(leaf_index),
                            b: node_index,
                        },
                        &mut *self.pool,
                    );
                }
            } else {
                let node_owner = if node_owner_is_a {
                    &*self.tree_a
                } else {
                    &*self.tree_b
                };
                let node = &*node_owner.nodes.get(node_index);
                let b_index = node.b.index;
                let b_leaf_count = node.b.leaf_count;
                let a_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.a);
                let b_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.b);
                if a_intersects {
                    self.dispatch_test_for_leaf(
                        node_owner_is_a,
                        leaf_index,
                        leaf_child,
                        node.a.index,
                        node.a.leaf_count,
                    );
                }
                if b_intersects {
                    self.dispatch_test_for_leaf(
                        node_owner_is_a,
                        leaf_index,
                        leaf_child,
                        b_index,
                        b_leaf_count,
                    );
                }
            }
        }
    }

    unsafe fn dispatch_test_for_nodes(&mut self, a: &NodeChild, b: &NodeChild) {
        let tree_a = &*self.tree_a;
        let tree_b = &*self.tree_b;
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
                        &*tree_a.nodes.get(a.index),
                        &*tree_b.nodes.get(b.index),
                    );
                }
            } else {
                // leaf B versus node A.
                let leaf_index = Tree::encode(b.index);
                let leaf = &*tree_b.leaves.get(leaf_index);
                let lnode = &*tree_b.nodes.get(leaf.node_index());
                let child_owning = if leaf.child_index() == 0 {
                    &lnode.a
                } else {
                    &lnode.b
                };
                // node A is in tree_a
                self.dispatch_test_for_leaf(true, leaf_index, child_owning, a.index, a.leaf_count);
            }
        } else if b.index >= 0 {
            // leaf A versus node B.
            let leaf_index = Tree::encode(a.index);
            let leaf = &*tree_a.leaves.get(leaf_index);
            let lnode = &*tree_a.nodes.get(leaf.node_index());
            let child_owning = if leaf.child_index() == 0 {
                &lnode.a
            } else {
                &lnode.b
            };
            // node B is in tree_b
            self.dispatch_test_for_leaf(false, leaf_index, child_owning, b.index, b.leaf_count);
        } else {
            // Two leaves.
            self.overlap_handlers[0].handle(Tree::encode(a.index), Tree::encode(b.index));
        }
    }

    unsafe fn get_jobs_between_different_nodes(&mut self, a: &Node, b: &Node) {
        let aa_intersects = BoundingBox::intersects_unsafe(&a.a, &b.a);
        let ab_intersects = BoundingBox::intersects_unsafe(&a.a, &b.b);
        let ba_intersects = BoundingBox::intersects_unsafe(&a.b, &b.a);
        let bb_intersects = BoundingBox::intersects_unsafe(&a.b, &b.b);

        if aa_intersects {
            self.dispatch_test_for_nodes(&a.a, &b.a);
        }
        if ab_intersects {
            self.dispatch_test_for_nodes(&a.a, &b.b);
        }
        if ba_intersects {
            self.dispatch_test_for_nodes(&a.b, &b.a);
        }
        if bb_intersects {
            self.dispatch_test_for_nodes(&a.b, &b.b);
        }
    }
}
