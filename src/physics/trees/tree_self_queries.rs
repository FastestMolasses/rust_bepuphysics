// Translated from BepuPhysics/Trees/Tree_SelfQueries.cs (core recursive overlap testing)

use super::node::{Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::{ContinuationHandle, Task, TaskStack};
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::ffi::c_void;

/// Overlap callback for tree overlap queries.
pub trait IOverlapHandler {
    /// Handles an overlap between leaves.
    fn handle(&mut self, index_a: i32, index_b: i32);
}

/// Overlap callback for tree overlap queries. Used in multithreaded contexts.
pub trait IThreadedOverlapHandler {
    /// Handles an overlap between leaves.
    fn handle(
        &mut self,
        index_a: i32,
        index_b: i32,
        worker_index: i32,
        managed_context: Option<&dyn std::any::Any>,
    );
}

impl Tree {
    fn dispatch_test_for_leaf<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        results: &mut TOverlapHandler,
    ) {
        if node_index < 0 {
            results.handle(leaf_index, Self::encode(node_index));
        } else {
            self.test_leaf_against_node(leaf_index, leaf_child, node_index, results);
        }
    }

    pub(crate) fn test_leaf_against_node<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_index: i32,
        leaf_child: &NodeChild,
        node_index: i32,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let node = self.nodes.get(node_index);
            let b_index = node.b.index;
            let a_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.a);
            let b_intersects = BoundingBox::intersects_unsafe(leaf_child, &node.b);
            if a_intersects {
                self.dispatch_test_for_leaf(leaf_index, leaf_child, node.a.index, results);
            }
            if b_intersects {
                self.dispatch_test_for_leaf(leaf_index, leaf_child, b_index, results);
            }
        }
    }

    #[inline(always)]
    fn dispatch_test_for_nodes<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &NodeChild,
        b: &NodeChild,
        results: &mut TOverlapHandler,
    ) {
        if a.index >= 0 {
            if b.index >= 0 {
                self.get_overlaps_between_different_nodes(
                    self.nodes.get(a.index),
                    self.nodes.get(b.index),
                    results,
                );
            } else {
                // leaf B versus node A.
                self.test_leaf_against_node(Self::encode(b.index), b, a.index, results);
            }
        } else if b.index >= 0 {
            // leaf A versus node B.
            self.test_leaf_against_node(Self::encode(a.index), a, b.index, results);
        } else {
            // Two leaves.
            results.handle(Self::encode(a.index), Self::encode(b.index));
        }
    }

    pub(crate) fn get_overlaps_between_different_nodes<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &Node,
        b: &Node,
        results: &mut TOverlapHandler,
    ) {
        // There are no shared children, so test them all.
        unsafe {
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
    }

    pub(crate) fn get_overlaps_in_node<TOverlapHandler: IOverlapHandler>(
        &self,
        node: &Node,
        results: &mut TOverlapHandler,
    ) {
        unsafe {
            let ab = BoundingBox::intersects_unsafe(&node.a, &node.b);
            if node.a.index >= 0 {
                self.get_overlaps_in_node(self.nodes.get(node.a.index), results);
            }
            if node.b.index >= 0 {
                self.get_overlaps_in_node(self.nodes.get(node.b.index), results);
            }
            // Test all different nodes.
            if ab {
                self.dispatch_test_for_nodes(&node.a, &node.b, results);
            }
        }
    }

    /// Gets pairs of leaf indices with bounding boxes which overlap within this tree.
    pub fn get_self_overlaps<TOverlapHandler: IOverlapHandler>(
        &self,
        results: &mut TOverlapHandler,
    ) {
        // If there are less than two leaves, there can't be any overlap.
        if self.leaf_count < 2 {
            return;
        }
        self.get_overlaps_in_node(self.nodes.get(0), results);
    }

    fn add_crossover_result<TOverlapHandler: IOverlapHandler>(
        &self,
        a: &NodeChild,
        b: &NodeChild,
        crossovers: &mut QuickList<IndexPair>,
        node_leaf: &mut QuickList<NodeLeafPair>,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
    ) {
        if a.index >= 0 && b.index >= 0 {
            *crossovers.allocate(pool) = IndexPair { a: a.index, b: b.index };
        } else if a.index < 0 && b.index < 0 {
            results.handle(Self::encode(a.index), Self::encode(b.index));
        } else {
            *node_leaf.allocate(pool) = if a.index >= 0 {
                NodeLeafPair {
                    leaf_parent: b as *const NodeChild as *mut NodeChild,
                    node_index: a.index,
                }
            } else {
                NodeLeafPair {
                    leaf_parent: a as *const NodeChild as *mut NodeChild,
                    node_index: b.index,
                }
            };
        }
    }

    fn execute_crossover_batch<TOverlapHandler: IOverlapHandler>(
        &self,
        crossovers: &mut QuickList<IndexPair>,
        node_leaf: &mut QuickList<NodeLeafPair>,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
    ) {
        while let Some(pair) = crossovers.try_pop() {
            unsafe {
                let a = self.nodes.get(pair.a);
                let b = self.nodes.get(pair.b);
                // There are no shared children, so test them all.
                let aa = &a.a;
                let ab = &a.b;
                let ba = &b.a;
                let bb = &b.b;
                let aa_intersects = BoundingBox::intersects_unsafe(aa, ba);
                let ab_intersects = BoundingBox::intersects_unsafe(aa, bb);
                let ba_intersects = BoundingBox::intersects_unsafe(ab, ba);
                let bb_intersects = BoundingBox::intersects_unsafe(ab, bb);

                if aa_intersects {
                    self.add_crossover_result(aa, ba, crossovers, node_leaf, results, pool);
                }
                if ab_intersects {
                    self.add_crossover_result(aa, bb, crossovers, node_leaf, results, pool);
                }
                if ba_intersects {
                    self.add_crossover_result(ab, ba, crossovers, node_leaf, results, pool);
                }
                if bb_intersects {
                    self.add_crossover_result(ab, bb, crossovers, node_leaf, results, pool);
                }
            }
        }
    }

    fn execute_node_leaf_batch<TOverlapHandler: IOverlapHandler>(
        &self,
        node_leaf: &mut QuickList<NodeLeafPair>,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
    ) {
        while let Some(pair) = node_leaf.try_pop() {
            unsafe {
                let leaf_child = &*pair.leaf_parent;
                let node = self.nodes.get(pair.node_index);
                let a = &node.a;
                let b = &node.b;
                let _b_index = b.index;
                let a_intersects = BoundingBox::intersects_unsafe(leaf_child, a);
                let b_intersects = BoundingBox::intersects_unsafe(leaf_child, b);
                if a_intersects {
                    if a.index < 0 {
                        results.handle(Self::encode(leaf_child.index), Self::encode(a.index));
                    } else {
                        *node_leaf.allocate(pool) = NodeLeafPair {
                            leaf_parent: pair.leaf_parent,
                            node_index: a.index,
                        };
                    }
                }
                if b_intersects {
                    if b.index < 0 {
                        results.handle(Self::encode(leaf_child.index), Self::encode(b.index));
                    } else {
                        *node_leaf.allocate(pool) = NodeLeafPair {
                            leaf_parent: pair.leaf_parent,
                            node_index: b.index,
                        };
                    }
                }
            }
        }
    }

    /// Flushes queued leaf-leaf pairs to the results handler. Unused upstream.
    #[allow(dead_code)]
    fn flush_leaf_leaf<TOverlapHandler: IOverlapHandler>(
        &self,
        leaf_leaf: &mut QuickList<IndexPair>,
        results: &mut TOverlapHandler,
    ) {
        for i in 0..leaf_leaf.len() {
            let pair = *leaf_leaf.get(i);
            results.handle(Self::encode(pair.a), Self::encode(pair.b));
        }
        leaf_leaf.clear();
    }

    /// Reports all bounding box overlaps between leaves in the tree to the given
    /// `TOverlapHandler`. Processes nodes in cache-friendly batches, unlike `get_self_overlaps`.
    pub fn get_self_overlaps_batched<TOverlapHandler: IOverlapHandler>(
        &self,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
    ) {
        const CROSSOVER_BATCH_SIZE_TARGET: i32 = 16;
        const NODE_LEAF_BATCH_SIZE_TARGET: i32 = 16;
        let mut crossovers =
            QuickList::<IndexPair>::with_capacity(CROSSOVER_BATCH_SIZE_TARGET * 16, pool);
        let mut node_leaf =
            QuickList::<NodeLeafPair>::with_capacity(NODE_LEAF_BATCH_SIZE_TARGET * 16, pool);
        for i in (0..self.node_count).rev() {
            unsafe {
                let node = self.nodes.get(i);
                let a = &node.a;
                let b = &node.b;
                if BoundingBox::intersects_unsafe(a, b) {
                    if a.index >= 0 && b.index >= 0 {
                        *crossovers.allocate(pool) = IndexPair { a: a.index, b: b.index };
                    } else if a.index < 0 && b.index < 0 {
                        results.handle(Self::encode(a.index), Self::encode(b.index));
                    } else {
                        // Leaf-node.
                        *node_leaf.allocate(pool) = if a.index >= 0 {
                            NodeLeafPair {
                                leaf_parent: b as *const NodeChild as *mut NodeChild,
                                node_index: a.index,
                            }
                        } else {
                            NodeLeafPair {
                                leaf_parent: a as *const NodeChild as *mut NodeChild,
                                node_index: b.index,
                            }
                        };
                    }
                }
            }
            if crossovers.len() >= CROSSOVER_BATCH_SIZE_TARGET {
                self.execute_crossover_batch(&mut crossovers, &mut node_leaf, results, pool);
            }
            if node_leaf.len() >= NODE_LEAF_BATCH_SIZE_TARGET {
                self.execute_node_leaf_batch(&mut node_leaf, results, pool);
            }
        }
        // Flush any remaining pairs.
        self.execute_crossover_batch(&mut crossovers, &mut node_leaf, results, pool);
        self.execute_node_leaf_batch(&mut node_leaf, results, pool);
        crossovers.dispose(pool);
        node_leaf.dispose(pool);
    }

    fn get_self_overlaps2_range<TOverlapHandler: IOverlapHandler>(
        &self,
        results: &mut TOverlapHandler,
        start: i32,
        end: i32,
    ) {
        debug_assert!(
            end >= 0 && end <= self.node_count && start >= 0 && start < self.node_count
        );
        for i in (start..end).rev() {
            unsafe {
                let node = self.nodes.get(i);
                let a = &node.a;
                let b = &node.b;
                let ab = BoundingBox::intersects_unsafe(a, b);
                if ab {
                    self.dispatch_test_for_nodes(a, b, results);
                }
            }
        }
    }

    /// Reports all bounding box overlaps between leaves in the tree to the given
    /// `TOverlapHandler`.
    pub fn get_self_overlaps2<TOverlapHandler: IOverlapHandler>(
        &self,
        results: &mut TOverlapHandler,
    ) {
        self.get_self_overlaps2_range(results, 0, self.node_count);
    }

    /// Worker-budgeted implementation shared by both multithreaded `GetSelfOverlaps2` overloads.
    ///
    /// # Safety
    /// `task_stack` must point to a valid, initialized `TaskStack`.
    #[allow(clippy::too_many_arguments)]
    unsafe fn get_self_overlaps2_mt_impl<TOverlapHandler: IThreadedOverlapHandler + Copy>(
        &self,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
        worker_index: i32,
        task_stack: *mut TaskStack,
        thread_dispatcher: &dyn IThreadDispatcher,
        internally_dispatch: bool,
        worker_count: i32,
        mut target_task_budget: i32,
        managed_context: Option<&dyn std::any::Any>,
    ) {
        if target_task_budget < 0 {
            target_task_budget = thread_dispatcher.thread_count();
        }
        target_task_budget *= 16;
        target_task_budget = self.node_count.min(target_task_budget);

        const LEAF_THRESHOLD_FOR_TASK: i32 = 256;

        let mut results_copy = *results;
        let mut context = SelfTestContext::<TOverlapHandler> {
            tree: *self,
            loop_task_count: target_task_budget,
            leaf_threshold_for_task: LEAF_THRESHOLD_FOR_TASK,
            results: &mut results_copy as *mut TOverlapHandler,
            task_stack,
        };

        // Go ahead and submit very large early nodes as independent tasks to help with load
        // balancing. (This isn't guaranteed, or even intended, to catch all large individual
        // nodes. It's just an easy way to get some of them.)
        const MAXIMUM_ISOLATED_NODE_CAPACITY: i32 = 32;
        let isolated_node_capacity = MAXIMUM_ISOLATED_NODE_CAPACITY.min(target_task_budget / 4);
        let mut early_isolated_nodes_memory = [0i32; MAXIMUM_ISOLATED_NODE_CAPACITY as usize];
        let early_isolated_nodes_buffer = Buffer::new(
            early_isolated_nodes_memory.as_mut_ptr(),
            isolated_node_capacity,
            -1,
        );
        let mut early_isolated_nodes = QuickList::<i32>::new(early_isolated_nodes_buffer);
        let mut i = 0i32;
        while i < self.node_count && early_isolated_nodes.len() < isolated_node_capacity {
            let node = self.nodes.get(i);
            let a = &node.a;
            let b = &node.b;
            if a.leaf_count.max(b.leaf_count) > LEAF_THRESHOLD_FOR_TASK {
                // Note that this technically does double work on the bounds test with the way
                // we're submitting this as a task. Don't care; it's constant bounded nanoseconds.
                if BoundingBox::intersects_unsafe(a, b) {
                    *early_isolated_nodes.allocate_unsafely() = i;
                }
            } else {
                break;
            }
            i += 1;
        }
        // Regardless of why the loop stopped, everything before i has been handled as an isolated node.
        let early_isolated_node_interval_end = i;

        let remaining_node_count = self.node_count - early_isolated_node_interval_end;
        let regular_loop_task_count = target_task_budget - early_isolated_nodes.len();
        let nodes_per_task_base = remaining_node_count / regular_loop_task_count;
        let remainder = remaining_node_count - nodes_per_task_base * regular_loop_task_count;
        let mut tasks = pool.take::<Task>(target_task_budget);
        let mut previous_end = early_isolated_node_interval_end;
        for i in 0..regular_loop_task_count {
            let task_start = previous_end;
            let node_count_for_task = if i < remainder {
                nodes_per_task_base + 1
            } else {
                nodes_per_task_base
            };
            let task_end = previous_end + node_count_for_task;
            previous_end = task_end;
            tasks[i] = Task::new(
                loop_entry_task::<TOverlapHandler>,
                &mut context as *mut SelfTestContext<TOverlapHandler> as *mut c_void,
                ((task_start as u32) as i64) | ((task_end as i64) << 32),
                ContinuationHandle::default(),
            );
        }
        // Stick the early isolated nodes at the end so they're popped first.
        let tasks_len = tasks.len();
        for i in 0..early_isolated_nodes.len() {
            let task_start = *early_isolated_nodes.get(i);
            tasks[tasks_len - i - 1] = Task::new(
                loop_entry_task::<TOverlapHandler>,
                &mut context as *mut SelfTestContext<TOverlapHandler> as *mut c_void,
                ((task_start as u32) as i64) | (((task_start + 1) as i64) << 32),
                ContinuationHandle::default(),
            );
        }
        if internally_dispatch {
            // There isn't an active dispatch, so we need to do it.
            let on_complete = TaskStack::get_request_stop_task(task_stack);
            (*task_stack).allocate_continuation_and_push(
                tasks.as_slice_mut(),
                worker_index,
                thread_dispatcher,
                0,
                on_complete,
            );
            thread_dispatcher.dispatch_workers(
                TaskStack::dispatch_worker_function,
                worker_count,
                task_stack as *mut (),
                managed_context,
            );
        } else {
            // We're executing from within a multithreaded dispatch already, so we can simply
            // run the tasks and trust that other threads are ready to steal.
            (*task_stack).run_tasks_unfiltered(
                tasks.as_slice_mut(),
                worker_index,
                thread_dispatcher,
                0,
            );
        }
        tasks.dispose(pool);
        // Have to copy back the results; it's a value type.
        *results = results_copy;
    }

    /// Reports all bounding box overlaps between leaves in the tree to the given
    /// `TOverlapHandler`. Uses the thread dispatcher to parallelize overlap testing.
    ///
    /// # Safety
    /// Must be called with a valid `thread_dispatcher`.
    pub unsafe fn get_self_overlaps2_mt<TOverlapHandler: IThreadedOverlapHandler + Copy>(
        &self,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
        thread_dispatcher: &dyn IThreadDispatcher,
        managed_context: Option<&dyn std::any::Any>,
    ) {
        let mut task_stack = TaskStack::new(
            pool,
            thread_dispatcher,
            thread_dispatcher.thread_count(),
            128,
            128,
        );
        self.get_self_overlaps2_mt_impl(
            results,
            pool,
            0,
            &mut task_stack as *mut TaskStack,
            thread_dispatcher,
            true,
            thread_dispatcher.thread_count(),
            thread_dispatcher.thread_count(),
            managed_context,
        );
        task_stack.dispose(pool, thread_dispatcher);
    }

    /// Reports all bounding box overlaps between leaves in the tree to the given
    /// `TOverlapHandler`. Pushes tasks into the provided `TaskStack`. Does not dispatch threads
    /// internally; this is intended to be used as a part of a caller-managed dispatch.
    ///
    /// `target_task_count`: number of tasks the overlap testing should try to create during
    /// execution. If negative, uses `thread_dispatcher.thread_count()`.
    ///
    /// # Safety
    /// `task_stack` must point to a valid, initialized `TaskStack`.
    pub unsafe fn get_self_overlaps2_with_stack<TOverlapHandler: IThreadedOverlapHandler + Copy>(
        &self,
        results: &mut TOverlapHandler,
        pool: &mut BufferPool,
        thread_dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
    ) {
        self.get_self_overlaps2_mt_impl(
            results,
            pool,
            worker_index,
            task_stack,
            thread_dispatcher,
            false,
            thread_dispatcher.thread_count(),
            target_task_count,
            None,
        );
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
struct IndexPair {
    a: i32,
    b: i32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct NodeLeafPair {
    leaf_parent: *mut NodeChild,
    node_index: i32,
}

/// Context shared by every task submitted by a multithreaded `GetSelfOverlaps2`.
#[allow(dead_code)]
struct SelfTestContext<TOverlapHandler> {
    tree: Tree,
    loop_task_count: i32,
    leaf_threshold_for_task: i32,
    results: *mut TOverlapHandler,
    task_stack: *mut TaskStack,
}

/// Adapts an `IThreadedOverlapHandler` to the plain `IOverlapHandler` interface expected by the
/// single-threaded recursive overlap testing routines, tagging every overlap with the worker
/// index and managed context of the task that found it.
struct WrappedOverlapHandler<'a, TOverlapHandler: IThreadedOverlapHandler> {
    worker_index: i32,
    managed_context: Option<&'a dyn std::any::Any>,
    inner: *mut TOverlapHandler,
}

impl<'a, TOverlapHandler: IThreadedOverlapHandler> IOverlapHandler
    for WrappedOverlapHandler<'a, TOverlapHandler>
{
    fn handle(&mut self, index_a: i32, index_b: i32) {
        unsafe {
            (*self.inner).handle(index_a, index_b, self.worker_index, self.managed_context);
        }
    }
}

/// Task entry point for a chunk of a multithreaded `GetSelfOverlaps2`.
unsafe fn loop_entry_task<TOverlapHandler: IThreadedOverlapHandler>(
    task_start_and_end: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let task_start = task_start_and_end as i32;
    let task_end = (task_start_and_end >> 32) as i32;
    let context = &*(untyped_context as *mut SelfTestContext<TOverlapHandler>);
    let mut wrapped = WrappedOverlapHandler {
        inner: context.results,
        worker_index,
        managed_context: dispatcher.managed_context(),
    };
    context
        .tree
        .get_self_overlaps2_range(&mut wrapped, task_start, task_end);
}
