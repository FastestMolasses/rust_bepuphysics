// Translated from BepuPhysics/Trees/Tree_Refit2.cs

use super::leaf::Leaf;
use super::node::{Node, NodeChild};
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::{Task, TaskStack};
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::ffi::c_void;
use std::mem::MaybeUninit;

/// Context for MT refit (plain, non-cache-optimizing).
#[repr(C)]
struct RefitContext {
    tree: Tree,
    task_stack: *mut TaskStack,
    leaf_count_per_task: i32,
}

/// Context for MT refit with cache optimization.
#[repr(C)]
struct RefitWithCacheOptimizationContext {
    source_nodes: Buffer<Node>,
    tree: Tree,
    leaf_count_per_task: i32,
    task_stack: *mut TaskStack,
}

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

    // ── MT Refit2 (plain) ──

    unsafe fn refit2_with_task_spawning(
        &self,
        child_in_parent: &mut NodeChild,
        context: *mut RefitContext,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        debug_assert!(self.leaf_count >= 2);
        let node = &mut *(self.nodes.as_ptr() as *mut Node).add(child_in_parent.index as usize);
        let leaf_count_per_task = (*context).leaf_count_per_task;
        debug_assert!(leaf_count_per_task > 1);

        if node.a.leaf_count >= leaf_count_per_task && node.b.leaf_count >= leaf_count_per_task {
            // Both children are big enough to warrant a task. Spawn one for B and recurse on A.
            let task = Task::with_context(
                Self::refit2_task,
                context as *mut c_void,
                child_in_parent.index as i64,
            );
            let continuation = (*(*context).task_stack).allocate_continuation_and_push(
                std::slice::from_mut(&mut { task }),
                worker_index,
                dispatcher,
                0,
                Task::default(),
            );
            debug_assert!(node.a.index >= 0);
            self.refit2_with_task_spawning(&mut node.a, context, worker_index, dispatcher);
            (*(*context).task_stack).wait_for_completion_unfiltered(continuation, worker_index, dispatcher);
        } else {
            if node.a.index >= 0 {
                if node.a.leaf_count >= leaf_count_per_task {
                    self.refit2_with_task_spawning(&mut node.a, context, worker_index, dispatcher);
                } else {
                    self.refit2_recursive(&mut node.a);
                }
            }
            if node.b.index >= 0 {
                if node.b.leaf_count >= leaf_count_per_task {
                    self.refit2_with_task_spawning(&mut node.b, context, worker_index, dispatcher);
                } else {
                    self.refit2_recursive(&mut node.b);
                }
            }
        }
        let mut merged = MaybeUninit::new(*child_in_parent);
        BoundingBox::create_merged_unsafe_with_preservation(&node.a, &node.b, &mut merged);
        *child_in_parent = merged.assume_init();
    }

    unsafe fn refit2_task(
        parent_node_index: i64,
        untyped_context: *mut c_void,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let context = untyped_context as *mut RefitContext;
        let node = &mut *((*context).tree.nodes.as_ptr() as *mut Node).add(parent_node_index as usize);
        (*context).tree.refit2_with_task_spawning(&mut node.b, context, worker_index, dispatcher);
    }

    unsafe fn refit2_root_entry_task(
        _id: i64,
        untyped_context: *mut c_void,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let context = untyped_context as *mut RefitContext;
        let mut stub = NodeChild::default();
        stub.index = 0;
        (*context).tree.refit2_with_task_spawning(&mut stub, context, worker_index, dispatcher);
        (*(*context).task_stack).request_stop();
    }

    unsafe fn refit2_internal_mt(
        &self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
        internally_dispatch: bool,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let target_task_count = if target_task_count < 0 {
            dispatcher.thread_count()
        } else {
            target_task_count
        };
        const MINIMUM_TASK_SIZE: i32 = 32;
        let leaf_count_per_task = MINIMUM_TASK_SIZE.max(
            (self.leaf_count as f32 / target_task_count as f32).ceil() as i32,
        );
        let mut refit_context = RefitContext {
            leaf_count_per_task,
            task_stack,
            tree: *self,
        };
        if internally_dispatch {
            (*task_stack).push_unsafely_single(
                Task::with_context(
                    Self::refit2_root_entry_task,
                    &mut refit_context as *mut RefitContext as *mut c_void,
                    0,
                ),
                worker_index,
                dispatcher,
                0,
            );
            TaskStack::dispatch_workers(
                dispatcher,
                task_stack,
                dispatcher.thread_count().min(target_task_count),
            );
        } else {
            let mut stub = NodeChild::default();
            stub.index = 0;
            self.refit2_with_task_spawning(&mut stub, &mut refit_context, worker_index, dispatcher);
        }
    }

    /// MT Refit2. Creates its own TaskStack and dispatches internally.
    pub unsafe fn refit2_mt(
        &self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let thread_count = dispatcher.thread_count();
        let mut task_stack = TaskStack::new(pool, dispatcher, thread_count, 128, 128);
        self.refit2_internal_mt(pool, dispatcher, &mut task_stack, 0, -1, true);
        task_stack.dispose(pool, dispatcher);
    }

    /// MT Refit2 with caller-managed TaskStack.
    pub unsafe fn refit2_mt_with_task_stack(
        &self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
    ) {
        self.refit2_internal_mt(pool, dispatcher, task_stack, worker_index, target_task_count, false);
    }

    // ── MT Refit2 with cache optimization ──

    unsafe fn refit2_with_cache_opt_and_task_spawning(
        source_node_index: i32,
        parent_index: i32,
        child_index_in_parent: i32,
        child_in_parent: &mut NodeChild,
        context: *mut RefitWithCacheOptimizationContext,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let ctx = &mut *context;
        debug_assert!(ctx.tree.leaf_count >= 2);
        let source_node = &*ctx.source_nodes.get(source_node_index);
        let target_node_index = child_in_parent.index;
        let target_node = &mut *(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
        let target_metanode = ctx.tree.metanodes.get_mut(target_node_index);
        target_metanode.parent = parent_index;
        target_metanode.index_in_parent = child_index_in_parent;

        let source_a = source_node.a;
        let source_b = source_node.b;
        let target_index_a = target_node_index + 1;
        let target_index_b = target_node_index + source_a.leaf_count;
        debug_assert!(ctx.leaf_count_per_task > 1);

        if source_a.leaf_count >= ctx.leaf_count_per_task && source_b.leaf_count >= ctx.leaf_count_per_task {
            // Both children big enough — spawn task for B, recurse on A.
            target_node.a.index = target_index_a;
            target_node.a.leaf_count = source_a.leaf_count;
            target_node.b.index = target_index_b;
            target_node.b.leaf_count = source_b.leaf_count;
            // Encode both target and source parent indices into the task id.
            let task_id = (child_in_parent.index as i64) | ((source_node_index as i64) << 32);
            let task = Task::with_context(
                Self::refit2_with_cache_opt_task,
                context as *mut c_void,
                task_id,
            );
            let continuation = (*ctx.task_stack).allocate_continuation_and_push(
                std::slice::from_mut(&mut { task }),
                worker_index,
                dispatcher,
                0,
                Task::default(),
            );
            debug_assert!(source_a.index >= 0);
            Self::refit2_with_cache_opt_and_task_spawning(
                source_a.index,
                target_node_index,
                0,
                &mut (*(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize)).a,
                context,
                worker_index,
                dispatcher,
            );
            (*ctx.task_stack).wait_for_completion_unfiltered(continuation, worker_index, dispatcher);
        } else {
            if source_a.index >= 0 {
                target_node.a.index = target_index_a;
                target_node.a.leaf_count = source_a.leaf_count;
                if source_a.leaf_count >= ctx.leaf_count_per_task {
                    Self::refit2_with_cache_opt_and_task_spawning(
                        source_a.index, target_node_index, 0,
                        &mut (*(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize)).a,
                        context, worker_index, dispatcher,
                    );
                } else {
                    Self::refit2_with_cache_optimization_recursive(
                        source_a.index, target_node_index, 0,
                        &mut (*(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize)).a,
                        &ctx.source_nodes, &mut ctx.tree,
                    );
                }
            } else {
                target_node.a = source_a;
                *ctx.tree.leaves.get_mut(Self::encode(source_a.index)) =
                    Leaf::new(target_node_index, 0);
            }
            if source_b.index >= 0 {
                let target_node = &mut *(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
                target_node.b.index = target_index_b;
                target_node.b.leaf_count = source_b.leaf_count;
                if source_b.leaf_count >= ctx.leaf_count_per_task {
                    Self::refit2_with_cache_opt_and_task_spawning(
                        source_b.index, target_node_index, 1,
                        &mut (*(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize)).b,
                        context, worker_index, dispatcher,
                    );
                } else {
                    Self::refit2_with_cache_optimization_recursive(
                        source_b.index, target_node_index, 1,
                        &mut (*(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize)).b,
                        &ctx.source_nodes, &mut ctx.tree,
                    );
                }
            } else {
                let target_node = &mut *(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
                target_node.b = source_b;
                *ctx.tree.leaves.get_mut(Self::encode(source_b.index)) =
                    Leaf::new(target_node_index, 1);
            }
        }
        let target_node = &mut *(ctx.tree.nodes.as_ptr() as *mut Node).add(target_node_index as usize);
        let mut merged = MaybeUninit::new(*child_in_parent);
        BoundingBox::create_merged_unsafe_with_preservation(&target_node.a, &target_node.b, &mut merged);
        *child_in_parent = merged.assume_init();
    }

    unsafe fn refit2_with_cache_opt_task(
        parent_node_indices: i64,
        untyped_context: *mut c_void,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let context = untyped_context as *mut RefitWithCacheOptimizationContext;
        let source_parent_index = (parent_node_indices >> 32) as i32;
        let target_parent_index = parent_node_indices as i32;
        let source_parent_node = &*(*context).source_nodes.get(source_parent_index);
        let target_parent_node = &mut *((*context).tree.nodes.as_ptr() as *mut Node).add(target_parent_index as usize);
        Self::refit2_with_cache_opt_and_task_spawning(
            source_parent_node.b.index,
            target_parent_index,
            1,
            &mut target_parent_node.b,
            context,
            worker_index,
            dispatcher,
        );
    }

    unsafe fn refit2_with_cache_opt_root_entry_task(
        _id: i64,
        untyped_context: *mut c_void,
        worker_index: i32,
        dispatcher: &dyn IThreadDispatcher,
    ) {
        let context = untyped_context as *mut RefitWithCacheOptimizationContext;
        let mut stub = NodeChild::default();
        stub.index = 0;
        Self::refit2_with_cache_opt_and_task_spawning(0, -1, -1, &mut stub, context, worker_index, dispatcher);
        (*(*context).task_stack).request_stop();
    }

    unsafe fn refit2_with_cache_opt_internal_mt(
        &mut self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
        internally_dispatch: bool,
        source_nodes: Buffer<Node>,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let target_task_count = if target_task_count < 0 {
            dispatcher.thread_count()
        } else {
            target_task_count
        };
        const MINIMUM_TASK_SIZE: i32 = 32;
        let leaf_count_per_task = MINIMUM_TASK_SIZE.max(
            (self.leaf_count as f32 / target_task_count as f32).ceil() as i32,
        );
        let mut refit_context = RefitWithCacheOptimizationContext {
            source_nodes,
            tree: *self,
            leaf_count_per_task,
            task_stack,
        };
        if internally_dispatch {
            (*task_stack).push_unsafely_single(
                Task::with_context(
                    Self::refit2_with_cache_opt_root_entry_task,
                    &mut refit_context as *mut RefitWithCacheOptimizationContext as *mut c_void,
                    0,
                ),
                worker_index,
                dispatcher,
                0,
            );
            TaskStack::dispatch_workers(
                dispatcher,
                task_stack,
                dispatcher.thread_count().min(target_task_count),
            );
        } else {
            let mut stub = NodeChild::default();
            stub.index = 0;
            Self::refit2_with_cache_opt_and_task_spawning(
                0, -1, -1, &mut stub, &mut refit_context, worker_index, dispatcher,
            );
        }
    }

    /// MT Refit2 with cache optimization. Creates its own TaskStack and dispatches internally.
    /// Reallocates the nodes buffer with depth-first traversal order.
    pub unsafe fn refit2_with_cache_optimization_mt(
        &mut self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        dispose_original_nodes: bool,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let thread_count = dispatcher.thread_count();
        let mut task_stack = TaskStack::new(pool, dispatcher, thread_count, 128, 128);
        let old_nodes = self.nodes;
        self.nodes = pool.take_at_least::<Node>(old_nodes.len());
        self.refit2_with_cache_opt_internal_mt(
            pool, dispatcher, &mut task_stack, 0, -1, true, old_nodes,
        );
        task_stack.dispose(pool, dispatcher);
        if dispose_original_nodes {
            let mut old = old_nodes;
            pool.return_buffer(&mut old);
        }
    }

    /// MT Refit2 with cache optimization using a caller-managed TaskStack.
    /// Reallocates the nodes buffer with depth-first traversal order.
    pub unsafe fn refit2_with_cache_optimization_mt_with_task_stack(
        &mut self,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
        dispose_original_nodes: bool,
    ) {
        if self.leaf_count <= 2 {
            return;
        }
        let old_nodes = self.nodes;
        self.nodes = pool.take_at_least::<Node>(old_nodes.len());
        self.refit2_with_cache_opt_internal_mt(
            pool, dispatcher, task_stack, worker_index, target_task_count, false, old_nodes,
        );
        if dispose_original_nodes {
            let mut old = old_nodes;
            pool.return_buffer(&mut old);
        }
    }

    /// MT Refit2 with cache optimization from caller-provided source nodes.
    /// Uses a caller-managed TaskStack.
    pub unsafe fn refit2_with_cache_optimization_from_source_mt(
        &mut self,
        source_nodes: Buffer<Node>,
        pool: &mut BufferPool,
        dispatcher: &dyn IThreadDispatcher,
        task_stack: *mut TaskStack,
        worker_index: i32,
        target_task_count: i32,
    ) {
        self.refit2_with_cache_opt_internal_mt(
            pool, dispatcher, task_stack, worker_index, target_task_count, false, source_nodes,
        );
    }
}
