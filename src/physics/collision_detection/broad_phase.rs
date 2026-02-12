// Translated from BepuPhysics/CollisionDetection/BroadPhase.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::ray_batchers::{IBroadPhaseRayTester, IBroadPhaseSweepTester, RayData};
use crate::physics::trees::ray_batcher::{RayData as TreeRayData, TreeRay};
use crate::physics::trees::tree::Tree;
use crate::physics::trees::tree_multithreaded_refit_refine::RefitAndRefineMultithreadedContext;
use crate::physics::trees::tree_ray_cast::IRayLeafTester;
use crate::physics::trees::tree_sweep::ISweepLeafTester;
use crate::utilities::for_each_ref::IBreakableForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use crate::utilities::task_scheduling::{Task, TaskStack};
use crate::utilities::bounding_box::BoundingBox;
use glam::Vec3;
use std::cell::UnsafeCell;
use std::ffi::c_void;
use std::sync::atomic::{AtomicI32, Ordering};

/// Returns the size and number of refinements to execute during the broad phase.
pub type RefinementScheduler = fn(
    frame_index: i32,
    tree: &Tree,
    root_refinement_size: &mut i32,
    subtree_refinement_count: &mut i32,
    subtree_refinement_size: &mut i32,
    use_priority_queue: &mut bool,
);

/// Core refinement scheduling logic. Computes refinement sizes and counts based on tree size and frame index.
pub fn default_refinement_scheduler(
    optimization_fraction: f32,
    root_refinement_period: i32,
    root_refinement_size_scale: f32,
    subtree_refinement_size_scale: f32,
    nonpriority_period: i32,
    frame_index: i32,
    tree: &Tree,
    root_refinement_size: &mut i32,
    subtree_refinement_count: &mut i32,
    subtree_refinement_size: &mut i32,
    use_priority_queue: &mut bool,
) {
    let refine_root = frame_index % root_refinement_period == 0;
    let target_optimized_leaf_count = (tree.leaf_count as f32 * optimization_fraction).ceil() as i32;
    // The square root of the leaf count gets us roughly halfway down the tree.
    // Root and subtree refinements need to be larger than that: subtrees must be able to exchange nodes,
    // and the root refinement is the intermediary.
    let sqrt_leaf_count = (tree.leaf_count as f32).sqrt();

    let target_root_refinement_size = (sqrt_leaf_count * root_refinement_size_scale).ceil() as i32;
    *subtree_refinement_size = (sqrt_leaf_count * subtree_refinement_size_scale).ceil() as i32;

    // Note that we scale up the cost of the root refinement; it uses a sequentialized priority queue
    // to collect subtrees for refinement and costs more.
    let root_ref_size_f = target_root_refinement_size as f32;
    let subtree_ref_size_f = *subtree_refinement_size as f32;
    let denom = subtree_ref_size_f * subtree_ref_size_f.log2();
    let subtree_refinements_per_root_refinement_in_cost = if denom > 0.0 {
        root_ref_size_f * root_ref_size_f.log2() / denom
    } else {
        0.0
    };
    // If we're refining the root, reduce the number of subtree refinements to avoid cost spikes.
    let adjustment = if refine_root {
        subtree_refinements_per_root_refinement_in_cost
    } else {
        0.0
    };
    let divisor = subtree_ref_size_f.max(1.0);
    *subtree_refinement_count = 0i32.max(
        (target_optimized_leaf_count as f32 / divisor - adjustment).round() as i32,
    );
    if !refine_root {
        *subtree_refinement_count = 1i32.max(*subtree_refinement_count);
    }

    *root_refinement_size = if refine_root {
        target_root_refinement_size
    } else {
        0
    };
    *use_priority_queue = (frame_index / root_refinement_period) % nonpriority_period != 0;
}

/// Default refinement schedule for the active tree.
pub fn default_active_refinement_scheduler(
    frame_index: i32,
    tree: &Tree,
    root_refinement_size: &mut i32,
    subtree_refinement_count: &mut i32,
    subtree_refinement_size: &mut i32,
    use_priority_queue: &mut bool,
) {
    default_refinement_scheduler(
        1.0 / 20.0,
        2,
        1.0,
        4.0,
        16,
        frame_index,
        tree,
        root_refinement_size,
        subtree_refinement_count,
        subtree_refinement_size,
        use_priority_queue,
    );
}

/// Default refinement schedule for the static tree.
pub fn default_static_refinement_scheduler(
    frame_index: i32,
    tree: &Tree,
    root_refinement_size: &mut i32,
    subtree_refinement_count: &mut i32,
    subtree_refinement_size: &mut i32,
    use_priority_queue: &mut bool,
) {
    default_refinement_scheduler(
        1.0 / 100.0,
        2,
        1.0,
        4.0,
        16,
        frame_index,
        tree,
        root_refinement_size,
        subtree_refinement_count,
        subtree_refinement_size,
        use_priority_queue,
    );
}

/// Manages scene acceleration structures for collision detection and queries.
pub struct BroadPhase {
    /// Collidable references contained within the active tree.
    pub active_leaves: Buffer<CollidableReference>,
    /// Collidable references contained within the static tree.
    pub static_leaves: Buffer<CollidableReference>,
    /// Pool used by the broad phase.
    pub pool: *mut BufferPool,
    /// Tree containing wakeful bodies.
    pub active_tree: Tree,
    /// Tree containing sleeping bodies and statics.
    pub static_tree: Tree,

    frame_index: i32,
    static_subtree_refinement_start_index: i32,
    active_subtree_refinement_start_index: i32,

    /// Refinement schedule used for the active tree.
    pub active_refinement_schedule: RefinementScheduler,
    /// Refinement schedule used for the static tree.
    pub static_refinement_schedule: RefinementScheduler,

    // MT refit/refine contexts for the old Update() path.
    active_refine_context: RefitAndRefineMultithreadedContext,
    static_refine_context: RefitAndRefineMultithreadedContext,
    remaining_job_count: UnsafeCell<i32>,
}

use crate::physics::trees::node::Node;

/// Context for MT broad phase refinement/refit tasks.
#[repr(C)]
struct BroadPhaseRefinementContext {
    task_stack: *mut TaskStack,
    tree: Tree,
    target_task_count: i32,
    target_total_task_count: i32,
    root_refinement_size: i32,
    subtree_refinement_count: i32,
    subtree_refinement_size: i32,
    subtree_refinement_start_index: i32,
    deterministic: bool,
    use_priority_queue: bool,
    /// Used for active tree to hold depth-first refit output nodes.
    target_nodes: Buffer<Node>,
}

/// Active entrypoint task: refine2 + refit2_with_cache_optimization.
unsafe fn active_entrypoint_task(
    _task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let context = &mut *(untyped_context as *mut BroadPhaseRefinementContext);
    let pool = &mut *dispatcher.worker_pool_ptr(worker_index);
    context.tree.refine2_mt_with_task_stack(
        context.root_refinement_size,
        &mut context.subtree_refinement_start_index,
        context.subtree_refinement_count,
        context.subtree_refinement_size,
        pool,
        dispatcher,
        context.task_stack,
        worker_index,
        context.target_task_count,
        context.deterministic,
        context.use_priority_queue,
    );
    // Now refit with cache optimization.
    let source_nodes = context.tree.nodes;
    context.tree.nodes = context.target_nodes;
    context.tree.refit2_with_cache_optimization_from_source_mt(
        source_nodes,
        pool,
        dispatcher,
        context.task_stack,
        worker_index,
        context.target_total_task_count,
    );
}

/// Static entrypoint task: refine2 only (no refit for static tree).
unsafe fn static_entrypoint_task(
    _task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let context = &mut *(untyped_context as *mut BroadPhaseRefinementContext);
    let pool = &mut *dispatcher.worker_pool_ptr(worker_index);
    context.tree.refine2_mt_with_task_stack(
        context.root_refinement_size,
        &mut context.subtree_refinement_start_index,
        context.subtree_refinement_count,
        context.subtree_refinement_size,
        pool,
        dispatcher,
        context.task_stack,
        worker_index,
        context.target_task_count,
        context.deterministic,
        context.use_priority_queue,
    );
}

impl BroadPhase {
    /// Creates a new BroadPhase.
    pub fn new(
        pool: *mut BufferPool,
        initial_active_leaf_capacity: i32,
        initial_static_leaf_capacity: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let active_tree = Tree::new(pool_ref, initial_active_leaf_capacity);
        let static_tree = Tree::new(pool_ref, initial_static_leaf_capacity);
        let active_leaves = pool_ref.take_at_least(initial_active_leaf_capacity);
        let static_leaves = pool_ref.take_at_least(initial_static_leaf_capacity);

        Self {
            active_leaves,
            static_leaves,
            pool,
            active_tree,
            static_tree,
            frame_index: 0,
            static_subtree_refinement_start_index: 0,
            active_subtree_refinement_start_index: 0,
            active_refinement_schedule: default_active_refinement_scheduler,
            static_refinement_schedule: default_static_refinement_scheduler,
            active_refine_context: RefitAndRefineMultithreadedContext::new(),
            static_refine_context: RefitAndRefineMultithreadedContext::new(),
            remaining_job_count: UnsafeCell::new(0),
        }
    }

    #[inline(always)]
    fn add_internal(
        collidable: CollidableReference,
        min: &Vec3,
        max: &Vec3,
        tree: &mut Tree,
        pool: &mut BufferPool,
        leaves: &mut Buffer<CollidableReference>,
    ) -> i32 {
        let bounds = BoundingBox { min: *min, _pad0: 0.0, max: *max, _pad1: 0.0 };
        let leaf_index = tree.add(bounds, pool);
        if leaf_index >= leaves.len() {
            pool.resize_to_at_least(leaves, tree.leaf_count + 1, leaves.len());
        }
        unsafe {
            *leaves.get_mut(leaf_index) = collidable;
        }
        leaf_index
    }

    /// Removes a leaf at the given index, returning whether a leaf was moved and the moved leaf's reference.
    #[inline(always)]
    pub fn remove_at(
        index: i32,
        tree: &mut Tree,
        leaves: &mut Buffer<CollidableReference>,
        moved_leaf: &mut CollidableReference,
    ) -> bool {
        debug_assert!(index >= 0);
        let moved_leaf_index = tree.remove_at(index);
        if moved_leaf_index >= 0 {
            unsafe {
                *moved_leaf = *leaves.get(moved_leaf_index);
                *leaves.get_mut(index) = *moved_leaf;
            }
            true
        } else {
            *moved_leaf = CollidableReference::default();
            false
        }
    }

    /// Adds an active collidable to the broad phase.
    #[inline(always)]
    pub fn add_active(&mut self, collidable: CollidableReference, min: &Vec3, max: &Vec3) -> i32 {
        let pool = unsafe { &mut *self.pool };
        Self::add_internal(
            collidable,
            min,
            max,
            &mut self.active_tree,
            pool,
            &mut self.active_leaves,
        )
    }

    /// Removes an active collidable at the given index.
    #[inline(always)]
    pub fn remove_active_at(&mut self, index: i32, moved_leaf: &mut CollidableReference) -> bool {
        Self::remove_at(
            index,
            &mut self.active_tree,
            &mut self.active_leaves,
            moved_leaf,
        )
    }

    /// Adds a static collidable to the broad phase.
    #[inline(always)]
    pub fn add_static(&mut self, collidable: CollidableReference, min: &Vec3, max: &Vec3) -> i32 {
        let pool = unsafe { &mut *self.pool };
        Self::add_internal(
            collidable,
            min,
            max,
            &mut self.static_tree,
            pool,
            &mut self.static_leaves,
        )
    }

    /// Removes a static collidable at the given index.
    #[inline(always)]
    pub fn remove_static_at(&mut self, index: i32, moved_leaf: &mut CollidableReference) -> bool {
        Self::remove_at(
            index,
            &mut self.static_tree,
            &mut self.static_leaves,
            moved_leaf,
        )
    }

    /// Gets pointers to the leaf's bounds stored in the active tree.
    #[inline(always)]
    pub unsafe fn get_active_bounds_pointers(
        &self,
        index: i32,
    ) -> (*mut Vec3, *mut Vec3) {
        self.active_tree.get_bounds_pointers(index)
    }

    /// Gets pointers to the leaf's bounds stored in the static tree.
    #[inline(always)]
    pub unsafe fn get_static_bounds_pointers(
        &self,
        index: i32,
    ) -> (*mut Vec3, *mut Vec3) {
        self.static_tree.get_bounds_pointers(index)
    }

    /// Applies updated bounds to the given active leaf index, refitting the tree to match.
    #[inline(always)]
    pub fn update_active_bounds(&mut self, broad_phase_index: i32, min: Vec3, max: Vec3) {
        self.active_tree.update_bounds(broad_phase_index, min, max);
    }

    /// Applies updated bounds to the given static leaf index, refitting the tree to match.
    #[inline(always)]
    pub fn update_static_bounds(&mut self, broad_phase_index: i32, min: Vec3, max: Vec3) {
        self.static_tree.update_bounds(broad_phase_index, min, max);
    }

    /// Worker function for multi-threaded refit-and-mark phase.
    fn execute_refit_and_mark_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let bp = &mut *(dispatcher.unmanaged_context() as *mut BroadPhase);
            let thread_pool = dispatcher.worker_pool_ptr(worker_index);
            loop {
                let job_index = AtomicI32::from_ptr(bp.remaining_job_count.get()).fetch_sub(1, Ordering::AcqRel) - 1;
                if job_index < 0 {
                    break;
                }
                let active_count = bp.active_refine_context.refit_nodes.count;
                if job_index < active_count {
                    bp.active_refine_context.execute_refit_and_mark_job(&mut *thread_pool, worker_index, job_index);
                } else {
                    let static_index = job_index - active_count;
                    bp.static_refine_context.execute_refit_and_mark_job(&mut *thread_pool, worker_index, static_index);
                }
            }
        }
    }

    /// Worker function for multi-threaded refine phase.
    fn execute_refine_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let bp = &mut *(dispatcher.unmanaged_context() as *mut BroadPhase);
            let thread_pool = dispatcher.worker_pool_ptr(worker_index);
            let max_subtrees = bp.active_refine_context.maximum_subtrees.max(bp.static_refine_context.maximum_subtrees);
            let mut subtree_references = crate::utilities::collections::quicklist::QuickList::with_capacity(max_subtrees, &mut *thread_pool);
            let mut treelet_internal_nodes = crate::utilities::collections::quicklist::QuickList::with_capacity(max_subtrees, &mut *thread_pool);
            // Create binned resources using the active tree (both contexts share the same max_subtrees).
            let (mut buffer, mut resources) = bp.active_tree.create_binned_resources(&mut *thread_pool, max_subtrees);
            loop {
                let job_index = AtomicI32::from_ptr(bp.remaining_job_count.get()).fetch_sub(1, Ordering::AcqRel) - 1;
                if job_index < 0 {
                    break;
                }
                let active_count = bp.active_refine_context.refinement_targets.count;
                if job_index < active_count {
                    bp.active_refine_context.execute_refine_job(
                        &mut subtree_references,
                        &mut treelet_internal_nodes,
                        &mut resources,
                        &mut *thread_pool,
                        job_index,
                    );
                } else {
                    let static_index = job_index - active_count;
                    bp.static_refine_context.execute_refine_job(
                        &mut subtree_references,
                        &mut treelet_internal_nodes,
                        &mut resources,
                        &mut *thread_pool,
                        static_index,
                    );
                }
            }
            subtree_references.dispose(&mut *thread_pool);
            treelet_internal_nodes.dispose(&mut *thread_pool);
            (&mut *thread_pool).return_buffer(&mut buffer);
        }
    }

    /// Updates the broad phase trees (supports multi-threaded path via old RefitAndRefine pipeline).
    pub fn update(&mut self, thread_dispatcher: Option<*mut dyn IThreadDispatcher>) {
        if self.frame_index == i32::MAX {
            self.frame_index = 0;
        }
        unsafe {
            let pool = &mut *self.pool;
            if let Some(td_ptr) = thread_dispatcher {
                let td = &*td_ptr;
                self.active_refine_context.create_refit_and_mark_jobs(&mut self.active_tree, pool, td);
                self.static_refine_context.create_refit_and_mark_jobs(&mut self.static_tree, pool, td);
                let active_refit = self.active_refine_context.refit_nodes.count;
                let static_refit = self.static_refine_context.refit_nodes.count;
                *self.remaining_job_count.get() = active_refit + static_refit;
                let self_ptr = self as *mut BroadPhase as *mut ();
                td.dispatch_workers(
                    Self::execute_refit_and_mark_worker,
                    active_refit + static_refit,
                    self_ptr,
                    None,
                );
                self.active_refine_context.create_refinement_jobs(pool, self.frame_index, 1.0);
                self.static_refine_context.create_refinement_jobs(pool, self.frame_index, 1.0);
                let active_refine = self.active_refine_context.refinement_targets.count;
                let static_refine = self.static_refine_context.refinement_targets.count;
                *self.remaining_job_count.get() = active_refine + static_refine;
                td.dispatch_workers(
                    Self::execute_refine_worker,
                    active_refine + static_refine,
                    self_ptr,
                    None,
                );
                self.active_refine_context.clean_up_for_refit_and_refine(pool);
                self.static_refine_context.clean_up_for_refit_and_refine(pool);
            } else {
                self.active_tree.refit_and_refine(pool, self.frame_index, 1.0);
                self.static_tree.refit_and_refine(pool, self.frame_index, 1.0);
            }
        }
        self.frame_index += 1;
    }

    /// Updates the broad phase trees using the Refine2/Refit2 pipeline with incremental refinement scheduling.
    pub unsafe fn update2(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        let active_schedule = self.active_refinement_schedule;
        let static_schedule = self.static_refinement_schedule;

        let mut active_root_refinement_size = 0i32;
        let mut active_subtree_refinement_count = 0i32;
        let mut active_subtree_refinement_size = 0i32;
        let mut use_priority_queue_active = false;
        (active_schedule)(
            self.frame_index,
            &self.active_tree,
            &mut active_root_refinement_size,
            &mut active_subtree_refinement_count,
            &mut active_subtree_refinement_size,
            &mut use_priority_queue_active,
        );

        let mut static_root_refinement_size = 0i32;
        let mut static_subtree_refinement_count = 0i32;
        let mut static_subtree_refinement_size = 0i32;
        let mut use_priority_queue_static = false;
        (static_schedule)(
            self.frame_index,
            &self.static_tree,
            &mut static_root_refinement_size,
            &mut static_subtree_refinement_count,
            &mut static_subtree_refinement_size,
            &mut use_priority_queue_static,
        );

        const MINIMUM_LEAF_COUNT_FOR_THREADING: i32 = 256;
        let pool = &mut *self.pool;

        if let Some(dispatcher) = thread_dispatcher {
            let thread_count = dispatcher.thread_count();
            if thread_count > 1
                && (self.active_tree.leaf_count >= MINIMUM_LEAF_COUNT_FOR_THREADING
                    || self.static_tree.leaf_count >= MINIMUM_LEAF_COUNT_FOR_THREADING)
            {
                // Distribute tasks proportional to cost.
                let active_cost = (active_root_refinement_size as f32 + 1.0).log2()
                    * active_root_refinement_size as f32
                    + (active_subtree_refinement_size as f32 + 1.0).log2()
                        * active_subtree_refinement_size as f32
                        * active_subtree_refinement_count as f32;
                let static_cost = (static_root_refinement_size as f32 + 1.0).log2()
                    * static_root_refinement_size as f32
                    + (static_subtree_refinement_size as f32 + 1.0).log2()
                        * static_subtree_refinement_size as f32
                        * static_subtree_refinement_count as f32;
                let active_task_fraction = active_cost / (active_cost + static_cost);
                let target_total_task_count = thread_count;
                let target_active_task_count =
                    (active_task_fraction * target_total_task_count as f32).ceil() as i32;

                let mut task_stack = TaskStack::new(pool, dispatcher, thread_count, 128, 128);

                let mut active_refine_context = BroadPhaseRefinementContext {
                    task_stack: &mut task_stack,
                    tree: self.active_tree,
                    target_total_task_count,
                    target_task_count: target_active_task_count,
                    root_refinement_size: active_root_refinement_size,
                    subtree_refinement_count: active_subtree_refinement_count,
                    subtree_refinement_size: active_subtree_refinement_size,
                    subtree_refinement_start_index: self.active_subtree_refinement_start_index,
                    deterministic,
                    use_priority_queue: use_priority_queue_active,
                    target_nodes: if self.active_tree.leaf_count > 2 {
                        pool.take_at_least::<Node>(self.active_tree.nodes.len())
                    } else {
                        Buffer::default()
                    },
                };
                let mut static_refine_context = BroadPhaseRefinementContext {
                    task_stack: &mut task_stack,
                    tree: self.static_tree,
                    target_total_task_count,
                    target_task_count: target_total_task_count - target_active_task_count,
                    root_refinement_size: static_root_refinement_size,
                    subtree_refinement_count: static_subtree_refinement_count,
                    subtree_refinement_size: static_subtree_refinement_size,
                    subtree_refinement_start_index: self.static_subtree_refinement_start_index,
                    deterministic,
                    use_priority_queue: use_priority_queue_static,
                    target_nodes: Buffer::default(),
                };

                let mut tasks = [
                    Task::with_context(
                        active_entrypoint_task,
                        &mut active_refine_context as *mut BroadPhaseRefinementContext
                            as *mut c_void,
                        0,
                    ),
                    Task::with_context(
                        static_entrypoint_task,
                        &mut static_refine_context as *mut BroadPhaseRefinementContext
                            as *mut c_void,
                        1,
                    ),
                ];
                let on_complete = TaskStack::get_request_stop_task(&mut task_stack);
                task_stack.allocate_continuation_and_push(
                    &mut tasks,
                    0,
                    dispatcher,
                    0,
                    on_complete,
                );
                TaskStack::dispatch_workers(
                    dispatcher,
                    &mut task_stack,
                    thread_count,
                );
                task_stack.dispose(pool, dispatcher);

                if self.active_tree.leaf_count > 2 {
                    // Cache-optimizing refit modifies the tree copy. Copy back.
                    pool.return_buffer(&mut self.active_tree.nodes);
                    self.active_tree.nodes = active_refine_context.target_nodes;
                }
                // Copy back start indices.
                self.active_subtree_refinement_start_index =
                    active_refine_context.subtree_refinement_start_index;
                self.static_subtree_refinement_start_index =
                    static_refine_context.subtree_refinement_start_index;
            } else {
                // Thread dispatcher available but trees too small — fall through to ST.
                self.update2_single_threaded(
                    pool,
                    active_root_refinement_size,
                    active_subtree_refinement_count,
                    active_subtree_refinement_size,
                    use_priority_queue_active,
                    static_root_refinement_size,
                    static_subtree_refinement_count,
                    static_subtree_refinement_size,
                    use_priority_queue_static,
                );
            }
        } else {
            // Single-threaded path.
            self.update2_single_threaded(
                pool,
                active_root_refinement_size,
                active_subtree_refinement_count,
                active_subtree_refinement_size,
                use_priority_queue_active,
                static_root_refinement_size,
                static_subtree_refinement_count,
                static_subtree_refinement_size,
                use_priority_queue_static,
            );
        }

        if self.frame_index == i32::MAX {
            self.frame_index = 0;
        } else {
            self.frame_index += 1;
        }
    }

    fn update2_single_threaded(
        &mut self,
        pool: &mut BufferPool,
        active_root_refinement_size: i32,
        active_subtree_refinement_count: i32,
        active_subtree_refinement_size: i32,
        use_priority_queue_active: bool,
        static_root_refinement_size: i32,
        static_subtree_refinement_count: i32,
        static_subtree_refinement_size: i32,
        use_priority_queue_static: bool,
    ) {
        // Note: we refine *before* refitting. Refinement works with slightly out-of-date data,
        // but the point is incremental improvement. Refit with cache optimization *after* refinement
        // ensures the rest of the library sees the optimized layout.
        self.static_tree.refine2(
            static_root_refinement_size,
            &mut self.static_subtree_refinement_start_index,
            static_subtree_refinement_count,
            static_subtree_refinement_size,
            pool,
            use_priority_queue_static,
        );
        self.active_tree.refine2(
            active_root_refinement_size,
            &mut self.active_subtree_refinement_start_index,
            active_subtree_refinement_count,
            active_subtree_refinement_size,
            pool,
            use_priority_queue_active,
        );
        self.active_tree.refit2_with_cache_optimization(pool, true);
    }

    /// Clears out the broad phase's structures without releasing any resources.
    pub fn clear(&mut self) {
        self.active_tree.clear();
        self.static_tree.clear();
    }

    fn ensure_capacity_internal(
        tree: &mut Tree,
        leaves: &mut Buffer<CollidableReference>,
        capacity: i32,
        pool: &mut BufferPool,
    ) {
        if tree.leaves.len() < capacity {
            tree.resize(pool, capacity);
        }
        if leaves.len() < capacity {
            pool.resize_to_at_least(leaves, capacity, tree.leaf_count);
        }
    }

    fn resize_capacity_internal(
        tree: &mut Tree,
        leaves: &mut Buffer<CollidableReference>,
        capacity: i32,
        pool: &mut BufferPool,
    ) {
        let capacity = i32::max(capacity, tree.leaf_count);
        let target_leaf_capacity =
            BufferPool::get_capacity_for_count::<crate::physics::trees::leaf::Leaf>(capacity);
        if tree.leaves.len() != target_leaf_capacity {
            tree.resize(pool, capacity);
        }
        let target_ref_capacity =
            BufferPool::get_capacity_for_count::<CollidableReference>(capacity);
        if leaves.len() != target_ref_capacity {
            pool.resize_to_at_least(leaves, capacity, tree.leaf_count);
        }
    }

    fn dispose_internal(
        tree: &mut Tree,
        leaves: &mut Buffer<CollidableReference>,
        pool: &mut BufferPool,
    ) {
        pool.return_buffer(leaves);
        tree.dispose(pool);
    }

    /// Ensures the broad phase can hold at least the given number of leaves.
    pub fn ensure_capacity(&mut self, active_capacity: i32, static_capacity: i32) {
        let pool = unsafe { &mut *self.pool };
        Self::ensure_capacity_internal(
            &mut self.active_tree,
            &mut self.active_leaves,
            active_capacity,
            pool,
        );
        Self::ensure_capacity_internal(
            &mut self.static_tree,
            &mut self.static_leaves,
            static_capacity,
            pool,
        );
    }

    /// Resizes the broad phase structures. Never orphans existing leaves.
    pub fn resize(&mut self, active_capacity: i32, static_capacity: i32) {
        let pool = unsafe { &mut *self.pool };
        Self::resize_capacity_internal(
            &mut self.active_tree,
            &mut self.active_leaves,
            active_capacity,
            pool,
        );
        Self::resize_capacity_internal(
            &mut self.static_tree,
            &mut self.static_leaves,
            static_capacity,
            pool,
        );
    }

    /// Releases memory used by the broad phase. Leaves the broad phase unusable.
    pub fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        Self::dispose_internal(&mut self.active_tree, &mut self.active_leaves, pool);
        Self::dispose_internal(&mut self.static_tree, &mut self.static_leaves, pool);
    }

    // ---- Query methods (translated from BroadPhase_Queries.cs) ----

    /// Finds any intersections between a ray and leaf bounding boxes.
    pub unsafe fn ray_cast<TRayTester: IBroadPhaseRayTester>(
        &self,
        origin: Vec3,
        direction: Vec3,
        maximum_t: f32,
        ray_tester: &mut TRayTester,
        id: i32,
    ) {
        let mut ray_data = std::mem::MaybeUninit::<TreeRayData>::uninit();
        let mut tree_ray = std::mem::MaybeUninit::<TreeRay>::uninit();
        TreeRay::create_from_ray(
            origin, direction, maximum_t, id,
            &mut *ray_data.as_mut_ptr(),
            &mut *tree_ray.as_mut_ptr(),
        );
        let mut active_tester = RayLeafTester {
            leaf_tester: ray_tester as *mut TRayTester,
            leaves: &self.active_leaves,
        };
        self.active_tree.ray_cast_internal(
            tree_ray.as_mut_ptr(),
            ray_data.as_mut_ptr(),
            &mut active_tester,
        );
        let mut static_tester = RayLeafTester {
            leaf_tester: ray_tester as *mut TRayTester,
            leaves: &self.static_leaves,
        };
        self.static_tree.ray_cast_internal(
            tree_ray.as_mut_ptr(),
            ray_data.as_mut_ptr(),
            &mut static_tester,
        );
    }

    /// Finds any intersections between a swept bounding box and leaf bounding boxes.
    pub unsafe fn sweep_minmax<TSweepTester: IBroadPhaseSweepTester>(
        &self,
        min: Vec3,
        max: Vec3,
        direction: Vec3,
        maximum_t: f32,
        sweep_tester: &mut TSweepTester,
    ) {
        let mut origin = Vec3::ZERO;
        let mut expansion = Vec3::ZERO;
        Tree::convert_box_to_centroid_with_extent(min, max, &mut origin, &mut expansion);
        let mut tree_ray = std::mem::MaybeUninit::<TreeRay>::uninit();
        TreeRay::create_from(origin, direction, maximum_t, &mut *tree_ray.as_mut_ptr());
        let mut active_tester = SweepLeafTester {
            leaf_tester: sweep_tester as *mut TSweepTester,
            leaves: &self.active_leaves,
        };
        self.active_tree.sweep_internal(
            expansion, origin, direction,
            tree_ray.as_mut_ptr(),
            &mut active_tester,
        );
        let mut static_tester = SweepLeafTester {
            leaf_tester: sweep_tester as *mut TSweepTester,
            leaves: &self.static_leaves,
        };
        self.static_tree.sweep_internal(
            expansion, origin, direction,
            tree_ray.as_mut_ptr(),
            &mut static_tester,
        );
    }

    /// Finds any intersections between a swept bounding box and leaf bounding boxes.
    pub unsafe fn sweep_box<TSweepTester: IBroadPhaseSweepTester>(
        &self,
        bounding_box: &BoundingBox,
        direction: Vec3,
        maximum_t: f32,
        sweep_tester: &mut TSweepTester,
    ) {
        self.sweep_minmax(bounding_box.min, bounding_box.max, direction, maximum_t, sweep_tester);
    }

    /// Finds any overlaps between a bounding box and leaf bounding boxes.
    pub fn get_overlaps_minmax<TOverlapEnumerator: IBreakableForEach<CollidableReference>>(
        &self,
        min: Vec3,
        max: Vec3,
        overlap_enumerator: &mut TOverlapEnumerator,
    ) {
        let mut active_enumerator = BoxQueryEnumerator {
            enumerator: overlap_enumerator as *mut TOverlapEnumerator,
            leaves: &self.active_leaves,
        };
        self.active_tree.get_overlaps_minmax(min, max, &mut active_enumerator);
        let mut static_enumerator = BoxQueryEnumerator {
            enumerator: overlap_enumerator as *mut TOverlapEnumerator,
            leaves: &self.static_leaves,
        };
        self.static_tree.get_overlaps_minmax(min, max, &mut static_enumerator);
    }

    /// Finds any overlaps between a bounding box and leaf bounding boxes.
    pub fn get_overlaps<TOverlapEnumerator: IBreakableForEach<CollidableReference>>(
        &self,
        bounding_box: &BoundingBox,
        overlap_enumerator: &mut TOverlapEnumerator,
    ) {
        self.get_overlaps_minmax(bounding_box.min, bounding_box.max, overlap_enumerator);
    }
}

// ---- Internal adapter types for BroadPhase queries ----

struct RayLeafTester<'a, TRayTester: IBroadPhaseRayTester> {
    leaf_tester: *mut TRayTester,
    leaves: &'a Buffer<CollidableReference>,
}

impl<'a, TRayTester: IBroadPhaseRayTester> IRayLeafTester for RayLeafTester<'a, TRayTester> {
    #[inline(always)]
    unsafe fn test_leaf(&mut self, leaf_index: i32, ray_data: *mut TreeRayData, maximum_t: *mut f32) {
        let collidable = *self.leaves.get(leaf_index);
        (*self.leaf_tester).ray_test(collidable, ray_data as *const RayData, maximum_t);
    }
}

struct SweepLeafTester<'a, TSweepTester: IBroadPhaseSweepTester> {
    leaf_tester: *mut TSweepTester,
    leaves: &'a Buffer<CollidableReference>,
}

impl<'a, TSweepTester: IBroadPhaseSweepTester> ISweepLeafTester for SweepLeafTester<'a, TSweepTester> {
    #[inline(always)]
    fn test_leaf(&mut self, leaf_index: i32, maximum_t: &mut f32) {
        let collidable = *self.leaves.get(leaf_index);
        unsafe { (*self.leaf_tester).test(collidable, maximum_t) };
    }
}

struct BoxQueryEnumerator<'a, TInnerEnumerator: IBreakableForEach<CollidableReference>> {
    enumerator: *mut TInnerEnumerator,
    leaves: &'a Buffer<CollidableReference>,
}

impl<'a, TInnerEnumerator: IBreakableForEach<CollidableReference>> IBreakableForEach<i32>
    for BoxQueryEnumerator<'a, TInnerEnumerator>
{
    #[inline(always)]
    fn loop_body(&mut self, leaf_index: i32) -> bool {
        let collidable = *self.leaves.get(leaf_index);
        unsafe { (*self.enumerator).loop_body(collidable) }
    }
}
