// Translated from BepuPhysics/CollisionDetection/BroadPhase.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::ray_batchers::{IBroadPhaseRayTester, IBroadPhaseSweepTester, RayData};
use crate::physics::trees::ray_batcher::{RayData as TreeRayData, TreeRay};
use crate::physics::trees::tree::Tree;
use crate::physics::trees::tree_ray_cast::IRayLeafTester;
use crate::physics::trees::tree_sweep::ISweepLeafTester;
use crate::utilities::for_each_ref::IBreakableForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use crate::utilities::bounding_box::BoundingBox;
use glam::Vec3;

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
        let bounds = BoundingBox { min: *min, max: *max };
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

    /// Updates the broad phase trees (single-threaded path).
    pub fn update(&mut self, _thread_dispatcher: Option<*mut dyn IThreadDispatcher>) {
        if self.frame_index == i32::MAX {
            self.frame_index = 0;
        }
        // NOTE: Multi-threaded update path not yet implemented.
        // For now, single-threaded path only
        let pool = unsafe { &mut *self.pool };
        self.active_tree.refit_and_refine(pool, self.frame_index, 1.0);
        self.static_tree.refit_and_refine(pool, self.frame_index, 1.0);
        self.frame_index += 1;
    }

    /// Updates the broad phase trees using the Refine2/Refit2 pipeline with incremental refinement scheduling.
    pub fn update2(
        &mut self,
        _thread_dispatcher: Option<*mut dyn IThreadDispatcher>,
        _deterministic: bool,
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

        // NOTE: Multi-threaded path using TaskStack when dispatcher available and tree large enough.
        // Requires multi-threaded overloads of Tree::refine2 and Tree::refit2_with_cache_optimization
        // (currently omitted in tree_refine2.rs/tree_refit2.rs).
        // The MT path would:
        // 1. Distribute refine tasks proportional to cost (active vs static)
        // 2. Create RefinementContext for active + static trees
        // 3. Use TaskStack with ActiveEntrypointTask + StaticEntrypointTask
        // 4. Active: refine2 + refit2_with_cache_optimization from source nodes
        // 5. Static: refine2 only
        // 6. Copy back modified trees and start indices
        // const MINIMUM_LEAF_COUNT_FOR_THREADING: i32 = 256;

        // Single-threaded path.
        // Note: we refine *before* refitting. Refinement works with slightly out-of-date data,
        // but the point is incremental improvement. Refit with cache optimization *after* refinement
        // ensures the rest of the library sees the optimized layout.
        let pool = unsafe { &mut *self.pool };
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

        if self.frame_index == i32::MAX {
            self.frame_index = 0;
        } else {
            self.frame_index += 1;
        }
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
