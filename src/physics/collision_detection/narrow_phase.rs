// Translated from BepuPhysics/CollisionDetection/NarrowPhase.cs,
// NarrowPhaseCCDContinuations.cs, NarrowPhaseConstraintUpdate.cs,
// NarrowPhasePendingConstraintAdds.cs, NarrowPhasePreflush.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::collidables::collidable::{ContinuousDetection, ContinuousDetectionMode};
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::collision_detection::collision_batcher::{
    CollisionBatcher, ICollisionCallbacks,
};
use crate::physics::collision_detection::collision_batcher_continuations::PairContinuation;
use crate::physics::collision_detection::collision_task_registry::CollisionTaskRegistry;
use crate::physics::collision_detection::constraint_remover::ConstraintRemover;
use crate::physics::collision_detection::contact_constraint_accessor::{
    ContactConstraintAccessor, ISolverContactDataExtractor,
    ISolverContactPrestepAndImpulsesExtractor,
};
use crate::physics::collision_detection::contact_manifold::{
    Contact, ConvexContactManifold, IContactManifold, NonconvexContactManifold,
};
use crate::physics::collision_detection::continuation_index::CCDContinuationIndex;
use crate::physics::collision_detection::freshness_checker::FreshnessChecker;
use crate::physics::collision_detection::narrow_phase_callbacks::{
    INarrowPhaseCallbacks, PairMaterialProperties,
};
use crate::physics::collision_detection::pair_cache::{
    CollidablePair, ConstraintCache, PairCache, PairCacheChangeIndex,
};
use crate::physics::collision_detection::sweep_task_registry::{ISweepFilter, SweepTaskRegistry};
use crate::physics::collision_detection::untyped_list::UntypedList;
use crate::physics::constraints::constraint_description::IConstraintDescription;
use crate::physics::handles::ConstraintHandle;
use crate::physics::pose_integration::PoseIntegration;
use crate::physics::solver::Solver;
use crate::physics::statics::Statics;
use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::collections::quicksort::Quicksort;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use glam::Vec3;
use std::cell::UnsafeCell;
use std::mem;
use std::sync::atomic::{AtomicI32, Ordering};

// ============================================================================
// Data types from NarrowPhaseConstraintUpdate.cs
// ============================================================================

/// Associated with a pair of two collidables that each are controlled by bodies.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct TwoBodyHandles {
    pub a: i32,
    pub b: i32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses1 {
    pub impulse0: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses2 {
    pub impulse0: f32,
    pub impulse1: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses3 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses4 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
    pub impulse3: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses5 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
    pub impulse3: f32,
    pub impulse4: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses6 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
    pub impulse3: f32,
    pub impulse4: f32,
    pub impulse5: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses7 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
    pub impulse3: f32,
    pub impulse4: f32,
    pub impulse5: f32,
    pub impulse6: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactImpulses8 {
    pub impulse0: f32,
    pub impulse1: f32,
    pub impulse2: f32,
    pub impulse3: f32,
    pub impulse4: f32,
    pub impulse5: f32,
    pub impulse6: f32,
    pub impulse7: f32,
}

// ============================================================================
// NarrowPhaseFlushJob types
// ============================================================================

/// Types of narrow phase flush jobs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NarrowPhaseFlushJobType {
    RemoveConstraintsFromBodyLists,
    ReturnConstraintHandles,
    RemoveConstraintFromBatchReferencedHandles,
    RemoveConstraintsFromFallbackBatch,
    RemoveConstraintFromTypeBatch,
    FlushPairCacheChanges,
}

/// A flush job to execute during the narrow phase postprocessing.
#[derive(Clone, Copy, Debug)]
pub struct NarrowPhaseFlushJob {
    pub job_type: NarrowPhaseFlushJobType,
    pub index: i32,
}

// ============================================================================
// Preflush job types
// ============================================================================

/// Types of preflush jobs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PreflushJobType {
    /// Phase one job in the awakener. JobIndex used to identify sub-job.
    AwakenerPhaseOne,
    /// Sorts the constraints of a single type across all workers.
    SortContactConstraintType,
    /// Identifies a first guess at the constraint batch for each new constraint.
    SpeculativeConstraintBatchSearch,
    /// Adds constraints deterministically.
    DeterministicConstraintAdd,
    /// Adds constraints in manifold-generation order (nondeterministic).
    NondeterministicConstraintAdd,
    /// Phase two job in the awakener.
    AwakenerPhaseTwo,
    /// Check the freshness bytes in a region to remove stale pairs.
    CheckFreshness,
}

/// A preflush job to execute during preflush multithreaded dispatch.
#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct PreflushJob {
    pub job_type: PreflushJobType,
    /// Start region (CheckFreshness, SpeculativeConstraintBatchSearch).
    pub start: i32,
    /// End region (CheckFreshness, SpeculativeConstraintBatchSearch).
    pub end: i32,
    /// Narrow phase constraint type index (SpeculativeConstraintBatchSearch, SortContactConstraintType).
    pub type_index: i32,
    /// Index of the worker (SpeculativeConstraintBatchSearch) or worker count (Sort/NondeterministicAdd).
    pub worker_index_or_count: i32,
    /// Index of the job (AwakenerPhaseOne/Two).
    pub job_index: i32,
}

impl Default for PreflushJob {
    fn default() -> Self {
        Self {
            job_type: PreflushJobType::AwakenerPhaseOne,
            start: 0,
            end: 0,
            type_index: 0,
            worker_index_or_count: 0,
            job_index: 0,
        }
    }
}

/// Sort target for deterministic constraint adds.
#[derive(Clone, Copy, Default)]
pub struct SortConstraintTarget {
    pub worker_index: i32,
    pub byte_index_in_cache: i32,
    pub sort_key: u64,
}

/// Comparer for sorting pending constraints by their sort key.
struct PendingConstraintComparer;

impl RefComparer<SortConstraintTarget> for PendingConstraintComparer {
    fn compare(&self, a: &SortConstraintTarget, b: &SortConstraintTarget) -> std::cmp::Ordering {
        a.sort_key.cmp(&b.sort_key)
    }
}

// ============================================================================
// ContinuationCache — per-type pool for CCD continuations
// ============================================================================

/// Cache of CCD continuations, backed by an IdPool and a Buffer.
struct ContinuationCache<T: Copy> {
    ids: IdPool,
    caches: Buffer<T>,
}

impl<T: Copy> ContinuationCache<T> {
    fn new(pool: &mut BufferPool) -> Self {
        let ids = IdPool::new(32, pool);
        let caches: Buffer<T> = pool.take_at_least(128);
        Self { ids, caches }
    }

    #[inline(always)]
    fn allocate(&mut self, pool: &mut BufferPool) -> (i32, *mut T) {
        let index = self.ids.take();
        if self.caches.len() <= index {
            let old_len = self.caches.len();
            pool.resize_to_at_least(&mut self.caches, index + 1, old_len);
        }
        let ptr = unsafe { self.caches.get_mut(index) as *mut T };
        (index, ptr)
    }

    #[inline(always)]
    fn return_index(&mut self, index: i32, pool: &mut BufferPool) {
        self.ids.return_id(index, pool);
    }

    fn dispose(&mut self, pool: &mut BufferPool) {
        self.ids.dispose(pool);
        pool.return_buffer(&mut self.caches);
    }
}

// ============================================================================
// DiscretePair / ContinuousPair — per-continuation data
// ============================================================================

/// Data for a discrete collision pair continuation.
#[repr(C)]
#[derive(Clone, Copy)]
struct DiscretePair {
    pair: CollidablePair,
}

/// Data for a continuous (CCD) collision pair continuation.
#[repr(C)]
#[derive(Clone, Copy)]
struct ContinuousPair {
    pair: CollidablePair,
    relative_linear_velocity: Vec3,
    angular_a: Vec3,
    angular_b: Vec3,
    t: f32,
}

/// Constraint generator type for CCD continuations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum ConstraintGeneratorType {
    /// Pair which will directly produce constraints.
    Discrete = 0,
    /// Pair which samples a swept location for contacts and needs depth rewind.
    Continuous = 1,
}

// ============================================================================
// CollisionCallbacks — implements ICollisionCallbacks for the NarrowPhase
// ============================================================================

/// Collision callbacks for the narrow phase, managing CCD continuations and
/// dispatching completed pair results to constraint creation.
pub struct CollisionCallbacks<TCallbacks: INarrowPhaseCallbacks> {
    worker_index: i32,
    pool: *mut BufferPool,
    narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    discrete: ContinuationCache<DiscretePair>,
    continuous: ContinuationCache<ContinuousPair>,
}

impl<TCallbacks: INarrowPhaseCallbacks> CollisionCallbacks<TCallbacks> {
    pub fn new(
        worker_index: i32,
        pool: *mut BufferPool,
        narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        Self {
            worker_index,
            pool,
            narrow_phase,
            discrete: ContinuationCache::new(pool_ref),
            continuous: ContinuationCache::new(pool_ref),
        }
    }

    /// Adds a discrete pair continuation. Returns the CCDContinuationIndex to use as pair id.
    #[inline(always)]
    pub fn add_discrete(&mut self, pair: &CollidablePair) -> CCDContinuationIndex {
        let pool_ref = unsafe { &mut *self.pool };
        let (index, ptr) = self.discrete.allocate(pool_ref);
        unsafe {
            (*ptr).pair = *pair;
        }
        CCDContinuationIndex::new(ConstraintGeneratorType::Discrete as i32, index)
    }

    /// Adds a continuous pair continuation. Returns the CCDContinuationIndex to use as pair id.
    #[inline(always)]
    pub fn add_continuous(
        &mut self,
        pair: &CollidablePair,
        relative_linear_velocity: Vec3,
        angular_velocity_a: Vec3,
        angular_velocity_b: Vec3,
        t: f32,
    ) -> CCDContinuationIndex {
        let pool_ref = unsafe { &mut *self.pool };
        let (index, ptr) = self.continuous.allocate(pool_ref);
        unsafe {
            (*ptr).pair = *pair;
            (*ptr).relative_linear_velocity = relative_linear_velocity;
            (*ptr).angular_a = angular_velocity_a;
            (*ptr).angular_b = angular_velocity_b;
            (*ptr).t = t;
        }
        CCDContinuationIndex::new(ConstraintGeneratorType::Continuous as i32, index)
    }

    /// Gets the collidable pair from a packed continuation id.
    #[inline(always)]
    fn get_collidable_pair(&self, pair_id: i32) -> CollidablePair {
        let continuation = CCDContinuationIndex::from_packed(pair_id);
        debug_assert!(continuation.exists());
        let index = continuation.index();
        match continuation.type_index() {
            0 => unsafe { (*self.discrete.caches.get(index)).pair },
            1 => unsafe { (*self.continuous.caches.get(index)).pair },
            _ => {
                debug_assert!(
                    false,
                    "Invalid collision continuation type. Corrupted data?"
                );
                CollidablePair::default()
            }
        }
    }

    pub(crate) fn dispose(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        self.discrete.dispose(pool_ref);
        self.continuous.dispose(pool_ref);
    }
}

impl<TCallbacks: INarrowPhaseCallbacks> ICollisionCallbacks for CollisionCallbacks<TCallbacks> {
    fn on_pair_completed<TManifold: IContactManifold>(
        &mut self,
        pair_id: i32,
        manifold: &mut TManifold,
    ) {
        let continuation_id = CCDContinuationIndex::from_packed(pair_id);
        debug_assert!(continuation_id.exists());
        let continuation_index = continuation_id.index();
        let pool_ref = unsafe { &mut *self.pool };

        match continuation_id.type_index() {
            // Discrete
            0 => {
                let continuation = unsafe { *self.discrete.caches.get(continuation_index) };
                let np = unsafe { &mut *self.narrow_phase };
                np.update_constraints_for_pair(self.worker_index, continuation.pair, manifold);
                self.discrete.return_index(continuation_index, pool_ref);
            }
            // Continuous
            1 => {
                let continuation = unsafe { *self.continuous.caches.get(continuation_index) };
                // Rewind depths from the future pose (at time t) back to the current timestep.
                // The manifold we received is for a future point in time.
                // We need to rewind it to the timestep start for consistency.
                if manifold.convex() {
                    // Safety: TManifold is ConvexContactManifold
                    let m =
                        unsafe { &mut *(manifold as *mut TManifold as *mut ConvexContactManifold) };
                    let offset_b = m.offset_b;
                    let normal = m.normal;
                    for i in 0..m.count {
                        let contact = unsafe { m.get_contact_mut(i) };
                        let angular_contribution_a = continuation.angular_a.cross(contact.offset);
                        let angular_contribution_b =
                            continuation.angular_b.cross(contact.offset - offset_b);
                        let velocity_at_contact = (angular_contribution_b - angular_contribution_a
                            + continuation.relative_linear_velocity)
                            .dot(normal);
                        contact.depth -= velocity_at_contact * continuation.t;
                    }
                } else {
                    // Safety: TManifold is NonconvexContactManifold
                    let m = unsafe {
                        &mut *(manifold as *mut TManifold as *mut NonconvexContactManifold)
                    };
                    for i in 0..m.count {
                        let contact =
                            unsafe { &mut *(&mut m.contact0 as *mut Contact).add(i as usize) };
                        let angular_contribution_a = continuation.angular_a.cross(contact.offset);
                        let angular_contribution_b =
                            continuation.angular_b.cross(contact.offset - m.offset_b);
                        let velocity_at_contact = (angular_contribution_b - angular_contribution_a
                            + continuation.relative_linear_velocity)
                            .dot(contact.normal);
                        contact.depth -= velocity_at_contact * continuation.t;
                    }
                }
                let np = unsafe { &mut *self.narrow_phase };
                np.update_constraints_for_pair(self.worker_index, continuation.pair, manifold);
                self.continuous.return_index(continuation_index, pool_ref);
            }
            _ => {
                debug_assert!(false, "Invalid collision continuation type.");
            }
        }
    }

    fn on_child_pair_completed(
        &mut self,
        pair_id: i32,
        child_a: i32,
        child_b: i32,
        manifold: &mut ConvexContactManifold,
    ) {
        let np = unsafe { &*self.narrow_phase };
        let keep_manifold = np.callbacks.configure_child_contact_manifold(
            self.worker_index,
            self.get_collidable_pair(pair_id),
            child_a,
            child_b,
            manifold,
        );
        if !keep_manifold {
            manifold.count = 0;
        }
    }

    fn allow_collision_testing(&self, pair_id: i32, child_a: i32, child_b: i32) -> bool {
        let np = unsafe { &*self.narrow_phase };
        np.callbacks.allow_contact_generation_for_children(
            self.worker_index,
            self.get_collidable_pair(pair_id),
            child_a,
            child_b,
        )
    }
}

// ============================================================================
// PendingConstraintAddCache
// ============================================================================

/// Per-worker cache of pending constraint additions.
/// Constraints are buffered per-type and flushed after overlap detection completes.
pub struct PendingConstraintAddCache {
    pool: *mut BufferPool,
    pub(crate) pending_constraints_by_type: Buffer<UntypedList>,
    pub(crate) speculative_batch_indices: Buffer<Buffer<u16>>,
    minimum_constraint_count_per_cache: i32,
}

impl PendingConstraintAddCache {
    pub fn new(pool: *mut BufferPool, minimum_constraint_count_per_cache: i32) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let mut pending: Buffer<UntypedList> =
            pool_ref.take_at_least(PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
        pending.clear(0, PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
        Self {
            pool,
            pending_constraints_by_type: pending,
            speculative_batch_indices: Buffer::default(),
            minimum_constraint_count_per_cache: minimum_constraint_count_per_cache,
        }
    }

    /// Adds a pending constraint of the given type.
    pub unsafe fn add_constraint<TBodyHandles: Copy, TDescription: Copy, TContactImpulses: Copy>(
        &mut self,
        manifold_constraint_type: i32,
        pair: CollidablePair,
        pair_cache_change: PairCacheChangeIndex,
        body_handles: TBodyHandles,
        description: &TDescription,
        impulses: &TContactImpulses,
    ) {
        // PendingConstraint layout: CollidablePair, PairCacheChangeIndex, TBodyHandles, TDescription, TContactImpulses
        // (packed sequentially for sorting by CollidablePair)
        let pending_size = mem::size_of::<CollidablePair>()
            + mem::size_of::<PairCacheChangeIndex>()
            + mem::size_of::<TBodyHandles>()
            + mem::size_of::<TDescription>()
            + mem::size_of::<TContactImpulses>();

        let pool_ref = &mut *self.pool;
        let cache = self
            .pending_constraints_by_type
            .get_mut(manifold_constraint_type);
        let byte_index = cache.allocate(
            pending_size as i32,
            self.minimum_constraint_count_per_cache,
            pool_ref,
        );

        let base = cache.buffer.as_ptr().add(byte_index as usize);
        let mut offset = 0usize;

        // CollidablePair
        std::ptr::write(base.add(offset) as *mut CollidablePair, pair);
        offset += mem::size_of::<CollidablePair>();

        // PairCacheChangeIndex
        std::ptr::write(
            base.add(offset) as *mut PairCacheChangeIndex,
            pair_cache_change,
        );
        offset += mem::size_of::<PairCacheChangeIndex>();

        // TBodyHandles
        std::ptr::write(base.add(offset) as *mut TBodyHandles, body_handles);
        offset += mem::size_of::<TBodyHandles>();

        // TDescription
        std::ptr::write(base.add(offset) as *mut TDescription, *description);
        offset += mem::size_of::<TDescription>();

        // TContactImpulses
        std::ptr::write(base.add(offset) as *mut TContactImpulses, *impulses);
    }

    /// Flushes pending constraints into the simulation sequentially.
    pub(crate) fn flush_sequentially(
        &mut self,
        narrow_phase: &NarrowPhase,
        solver: &mut Solver,
        pair_cache: &mut PairCache,
    ) {
        for i in 0..narrow_phase.contact_constraint_accessors.len() {
            if let Some(ref accessor) = narrow_phase.contact_constraint_accessors[i] {
                let list = unsafe { self.pending_constraints_by_type.get_mut(i as i32) };
                accessor.flush_sequentially(
                    list,
                    i as i32,
                    narrow_phase as *const NarrowPhase,
                    solver,
                    pair_cache,
                );
            }
        }
    }

    /// Performs speculative batch search for a range of constraints of a given type.
    #[inline(always)]
    pub(crate) unsafe fn speculative_constraint_batch_search(
        &mut self,
        solver: &Solver,
        type_index: i32,
        start: i32,
        end: i32,
    ) {
        let list = self.pending_constraints_by_type.get(type_index);
        debug_assert!(list.buffer.allocated());
        debug_assert!(list.count > 0);

        let element_size = list.element_size_in_bytes;
        let mut byte_index = start as usize * element_size as usize;
        let speculative = self.speculative_batch_indices.get_mut(type_index);
        for i in start..end {
            let pair_ptr = list.buffer.as_ptr().add(byte_index) as *const CollidablePair;
            *speculative.get_mut(i) = solver.find_candidate_batch(&*pair_ptr) as u16;
            byte_index += element_size as usize;
        }
    }

    /// Allocates buffers for speculative batch search.
    pub(crate) fn allocate_for_speculative_search(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        self.speculative_batch_indices =
            pool_ref.take_at_least(PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
        self.speculative_batch_indices
            .clear(0, PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
        for i in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
            let type_list = unsafe { self.pending_constraints_by_type.get(i) };
            if type_list.buffer.allocated() {
                debug_assert!(type_list.count > 0);
                let indices: Buffer<u16> = pool_ref.take_at_least(type_list.count as i32);
                unsafe {
                    *self.speculative_batch_indices.get_mut(i) = indices;
                }
            }
        }
    }

    /// Disposes speculative search buffers.
    pub(crate) fn dispose_speculative_search(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        for i in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
            let indices = unsafe { self.speculative_batch_indices.get_mut(i) };
            if indices.allocated() {
                pool_ref.return_buffer(indices);
            }
        }
        pool_ref.return_buffer(&mut self.speculative_batch_indices);
    }

    /// Counts total pending constraints across all types.
    pub(crate) fn count_constraints(&self) -> i32 {
        let mut count = 0;
        for i in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
            count += unsafe { self.pending_constraints_by_type.get(i).count as i32 };
        }
        count
    }

    /// Flushes pending constraints using speculative batch indices. Nondeterministic order.
    pub(crate) fn flush_with_speculative_batches(
        &mut self,
        accessors: &[Option<Box<dyn ContactConstraintAccessor>>],
        narrow_phase: *const NarrowPhase,
        solver: &mut Solver,
        pair_cache: &mut PairCache,
    ) {
        for i in 0..accessors.len() {
            if let Some(ref accessor) = accessors[i] {
                let list = unsafe { self.pending_constraints_by_type.get_mut(i as i32) };
                let speculative = &mut self.speculative_batch_indices;
                accessor.flush_with_speculative_batches(
                    list,
                    i as i32,
                    speculative,
                    narrow_phase,
                    solver,
                    pair_cache,
                );
            }
        }
    }

    /// Disposes all pending constraint buffers.
    pub fn dispose(&mut self) {
        let pool_ref = unsafe { &mut *self.pool };
        for i in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
            let list = unsafe { self.pending_constraints_by_type.get_mut(i) };
            if list.buffer.allocated() {
                pool_ref.return_buffer(&mut list.buffer);
            }
        }
        pool_ref.return_buffer(&mut self.pending_constraints_by_type);
    }
}

// ============================================================================
// OverlapWorker — per-thread worker for NarrowPhase overlap processing
// ============================================================================

/// Per-thread worker holding a collision batcher and pending constraint adds.
pub struct OverlapWorker<TCallbacks: INarrowPhaseCallbacks> {
    pub batcher: CollisionBatcher<CollisionCallbacks<TCallbacks>>,
    pub pending_constraints: PendingConstraintAddCache,
    pub pending_set_awakenings: QuickList<i32>,
}

impl<TCallbacks: INarrowPhaseCallbacks> OverlapWorker<TCallbacks> {
    pub fn new(
        worker_index: i32,
        pool: *mut BufferPool,
        narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    ) -> Self {
        let np = unsafe { &*narrow_phase };
        let batcher = CollisionBatcher::new(
            pool,
            np.base.shapes,
            np.base.collision_task_registry,
            np.base.timestep_duration,
            CollisionCallbacks::new(worker_index, pool, narrow_phase),
        );
        let pending_constraints = PendingConstraintAddCache::new(pool, 128);
        let pending_set_awakenings = QuickList::with_capacity(16, unsafe { &mut *pool });
        Self {
            batcher,
            pending_constraints,
            pending_set_awakenings,
        }
    }
}

// ============================================================================
// NarrowPhase (base, non-generic)
// ============================================================================

/// Turns broad phase overlaps into contact manifolds and uses them to manage constraints in the solver.
pub struct NarrowPhase {
    pub pool: *mut BufferPool,
    pub bodies: *mut Bodies,
    pub statics: *mut Statics,
    pub solver: *mut Solver,
    pub shapes: *mut crate::physics::collidables::shapes::Shapes,
    pub sweep_task_registry: *mut SweepTaskRegistry,
    pub collision_task_registry: *mut CollisionTaskRegistry,
    pub broad_phase: *mut BroadPhase,
    pub constraint_remover: ConstraintRemover,
    pub(crate) freshness_checker: Option<FreshnessChecker>,
    pub pair_cache: PairCache,
    pub(crate) timestep_duration: f32,
    pub(crate) awakener: *mut crate::physics::island_awakener::IslandAwakener,

    pub(crate) contact_constraint_accessors: Vec<Option<Box<dyn ContactConstraintAccessor>>>,

    /// Per-worker pending set awakening lists and associated buffer pool pointers.
    /// These are set by NarrowPhaseGeneric when overlap_workers are created.
    /// Each entry is (pending_set_awakenings_ptr, buffer_pool_ptr).
    /// Used by update_constraint_core to enqueue set awakenings without needing
    /// access to the generic overlap_workers field.
    pub(crate) worker_awakening_ptrs: Vec<(*mut QuickList<i32>, *mut BufferPool)>,

    /// Per-worker pending constraint caches.
    /// Set by NarrowPhaseGeneric::prepare to allow type-erased constraint addition.
    pub(crate) worker_pending_constraints: Vec<*mut PendingConstraintAddCache>,

    /// Atomic index for MT flush job dispatch.
    pub(crate) flush_job_index: UnsafeCell<i32>,
    /// Flush jobs stored for MT dispatch (valid only during flush).
    pub(crate) flush_jobs: Vec<NarrowPhaseFlushJob>,
}

impl NarrowPhase {
    /// Creates a new NarrowPhase.
    pub fn new(
        pool: *mut BufferPool,
        bodies: *mut Bodies,
        statics: *mut Statics,
        solver: *mut Solver,
        shapes: *mut crate::physics::collidables::shapes::Shapes,
        collision_task_registry: *mut CollisionTaskRegistry,
        sweep_task_registry: *mut SweepTaskRegistry,
        broad_phase: *mut BroadPhase,
        constraint_remover: ConstraintRemover,
        initial_set_capacity: i32,
        minimum_mapping_size: i32,
        minimum_pending_size: i32,
    ) -> Self {
        let pair_cache = PairCache::new(
            pool,
            initial_set_capacity,
            minimum_mapping_size,
            minimum_pending_size,
        );
        Self {
            pool,
            bodies,
            statics,
            solver,
            shapes,
            sweep_task_registry,
            collision_task_registry,
            broad_phase,
            constraint_remover,
            freshness_checker: None,
            pair_cache,
            timestep_duration: 0.0,
            awakener: std::ptr::null_mut(),
            contact_constraint_accessors: Vec::new(),
            worker_awakening_ptrs: Vec::new(),
            worker_pending_constraints: Vec::new(),
            flush_job_index: UnsafeCell::new(-1),
            flush_jobs: Vec::new(),
        }
    }

    /// Registers a contact constraint accessor for a given constraint type.
    pub fn register_contact_constraint_accessor(
        &mut self,
        accessor: Box<dyn ContactConstraintAccessor>,
    ) {
        let id = accessor.constraint_type_id() as usize;
        if self.contact_constraint_accessors.len() <= id {
            self.contact_constraint_accessors
                .resize_with(id + 1, || None);
        }
        assert!(
            self.contact_constraint_accessors[id].is_none(),
            "Cannot register accessor for type id {}; it is already registered.",
            id
        );
        self.contact_constraint_accessors[id] = Some(accessor);
    }

    /// Looks up the contact constraint accessor for the given constraint type id if it exists.
    pub fn try_get_contact_constraint_accessor(
        &self,
        constraint_type_id: i32,
    ) -> Option<&dyn ContactConstraintAccessor> {
        if Self::is_contact_constraint_type(constraint_type_id)
            && (constraint_type_id as usize) < self.contact_constraint_accessors.len()
        {
            self.contact_constraint_accessors[constraint_type_id as usize]
                .as_ref()
                .map(|a| a.as_ref())
        } else {
            None
        }
    }

    /// Gets whether a constraint type id maps to a contact constraint.
    #[inline(always)]
    pub fn is_contact_constraint_type(constraint_type_id: i32) -> bool {
        debug_assert!(constraint_type_id >= 0);
        constraint_type_id < PairCache::COLLISION_CONSTRAINT_TYPE_COUNT
    }

    /// Tries to extract solver contact data from a constraint. Returns true if the constraint
    /// type is a contact constraint and was successfully extracted.
    pub fn try_extract_solver_contact_data(
        &self,
        constraint_handle: ConstraintHandle,
        extractor: &mut dyn ISolverContactDataExtractor,
    ) -> bool {
        let solver = unsafe { &*self.solver };
        let constraint_location = solver.handle_to_constraint.get(constraint_handle.0);
        if let Some(accessor) =
            self.try_get_contact_constraint_accessor(constraint_location.type_id)
        {
            accessor.extract_contact_data(constraint_location, solver, extractor);
            true
        } else {
            false
        }
    }

    /// Tries to extract solver contact prestep and impulse data from a constraint.
    /// Returns true if the constraint type is a contact constraint and was successfully extracted.
    pub fn try_extract_solver_contact_prestep_and_impulses(
        &self,
        constraint_handle: ConstraintHandle,
        extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
    ) -> bool {
        let solver = unsafe { &*self.solver };
        let constraint_location = solver.handle_to_constraint.get(constraint_handle.0);
        if let Some(accessor) =
            self.try_get_contact_constraint_accessor(constraint_location.type_id)
        {
            accessor.extract_contact_prestep_and_impulses(constraint_location, solver, extractor);
            true
        } else {
            false
        }
    }

    /// Prepares the narrow phase for a new timestep.
    pub fn prepare(&mut self, dt: f32, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        self.timestep_duration = dt;
        self.pair_cache.prepare(
            thread_dispatcher
                .map(|d| d as *const dyn IThreadDispatcher as *mut dyn IThreadDispatcher),
        );
        self.constraint_remover.prepare(thread_dispatcher);
    }

    /// Sorts references to guarantee that two collidables in the same pair will always be in the same order.
    #[inline(always)]
    pub fn sort_collidable_references_for_pair(
        a: CollidableReference,
        b: CollidableReference,
    ) -> (
        CollidableMobility,
        CollidableMobility,
        CollidableReference,
        CollidableReference,
    ) {
        let a_mobility = a.mobility();
        let b_mobility = b.mobility();
        if (a_mobility != CollidableMobility::Static
            && b_mobility != CollidableMobility::Static
            && a.body_handle().0 > b.body_handle().0)
            || a_mobility == CollidableMobility::Static
        {
            (b_mobility, a_mobility, b, a)
        } else {
            (a_mobility, b_mobility, a, b)
        }
    }

    /// Executes a single flush job.
    fn execute_flush_job(&mut self, job: &NarrowPhaseFlushJob) {
        match job.job_type {
            NarrowPhaseFlushJobType::RemoveConstraintsFromBodyLists => {
                self.constraint_remover.remove_constraints_from_body_lists();
            }
            NarrowPhaseFlushJobType::ReturnConstraintHandles => {
                self.constraint_remover.return_constraint_handles();
            }
            NarrowPhaseFlushJobType::RemoveConstraintFromBatchReferencedHandles => {
                self.constraint_remover
                    .remove_constraints_from_batch_referenced_handles();
            }
            NarrowPhaseFlushJobType::RemoveConstraintsFromFallbackBatch => {
                self.constraint_remover
                    .remove_constraints_from_fallback_batch_referenced_handles();
            }
            NarrowPhaseFlushJobType::RemoveConstraintFromTypeBatch => {
                self.constraint_remover
                    .remove_constraints_from_type_batch(job.index);
            }
            NarrowPhaseFlushJobType::FlushPairCacheChanges => {
                self.pair_cache.flush_mapping_changes();
            }
        }
    }

    /// Flushes all pending narrow phase work.
    pub fn flush(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        self.flush_jobs = Vec::with_capacity(128);

        self.pair_cache.prepare_flush_jobs(&mut self.flush_jobs);
        let removal_batch_job_count = self.constraint_remover.create_flush_jobs(deterministic);

        self.flush_jobs.push(NarrowPhaseFlushJob {
            job_type: NarrowPhaseFlushJobType::RemoveConstraintsFromBodyLists,
            index: 0,
        });
        self.flush_jobs.push(NarrowPhaseFlushJob {
            job_type: NarrowPhaseFlushJobType::ReturnConstraintHandles,
            index: 0,
        });
        self.flush_jobs.push(NarrowPhaseFlushJob {
            job_type: NarrowPhaseFlushJobType::RemoveConstraintFromBatchReferencedHandles,
            index: 0,
        });

        // Add fallback batch removal if the solver has enough batches.
        unsafe {
            let solver = &*self.solver;
            if solver.active_set().batches.count > solver.fallback_batch_threshold() {
                self.flush_jobs.push(NarrowPhaseFlushJob {
                    job_type: NarrowPhaseFlushJobType::RemoveConstraintsFromFallbackBatch,
                    index: 0,
                });
            }
        }

        for i in 0..removal_batch_job_count {
            self.flush_jobs.push(NarrowPhaseFlushJob {
                job_type: NarrowPhaseFlushJobType::RemoveConstraintFromTypeBatch,
                index: i,
            });
        }

        if let Some(dispatcher) = thread_dispatcher {
            // Multithreaded: use atomic job index for work stealing.
            unsafe {
                *self.flush_job_index.get() = -1;
            }
            let self_ptr = self as *mut NarrowPhase as *mut ();
            unsafe {
                dispatcher.dispatch_workers(
                    Self::flush_worker_loop_fn,
                    self.flush_jobs.len() as i32,
                    self_ptr,
                    None,
                );
            }
        } else {
            // Single-threaded: execute all jobs sequentially.
            for i in 0..self.flush_jobs.len() {
                let job = self.flush_jobs[i];
                self.execute_flush_job(&job);
            }
        }

        self.flush_jobs.clear();
        self.pair_cache.postflush();
        self.constraint_remover
            .mark_affected_constraints_as_removed_from_solver();
        self.constraint_remover.postflush();
    }

    /// Worker loop function for multithreaded flush dispatch.
    /// Passed as a function pointer to `dispatch_workers`.
    fn flush_worker_loop_fn(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        let _ = worker_index;
        unsafe {
            let narrow_phase = &mut *(dispatcher.unmanaged_context() as *mut NarrowPhase);
            loop {
                let job_index = AtomicI32::from_ptr(narrow_phase.flush_job_index.get())
                    .fetch_add(1, Ordering::AcqRel)
                    + 1;
                if job_index >= narrow_phase.flush_jobs.len() as i32 {
                    break;
                }
                let job = narrow_phase.flush_jobs[job_index as usize];
                narrow_phase.execute_flush_job(&job);
            }
        }
    }

    /// Clears all narrow phase data.
    pub fn clear(&mut self) {
        self.pair_cache.clear();
    }

    /// Disposes of all narrow phase resources.
    pub fn dispose(&mut self) {
        self.pair_cache.dispose();
    }

    /// Extracts the body count (1 or 2) from a contact constraint type ID.
    #[inline(always)]
    pub fn extract_contact_constraint_body_count(contact_constraint_type_id: i32) -> i32 {
        debug_assert!(
            contact_constraint_type_id >= 0
                && contact_constraint_type_id < PairCache::COLLISION_CONSTRAINT_TYPE_COUNT
        );
        // [0, 3] and [8, 14] are one body. Otherwise, two.
        if contact_constraint_type_id <= 3
            || (contact_constraint_type_id >= 8 && contact_constraint_type_id <= 14)
        {
            1
        } else {
            2
        }
    }
}

// ============================================================================
// NarrowPhaseGeneric<TCallbacks> — generic wrapper (maps to C# NarrowPhase<TCallbacks>)
// ============================================================================

/// Generic narrow phase with user-defined callbacks.
/// This wraps the base NarrowPhase and adds callback-specific behavior.
/// Note: #[repr(C)] ensures `base` is at offset 0 so that type-erased pointers
/// (from contact constraint accessors) can safely access NarrowPhase fields.
#[repr(C)]
pub struct NarrowPhaseGeneric<TCallbacks: INarrowPhaseCallbacks> {
    pub base: NarrowPhase,
    pub callbacks: TCallbacks,
    pub(crate) overlap_workers: Vec<OverlapWorker<TCallbacks>>,
    /// Sorted constraint targets for deterministic preflush.
    sorted_constraints: Buffer<QuickList<SortConstraintTarget>>,
    /// Job index for multithreaded preflush (accessed via Interlocked.Increment in C#).
    preflush_job_index: UnsafeCell<i32>,
    /// List of preflush jobs for multithreaded dispatch.
    preflush_jobs: QuickList<PreflushJob>,
}

/// Filter for CCD sweep tests that delegates to the narrow phase callbacks.
struct CCDSweepFilter<TCallbacks: INarrowPhaseCallbacks> {
    narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    pair: CollidablePair,
    worker_index: i32,
}

impl<TCallbacks: INarrowPhaseCallbacks> ISweepFilter for CCDSweepFilter<TCallbacks> {
    #[inline(always)]
    fn allow_test(&self, child_a: i32, child_b: i32) -> bool {
        unsafe {
            (*self.narrow_phase)
                .callbacks
                .allow_contact_generation_for_children(
                    self.worker_index,
                    self.pair,
                    child_a,
                    child_b,
                )
        }
    }
}

impl<TCallbacks: INarrowPhaseCallbacks> NarrowPhaseGeneric<TCallbacks> {
    /// Creates a new generic narrow phase.
    pub fn new(
        pool: *mut BufferPool,
        bodies: *mut Bodies,
        statics: *mut Statics,
        solver: *mut Solver,
        shapes: *mut crate::physics::collidables::shapes::Shapes,
        collision_task_registry: *mut CollisionTaskRegistry,
        sweep_task_registry: *mut SweepTaskRegistry,
        broad_phase: *mut BroadPhase,
        constraint_remover: ConstraintRemover,
        callbacks: TCallbacks,
        initial_set_capacity: i32,
        minimum_mapping_size: i32,
        minimum_pending_size: i32,
    ) -> Self {
        let base = NarrowPhase::new(
            pool,
            bodies,
            statics,
            solver,
            shapes,
            collision_task_registry,
            sweep_task_registry,
            broad_phase,
            constraint_remover,
            initial_set_capacity,
            minimum_mapping_size,
            minimum_pending_size,
        );
        Self {
            base,
            callbacks,
            overlap_workers: Vec::new(),
            sorted_constraints: Buffer::default(),
            preflush_job_index: UnsafeCell::new(-1),
            preflush_jobs: QuickList::default(),
        }
    }

    // --- Lifecycle ---

    /// Prepares the narrow phase for a new timestep, creating overlap workers.
    pub fn prepare(&mut self, dt: f32, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        self.base.prepare(dt, thread_dispatcher);

        let thread_count = thread_dispatcher.map_or(1, |d| d.thread_count());
        if self.overlap_workers.len() < thread_count as usize {
            self.overlap_workers.resize_with(thread_count as usize, || {
                // Placeholder — will be overwritten below
                unsafe { mem::zeroed() }
            });
        }

        let self_ptr = self as *mut NarrowPhaseGeneric<TCallbacks>;
        for i in 0..thread_count as usize {
            let pool = if let Some(d) = thread_dispatcher {
                d.worker_pool_ptr(i as i32)
            } else {
                self.base.pool
            };
            self.overlap_workers[i] = OverlapWorker::new(i as i32, pool, self_ptr);
        }

        // Populate worker_awakening_ptrs so the type-erased update_constraint_core can
        // enqueue set awakenings without needing access to the generic overlap_workers.
        self.base.worker_awakening_ptrs.clear();
        self.base.worker_pending_constraints.clear();
        for i in 0..thread_count as usize {
            let worker = &mut self.overlap_workers[i];
            let awakenings_ptr = &mut worker.pending_set_awakenings as *mut QuickList<i32>;
            let pool_ptr = worker.batcher.pool;
            self.base
                .worker_awakening_ptrs
                .push((awakenings_ptr, pool_ptr));
            self.base
                .worker_pending_constraints
                .push(&mut worker.pending_constraints as *mut PendingConstraintAddCache);
        }
    }

    /// Postflush: disposes collision callbacks.
    pub fn postflush(&mut self, thread_dispatcher: Option<&dyn IThreadDispatcher>) {
        let thread_count = thread_dispatcher.map_or(1, |d| d.thread_count());
        for i in 0..thread_count as usize {
            self.overlap_workers[i].batcher.callbacks.dispose();
        }
    }

    /// Disposes user callbacks.
    pub fn dispose(&mut self) {
        self.base.dispose();
        self.callbacks.dispose();
    }

    // ========================================================================
    // Constraint update methods (from NarrowPhaseConstraintUpdate.cs)
    // ========================================================================

    /// Redistributes impulses from old contacts to new contacts by matching feature IDs.
    /// Unmatched impulse is distributed evenly among unmatched new contacts.
    unsafe fn redistribute_impulses(
        old_contact_count: i32,
        old_feature_ids: *mut i32,
        old_impulses: *mut f32,
        new_contact_count: i32,
        new_feature_ids: *mut i32,
        new_impulses: *mut f32,
    ) {
        // Delegate to the standalone function.
        redistribute_impulses(
            old_contact_count,
            old_feature_ids,
            old_impulses,
            new_contact_count,
            new_feature_ids,
            new_impulses,
        );
    }

    /// Gets the convex constraint type ID for a given body handle type and contact count.
    /// Convex one body constraints, contact count 1 through 4: [0, 3]
    /// Convex two body constraints, contact count 1 through 4: [4, 7]
    #[inline(always)]
    fn get_convex_constraint_type_id(contact_count: i32, two_body: bool) -> i32 {
        debug_assert!(contact_count > 0);
        let mut id = contact_count - 1;
        if two_body {
            id |= 0x4;
        }
        id
    }

    /// Computes the constraint type ID for a given contact manifold.
    #[inline(always)]
    fn compute_manifold_constraint_type_id(manifold: &dyn IContactManifold, two_body: bool) -> i32 {
        if manifold.convex() {
            Self::get_convex_constraint_type_id(manifold.count(), two_body)
        } else {
            debug_assert!(manifold.count() > 0);
            if manifold.count() == 1 {
                // No 'nonconvex' one contact constraints — use convex.
                Self::get_convex_constraint_type_id(1, two_body)
            } else {
                // Nonconvex constraints:
                // One body constraints, contact count 2 through 8: [8, 14]
                // Two body constraints, contact count 2 through 8: [15, 21]
                let mut id = 8 + (manifold.count() - 2);
                if two_body {
                    id += 7;
                }
                id
            }
        }
    }

    /// Queues a constraint add to the per-worker pending constraint cache.
    #[inline(always)]
    fn add_constraint<TBodyHandles: Copy, TDescription: Copy, TContactImpulses: Copy>(
        &mut self,
        worker_index: i32,
        manifold_constraint_type: i32,
        pair: CollidablePair,
        pair_cache_change: PairCacheChangeIndex,
        impulses: &TContactImpulses,
        body_handles: TBodyHandles,
        description: &TDescription,
    ) {
        unsafe {
            self.overlap_workers[worker_index as usize]
                .pending_constraints
                .add_constraint(
                    manifold_constraint_type,
                    pair,
                    pair_cache_change,
                    body_handles,
                    description,
                    impulses,
                );
        }
    }

    /// Updates the constraint associated with a pair, dispatching through the contact constraint
    /// accessor for the manifold's type. Handles constraint creation, update, and removal.
    pub unsafe fn update_constraint<
        TBodyHandles: Copy,
        TDescription: Copy + IConstraintDescription,
        TContactImpulses: Copy + Default,
    >(
        &mut self,
        worker_index: i32,
        pair: CollidablePair,
        manifold_type_as_constraint_type: i32,
        new_constraint_cache: &mut ConstraintCache,
        new_contact_count: i32,
        description: &TDescription,
        body_handles: TBodyHandles,
    ) {
        let pair_cache = &mut self.base.pair_cache;
        let solver = &mut *self.base.solver;

        if let Some(index) = pair_cache.index_of(&pair) {
            // Previous frame had a constraint for this pair.
            let cache = *pair_cache.get_cache(index);
            let old_constraint_handle = cache.constraint_handle;
            let constraint_reference = solver.get_constraint_reference(old_constraint_handle);
            let type_batch = &*constraint_reference.type_batch_pointer;

            let mut new_impulses = TContactImpulses::default();
            let accessor = self.base.contact_constraint_accessors[type_batch.type_id as usize]
                .as_ref()
                .expect("accessor must exist");

            let old_impulse_count = accessor.contact_count();
            let mut old_impulses = [0.0f32; 8];
            accessor.gather_old_impulses(&constraint_reference, old_impulses.as_mut_ptr());

            // Redistribute impulses from old contacts to new contacts.
            let old_feature_ids = (&cache.feature_id0 as *const i32) as *mut i32;
            Self::redistribute_impulses(
                old_impulse_count,
                old_feature_ids,
                old_impulses.as_mut_ptr(),
                new_contact_count,
                &mut new_constraint_cache.feature_id0 as *mut i32,
                &mut new_impulses as *mut TContactImpulses as *mut f32,
            );

            if manifold_type_as_constraint_type == type_batch.type_id {
                // Same type — just update in place.
                new_constraint_cache.constraint_handle = old_constraint_handle;
                pair_cache.update(index, new_constraint_cache);
                unsafe {
                    solver.apply_description_without_waking(
                        &constraint_reference,
                        |type_batch, bundle_index, inner_index| {
                            description.apply_description(type_batch, bundle_index, inner_index);
                        },
                    );
                }
                accessor.scatter_new_impulses(
                    &constraint_reference,
                    &new_impulses as *const TContactImpulses as *const f32,
                );
            } else {
                // Different type — remove old, add new.
                let pair_cache_change = pair_cache.update(index, new_constraint_cache);
                self.add_constraint(
                    worker_index,
                    manifold_type_as_constraint_type,
                    pair,
                    pair_cache_change,
                    &new_impulses,
                    body_handles,
                    description,
                );
                self.base
                    .constraint_remover
                    .enqueue_removal(worker_index, old_constraint_handle);
            }
        } else {
            // No preexisting constraint; add a fresh constraint and pair cache entry.
            let pending_pair_cache_change =
                pair_cache.add(worker_index, pair, new_constraint_cache);
            let new_impulses = TContactImpulses::default();
            self.add_constraint(
                worker_index,
                manifold_type_as_constraint_type,
                pair,
                pending_pair_cache_change,
                &new_impulses,
                body_handles,
                description,
            );

            // Check for inactive body set awakenings (two-body case only).
            if mem::size_of::<TBodyHandles>() == mem::size_of::<TwoBodyHandles>() {
                let two_body_handles =
                    &*((&body_handles) as *const TBodyHandles as *const TwoBodyHandles);
                let bodies = &*self.base.bodies;
                let loc_a = &bodies.handle_to_location[two_body_handles.a as usize];
                let loc_b = &bodies.handle_to_location[two_body_handles.b as usize];
                // Only one of the two can be inactive.
                if loc_a.set_index != loc_b.set_index {
                    let worker = &mut self.overlap_workers[worker_index as usize];
                    let set_index = if loc_a.set_index > 0 {
                        loc_a.set_index
                    } else {
                        loc_b.set_index
                    };
                    let pool_ref = &mut *worker.batcher.pool;
                    worker.pending_set_awakenings.add(set_index, pool_ref);
                }
            }
        }
    }

    /// Entry point for processing a completed pair manifold.
    /// Calls user callback and dispatches to constraint update.
    pub fn update_constraints_for_pair<TManifold: IContactManifold>(
        &mut self,
        worker_index: i32,
        pair: CollidablePair,
        manifold: &mut TManifold,
    ) {
        let a_mobility = pair.a.mobility();
        let b_mobility = pair.b.mobility();
        debug_assert!(
            a_mobility != CollidableMobility::Static,
            "The broad phase should not generate static-static pairs, and any static should be in slot B."
        );

        let mut pair_material = PairMaterialProperties::default();
        let allow_constraint = self.callbacks.configure_contact_manifold(
            worker_index,
            pair,
            manifold,
            &mut pair_material,
        ) && manifold.count() > 0;

        if allow_constraint
            && (a_mobility == CollidableMobility::Dynamic
                || b_mobility == CollidableMobility::Dynamic)
        {
            if b_mobility != CollidableMobility::Static {
                // Two bodies.
                let body_handles = TwoBodyHandles {
                    a: pair.a.body_handle().0,
                    b: pair.b.body_handle().0,
                };
                self.update_constraint_for_manifold(
                    worker_index,
                    &pair,
                    manifold,
                    &pair_material,
                    body_handles,
                    true,
                );
            } else {
                // One body + static.
                let body_handle = pair.a.body_handle().0;
                self.update_constraint_for_manifold(
                    worker_index,
                    &pair,
                    manifold,
                    &pair_material,
                    body_handle,
                    false,
                );
            }
        }
    }

    /// Internal: dispatches to the ContactConstraintAccessor to build and submit the constraint.
    fn update_constraint_for_manifold<TManifold: IContactManifold, TBodyHandles: Copy>(
        &mut self,
        worker_index: i32,
        pair: &CollidablePair,
        manifold: &mut TManifold,
        material: &PairMaterialProperties,
        body_handles: TBodyHandles,
        two_body: bool,
    ) {
        let manifold_type_as_constraint_type =
            Self::compute_manifold_constraint_type_id(manifold, two_body);

        // Get the accessor as a raw pointer to avoid borrow conflict with self.
        let accessor_ptr = self.base.contact_constraint_accessors
            [manifold_type_as_constraint_type as usize]
            .as_ref()
            .map(|a| a.as_ref() as *const dyn ContactConstraintAccessor);

        if let Some(accessor) = accessor_ptr {
            unsafe {
                (*accessor).update_constraint_for_manifold_raw(
                    self as *mut _ as *mut u8,
                    manifold_type_as_constraint_type,
                    worker_index,
                    pair,
                    manifold as *mut TManifold as *mut u8,
                    material,
                    &body_handles as *const TBodyHandles as *const u8,
                );
            }
        }
    }

    // ========================================================================
    // HandleOverlap (from NarrowPhase.cs — NarrowPhase<TCallbacks>)
    // ========================================================================

    /// Processes a broad phase overlap between two collidables.
    pub fn handle_overlap(
        &mut self,
        worker_index: i32,
        a: CollidableReference,
        b: CollidableReference,
    ) {
        debug_assert!(
            a.packed != b.packed,
            "An object cannot collide with itself!"
        );
        let (a_mobility, b_mobility, a, b) = NarrowPhase::sort_collidable_references_for_pair(a, b);
        debug_assert!(
            a_mobility != CollidableMobility::Static || b_mobility != CollidableMobility::Static,
            "Broad phase should not generate static-static pairs."
        );

        let two_bodies = b_mobility != CollidableMobility::Static;
        let bodies = unsafe { &*self.base.bodies };

        let body_location_a = &bodies.handle_to_location[a.body_handle().0 as usize];
        let set_a = unsafe { &*bodies.sets.get(body_location_a.set_index) };
        let collidable_a = unsafe { &*set_a.collidables.get(body_location_a.index) };

        let speculative_margin_b;
        if two_bodies {
            let body_location_b = &bodies.handle_to_location[b.body_handle().0 as usize];
            let set_b = unsafe { &*bodies.sets.get(body_location_b.set_index) };
            let collidable_b = unsafe { &*set_b.collidables.get(body_location_b.index) };
            speculative_margin_b = collidable_b.speculative_margin;
        } else {
            speculative_margin_b = 0.0;
        }

        let mut speculative_margin = collidable_a.speculative_margin + speculative_margin_b;

        if !self
            .callbacks
            .allow_contact_generation(worker_index, a, b, &mut speculative_margin)
        {
            return;
        }

        let pair = CollidablePair::new(a, b);

        if two_bodies {
            let body_location_b = &bodies.handle_to_location[b.body_handle().0 as usize];
            let set_b = unsafe { &*bodies.sets.get(body_location_b.set_index) };
            let state_b = unsafe { &*set_b.dynamics_state.get(body_location_b.index) };
            let collidable_b = unsafe { &*set_b.collidables.get(body_location_b.index) };
            let state_a = unsafe { &*set_a.dynamics_state.get(body_location_a.index) };

            self.add_batch_entries(
                worker_index,
                &pair,
                &collidable_a.continuity,
                &collidable_b.continuity,
                collidable_a.shape,
                collidable_b.shape,
                collidable_a.broad_phase_index,
                collidable_b.broad_phase_index,
                speculative_margin,
                &state_a.motion.pose,
                &state_b.motion.pose,
                &state_a.motion.velocity,
                &state_b.motion.velocity,
            );
        } else {
            // Body-static pair.
            let state_a = unsafe { &*set_a.dynamics_state.get(body_location_a.index) };
            let statics = unsafe { &*self.base.statics };
            let static_b = statics.get_direct_reference(b.static_handle());

            let zero_velocity = BodyVelocity::default();
            self.add_batch_entries(
                worker_index,
                &pair,
                &collidable_a.continuity,
                &static_b.continuity,
                collidable_a.shape,
                static_b.shape,
                collidable_a.broad_phase_index,
                static_b.broad_phase_index,
                speculative_margin,
                &state_a.motion.pose,
                &static_b.pose,
                &state_a.motion.velocity,
                &zero_velocity,
            );
        }
    }

    /// Creates batch entries for a pair given all their properties.
    #[inline(always)]
    fn add_batch_entries(
        &mut self,
        worker_index: i32,
        pair: &CollidablePair,
        continuity_a: &ContinuousDetection,
        continuity_b: &ContinuousDetection,
        shape_a: TypedIndex,
        shape_b: TypedIndex,
        broad_phase_index_a: i32,
        broad_phase_index_b: i32,
        speculative_margin: f32,
        pose_a: &RigidPose,
        pose_b: &RigidPose,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
    ) {
        debug_assert!(pair.a.packed != pair.b.packed);
        let allow_expansion = continuity_a.allow_expansion_beyond_speculative_margin()
            | continuity_b.allow_expansion_beyond_speculative_margin();
        let maximum_expansion = if allow_expansion {
            f32::MAX
        } else {
            speculative_margin
        };

        // Create continuation for the pair given CCD state.
        let mut continuation_index = CCDContinuationIndex::default();

        if continuity_a.mode == ContinuousDetectionMode::Continuous
            || continuity_b.mode == ContinuousDetectionMode::Continuous
        {
            let sweep_task_registry = unsafe { &*self.base.sweep_task_registry };
            if let Some(sweep_task) =
                sweep_task_registry.get_task(shape_a.type_id(), shape_b.type_id())
            {
                // Estimate max radius from broadphase bounding boxes (hot in L1 cache).
                let broad_phase = unsafe { &*self.base.broad_phase };
                let bodies = unsafe { &*self.base.bodies };
                let a_in_static_tree = pair.a.mobility() == CollidableMobility::Static
                    || unsafe {
                        (*bodies.handle_to_location.get(pair.a.raw_handle_value())).set_index > 0
                    };
                let b_in_static_tree = pair.b.mobility() == CollidableMobility::Static
                    || unsafe {
                        (*bodies.handle_to_location.get(pair.b.raw_handle_value())).set_index > 0
                    };
                let a_tree = if a_in_static_tree {
                    &broad_phase.static_tree
                } else {
                    &broad_phase.active_tree
                };
                let b_tree = if b_in_static_tree {
                    &broad_phase.static_tree
                } else {
                    &broad_phase.active_tree
                };
                let (a_min, a_max) = unsafe { a_tree.get_bounds_pointers(broad_phase_index_a) };
                let (b_min, b_max) = unsafe { b_tree.get_bounds_pointers(broad_phase_index_b) };
                let maximum_radius_a = unsafe { (*a_max - *a_min).length() * 0.5 };
                let maximum_radius_b = unsafe { (*b_max - *b_min).length() * 0.5 };
                let timestep_duration = self.base.timestep_duration;

                // Check whether sweep is needed: angular+linear displacement > speculative margin.
                if (velocity_a.angular.length() * maximum_radius_a
                    + velocity_b.angular.length() * maximum_radius_b
                    + (velocity_b.linear - velocity_a.linear).length())
                    * timestep_duration
                    > speculative_margin
                {
                    let shapes = unsafe { &*self.base.shapes };
                    let (shape_data_a, _) = unsafe {
                        shapes
                            .get_batch(shape_a.type_id() as usize)
                            .unwrap()
                            .get_shape_data(shape_a.index() as usize)
                    };
                    let (shape_data_b, _) = unsafe {
                        shapes
                            .get_batch(shape_b.type_id() as usize)
                            .unwrap()
                            .get_shape_data(shape_b.index() as usize)
                    };

                    // Get sweep thresholds — use the smaller of the two collidables' values.
                    let (min_timestep_a, convergence_a) =
                        if continuity_a.mode == ContinuousDetectionMode::Continuous {
                            (
                                continuity_a.minimum_sweep_timestep,
                                continuity_a.sweep_convergence_threshold,
                            )
                        } else {
                            (f32::MAX, f32::MAX)
                        };
                    let (min_timestep_b, convergence_b) =
                        if continuity_b.mode == ContinuousDetectionMode::Continuous {
                            (
                                continuity_b.minimum_sweep_timestep,
                                continuity_b.sweep_convergence_threshold,
                            )
                        } else {
                            (f32::MAX, f32::MAX)
                        };

                    // Create filter that delegates to the narrow phase callbacks.
                    let mut filter = CCDSweepFilter {
                        narrow_phase: self as *mut _,
                        pair: *pair,
                        worker_index,
                    };

                    let mut t0 = 0.0f32;
                    let mut t1 = 0.0f32;
                    let mut hit_location = Vec3::ZERO;
                    let mut hit_normal = Vec3::ZERO;

                    let hit = unsafe {
                        sweep_task.sweep_filtered(
                            shape_data_a,
                            shape_a.type_id(),
                            pose_a.orientation,
                            velocity_a,
                            shape_data_b,
                            shape_b.type_id(),
                            pose_b.position - pose_a.position,
                            pose_b.orientation,
                            velocity_b,
                            timestep_duration,
                            min_timestep_a.min(min_timestep_b),
                            convergence_a.min(convergence_b),
                            25, // fixed high iteration threshold
                            &mut filter as *mut CCDSweepFilter<TCallbacks> as *mut u8,
                            self.base.shapes as *mut crate::physics::collidables::shapes::Shapes,
                            self.base.sweep_task_registry as *mut SweepTaskRegistry,
                            self.base.pool as *mut BufferPool,
                            &mut t0,
                            &mut t1,
                            &mut hit_location,
                            &mut hit_normal,
                        )
                    };

                    if hit {
                        // Create the pair at the time of intersection (t1).
                        // The continuation handler will rewind depths to create speculative contacts.
                        let worker = &mut self.overlap_workers[worker_index as usize];
                        continuation_index = worker.batcher.callbacks.add_continuous(
                            pair,
                            velocity_b.linear - velocity_a.linear,
                            velocity_a.angular,
                            velocity_b.angular,
                            t1,
                        );

                        // Integrate poses to t1.
                        let mut integrated_orientation_a = glam::Quat::IDENTITY;
                        let mut integrated_orientation_b = glam::Quat::IDENTITY;
                        PoseIntegration::integrate_orientation(
                            pose_a.orientation,
                            velocity_a.angular,
                            t1,
                            &mut integrated_orientation_a,
                        );
                        PoseIntegration::integrate_orientation(
                            pose_b.orientation,
                            velocity_b.angular,
                            t1,
                            &mut integrated_orientation_b,
                        );
                        let offset_b = pose_b.position - pose_a.position
                            + (velocity_b.linear - velocity_a.linear) * t1;

                        unsafe {
                            worker.batcher.add(
                                shape_a,
                                shape_b,
                                offset_b,
                                integrated_orientation_a,
                                integrated_orientation_b,
                                velocity_a,
                                velocity_b,
                                speculative_margin,
                                maximum_expansion,
                                PairContinuation::direct(continuation_index.packed as i32),
                            );
                        }
                    }
                }
            }
        }

        if !continuation_index.exists() {
            // No CCD continuation was created, so create a discrete one.
            let worker = &mut self.overlap_workers[worker_index as usize];
            continuation_index = worker.batcher.callbacks.add_discrete(pair);
            let offset_b = pose_b.position - pose_a.position;
            unsafe {
                worker.batcher.add(
                    shape_a,
                    shape_b,
                    offset_b,
                    pose_a.orientation,
                    pose_b.orientation,
                    velocity_a,
                    velocity_b,
                    speculative_margin,
                    maximum_expansion,
                    PairContinuation::direct(continuation_index.packed as i32),
                );
            }
        }
    }

    // ========================================================================
    // Preflush (from NarrowPhasePreflush.cs) — multithreaded infrastructure
    // ========================================================================

    /// Builds sorting targets from all workers for a given constraint type.
    fn build_sorting_targets(
        &self,
        list: &mut QuickList<SortConstraintTarget>,
        type_index: i32,
        worker_count: i32,
    ) {
        for i in 0..worker_count {
            let worker_list = unsafe {
                &(*self.overlap_workers[i as usize]
                    .pending_constraints
                    .pending_constraints_by_type
                    .get(type_index))
            };
            if worker_list.count > 0 {
                let entry_size_in_bytes = worker_list.byte_count / worker_list.count;
                let mut index_in_bytes = 0i32;
                for _j in 0..worker_list.count {
                    let constraint = list.allocate_unsafely();
                    constraint.worker_index = i;
                    constraint.byte_index_in_cache = index_in_bytes;
                    // Sort key is the first 8 bytes (CollidablePair) of each pending constraint.
                    constraint.sort_key = unsafe {
                        *(worker_list.buffer.as_ptr().add(index_in_bytes as usize) as *const u64)
                    };
                    index_in_bytes += entry_size_in_bytes;
                }
            }
        }
    }

    /// Executes a single preflush job.
    fn execute_preflush_job(&mut self, _worker_index: i32, job: &PreflushJob) {
        match job.job_type {
            PreflushJobType::CheckFreshness => {
                if let Some(ref mut checker) = self.base.freshness_checker {
                    unsafe {
                        checker.check_freshness_in_region(_worker_index, job.start, job.end);
                    }
                }
            }
            PreflushJobType::SortContactConstraintType => {
                // Use raw pointer to avoid borrow conflicts between sorted_constraints and overlap_workers.
                let self_ptr = self as *mut Self;
                let sorted = unsafe { (*self_ptr).sorted_constraints.get_mut(job.type_index) };
                unsafe {
                    (*self_ptr).build_sorting_targets(
                        sorted,
                        job.type_index,
                        job.worker_index_or_count,
                    );
                }
                let comparer = PendingConstraintComparer;
                if sorted.count > 1 {
                    Quicksort::sort_keys(
                        sorted.span.as_slice_mut(),
                        0,
                        sorted.count - 1,
                        &comparer,
                    );
                }
            }
            PreflushJobType::SpeculativeConstraintBatchSearch => {
                let solver = unsafe { &*self.base.solver };
                unsafe {
                    self.overlap_workers[job.worker_index_or_count as usize]
                        .pending_constraints
                        .speculative_constraint_batch_search(
                            solver,
                            job.type_index,
                            job.start,
                            job.end,
                        );
                }
            }
            PreflushJobType::NondeterministicConstraintAdd => {
                let np_ptr = &self.base as *const NarrowPhase;
                let solver = unsafe { &mut *self.base.solver };
                let pair_cache = &mut self.base.pair_cache;
                let accessors = &self.base.contact_constraint_accessors;
                for i in 0..job.worker_index_or_count {
                    self.overlap_workers[i as usize]
                        .pending_constraints
                        .flush_with_speculative_batches(accessors, np_ptr, solver, pair_cache);
                }
            }
            PreflushJobType::DeterministicConstraintAdd => {
                let np_ptr = &self.base as *const NarrowPhase;
                let solver = unsafe { &mut *self.base.solver };
                let pair_cache = &mut self.base.pair_cache;
                let accessors = &self.base.contact_constraint_accessors;
                let worker_ptrs = &self.base.worker_pending_constraints;
                for type_index in 0..accessors.len() {
                    if let Some(ref accessor) = accessors[type_index] {
                        let sorted = unsafe { self.sorted_constraints.get_mut(type_index as i32) };
                        accessor.deterministically_add(
                            type_index as i32,
                            worker_ptrs,
                            sorted,
                            np_ptr,
                            solver,
                            pair_cache,
                        );
                    }
                }
            }
            PreflushJobType::AwakenerPhaseOne => {
                if !self.base.awakener.is_null() {
                    unsafe {
                        (*self.base.awakener).execute_phase_one_job(job.job_index);
                    }
                }
            }
            PreflushJobType::AwakenerPhaseTwo => {
                if !self.base.awakener.is_null() {
                    unsafe {
                        (*self.base.awakener).execute_phase_two_job(job.job_index);
                    }
                }
            }
        }
    }

    /// Worker loop for multithreaded preflush. Uses atomic increment on preflush_job_index.
    fn preflush_worker_loop(&mut self, _worker_index: i32) {
        loop {
            let job_index = unsafe {
                AtomicI32::from_ptr(self.preflush_job_index.get()).fetch_add(1, Ordering::AcqRel)
                    + 1
            };
            if job_index >= self.preflush_jobs.count {
                break;
            }
            let job = self.preflush_jobs[job_index];
            self.execute_preflush_job(_worker_index, &job);
        }
    }

    /// Static function pointer for preflush worker dispatch.
    /// Retrieves `*mut NarrowPhaseGeneric<TCallbacks>` from `dispatcher.unmanaged_context()`.
    fn preflush_worker_loop_fn(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let narrow_phase =
                &mut *(dispatcher.unmanaged_context() as *mut NarrowPhaseGeneric<TCallbacks>);
            narrow_phase.preflush_worker_loop(worker_index);
        }
    }

    // ========================================================================
    // Preflush (from NarrowPhasePreflush.cs) — single-threaded path
    // ========================================================================

    /// Preflush for single-threaded execution.
    pub(crate) fn preflush_single_threaded(
        &mut self,
        phase_one_job_count: i32,
        phase_two_job_count: i32,
    ) {
        // Single threaded: awakenings, constraint flush, freshness check.
        let original_mapping_count = self.base.pair_cache.mapping.count;

        // Execute awakener phase one + phase two jobs.
        if !self.base.awakener.is_null() {
            unsafe {
                for i in 0..phase_one_job_count {
                    (*self.base.awakener).execute_phase_one_job(i);
                }
                for i in 0..phase_two_job_count {
                    (*self.base.awakener).execute_phase_two_job(i);
                }
            }
        }

        // Flush pending constraints sequentially.
        let solver = unsafe { &mut *self.base.solver };
        // Use raw pointer to avoid borrow conflict between self.base and self.overlap_workers.
        let base_ptr = &self.base as *const NarrowPhase;
        let pair_cache = &mut self.base.pair_cache;
        self.overlap_workers[0]
            .pending_constraints
            .flush_sequentially(unsafe { &*base_ptr }, solver, pair_cache);

        // Check freshness.
        if let Some(ref mut checker) = self.base.freshness_checker {
            unsafe {
                checker.check_freshness_in_region(0, 0, original_mapping_count);
            }
        }
    }

    /// Preflush entry point (wraps single or multithreaded).
    pub fn preflush(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        let thread_count = thread_dispatcher.map_or(1, |d| d.thread_count()) as usize;

        // Count new constraints and prepare capacity.
        let mut new_constraint_count = 0i32;
        let mut _sets_to_awaken_capacity = 0i32;
        for i in 0..thread_count {
            new_constraint_count += self.overlap_workers[i]
                .pending_constraints
                .count_constraints();
            _sets_to_awaken_capacity += self.overlap_workers[i].pending_set_awakenings.count;
        }

        // Ensure solver + pair cache capacities for incoming constraints.
        unsafe {
            let solver = &mut *self.base.solver;
            let target_capacity =
                solver.handle_pool.highest_possibly_claimed_id() + 1 + new_constraint_count;
            self.base
                .pair_cache
                .ensure_constraint_to_pair_mapping_capacity(solver, target_capacity);
            solver.ensure_solver_capacities(1, target_capacity);
        }

        // Accumulate unique awakening indices and prepare awakener jobs.
        let mut sets_to_awaken;
        let mut awakener_phase_one_job_count = 0i32;
        let mut awakener_phase_two_job_count = 0i32;
        unsafe {
            let pool = &mut *self.base.pool;
            sets_to_awaken = QuickList::with_capacity(_sets_to_awaken_capacity, pool);
            if !self.base.awakener.is_null() {
                let bodies = &*self.base.bodies;
                let mut unique_set = IndexSet::new(pool, bodies.sets.len() as i32);
                for i in 0..thread_count {
                    (*self.base.awakener).accumulate_unique_indices(
                        &self.overlap_workers[i].pending_set_awakenings,
                        &mut unique_set,
                        &mut sets_to_awaken,
                    );
                }
                unique_set.dispose(pool);
                for i in 0..thread_count {
                    let worker_pool = if thread_count > 1 {
                        // Multi-threaded: use per-worker pool (but currently not available, use main)
                        pool as *mut BufferPool
                    } else {
                        pool as *mut BufferPool
                    };
                    self.overlap_workers[i]
                        .pending_set_awakenings
                        .dispose(&mut *worker_pool);
                }
                (awakener_phase_one_job_count, awakener_phase_two_job_count) = (*self
                    .base
                    .awakener)
                    .prepare_jobs(&mut sets_to_awaken, false, thread_count as i32);
            }
        }

        let _awakener_phase_one_job_count = awakener_phase_one_job_count;
        let _awakener_phase_two_job_count = awakener_phase_two_job_count;

        if thread_count > 1 && thread_dispatcher.is_some() {
            let pool = unsafe { &mut *self.base.pool };
            // Fixed initial capacity of 128 + max awakener jobs
            let initial_capacity =
                128 + std::cmp::max(_awakener_phase_one_job_count, _awakener_phase_two_job_count);
            self.preflush_jobs = QuickList::with_capacity(initial_capacity, pool);

            // FIRST PHASE: sort, awakener phase one, speculative batch search
            for i in 0..thread_count {
                self.overlap_workers[i]
                    .pending_constraints
                    .allocate_for_speculative_search();
            }
            for i in 0.._awakener_phase_one_job_count {
                self.preflush_jobs.add(
                    PreflushJob {
                        job_type: PreflushJobType::AwakenerPhaseOne,
                        job_index: i,
                        ..PreflushJob::default()
                    },
                    pool,
                );
            }

            if deterministic {
                self.sorted_constraints =
                    pool.take_at_least(PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
                self.sorted_constraints
                    .clear(0, PairCache::COLLISION_CONSTRAINT_TYPE_COUNT);
                for type_index in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
                    let mut count_in_type = 0;
                    for worker_index in 0..thread_count {
                        count_in_type += unsafe {
                            self.overlap_workers[worker_index]
                                .pending_constraints
                                .pending_constraints_by_type
                                .get(type_index)
                                .count
                        };
                    }
                    if count_in_type > 0 {
                        unsafe {
                            *self.sorted_constraints.get_mut(type_index) =
                                QuickList::with_capacity(count_in_type, pool);
                        }
                        self.preflush_jobs.add(
                            PreflushJob {
                                job_type: PreflushJobType::SortContactConstraintType,
                                type_index: type_index as i32,
                                worker_index_or_count: thread_count as i32,
                                ..PreflushJob::default()
                            },
                            pool,
                        );
                    }
                }
            }

            const MAXIMUM_CONSTRAINTS_PER_JOB: i32 = 16;
            for type_index in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
                for worker_index in 0..thread_count {
                    let count = unsafe {
                        self.overlap_workers[worker_index]
                            .pending_constraints
                            .pending_constraints_by_type
                            .get(type_index)
                            .count
                    };
                    if count > 0 {
                        let job_count = 1 + count / MAXIMUM_CONSTRAINTS_PER_JOB;
                        let job_size = count / job_count;
                        let remainder = count - job_count * job_size;
                        let mut previous_end = 0;
                        for i in 0..job_count {
                            let job_start = previous_end;
                            let mut constraints_in_job = job_size;
                            if i < remainder {
                                constraints_in_job += 1;
                            }
                            previous_end += constraints_in_job;
                            self.preflush_jobs.add(
                                PreflushJob {
                                    job_type: PreflushJobType::SpeculativeConstraintBatchSearch,
                                    start: job_start,
                                    end: previous_end,
                                    type_index: type_index as i32,
                                    worker_index_or_count: worker_index as i32,
                                    job_index: 0,
                                },
                                pool,
                            );
                        }
                        debug_assert!(previous_end == count);
                    }
                }
            }

            let original_pair_cache_mapping_count = self.base.pair_cache.mapping.count;
            // Dispatch phase 1
            unsafe {
                *self.preflush_job_index.get() = -1;
            }
            let dispatcher = thread_dispatcher.unwrap();
            let self_ptr = self as *mut NarrowPhaseGeneric<TCallbacks> as *mut ();
            unsafe {
                dispatcher.dispatch_workers(
                    Self::preflush_worker_loop_fn,
                    self.preflush_jobs.count,
                    self_ptr,
                    None,
                );
            }

            // SECOND PHASE: awakener phase two
            self.preflush_jobs.clear();
            for i in 0.._awakener_phase_two_job_count {
                let pool = unsafe { &mut *self.base.pool };
                self.preflush_jobs.add(
                    PreflushJob {
                        job_type: PreflushJobType::AwakenerPhaseTwo,
                        job_index: i,
                        ..PreflushJob::default()
                    },
                    pool,
                );
            }
            unsafe {
                *self.preflush_job_index.get() = -1;
            }
            unsafe {
                dispatcher.dispatch_workers(
                    Self::preflush_worker_loop_fn,
                    self.preflush_jobs.count,
                    self_ptr,
                    None,
                );
            }

            // THIRD PHASE: constraint adds + freshness checker
            self.preflush_jobs.clear();
            let pool = unsafe { &mut *self.base.pool };
            if deterministic {
                self.preflush_jobs.add(
                    PreflushJob {
                        job_type: PreflushJobType::DeterministicConstraintAdd,
                        ..PreflushJob::default()
                    },
                    pool,
                );
            } else {
                self.preflush_jobs.add(
                    PreflushJob {
                        job_type: PreflushJobType::NondeterministicConstraintAdd,
                        worker_index_or_count: thread_count as i32,
                        ..PreflushJob::default()
                    },
                    pool,
                );
            }
            // Create freshness check jobs (matching FreshnessChecker.CreateJobs logic).
            if self.base.freshness_checker.is_some() && original_pair_cache_mapping_count > 0 {
                let pool = unsafe { &mut *self.base.pool };
                if thread_count > 1 {
                    const JOBS_PER_THREAD: i32 = 2;
                    let mapping_count = original_pair_cache_mapping_count;
                    let freshness_job_count =
                        i32::min(thread_count as i32 * JOBS_PER_THREAD, mapping_count);
                    let pairs_per_job = mapping_count / freshness_job_count;
                    let remainder = mapping_count - pairs_per_job * freshness_job_count;
                    let mut previous_end = 0i32;
                    let mut job_index = 0i32;
                    while previous_end < mapping_count {
                        let pairs_in_job = if job_index < remainder {
                            pairs_per_job + 1
                        } else {
                            pairs_per_job
                        };
                        let mut next_end = ((previous_end + pairs_in_job + 7) >> 3) << 3;
                        if next_end > mapping_count {
                            next_end = mapping_count;
                        }
                        self.preflush_jobs.add(
                            PreflushJob {
                                job_type: PreflushJobType::CheckFreshness,
                                start: previous_end,
                                end: next_end,
                                ..PreflushJob::default()
                            },
                            pool,
                        );
                        previous_end = next_end;
                        job_index += 1;
                    }
                } else {
                    self.preflush_jobs.add(
                        PreflushJob {
                            job_type: PreflushJobType::CheckFreshness,
                            start: 0,
                            end: original_pair_cache_mapping_count,
                            ..PreflushJob::default()
                        },
                        pool,
                    );
                }
            }
            unsafe {
                *self.preflush_job_index.get() = -1;
            }
            unsafe {
                dispatcher.dispatch_workers(
                    Self::preflush_worker_loop_fn,
                    self.preflush_jobs.count,
                    self_ptr,
                    None,
                );
            }

            // Cleanup
            for i in 0..thread_count {
                self.overlap_workers[i]
                    .pending_constraints
                    .dispose_speculative_search();
            }
            if deterministic {
                let pool = unsafe { &mut *self.base.pool };
                for i in 0..PairCache::COLLISION_CONSTRAINT_TYPE_COUNT {
                    let type_list = unsafe { self.sorted_constraints.get_mut(i) };
                    if type_list.span.allocated() {
                        type_list.dispose(pool);
                    }
                }
                pool.return_buffer(&mut self.sorted_constraints);
            }
            let pool = unsafe { &mut *self.base.pool };
            self.preflush_jobs.dispose(pool);
        } else {
            self.preflush_single_threaded(
                awakener_phase_one_job_count,
                awakener_phase_two_job_count,
            );
        }

        // Dispose pending constraint caches.
        for i in 0..thread_count {
            self.overlap_workers[i].pending_constraints.dispose();
        }

        // Dispose awakener resources.
        if !self.base.awakener.is_null() && sets_to_awaken.count > 0 {
            unsafe {
                let pool = &mut *self.base.pool;
                (*self.base.awakener).dispose_for_completed_awakenings(&mut sets_to_awaken);
                sets_to_awaken.dispose(pool);
            }
        } else if sets_to_awaken.span.allocated() {
            let pool = unsafe { &mut *self.base.pool };
            sets_to_awaken.dispose(pool);
        }
    }

    /// Full flush including preflush + constraint removal.
    pub fn flush_with_preflush(
        &mut self,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        self.preflush(thread_dispatcher, deterministic);
        self.base.flush(thread_dispatcher, deterministic);
        self.postflush(thread_dispatcher);
    }
}

// =============================================================================
// Standalone redistribute_impulses function (extracted from NarrowPhaseGeneric).
// =============================================================================

/// Redistributes impulses from old contacts to new contacts by matching feature IDs.
/// Unmatched impulse is distributed evenly among unmatched new contacts.
unsafe fn redistribute_impulses(
    old_contact_count: i32,
    old_feature_ids: *mut i32,
    old_impulses: *mut f32,
    new_contact_count: i32,
    new_feature_ids: *mut i32,
    new_impulses: *mut f32,
) {
    let mut unmatched_count = 0;
    for i in 0..new_contact_count {
        let new_impulse = &mut *new_impulses.add(i as usize);
        // Accumulated impulses cannot be negative; we use a negative value as an 'unmatched' flag.
        *new_impulse = -1.0;
        for j in 0..old_contact_count {
            if *old_feature_ids.add(j as usize) == *new_feature_ids.add(i as usize) {
                *new_impulse = *old_impulses.add(j as usize);
                // Eliminate so it won't be distributed to unmatched contacts.
                *old_impulses.add(j as usize) = 0.0;
                break;
            }
        }
        if *new_impulse < 0.0 {
            unmatched_count += 1;
        }
    }
    // Distribute missing impulse evenly over unmatched contacts.
    if unmatched_count > 0 {
        let mut unmatched_impulse: f32 = 0.0;
        for i in 0..old_contact_count {
            unmatched_impulse += *old_impulses.add(i as usize);
        }
        let impulse_per_unmatched = unmatched_impulse / unmatched_count as f32;
        for i in 0..new_contact_count {
            let new_impulse = &mut *new_impulses.add(i as usize);
            if *new_impulse < 0.0 {
                *new_impulse = impulse_per_unmatched;
            }
        }
    }
}

// =============================================================================
// NarrowPhaseUpdateConstraint — type-erased update_constraint for accessors.
//
// The contact constraint accessors need to call update_constraint, but they
// receive a `*mut u8` pointing to NarrowPhaseGeneric<T> (with unknown T).
// Since NarrowPhaseGeneric<T> is #[repr(C)] with `base: NarrowPhase` as the
// first field, the pointer is also a valid *mut NarrowPhase.
//
// This struct provides a static method that performs the same logic as
// NarrowPhaseGeneric::update_constraint using only NarrowPhase fields.
// =============================================================================

/// Provides type-erased access to the narrow phase's update_constraint logic.
/// Used by ContactConstraintAccessor implementations.
pub struct NarrowPhaseUpdateConstraint;

impl NarrowPhaseUpdateConstraint {
    /// Core update_constraint logic operating on a raw NarrowPhase pointer.
    ///
    /// # Safety
    /// - `narrow_phase_ptr` must point to a valid `NarrowPhaseGeneric<T>` (with NarrowPhase at offset 0).
    /// - All pointer fields in NarrowPhase must be valid.
    pub unsafe fn update_constraint<
        TBodyHandles: Copy,
        TDescription: Copy + IConstraintDescription,
        TContactImpulses: Copy + Default,
    >(
        narrow_phase_ptr: *mut u8,
        worker_index: i32,
        pair: CollidablePair,
        manifold_type_as_constraint_type: i32,
        new_constraint_cache: &mut ConstraintCache,
        new_contact_count: i32,
        description: &TDescription,
        body_handles: TBodyHandles,
    ) {
        let np = &mut *(narrow_phase_ptr as *mut NarrowPhase);
        let pair_cache = &mut np.pair_cache;
        let solver = &mut *np.solver;

        if let Some(index) = pair_cache.index_of(&pair) {
            // Previous frame had a constraint for this pair.
            let cache = *pair_cache.get_cache(index);
            let old_constraint_handle = cache.constraint_handle;
            let constraint_reference = solver.get_constraint_reference(old_constraint_handle);
            let type_batch = &*constraint_reference.type_batch_pointer;

            let mut new_impulses = TContactImpulses::default();
            let accessor = np.contact_constraint_accessors[type_batch.type_id as usize]
                .as_ref()
                .expect("accessor must exist");

            let old_impulse_count = accessor.contact_count();
            let mut old_impulses = [0.0f32; 8];
            accessor.gather_old_impulses(&constraint_reference, old_impulses.as_mut_ptr());

            // Redistribute impulses from old contacts to new contacts.
            let old_feature_ids = (&cache.feature_id0 as *const i32) as *mut i32;
            redistribute_impulses(
                old_impulse_count,
                old_feature_ids,
                old_impulses.as_mut_ptr(),
                new_contact_count,
                &mut new_constraint_cache.feature_id0 as *mut i32,
                &mut new_impulses as *mut TContactImpulses as *mut f32,
            );

            if manifold_type_as_constraint_type == type_batch.type_id {
                // Same type — just update in place.
                new_constraint_cache.constraint_handle = old_constraint_handle;
                pair_cache.update(index, new_constraint_cache);
                solver.apply_description_without_waking(
                    &constraint_reference,
                    |type_batch, bundle_index, inner_index| {
                        description.apply_description(type_batch, bundle_index, inner_index);
                    },
                );
                accessor.scatter_new_impulses(
                    &constraint_reference,
                    &new_impulses as *const TContactImpulses as *const f32,
                );
            } else {
                // Different type — remove old, add new.
                let pair_cache_change = pair_cache.update(index, new_constraint_cache);
                add_constraint_core::<TBodyHandles, TDescription, TContactImpulses>(
                    np,
                    worker_index,
                    manifold_type_as_constraint_type,
                    pair,
                    pair_cache_change,
                    &new_impulses,
                    body_handles,
                    description,
                );
                np.constraint_remover
                    .enqueue_removal(worker_index, old_constraint_handle);
            }
        } else {
            // No preexisting constraint; add a fresh constraint and pair cache entry.
            let pending_pair_cache_change =
                pair_cache.add(worker_index, pair, new_constraint_cache);
            let new_impulses = TContactImpulses::default();
            add_constraint_core::<TBodyHandles, TDescription, TContactImpulses>(
                np,
                worker_index,
                manifold_type_as_constraint_type,
                pair,
                pending_pair_cache_change,
                &new_impulses,
                body_handles,
                description,
            );

            // Check for inactive body set awakenings (two-body case only).
            if mem::size_of::<TBodyHandles>() == mem::size_of::<TwoBodyHandles>() {
                let two_body_handles =
                    &*(&body_handles as *const TBodyHandles as *const TwoBodyHandles);
                let bodies = &*np.bodies;
                let loc_a = &bodies.handle_to_location[two_body_handles.a as usize];
                let loc_b = &bodies.handle_to_location[two_body_handles.b as usize];
                // Only one of the two can be inactive.
                if loc_a.set_index != loc_b.set_index {
                    if (worker_index as usize) < np.worker_awakening_ptrs.len() {
                        let (awakenings_ptr, pool_ptr) =
                            np.worker_awakening_ptrs[worker_index as usize];
                        let set_index = if loc_a.set_index > 0 {
                            loc_a.set_index
                        } else {
                            loc_b.set_index
                        };
                        (*awakenings_ptr).add(set_index, &mut *pool_ptr);
                    }
                }
            }
        }
    }
}

/// Standalone helper to add a constraint (used by the type-erased update path).
///
/// # Safety
/// All pointer fields in np must be valid.
unsafe fn add_constraint_core<TBodyHandles: Copy, TDescription: Copy, TContactImpulses: Copy>(
    np: &mut NarrowPhase,
    worker_index: i32,
    manifold_constraint_type: i32,
    pair: CollidablePair,
    pair_cache_change: PairCacheChangeIndex,
    impulses: &TContactImpulses,
    body_handles: TBodyHandles,
    description: &TDescription,
) {
    let pending = &mut *np.worker_pending_constraints[worker_index as usize];
    pending.add_constraint(
        manifold_constraint_type,
        pair,
        pair_cache_change,
        body_handles,
        description,
        impulses,
    );
}
