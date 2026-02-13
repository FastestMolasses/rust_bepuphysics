// Translated from BepuPhysics/Solver.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::BodyInertia;
use crate::physics::body_set::BodyConstraintReference;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collision_detection::pair_cache::{CollidablePair, PairCache};
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraint_reference::ConstraintReference;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::{ITypeProcessor, TypeProcessor};
use crate::physics::constraints::constraint_description::{
    IConstraintDescription, IOneBodyConstraintDescription, ITwoBodyConstraintDescription,
    IThreeBodyConstraintDescription, IFourBodyConstraintDescription,
};
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::sequential_fallback_batch::SequentialFallbackBatch;
use crate::physics::solve_description::{SolveDescription, SubstepVelocityIterationScheduler};
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::collections::quick_set::QuickSet;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;
use crate::physics::pose_integrator::IPoseIntegrator;
use std::simd::prelude::*;
use crate::utilities::vector::Vector;

/// Holds and solves constraints between bodies in a simulation.
pub struct Solver {
    /// Buffer containing all constraint sets. The first slot is dedicated to the active set;
    /// subsequent slots may be occupied by the constraints associated with inactive islands.
    pub sets: Buffer<ConstraintSet>,

    // Note that the referenced handles for the active set are stored outside the constraint set;
    // inactive islands do not store the referenced handles since no new constraints are ever added.
    pub(crate) batch_referenced_handles: QuickList<IndexSet>,

    /// Set of processors applied to batches of constraints of particular types, indexed by the constraint type id.
    pub type_processors: Vec<Option<TypeProcessor>>,

    pub(crate) bodies: *mut Bodies,
    pub(crate) pair_cache: *mut PairCache,
    pub(crate) awakener: *mut IslandAwakener,
    pub(crate) pose_integrator: Option<*mut dyn IPoseIntegrator>,

    /// Pool to retrieve constraint handles from when creating new constraints.
    pub handle_pool: IdPool,
    pub(crate) pool: *mut BufferPool,

    /// Mapping from constraint handle (via its internal integer value) to the location of a constraint in memory.
    pub handle_to_constraint: Buffer<ConstraintLocation>,

    /// Gets the maximum number of solver batches to allow before resorting to a fallback solver.
    fallback_batch_threshold: i32,

    /// Lock used to add to the constrained kinematic handles from multiple threads, if necessary.
    // Note: C# uses System.Threading.SpinLock; here we use a simple bool-based spinlock
    pub(crate) constrained_kinematic_lock: std::sync::atomic::AtomicBool,

    /// Set of body handles associated with constrained kinematic bodies. These will be integrated during substepping.
    pub constrained_kinematic_handles: QuickSet<i32, PrimitiveComparer<i32>>,

    substep_count: i32,
    velocity_iteration_count: i32,

    /// Callback executed to determine how many velocity iterations should be used for a given substep.
    pub velocity_iteration_scheduler: Option<SubstepVelocityIterationScheduler>,

    minimum_capacity_per_type_batch: i32,
    minimum_initial_capacity_per_type_batch: Vec<i32>,

    // Substep event callbacks
    pub substep_started: Option<Box<dyn Fn(i32)>>,
    pub substep_ended: Option<Box<dyn Fn(i32)>>,

    // Integration responsibility tracking
    // Per-batch, per-type-batch, per-body-in-constraint integration flag IndexSets.
    // integrationFlags[batchIndex][typeBatchIndex][bodyIndexInConstraint] = IndexSet
    // Batch 0 is always default (all batch-0 constraints integrate).
    pub(crate) integration_flags: Buffer<Buffer<Buffer<IndexSet>>>,
    /// Caches a single bool for whether type batches within batches have constraints with any integration responsibilities.
    /// Type batches with no integration responsibilities can use a codepath with no integration checks at all.
    pub(crate) coarse_batch_integration_responsibilities: Buffer<Buffer<bool>>,
    /// Merged set of all body handles that appear in any constraint batch (including constrained kinematics).
    pub(crate) merged_constrained_body_handles: IndexSet,
    /// Per-batch IndexSets tracking which body handles are first observed in that batch.
    pub(crate) bodies_first_observed_in_batches: Buffer<IndexSet>,

    // Multithreaded integration responsibility computation state
    pub(crate) next_constraint_integration_responsibility_job_index: std::sync::atomic::AtomicI32,
    pub(crate) integration_responsibility_prepass_jobs: Vec<(i32, i32, i32, i32)>, // (batch, typeBatch, start, end)
    pub(crate) job_aligned_integration_responsibilities: Buffer<bool>,

    // Multithreaded solve state
    pub(crate) substep_context: SubstepMultithreadingContext,
}

// Re-export IslandAwakener from its real module for use here.
pub use crate::physics::island_awakener::IslandAwakener;

/// Simple primitive comparer for use with QuickSet<i32>.
#[derive(Clone, Copy, Default)]
pub struct PrimitiveComparer<T: Copy> {
    _marker: std::marker::PhantomData<T>,
}

impl crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer<i32> for PrimitiveComparer<i32> {
    #[inline(always)]
    fn hash(&self, item: &i32) -> i32 {
        crate::utilities::collections::quick_dictionary::HashHelper::rehash(*item)
    }

    #[inline(always)]
    fn equals(&self, a: &i32, b: &i32) -> bool {
        *a == *b
    }
}

impl Solver {
    /// Gets a reference to the active set of constraints, stored in the first set slot.
    #[inline(always)]
    pub fn active_set(&self) -> &ConstraintSet {
        self.sets.get(0)
    }

    /// Gets a mutable reference to the active set of constraints.
    #[inline(always)]
    pub fn active_set_mut(&mut self) -> &mut ConstraintSet {
        self.sets.get_mut(0)
    }

    #[inline(always)]
    pub fn fallback_batch_threshold(&self) -> i32 {
        self.fallback_batch_threshold
    }

    pub fn substep_count(&self) -> i32 {
        self.substep_count
    }

    pub fn set_substep_count(&mut self, value: i32) {
        assert!(value >= 1, "Substep count must be positive.");
        self.substep_count = value;
    }

    pub fn velocity_iteration_count(&self) -> i32 {
        self.velocity_iteration_count
    }

    pub fn set_velocity_iteration_count(&mut self, value: i32) {
        assert!(value >= 1, "Iteration count must be positive.");
        self.velocity_iteration_count = value;
    }

    pub fn minimum_capacity_per_type_batch(&self) -> i32 {
        self.minimum_capacity_per_type_batch
    }

    pub fn set_minimum_capacity_per_type_batch(&mut self, value: i32) {
        self.minimum_capacity_per_type_batch = value.max(1);
    }

    /// Sets the minimum capacity initially allocated to a new type batch of the given type.
    pub fn set_minimum_capacity_for_type(&mut self, type_id: i32, minimum_initial_capacity_for_type: i32) {
        debug_assert!(type_id >= 0, "Type id must be nonnegative.");
        debug_assert!(minimum_initial_capacity_for_type >= 0, "Capacity must be nonnegative.");
        if type_id as usize >= self.minimum_initial_capacity_per_type_batch.len() {
            self.minimum_initial_capacity_per_type_batch.resize(type_id as usize + 1, 0);
        }
        self.minimum_initial_capacity_per_type_batch[type_id as usize] = minimum_initial_capacity_for_type;
    }

    /// Gets the minimum initial capacity for a given type.
    #[inline(always)]
    pub fn get_minimum_capacity_for_type(&self, type_id: i32) -> i32 {
        debug_assert!(type_id >= 0, "Type ids must be nonnegative.");
        if type_id as usize >= self.minimum_initial_capacity_per_type_batch.len() {
            return self.minimum_capacity_per_type_batch;
        }
        self.minimum_initial_capacity_per_type_batch[type_id as usize].max(self.minimum_capacity_per_type_batch)
    }

    /// Resets all per-type initial capacities to zero. Leaves the minimum capacity across all constraints unchanged.
    pub fn reset_per_type_initial_capacities(&mut self) {
        for v in self.minimum_initial_capacity_per_type_batch.iter_mut() {
            *v = 0;
        }
    }

    fn pool(&self) -> &mut BufferPool {
        unsafe { &mut *self.pool }
    }

    fn bodies(&self) -> &Bodies {
        unsafe { &*self.bodies }
    }

    fn bodies_mut(&self) -> &mut Bodies {
        unsafe { &mut *self.bodies }
    }

    /// Counts the number of constraints in a particular type batch.
    pub fn count_constraints_in_type_batch(&self, set_index: i32, batch_index: i32, type_batch_index: i32) -> i32 {
        let set = self.sets.get(set_index);
        debug_assert!(set.allocated());
        let batch = set.batches.get(batch_index);
        let type_batch = batch.type_batches.get(type_batch_index);
        if set_index == 0 && batch_index > self.fallback_batch_threshold {
            // Sequential fallback batches store constraints noncontiguously.
            let constraint_count = type_batch.constraint_count;
            let mut count = 0;
            for i in 0..constraint_count {
                if type_batch.index_to_handle.get(i).0 >= 0 {
                    count += 1;
                }
            }
            count
        } else {
            type_batch.constraint_count
        }
    }

    /// Gets the total number of constraints across all sets, batches, and types.
    pub fn count_constraints(&self) -> i32 {
        let mut count = 0;
        for set_index in 0..self.sets.len() {
            let set = self.sets.get(set_index);
            if set.allocated() {
                let set_is_active_and_fallback_exists = set_index == 0 && set.batches.count > self.fallback_batch_threshold;
                let contiguous_batch_count = if set_is_active_and_fallback_exists {
                    self.fallback_batch_threshold
                } else {
                    set.batches.count
                };
                for batch_index in 0..contiguous_batch_count {
                    let batch = set.batches.get(batch_index);
                    for type_batch_index in 0..batch.type_batches.count {
                        count += batch.type_batches.get(type_batch_index).constraint_count;
                    }
                }
                if set_is_active_and_fallback_exists {
                    let batch = set.batches.get(self.fallback_batch_threshold);
                    for type_batch_index in 0..batch.type_batches.count {
                        let type_batch = batch.type_batches.get(type_batch_index);
                        for i in 0..type_batch.constraint_count {
                            if type_batch.index_to_handle.get(i).0 >= 0 {
                                count += 1;
                            }
                        }
                    }
                }
            }
        }
        count
    }

    pub fn new(
        bodies: *mut Bodies,
        pool: *mut BufferPool,
        solve_description: SolveDescription,
        initial_capacity: i32,
        initial_island_capacity: i32,
        minimum_capacity_per_type_batch: i32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let bodies_ref = unsafe { &*bodies };

        let mut solver = Self {
            sets: Buffer::default(),
            batch_referenced_handles: QuickList::default(),
            type_processors: Vec::new(),
            bodies,
            pair_cache: std::ptr::null_mut(),
            awakener: std::ptr::null_mut(),
            pose_integrator: None,
            handle_pool: IdPool::new(128, pool_ref),
            pool,
            handle_to_constraint: Buffer::default(),
            fallback_batch_threshold: solve_description.fallback_batch_threshold,
            constrained_kinematic_lock: std::sync::atomic::AtomicBool::new(false),
            constrained_kinematic_handles: QuickSet::with_capacity(
                bodies_ref.handle_to_location.len(),
                3,
                pool_ref,
                PrimitiveComparer::default(),
            ),
            substep_count: solve_description.substep_count.max(1),
            velocity_iteration_count: solve_description.velocity_iteration_count.max(1),
            velocity_iteration_scheduler: solve_description.velocity_iteration_scheduler,
            minimum_capacity_per_type_batch,
            minimum_initial_capacity_per_type_batch: Vec::new(),
            substep_started: None,
            substep_ended: None,
            integration_flags: Buffer::default(),
            coarse_batch_integration_responsibilities: Buffer::default(),
            merged_constrained_body_handles: IndexSet::empty(),
            bodies_first_observed_in_batches: Buffer::default(),
            next_constraint_integration_responsibility_job_index: std::sync::atomic::AtomicI32::new(0),
            integration_responsibility_prepass_jobs: Vec::new(),
            job_aligned_integration_responsibilities: Buffer::default(),
            substep_context: SubstepMultithreadingContext::default(),
        };

        solver.resize_sets_capacity(initial_island_capacity + 1, 0);
        *solver.sets.get_mut(0) = ConstraintSet::new(pool_ref, solver.fallback_batch_threshold + 1);
        solver.batch_referenced_handles = QuickList::with_capacity(solver.fallback_batch_threshold + 1, pool_ref);
        solver.resize_handle_capacity(initial_capacity);
        solver
    }

    /// Registers a constraint type with the solver, creating a type processor for the type
    /// internally and allowing constraints of that type to be added to the solver.
    ///
    /// This is the generic version matching C#'s `Solver.Register<TDescription>()`.
    /// [`DefaultTypes::register_defaults`] is called during simulation creation and registers all
    /// the built-in types. Calling `register` manually is only necessary for custom types.
    pub fn register<TDescription: crate::physics::constraints::constraint_description::IConstraintDescription>(&mut self) {
        let type_id = TDescription::constraint_type_id();
        debug_assert!(type_id >= 0, "Constraint type ids should never be negative. They're used for array indexing.");
        let type_id_usize = type_id as usize;

        // Grow the type processors array if necessary.
        if type_id_usize >= self.type_processors.len() {
            self.type_processors.resize_with(type_id_usize + 1, || None);
        }

        if self.type_processors[type_id_usize].is_none() {
            let processor = TDescription::create_type_processor();
            self.type_processors[type_id_usize] = Some(processor);
        } else {
            debug_assert!(
                false,
                "A type processor has already been registered for constraint type id {}.",
                type_id
            );
        }
    }

    /// Registers a constraint type with the solver using a pre-built type processor.
    /// This is the non-generic version for cases where you already have a TypeProcessor instance.
    pub fn register_type_processor(&mut self, type_id: i32, processor: TypeProcessor) {
        debug_assert!(type_id >= 0, "Constraint type ids should never be negative.");
        let type_id_usize = type_id as usize;
        if type_id_usize >= self.type_processors.len() {
            self.type_processors.resize_with(type_id_usize + 1, || None);
        }
        debug_assert!(
            self.type_processors[type_id_usize].is_none(),
            "A type processor has already been registered for constraint type id {}.",
            type_id
        );
        self.type_processors[type_id_usize] = Some(processor);
    }

    /// Gets whether the given constraint handle refers to a constraint in the solver.
    #[inline(always)]
    pub fn constraint_exists(&self, constraint_handle: ConstraintHandle) -> bool {
        constraint_handle.0 >= 0
            && constraint_handle.0 <= self.handle_pool.highest_possibly_claimed_id()
            && self.handle_to_constraint.get(constraint_handle.0).set_index >= 0
    }

    /// Gets a direct reference to the constraint associated with a handle.
    #[inline(always)]
    pub unsafe fn get_constraint_reference(&mut self, handle: ConstraintHandle) -> ConstraintReference {
        debug_assert!(self.constraint_exists(handle));
        let location = *self.handle_to_constraint.get(handle.0);
        let type_batch_ptr = self.sets.get_mut(location.set_index)
            .batches.get_mut(location.batch_index)
            .get_type_batch_pointer(location.type_id);
        ConstraintReference::new(type_batch_ptr, location.index_in_type_batch)
    }

    /// Attempts to locate a spot for a new constraint. Does not perform allocation.
    /// Returns the index of the batch that the constraint would fit in.
    pub(crate) fn find_candidate_batch_with_handles(&self, dynamic_handles: &[i32]) -> i32 {
        let (sync_count, fallback_exists) = self.get_synchronized_batch_count();
        if dynamic_handles.len() == 2 {
            let a = dynamic_handles[0];
            let b = dynamic_handles[1];
            for batch_index in 0..sync_count {
                let handles = &self.batch_referenced_handles.get(batch_index);
                if !handles.contains(a) && !handles.contains(b) {
                    return batch_index;
                }
            }
        } else if dynamic_handles.len() == 1 {
            let handle = dynamic_handles[0];
            for batch_index in 0..sync_count {
                if !self.batch_referenced_handles.get(batch_index).contains(handle) {
                    return batch_index;
                }
            }
        } else {
            for batch_index in 0..sync_count {
                let handles = &self.batch_referenced_handles.get(batch_index);
                let mut fits = true;
                for &h in dynamic_handles {
                    if handles.contains(h) {
                        fits = false;
                        break;
                    }
                }
                if fits {
                    return batch_index;
                }
            }
        }
        sync_count
    }

    /// Finds a candidate batch for a constraint associated with a collidable pair.
    /// Extracts dynamic body handles from the pair's collidable references and delegates
    /// to find_candidate_batch_with_handles.
    pub(crate) fn find_candidate_batch(&self, pair: &CollidablePair) -> i32 {
        let a_is_dynamic = pair.a.mobility() == CollidableMobility::Dynamic;
        if a_is_dynamic && pair.b.mobility() == CollidableMobility::Dynamic {
            // Both collidables are dynamic.
            let handles = [pair.a.body_handle().0, pair.b.body_handle().0];
            self.find_candidate_batch_with_handles(&handles)
        } else {
            // Only one collidable is dynamic. Statics and kinematics will not block batch containment.
            debug_assert!(
                a_is_dynamic || pair.b.mobility() == CollidableMobility::Dynamic,
                "Constraints can only be created when at least one body in the pair is dynamic."
            );
            let dynamic_handle = if a_is_dynamic {
                pair.a.body_handle().0
            } else {
                pair.b.body_handle().0
            };
            self.find_candidate_batch_with_handles(&[dynamic_handle])
        }
    }

    pub(crate) unsafe fn allocate_in_batch(
        &mut self,
        target_batch_index: i32,
        constraint_handle: ConstraintHandle,
        dynamic_body_handles: &[BodyHandle],
        encoded_body_indices: &[i32],
        type_id: i32,
    ) -> ConstraintReference {
        let self_ptr = self as *mut Self;
        let pool = &mut *(*self_ptr).pool;
        let batch = (*self_ptr).sets.get_mut(0).batches.get_mut(target_batch_index);
        let bodies = &*(*self_ptr).bodies;

        // Include the body in the constrained kinematics set if necessary.
        for &encoded_body_index in encoded_body_indices {
            if Bodies::is_encoded_kinematic_reference(encoded_body_index) {
                let body_index = encoded_body_index & Bodies::BODY_REFERENCE_MASK;
                let handle = bodies.active_set().index_to_handle.get(body_index).0;
                (*self_ptr).constrained_kinematic_handles.add(&handle, pool);
            }
        }

        let type_processors = &(*self_ptr).type_processors;
        let type_processor = type_processors[type_id as usize].as_ref().unwrap();
        debug_assert!(type_processor.bodies_per_constraint == encoded_body_indices.len() as i32);
        let min_capacity = (*self_ptr).get_minimum_capacity_for_type(type_id);
        let type_batch = batch.get_or_create_type_batch(
            type_id,
            type_processor.inner(),
            min_capacity,
            pool,
        );

        let index_in_type_batch = if target_batch_index == (*self_ptr).fallback_batch_threshold {
            type_processor.inner().allocate_in_type_batch_for_fallback(
                &mut *type_batch,
                constraint_handle,
                encoded_body_indices,
                pool as *mut BufferPool,
            )
        } else {
            type_processor.inner().allocate_in_type_batch(
                &mut *type_batch,
                constraint_handle,
                encoded_body_indices,
                pool,
            )
        };

        let reference = ConstraintReference::new(type_batch, index_in_type_batch);

        let handles_set = (*self_ptr).batch_referenced_handles.get_mut(target_batch_index);
        for &dynamic_handle in dynamic_body_handles {
            handles_set.set(dynamic_handle.0, pool);
        }

        if target_batch_index == (*self_ptr).fallback_batch_threshold {
            let bodies_ref = &*(*self_ptr).bodies;
            (*self_ptr).sets.get_mut(0).sequential_fallback.allocate_for_active(
                dynamic_body_handles,
                bodies_ref,
                pool,
                bodies_ref.active_set().count,
            );
        }

        reference
    }

    pub(crate) fn get_blocking_body_handles(
        &self,
        body_handles: &[BodyHandle],
        encoded_body_indices: &mut [i32],
    ) -> Vec<BodyHandle> {
        let bodies = self.bodies();
        let solver_states = &bodies.active_set().dynamics_state;
        let mut blocking = Vec::with_capacity(body_handles.len());

        for (i, &handle) in body_handles.iter().enumerate() {
            let location = bodies.handle_to_location.get(handle.0);
            debug_assert!(location.set_index == 0);
            if Bodies::is_kinematic(unsafe { &(*solver_states.get(location.index)).inertia.local }) {
                encoded_body_indices[i] = location.index | Bodies::KINEMATIC_MASK as i32;
            } else {
                blocking.push(handle);
                encoded_body_indices[i] = location.index;
            }
        }
        blocking
    }

    pub(crate) fn allocate_new_constraint_batch(&mut self) -> i32 {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            let set = (*self_ptr).sets.get_mut(0);
            if set.batches.count == set.batches.span.len() {
                set.batches.resize(set.batches.count + 1, pool);
            }
            let type_proc_len = (*self_ptr).type_processors.len() as i32;
            *set.batches.allocate_unsafely() = ConstraintBatch::new(pool, type_proc_len);

            let bodies = &*(*self_ptr).bodies;
            let active_count = bodies.active_set().count;
            let set = (*self_ptr).sets.get(0);
            if set.batches.count == (*self_ptr).batch_referenced_handles.span.len() {
                (*self_ptr).batch_referenced_handles.resize((*self_ptr).batch_referenced_handles.count + 1, pool);
            }
            *(*self_ptr).batch_referenced_handles.allocate_unsafely() = IndexSet::new(pool, active_count);
            (*self_ptr).sets.get(0).batches.count - 1
        }
    }

    pub(crate) unsafe fn try_allocate_in_batch(
        &mut self,
        type_id: i32,
        target_batch_index: i32,
        dynamic_body_handles: &[BodyHandle],
        encoded_body_indices: &[i32],
    ) -> Option<(ConstraintHandle, ConstraintReference)> {
        let self_ptr = self as *mut Self;
        let set_batch_count = (*self_ptr).sets.get(0).batches.count;
        debug_assert!(
            target_batch_index <= set_batch_count,
            "Target batch cannot be more than one slot beyond the end of the batch list."
        );

        if target_batch_index == set_batch_count {
            (*self_ptr).allocate_new_constraint_batch();
        } else if target_batch_index < (*self_ptr).fallback_batch_threshold {
            // A non-fallback constraint batch already exists here. This may fail.
            let int_handles: Vec<i32> = dynamic_body_handles.iter().map(|h| h.0).collect();
            if !(*self_ptr).batch_referenced_handles.get(target_batch_index).can_fit(&int_handles) {
                return None;
            }
        }

        let constraint_handle = ConstraintHandle((*self_ptr).handle_pool.take());
        let reference = (*self_ptr).allocate_in_batch(
            target_batch_index,
            constraint_handle,
            dynamic_body_handles,
            encoded_body_indices,
            type_id,
        );

        let pool = &mut *(*self_ptr).pool;
        if constraint_handle.0 >= (*self_ptr).handle_to_constraint.len() {
            let old_len = (*self_ptr).handle_to_constraint.len();
            pool.resize_to_at_least(
                &mut (*self_ptr).handle_to_constraint,
                old_len * 2,
                old_len,
            );
            debug_assert!(constraint_handle.0 < (*self_ptr).handle_to_constraint.len());
        }
        let location = (*self_ptr).handle_to_constraint.get_mut(constraint_handle.0);
        location.set_index = 0;
        location.index_in_type_batch = reference.index_in_type_batch;
        location.type_id = type_id;
        location.batch_index = target_batch_index;

        Some((constraint_handle, reference))
    }

    /// Applies a description to a constraint slot without waking up the associated island.
    pub unsafe fn apply_description_without_waking<F>(
        &mut self,
        reference: &ConstraintReference,
        apply_fn: F,
    ) where
        F: FnOnce(&mut TypeBatch, i32, i32),
    {
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(reference.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        apply_fn(&mut *reference.type_batch_pointer, bundle_index as i32, inner_index as i32);
    }

    /// Internal add logic. Finds a batch for the constraint and allocates it.
    unsafe fn add_internal(
        &mut self,
        body_handles: &[BodyHandle],
        type_id: i32,
        apply_fn: impl FnOnce(&mut TypeBatch, i32, i32),
    ) -> ConstraintHandle {
        let mut encoded_body_indices = [0i32; 8]; // stack alloc — matches C# stackalloc
        debug_assert!(body_handles.len() <= 8, "Body handles exceeds stack buffer size");
        let blocking = self.get_blocking_body_handles(body_handles, &mut encoded_body_indices[..body_handles.len()]);

        let batch_count = self.active_set().batches.count;
        for i in 0..=batch_count {
            if let Some((handle, reference)) = self.try_allocate_in_batch(
                type_id,
                i,
                &blocking,
                &encoded_body_indices,
            ) {
                let (mut bundle_index, mut inner_index) = (0usize, 0usize);
                BundleIndexing::get_bundle_indices(reference.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
                apply_fn(&mut *reference.type_batch_pointer, bundle_index as i32, inner_index as i32);
                return handle;
            }
        }
        unreachable!("The above loop checks every batch and also one beyond. It should be guaranteed to succeed.");
    }

    /// Allocates a constraint slot and sets up a constraint.
    /// Body handles must refer to active bodies. The caller is responsible for waking sleeping bodies first.
    pub unsafe fn add(
        &mut self,
        body_handles: &[BodyHandle],
        type_id: i32,
        apply_fn: impl FnOnce(&mut TypeBatch, i32, i32),
    ) -> ConstraintHandle {
        // Adding a constraint assumes that the involved bodies are active, so wake up anything sleeping.
        if !self.awakener.is_null() {
            for &handle in body_handles {
                (*self.awakener).awaken_body(handle);
            }
        }

        let constraint_handle = self.add_internal(body_handles, type_id, apply_fn);

        // Register the constraint with each connected body.
        let bodies = &mut *self.bodies;
        for (i, &body_handle) in body_handles.iter().enumerate() {
            bodies.validate_existing_handle(body_handle);
            let body_index = bodies.handle_to_location.get(body_handle.0).index;
            bodies.add_constraint(body_index, constraint_handle, i as i32);
        }

        constraint_handle
    }

    // --- Removal ---

    pub(crate) fn remove_batch_if_empty(&mut self, batch_index: i32) {
        let self_ptr = self as *mut Self;
        unsafe {
            let set = (*self_ptr).sets.get_mut(0);
            if set.batches.get(batch_index).type_batches.count == 0 {
                if batch_index == set.batches.count - 1 {
                    let pool = &mut *(*self_ptr).pool;
                    while set.batches.count > 0 {
                        let last_batch_index = set.batches.count - 1;
                        if set.batches.get(last_batch_index).type_batches.count == 0 {
                            set.batches.get_mut(last_batch_index).dispose(pool);
                            (*self_ptr).batch_referenced_handles.get_mut(last_batch_index).dispose(pool);
                            (*self_ptr).batch_referenced_handles.count -= 1;
                            set.batches.count -= 1;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
    }

    /// Removes a constraint from a batch, performing any necessary batch cleanup, but does not return the handle.
    pub(crate) unsafe fn remove_from_batch(&mut self, batch_index: i32, type_id: i32, index_in_type_batch: i32) {
        let self_ptr = self as *mut Self;
        let pool = &mut *(*self_ptr).pool;
        let set = (*self_ptr).sets.get_mut(0);
        let batch = set.batches.get_mut(batch_index);

        if batch_index == (*self_ptr).fallback_batch_threshold {
            set.sequential_fallback.remove(self_ptr as *const Solver, pool, batch, (*self_ptr).batch_referenced_handles.get_mut(batch_index), type_id, index_in_type_batch);
        } else {
            // Remove body handles from the batch referenced handles for this constraint.
            (*self_ptr).remove_body_handles_from_batch(batch_index, type_id, index_in_type_batch);
        }

        let type_processors = &(*self_ptr).type_processors;
        let type_processor = type_processors[type_id as usize].as_ref().unwrap();
        let batch = (*self_ptr).sets.get_mut(0).batches.get_mut(batch_index);
        let type_batch_index = *batch.type_index_to_type_batch_index.get(type_id);
        let type_batch = batch.type_batches.get_mut(type_batch_index);
        type_processor.inner().remove(
            type_batch,
            index_in_type_batch,
            &mut (*self_ptr).handle_to_constraint,
            batch_index == (*self_ptr).fallback_batch_threshold,
        );
        batch.remove_type_batch_if_empty(type_batch_index, pool);
        (*self_ptr).remove_batch_if_empty(batch_index);
    }

    /// Removes body handle references from the batch referenced handles for a constraint being removed.
    unsafe fn remove_body_handles_from_batch(&mut self, batch_index: i32, type_id: i32, index_in_type_batch: i32) {
        let self_ptr = self as *mut Self;
        let batch = (*self_ptr).sets.get(0).batches.get(batch_index);
        let type_batch = batch.get_type_batch(type_id);
        let bodies_per_constraint = (&(*self_ptr).type_processors)[type_id as usize].as_ref().unwrap().bodies_per_constraint;
        let bodies = &*(*self_ptr).bodies;
        let bytes_per_bundle = bodies_per_constraint * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32;
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        let start_byte = bundle_index as i32 * bytes_per_bundle + inner_index as i32 * 4;

        let handles_set = (*self_ptr).batch_referenced_handles.get_mut(batch_index);
        for body_index_in_constraint in 0..bodies_per_constraint {
            let encoded = *(type_batch.body_references.as_ptr().add(
                (start_byte + body_index_in_constraint * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32) as usize,
            ) as *const i32);
            if Bodies::is_encoded_dynamic_reference(encoded) {
                let body_index = encoded & Bodies::BODY_REFERENCE_MASK;
                let body_handle = bodies.active_set().index_to_handle.get(body_index).0;
                handles_set.unset(body_handle);
            }
        }
    }

    /// Removes the constraint associated with the given handle.
    pub unsafe fn remove(&mut self, handle: ConstraintHandle) {
        debug_assert!(self.constraint_exists(handle));
        let location = *self.handle_to_constraint.get(handle.0);
        if location.set_index > 0 {
            // In order to remove a constraint, it must be active.
            if !self.awakener.is_null() {
                (*self.awakener).awaken_constraint(handle);
            }
        }
        debug_assert!(self.handle_to_constraint.get(handle.0).set_index == 0);

        // Remove constraint from all connected body constraints lists.
        self.remove_constraint_references_from_bodies(handle);

        if !self.pair_cache.is_null() {
            (*self.pair_cache).remove_reference_if_contact_constraint(handle, location.type_id);
        }

        self.remove_from_batch(location.batch_index, location.type_id, location.index_in_type_batch);

        // A negative set index marks a slot in the handle->constraint mapping as unused.
        self.handle_to_constraint.get_mut(handle.0).set_index = -1;
        let pool = unsafe { &mut *self.pool };
        self.handle_pool.return_id(handle.0, pool);
    }

    /// Enumerates the bodies attached to an active constraint and removes the constraint's handle from all connected body constraint reference lists.
    unsafe fn remove_constraint_references_from_bodies(&mut self, constraint_handle: ConstraintHandle) {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let set = self.sets.get(location.set_index);
        let batch = set.batches.get(location.batch_index);
        let type_batch = batch.get_type_batch(location.type_id);
        let bodies_per_constraint = self.type_processors[location.type_id as usize].as_ref().unwrap().bodies_per_constraint;
        let bytes_per_bundle = bodies_per_constraint * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32;
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(location.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        let start_byte = bundle_index as i32 * bytes_per_bundle + inner_index as i32 * 4;

        let bodies = &mut *self.bodies;
        for body_index_in_constraint in 0..bodies_per_constraint {
            let encoded = *(type_batch.body_references.as_ptr().add(
                (start_byte + body_index_in_constraint * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32) as usize,
            ) as *const i32);
            let body_index = encoded & Bodies::BODY_REFERENCE_MASK;
            if bodies.remove_constraint_reference(body_index, constraint_handle) && Bodies::is_encoded_kinematic_reference(encoded) {
                let body_handle = bodies.active_set().index_to_handle.get(body_index).0;
                self.constrained_kinematic_handles.fast_remove(&body_handle);
            }
        }
    }

    /// Enumerates connected raw body references for a given constraint handle.
    pub unsafe fn enumerate_connected_raw_body_references<E: IForEach<i32>>(&self, constraint_handle: ConstraintHandle, enumerator: &mut E) {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let set = self.sets.get(location.set_index);
        let batch = set.batches.get(location.batch_index);
        let type_batch = batch.get_type_batch(location.type_id);
        self.enumerate_connected_body_references_from_type_batch(type_batch, location.index_in_type_batch, location.type_id, enumerator, EnumerateMode::Raw);
    }

    /// Enumerates connected dynamic body references for a given constraint handle.
    pub unsafe fn enumerate_connected_dynamic_bodies<E: IForEach<i32>>(&self, constraint_handle: ConstraintHandle, enumerator: &mut E) {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let set = self.sets.get(location.set_index);
        let batch = set.batches.get(location.batch_index);
        let type_batch = batch.get_type_batch(location.type_id);
        self.enumerate_connected_body_references_from_type_batch(type_batch, location.index_in_type_batch, location.type_id, enumerator, EnumerateMode::DynamicOnly);
    }

    unsafe fn enumerate_connected_body_references_from_type_batch<E: IForEach<i32>>(
        &self,
        type_batch: &TypeBatch,
        index_in_type_batch: i32,
        type_id: i32,
        enumerator: &mut E,
        mode: EnumerateMode,
    ) {
        let bodies_per_constraint = self.type_processors[type_id as usize].as_ref().unwrap().bodies_per_constraint;
        let bytes_per_bundle = bodies_per_constraint * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32;
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        let start_byte = bundle_index as i32 * bytes_per_bundle + inner_index as i32 * 4;

        for i in 0..bodies_per_constraint {
            let raw = *(type_batch.body_references.as_ptr().add(
                (start_byte + i * std::mem::size_of::<Simd<i32, { crate::utilities::vector::optimal_lanes::<i32>() }>>() as i32) as usize,
            ) as *const i32);

            match mode {
                EnumerateMode::Raw => enumerator.loop_body(raw),
                EnumerateMode::Decoded => enumerator.loop_body(raw & Bodies::BODY_REFERENCE_MASK),
                EnumerateMode::DynamicOnly => {
                    if Bodies::is_encoded_dynamic_reference(raw) {
                        enumerator.loop_body(raw & Bodies::BODY_REFERENCE_MASK);
                    }
                }
            }
        }
    }

    pub(crate) fn get_synchronized_batch_count(&self) -> (i32, bool) {
        let batch_count = self.active_set().batches.count;
        let synchronized = batch_count.min(self.fallback_batch_threshold);
        let fallback_exists = batch_count > self.fallback_batch_threshold;
        debug_assert!(
            batch_count <= self.fallback_batch_threshold + 1,
            "There cannot be more than FallbackBatchThreshold + 1 constraint batches."
        );
        (synchronized, fallback_exists)
    }

    /// Updates constraint body references when a body transitions from dynamic to kinematic.
    /// Any constraints that connect only kinematic bodies together will be removed.
    pub(crate) unsafe fn update_references_for_body_becoming_kinematic(&mut self, body_handle: BodyHandle, body_index: i32) {
        let self_ptr = self as *mut Self;
        let bodies = &*(*self_ptr).bodies;
        // bodies[bodyHandle].Kinematic in C#
        debug_assert!({
            let loc = *bodies.handle_to_location.get(body_handle.0);
            Bodies::is_kinematic(unsafe { &(*bodies.active_set().dynamics_state.get(loc.index)).inertia.local })
        });

        // DynamicToKinematicEnumerator: simply counts dynamic body references.
        struct DynamicToKinematicEnumerator {
            dynamic_count: i32,
        }
        impl IForEach<i32> for DynamicToKinematicEnumerator {
            fn loop_body(&mut self, _encoded_body_reference: i32) {
                self.dynamic_count += 1;
            }
        }

        let constraints = &bodies.active_set().constraints.get(body_index);
        let mut present_in_fallback = false;

        // Note reverse iteration: if the solver removes a constraint for being between two kinematics,
        // we don't want to break enumeration.
        for i in (0..constraints.count).rev() {
            let constraint = *constraints.get(i);
            let constraint_handle = constraint.connecting_constraint_handle;
            let mut enumerator = DynamicToKinematicEnumerator { dynamic_count: 0 };
            (*self_ptr).enumerate_connected_dynamic_bodies(constraint_handle, &mut enumerator);

            if enumerator.dynamic_count == 1 {
                // This body was the only dynamic. The constraint now connects only kinematics — remove it.
                (*self_ptr).remove(constraint_handle);
            } else {
                // The constraint survived. Update the kinematic flag for this body's reference.
                let location = *(*self_ptr).handle_to_constraint.get(constraint_handle.0);
                let batch = (*self_ptr).sets.get_mut(location.set_index).batches.get_mut(location.batch_index);
                let type_batch = batch.get_type_batch_mut(location.type_id);
                let bodies_per_constraint = (&(*self_ptr).type_processors)[location.type_id as usize]
                    .as_ref()
                    .unwrap()
                    .bodies_per_constraint;
                let ints_per_bundle = Vector::<i32>::LEN as i32 * bodies_per_constraint;
                let (mut bundle_index, mut inner_index) = (0usize, 0usize);
                BundleIndexing::get_bundle_indices(location.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
                let first_body_reference = type_batch.body_references.as_ptr() as *mut u32;
                let body_ref_slot = first_body_reference.add(
                    (ints_per_bundle as usize * bundle_index)
                        + inner_index
                        + (constraint.body_index_in_constraint as usize * Vector::<i32>::LEN),
                );
                let old_dynamic_index = *body_ref_slot;
                *body_ref_slot = old_dynamic_index | Bodies::KINEMATIC_MASK;

                if location.batch_index < (*self_ptr).fallback_batch_threshold {
                    // Non-fallback batch: remove the former dynamic's handle reference.
                    (*self_ptr)
                        .batch_referenced_handles
                        .get_mut(location.batch_index)
                        .remove(body_handle.0);
                } else {
                    present_in_fallback = true;
                }
            }
        }

        if present_in_fallback {
            // Body is now kinematic — remove it from fallback tracking entirely.
            let mut ids_buf = [0i32; 3];
            let ids_buffer = Buffer::new(ids_buf.as_mut_ptr(), 3, -1);
            let mut allocation_ids_to_free = QuickList::<i32>::new(ids_buffer);
            (*self_ptr).try_remove_dynamic_body_from_fallback(body_handle, body_index, &mut allocation_ids_to_free);
            let pool = &mut *(*self_ptr).pool;
            for i in 0..allocation_ids_to_free.count {
                pool.return_unsafely(*allocation_ids_to_free.get(i));
            }
        }

        let constraints = &(*(*self_ptr).bodies).active_set().constraints.get(body_index);
        if constraints.count > 0 {
            // This body is now kinematic and constrained — add to constrained kinematics set.
            let pool = &mut *(*self_ptr).pool;
            (*self_ptr).constrained_kinematic_handles.add(&body_handle.0, pool);
        }
    }

    /// Tries to remove a dynamic body from the fallback batch tracking.
    pub(crate) unsafe fn try_remove_dynamic_body_from_fallback(
        &mut self,
        body_handle: BodyHandle,
        body_index: i32,
        allocation_ids_to_free: &mut QuickList<i32>,
    ) {
        if self
            .active_set_mut()
            .sequential_fallback
            .try_remove_dynamic_body_from_tracking(body_index, allocation_ids_to_free)
        {
            self.batch_referenced_handles
                .get_mut(self.fallback_batch_threshold)
                .unset(body_handle.0);
        }
    }

    /// Updates constraint body references when a body transitions from kinematic to dynamic.
    /// Ensures constraints belong to appropriate batches for the new dynamic status.
    pub(crate) unsafe fn update_references_for_body_becoming_dynamic(&mut self, body_handle: BodyHandle, body_index: i32) {
        let self_ptr = self as *mut Self;
        let bodies = &*(*self_ptr).bodies;

        // KinematicToDynamicEnumerator: collects dynamic body handles and encoded body indices.
        const MAXIMUM_BODIES_PER_CONSTRAINT: usize = 4;

        struct KinematicToDynamicEnumerator<'a> {
            index_to_handle: &'a Buffer<BodyHandle>,
            dynamic_body_handles: *mut i32,
            dynamic_count: i32,
            encoded_body_indices: *mut i32,
            encoded_count: i32,
        }
        impl<'a> IForEach<i32> for KinematicToDynamicEnumerator<'a> {
            fn loop_body(&mut self, encoded_body_reference: i32) {
                unsafe {
                    debug_assert!(
                        (self.encoded_count as usize) < MAXIMUM_BODIES_PER_CONSTRAINT,
                        "Assumed max bodies per constraint exceeded"
                    );
                    if Bodies::is_encoded_dynamic_reference(encoded_body_reference) {
                        *self.dynamic_body_handles.add(self.dynamic_count as usize) =
                            self.index_to_handle.get(encoded_body_reference).0;
                        self.dynamic_count += 1;
                    }
                    *self.encoded_body_indices.add(self.encoded_count as usize) = encoded_body_reference;
                    self.encoded_count += 1;
                }
            }
        }

        let mut dynamic_body_handles_buf = [0i32; MAXIMUM_BODIES_PER_CONSTRAINT];
        let mut encoded_body_indices_buf = [0i32; MAXIMUM_BODIES_PER_CONSTRAINT];
        let index_to_handle = &bodies.active_set().index_to_handle;
        let constraints = &bodies.active_set().constraints.get(body_index);

        for constraint_index in 0..constraints.count {
            let constraint = *constraints.get(constraint_index);
            let mut enumerator = KinematicToDynamicEnumerator {
                index_to_handle,
                dynamic_body_handles: dynamic_body_handles_buf.as_mut_ptr(),
                dynamic_count: 0,
                encoded_body_indices: encoded_body_indices_buf.as_mut_ptr(),
                encoded_count: 0,
            };
            (*self_ptr).enumerate_connected_raw_body_references(constraint.connecting_constraint_handle, &mut enumerator);

            // Since we haven't updated the constraint reference to this body's kinematicity yet,
            // it was not included in the dynamicBodyHandles. Include it here.
            dynamic_body_handles_buf[enumerator.dynamic_count as usize] = body_handle.0;
            enumerator.dynamic_count += 1;

            // Remove the kinematic flag from the body's encoded index.
            encoded_body_indices_buf[constraint.body_index_in_constraint as usize] &= Bodies::BODY_REFERENCE_MASK;

            let dynamic_handles_slice: Vec<BodyHandle> = dynamic_body_handles_buf[..enumerator.dynamic_count as usize]
                .iter()
                .map(|&v| BodyHandle(v))
                .collect();
            let encoded_slice = &encoded_body_indices_buf[..enumerator.encoded_count as usize];

            let (synchronized_batch_count, fallback_exists) = (*self_ptr).get_synchronized_batch_count();
            let constraint_location = *(*self_ptr).handle_to_constraint.get(constraint.connecting_constraint_handle.0);

            let mut target_batch_index = -1i32;
            for batch_index in 0..synchronized_batch_count {
                let dynamic_handle_ints: Vec<i32> = dynamic_handles_slice.iter().map(|h| h.0).collect();
                if (*self_ptr).batch_referenced_handles.get(batch_index).can_fit(&dynamic_handle_ints) {
                    debug_assert!(
                        batch_index != constraint_location.batch_index,
                        "It should not be possible for a newly dynamic reference to insert itself into the same batch."
                    );
                    target_batch_index = batch_index;
                    break;
                }
            }
            if target_batch_index == -1 {
                if fallback_exists {
                    target_batch_index = (*self_ptr).fallback_batch_threshold;
                } else {
                    target_batch_index = (*self_ptr).allocate_new_constraint_batch();
                }
            }

            // Perform the transfer.
            let batch = (*self_ptr).sets.get(0).batches.get(constraint_location.batch_index);
            let type_batch_index = *batch.type_index_to_type_batch_index.get(constraint_location.type_id);
            let type_batch = (*self_ptr)
                .sets
                .get_mut(0)
                .batches
                .get_mut(constraint_location.batch_index)
                .type_batches
                .get_mut(type_batch_index);

            let type_proc = (&(*self_ptr).type_processors)[constraint_location.type_id as usize]
                .as_ref()
                .unwrap();
            type_proc.inner().transfer_constraint(
                type_batch,
                constraint_location.batch_index,
                constraint_location.index_in_type_batch,
                self_ptr,
                (*self_ptr).bodies,
                target_batch_index,
                &dynamic_handles_slice,
                encoded_slice,
            );
        }

        let constraints = &(*(*self_ptr).bodies).active_set().constraints.get(body_index);
        if constraints.count > 0 {
            (*self_ptr).constrained_kinematic_handles.fast_remove(&body_handle.0);
        }
    }

    /// Changes the body references of all constraints associated with a body in response to its movement into a new slot.
    pub(crate) unsafe fn update_for_body_memory_move(&mut self, original_body_index: i32, new_body_location: i32) {
        if self.update_constraints_for_body_memory_move(original_body_index, new_body_location) {
            self.active_set_mut().sequential_fallback.update_for_dynamic_body_memory_move(original_body_index, new_body_location);
        }
    }

    unsafe fn update_constraints_for_body_memory_move(&mut self, original_index: i32, new_index: i32) -> bool {
        let self_ptr = self as *mut Self;
        let bodies = &*(*self_ptr).bodies;
        let list = &bodies.active_set().constraints.get(original_index);
        let mut body_should_be_present_in_fallback = false;
        for i in 0..list.count {
            let constraint = list.get(i);
            let constraint_location = *(*self_ptr).handle_to_constraint.get(constraint.connecting_constraint_handle.0);
            let type_proc = (&(*self_ptr).type_processors)[constraint_location.type_id as usize].as_ref().unwrap();
            let batch = (*self_ptr).sets.get_mut(0).batches.get_mut(constraint_location.batch_index);
            let type_batch = batch.get_type_batch_mut(constraint_location.type_id);
            let body_is_kinematic = type_proc.inner().update_for_body_memory_move(
                type_batch,
                constraint_location.index_in_type_batch,
                constraint.body_index_in_constraint,
                new_index,
            );
            if !body_is_kinematic && constraint_location.batch_index == (*self_ptr).fallback_batch_threshold {
                body_should_be_present_in_fallback = true;
            }
        }
        body_should_be_present_in_fallback
    }

    /// Scales all accumulated impulses in the active set.
    pub fn scale_active_accumulated_impulses(&mut self, scale: f32) {
        self.scale_accumulated_impulses_in_set(0, scale);
    }

    /// Scales all accumulated impulses in all constraint sets.
    pub fn scale_all_accumulated_impulses(&mut self, scale: f32) {
        for i in 0..self.sets.len() {
            if self.sets.get(i).allocated() {
                self.scale_accumulated_impulses_in_set(i, scale);
            }
        }
    }

    fn scale_accumulated_impulses_in_set(&mut self, set_index: i32, scale: f32) {
        let self_ptr = self as *mut Self;
        unsafe {
            let set = (*self_ptr).sets.get_mut(set_index);
            for batch_index in 0..set.batches.count {
                let batch = set.batches.get_mut(batch_index);
                for type_batch_index in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get_mut(type_batch_index);
                    let type_id = type_batch.type_id;
                    (&(*self_ptr).type_processors)[type_id as usize]
                        .as_ref()
                        .unwrap()
                        .inner()
                        .scale_accumulated_impulses(type_batch, scale);
                }
            }
        }
    }

    /// Enumerates connected raw body references from a type batch.
    pub unsafe fn enumerate_connected_raw_body_references_from_type_batch<E: IForEach<i32>>(
        &self,
        type_batch: &TypeBatch,
        index_in_type_batch: i32,
        enumerator: &mut E,
    ) {
        self.enumerate_connected_body_references_from_type_batch(
            type_batch,
            index_in_type_batch,
            type_batch.type_id,
            enumerator,
            EnumerateMode::Raw,
        );
    }

    /// Enumerates connected decoded (handle-valued) body references from a type batch.
    pub unsafe fn enumerate_connected_body_references<E: IForEach<i32>>(
        &self,
        type_batch: &TypeBatch,
        index_in_type_batch: i32,
        enumerator: &mut E,
    ) {
        self.enumerate_connected_body_references_from_type_batch(
            type_batch,
            index_in_type_batch,
            type_batch.type_id,
            enumerator,
            EnumerateMode::Decoded,
        );
    }

    /// Enumerates connected decoded body references for a constraint handle.
    pub unsafe fn enumerate_connected_body_references_by_handle<E: IForEach<i32>>(
        &self,
        constraint_handle: ConstraintHandle,
        enumerator: &mut E,
    ) {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let set = self.sets.get(location.set_index);
        let batch = set.batches.get(location.batch_index);
        let type_batch = batch.get_type_batch(location.type_id);
        self.enumerate_connected_body_references_from_type_batch(type_batch, location.index_in_type_batch, location.type_id, enumerator, EnumerateMode::Decoded);
    }

    /// Enumerates connected dynamic bodies from a type batch (decoded, dynamic only).
    pub unsafe fn enumerate_connected_dynamic_bodies_from_type_batch<E: IForEach<i32>>(
        &self,
        type_batch: &TypeBatch,
        index_in_type_batch: i32,
        enumerator: &mut E,
    ) {
        self.enumerate_connected_body_references_from_type_batch(
            type_batch,
            index_in_type_batch,
            type_batch.type_id,
            enumerator,
            EnumerateMode::DynamicOnly,
        );
    }

    /// Enumerates accumulated impulses for a constraint, collecting each scalar DOF impulse.
    pub unsafe fn enumerate_accumulated_impulses<E: IForEach<f32>>(
        &self,
        constraint_handle: ConstraintHandle,
        enumerator: &mut E,
    ) {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let set = self.sets.get(location.set_index);
        let type_batch = set.batches.get(location.batch_index).get_type_batch(location.type_id);
        debug_assert!(location.index_in_type_batch >= 0 && location.index_in_type_batch < type_batch.constraint_count);
        let type_processor = self.type_processors[location.type_id as usize].as_ref().unwrap();
        let constrained_dof = type_processor.constrained_degrees_of_freedom;
        let vector_width = crate::utilities::vector::VECTOR_WIDTH;
        let bundle_size_in_floats = constrained_dof as usize * vector_width;
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(location.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        let mut impulse_address = (type_batch.accumulated_impulses.as_ptr() as *const f32)
            .add(bundle_index * bundle_size_in_floats + inner_index);
        enumerator.loop_body(*impulse_address);
        for _ in 1..constrained_dof {
            impulse_address = impulse_address.add(vector_width);
            enumerator.loop_body(*impulse_address);
        }
    }

    /// Computes the squared magnitude of accumulated impulses for a constraint.
    pub unsafe fn get_accumulated_impulse_magnitude_squared(&self, constraint_handle: ConstraintHandle) -> f32 {
        let location = *self.handle_to_constraint.get(constraint_handle.0);
        let type_processor = self.type_processors[location.type_id as usize].as_ref().unwrap();
        let dof = type_processor.constrained_degrees_of_freedom;
        debug_assert!(dof <= 16, "Constrained DOF exceeds stack buffer size");
        let mut impulses = [0.0f32; 16];
        let mut collector = crate::physics::handy_enumerators::FloatCollector::new(impulses.as_mut_ptr());
        self.enumerate_accumulated_impulses(constraint_handle, &mut collector);
        let mut sum_of_squares = 0.0f32;
        for i in 0..dof as usize {
            sum_of_squares += impulses[i] * impulses[i];
        }
        sum_of_squares
    }

    /// Computes the magnitude of accumulated impulses for a constraint.
    pub unsafe fn get_accumulated_impulse_magnitude(&self, constraint_handle: ConstraintHandle) -> f32 {
        self.get_accumulated_impulse_magnitude_squared(constraint_handle).sqrt()
    }

    /// Gets the description of a constraint given a constraint reference.
    pub unsafe fn get_description<TDescription: IConstraintDescription>(
        &self,
        reference: &ConstraintReference,
    ) -> TDescription {
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(reference.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        debug_assert!((*reference.type_batch_pointer).type_id == TDescription::constraint_type_id());
        TDescription::build_description(&*reference.type_batch_pointer, bundle_index as i32, inner_index as i32)
    }

    /// Gets the description of a constraint given a constraint handle.
    pub unsafe fn get_description_by_handle<TDescription: IConstraintDescription>(
        &self,
        handle: ConstraintHandle,
    ) -> TDescription {
        let location = *self.handle_to_constraint.get(handle.0);
        debug_assert!(TDescription::constraint_type_id() == location.type_id);
        let type_batch = self.sets.get(location.set_index)
            .batches.get(location.batch_index)
            .get_type_batch(location.type_id);
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(location.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        TDescription::build_description(type_batch, bundle_index as i32, inner_index as i32)
    }

    /// Applies a description to a constraint without waking, using handle lookup.
    pub unsafe fn apply_description_without_waking_by_handle<TDescription: IConstraintDescription>(
        &mut self,
        handle: ConstraintHandle,
        description: &TDescription,
    ) {
        let reference = self.get_constraint_reference(handle);
        let (mut bundle_index, mut inner_index) = (0usize, 0usize);
        BundleIndexing::get_bundle_indices(reference.index_in_type_batch as usize, &mut bundle_index, &mut inner_index);
        description.apply_description(&mut *reference.type_batch_pointer, bundle_index as i32, inner_index as i32);
    }

    /// Applies a description to a constraint, waking the affected island first.
    pub unsafe fn apply_description<TDescription: IConstraintDescription>(
        &mut self,
        handle: ConstraintHandle,
        description: &TDescription,
    ) {
        if !self.awakener.is_null() {
            (*self.awakener).awaken_constraint(handle);
        }
        self.apply_description_without_waking_by_handle(handle, description);
    }

    /// Adds a one-body constraint using a typed description.
    pub unsafe fn add_one_body<TDescription: IOneBodyConstraintDescription>(
        &mut self,
        body_handle: BodyHandle,
        description: &TDescription,
    ) -> ConstraintHandle {
        let body_handles = [body_handle];
        self.add(&body_handles, TDescription::constraint_type_id(), |batch, bundle_index, inner_index| {
            description.apply_description(batch, bundle_index, inner_index);
        })
    }

    /// Adds a two-body constraint using a typed description.
    pub unsafe fn add_two_body<TDescription: ITwoBodyConstraintDescription>(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        description: &TDescription,
    ) -> ConstraintHandle {
        let body_handles = [body_handle_a, body_handle_b];
        self.add(&body_handles, TDescription::constraint_type_id(), |batch, bundle_index, inner_index| {
            description.apply_description(batch, bundle_index, inner_index);
        })
    }

    /// Adds a three-body constraint using a typed description.
    pub unsafe fn add_three_body<TDescription: IThreeBodyConstraintDescription>(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        body_handle_c: BodyHandle,
        description: &TDescription,
    ) -> ConstraintHandle {
        let body_handles = [body_handle_a, body_handle_b, body_handle_c];
        self.add(&body_handles, TDescription::constraint_type_id(), |batch, bundle_index, inner_index| {
            description.apply_description(batch, bundle_index, inner_index);
        })
    }

    /// Adds a four-body constraint using a typed description.
    pub unsafe fn add_four_body<TDescription: IFourBodyConstraintDescription>(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        body_handle_c: BodyHandle,
        body_handle_d: BodyHandle,
        description: &TDescription,
    ) -> ConstraintHandle {
        let body_handles = [body_handle_a, body_handle_b, body_handle_c, body_handle_d];
        self.add(&body_handles, TDescription::constraint_type_id(), |batch, bundle_index, inner_index| {
            description.apply_description(batch, bundle_index, inner_index);
        })
    }

    // --- Capacity management ---

    pub fn ensure_solver_capacities(&mut self, body_handle_capacity: i32, constraint_handle_capacity: i32) {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            if (*self_ptr).handle_to_constraint.len() < constraint_handle_capacity {
                let copy_count = (*self_ptr).handle_pool.highest_possibly_claimed_id() + 1;
                pool.resize_to_at_least(
                    &mut (*self_ptr).handle_to_constraint,
                    constraint_handle_capacity,
                    copy_count,
                );
            }
            let bodies = &*(*self_ptr).bodies;
            let target = (bodies.handle_pool.highest_possibly_claimed_id() + 1).max(body_handle_capacity);
            for i in 0..(*self_ptr).sets.get(0).batches.count {
                (*self_ptr).batch_referenced_handles.get_mut(i).ensure_capacity(target, pool);
            }
            (*self_ptr).constrained_kinematic_handles.ensure_capacity(body_handle_capacity, pool);
        }
    }

    fn resize_handle_capacity(&mut self, constraint_handle_capacity: i32) {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            let copy_count = (*self_ptr).handle_pool.highest_possibly_claimed_id() + 1;
            pool.resize_to_at_least(
                &mut (*self_ptr).handle_to_constraint,
                constraint_handle_capacity,
                copy_count,
            );
            for i in ((*self_ptr).handle_pool.highest_possibly_claimed_id() + 1)..(*self_ptr).handle_to_constraint.len() {
                (*self_ptr).handle_to_constraint.get_mut(i).set_index = -1;
            }
        }
    }

    pub fn resize_solver_capacities(&mut self, body_handle_capacity: i32, constraint_handle_capacity: i32) {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            let target_constraint = BufferPool::get_capacity_for_count::<ConstraintLocation>(
                constraint_handle_capacity.max((*self_ptr).handle_pool.highest_possibly_claimed_id() + 1),
            );
            if (*self_ptr).handle_to_constraint.len() != target_constraint {
                (*self_ptr).resize_handle_capacity(target_constraint);
            }
            let bodies = &*(*self_ptr).bodies;
            let target_handles = (bodies.handle_pool.highest_possibly_claimed_id() + 1).max(body_handle_capacity);
            for i in 0..(*self_ptr).sets.get(0).batches.count {
                (*self_ptr).batch_referenced_handles.get_mut(i).resize(target_handles, pool);
            }
            let target_kinematic = (*self_ptr).constrained_kinematic_handles.count.max(body_handle_capacity);
            (*self_ptr).constrained_kinematic_handles.resize(target_kinematic, pool);
        }
    }

    pub(crate) fn resize_sets_capacity(&mut self, sets_capacity: i32, potentially_allocated_count: i32) {
        let pool = unsafe { &mut *self.pool };
        let target = BufferPool::get_capacity_for_count::<ConstraintSet>(sets_capacity);
        if self.sets.len() != target {
            let old_capacity = self.sets.len();
            // ConstraintSet doesn't implement Copy, so we use manual buffer management.
            // Take a new buffer and copy the existing data via raw pointer copy.
            let mut new_sets: Buffer<ConstraintSet> = pool.take_at_least(target);
            let copy_count = potentially_allocated_count.min(old_capacity).min(new_sets.len());
            if copy_count > 0 {
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        self.sets.as_ptr() as *const u8,
                        new_sets.as_mut_ptr() as *mut u8,
                        copy_count as usize * std::mem::size_of::<ConstraintSet>(),
                    );
                }
            }
            if self.sets.allocated() {
                pool.return_buffer(&mut self.sets);
            }
            self.sets = new_sets;
            if old_capacity < self.sets.len() {
                self.sets.clear(old_capacity, self.sets.len() - old_capacity);
            }
        }
    }

    /// Ensures all existing active type batches meet or exceed the current solver-defined minimum capacities.
    pub fn ensure_type_batch_capacities(&mut self) {
        let self_ptr = self as *mut Self;
        unsafe {
            let set = (*self_ptr).sets.get_mut(0);
            for i in 0..set.batches.count {
                let batch = set.batches.get_mut(i);
                for j in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get_mut(j);
                    let min = (*self_ptr).get_minimum_capacity_for_type(type_batch.type_id);
                    if type_batch.capacity() < min {
                        let type_id = type_batch.type_id;
                        let pool = &mut *(*self_ptr).pool;
                        (&(*self_ptr).type_processors)[type_id as usize]
                            .as_ref()
                            .unwrap()
                            .inner()
                            .resize(type_batch, min, pool);
                    }
                }
            }
        }
    }

    /// Applies the current solver-defined minimum capacities to existing type batches (may shrink or grow).
    pub fn resize_type_batch_capacities(&mut self) {
        let self_ptr = self as *mut Self;
        unsafe {
            let set = (*self_ptr).sets.get_mut(0);
            for i in 0..set.batches.count {
                let batch = set.batches.get_mut(i);
                for j in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get_mut(j);
                    let target = (*self_ptr).get_minimum_capacity_for_type(type_batch.type_id);
                    let current = type_batch.capacity();
                    if current != target {
                        let type_id = type_batch.type_id;
                        let pool = &mut *(*self_ptr).pool;
                        (&(*self_ptr).type_processors)[type_id as usize]
                            .as_ref()
                            .unwrap()
                            .inner()
                            .resize(type_batch, target, pool);
                    }
                }
            }
        }
    }

    /// Removes all objects from the solver.
    pub fn clear(&mut self) {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            let batch_count = (*self_ptr).sets.get(0).batches.count;
            for batch_index in 0..batch_count {
                (*self_ptr).batch_referenced_handles.get_mut(batch_index).dispose(pool);
            }
            (*self_ptr).constrained_kinematic_handles.clear();
            (*self_ptr).batch_referenced_handles.clear();
            (*self_ptr).sets.get_mut(0).clear(pool);
            // All inactive sets are returned to the pool.
            for i in 1..(*self_ptr).sets.len() {
                if (*self_ptr).sets.get(i).allocated() {
                    (*self_ptr).sets.get_mut(i).dispose(pool);
                }
            }
            (*self_ptr).handle_pool.clear();
        }
    }

    /// Returns all pool-retrieved resources to the pool.
    pub fn dispose(&mut self) {
        let self_ptr = self as *mut Self;
        unsafe {
            let pool = &mut *(*self_ptr).pool;
            let batch_count = (*self_ptr).sets.get(0).batches.count;
            for i in 0..batch_count {
                (*self_ptr).batch_referenced_handles.get_mut(i).dispose(pool);
            }
            (*self_ptr).batch_referenced_handles.dispose(pool);
            (*self_ptr).constrained_kinematic_handles.dispose(pool);
            for i in 0..(*self_ptr).sets.len() {
                if (*self_ptr).sets.get(i).allocated() {
                    (*self_ptr).sets.get_mut(i).dispose(pool);
                }
            }
            pool.return_buffer(&mut (*self_ptr).sets);
            pool.return_buffer(&mut (*self_ptr).handle_to_constraint);
            (*self_ptr).handle_pool.dispose(pool);
        }
    }
}

// --- Solver_Solve types (translated from Solver_Solve.cs) ---

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub(crate) enum SolverStageType {
    IncrementalUpdate = 0,
    IntegrateConstrainedKinematics = 1,
    WarmStart = 2,
    Solve = 3,
}

#[repr(C)]
pub(crate) struct SolverSyncStage {
    pub claims: Buffer<i32>,
    pub batch_index: i32,
    pub work_block_start_index: i32,
    pub stage_type: SolverStageType,
}

impl SolverSyncStage {
    pub fn new(claims: Buffer<i32>, work_block_start_index: i32, stage_type: SolverStageType, batch_index: i32) -> Self {
        Self {
            claims,
            batch_index,
            work_block_start_index,
            stage_type,
        }
    }

    pub fn new_no_batch(claims: Buffer<i32>, work_block_start_index: i32, stage_type: SolverStageType) -> Self {
        Self::new(claims, work_block_start_index, stage_type, -1)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct WorkBlock {
    pub batch_index: i32,
    pub type_batch_index: i32,
    /// Index of the first bundle in the block.
    pub start_bundle: i32,
    /// Exclusive end index of the bundle. Index of the last bundle in the block is End - 1.
    pub end: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct IntegrationWorkBlock {
    pub start_bundle_index: i32,
    pub end_bundle_index: i32,
}

/// Multithreading context for substepped constraint solving.
/// Uses explicit layout with cache-line isolation between fields
/// written by different threads to prevent false sharing.
#[repr(C)]
pub(crate) struct SubstepMultithreadingContext {
    // --- Frequently read, rarely written (offsets 0..112) ---
    pub stages: Buffer<SolverSyncStage>,           // offset 0  (16 bytes)
    pub incremental_update_blocks: Buffer<WorkBlock>, // offset 16 (16 bytes)
    pub kinematic_integration_blocks: Buffer<IntegrationWorkBlock>, // offset 32 (16 bytes)
    pub constraint_blocks: Buffer<WorkBlock>,       // offset 48 (16 bytes)
    pub constraint_batch_boundaries: Buffer<i32>,   // offset 64 (16 bytes)
    pub dt: f32,                                    // offset 80
    pub inverse_dt: f32,                            // offset 84
    pub worker_count: i32,                          // offset 88
    pub highest_velocity_iteration_count: i32,      // offset 92
    pub velocity_iteration_counts: Buffer<i32>,     // offset 96 (16 bytes)

    // Padding to push SyncIndex to a separate cache line (offset 256)
    _pad0: [u8; 256 - 112],

    /// Monotonically increasing index of executed stages during a frame.
    /// Written by the main thread via Volatile.Write; read by workers via Volatile.Read.
    /// Uses UnsafeCell for mixed atomic/plain access pattern.
    pub sync_index: std::cell::UnsafeCell<i32>,     // offset 256

    // Padding to push CompletedWorkBlockCount to another cache line (offset 384)
    _pad1: [u8; 384 - 260],

    /// Counter of work completed for the current stage.
    /// Atomically incremented by workers, read/reset by main thread.
    pub completed_work_block_count: std::cell::UnsafeCell<i32>, // offset 384
}

// Compile-time layout assertions matching C# [StructLayout(LayoutKind.Explicit)]
const _: () = {
    assert!(std::mem::offset_of!(SubstepMultithreadingContext, sync_index) == 256);
    assert!(std::mem::offset_of!(SubstepMultithreadingContext, completed_work_block_count) == 384);
};

impl Default for SubstepMultithreadingContext {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

/// Trait for stage function dispatch in the MT solver.
/// Each stage function knows how to execute a specific work block type.
pub(crate) trait ISolverStageFunction {
    unsafe fn execute(&self, solver: *mut Solver, block_index: i32, worker_index: i32);
}

/// Warm start stage: applies accumulated impulses to body velocities.
pub(crate) struct WarmStartStageFunction {
    pub dt: f32,
    pub inverse_dt: f32,
    pub substep_index: i32,
    pub solver: *mut Solver,
}

impl ISolverStageFunction for WarmStartStageFunction {
    #[inline(always)]
    unsafe fn execute(&self, solver: *mut Solver, block_index: i32, worker_index: i32) {
        let ctx = &(*self.solver).substep_context;
        let block = ctx.constraint_blocks.get(block_index);
        let active_set = (*solver).sets.get_mut(0);
        let type_batch = active_set.batches.get_mut(block.batch_index)
            .type_batches.get_mut(block.type_batch_index);
        let type_id = type_batch.type_id;
        let processor = (&(*solver).type_processors)[type_id as usize].as_ref().unwrap();
        let allow_pose_integration = self.substep_index > 0;
        (*self.solver).warm_start_block(
            worker_index,
            block.batch_index,
            block.type_batch_index,
            block.start_bundle,
            block.end,
            type_batch,
            processor.inner(),
            allow_pose_integration,
            self.dt,
            self.inverse_dt,
        );
    }
}

/// Solve stage: iterative velocity solve.
pub(crate) struct SolveStageFunction {
    pub dt: f32,
    pub inverse_dt: f32,
    pub solver: *mut Solver,
}

impl ISolverStageFunction for SolveStageFunction {
    #[inline(always)]
    unsafe fn execute(&self, solver: *mut Solver, block_index: i32, worker_index: i32) {
        let ctx = &(*self.solver).substep_context;
        let block = ctx.constraint_blocks.get(block_index);
        let active_set = (*solver).sets.get_mut(0);
        let type_batch = active_set.batches.get_mut(block.batch_index)
            .type_batches.get_mut(block.type_batch_index);
        let type_id = type_batch.type_id;
        let processor = (&(*solver).type_processors)[type_id as usize].as_ref().unwrap();
        let bodies = &*(*solver).bodies;
        processor.inner().solve(
            type_batch, bodies, self.dt, self.inverse_dt,
            block.start_bundle, block.end,
        );
        let _ = worker_index;
    }
}

/// Incremental update stage: updates constraint data for substeps > 0.
pub(crate) struct IncrementalUpdateStageFunction {
    pub dt: f32,
    pub inverse_dt: f32,
    pub solver: *mut Solver,
}

impl ISolverStageFunction for IncrementalUpdateStageFunction {
    #[inline(always)]
    unsafe fn execute(&self, solver: *mut Solver, block_index: i32, worker_index: i32) {
        let ctx = &(*self.solver).substep_context;
        let block = ctx.incremental_update_blocks.get(block_index);
        let active_set = (*solver).sets.get_mut(0);
        let type_batch = active_set.batches.get_mut(block.batch_index)
            .type_batches.get_mut(block.type_batch_index);
        let type_id = type_batch.type_id;
        let processor = (&(*solver).type_processors)[type_id as usize].as_ref().unwrap();
        let bodies = &*(*solver).bodies;
        processor.inner().incrementally_update_for_substep(
            type_batch, bodies, self.dt, self.inverse_dt,
            block.start_bundle, block.end,
        );
        let _ = worker_index;
    }
}

/// Integrate constrained kinematics stage: integrates kinematic velocities/poses.
pub(crate) struct IntegrateConstrainedKinematicsStageFunction {
    pub dt: f32,
    pub inverse_dt: f32,
    pub substep_index: i32,
    pub solver: *mut Solver,
}

impl ISolverStageFunction for IntegrateConstrainedKinematicsStageFunction {
    #[inline(always)]
    unsafe fn execute(&self, solver: *mut Solver, block_index: i32, worker_index: i32) {
        let ctx = &(*self.solver).substep_context;
        let block = ctx.kinematic_integration_blocks.get(block_index);
        let handles = &(*solver).constrained_kinematic_handles;
        let handle_buf = handles.span.slice_count(handles.count);
        if let Some(pi_ptr) = (*solver).pose_integrator {
            let pi = &*pi_ptr;
            if self.substep_index == 0 {
                pi.integrate_kinematic_velocities_dispatch(
                    &handle_buf,
                    block.start_bundle_index,
                    block.end_bundle_index,
                    self.dt,
                    worker_index,
                );
            } else {
                pi.integrate_kinematic_poses_and_velocities_dispatch(
                    &handle_buf,
                    block.start_bundle_index,
                    block.end_bundle_index,
                    self.dt,
                    worker_index,
                );
            }
        }
    }
}

/// Filter trait for determining which type batches should be included in work blocks.
pub(crate) trait ITypeBatchSolveFilter {
    fn include_fallback_batch_for_work_blocks(&self) -> bool;
    fn allow_type(&self, type_id: i32) -> bool;
}

/// Filter for the main solve/warmstart pass — includes all types but excludes fallback batch.
pub(crate) struct MainSolveFilter;
impl ITypeBatchSolveFilter for MainSolveFilter {
    #[inline(always)]
    fn include_fallback_batch_for_work_blocks(&self) -> bool { false }
    #[inline(always)]
    fn allow_type(&self, _type_id: i32) -> bool { true }
}

/// Filter for incremental update — only types with incremental substep updates, includes fallback.
pub(crate) struct IncrementalUpdateForSubstepFilter<'a> {
    pub type_processors: &'a Vec<Option<TypeProcessor>>,
}
impl<'a> ITypeBatchSolveFilter for IncrementalUpdateForSubstepFilter<'a> {
    #[inline(always)]
    fn include_fallback_batch_for_work_blocks(&self) -> bool { true }
    #[inline(always)]
    fn allow_type(&self, type_id: i32) -> bool {
        if let Some(ref tp) = self.type_processors[type_id as usize] {
            tp.inner().requires_incremental_substep_updates()
        } else {
            false
        }
    }
}

impl Solver {
    // --- Solve methods (translated from Solver_Solve.cs) ---

    fn on_substep_started(&self, substep_index: i32) {
        if let Some(ref callback) = self.substep_started {
            callback(substep_index);
        }
    }

    fn on_substep_ended(&self, substep_index: i32) {
        if let Some(ref callback) = self.substep_ended {
            callback(substep_index);
        }
    }

    /// Dispatches a warm start for a specific batch/type-batch range, choosing the correct
    /// integration mode (Always for batch 0, Conditional/Never for other batches based on
    /// coarse integration responsibilities).
    #[inline(always)]
    unsafe fn warm_start_block(
        &self,
        worker_index: i32,
        batch_index: i32,
        type_batch_index: i32,
        start_bundle: i32,
        end_bundle: i32,
        type_batch: &mut crate::physics::constraints::type_batch::TypeBatch,
        type_processor: &dyn crate::physics::constraints::type_processor::ITypeProcessor,
        allow_pose_integration: bool,
        dt: f32,
        inverse_dt: f32,
    ) {
        use crate::physics::constraints::batch_integration_mode::BatchIntegrationMode;
        let bodies = &*self.bodies;
        if batch_index == 0 {
            let no_flags: Buffer<IndexSet> = Buffer::default();
            type_processor.warm_start(
                type_batch, bodies, &no_flags,
                self.pose_integrator.map(|p| &*p),
                BatchIntegrationMode::Always,
                allow_pose_integration,
                dt, inverse_dt,
                start_bundle, end_bundle,
                worker_index,
            );
        } else {
            if *self.coarse_batch_integration_responsibilities.get(batch_index).get(type_batch_index) {
                let flags = self.integration_flags.get(batch_index).get(type_batch_index);
                type_processor.warm_start(
                    type_batch, bodies, flags,
                    self.pose_integrator.map(|p| &*p),
                    BatchIntegrationMode::Conditional,
                    allow_pose_integration,
                    dt, inverse_dt,
                    start_bundle, end_bundle,
                    worker_index,
                );
            } else {
                let flags = self.integration_flags.get(batch_index).get(type_batch_index);
                type_processor.warm_start(
                    type_batch, bodies, flags,
                    self.pose_integrator.map(|p| &*p),
                    BatchIntegrationMode::Never,
                    allow_pose_integration,
                    dt, inverse_dt,
                    start_bundle, end_bundle,
                    worker_index,
                );
            }
        }
    }

    fn get_velocity_iteration_count_for_substep_index(&self, substep_index: i32) -> i32 {
        if let Some(ref scheduler) = self.velocity_iteration_scheduler {
            let scheduled_count = scheduler(substep_index);
            if scheduled_count < 1 {
                self.velocity_iteration_count
            } else {
                scheduled_count
            }
        } else {
            self.velocity_iteration_count
        }
    }

    // --- MT sync index helpers ---

    #[inline(always)]
    fn get_previous_sync_index_for_incremental_update(
        &self,
        substep_index: i32,
        sync_index: i32,
        sync_stages_per_substep: i32,
    ) -> i32 {
        if substep_index == 1 {
            0
        } else {
            0i32.max(sync_index - sync_stages_per_substep)
        }
    }

    #[inline(always)]
    fn get_previous_sync_index_for_integrate_constrained_kinematics(
        &self,
        substep_index: i32,
        sync_index: i32,
        sync_stages_per_substep: i32,
    ) -> i32 {
        if substep_index == 1 {
            if self.pose_integrator.map_or(false, |pi| unsafe { (*pi).integrate_velocity_for_kinematics() }) {
                2
            } else {
                0
            }
        } else {
            0i32.max(sync_index - sync_stages_per_substep)
        }
    }

    #[inline(always)]
    fn get_warm_start_lookback(&self, substep_index: i32, synchronized_batch_count: i32) -> i32 {
        // Warm start and solve share the same claims buffer, so look back past the last solve execution.
        // "+2" accounts for the incremental update and kinematic integration stages.
        let mut lookback = synchronized_batch_count + 2;
        if substep_index > 0 {
            let ctx = &self.substep_context;
            lookback += synchronized_batch_count
                * (ctx.highest_velocity_iteration_count
                    - unsafe { *ctx.velocity_iteration_counts.get((substep_index - 1)) });
        }
        lookback
    }

    #[inline(always)]
    fn get_previous_sync_index_for_warm_start(sync_index: i32, warm_start_lookback: i32) -> i32 {
        0i32.max(sync_index - warm_start_lookback)
    }

    #[inline(always)]
    fn get_previous_sync_index_for_solve(sync_index: i32, synchronized_batch_count: i32) -> i32 {
        0i32.max(sync_index - synchronized_batch_count)
    }

    fn get_uniformly_distributed_start(
        worker_index: i32,
        block_count: i32,
        worker_count: i32,
        offset: i32,
    ) -> i32 {
        if block_count <= worker_count {
            if worker_index < block_count {
                offset + worker_index
            } else {
                -1
            }
        } else {
            let blocks_per_worker = block_count / worker_count;
            let remainder = block_count - blocks_per_worker * worker_count;
            offset + blocks_per_worker * worker_index + remainder.min(worker_index)
        }
    }

    /// Lock-free work-stealing execution for worker threads.
    /// Workers try to claim adjacent blocks via compare-exchange on the claims buffer.
    unsafe fn execute_worker_stage<TStage: ISolverStageFunction>(
        &self,
        stage_function: &TStage,
        worker_index: i32,
        worker_start: i32,
        available_blocks_start_index: i32,
        claims: &Buffer<i32>,
        previous_sync_index: i32,
        sync_index: i32,
        completed_work_blocks: *mut i32,
    ) {
        use std::sync::atomic::{AtomicI32, Ordering};

        if worker_start == -1 {
            return;
        }
        let self_ptr = self as *const Self as *mut Self;
        let mut work_block_index = worker_start;
        let mut locally_completed_count = 0i32;

        // Try to claim blocks traversing forward.
        loop {
            let claim_ptr = claims.get_ptr(work_block_index) as *mut i32;
            let atomic = AtomicI32::from_ptr(claim_ptr);
            if atomic.compare_exchange(
                previous_sync_index,
                sync_index,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ).is_ok()
            {
                stage_function.execute(self_ptr, available_blocks_start_index + work_block_index, worker_index);
                locally_completed_count += 1;
                work_block_index += 1;
                if work_block_index >= claims.len() as i32 {
                    work_block_index = 0;
                }
            } else {
                break;
            }
        }

        // Try to claim work blocks going backward.
        work_block_index = worker_start - 1;
        loop {
            if work_block_index < 0 {
                work_block_index = claims.len() as i32 - 1;
            }
            let claim_ptr = claims.get_ptr(work_block_index) as *mut i32;
            let atomic = AtomicI32::from_ptr(claim_ptr);
            if atomic.compare_exchange(
                previous_sync_index,
                sync_index,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ).is_ok()
            {
                stage_function.execute(self_ptr, available_blocks_start_index + work_block_index, worker_index);
                locally_completed_count += 1;
                work_block_index -= 1;
            } else {
                break;
            }
        }

        // Report completed count.
        let atomic_completed = AtomicI32::from_ptr(completed_work_blocks);
        atomic_completed.fetch_add(locally_completed_count, Ordering::AcqRel);
    }

    /// Main thread stage execution with spin-wait synchronization.
    /// The main thread never yields — it only spins waiting for workers to complete.
    unsafe fn execute_main_stage<TStage: ISolverStageFunction>(
        &self,
        stage_function: &TStage,
        worker_index: i32,
        worker_start: i32,
        stage: &SolverSyncStage,
        previous_sync_index: i32,
        sync_index: i32,
    ) {
        use std::sync::atomic::{AtomicI32, Ordering};

        let available_blocks_count = stage.claims.len() as i32;
        if available_blocks_count == 0 {
            return;
        }

        let self_ptr = self as *const Self as *mut Self;
        let ctx = &(*self_ptr).substep_context;



        if available_blocks_count == 1 {
            // Single work block — just execute directly, no need to notify workers.
            stage_function.execute(self_ptr, stage.work_block_start_index, worker_index);
        } else {
            // Write the new stage index so other spinning threads will begin work.
            let sync_ptr = ctx.sync_index.get();
            AtomicI32::from_ptr(sync_ptr).store(sync_index, Ordering::Release);

            // Execute our own portion.
            let completed_ptr = ctx.completed_work_block_count.get();
            self.execute_worker_stage(
                stage_function,
                worker_index,
                worker_start,
                stage.work_block_start_index,
                &stage.claims,
                previous_sync_index,
                sync_index,
                completed_ptr,
            );

            // Spin-wait until all workers have completed.
            // DO NOT yield on the main thread — this significantly increases the chance
            // *some* progress will be made, even if workers are stuck unscheduled.
            let atomic_completed = AtomicI32::from_ptr(completed_ptr);
            while atomic_completed.load(Ordering::Acquire) != available_blocks_count {
                std::hint::spin_loop();
                std::hint::spin_loop();
                std::hint::spin_loop();
            }
            // All workers done. Reset counter for next stage.
            *completed_ptr = 0;
        }
    }

    /// The SolveWorker dispatch function for multithreaded solving.
    /// Worker 0 is the main orchestrator; other workers spin-wait and execute dispatched stages.
    unsafe fn solve_worker(&mut self, worker_index: i32) {
        use std::sync::atomic::{AtomicI32, Ordering};

        let self_ptr = self as *mut Self;
        let ctx = &(*self_ptr).substep_context;
        let worker_count = ctx.worker_count;

        let incremental_update_worker_start = Self::get_uniformly_distributed_start(
            worker_index,
            ctx.incremental_update_blocks.len() as i32,
            worker_count,
            0,
        );
        let kinematic_integration_worker_start = Self::get_uniformly_distributed_start(
            worker_index,
            ctx.kinematic_integration_blocks.len() as i32,
            worker_count,
            0,
        );

        let active_set = (*self_ptr).sets.get(0);
        let batch_count = active_set.batches.count;

        // Compute per-batch starting positions for constraint work blocks.
        let mut batch_starts_storage = vec![0i32; batch_count as usize];
        let (synchronized_batch_count, _fallback_exists) = (*self_ptr).get_synchronized_batch_count();
        for batch_index in 0..synchronized_batch_count {
            let batch_offset = if batch_index > 0 {
                *ctx.constraint_batch_boundaries.get((batch_index - 1))
            } else {
                0
            };
            let batch_count_blocks = *ctx.constraint_batch_boundaries.get(batch_index) - batch_offset;
            batch_starts_storage[batch_index as usize] =
                Self::get_uniformly_distributed_start(worker_index, batch_count_blocks, worker_count, 0);
        }

        debug_assert!(batch_count > 0, "Don't dispatch if there are no constraints.");

        let incremental_update_stage = IncrementalUpdateStageFunction {
            dt: ctx.dt,
            inverse_dt: ctx.inverse_dt,
            solver: self_ptr,
        };
        let mut integrate_constrained_kinematics_stage = IntegrateConstrainedKinematicsStageFunction {
            dt: ctx.dt,
            inverse_dt: ctx.inverse_dt,
            substep_index: 0,
            solver: self_ptr,
        };
        let mut warmstart_stage = WarmStartStageFunction {
            dt: ctx.dt,
            inverse_dt: ctx.inverse_dt,
            substep_index: 0,
            solver: self_ptr,
        };
        let solve_stage = SolveStageFunction {
            dt: ctx.dt,
            inverse_dt: ctx.inverse_dt,
            solver: self_ptr,
        };

        let maximum_sync_stages_per_substep = 2 + synchronized_batch_count * (1 + ctx.highest_velocity_iteration_count);

        if worker_index == 0 {
            // ---- Main orchestrator thread ----
            for substep_index in 0..(*self_ptr).substep_count {
                (*self_ptr).on_substep_started(substep_index);

                let mut sync_index = substep_index * maximum_sync_stages_per_substep + 1;

                // Incremental update (substeps > 0).
                if substep_index > 0 {
                    (*self_ptr).execute_main_stage(
                        &incremental_update_stage,
                        worker_index,
                        incremental_update_worker_start,
                        &(*self_ptr).substep_context.stages.get(0),
                        (*self_ptr).get_previous_sync_index_for_incremental_update(substep_index, sync_index, maximum_sync_stages_per_substep),
                        sync_index,
                    );
                }

                sync_index += 1;

                // Integrate constrained kinematics.
                let integrate_velocity_for_kinematics = (*self_ptr).pose_integrator.map_or(false, |pi| (*pi).integrate_velocity_for_kinematics());
                if substep_index > 0 || integrate_velocity_for_kinematics {
                    integrate_constrained_kinematics_stage.substep_index = substep_index;
                    (*self_ptr).execute_main_stage(
                        &integrate_constrained_kinematics_stage,
                        worker_index,
                        kinematic_integration_worker_start,
                        &(*self_ptr).substep_context.stages.get(1),
                        (*self_ptr).get_previous_sync_index_for_integrate_constrained_kinematics(substep_index, sync_index, maximum_sync_stages_per_substep),
                        sync_index,
                    );
                }

                // Warm start.
                warmstart_stage.substep_index = substep_index;
                let warm_start_lookback = (*self_ptr).get_warm_start_lookback(substep_index, synchronized_batch_count);
                for batch_index in 0..synchronized_batch_count {
                    sync_index += 1;
                    (*self_ptr).execute_main_stage(
                        &warmstart_stage,
                        worker_index,
                        batch_starts_storage[batch_index as usize],
                        &(*self_ptr).substep_context.stages.get((batch_index + 2)),
                        Self::get_previous_sync_index_for_warm_start(sync_index, warm_start_lookback),
                        sync_index,
                    );
                }

                // Fallback batch warm start (single-threaded).
                let (_sync_count, fallback_exists) = (*self_ptr).get_synchronized_batch_count();
                if fallback_exists {
                    let fallback_batch_threshold = (*self_ptr).fallback_batch_threshold;
                    let active_set = (*self_ptr).sets.get_mut(0);
                    let batch = active_set.batches.get_mut(fallback_batch_threshold);
                    let allow_pose_integration = substep_index > 0;
                    for j in 0..batch.type_batches.count {
                        let type_batch = batch.type_batches.get_mut(j);
                        let type_id = type_batch.type_id;
                        let bundle_count = type_batch.bundle_count() as i32;
                        let processor = (&(*self_ptr).type_processors)[type_id as usize]
                            .as_ref().unwrap();
                        (*self_ptr).warm_start_block(
                            0, fallback_batch_threshold, j,
                            0, bundle_count,
                            type_batch, processor.inner(),
                            allow_pose_integration,
                            ctx.dt, ctx.inverse_dt,
                        );
                    }
                }

                // Velocity iterations.
                let velocity_iteration_count = *ctx.velocity_iteration_counts.get(substep_index);
                for _iteration_index in 0..velocity_iteration_count {
                    for batch_index in 0..synchronized_batch_count {
                        sync_index += 1;
                        (*self_ptr).execute_main_stage(
                            &solve_stage,
                            worker_index,
                            batch_starts_storage[batch_index as usize],
                            &(*self_ptr).substep_context.stages.get((batch_index + 2)),
                            Self::get_previous_sync_index_for_solve(sync_index, synchronized_batch_count),
                            sync_index,
                        );
                    }
                    // Fallback solve (single-threaded).
                    if fallback_exists {
                        let fallback_batch_threshold = (*self_ptr).fallback_batch_threshold;
                        let active_set = (*self_ptr).sets.get_mut(0);
                        let batch = active_set.batches.get_mut(fallback_batch_threshold);
                        let bodies = &*(*self_ptr).bodies;
                        for j in 0..batch.type_batches.count {
                            let type_batch = batch.type_batches.get_mut(j);
                            let type_id = type_batch.type_id;
                            let bundle_count = type_batch.bundle_count() as i32;
                            (&(*self_ptr).type_processors)[type_id as usize]
                                .as_ref().unwrap().inner()
                                .solve(type_batch, bodies, ctx.dt, ctx.inverse_dt, 0, bundle_count);
                        }
                    }
                }

                (*self_ptr).on_substep_ended(substep_index);
            }

            // All done; notify waiting threads to join.
            let sync_ptr = ctx.sync_index.get();
            AtomicI32::from_ptr(sync_ptr).store(i32::MIN, Ordering::Release);
        } else {
            // ---- Worker thread ----
            let mut latest_completed_sync_index = 0i32;
            let mut sync_index_in_substep = -1i32;
            let mut substep_index = 0i32;

            loop {
                let mut spin_wait = crate::physics::local_spin_wait::LocalSpinWait::new();
                let sync_index;
                loop {
                    let sync_ptr = ctx.sync_index.get();
                    let current = AtomicI32::from_ptr(sync_ptr).load(Ordering::Acquire);
                    if latest_completed_sync_index != current {
                        sync_index = current;
                        break;
                    }
                    spin_wait.spin_once();
                }

                if sync_index == i32::MIN {
                    break;
                }

                let sync_steps_since_last = sync_index - latest_completed_sync_index;
                sync_index_in_substep += sync_steps_since_last;
                while sync_index_in_substep >= maximum_sync_stages_per_substep {
                    sync_index_in_substep -= maximum_sync_stages_per_substep;
                    substep_index += 1;
                }

                let stage = (*self_ptr).substep_context.stages.get(sync_index_in_substep);
                let completed_ptr = ctx.completed_work_block_count.get();

                match stage.stage_type {
                    SolverStageType::IncrementalUpdate => {
                        (*self_ptr).execute_worker_stage(
                            &incremental_update_stage,
                            worker_index,
                            incremental_update_worker_start,
                            0,
                            &stage.claims,
                            (*self_ptr).get_previous_sync_index_for_incremental_update(substep_index, sync_index, maximum_sync_stages_per_substep),
                            sync_index,
                            completed_ptr,
                        );
                    }
                    SolverStageType::IntegrateConstrainedKinematics => {
                        integrate_constrained_kinematics_stage.substep_index = substep_index;
                        (*self_ptr).execute_worker_stage(
                            &integrate_constrained_kinematics_stage,
                            worker_index,
                            kinematic_integration_worker_start,
                            0,
                            &stage.claims,
                            (*self_ptr).get_previous_sync_index_for_integrate_constrained_kinematics(substep_index, sync_index, maximum_sync_stages_per_substep),
                            sync_index,
                            completed_ptr,
                        );
                    }
                    SolverStageType::WarmStart => {
                        warmstart_stage.substep_index = substep_index;
                        let warm_start_lookback = (*self_ptr).get_warm_start_lookback(substep_index, synchronized_batch_count);
                        (*self_ptr).execute_worker_stage(
                            &warmstart_stage,
                            worker_index,
                            batch_starts_storage[stage.batch_index as usize],
                            stage.work_block_start_index,
                            &stage.claims,
                            Self::get_previous_sync_index_for_warm_start(sync_index, warm_start_lookback),
                            sync_index,
                            completed_ptr,
                        );
                    }
                    SolverStageType::Solve => {
                        (*self_ptr).execute_worker_stage(
                            &solve_stage,
                            worker_index,
                            batch_starts_storage[stage.batch_index as usize],
                            stage.work_block_start_index,
                            &stage.claims,
                            Self::get_previous_sync_index_for_solve(sync_index, synchronized_batch_count),
                            sync_index,
                            completed_ptr,
                        );
                    }
                }
                latest_completed_sync_index = sync_index;
            }
        }
    }

    /// Builds kinematic integration work blocks for multithreaded dispatch.
    unsafe fn build_kinematic_integration_work_blocks(
        &self,
        minimum_block_size_in_bundles: i32,
        maximum_block_size_in_bundles: i32,
        target_block_count: i32,
    ) -> Buffer<IntegrationWorkBlock> {
        let bundle_count = BundleIndexing::get_bundle_count(self.constrained_kinematic_handles.count as usize) as i32;
        if bundle_count > 0 {
            let mut target_bundles_per_block = bundle_count / target_block_count;
            if target_bundles_per_block < minimum_block_size_in_bundles {
                target_bundles_per_block = minimum_block_size_in_bundles;
            }
            if target_bundles_per_block > maximum_block_size_in_bundles {
                target_bundles_per_block = maximum_block_size_in_bundles;
            }
            let block_count = (bundle_count + target_bundles_per_block - 1) / target_bundles_per_block;
            let bundles_per_block = bundle_count / block_count;
            let remainder = bundle_count - bundles_per_block * block_count;
            let mut previous_end = 0;
            let pool = &mut *self.pool;
            let mut work_blocks: Buffer<IntegrationWorkBlock> = pool.take(block_count);
            for i in 0..block_count {
                let mut bundle_count_for_block = bundles_per_block;
                if i < remainder {
                    bundle_count_for_block += 1;
                }
                *work_blocks.get_mut(i) = IntegrationWorkBlock {
                    start_bundle_index: previous_end,
                    end_bundle_index: {
                        previous_end += bundle_count_for_block;
                        previous_end
                    },
                };
            }
            work_blocks
        } else {
            Buffer::default()
        }
    }

    /// Builds work blocks for constraint batches, distributing work evenly across target block count.
    unsafe fn build_work_blocks<TFilter: ITypeBatchSolveFilter>(
        &self,
        pool: &mut BufferPool,
        minimum_block_size_in_bundles: i32,
        maximum_block_size_in_bundles: i32,
        target_blocks_per_batch: i32,
        type_batch_filter: &TFilter,
    ) -> (QuickList<WorkBlock>, Buffer<i32>) {
        let active_set = self.sets.get(0);
        let batch_count;
        if type_batch_filter.include_fallback_batch_for_work_blocks() {
            batch_count = active_set.batches.count;
        } else {
            let (sync_count, _) = self.get_synchronized_batch_count();
            batch_count = sync_count;
        }

        let mut work_blocks: QuickList<WorkBlock> = QuickList::with_capacity(target_blocks_per_batch * batch_count, pool);
        let mut batch_boundaries: Buffer<i32> = pool.take(batch_count);
        let inverse_minimum_block_size = 1.0f32 / minimum_block_size_in_bundles as f32;
        let inverse_maximum_block_size = 1.0f32 / maximum_block_size_in_bundles as f32;

        for batch_index in 0..batch_count {
            let type_batches = &active_set.batches.get(batch_index).type_batches;
            let mut bundle_count = 0i32;
            for type_batch_index in 0..type_batches.count {
                let type_batch = type_batches.get(type_batch_index);
                if type_batch_filter.allow_type(type_batch.type_id) {
                    bundle_count += type_batch.bundle_count() as i32;
                }
            }
            for type_batch_index in 0..type_batches.count {
                let type_batch = type_batches.get(type_batch_index);
                if !type_batch_filter.allow_type(type_batch.type_id) {
                    continue;
                }
                let tb_bundle_count = type_batch.bundle_count() as i32;
                let type_batch_size_fraction = tb_bundle_count as f32 / bundle_count as f32;
                let type_batch_maximum_block_count = tb_bundle_count as f32 * inverse_minimum_block_size;
                let type_batch_minimum_block_count = tb_bundle_count as f32 * inverse_maximum_block_size;
                let type_batch_block_count = 1i32.max(
                    type_batch_maximum_block_count.min(
                        type_batch_minimum_block_count.max(target_blocks_per_batch as f32 * type_batch_size_fraction)
                    ) as i32
                );
                let mut previous_end = 0;
                let base_block_size = tb_bundle_count / type_batch_block_count;
                let remainder = tb_bundle_count - base_block_size * type_batch_block_count;
                for new_block_index in 0..type_batch_block_count {
                    let block_bundle_count = if new_block_index < remainder {
                        base_block_size + 1
                    } else {
                        base_block_size
                    };
                    let block = work_blocks.allocate_unsafely();
                    block.batch_index = batch_index;
                    block.type_batch_index = type_batch_index;
                    block.start_bundle = previous_end;
                    block.end = previous_end + block_bundle_count;
                    previous_end = block.end;
                }
            }
            *batch_boundaries.get_mut(batch_index) = work_blocks.count;
        }

        (work_blocks, batch_boundaries)
    }

    /// Executes the multithreaded solve path.
    unsafe fn execute_multithreaded(
        &mut self,
        dt: f32,
        thread_dispatcher: &dyn crate::utilities::thread_dispatcher::IThreadDispatcher,
    ) {
        let self_ptr = self as *mut Self;
        let ctx = &mut (*self_ptr).substep_context;
        let worker_count = thread_dispatcher.thread_count();
        ctx.worker_count = worker_count;
        ctx.dt = dt;
        ctx.inverse_dt = 1.0 / dt;

        let pool = &mut *self.pool;
        ctx.velocity_iteration_counts = pool.take(self.substep_count);

        // Each substep can have a different number of velocity iterations.
        if self.velocity_iteration_scheduler.is_none() {
            for i in 0..self.substep_count {
                *ctx.velocity_iteration_counts.get_mut(i) = self.velocity_iteration_count;
            }
        } else {
            for i in 0..self.substep_count {
                *ctx.velocity_iteration_counts.get_mut(i) =
                    self.get_velocity_iteration_count_for_substep_index(i);
            }
        }

        const TARGET_BLOCKS_PER_BATCH_PER_WORKER: i32 = 4;
        const MINIMUM_BLOCK_SIZE_IN_BUNDLES: i32 = 1;
        const MAXIMUM_BLOCK_SIZE_IN_BUNDLES: i32 = 1024;

        let target_blocks_per_batch = worker_count * TARGET_BLOCKS_PER_BATCH_PER_WORKER;

        let main_filter = MainSolveFilter;
        let incremental_filter = IncrementalUpdateForSubstepFilter {
            type_processors: &self.type_processors,
        };

        let (constraint_blocks, constraint_batch_boundaries) = (*self_ptr).build_work_blocks(
            &mut *(*self_ptr).pool,
            MINIMUM_BLOCK_SIZE_IN_BUNDLES,
            MAXIMUM_BLOCK_SIZE_IN_BUNDLES,
            target_blocks_per_batch,
            &main_filter,
        );
        let (incremental_blocks, mut incremental_update_batch_boundaries) = (*self_ptr).build_work_blocks(
            &mut *(*self_ptr).pool,
            MINIMUM_BLOCK_SIZE_IN_BUNDLES,
            MAXIMUM_BLOCK_SIZE_IN_BUNDLES,
            target_blocks_per_batch,
            &incremental_filter,
        );
        (&mut *(*self_ptr).pool).return_buffer(&mut incremental_update_batch_boundaries);

        let ctx = &mut (*self_ptr).substep_context;
        ctx.constraint_blocks = constraint_blocks.span.slice_count(constraint_blocks.count);
        ctx.incremental_update_blocks = incremental_blocks.span.slice_count(incremental_blocks.count);
        ctx.constraint_batch_boundaries = constraint_batch_boundaries;
        ctx.kinematic_integration_blocks = self.build_kinematic_integration_work_blocks(
            MINIMUM_BLOCK_SIZE_IN_BUNDLES,
            MAXIMUM_BLOCK_SIZE_IN_BUNDLES,
            target_blocks_per_batch,
        );

        let ctx = &mut (*self_ptr).substep_context;
        *ctx.sync_index.get_mut() = 0;

        let total_constraint_batch_work_block_count = if ctx.constraint_batch_boundaries.len() == 0 {
            0
        } else {
            *ctx.constraint_batch_boundaries.get(ctx.constraint_batch_boundaries.len() - 1)
        };
        let total_claim_count = incremental_blocks.count
            + ctx.kinematic_integration_blocks.len()
            + total_constraint_batch_work_block_count;

        let (stages_per_iteration, _fallback_exists) = self.get_synchronized_batch_count();

        ctx.highest_velocity_iteration_count = 0;
        for i in 0..ctx.velocity_iteration_counts.len() {
            ctx.highest_velocity_iteration_count = ctx.highest_velocity_iteration_count.max(
                *ctx.velocity_iteration_counts.get(i),
            );
        }

        let pool = &mut *self.pool;
        ctx.stages = pool.take(
            2 + stages_per_iteration * (1 + ctx.highest_velocity_iteration_count),
        );

        // Allocate claims buffer, initialized to 0 to match initial sync index.
        let mut claims: Buffer<i32> = pool.take(total_claim_count);
        claims.clear(0, claims.len());

        // Set up stages.
        let inc_block_count = incremental_blocks.count;
        let kin_block_count = ctx.kinematic_integration_blocks.len();

        *ctx.stages.get_mut(0) = SolverSyncStage::new_no_batch(
            claims.slice_count(inc_block_count),
            0,
            SolverStageType::IncrementalUpdate,
        );
        *ctx.stages.get_mut(1) = SolverSyncStage::new_no_batch(
            claims.slice_offset(inc_block_count, kin_block_count),
            0,
            SolverStageType::IntegrateConstrainedKinematics,
        );

        let mut target_stage_index = 2;
        let preamble_claim_count = inc_block_count + kin_block_count;
        let mut claim_start = preamble_claim_count;
        let mut highest_job_count_in_solve = 0;

        // Warm start stages.
        for batch_index in 0..stages_per_iteration {
            let stage_index = target_stage_index;
            target_stage_index += 1;
            let batch_start = if batch_index == 0 {
                0
            } else {
                *ctx.constraint_batch_boundaries.get((batch_index - 1))
            };
            let work_blocks_in_batch = *ctx.constraint_batch_boundaries.get(batch_index) - batch_start;
            *ctx.stages.get_mut(stage_index) = SolverSyncStage::new(
                claims.slice_offset(claim_start, work_blocks_in_batch),
                batch_start,
                SolverStageType::WarmStart,
                batch_index,
            );
            claim_start += work_blocks_in_batch;
            highest_job_count_in_solve = highest_job_count_in_solve.max(work_blocks_in_batch);
        }

        // Solve stages (reusing same claims as warm start).
        for _iteration_index in 0..ctx.highest_velocity_iteration_count {
            claim_start = preamble_claim_count;
            for batch_index in 0..stages_per_iteration {
                let stage_index = target_stage_index;
                target_stage_index += 1;
                let batch_start = if batch_index == 0 {
                    0
                } else {
                    *ctx.constraint_batch_boundaries.get((batch_index - 1))
                };
                let work_blocks_in_batch = *ctx.constraint_batch_boundaries.get(batch_index) - batch_start;
                *ctx.stages.get_mut(stage_index) = SolverSyncStage::new(
                    claims.slice_offset(claim_start, work_blocks_in_batch),
                    batch_start,
                    SolverStageType::Solve,
                    batch_index,
                );
                claim_start += work_blocks_in_batch;
                highest_job_count_in_solve = highest_job_count_in_solve.max(work_blocks_in_batch);
            }
        }

        // Dispatch workers if there are any constraints.
        let active_batch_count = self.sets.get(0).batches.count;
        if active_batch_count > 0 {
            // Pass the solver pointer through unmanaged_context so the worker fn can access it.
            let self_ptr = self as *mut Self;
            fn solve_worker_fn(worker_index: i32, _dispatcher: &dyn crate::utilities::thread_dispatcher::IThreadDispatcher) {
                // The solver pointer is stored in the dispatcher's unmanaged context.
                let ctx = _dispatcher.unmanaged_context();
                let solver = ctx as *mut Solver;
                unsafe { (*solver).solve_worker(worker_index); }
            }
            thread_dispatcher.dispatch_workers(
                solve_worker_fn,
                highest_job_count_in_solve,
                self_ptr as *mut (),
                None,
            );
        }

        // Cleanup.
        let pool = &mut *self.pool;
        pool.return_buffer(&mut claims);
        pool.return_buffer(&mut self.substep_context.stages);
        pool.return_buffer(&mut self.substep_context.constraint_batch_boundaries);
        pool.return_buffer(&mut self.substep_context.incremental_update_blocks);
        if self.substep_context.kinematic_integration_blocks.allocated() {
            pool.return_buffer(&mut self.substep_context.kinematic_integration_blocks);
        }
        pool.return_buffer(&mut self.substep_context.constraint_blocks);
        pool.return_buffer(&mut self.substep_context.velocity_iteration_counts);
    }

    /// Computes integration responsibilities for a region of constraints within a type batch.
    /// Returns true if any constraint in the region has integration responsibility.
    ///
    /// For non-fallback batches: a body whose handle is first observed in this batch gets
    /// integration responsibility assigned to the constraint that references it.
    /// For fallback batches: additionally checks that this is the earliest constraint slot
    /// referencing that body (by encoded (typeBatchIndex, indexInTypeBatch) ordering).
    unsafe fn compute_integration_responsibilities_for_constraint_region(
        &self,
        batch_index: i32,
        type_batch_index: i32,
        constraint_start: i32,
        exclusive_constraint_end: i32,
        is_fallback_batch: bool,
    ) -> bool {
        let first_observed_for_batch = &*self.bodies_first_observed_in_batches.get(batch_index);
        // Use raw pointer to get mutable access to integration flags (safe: we guarantee exclusive access by design)
        let integration_flags_ptr = self.integration_flags.as_ptr()
            .add(batch_index as usize) as *mut Buffer<Buffer<IndexSet>>;
        let integration_flags_for_type_batch = (*integration_flags_ptr)
            .as_mut_ptr().add(type_batch_index as usize);
        let active_set = self.sets.get(0);
        let type_batch = active_set.batches.get(batch_index).type_batches.get(type_batch_index);
        let type_batch_body_references = type_batch.body_references.as_ptr() as *const i32;
        let bodies_per_constraint_in_type_batch = self.type_processors[type_batch.type_id as usize]
            .as_ref().unwrap().bodies_per_constraint;
        let vector_width = crate::utilities::vector::VECTOR_WIDTH as i32;
        let ints_per_bundle = vector_width * bodies_per_constraint_in_type_batch;
        let bundle_start_index = constraint_start / vector_width;
        let bundle_end_index = (exclusive_constraint_end + vector_width - 1) / vector_width;
        let bodies = &*self.bodies;
        let body_active_set = bodies.active_set();

        for bundle_index in bundle_start_index..bundle_end_index {
            let bundle_start_index_in_constraints = bundle_index * vector_width;
            let count_in_bundle = (type_batch.constraint_count - bundle_start_index_in_constraints).min(vector_width);
            let bundle_body_references_start = type_batch_body_references.add((bundle_index * ints_per_bundle) as usize);

            for body_index_in_constraint in 0..bodies_per_constraint_in_type_batch {
                let integration_flags_for_body = (*integration_flags_for_type_batch).get_mut(body_index_in_constraint);
                let bundle_start = bundle_body_references_start.add((body_index_in_constraint * vector_width) as usize);

                for bundle_inner_index in 0..count_in_bundle {
                    let raw_body_index = *bundle_start.add(bundle_inner_index as usize);

                    if is_fallback_batch {
                        // Fallback batches can contain empty lanes marked with -1.
                        if raw_body_index == -1 {
                            continue;
                        }
                    }

                    let body_index = raw_body_index & Bodies::BODY_REFERENCE_MASK;
                    let body_handle = body_active_set.index_to_handle.get(body_index).0;

                    if first_observed_for_batch.contains(body_handle) {
                        if is_fallback_batch {
                            // For fallback batch: must also be the *earliest* constraint slot for this body.
                            let mut earliest_index: u64 = u64::MAX;
                            let constraints_for_body = body_active_set.constraints.get(body_index);
                            let fallback_batch = active_set.batches.get(self.fallback_batch_threshold);
                            for ci in 0..constraints_for_body.count {
                                let constraint_ref = constraints_for_body.span.get(ci);
                                let location = self.handle_to_constraint.get(constraint_ref.connecting_constraint_handle.0);
                                let tbi = *fallback_batch.type_index_to_type_batch_index.get(location.type_id);
                                let candidate = ((tbi as u64) << 32) | (location.index_in_type_batch as u32 as u64);
                                if candidate < earliest_index {
                                    earliest_index = candidate;
                                }
                            }
                            let index_in_type_batch = bundle_start_index_in_constraints + bundle_inner_index;
                            let current_slot = ((type_batch_index as u64) << 32) | (index_in_type_batch as u32 as u64);
                            if current_slot == earliest_index {
                                integration_flags_for_body.add_unsafely(index_in_type_batch);
                            }
                        } else {
                            // Not a fallback: being contained in the observed set is sufficient.
                            integration_flags_for_body.add_unsafely(bundle_start_index_in_constraints + bundle_inner_index);
                        }
                    }
                }
            }
        }

        // Precompute whether this type batch has *any* integration responsibilities.
        let flag_bundle_count = IndexSet::get_bundle_capacity(type_batch.constraint_count);
        let mut merged_flag_bundles: u64 = 0;
        for body_index_in_constraint in 0..bodies_per_constraint_in_type_batch {
            let flags_for_body = (*integration_flags_for_type_batch).get(body_index_in_constraint);
            for i in 0..flag_bundle_count {
                merged_flag_bundles |= *flags_for_body.flags.get(i);
            }
        }
        merged_flag_bundles != 0
    }

    /// Prepares constraint integration responsibilities. Returns an IndexSet of constrained body handles.
    ///
    /// This computes per-batch merged body handle sets and identifies which constraint is
    /// first to see each body, which determines integration responsibility during substepping.
    pub unsafe fn prepare_constraint_integration_responsibilities(
        &mut self,
        _thread_dispatcher: Option<&dyn crate::utilities::thread_dispatcher::IThreadDispatcher>,
    ) -> IndexSet {
        if self.active_set().batches.count > 0 {
            let pool = &mut *self.pool;
            let bodies = &*self.bodies;
            let active_set_ptr = self.sets.get(0) as *const ConstraintSet;
            let batch_count = (*active_set_ptr).batches.count;

            // Allocate integration flags: integrationFlags[batch][typeBatch][bodyInConstraint] = IndexSet
            self.integration_flags = pool.take(batch_count);
            *self.integration_flags.get_mut(0) = Buffer::default(); // batch 0 is always-integrate, no flags needed

            // Allocate coarse batch integration responsibilities
            self.coarse_batch_integration_responsibilities = pool.take(batch_count);

            for batch_index in 1..self.integration_flags.len() {
                let batch = (*active_set_ptr).batches.get(batch_index);
                let type_batch_count = batch.type_batches.count;
                let mut flags_for_batch: Buffer<Buffer<IndexSet>> = pool.take(type_batch_count);
                let coarse_for_batch: Buffer<bool> = pool.take(type_batch_count);

                for type_batch_index in 0..type_batch_count {
                    let type_batch = batch.type_batches.get(type_batch_index);
                    let bodies_per_constraint = self.type_processors[type_batch.type_id as usize]
                        .as_ref().unwrap().bodies_per_constraint;
                    let mut flags_for_type_batch: Buffer<IndexSet> = pool.take(bodies_per_constraint);
                    for body_index_in_constraint in 0..bodies_per_constraint {
                        *flags_for_type_batch.get_mut(body_index_in_constraint) =
                            IndexSet::new(pool, type_batch.constraint_count);
                    }
                    *flags_for_batch.get_mut(type_batch_index) = flags_for_type_batch;
                }

                *self.integration_flags.get_mut(batch_index) = flags_for_batch;
                *self.coarse_batch_integration_responsibilities.get_mut(batch_index) = coarse_for_batch;
            }

            // Build bodiesFirstObservedInBatches by computing which body handles appear in each batch
            // but not in any prior batch.
            let highest_id = bodies.handle_pool.highest_possibly_claimed_id();
            // Note: "+64" instead of "+63" because highest_possibly_claimed_id is inclusive
            let merged_flags_len = (highest_id + 64) / 64;
            self.merged_constrained_body_handles.flags = pool.take(merged_flags_len);

            // Copy batch 0's handles to initialize the merged set
            if self.batch_referenced_handles.count > 0 {
                let batch0_flags = &self.batch_referenced_handles.get(0).flags;
                let copy_len = batch0_flags.len().min(self.merged_constrained_body_handles.flags.len());
                batch0_flags.copy_to(0, &mut self.merged_constrained_body_handles.flags, 0, copy_len);
                self.merged_constrained_body_handles.flags.clear(
                    copy_len,
                    self.merged_constrained_body_handles.flags.len() - copy_len,
                );
            }

            // First slot is unallocated (batch 0 always integrates, no first-observed tracking needed)
            self.bodies_first_observed_in_batches = pool.take(self.batch_referenced_handles.count);
            *self.bodies_first_observed_in_batches.get_mut(0) = IndexSet::empty();

            let mut batch_has_any_integration_responsibilities: Buffer<bool> = pool.take(self.batch_referenced_handles.count);

            for batch_index in 1..self.bodies_first_observed_in_batches.len() {
                let batch_handles = &self.batch_referenced_handles.get(batch_index);
                let bundle_count = self.merged_constrained_body_handles.flags.len().min(batch_handles.flags.len());
                // Allocate without zeroing — every bundle will be fully assigned
                let first_observed_flags: Buffer<u64> = pool.take(bundle_count);
                *self.bodies_first_observed_in_batches.get_mut(batch_index) = IndexSet { flags: first_observed_flags };
            }

            // Compute firstObserved and merge. No multithreading — typically microseconds.
            for batch_index in 1..batch_count {
                let batch_handles = &self.batch_referenced_handles.get(batch_index);
                let first_observed = self.bodies_first_observed_in_batches.get_mut(batch_index);
                let flag_bundle_count = self.merged_constrained_body_handles.flags.len().min(batch_handles.flags.len());

                let mut horizontal_merge: u64 = 0;
                for flag_bundle_index in 0..flag_bundle_count {
                    let merge_bundle = *self.merged_constrained_body_handles.flags.get(flag_bundle_index);
                    let batch_bundle = *batch_handles.flags.get(flag_bundle_index);
                    *self.merged_constrained_body_handles.flags.get_mut(flag_bundle_index) = merge_bundle | batch_bundle;
                    // If this batch contains a body, and the merged set does not, then it's first observed in this batch
                    let first_observed_bundle = !merge_bundle & batch_bundle;
                    horizontal_merge |= first_observed_bundle;
                    *first_observed.flags.get_mut(flag_bundle_index) = first_observed_bundle;
                }
                *batch_has_any_integration_responsibilities.get_mut(batch_index) = horizontal_merge != 0;
            }

            // Compute per-constraint integration responsibilities (single-threaded path)
            let (synchronized_batch_count, fallback_exists) = self.get_synchronized_batch_count();
            for i in 1..synchronized_batch_count {
                if !*batch_has_any_integration_responsibilities.get(i) {
                    // Initialize coarse to false for batches with no responsibilities
                    let batch = (*active_set_ptr).batches.get(i);
                    for j in 0..batch.type_batches.count {
                        *self.coarse_batch_integration_responsibilities.get_mut(i).get_mut(j) = false;
                    }
                    continue;
                }
                let batch = (*active_set_ptr).batches.get(i);
                for j in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(j);
                    let has_responsibilities = self.compute_integration_responsibilities_for_constraint_region(
                        i, j, 0, type_batch.constraint_count, false,
                    );
                    *self.coarse_batch_integration_responsibilities.get_mut(i).get_mut(j) = has_responsibilities;
                }
            }
            if fallback_exists && *batch_has_any_integration_responsibilities.get(self.fallback_batch_threshold) {
                let batch = (*active_set_ptr).batches.get(self.fallback_batch_threshold);
                for j in 0..batch.type_batches.count {
                    let type_batch = batch.type_batches.get(j);
                    let has_responsibilities = self.compute_integration_responsibilities_for_constraint_region(
                        self.fallback_batch_threshold, j, 0, type_batch.constraint_count, true,
                    );
                    *self.coarse_batch_integration_responsibilities.get_mut(self.fallback_batch_threshold).get_mut(j) = has_responsibilities;
                }
            } else if fallback_exists {
                // Initialize coarse to false for fallback batch with no responsibilities
                let batch = (*active_set_ptr).batches.get(self.fallback_batch_threshold);
                for j in 0..batch.type_batches.count {
                    *self.coarse_batch_integration_responsibilities.get_mut(self.fallback_batch_threshold).get_mut(j) = false;
                }
            }

            pool.return_buffer(&mut batch_has_any_integration_responsibilities);

            // Dispose bodiesFirstObservedInBatches (no longer needed after responsibilities computed)
            debug_assert!(
                !self.bodies_first_observed_in_batches.get(0).flags.allocated(),
                "First batch's slot should be empty"
            );
            for batch_index in 1..self.bodies_first_observed_in_batches.len() {
                self.bodies_first_observed_in_batches.get_mut(batch_index).dispose(pool);
            }
            pool.return_buffer(&mut self.bodies_first_observed_in_batches);

            // Add the constrained kinematics to the merged constrained body handles.
            for i in 0..self.constrained_kinematic_handles.count {
                self.merged_constrained_body_handles.add_unsafely(
                    *self.constrained_kinematic_handles.span.get(i),
                );
            }

            // Return a copy of the merged handles (caller owns it)
            let result_len = self.merged_constrained_body_handles.flags.len();
            let mut result = IndexSet { flags: pool.take(result_len) };
            self.merged_constrained_body_handles.flags.copy_to(0, &mut result.flags, 0, result_len);
            result
        } else {
            IndexSet::empty()
        }
    }

    /// Disposes the constraint integration responsibilities created by prepare_constraint_integration_responsibilities.
    pub unsafe fn dispose_constraint_integration_responsibilities(&mut self) {
        if self.active_set().batches.count > 0 && self.integration_flags.allocated() {
            let pool = &mut *self.pool;

            debug_assert!(
                !self.integration_flags.get(0).allocated(),
                "Batch 0's integration flags should be empty"
            );
            for batch_index in 1..self.integration_flags.len() {
                let flags_for_batch = self.integration_flags.get_mut(batch_index);
                for type_batch_index in 0..flags_for_batch.len() {
                    let flags_for_type_batch = flags_for_batch.get_mut(type_batch_index);
                    for body_index_in_constraint in 0..flags_for_type_batch.len() {
                        flags_for_type_batch.get_mut(body_index_in_constraint).dispose(pool);
                    }
                    pool.return_buffer(flags_for_type_batch);
                }
                pool.return_buffer(flags_for_batch);
                pool.return_buffer(self.coarse_batch_integration_responsibilities.get_mut(batch_index));
            }
            pool.return_buffer(&mut self.integration_flags);
            pool.return_buffer(&mut self.coarse_batch_integration_responsibilities);
            self.merged_constrained_body_handles.dispose(pool);
        }
    }

    /// Solves all constraints using substepped velocity iterations.
    ///
    /// When a thread dispatcher is provided, uses the multithreaded solve path with lock-free
    /// work stealing and spin-wait synchronization. Otherwise falls back to sequential execution.
    pub unsafe fn solve(
        &mut self,
        total_dt: f32,
        thread_dispatcher: Option<&dyn crate::utilities::thread_dispatcher::IThreadDispatcher>,
    ) {
        let substep_dt = total_dt / self.substep_count as f32;

        // Prepare integration callbacks for this substep timestep.
        if let Some(pi_ptr) = self.pose_integrator {
            (*pi_ptr).prepare_for_integration(substep_dt);
        }

        if let Some(dispatcher) = thread_dispatcher {
            // Multithreaded path.
            self.execute_multithreaded(substep_dt, dispatcher);
        } else {
            // Single-threaded path.
            let inverse_dt = 1.0 / substep_dt;
            let self_ptr = self as *mut Self;

            let active_set = (*self_ptr).sets.get_mut(0);
            let batch_count = active_set.batches.count;

            if batch_count == 0 {
                return;
            }

            let bodies = &*(*self_ptr).bodies;

            for substep_index in 0..(*self_ptr).substep_count {
                (*self_ptr).on_substep_started(substep_index);

                if substep_index > 0 {
                    // Incremental update for substeps beyond the first.
                    let active_set = (*self_ptr).sets.get_mut(0);
                    for i in 0..active_set.batches.count {
                        let batch = active_set.batches.get_mut(i);
                        for j in 0..batch.type_batches.count {
                            let type_batch = batch.type_batches.get_mut(j);
                            let type_id = type_batch.type_id;
                            let processor = (&(*self_ptr).type_processors)[type_id as usize]
                                .as_ref()
                                .unwrap();
                            if processor.inner().requires_incremental_substep_updates() {
                                processor.inner().incrementally_update_for_substep(
                                    type_batch,
                                    bodies,
                                    substep_dt,
                                    inverse_dt,
                                    0,
                                    type_batch.bundle_count() as i32,
                                );
                            }
                        }
                    }
                    // Integrate kinematic poses and velocities for substeps > 0.
                    if let Some(pi_ptr) = (*self_ptr).pose_integrator {
                        let pi = &*pi_ptr;
                        let handles = &(*self_ptr).constrained_kinematic_handles;
                        let handle_buf = handles.span.slice_count(handles.count);
                        let bundle_count = BundleIndexing::get_bundle_count(handles.count as usize) as i32;
                        pi.integrate_kinematic_poses_and_velocities_dispatch(
                            &handle_buf, 0, bundle_count, substep_dt, 0,
                        );
                    }
                } else {
                    // First substep: integrate kinematic velocities if needed.
                    if let Some(pi_ptr) = (*self_ptr).pose_integrator {
                        let pi = &*pi_ptr;
                        if pi.integrate_velocity_for_kinematics() {
                            let handles = &(*self_ptr).constrained_kinematic_handles;
                            let handle_buf = handles.span.slice_count(handles.count);
                            let bundle_count = BundleIndexing::get_bundle_count(handles.count as usize) as i32;
                            pi.integrate_kinematic_velocities_dispatch(
                                &handle_buf, 0, bundle_count, substep_dt, 0,
                            );
                        }
                    }
                }

                // Warm start all batches.
                let active_set = (*self_ptr).sets.get_mut(0);
                let allow_pose_integration = substep_index > 0;
                for i in 0..active_set.batches.count {
                    let batch = active_set.batches.get_mut(i);
                    for j in 0..batch.type_batches.count {
                        let type_batch = batch.type_batches.get_mut(j);
                        let type_id = type_batch.type_id;
                        let bundle_count = type_batch.bundle_count() as i32;
                        let processor = (&(*self_ptr).type_processors)[type_id as usize]
                            .as_ref()
                            .unwrap();
                        (*self_ptr).warm_start_block(
                            0, i, j,
                            0, bundle_count,
                            type_batch, processor.inner(),
                            allow_pose_integration,
                            substep_dt, inverse_dt,
                        );
                    }
                }

                // Velocity iterations.
                let velocity_iteration_count =
                    (*self_ptr).get_velocity_iteration_count_for_substep_index(substep_index);
                for _iteration_index in 0..velocity_iteration_count {
                    let active_set = (*self_ptr).sets.get_mut(0);
                    for i in 0..active_set.batches.count {
                        let batch = active_set.batches.get_mut(i);
                        for j in 0..batch.type_batches.count {
                            let type_batch = batch.type_batches.get_mut(j);
                            let type_id = type_batch.type_id;
                            let bundle_count = type_batch.bundle_count() as i32;
                            (&(*self_ptr).type_processors)[type_id as usize]
                                .as_ref()
                                .unwrap()
                                .inner()
                                .solve(
                                    type_batch,
                                    bodies,
                                    substep_dt,
                                    inverse_dt,
                                    0,
                                    bundle_count,
                                );
                        }
                    }
                }

                (*self_ptr).on_substep_ended(substep_index);
            }
        }
    }
}

#[derive(Clone, Copy)]
enum EnumerateMode {
    Raw,
    Decoded,
    DynamicOnly,
}

unsafe impl Send for Solver {}
unsafe impl Sync for Solver {}
