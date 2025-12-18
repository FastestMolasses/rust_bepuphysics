// Translated from BepuPhysics/Solver.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::BodyInertia;
use crate::physics::body_set::BodyConstraintReference;
use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collision_detection::pair_cache::CollidablePair;
use crate::physics::constraint_batch::ConstraintBatch;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraint_reference::ConstraintReference;
use crate::physics::constraint_set::ConstraintSet;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::{ITypeProcessor, TypeProcessor};
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
    // TODO: pub(crate) pair_cache: *mut PairCache,
    pub(crate) awakener: *mut IslandAwakener,

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
        debug_assert!(value >= 1, "Substep count must be positive.");
        self.substep_count = value;
    }

    pub fn velocity_iteration_count(&self) -> i32 {
        self.velocity_iteration_count
    }

    pub fn set_velocity_iteration_count(&mut self, value: i32) {
        debug_assert!(value >= 1, "Iteration count must be positive.");
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
            awakener: std::ptr::null_mut(),
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
                pool,
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
        let mut encoded_body_indices = vec![0i32; body_handles.len()];
        let blocking = self.get_blocking_body_handles(body_handles, &mut encoded_body_indices);

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

        // TODO: pair_cache.remove_reference_if_contact_constraint(handle, location.type_id);

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
    unsafe fn try_remove_dynamic_body_from_fallback(
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

    /// Prepares constraint integration responsibilities. Returns an IndexSet of constrained body handles.
    ///
    /// In the full C# implementation, this computes per-batch, per-type-batch integration flags
    /// that determine which constraint is responsible for integrating each body's velocities
    /// during substepping. For now, this provides a simplified implementation that marks all
    /// constrained body handles, which is correct for single-threaded execution where integration
    /// responsibilities are handled by the warm-start's batch-0 special case (batch 0 always
    /// integrates all its bodies).
    pub unsafe fn prepare_constraint_integration_responsibilities(
        &mut self,
        _thread_dispatcher: Option<&dyn crate::utilities::thread_dispatcher::IThreadDispatcher>,
    ) -> IndexSet {
        if self.active_set().batches.count > 0 {
            let pool = &mut *self.pool;
            let bodies = &*self.bodies;
            // Build a merged set of all constrained body handles.
            // Start by copying batch 0's referenced handles.
            let highest_id = bodies.handle_pool.highest_possibly_claimed_id();
            let flags_len = (highest_id + 64) / 64;
            let mut merged = IndexSet::new(pool, flags_len * 64);

            if self.batch_referenced_handles.count > 0 {
                let batch0_flags = &self.batch_referenced_handles.get(0).flags;
                let copy_len = batch0_flags.len().min(merged.flags.len());
                batch0_flags.copy_to(0, &mut merged.flags, 0, copy_len);
                if copy_len < merged.flags.len() {
                    merged.flags.clear(copy_len, merged.flags.len() - copy_len);
                }
            }
            // Merge remaining batches.
            for batch_index in 1..self.batch_referenced_handles.count {
                let batch_handles = &self.batch_referenced_handles.get(batch_index);
                let bundle_count = merged.flags.len().min(batch_handles.flags.len());
                for i in 0..bundle_count {
                    *merged.flags.get_mut(i) |= *batch_handles.flags.get(i);
                }
            }
            // Add constrained kinematics.
            for i in 0..self.constrained_kinematic_handles.count {
                merged.add_unsafely(*self.constrained_kinematic_handles.span.get(i));
            }
            merged
        } else {
            IndexSet::empty()
        }
    }

    /// Disposes the constraint integration responsibilities created by prepare_constraint_integration_responsibilities.
    pub fn dispose_constraint_integration_responsibilities(&mut self) {
        // In the full implementation, this disposes per-batch integration flags.
        // With the simplified implementation above, there's nothing to dispose beyond
        // the merged IndexSet which is returned by value and disposed by the caller.
    }

    /// Solves all constraints using substepped velocity iterations.
    ///
    /// This is the single-threaded solve path translated from `Solver<TIntegrationCallbacks>.Solve()`
    /// in Solver_Solve.cs. The multithreaded path will be added when thread dispatch infrastructure
    /// is complete.
    pub unsafe fn solve(
        &mut self,
        total_dt: f32,
        _thread_dispatcher: Option<&dyn crate::utilities::thread_dispatcher::IThreadDispatcher>,
    ) {
        let substep_dt = total_dt / self.substep_count as f32;
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
                // TODO: Integrate kinematic poses and velocities for substeps > 0
                // PoseIntegrator.IntegrateKinematicPosesAndVelocities(...)
            } else {
                // First substep: integrate kinematic velocities if needed.
                // TODO: if PoseIntegrator.Callbacks.IntegrateVelocityForKinematics {
                //     PoseIntegrator.IntegrateKinematicVelocities(...)
                // }
            }

            // Warm start all batches.
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
                        .warm_start(
                            type_batch,
                            bodies,
                            substep_dt,
                            inverse_dt,
                            0,
                            bundle_count,
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

#[derive(Clone, Copy)]
enum EnumerateMode {
    Raw,
    Decoded,
    DynamicOnly,
}

unsafe impl Send for Solver {}
unsafe impl Sync for Solver {}
