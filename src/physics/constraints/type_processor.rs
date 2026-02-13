// Translated from BepuPhysics/Constraints/TypeProcessor.cs
//
// TypeProcessor is an abstract class in C# that acts as a vtable / function-pointer-like
// processor for TypeBatch data. It holds no constraint state itself.
// In Rust, we model this as a trait + a concrete struct holding cached metadata.

use crate::physics::bodies::Bodies;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::for_each_ref::IForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Collects body references from active constraints and converts them into properly flagged body handles.
/// Used by the convenience `transfer_constraint_auto_collect` to gather dynamic body handles
/// and encoded body indices before calling the full `transfer_constraint`.
struct ActiveKinematicFlaggedBodyHandleCollector<'a> {
    bodies: &'a Bodies,
    dynamic_body_handles: *mut i32,
    dynamic_count: *mut usize,
    encoded_body_indices: *mut i32,
    index_count: usize,
}

impl<'a> IForEach<i32> for ActiveKinematicFlaggedBodyHandleCollector<'a> {
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            if Bodies::is_encoded_dynamic_reference(encoded_body_index) {
                let count = *self.dynamic_count;
                *self.dynamic_body_handles.add(count) = self.bodies.active_set().index_to_handle.get(encoded_body_index).0;
                *self.dynamic_count = count + 1;
            }
            *self.encoded_body_indices.add(self.index_count) = encoded_body_index;
            self.index_count += 1;
        }
    }
}

/// Superclass of constraint type batch processors. Responsible for interpreting raw type batches
/// for the purposes of bookkeeping and solving.
///
/// This holds no actual constraint state. A solver creates a unique type processor for each
/// registered constraint type, and all instances are held in untyped memory.
/// Conceptually, the solver's array of TypeProcessors are like C function pointers.
///
/// Raw lane copy helper for transfer_constraint — copies one lane from source bundle to target bundle.
/// Works with raw byte pointers since we're type-erased through trait objects.
/// Equivalent to `GatherScatter.CopyLane<T>` in C#.
#[inline]
unsafe fn copy_lane_raw(
    source_bundle: *const u8,
    source_inner: usize,
    target_bundle: *mut u8,
    target_inner: usize,
    bundle_size_bytes: usize,
) {
    let count = crate::utilities::vector::VECTOR_WIDTH;
    // size_in_ints = (bundle_size_bytes >> 2) & !vector_mask
    let size_in_ints = (bundle_size_bytes >> 2) & !crate::utilities::bundle_indexing::BundleIndexing::vector_mask();

    let source_base = (source_bundle as *const i32).add(source_inner);
    let target_base = (target_bundle as *mut i32).add(target_inner);

    if size_in_ints == 0 {
        return;
    }
    *target_base = *source_base;

    let mut offset = count;
    while offset < size_in_ints {
        *target_base.add(offset) = *source_base.add(offset);
        offset += count;
    }
}

pub trait ITypeProcessor {
    /// Gets the number of bodies associated with each constraint in this type processor.
    fn bodies_per_constraint(&self) -> i32;

    /// Gets the number of degrees of freedom that each constraint constrains.
    fn constrained_degrees_of_freedom(&self) -> i32;

    /// Gets the type id assigned to this processor.
    fn type_id(&self) -> i32;

    /// Whether this constraint type requires incremental substep updates (e.g. contacts).
    fn requires_incremental_substep_updates(&self) -> bool {
        false
    }

    /// Allocates a slot in the type batch (non-fallback).
    fn allocate_in_type_batch(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: &mut BufferPool,
    ) -> i32;

    /// Allocates a slot in the type batch for a fallback batch.
    fn allocate_in_type_batch_for_fallback(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: *mut BufferPool,
    ) -> i32;

    /// Removes a constraint at the given index.
    fn remove(
        &self,
        type_batch: &mut TypeBatch,
        index: i32,
        handles_to_constraints: &mut Buffer<ConstraintLocation>,
        is_fallback: bool,
    );

    /// Initializes a type batch with the given capacity.
    fn initialize(&self, type_batch: &mut TypeBatch, initial_capacity: i32, pool: &mut BufferPool);

    /// Resizes a type batch.
    fn resize(&self, type_batch: &mut TypeBatch, new_capacity: i32, pool: &mut BufferPool);

    /// Scales all accumulated impulses by the given factor.
    fn scale_accumulated_impulses(&self, type_batch: &mut TypeBatch, scale: f32);

    /// Copies constraint data from a sleeping set's type batch into the active set's type batch.
    fn copy_sleeping_to_active(
        &self,
        _source_set: i32,
        _batch_index: i32,
        _source_type_batch_index: i32,
        _target_type_batch_index: i32,
        _source_start: i32,
        _target_start: i32,
        _count: i32,
        _bodies: &crate::physics::bodies::Bodies,
        _solver: &crate::physics::solver::Solver,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Gathers constraint data from the active set into an inactive type batch.
    /// For each constraint handle in the source scaffold, copies prestep data, accumulated impulses,
    /// sets index_to_handle, and converts body references from encoded indices to handles.
    fn gather_active_constraints(
        &self,
        _bodies: &crate::physics::bodies::Bodies,
        _solver: &crate::physics::solver::Solver,
        _source_scaffold: &crate::physics::island_scaffold::IslandScaffoldTypeBatch,
        _start_index: i32,
        _end_index: i32,
        _target_type_batch: &mut TypeBatch,
    ) {
        // Default: no-op stub. Concrete type processors will override with
        // lane-copy logic and body reference index→handle conversion.
    }

    /// Adds body handles referenced by the waking type batch to the batch's referenced handles set.
    fn add_waking_body_handles_to_batch_references(
        &self,
        _type_batch: &TypeBatch,
        _target_batch_referenced_handles: &mut IndexSet,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Adds constraints from a sleeping fallback type batch into the active set's fallback type batch.
    /// Unlike bulk copy, this goes through per-constraint allocation to maintain the fallback batch invariant
    /// (no duplicate bodies per bundle).
    fn add_sleeping_to_active_for_fallback(
        &self,
        _source_set: i32,
        _source_type_batch_index: i32,
        _target_type_batch_index: i32,
        _bodies: &crate::physics::bodies::Bodies,
        _solver: &crate::physics::solver::Solver,
    ) {
        // Default: no-op stub. Concrete implementations will override.
    }

    /// Transfers a constraint from one batch's type batch to another batch's type batch of the same type.
    /// Uses pre-computed dynamic body handles and encoded body indices.
    fn transfer_constraint(
        &self,
        source_type_batch: &mut TypeBatch,
        _source_batch_index: i32,
        index_in_type_batch: i32,
        solver: *mut crate::physics::solver::Solver,
        _bodies: *mut crate::physics::bodies::Bodies,
        target_batch_index: i32,
        dynamic_body_handles: &[crate::physics::handles::BodyHandle],
        encoded_body_indices: &[i32],
    ) {
        unsafe {
            let constraint_handle = *source_type_batch.index_to_handle.get(index_in_type_batch);
            let type_id = source_type_batch.type_id;

            // Allocate a spot in the target batch. This sets up body references and batch-referenced handles.
            let target_reference = (*solver).allocate_in_batch(
                target_batch_index,
                constraint_handle,
                dynamic_body_handles,
                encoded_body_indices,
                type_id,
            );

            // Copy prestep data and accumulated impulses lane-by-lane.
            let (mut _body_ref_size, mut prestep_size, mut accum_size) = (0i32, 0i32, 0i32);
            self.get_bundle_type_sizes(&mut _body_ref_size, &mut prestep_size, &mut accum_size);

            let (mut source_bundle, mut source_inner) = (0usize, 0usize);
            crate::utilities::bundle_indexing::BundleIndexing::get_bundle_indices(
                index_in_type_batch as usize, &mut source_bundle, &mut source_inner,
            );
            let (mut target_bundle, mut target_inner) = (0usize, 0usize);
            crate::utilities::bundle_indexing::BundleIndexing::get_bundle_indices(
                target_reference.index_in_type_batch as usize, &mut target_bundle, &mut target_inner,
            );

            let target_type_batch = &mut *target_reference.type_batch_pointer;

            // Copy prestep data
            copy_lane_raw(
                source_type_batch.prestep_data.as_ptr().add(source_bundle as usize * prestep_size as usize),
                source_inner,
                target_type_batch.prestep_data.as_mut_ptr().add(target_bundle as usize * prestep_size as usize),
                target_inner,
                prestep_size as usize,
            );

            // Copy accumulated impulses
            copy_lane_raw(
                source_type_batch.accumulated_impulses.as_ptr().add(source_bundle as usize * accum_size as usize),
                source_inner,
                target_type_batch.accumulated_impulses.as_mut_ptr().add(target_bundle as usize * accum_size as usize),
                target_inner,
                accum_size as usize,
            );

            // Update the solver's handle-to-constraint mapping.
            let location = (*solver).handle_to_constraint.get_mut(constraint_handle.0);
            location.batch_index = target_batch_index;
            location.index_in_type_batch = target_reference.index_in_type_batch;
            location.type_id = type_id;

            // Remove from the source batch. Use RemoveFromBatch (not Remove) to avoid
            // returning the constraint handle to the pool.
            (*solver).remove_from_batch(_source_batch_index, type_id, index_in_type_batch);
        }
    }

    /// Convenience wrapper that auto-collects dynamic body handles and encoded body indices,
    /// then calls the full `transfer_constraint`.
    fn transfer_constraint_auto_collect(
        &self,
        source_type_batch: &mut TypeBatch,
        source_batch_index: i32,
        index_in_type_batch: i32,
        solver: *mut crate::physics::solver::Solver,
        bodies: *mut crate::physics::bodies::Bodies,
        target_batch_index: i32,
    ) {
        unsafe {
            let bodies_per_constraint = self.bodies_per_constraint() as usize;
            let mut dynamic_body_handles = [0i32; 8]; // stack alloc — matches C# stackalloc
            let mut encoded_body_indices = [0i32; 8];
            debug_assert!(bodies_per_constraint <= 8, "Bodies per constraint exceeds stack buffer size");
            let mut dynamic_count = 0usize;
            let bodies_ref = &*bodies;

            // Enumerate raw body references to collect handles and indices
            (*solver).enumerate_connected_raw_body_references_from_type_batch(
                &*source_type_batch,
                index_in_type_batch,
                &mut ActiveKinematicFlaggedBodyHandleCollector {
                    bodies: bodies_ref,
                    dynamic_body_handles: dynamic_body_handles.as_mut_ptr(),
                    dynamic_count: &mut dynamic_count,
                    encoded_body_indices: encoded_body_indices.as_mut_ptr(),
                    index_count: 0,
                },
            );

            let dynamic_handles: &[crate::physics::handles::BodyHandle] = std::slice::from_raw_parts(
                dynamic_body_handles.as_ptr() as *const crate::physics::handles::BodyHandle,
                dynamic_count,
            );

            self.transfer_constraint(
                source_type_batch,
                source_batch_index,
                index_in_type_batch,
                solver,
                bodies,
                target_batch_index,
                dynamic_handles,
                &encoded_body_indices[..bodies_per_constraint],
            );
        }
    }

    /// Updates body references within a constraint when a body is moved in memory.
    /// Returns true if the moved body is kinematic.
    fn update_for_body_memory_move(
        &self,
        type_batch: &mut TypeBatch,
        index_in_type_batch: i32,
        body_index_in_constraint: i32,
        new_body_location: i32,
    ) -> bool {
        unsafe {
            let (mut bundle_index, mut inner_index) = (0usize, 0usize);
            crate::utilities::bundle_indexing::BundleIndexing::get_bundle_indices(
                index_in_type_batch as usize,
                &mut bundle_index,
                &mut inner_index,
            );
            // Body references are laid out as N consecutive Vector<int> (one per body).
            // Access the correct lane: innerIndex + bodyIndexInConstraint * Vector<int>::LEN
            let vector_len = crate::utilities::vector::VECTOR_WIDTH;
            let body_refs_base = type_batch.body_references.as_mut_ptr() as *mut i32;
            let vector_size_bytes = vector_len * std::mem::size_of::<i32>();
            let bundle_offset = bundle_index * self.bodies_per_constraint() as usize * vector_size_bytes;
            let lane_offset = inner_index + body_index_in_constraint as usize * vector_len;
            let reference_location = &mut *body_refs_base.add(bundle_offset / std::mem::size_of::<i32>() + lane_offset);

            // Preserve the old kinematic mask so the caller doesn't have to re-query.
            let is_kinematic = Bodies::is_encoded_kinematic_reference(*reference_location);
            *reference_location = new_body_location | (*reference_location & Bodies::KINEMATIC_MASK as i32);
            is_kinematic
        }
    }

    /// Returns the capacity of the given type batch.
    fn capacity(&self, type_batch: &TypeBatch) -> i32 {
        type_batch.index_to_handle.len()
    }

    /// Gets the size in bytes of the body references, prestep, and accumulated impulse bundle types.
    fn get_bundle_type_sizes(
        &self,
        body_references_bundle_size: &mut i32,
        prestep_bundle_size: &mut i32,
        accumulated_impulse_bundle_size: &mut i32,
    );

    /// Generates sort keys for constraints in the given range and copies body references to a cache buffer.
    /// Used by constraint layout optimization (batch compressor).
    fn generate_sort_keys_and_copy_references(
        &self,
        type_batch: &mut TypeBatch,
        bundle_start: i32,
        local_bundle_start: i32,
        bundle_count: i32,
        constraint_start: i32,
        local_constraint_start: i32,
        constraint_count: i32,
        first_sort_key: *mut i32,
        first_source_index: *mut i32,
        body_references_cache: &mut Buffer<u8>,
    );

    /// Copies index-to-handle mappings, prestep data, and accumulated impulses to cache buffers.
    fn copy_to_cache(
        &self,
        type_batch: &mut TypeBatch,
        bundle_start: i32,
        local_bundle_start: i32,
        bundle_count: i32,
        constraint_start: i32,
        local_constraint_start: i32,
        constraint_count: i32,
        index_to_handle_cache: &mut Buffer<ConstraintHandle>,
        prestep_cache: &mut Buffer<u8>,
        accumulated_impulses_cache: &mut Buffer<u8>,
    );

    /// After sorting, regathers constraints from cache buffers back into the type batch in sorted order.
    fn regather(
        &self,
        type_batch: &mut TypeBatch,
        constraint_start: i32,
        constraint_count: i32,
        first_source_index: *mut i32,
        index_to_handle_cache: &mut Buffer<ConstraintHandle>,
        body_references_cache: &mut Buffer<u8>,
        prestep_cache: &mut Buffer<u8>,
        accumulated_impulses_cache: &mut Buffer<u8>,
        handles_to_constraints: &mut Buffer<ConstraintLocation>,
    );

    /// Gets the count of body references in the type batch matching the given body index.
    /// Debug-only function. Performance does not matter.
    fn get_body_reference_count(&self, _type_batch: &TypeBatch, _body: i32) -> i32 {
        0
    }

    /// Warm-starts the constraints in the given type batch. The warm start applies previously
    /// accumulated impulses to the body velocities to accelerate convergence.
    ///
    /// When `batch_integration_mode` is not `Never`, the warm start also integrates body
    /// state (pose and/or velocity) for bodies whose first constraint appearance is in this batch.
    fn warm_start(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        integration_flags: &crate::utilities::memory::buffer::Buffer<crate::utilities::collections::index_set::IndexSet>,
        pose_integrator: Option<&dyn crate::physics::pose_integrator::IPoseIntegrator>,
        batch_integration_mode: crate::physics::constraints::batch_integration_mode::BatchIntegrationMode,
        allow_pose_integration: bool,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
        worker_index: i32,
    ) {
        // Default: no-op. Concrete implementations will override.
        let _ = (type_batch, bodies, integration_flags, pose_integrator, batch_integration_mode, allow_pose_integration, dt, inverse_dt, start_bundle, exclusive_end_bundle, worker_index);
    }

    /// Solves the constraint velocity iteration for the given bundle range.
    fn solve(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        // Default: no-op. Concrete implementations will override.
        let _ = (type_batch, bodies, dt, inverse_dt, start_bundle, exclusive_end_bundle);
    }

    /// Performs an incremental update for substeps beyond the first.
    /// Only called for constraint types where `requires_incremental_substep_updates` returns true.
    fn incrementally_update_for_substep(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &crate::physics::bodies::Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        // Default: debug fail, should not be called unless the type supports it.
        debug_assert!(false, "An incremental update was scheduled for a type batch that does not have a contact data update implementation.");
        let _ = (type_batch, bodies, dt, inverse_dt, start_bundle, exclusive_end_bundle);
    }
}

/// Concrete metadata holder for a type processor. Stores cached type id and bodies-per-constraint
/// alongside a boxed trait implementation for virtual dispatch.
pub struct TypeProcessor {
    pub type_id: i32,
    pub bodies_per_constraint: i32,
    pub constrained_degrees_of_freedom: i32,
    inner: Box<dyn ITypeProcessor>,
}

impl TypeProcessor {
    pub fn new(type_id: i32, inner: Box<dyn ITypeProcessor>) -> Self {
        let bodies_per_constraint = inner.bodies_per_constraint();
        let constrained_degrees_of_freedom = inner.constrained_degrees_of_freedom();
        Self {
            type_id,
            bodies_per_constraint,
            constrained_degrees_of_freedom,
            inner,
        }
    }

    #[inline(always)]
    pub fn inner(&self) -> &dyn ITypeProcessor {
        &*self.inner
    }

    #[inline(always)]
    pub fn inner_mut(&mut self) -> &mut dyn ITypeProcessor {
        &mut *self.inner
    }
}
