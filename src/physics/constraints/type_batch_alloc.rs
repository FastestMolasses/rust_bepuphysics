// Translated from BepuPhysics/Constraints/TypeProcessor.cs
// Shared implementation of TypeBatch allocation, removal, initialization, and resize.
// These are the generic methods from TypeProcessor<TBodyReferences, TPrestepData, TAccumulatedImpulse>
// that are identical regardless of body count. They are expressed as free functions parameterized
// by type sizes, so 1/2/3/4-body type processor impls can all call into them.

use crate::physics::bodies::Bodies;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::collections::quick_dictionary::HashHelper;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::vector::Vector;
use std::simd::prelude::*;

/// Sets one lane of the body references bundle at `inner_index` to the given body indices.
///
/// C# equivalent: `SetBodyReferencesLane<TBodyReferences>(ref bundle, innerIndex, bodyIndices)`.
/// The body references bundle is a struct of N `Vector<int>` (one per body in the constraint).
/// We treat it as a flat array of i32 with stride `Vector<int>::LEN` between consecutive bodies.
#[inline(always)]
pub unsafe fn set_body_references_lane(
    bundle_ptr: *mut u8,
    inner_index: usize,
    body_indices: &[i32],
    bodies_per_constraint: usize,
) {
    let base = bundle_ptr as *mut i32;
    let target_lane = base.add(inner_index);
    for i in 0..bodies_per_constraint {
        *target_lane.add(i * Vector::<i32>::LEN) = body_indices[i];
    }
}

/// Sets one lane of the body references bundle, initializing the entire bundle to -1 if this
/// is lane 0 (first constraint in a new bundle).
///
/// C# equivalent: `AddBodyReferencesLane<TBodyReferences>(ref bundle, innerIndex, bodyIndices)`.
#[inline(always)]
pub unsafe fn add_body_references_lane(
    bundle_ptr: *mut u8,
    inner_index: usize,
    body_indices: &[i32],
    bodies_per_constraint: usize,
) {
    if inner_index == 0 {
        // Clear the entire bundle to -1. The bundle contains `bodies_per_constraint` vectors.
        let neg_one: Vector<i32> = Simd::splat(-1);
        let body_ref_vector_ptr = bundle_ptr as *mut Vector<i32>;
        for i in 0..bodies_per_constraint {
            *body_ref_vector_ptr.add(i) = neg_one;
        }
    }
    set_body_references_lane(bundle_ptr, inner_index, body_indices, bodies_per_constraint);
}

/// Clears one lane of the body references bundle to -1.
///
/// C# equivalent: `RemoveBodyReferencesLane<TBodyReferences>(ref bundle, innerIndex)`.
#[inline(always)]
pub unsafe fn remove_body_references_lane(
    bundle_ptr: *mut u8,
    inner_index: usize,
    bodies_per_constraint: usize,
) {
    let base = bundle_ptr as *mut i32;
    let target_lane = base.add(inner_index);
    for i in 0..bodies_per_constraint {
        *target_lane.add(i * Vector::<i32>::LEN) = -1;
    }
}

/// Copies constraint data (body references, prestep, accumulated impulses, handle) from one slot
/// to another. Used during removal (swap-back).
///
/// C# equivalent: `Move(...)` in TypeProcessor<TBodyReferences, TPrestepData, TAccumulatedImpulse>.
#[inline(always)]
pub unsafe fn move_constraint<
    TBodyReferences: Copy,
    TPrestepData: Copy,
    TAccumulatedImpulse: Copy,
>(
    source_references_bundle: *const TBodyReferences,
    source_prestep_bundle: *const TPrestepData,
    source_accumulated_bundle: *const TAccumulatedImpulse,
    source_handle: ConstraintHandle,
    source_inner: usize,
    target_references_bundle: *mut TBodyReferences,
    target_prestep_bundle: *mut TPrestepData,
    target_accumulated_bundle: *mut TAccumulatedImpulse,
    target_index_to_handle: *mut ConstraintHandle,
    target_inner: usize,
    target_index: i32,
    handles_to_constraints: &mut Buffer<ConstraintLocation>,
) {
    GatherScatter::copy_lane(
        &*source_references_bundle,
        source_inner,
        &mut *target_references_bundle,
        target_inner,
    );
    GatherScatter::copy_lane(
        &*source_prestep_bundle,
        source_inner,
        &mut *target_prestep_bundle,
        target_inner,
    );
    GatherScatter::copy_lane(
        &*source_accumulated_bundle,
        source_inner,
        &mut *target_accumulated_bundle,
        target_inner,
    );
    *target_index_to_handle = source_handle;
    handles_to_constraints
        .get_mut(source_handle.0)
        .index_in_type_batch = target_index;
}

/// Resizes the internal buffers of a TypeBatch to accommodate at least `constraint_capacity`
/// constraints.
///
/// C# equivalent: `InternalResize(ref TypeBatch, BufferPool, int constraintCapacity)`.
pub fn internal_resize(
    type_batch: &mut TypeBatch,
    pool: &mut BufferPool,
    constraint_capacity: i32,
    body_references_bundle_size: usize,
    prestep_bundle_size: usize,
    accumulated_impulse_bundle_size: usize,
) {
    debug_assert!(
        constraint_capacity >= 0,
        "The constraint capacity should have already been validated."
    );
    let copy_count = type_batch.constraint_count;
    pool.resize_to_at_least(
        &mut type_batch.index_to_handle,
        constraint_capacity,
        copy_count,
    );
    let bundle_capacity =
        BundleIndexing::get_bundle_count(type_batch.index_to_handle.len() as usize);
    let bundle_count = type_batch.bundle_count();
    pool.resize_to_at_least(
        &mut type_batch.body_references,
        (bundle_capacity * body_references_bundle_size) as i32,
        (bundle_count * body_references_bundle_size) as i32,
    );
    pool.resize_to_at_least(
        &mut type_batch.prestep_data,
        (bundle_capacity * prestep_bundle_size) as i32,
        (bundle_count * prestep_bundle_size) as i32,
    );
    pool.resize_to_at_least(
        &mut type_batch.accumulated_impulses,
        (bundle_capacity * accumulated_impulse_bundle_size) as i32,
        (bundle_count * accumulated_impulse_bundle_size) as i32,
    );
}

/// Initializes a TypeBatch with the given capacity.
///
/// C# equivalent: `Initialize(ref TypeBatch, int initialCapacity, BufferPool pool)`.
pub fn initialize_type_batch(
    type_batch: &mut TypeBatch,
    type_id: i32,
    initial_capacity: i32,
    pool: &mut BufferPool,
    body_references_bundle_size: usize,
    prestep_bundle_size: usize,
    accumulated_impulse_bundle_size: usize,
) {
    *type_batch = TypeBatch::default();
    type_batch.type_id = type_id;
    internal_resize(
        type_batch,
        pool,
        initial_capacity,
        body_references_bundle_size,
        prestep_bundle_size,
        accumulated_impulse_bundle_size,
    );
}

/// Resizes a TypeBatch to the desired capacity.
///
/// C# equivalent: `Resize(ref TypeBatch, int desiredCapacity, BufferPool pool)`.
pub fn resize_type_batch(
    type_batch: &mut TypeBatch,
    desired_capacity: i32,
    pool: &mut BufferPool,
    body_references_bundle_size: usize,
    prestep_bundle_size: usize,
    accumulated_impulse_bundle_size: usize,
) {
    let desired_constraint_capacity = BufferPool::get_capacity_for_count::<i32>(desired_capacity);
    if desired_constraint_capacity != type_batch.index_to_handle.len() {
        internal_resize(
            type_batch,
            pool,
            desired_constraint_capacity,
            body_references_bundle_size,
            prestep_bundle_size,
            accumulated_impulse_bundle_size,
        );
    }
}

/// Allocates a constraint slot in a type batch (non-fallback path).
/// Returns the index of the newly allocated constraint.
///
/// C# equivalent: `AllocateInTypeBatch(ref TypeBatch, ConstraintHandle, Span<int>, BufferPool)`.
pub unsafe fn allocate_in_type_batch(
    type_batch: &mut TypeBatch,
    handle: ConstraintHandle,
    body_indices: &[i32],
    pool: &mut BufferPool,
    bodies_per_constraint: usize,
    body_references_bundle_size: usize,
    prestep_bundle_size: usize,
    accumulated_impulse_bundle_size: usize,
) -> i32 {
    debug_assert!(
        type_batch.body_references.allocated(),
        "Should initialize the batch before allocating anything from it."
    );
    if type_batch.constraint_count == type_batch.index_to_handle.len() {
        internal_resize(
            type_batch,
            pool,
            type_batch.constraint_count * 2,
            body_references_bundle_size,
            prestep_bundle_size,
            accumulated_impulse_bundle_size,
        );
    }
    let index = type_batch.constraint_count;
    type_batch.constraint_count += 1;
    *type_batch.index_to_handle.get_mut(index) = handle;

    let mut bundle_index = 0usize;
    let mut inner_index = 0usize;
    BundleIndexing::get_bundle_indices(index as usize, &mut bundle_index, &mut inner_index);

    let bundle_ptr =
        (type_batch.body_references.as_mut_ptr()).add(bundle_index * body_references_bundle_size);
    add_body_references_lane(bundle_ptr, inner_index, body_indices, bodies_per_constraint);

    // Clear the slot's accumulated impulse. The backing memory could be initialized to any value.
    let impulse_bundle_ptr = type_batch
        .accumulated_impulses
        .as_mut_ptr()
        .add(bundle_index * accumulated_impulse_bundle_size);
    clear_lane_raw(
        impulse_bundle_ptr,
        inner_index,
        accumulated_impulse_bundle_size,
    );

    index
}

/// Removes a constraint from a type batch (non-fallback path, swap-back).
///
/// C# equivalent: non-fallback branch of `Remove(ref TypeBatch, int, ref Buffer<ConstraintLocation>, bool)`.
pub unsafe fn remove_from_type_batch<
    TBodyReferences: Copy,
    TPrestepData: Copy,
    TAccumulatedImpulse: Copy,
>(
    type_batch: &mut TypeBatch,
    index: i32,
    handles_to_constraints: &mut Buffer<ConstraintLocation>,
    bodies_per_constraint: usize,
) {
    debug_assert!(
        index >= 0 && index < type_batch.constraint_count,
        "Can only remove elements that are actually in the batch!"
    );
    let last_index = type_batch.constraint_count - 1;
    type_batch.constraint_count = last_index;

    let mut source_bundle_index = 0usize;
    let mut source_inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        last_index as usize,
        &mut source_bundle_index,
        &mut source_inner_index,
    );

    let body_references = type_batch.body_references.as_mut_ptr() as *mut TBodyReferences;
    if index < last_index {
        let prestep_data = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
        let accumulated_impulses =
            type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

        let mut target_bundle_index = 0usize;
        let mut target_inner_index = 0usize;
        BundleIndexing::get_bundle_indices(
            index as usize,
            &mut target_bundle_index,
            &mut target_inner_index,
        );

        move_constraint::<TBodyReferences, TPrestepData, TAccumulatedImpulse>(
            body_references.add(source_bundle_index),
            prestep_data.add(source_bundle_index),
            accumulated_impulses.add(source_bundle_index),
            *type_batch.index_to_handle.get(last_index),
            source_inner_index,
            body_references.add(target_bundle_index),
            prestep_data.add(target_bundle_index),
            accumulated_impulses.add(target_bundle_index),
            type_batch.index_to_handle.get_mut_ptr(index),
            target_inner_index,
            index,
            handles_to_constraints,
        );
    }
    remove_body_references_lane(
        body_references.add(source_bundle_index) as *mut u8,
        source_inner_index,
        bodies_per_constraint,
    );
}

/// Removes a constraint from a fallback type batch.
/// In fallback batches, constraints can have gaps within bundles, so removal is more complex.
///
/// C# equivalent: fallback branch of `Remove(ref TypeBatch, int, ref Buffer<ConstraintLocation>, bool)`.
pub unsafe fn remove_from_type_batch_fallback<
    TBodyReferences: Copy,
    TPrestepData: Copy,
    TAccumulatedImpulse: Copy,
>(
    type_batch: &mut TypeBatch,
    index: i32,
    handles_to_constraints: &mut Buffer<ConstraintLocation>,
    bodies_per_constraint: usize,
) {
    debug_assert!(
        index >= 0 && index < type_batch.constraint_count,
        "Can only remove elements that are actually in the batch!"
    );

    let mut removed_bundle_index = 0usize;
    let mut removed_inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        index as usize,
        &mut removed_bundle_index,
        &mut removed_inner_index,
    );

    let body_references = type_batch.body_references.as_mut_ptr() as *mut TBodyReferences;
    let removed_bundle_slot = body_references.add(removed_bundle_index);

    // Mark handle slot as empty.
    type_batch.index_to_handle.get_mut(index).0 = -1;
    // Clear the body references lane.
    remove_body_references_lane(
        removed_bundle_slot as *mut u8,
        removed_inner_index,
        bodies_per_constraint,
    );

    // Check if all slots in the bundle are now empty by inspecting the first body's vector.
    let first_body_refs = *(removed_bundle_slot as *const Vector<i32>);
    let all_negative = first_body_refs.simd_lt(Simd::splat(0i32)).all();

    if all_negative {
        // All slots in the bundle are now empty; this bundle should be removed.
        let mut last_bundle_index = type_batch.bundle_count() - 1;
        if removed_bundle_index != last_bundle_index {
            let prestep_data = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let accumulated_impulses =
                type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            // Move the last bundle into the removed slot.
            *body_references.add(removed_bundle_index) = *body_references.add(last_bundle_index);
            *prestep_data.add(removed_bundle_index) = *prestep_data.add(last_bundle_index);
            *accumulated_impulses.add(removed_bundle_index) =
                *accumulated_impulses.add(last_bundle_index);

            // Update handle-to-constraint mappings for constraints that moved.
            let first_body_lane_for_moved_bundle =
                body_references.add(last_bundle_index) as *const i32;
            let constraint_index_shift =
                (last_bundle_index - removed_bundle_index) * Vector::<i32>::LEN;
            let bundle_start_index_in_constraints = last_bundle_index * Vector::<i32>::LEN;
            for i in 0..Vector::<i32>::LEN {
                if *first_body_lane_for_moved_bundle.add(i) >= 0 {
                    let constraint_index = (bundle_start_index_in_constraints + i) as i32;
                    let new_constraint_index = constraint_index - constraint_index_shift as i32;
                    let handle = *type_batch.index_to_handle.get(constraint_index);
                    type_batch.index_to_handle.get_mut(constraint_index).0 = -1;
                    *type_batch.index_to_handle.get_mut(new_constraint_index) = handle;
                    handles_to_constraints.get_mut(handle.0).index_in_type_batch =
                        new_constraint_index;
                }
            }
            last_bundle_index -= 1;
        }

        // Determine the new constraint count from the last bundle.
        let last_bundle_body_refs = *(body_references.add(last_bundle_index) as *const Vector<i32>);
        let inner_lane_count = BundleIndexing::get_last_set_lane_count(
            last_bundle_body_refs.simd_ge(Simd::splat(0i32)).to_int(),
        );
        type_batch.constraint_count =
            (last_bundle_index * Vector::<i32>::LEN + inner_lane_count) as i32;
    }
}

/// Clears one lane of a type (treated as array of f32 vectors) by zeroing out
/// the lane at `inner_index` across all vector slots.
#[inline(always)]
unsafe fn clear_lane_raw(bundle_ptr: *mut u8, inner_index: usize, bundle_byte_size: usize) {
    let vector_count = bundle_byte_size / std::mem::size_of::<Vector<f32>>();
    let lane_base = (bundle_ptr as *mut f32).add(inner_index);
    for i in 0..vector_count {
        *lane_base.add(i * Vector::<f32>::LEN) = 0.0;
    }
}

/// Checks whether a new constraint with the given broadcasted body indices can be placed
/// in the given bundle without conflicting with any existing body references.
///
/// C# equivalent: `AllowFallbackBundleAllocation(ref bundle, broadcastedBodyIndices)`.
#[inline(always)]
unsafe fn allow_fallback_bundle_allocation(
    bundle_ptr: *const u8,
    broadcasted_body_indices: *const Vector<i32>,
    bodies_per_constraint: usize,
) -> bool {
    let bundle_body_indices = bundle_ptr as *const Vector<i32>;
    for broadcasted_body_in_constraint in 0..bodies_per_constraint {
        let broadcasted_bodies = *broadcasted_body_indices.add(broadcasted_body_in_constraint);
        for bundle_body_in_constraint in 0..bodies_per_constraint {
            // The broadcastedBodies were created with the kinematic flag stripped, so
            // kinematics will never match against constraint-held references.
            let bundle_body = *bundle_body_indices.add(bundle_body_in_constraint);
            if broadcasted_bodies.simd_eq(bundle_body).any() {
                return false;
            }
        }
    }
    // Not blocked by matching indices, but is there room? At least one slot must have -1.
    let first_body_refs = *bundle_body_indices;
    first_body_refs.simd_lt(Simd::splat(0i32)).any()
}

/// Finds the first empty lane (containing -1) in the first body references vector of the bundle.
///
/// C# equivalent: `GetInnerIndexForFallbackAllocation(ref bundle)`.
#[inline(always)]
unsafe fn get_inner_index_for_fallback_allocation(bundle_ptr: *const u8) -> i32 {
    let first_body_refs = *(bundle_ptr as *const Vector<i32>);
    BundleIndexing::get_first_set_lane_index(first_body_refs.simd_lt(Simd::splat(0i32)).to_int())
}

/// Probes a single bundle for fallback allocation viability.
/// If the bundle can accept the constraint, sets the body references lane and returns true.
///
/// C# equivalent: `ProbeBundleForFallback(...)`.
#[inline(always)]
unsafe fn probe_bundle_for_fallback(
    body_references_ptr: *mut u8,
    broadcasted_body_indices: *const Vector<i32>,
    encoded_body_indices: &[i32],
    bundle_index: usize,
    body_references_bundle_size: usize,
    bodies_per_constraint: usize,
    target_bundle_index: &mut i32,
    target_inner_index: &mut i32,
) -> bool {
    let bundle_ptr = body_references_ptr.add(bundle_index * body_references_bundle_size);
    if allow_fallback_bundle_allocation(bundle_ptr, broadcasted_body_indices, bodies_per_constraint)
    {
        *target_bundle_index = bundle_index as i32;
        *target_inner_index = get_inner_index_for_fallback_allocation(bundle_ptr);
        set_body_references_lane(
            bundle_ptr as *mut u8,
            *target_inner_index as usize,
            encoded_body_indices,
            bodies_per_constraint,
        );
        true
    } else {
        false
    }
}

/// Allocates a constraint slot in a type batch for the sequential fallback batch.
/// Uses stochastic probing to find bundles without shared body references.
///
/// C# equivalent: `AllocateInTypeBatchForFallback(ref TypeBatch, ConstraintHandle, Span<int>, BufferPool)`.
pub unsafe fn allocate_in_type_batch_for_fallback(
    type_batch: &mut TypeBatch,
    handle: ConstraintHandle,
    encoded_body_indices: &[i32],
    pool: &mut BufferPool,
    bodies_per_constraint: usize,
    body_references_bundle_size: usize,
    prestep_bundle_size: usize,
    accumulated_impulse_bundle_size: usize,
) -> i32 {
    debug_assert!(
        type_batch.body_references.allocated(),
        "Should initialize the batch before allocating anything from it."
    );
    if type_batch.constraint_count == type_batch.index_to_handle.len() {
        // This isn't technically required (since probing might find an earlier slot),
        // but it makes things simpler and rarely allocates more than necessary.
        internal_resize(
            type_batch,
            pool,
            type_batch.constraint_count * 2,
            body_references_bundle_size,
            prestep_bundle_size,
            accumulated_impulse_bundle_size,
        );
    }

    const PROBE_LOCATION_COUNT: usize = 16;

    let mut target_bundle_index: i32 = -1;
    let mut target_inner_index: i32 = -1;

    // Build broadcasted body indices with kinematic flag stripped.
    // Using a fixed-size array (max 4 bodies per constraint) instead of stackalloc.
    let mut broadcasted_storage = [Simd::splat(0i32); 4];
    for i in 0..bodies_per_constraint {
        broadcasted_storage[i] = Simd::splat(encoded_body_indices[i] & Bodies::BODY_REFERENCE_MASK);
    }
    let broadcasted_body_indices = broadcasted_storage.as_ptr();

    let bundle_count = type_batch.bundle_count();
    let body_references_ptr = type_batch.body_references.as_mut_ptr();

    if bundle_count <= PROBE_LOCATION_COUNT + 1 {
        // Small batch: linear scan all bundles.
        for bundle_index in 0..bundle_count {
            if probe_bundle_for_fallback(
                body_references_ptr,
                broadcasted_body_indices,
                encoded_body_indices,
                bundle_index,
                body_references_bundle_size,
                bodies_per_constraint,
                &mut target_bundle_index,
                &mut target_inner_index,
            ) {
                break;
            }
        }
    } else {
        // Large batch: stochastic probing.
        // First, probe the final bundle just in case it's nice and simple.
        let last_bundle_index = bundle_count - 1;
        if !probe_bundle_for_fallback(
            body_references_ptr,
            broadcasted_body_indices,
            encoded_body_indices,
            last_bundle_index,
            body_references_bundle_size,
            bodies_per_constraint,
            &mut target_bundle_index,
            &mut target_inner_index,
        ) {
            // No room in the final bundle; do stochastic probes.
            let mut next_probe_index =
                ((HashHelper::rehash(handle.0) & 0x7FFF_FFFF) as usize) % last_bundle_index;
            let bundle_jump = bundle_count / PROBE_LOCATION_COUNT;
            let remainder = last_bundle_index - bundle_jump * PROBE_LOCATION_COUNT;
            for probe_index in 0..PROBE_LOCATION_COUNT {
                if probe_bundle_for_fallback(
                    body_references_ptr,
                    broadcasted_body_indices,
                    encoded_body_indices,
                    next_probe_index,
                    body_references_bundle_size,
                    bodies_per_constraint,
                    &mut target_bundle_index,
                    &mut target_inner_index,
                ) {
                    break;
                }
                next_probe_index += bundle_jump;
                if probe_index < remainder {
                    next_probe_index += 1;
                }
                if next_probe_index >= bundle_count {
                    next_probe_index -= bundle_count;
                }
            }
        }
    }

    if target_bundle_index == -1 {
        // None of the existing bundles can hold the constraint; allocate a new one.
        let old_count = type_batch.constraint_count;
        let index_in_type_batch = (bundle_count * Vector::<i32>::LEN) as i32;
        let new_constraint_count = index_in_type_batch + 1;
        if new_constraint_count >= type_batch.index_to_handle.len() {
            internal_resize(
                type_batch,
                pool,
                new_constraint_count * 2,
                body_references_bundle_size,
                prestep_bundle_size,
                accumulated_impulse_bundle_size,
            );
        }
        type_batch.constraint_count = new_constraint_count;
        *type_batch.index_to_handle.get_mut(index_in_type_batch) = handle;

        // Recalculate ptr after potential resize
        let body_references_ptr = type_batch.body_references.as_mut_ptr();
        let bundle_ptr = body_references_ptr.add(bundle_count * body_references_bundle_size);
        add_body_references_lane(bundle_ptr, 0, encoded_body_indices, bodies_per_constraint);

        // Clear the slot's accumulated impulse.
        let impulse_ptr = type_batch
            .accumulated_impulses
            .as_mut_ptr()
            .add(bundle_count * accumulated_impulse_bundle_size);
        clear_lane_raw(impulse_ptr, 0, accumulated_impulse_bundle_size);

        // All unoccupied slots between old count and the new constraint must have IndexToHandle of -1.
        // Batch compression relies on this.
        debug_assert!(index_in_type_batch == type_batch.constraint_count - 1);
        for i in old_count..index_in_type_batch {
            type_batch.index_to_handle.get_mut(i).0 = -1;
        }

        index_in_type_batch
    } else {
        // Found a bundle with room. Clear accumulated impulse and update bookkeeping.
        let impulse_ptr = type_batch
            .accumulated_impulses
            .as_mut_ptr()
            .add(target_bundle_index as usize * accumulated_impulse_bundle_size);
        clear_lane_raw(
            impulse_ptr,
            target_inner_index as usize,
            accumulated_impulse_bundle_size,
        );

        let index_in_type_batch =
            target_bundle_index * Vector::<i32>::LEN as i32 + target_inner_index;
        // If the constraint was added after the highest index currently existing, boost the count.
        type_batch.constraint_count = (index_in_type_batch + 1).max(type_batch.constraint_count);
        debug_assert!(type_batch.index_to_handle.len() >= type_batch.constraint_count);
        *type_batch.index_to_handle.get_mut(index_in_type_batch) = handle;

        index_in_type_batch
    }
}
