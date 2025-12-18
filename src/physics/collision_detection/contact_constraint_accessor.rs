// Translated from BepuPhysics/CollisionDetection/ContactConstraintAccessor.cs

use crate::physics::collision_detection::contact_manifold::{
    ConvexContact, ConvexContactManifold, NonconvexContactManifold,
};
use crate::physics::collision_detection::narrow_phase_callbacks::PairMaterialProperties;
use crate::physics::collision_detection::narrow_phase::{PendingConstraintAddCache, SortConstraintTarget};
use crate::physics::collision_detection::pair_cache::{CollidablePair, ConstraintCache, PairCache};
use crate::physics::collision_detection::untyped_list::UntypedList;
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraint_reference::ConstraintReference;
use crate::physics::handles::{BodyHandle, ConstraintHandle};
use crate::physics::solver::Solver;
use crate::physics::constraints::contact::contact_constraint_description::{
    ConstraintContactData, NonconvexConstraintContactData,
};
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use glam::Vec3;

/// Interface for extracting solver contact data from constraints.
pub trait ISolverContactDataExtractor {
    fn convex_one_body(
        &mut self,
        body_handle: BodyHandle,
        prestep: *const u8,
        impulses: *const u8,
    );
    fn convex_two_body(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        prestep: *const u8,
        impulses: *const u8,
    );
    fn nonconvex_one_body(
        &mut self,
        body_handle: BodyHandle,
        prestep: *const u8,
        impulses: *const u8,
    );
    fn nonconvex_two_body(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        prestep: *const u8,
        impulses: *const u8,
    );
}

/// Interface for extracting just prestep and impulse data.
pub trait ISolverContactPrestepAndImpulsesExtractor {
    fn convex_one_body(&mut self, prestep: *const u8, impulses: *const u8);
    fn convex_two_body(&mut self, prestep: *const u8, impulses: *const u8);
    fn nonconvex_one_body(&mut self, prestep: *const u8, impulses: *const u8);
    fn nonconvex_two_body(&mut self, prestep: *const u8, impulses: *const u8);
}

/// Provides indirection for reading from and updating constraints in the narrow phase.
/// This replaces the abstract class in C# with a trait object approach.
pub trait ContactConstraintAccessor: Send + Sync {
    /// The constraint type ID associated with this accessor.
    fn constraint_type_id(&self) -> i32;

    /// Number of contacts this accessor handles.
    fn contact_count(&self) -> i32;

    /// Whether the constraint type is convex.
    fn is_convex(&self) -> bool;

    /// The stride in bytes for accumulated impulse bundles.
    fn accumulated_impulse_bundle_stride_in_bytes(&self) -> i32;

    /// Gathers old impulses from a constraint reference.
    unsafe fn gather_old_impulses(
        &self,
        constraint_reference: &ConstraintReference,
        old_impulses: *mut f32,
    );

    /// Scatters new impulses into a constraint reference.
    unsafe fn scatter_new_impulses(
        &self,
        constraint_reference: &ConstraintReference,
        contact_impulses: *const f32,
    );

    /// Extracts contact data from a constraint.
    fn extract_contact_data(
        &self,
        constraint_location: &ConstraintLocation,
        solver: &Solver,
        extractor: &mut dyn ISolverContactDataExtractor,
    );

    /// Extracts prestep and impulse data from a constraint.
    fn extract_contact_prestep_and_impulses(
        &self,
        constraint_location: &ConstraintLocation,
        solver: &Solver,
        extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
    );

    /// Flushes pending constraints sequentially.
    fn flush_sequentially(
        &self,
        list: &mut UntypedList,
        narrow_phase_constraint_type_id: i32,
        narrow_phase: *const super::narrow_phase::NarrowPhase,
        solver: &mut Solver,
        pair_cache: &mut PairCache,
    );

    /// Flushes pending constraints using speculative batch indices.
    fn flush_with_speculative_batches(
        &self,
        list: &mut UntypedList,
        narrow_phase_constraint_type_id: i32,
        speculative_batch_indices: &mut Buffer<Buffer<u16>>,
        narrow_phase: *const super::narrow_phase::NarrowPhase,
        solver: &mut Solver,
        pair_cache: &mut PairCache,
    );

    /// Deterministically adds constraints from sorted targets across all workers.
    fn deterministically_add(
        &self,
        type_index: i32,
        overlap_workers_pending: &[*mut PendingConstraintAddCache],
        sorted_constraints: &mut QuickList<SortConstraintTarget>,
        narrow_phase: *const super::narrow_phase::NarrowPhase,
        solver: &mut Solver,
        pair_cache: &mut PairCache,
    );

    /// Type-erased dispatch for updating a constraint from a contact manifold.
    /// The concrete implementation knows the actual types and casts them back from raw pointers.
    /// This is the Rust equivalent of C#'s virtual generic dispatch through
    /// `ContactConstraintAccessor.UpdateConstraintForManifold<TManifold, TBodyHandles, TCallbacks>`.
    ///
    /// # Safety
    /// - `narrow_phase_ptr` must point to a valid `NarrowPhaseGeneric<TCallbacks>`.
    /// - `manifold_ptr` must point to a valid manifold of the appropriate type for this accessor.
    /// - `body_handles_ptr` must point to valid body handles (TwoBodyHandles or single i32).
    unsafe fn update_constraint_for_manifold_raw(
        &self,
        narrow_phase_ptr: *mut u8,
        manifold_type_as_constraint_type: i32,
        worker_index: i32,
        pair: &CollidablePair,
        manifold_ptr: *mut u8,
        material: &PairMaterialProperties,
        body_handles_ptr: *const u8,
    );
}

/// Helper functions for copying contact data from manifolds into constraint descriptions.
pub struct ContactDataCopier;

impl ContactDataCopier {
    /// Copies convex contact data from a manifold into constraint contact data and cache.
    pub unsafe fn copy_convex_contact_data(
        manifold: &ConvexContactManifold,
        constraint_cache: &mut ConstraintCache,
        target_contacts: *mut ConstraintContactData,
    ) {
        let contact_count = manifold.count;
        for i in 0..contact_count as isize {
            let source_contact = &*(&manifold.contact0 as *const ConvexContact).offset(i);
            let target_contact = &mut *target_contacts.offset(i);
            *(&mut constraint_cache.feature_id0 as *mut i32).offset(i) = source_contact.feature_id;
            target_contact.offset_a = source_contact.offset;
            target_contact.penetration_depth = source_contact.depth;
        }
    }

    /// Copies nonconvex contact data from a manifold into nonconvex constraint contact data and cache.
    pub unsafe fn copy_nonconvex_contact_data(
        manifold: &NonconvexContactManifold,
        constraint_cache: &mut ConstraintCache,
        target_contacts: *mut NonconvexConstraintContactData,
    ) {
        let contact_count = manifold.count;
        for i in 0..contact_count as isize {
            let source_contact = &*(&manifold.contact0 as *const crate::physics::collision_detection::contact_manifold::Contact).offset(i);
            let target_contact = &mut *target_contacts.offset(i);
            *(&mut constraint_cache.feature_id0 as *mut i32).offset(i) = source_contact.feature_id;
            target_contact.offset_a = source_contact.offset;
            target_contact.normal = source_contact.normal;
            target_contact.penetration_depth = source_contact.depth;
        }
    }

    /// Copies a single-contact nonconvex manifold into convex constraint contact data and cache.
    /// Used when a nonconvex manifold has exactly 1 contact and is treated as convex.
    pub unsafe fn copy_nonconvex_to_convex_contact_data(
        manifold: &NonconvexContactManifold,
        constraint_cache: &mut ConstraintCache,
        target_contacts: *mut ConstraintContactData,
    ) {
        constraint_cache.feature_id0 = manifold.contact0.feature_id;
        let target = &mut *target_contacts;
        target.offset_a = manifold.contact0.offset;
        target.penetration_depth = manifold.contact0.depth;
    }
}

// =============================================================================
// Common base logic shared by all concrete accessors.
// In C# this was the abstract ContactConstraintAccessor base class.
// =============================================================================

/// Common fields for all contact constraint accessors.
struct AccessorBase {
    constraint_type_id: i32,
    accumulated_impulse_bundle_stride_in_bytes: i32,
    contact_count: i32,
    convex: bool,
}

impl AccessorBase {
    /// Constructs the base with computed fields from type sizes.
    fn new(constraint_type_id: i32, contact_count: i32, convex: bool, accumulated_impulse_size: usize) -> Self {
        Self {
            constraint_type_id,
            accumulated_impulse_bundle_stride_in_bytes: accumulated_impulse_size as i32,
            contact_count,
            convex,
        }
    }

    /// Gathers old impulses from a constraint reference.
    /// Convex: reads penetration impulses starting after the tangent friction (Vector2Wide offset).
    /// Nonconvex: reads penetration impulses from NonconvexAccumulatedImpulses layout.
    unsafe fn gather_old_impulses(
        &self,
        constraint_reference: &ConstraintReference,
        old_impulses: *mut f32,
    ) {
        use crate::utilities::bundle_indexing::BundleIndexing;

        let mut bundle_index = 0usize;
        let mut inner = 0usize;
        BundleIndexing::get_bundle_indices(
            constraint_reference.index_in_type_batch as usize,
            &mut bundle_index,
            &mut inner,
        );

        let buffer_ptr = (*constraint_reference.type_batch_pointer).accumulated_impulses.as_ptr();
        let stride = self.accumulated_impulse_bundle_stride_in_bytes as usize;
        // Vector<f32> is VECTOR_LEN contiguous f32s.
        let vector_size = std::mem::size_of::<f32>() * crate::utilities::vector::optimal_lanes::<f32>();

        if self.convex {
            // Convex layout: [tangent: Vector2Wide (2 vectors)] [penetration0: Vector<f32>] [penetration1..] [twist: Vector<f32>]
            // Skip Vector2Wide (2 * vector_size) to reach penetration impulses.
            let vector2wide_size = 2 * vector_size;
            let base_byte_ptr = buffer_ptr.add(stride * bundle_index + vector2wide_size);
            for i in 0..self.contact_count as usize {
                // Each penetration vector is vector_size bytes. Lane `inner` is at offset `inner * sizeof(f32)`.
                let f32_ptr = base_byte_ptr.add(i * vector_size + inner * std::mem::size_of::<f32>()) as *const f32;
                *old_impulses.add(i) = *f32_ptr;
            }
        } else {
            // Nonconvex layout: each contact has [tangent: Vector2Wide (2 vectors)] [penetration: Vector<f32>]
            // = 3 * vector_size per contact.
            let nonconvex_impulse_stride = 3 * vector_size;
            let base_byte_ptr = buffer_ptr.add(stride * bundle_index);
            for i in 0..self.contact_count as usize {
                // Penetration is at offset 2 * vector_size within each NonconvexAccumulatedImpulses
                let f32_ptr = base_byte_ptr.add(i * nonconvex_impulse_stride + 2 * vector_size + inner * std::mem::size_of::<f32>()) as *const f32;
                *old_impulses.add(i) = *f32_ptr;
            }
        }
    }

    /// Scatters new impulses into a constraint reference.
    unsafe fn scatter_new_impulses(
        &self,
        constraint_reference: &ConstraintReference,
        contact_impulses: *const f32,
    ) {
        use crate::utilities::bundle_indexing::BundleIndexing;

        let mut bundle_index = 0usize;
        let mut inner = 0usize;
        BundleIndexing::get_bundle_indices(
            constraint_reference.index_in_type_batch as usize,
            &mut bundle_index,
            &mut inner,
        );

        let buffer_ptr = (*constraint_reference.type_batch_pointer).accumulated_impulses.as_mut_ptr();
        let stride = self.accumulated_impulse_bundle_stride_in_bytes as usize;
        let vector_size = std::mem::size_of::<f32>() * crate::utilities::vector::optimal_lanes::<f32>();

        debug_assert!((*constraint_reference.type_batch_pointer).type_id == self.constraint_type_id);

        if self.convex {
            let vector2wide_size = 2 * vector_size;
            let base_byte_ptr = buffer_ptr.add(stride * bundle_index + vector2wide_size);
            for i in 0..self.contact_count as usize {
                let f32_ptr = base_byte_ptr.add(i * vector_size + inner * std::mem::size_of::<f32>()) as *mut f32;
                *f32_ptr = *contact_impulses.add(i);
            }
        } else {
            let nonconvex_impulse_stride = 3 * vector_size;
            let base_byte_ptr = buffer_ptr.add(stride * bundle_index);
            for i in 0..self.contact_count as usize {
                let f32_ptr = base_byte_ptr.add(i * nonconvex_impulse_stride + 2 * vector_size + inner * std::mem::size_of::<f32>()) as *mut f32;
                *f32_ptr = *contact_impulses.add(i);
            }
        }
    }
}

// =============================================================================
// Extract helper functions for contact data and prestep/impulse extraction.
// These are called by the macros below to avoid code duplication.
// =============================================================================

/// Extracts contact data for a one-body constraint (convex or nonconvex).
unsafe fn extract_one_body_contact_data<TPrestep: Copy, TImpulses: Copy>(
    constraint_location: &ConstraintLocation,
    solver: &Solver,
    extractor: &mut dyn ISolverContactDataExtractor,
    convex: bool,
) {
    use crate::utilities::bundle_indexing::BundleIndexing;
    use crate::utilities::gather_scatter::GatherScatter;
    use crate::utilities::vector::Vector;
    use crate::physics::bodies::Bodies;

    let set = &solver.sets[constraint_location.set_index as usize];
    let batch = &set.batches[constraint_location.batch_index];
    let type_batch_index = batch.type_index_to_type_batch_index[constraint_location.type_id as usize];
    let type_batch = &batch.type_batches[type_batch_index];

    let mut bundle_index = 0usize;
    let mut inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        constraint_location.index_in_type_batch as usize,
        &mut bundle_index,
        &mut inner_index,
    );

    // One-body: body references are Vector<i32> bundles
    let body_ref_ptr = type_batch.body_references.as_ptr() as *const Vector<i32>;
    let body_reference = (*body_ref_ptr.add(bundle_index)).as_array()[inner_index];
    let body_handle = if constraint_location.set_index == 0 {
        let bodies = &*solver.bodies;
        bodies.active_set().index_to_handle[body_reference as usize]
    } else {
        BodyHandle(body_reference)
    };

    let prestep_ptr = (type_batch.prestep_data.as_ptr() as *const TPrestep).add(bundle_index);
    let prestep = GatherScatter::get_offset_instance(&*prestep_ptr, inner_index);
    let impulses_ptr = (type_batch.accumulated_impulses.as_ptr() as *const TImpulses).add(bundle_index);
    let impulses = GatherScatter::get_offset_instance(&*impulses_ptr, inner_index);

    if convex {
        extractor.convex_one_body(
            body_handle,
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    } else {
        extractor.nonconvex_one_body(
            body_handle,
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    }
}

/// Extracts contact data for a two-body constraint (convex or nonconvex).
unsafe fn extract_two_body_contact_data<TPrestep: Copy, TImpulses: Copy>(
    constraint_location: &ConstraintLocation,
    solver: &Solver,
    extractor: &mut dyn ISolverContactDataExtractor,
    convex: bool,
) {
    use crate::utilities::bundle_indexing::BundleIndexing;
    use crate::utilities::gather_scatter::GatherScatter;
    use crate::physics::bodies::Bodies;
    use crate::physics::constraints::body_references::TwoBodyReferences;

    let set = &solver.sets[constraint_location.set_index as usize];
    let batch = &set.batches[constraint_location.batch_index];
    let type_batch_index = batch.type_index_to_type_batch_index[constraint_location.type_id as usize];
    let type_batch = &batch.type_batches[type_batch_index];

    let mut bundle_index = 0usize;
    let mut inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        constraint_location.index_in_type_batch as usize,
        &mut bundle_index,
        &mut inner_index,
    );

    let prestep_ptr = (type_batch.prestep_data.as_ptr() as *const TPrestep).add(bundle_index);
    let prestep = GatherScatter::get_offset_instance(&*prestep_ptr, inner_index);
    let impulses_ptr = (type_batch.accumulated_impulses.as_ptr() as *const TImpulses).add(bundle_index);
    let impulses = GatherScatter::get_offset_instance(&*impulses_ptr, inner_index);

    // Two-body: body references are TwoBodyReferences bundles
    let body_refs_ptr = (type_batch.body_references.as_ptr() as *const TwoBodyReferences).add(bundle_index);
    let body_refs = GatherScatter::get_offset_instance(&*body_refs_ptr, inner_index);
    let index_a = body_refs.index_a.as_array()[0] & Bodies::BODY_REFERENCE_MASK;
    let index_b = body_refs.index_b.as_array()[0] & Bodies::BODY_REFERENCE_MASK;

    let (body_handle_a, body_handle_b) = if constraint_location.set_index == 0 {
        let bodies = &*solver.bodies;
        (
            bodies.active_set().index_to_handle[index_a as usize],
            bodies.active_set().index_to_handle[index_b as usize],
        )
    } else {
        (BodyHandle(index_a), BodyHandle(index_b))
    };

    if convex {
        extractor.convex_two_body(
            body_handle_a,
            body_handle_b,
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    } else {
        extractor.nonconvex_two_body(
            body_handle_a,
            body_handle_b,
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    }
}

/// Extracts prestep and impulse data for a one-body constraint.
unsafe fn extract_one_body_prestep_and_impulses<TPrestep: Copy, TImpulses: Copy>(
    constraint_location: &ConstraintLocation,
    solver: &Solver,
    extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
    convex: bool,
) {
    use crate::utilities::bundle_indexing::BundleIndexing;
    use crate::utilities::gather_scatter::GatherScatter;

    let set = &solver.sets[constraint_location.set_index as usize];
    let batch = &set.batches[constraint_location.batch_index];
    let type_batch_index = batch.type_index_to_type_batch_index[constraint_location.type_id as usize];
    let type_batch = &batch.type_batches[type_batch_index];

    let mut bundle_index = 0usize;
    let mut inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        constraint_location.index_in_type_batch as usize,
        &mut bundle_index,
        &mut inner_index,
    );

    let prestep_ptr = (type_batch.prestep_data.as_ptr() as *const TPrestep).add(bundle_index);
    let prestep = GatherScatter::get_offset_instance(&*prestep_ptr, inner_index);
    let impulses_ptr = (type_batch.accumulated_impulses.as_ptr() as *const TImpulses).add(bundle_index);
    let impulses = GatherScatter::get_offset_instance(&*impulses_ptr, inner_index);

    if convex {
        extractor.convex_one_body(
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    } else {
        extractor.nonconvex_one_body(
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    }
}

/// Extracts prestep and impulse data for a two-body constraint.
unsafe fn extract_two_body_prestep_and_impulses<TPrestep: Copy, TImpulses: Copy>(
    constraint_location: &ConstraintLocation,
    solver: &Solver,
    extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
    convex: bool,
) {
    use crate::utilities::bundle_indexing::BundleIndexing;
    use crate::utilities::gather_scatter::GatherScatter;

    let set = &solver.sets[constraint_location.set_index as usize];
    let batch = &set.batches[constraint_location.batch_index];
    let type_batch_index = batch.type_index_to_type_batch_index[constraint_location.type_id as usize];
    let type_batch = &batch.type_batches[type_batch_index];

    let mut bundle_index = 0usize;
    let mut inner_index = 0usize;
    BundleIndexing::get_bundle_indices(
        constraint_location.index_in_type_batch as usize,
        &mut bundle_index,
        &mut inner_index,
    );

    let prestep_ptr = (type_batch.prestep_data.as_ptr() as *const TPrestep).add(bundle_index);
    let prestep = GatherScatter::get_offset_instance(&*prestep_ptr, inner_index);
    let impulses_ptr = (type_batch.accumulated_impulses.as_ptr() as *const TImpulses).add(bundle_index);
    let impulses = GatherScatter::get_offset_instance(&*impulses_ptr, inner_index);

    if convex {
        extractor.convex_two_body(
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    } else {
        extractor.nonconvex_two_body(
            prestep as *const _ as *const u8,
            impulses as *const _ as *const u8,
        );
    }
}

// =============================================================================
// Concrete accessor types, one per (convex/nonconvex) × (one-body/two-body) × contact count.
// The C# code uses generics; here we use macros to generate concrete structs.
// =============================================================================

use crate::physics::collision_detection::narrow_phase::{
    NarrowPhaseGeneric, TwoBodyHandles,
    ContactImpulses1, ContactImpulses2, ContactImpulses3, ContactImpulses4,
};
use crate::physics::collision_detection::narrow_phase_callbacks::INarrowPhaseCallbacks;
use crate::physics::constraints::constraint_description::IConstraintDescription;
use crate::physics::constraints::contact::contact_convex_descriptions::*;
use crate::physics::constraints::contact::contact_nonconvex_descriptions::*;
use crate::physics::collision_detection::pair_cache::PairCacheChangeIndex;

/// Reads a PendingConstraint from an UntypedList by byte offset.
///
/// PendingConstraint layout: CollidablePair | PairCacheChangeIndex | TBodyHandles | TDescription | TContactImpulses
#[inline(always)]
unsafe fn read_pending_constraint<TBodyHandles: Copy, TDescription: IConstraintDescription + Copy, TContactImpulses: Copy>(
    list: &UntypedList,
    index: i32,
) -> (*mut CollidablePair, PairCacheChangeIndex, *const TBodyHandles, *const TDescription, *const TContactImpulses) {
    let element_size = std::mem::size_of::<CollidablePair>()
        + std::mem::size_of::<PairCacheChangeIndex>()
        + std::mem::size_of::<TBodyHandles>()
        + std::mem::size_of::<TDescription>()
        + std::mem::size_of::<TContactImpulses>();
    let base = list.buffer.as_ptr().add(index as usize * element_size);
    let mut offset = 0usize;

    let pair = base.add(offset) as *mut CollidablePair;
    offset += std::mem::size_of::<CollidablePair>();

    let change = std::ptr::read_unaligned(base.add(offset) as *const PairCacheChangeIndex);
    offset += std::mem::size_of::<PairCacheChangeIndex>();

    let body_handles = base.add(offset) as *const TBodyHandles;
    offset += std::mem::size_of::<TBodyHandles>();

    let description = base.add(offset) as *const TDescription;
    offset += std::mem::size_of::<TDescription>();

    let impulses = base.add(offset) as *const TContactImpulses;

    (pair, change, body_handles, description, impulses)
}

/// Sequential add to simulation — iterates pending constraints and adds them one by one.
unsafe fn sequential_add_to_simulation<TBodyHandles: Copy, TDescription: IConstraintDescription + Copy, TContactImpulses: Copy>(
    list: &mut UntypedList,
    narrow_phase: *const super::narrow_phase::NarrowPhase,
    solver: &mut Solver,
    pair_cache: &mut PairCache,
    body_count: i32,
) {
    if !list.buffer.allocated() {
        return;
    }
    let solver_ptr = solver as *mut Solver;
    for i in 0..list.count as i32 {
        let (pair, change, body_handles_ptr, description_ptr, impulses_ptr) =
            read_pending_constraint::<TBodyHandles, TDescription, TContactImpulses>(list, i);

        // Create a BodyHandle slice from the body handles pointer.
        let body_handle_slice = std::slice::from_raw_parts(
            body_handles_ptr as *const BodyHandle,
            body_count as usize,
        );
        let desc = &*description_ptr;
        let type_id = TDescription::constraint_type_id();
        let handle = (*solver_ptr).add(body_handle_slice, type_id, |batch, bi, ii| {
            desc.apply_description(batch, bi, ii);
        });
        pair_cache.complete_constraint_add(
            narrow_phase,
            solver_ptr,
            impulses_ptr as *const u8,
            change,
            handle,
            &*pair,
        );
    }
}

/// Speculative batch add — uses precomputed batch indices for faster constraint batch allocation.
unsafe fn add_to_simulation_speculative<TBodyHandles: Copy, TDescription: IConstraintDescription + Copy, TContactImpulses: Copy>(
    narrow_phase: *const super::narrow_phase::NarrowPhase,
    solver: &mut Solver,
    pair_cache: &mut PairCache,
    pair: &mut CollidablePair,
    change: PairCacheChangeIndex,
    body_handles_ptr: *const TBodyHandles,
    description_ptr: *const TDescription,
    impulses_ptr: *const TContactImpulses,
    batch_index: i32,
    body_count: i32,
) {
    let solver_ptr = solver as *mut Solver;
    let body_handle_slice = std::slice::from_raw_parts(
        body_handles_ptr as *const BodyHandle,
        body_count as usize,
    );

    let mut encoded_body_indices = vec![0i32; body_count as usize];
    let blocking = (*solver_ptr).get_blocking_body_handles(body_handle_slice, &mut encoded_body_indices);
    let type_id = TDescription::constraint_type_id();

    let mut target_batch = batch_index;
    let (constraint_handle, reference) = loop {
        if let Some(result) = (*solver_ptr).try_allocate_in_batch(
            type_id,
            target_batch,
            &blocking,
            &encoded_body_indices,
        ) {
            break result;
        }
        target_batch += 1;
    };

    let desc = &*description_ptr;
    (*solver_ptr).apply_description_without_waking(&reference, |batch, bi, ii| {
        desc.apply_description(batch, bi, ii);
    });

    // Register the constraint with each connected body.
    let bodies = &mut *(*solver_ptr).bodies;
    for (idx, &bh) in body_handle_slice.iter().enumerate() {
        let body_index = bodies.handle_to_location.get(bh.0).index;
        bodies.add_constraint(body_index, constraint_handle, idx as i32);
    }

    pair_cache.complete_constraint_add(
        narrow_phase,
        solver_ptr,
        impulses_ptr as *const u8,
        change,
        constraint_handle,
        pair,
    );
}

/// Sequential add with speculative batch indices.
unsafe fn sequential_add_to_simulation_speculative<TBodyHandles: Copy, TDescription: IConstraintDescription + Copy, TContactImpulses: Copy>(
    list: &mut UntypedList,
    narrow_phase_constraint_type_id: i32,
    speculative_batch_indices: &mut Buffer<Buffer<u16>>,
    narrow_phase: *const super::narrow_phase::NarrowPhase,
    solver: &mut Solver,
    pair_cache: &mut PairCache,
    body_count: i32,
) {
    if !list.buffer.allocated() {
        return;
    }
    let batch_indices_for_type = &*speculative_batch_indices.get(narrow_phase_constraint_type_id);
    for i in 0..list.count as i32 {
        let (pair, change, body_handles_ptr, description_ptr, impulses_ptr) =
            read_pending_constraint::<TBodyHandles, TDescription, TContactImpulses>(list, i);
        let batch_index = *batch_indices_for_type.get(i) as i32;
        add_to_simulation_speculative::<TBodyHandles, TDescription, TContactImpulses>(
            narrow_phase, solver, pair_cache, &mut *pair, change, body_handles_ptr, description_ptr, impulses_ptr, batch_index, body_count,
        );
    }
}

/// Deterministic add — iterate sorted constraint targets from all workers and add with speculative batches.
unsafe fn deterministic_add_all<TBodyHandles: Copy, TDescription: IConstraintDescription + Copy, TContactImpulses: Copy>(
    type_index: i32,
    overlap_workers_pending: &[*mut PendingConstraintAddCache],
    sorted_constraints: &mut QuickList<SortConstraintTarget>,
    narrow_phase: *const super::narrow_phase::NarrowPhase,
    solver: &mut Solver,
    pair_cache: &mut PairCache,
    body_count: i32,
) {
    let element_size = std::mem::size_of::<CollidablePair>()
        + std::mem::size_of::<PairCacheChangeIndex>()
        + std::mem::size_of::<TBodyHandles>()
        + std::mem::size_of::<TDescription>()
        + std::mem::size_of::<TContactImpulses>();
    for i in 0..sorted_constraints.count {
        let target = sorted_constraints.get(i);
        let worker_cache = &*overlap_workers_pending[target.worker_index as usize];
        let list = &*worker_cache.pending_constraints_by_type.get(type_index);
        let base = list.buffer.as_ptr().add(target.byte_index_in_cache as usize);
        let mut offset = 0usize;

        let pair = base.add(offset) as *mut CollidablePair;
        offset += std::mem::size_of::<CollidablePair>();

        let change = std::ptr::read_unaligned(base.add(offset) as *const PairCacheChangeIndex);
        offset += std::mem::size_of::<PairCacheChangeIndex>();

        let body_handles_ptr = base.add(offset) as *const TBodyHandles;
        offset += std::mem::size_of::<TBodyHandles>();

        let description_ptr = base.add(offset) as *const TDescription;
        offset += std::mem::size_of::<TDescription>();

        let impulses_ptr = base.add(offset) as *const TContactImpulses;

        let index = target.byte_index_in_cache as usize / element_size;
        let batch_index = *worker_cache.speculative_batch_indices.get(type_index).get(index as i32) as i32;
        add_to_simulation_speculative::<TBodyHandles, TDescription, TContactImpulses>(
            narrow_phase, solver, pair_cache, &mut *pair, change, body_handles_ptr, description_ptr, impulses_ptr, batch_index, body_count,
        );
    }
}

/// Generates a ConvexOneBodyAccessor for a given contact count.
///
/// ConvexOneBody accessors handle both ConvexContactManifold and NonconvexContactManifold
/// (the latter only when contact count == 1, treating a single nonconvex contact as convex).
macro_rules! impl_convex_one_body_accessor {
    ($name:ident, $desc:ty, $prestep:ty, $impulse:ty, $contact_impulses:ty, $contact_count:expr) => {
        pub struct $name;

        impl $name {
            pub fn new() -> Self {
                Self
            }
        }

        impl ContactConstraintAccessor for $name {
            fn constraint_type_id(&self) -> i32 {
                <$desc>::constraint_type_id()
            }

            fn contact_count(&self) -> i32 {
                $contact_count
            }

            fn is_convex(&self) -> bool {
                true
            }

            fn accumulated_impulse_bundle_stride_in_bytes(&self) -> i32 {
                std::mem::size_of::<$impulse>() as i32
            }

            unsafe fn gather_old_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                old_impulses: *mut f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    true,
                    std::mem::size_of::<$impulse>(),
                );
                base.gather_old_impulses(constraint_reference, old_impulses);
            }

            unsafe fn scatter_new_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                contact_impulses: *const f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    true,
                    std::mem::size_of::<$impulse>(),
                );
                base.scatter_new_impulses(constraint_reference, contact_impulses);
            }

            fn extract_contact_data(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactDataExtractor,
            ) {
                unsafe { extract_one_body_contact_data::<$prestep, $impulse>(constraint_location, solver, extractor, true); }
            }

            fn extract_contact_prestep_and_impulses(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
            ) {
                unsafe { extract_one_body_prestep_and_impulses::<$prestep, $impulse>(constraint_location, solver, extractor, true); }
            }

            fn flush_sequentially(
                &self,
                list: &mut UntypedList,
                _narrow_phase_constraint_type_id: i32,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation::<i32, $desc, $contact_impulses>(list, narrow_phase, solver, pair_cache, 1);
                }
            }

            fn flush_with_speculative_batches(
                &self,
                list: &mut UntypedList,
                narrow_phase_constraint_type_id: i32,
                speculative_batch_indices: &mut Buffer<Buffer<u16>>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation_speculative::<i32, $desc, $contact_impulses>(
                        list, narrow_phase_constraint_type_id, speculative_batch_indices, narrow_phase, solver, pair_cache, 1,
                    );
                }
            }

            fn deterministically_add(
                &self,
                type_index: i32,
                overlap_workers_pending: &[*mut PendingConstraintAddCache],
                sorted_constraints: &mut QuickList<SortConstraintTarget>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    deterministic_add_all::<i32, $desc, $contact_impulses>(
                        type_index, overlap_workers_pending, sorted_constraints, narrow_phase, solver, pair_cache, 1,
                    );
                }
            }

            unsafe fn update_constraint_for_manifold_raw(
                &self,
                narrow_phase_ptr: *mut u8,
                manifold_type_as_constraint_type: i32,
                worker_index: i32,
                pair: &CollidablePair,
                manifold_ptr: *mut u8,
                material: &PairMaterialProperties,
                body_handles_ptr: *const u8,
            ) {
                let body_handle = *(body_handles_ptr as *const i32);

                // Check if the manifold is convex or nonconvex.
                // Type IDs 0-7 are convex, 8+ are nonconvex.
                // The concrete accessor knows the description type; the manifold type is determined
                // by the caller's generic parameter. Since we erase that, we use the type ID ranges.
                if manifold_type_as_constraint_type < 8 {
                    // ConvexContactManifold case
                    let manifold = &*(manifold_ptr as *const ConvexContactManifold);
                    let mut constraint_cache = ConstraintCache::default();
                    let mut description: $desc = std::mem::zeroed();
                    ContactDataCopier::copy_convex_contact_data(
                        manifold,
                        &mut constraint_cache,
                        &mut description as *mut $desc as *mut ConstraintContactData,
                    );
                    description.copy_manifold_wide_properties(&manifold.normal, material);
                    let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                    narrow_phase.update_constraint_erased::<i32, $desc, $contact_impulses>(
                        worker_index,
                        *pair,
                        manifold_type_as_constraint_type,
                        &mut constraint_cache,
                        manifold.count,
                        &description,
                        body_handle,
                    );
                } else {
                    // NonconvexContactManifold case (1 contact treated as convex)
                    let manifold = &*(manifold_ptr as *const NonconvexContactManifold);
                    debug_assert!(manifold.count == 1, "Nonconvex manifolds should only result in convex constraints when the contact count is 1.");
                    let mut constraint_cache: ConstraintCache = std::mem::zeroed();
                    let mut description: $desc = std::mem::zeroed();
                    ContactDataCopier::copy_nonconvex_to_convex_contact_data(
                        manifold,
                        &mut constraint_cache,
                        description.get_first_contact_mut() as *mut ConstraintContactData,
                    );
                    description.copy_manifold_wide_properties(&manifold.contact0.normal, material);
                    let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                    narrow_phase.update_constraint_erased::<i32, $desc, $contact_impulses>(
                        worker_index,
                        *pair,
                        manifold_type_as_constraint_type,
                        &mut constraint_cache,
                        manifold.count,
                        &description,
                        body_handle,
                    );
                }
            }
        }
    };
}

/// Generates a ConvexTwoBodyAccessor for a given contact count.
macro_rules! impl_convex_two_body_accessor {
    ($name:ident, $desc:ty, $prestep:ty, $impulse:ty, $contact_impulses:ty, $contact_count:expr) => {
        pub struct $name;

        impl $name {
            pub fn new() -> Self {
                Self
            }
        }

        impl ContactConstraintAccessor for $name {
            fn constraint_type_id(&self) -> i32 {
                <$desc>::constraint_type_id()
            }

            fn contact_count(&self) -> i32 {
                $contact_count
            }

            fn is_convex(&self) -> bool {
                true
            }

            fn accumulated_impulse_bundle_stride_in_bytes(&self) -> i32 {
                std::mem::size_of::<$impulse>() as i32
            }

            unsafe fn gather_old_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                old_impulses: *mut f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    true,
                    std::mem::size_of::<$impulse>(),
                );
                base.gather_old_impulses(constraint_reference, old_impulses);
            }

            unsafe fn scatter_new_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                contact_impulses: *const f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    true,
                    std::mem::size_of::<$impulse>(),
                );
                base.scatter_new_impulses(constraint_reference, contact_impulses);
            }

            fn extract_contact_data(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactDataExtractor,
            ) {
                unsafe { extract_two_body_contact_data::<$prestep, $impulse>(constraint_location, solver, extractor, true); }
            }

            fn extract_contact_prestep_and_impulses(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
            ) {
                unsafe { extract_two_body_prestep_and_impulses::<$prestep, $impulse>(constraint_location, solver, extractor, true); }
            }

            fn flush_sequentially(
                &self,
                list: &mut UntypedList,
                _narrow_phase_constraint_type_id: i32,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation::<TwoBodyHandles, $desc, $contact_impulses>(list, narrow_phase, solver, pair_cache, 2);
                }
            }

            fn flush_with_speculative_batches(
                &self,
                list: &mut UntypedList,
                narrow_phase_constraint_type_id: i32,
                speculative_batch_indices: &mut Buffer<Buffer<u16>>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation_speculative::<TwoBodyHandles, $desc, $contact_impulses>(
                        list, narrow_phase_constraint_type_id, speculative_batch_indices, narrow_phase, solver, pair_cache, 2,
                    );
                }
            }

            fn deterministically_add(
                &self,
                type_index: i32,
                overlap_workers_pending: &[*mut PendingConstraintAddCache],
                sorted_constraints: &mut QuickList<SortConstraintTarget>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    deterministic_add_all::<TwoBodyHandles, $desc, $contact_impulses>(
                        type_index, overlap_workers_pending, sorted_constraints, narrow_phase, solver, pair_cache, 2,
                    );
                }
            }

            unsafe fn update_constraint_for_manifold_raw(
                &self,
                narrow_phase_ptr: *mut u8,
                manifold_type_as_constraint_type: i32,
                worker_index: i32,
                pair: &CollidablePair,
                manifold_ptr: *mut u8,
                material: &PairMaterialProperties,
                body_handles_ptr: *const u8,
            ) {
                let body_handles = *(body_handles_ptr as *const TwoBodyHandles);

                if manifold_type_as_constraint_type < 8 {
                    // ConvexContactManifold case
                    let manifold = &*(manifold_ptr as *const ConvexContactManifold);
                    let mut constraint_cache = ConstraintCache::default();
                    let mut description: $desc = std::mem::zeroed();
                    ContactDataCopier::copy_convex_contact_data(
                        manifold,
                        &mut constraint_cache,
                        &mut description as *mut $desc as *mut ConstraintContactData,
                    );
                    description.copy_manifold_wide_properties(&manifold.offset_b, &manifold.normal, material);
                    let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                    narrow_phase.update_constraint_erased::<TwoBodyHandles, $desc, $contact_impulses>(
                        worker_index,
                        *pair,
                        manifold_type_as_constraint_type,
                        &mut constraint_cache,
                        manifold.count,
                        &description,
                        body_handles,
                    );
                } else {
                    // NonconvexContactManifold case (1 contact treated as convex)
                    let manifold = &*(manifold_ptr as *const NonconvexContactManifold);
                    debug_assert!(manifold.count == 1, "Nonconvex manifolds should only result in convex constraints when the contact count is 1.");
                    let mut constraint_cache: ConstraintCache = std::mem::zeroed();
                    let mut description: $desc = std::mem::zeroed();
                    ContactDataCopier::copy_nonconvex_to_convex_contact_data(
                        manifold,
                        &mut constraint_cache,
                        description.get_first_contact_mut() as *mut ConstraintContactData,
                    );
                    description.copy_manifold_wide_properties(&manifold.offset_b, &manifold.contact0.normal, material);
                    let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                    narrow_phase.update_constraint_erased::<TwoBodyHandles, $desc, $contact_impulses>(
                        worker_index,
                        *pair,
                        manifold_type_as_constraint_type,
                        &mut constraint_cache,
                        manifold.count,
                        &description,
                        body_handles,
                    );
                }
            }
        }
    };
}

/// Generates a NonconvexOneBodyAccessor for a given contact count.
macro_rules! impl_nonconvex_one_body_accessor {
    ($name:ident, $desc:ty, $prestep:ty, $impulse:ty, $contact_impulses:ty, $contact_count:expr) => {
        pub struct $name;

        impl $name {
            pub fn new() -> Self {
                Self
            }
        }

        impl ContactConstraintAccessor for $name {
            fn constraint_type_id(&self) -> i32 {
                <$desc>::constraint_type_id()
            }

            fn contact_count(&self) -> i32 {
                $contact_count
            }

            fn is_convex(&self) -> bool {
                false
            }

            fn accumulated_impulse_bundle_stride_in_bytes(&self) -> i32 {
                std::mem::size_of::<$impulse>() as i32
            }

            unsafe fn gather_old_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                old_impulses: *mut f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    false,
                    std::mem::size_of::<$impulse>(),
                );
                base.gather_old_impulses(constraint_reference, old_impulses);
            }

            unsafe fn scatter_new_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                contact_impulses: *const f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    false,
                    std::mem::size_of::<$impulse>(),
                );
                base.scatter_new_impulses(constraint_reference, contact_impulses);
            }

            fn extract_contact_data(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactDataExtractor,
            ) {
                unsafe { extract_one_body_contact_data::<$prestep, $impulse>(constraint_location, solver, extractor, false); }
            }

            fn extract_contact_prestep_and_impulses(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
            ) {
                unsafe { extract_one_body_prestep_and_impulses::<$prestep, $impulse>(constraint_location, solver, extractor, false); }
            }

            fn flush_sequentially(
                &self,
                list: &mut UntypedList,
                _narrow_phase_constraint_type_id: i32,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation::<i32, $desc, $contact_impulses>(list, narrow_phase, solver, pair_cache, 1);
                }
            }

            fn flush_with_speculative_batches(
                &self,
                list: &mut UntypedList,
                narrow_phase_constraint_type_id: i32,
                speculative_batch_indices: &mut Buffer<Buffer<u16>>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation_speculative::<i32, $desc, $contact_impulses>(
                        list, narrow_phase_constraint_type_id, speculative_batch_indices, narrow_phase, solver, pair_cache, 1,
                    );
                }
            }

            fn deterministically_add(
                &self,
                type_index: i32,
                overlap_workers_pending: &[*mut PendingConstraintAddCache],
                sorted_constraints: &mut QuickList<SortConstraintTarget>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    deterministic_add_all::<i32, $desc, $contact_impulses>(
                        type_index, overlap_workers_pending, sorted_constraints, narrow_phase, solver, pair_cache, 1,
                    );
                }
            }

            unsafe fn update_constraint_for_manifold_raw(
                &self,
                narrow_phase_ptr: *mut u8,
                manifold_type_as_constraint_type: i32,
                worker_index: i32,
                pair: &CollidablePair,
                manifold_ptr: *mut u8,
                material: &PairMaterialProperties,
                body_handles_ptr: *const u8,
            ) {
                let body_handle = *(body_handles_ptr as *const i32);
                let manifold = &*(manifold_ptr as *const NonconvexContactManifold);
                let mut constraint_cache: ConstraintCache = std::mem::zeroed();
                let mut description: $desc = std::mem::zeroed();
                ContactDataCopier::copy_nonconvex_contact_data(
                    manifold,
                    &mut constraint_cache,
                    description.get_first_contact_mut() as *mut NonconvexConstraintContactData,
                );
                description.copy_manifold_wide_properties(material);
                let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                narrow_phase.update_constraint_erased::<i32, $desc, $contact_impulses>(
                    worker_index,
                    *pair,
                    manifold_type_as_constraint_type,
                    &mut constraint_cache,
                    manifold.count,
                    &description,
                    body_handle,
                );
            }
        }
    };
}

/// Generates a NonconvexTwoBodyAccessor for a given contact count.
macro_rules! impl_nonconvex_two_body_accessor {
    ($name:ident, $desc:ty, $prestep:ty, $impulse:ty, $contact_impulses:ty, $contact_count:expr) => {
        pub struct $name;

        impl $name {
            pub fn new() -> Self {
                Self
            }
        }

        impl ContactConstraintAccessor for $name {
            fn constraint_type_id(&self) -> i32 {
                <$desc>::constraint_type_id()
            }

            fn contact_count(&self) -> i32 {
                $contact_count
            }

            fn is_convex(&self) -> bool {
                false
            }

            fn accumulated_impulse_bundle_stride_in_bytes(&self) -> i32 {
                std::mem::size_of::<$impulse>() as i32
            }

            unsafe fn gather_old_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                old_impulses: *mut f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    false,
                    std::mem::size_of::<$impulse>(),
                );
                base.gather_old_impulses(constraint_reference, old_impulses);
            }

            unsafe fn scatter_new_impulses(
                &self,
                constraint_reference: &ConstraintReference,
                contact_impulses: *const f32,
            ) {
                let base = AccessorBase::new(
                    self.constraint_type_id(),
                    self.contact_count(),
                    false,
                    std::mem::size_of::<$impulse>(),
                );
                base.scatter_new_impulses(constraint_reference, contact_impulses);
            }

            fn extract_contact_data(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactDataExtractor,
            ) {
                unsafe { extract_two_body_contact_data::<$prestep, $impulse>(constraint_location, solver, extractor, false); }
            }

            fn extract_contact_prestep_and_impulses(
                &self,
                constraint_location: &ConstraintLocation,
                solver: &Solver,
                extractor: &mut dyn ISolverContactPrestepAndImpulsesExtractor,
            ) {
                unsafe { extract_two_body_prestep_and_impulses::<$prestep, $impulse>(constraint_location, solver, extractor, false); }
            }

            fn flush_sequentially(
                &self,
                list: &mut UntypedList,
                _narrow_phase_constraint_type_id: i32,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation::<TwoBodyHandles, $desc, $contact_impulses>(list, narrow_phase, solver, pair_cache, 2);
                }
            }

            fn flush_with_speculative_batches(
                &self,
                list: &mut UntypedList,
                narrow_phase_constraint_type_id: i32,
                speculative_batch_indices: &mut Buffer<Buffer<u16>>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    sequential_add_to_simulation_speculative::<TwoBodyHandles, $desc, $contact_impulses>(
                        list, narrow_phase_constraint_type_id, speculative_batch_indices, narrow_phase, solver, pair_cache, 2,
                    );
                }
            }

            fn deterministically_add(
                &self,
                type_index: i32,
                overlap_workers_pending: &[*mut PendingConstraintAddCache],
                sorted_constraints: &mut QuickList<SortConstraintTarget>,
                narrow_phase: *const super::narrow_phase::NarrowPhase,
                solver: &mut Solver,
                pair_cache: &mut PairCache,
            ) {
                unsafe {
                    deterministic_add_all::<TwoBodyHandles, $desc, $contact_impulses>(
                        type_index, overlap_workers_pending, sorted_constraints, narrow_phase, solver, pair_cache, 2,
                    );
                }
            }

            unsafe fn update_constraint_for_manifold_raw(
                &self,
                narrow_phase_ptr: *mut u8,
                manifold_type_as_constraint_type: i32,
                worker_index: i32,
                pair: &CollidablePair,
                manifold_ptr: *mut u8,
                material: &PairMaterialProperties,
                body_handles_ptr: *const u8,
            ) {
                let body_handles = *(body_handles_ptr as *const TwoBodyHandles);
                let manifold = &*(manifold_ptr as *const NonconvexContactManifold);
                let mut constraint_cache: ConstraintCache = std::mem::zeroed();
                let mut description: $desc = std::mem::zeroed();
                ContactDataCopier::copy_nonconvex_contact_data(
                    manifold,
                    &mut constraint_cache,
                    description.get_first_contact_mut() as *mut NonconvexConstraintContactData,
                );
                description.copy_manifold_wide_properties(&manifold.offset_b, material);
                let narrow_phase = &mut *(narrow_phase_ptr as *mut NarrowPhaseGenericOpaque);
                narrow_phase.update_constraint_erased::<TwoBodyHandles, $desc, $contact_impulses>(
                    worker_index,
                    *pair,
                    manifold_type_as_constraint_type,
                    &mut constraint_cache,
                    manifold.count,
                    &description,
                    body_handles,
                );
            }
        }
    };
}

// =============================================================================
// Type-erased NarrowPhaseGeneric wrapper for calling update_constraint.
//
// The concrete accessors need to call `NarrowPhaseGeneric::update_constraint`,
// but through a type-erased pointer (since TCallbacks is unknown at accessor time).
// NarrowPhaseGeneric has identical memory layout regardless of TCallbacks,
// so we use a dummy type for the cast. The update_constraint method only
// accesses fields in the `base: NarrowPhase` portion anyway.
// =============================================================================

/// Opaque stand-in for NarrowPhaseGeneric<AnyCallbacks>.
/// Has identical layout since TCallbacks doesn't affect NarrowPhase base fields
/// used by update_constraint.
#[repr(C)]
struct NarrowPhaseGenericOpaque {
    // We only access through the pointer cast back to the concrete generic type.
    // This struct is never instantiated directly.
    _opaque: [u8; 0],
}

impl NarrowPhaseGenericOpaque {
    /// Calls update_constraint on the narrow phase through the type-erased pointer.
    ///
    /// # Safety
    /// - `self` must actually point to a valid `NarrowPhaseGeneric<T>` for some T.
    /// - The `update_constraint` method only accesses `base: NarrowPhase` fields
    ///   which are layout-invariant across all `T`.
    #[inline(always)]
    unsafe fn update_constraint_erased<
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
        // We need to cast to some concrete NarrowPhaseGeneric<T>.
        // Since update_constraint only uses base NarrowPhase fields,
        // any T works. We use a minimal dummy callbacks type.
        // However, for correct compilation we need to go through the generic function.
        // The actual callback type doesn't matter because update_constraint doesn't
        // invoke callbacks - it just manipulates solver/pair_cache state.
        //
        // We use a helper function pointer approach: the narrow_phase.rs module
        // provides update_constraint which is a method on NarrowPhaseGeneric<T>,
        // but since T is erased here, we call it through a raw pointer offset
        // to the base NarrowPhase and use the base's update_constraint implementation.
        //
        // Actually, update_constraint is defined on NarrowPhaseGeneric<T> but only
        // accesses self.base fields. So we just cast to NarrowPhaseGeneric<DummyCallbacks>
        // and call the method. Since it's monomorphized on TBodyHandles/TDescription/TContactImpulses
        // (not on TCallbacks for the actual constraint update logic), this is safe.
        use crate::physics::collision_detection::narrow_phase::NarrowPhaseUpdateConstraint;
        let np = self as *mut Self as *mut u8;
        NarrowPhaseUpdateConstraint::update_constraint::<TBodyHandles, TDescription, TContactImpulses>(
            np,
            worker_index,
            pair,
            manifold_type_as_constraint_type,
            new_constraint_cache,
            new_contact_count,
            description,
            body_handles,
        );
    }
}

// =============================================================================
// Concrete accessor instantiations
// =============================================================================

use crate::physics::constraints::contact::contact_convex_types::{
    Contact1OneBodyPrestepData, Contact1AccumulatedImpulses,
    Contact2OneBodyPrestepData, Contact2AccumulatedImpulses,
    Contact3OneBodyPrestepData, Contact3AccumulatedImpulses,
    Contact4OneBodyPrestepData, Contact4AccumulatedImpulses,
    Contact1PrestepData, Contact2PrestepData, Contact3PrestepData, Contact4PrestepData,
};
use crate::physics::constraints::contact::contact_nonconvex_types::{
    Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses,
    Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses,
    Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses,
    Contact2NonconvexPrestepData, Contact3NonconvexPrestepData, Contact4NonconvexPrestepData,
};

// Convex one-body (type IDs 0-3)
impl_convex_one_body_accessor!(
    Contact1OneBodyAccessor, Contact1OneBody, Contact1OneBodyPrestepData,
    Contact1AccumulatedImpulses, ContactImpulses1, 1
);
impl_convex_one_body_accessor!(
    Contact2OneBodyAccessor, Contact2OneBody, Contact2OneBodyPrestepData,
    Contact2AccumulatedImpulses, ContactImpulses2, 2
);
impl_convex_one_body_accessor!(
    Contact3OneBodyAccessor, Contact3OneBody, Contact3OneBodyPrestepData,
    Contact3AccumulatedImpulses, ContactImpulses3, 3
);
impl_convex_one_body_accessor!(
    Contact4OneBodyAccessor, Contact4OneBody, Contact4OneBodyPrestepData,
    Contact4AccumulatedImpulses, ContactImpulses4, 4
);

// Convex two-body (type IDs 4-7)
impl_convex_two_body_accessor!(
    Contact1TwoBodyAccessor, Contact1TwoBody, Contact1PrestepData,
    Contact1AccumulatedImpulses, ContactImpulses1, 1
);
impl_convex_two_body_accessor!(
    Contact2TwoBodyAccessor, Contact2TwoBody, Contact2PrestepData,
    Contact2AccumulatedImpulses, ContactImpulses2, 2
);
impl_convex_two_body_accessor!(
    Contact3TwoBodyAccessor, Contact3TwoBody, Contact3PrestepData,
    Contact3AccumulatedImpulses, ContactImpulses3, 3
);
impl_convex_two_body_accessor!(
    Contact4TwoBodyAccessor, Contact4TwoBody, Contact4PrestepData,
    Contact4AccumulatedImpulses, ContactImpulses4, 4
);

// Nonconvex one-body (type IDs 8-10)
impl_nonconvex_one_body_accessor!(
    Contact2NonconvexOneBodyAccessor, Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData,
    Contact2NonconvexAccumulatedImpulses, ContactImpulses2, 2
);
impl_nonconvex_one_body_accessor!(
    Contact3NonconvexOneBodyAccessor, Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData,
    Contact3NonconvexAccumulatedImpulses, ContactImpulses3, 3
);
impl_nonconvex_one_body_accessor!(
    Contact4NonconvexOneBodyAccessor, Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData,
    Contact4NonconvexAccumulatedImpulses, ContactImpulses4, 4
);

// Nonconvex two-body (type IDs 15-17)
impl_nonconvex_two_body_accessor!(
    Contact2NonconvexTwoBodyAccessor, Contact2Nonconvex, Contact2NonconvexPrestepData,
    Contact2NonconvexAccumulatedImpulses, ContactImpulses2, 2
);
impl_nonconvex_two_body_accessor!(
    Contact3NonconvexTwoBodyAccessor, Contact3Nonconvex, Contact3NonconvexPrestepData,
    Contact3NonconvexAccumulatedImpulses, ContactImpulses3, 3
);
impl_nonconvex_two_body_accessor!(
    Contact4NonconvexTwoBodyAccessor, Contact4Nonconvex, Contact4NonconvexPrestepData,
    Contact4NonconvexAccumulatedImpulses, ContactImpulses4, 4
);
