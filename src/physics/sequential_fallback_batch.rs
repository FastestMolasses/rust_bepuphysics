// Translated from BepuPhysics/SequentialFallbackBatch.cs

use crate::physics::bodies::Bodies;
use crate::physics::handles::BodyHandle;
use crate::utilities::collections::primitive_comparer::PrimitiveComparer;
use crate::utilities::collections::quick_dictionary::QuickDictionary;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Trait for getting body references — active set returns body index, inactive set returns handle.
pub(crate) trait IBodyReferenceGetter {
    fn get_body_reference(bodies: &Bodies, handle: BodyHandle) -> i32;
}

pub(crate) struct ActiveSetGetter;
impl IBodyReferenceGetter for ActiveSetGetter {
    #[inline(always)]
    fn get_body_reference(bodies: &Bodies, body_handle: BodyHandle) -> i32 {
        let body_location = bodies.handle_to_location.get(body_handle.0);
        debug_assert!(
            body_location.set_index == 0,
            "When creating a fallback batch for the active set, all bodies associated with it must be active."
        );
        body_location.index
    }
}

pub(crate) struct InactiveSetGetter;
impl IBodyReferenceGetter for InactiveSetGetter {
    #[inline(always)]
    fn get_body_reference(_bodies: &Bodies, body_handle: BodyHandle) -> i32 {
        body_handle.0
    }
}

/// Contains constraints that could not belong to any lower constraint batch due to their involved
/// bodies. All of the contained constraints will be solved using a fallback solver that trades
/// rigidity for parallelism.
pub struct SequentialFallbackBatch {
    // In order to maintain the batch referenced handles for the fallback batch (which can have
    // the same body appear more than once), every body must maintain a count of fallback
    // constraints associated with it.
    // Note that this dictionary uses active set body *indices* while active, but body *handles*
    // when associated with an inactive set. This is consistent with body references stored by
    // active/inactive constraints.
    pub(crate) dynamic_body_constraint_counts:
        QuickDictionary<i32, i32, PrimitiveComparer<i32>>,
}

impl SequentialFallbackBatch {
    /// Gets the number of bodies in the fallback batch.
    pub fn body_count(&self) -> i32 {
        self.dynamic_body_constraint_counts.count
    }

    fn allocate<TGetter: IBodyReferenceGetter>(
        &mut self,
        dynamic_body_handles: &[BodyHandle],
        bodies: &Bodies,
        pool: &mut BufferPool,
        minimum_body_capacity: i32,
    ) {
        let target = (self.dynamic_body_constraint_counts.count + dynamic_body_handles.len() as i32)
            .max(minimum_body_capacity);
        self.ensure_capacity(target, pool);
        for i in 0..dynamic_body_handles.len() {
            let body_reference =
                TGetter::get_body_reference(bodies, dynamic_body_handles[i]);

            let mut slot_index = 0i32;
            if self
                .dynamic_body_constraint_counts
                .find_or_allocate_slot_unsafely(&body_reference, &mut slot_index)
            {
                *self.dynamic_body_constraint_counts.value_at_mut(slot_index) += 1;
            } else {
                *self.dynamic_body_constraint_counts.value_at_mut(slot_index) = 1;
            }
        }
    }

    #[inline(always)]
    pub(crate) fn allocate_for_active(
        &mut self,
        dynamic_body_handles: &[BodyHandle],
        bodies: &Bodies,
        pool: &mut BufferPool,
        minimum_body_capacity: i32,
    ) {
        self.allocate::<ActiveSetGetter>(
            dynamic_body_handles,
            bodies,
            pool,
            minimum_body_capacity.max(8),
        );
    }

    #[inline(always)]
    pub(crate) fn allocate_for_inactive(
        &mut self,
        dynamic_body_handles: &[BodyHandle],
        bodies: &Bodies,
        pool: &mut BufferPool,
        minimum_body_capacity: i32,
    ) {
        self.allocate::<InactiveSetGetter>(
            dynamic_body_handles,
            bodies,
            pool,
            minimum_body_capacity.max(8),
        );
    }

    /// Removes a constraint from a body in the fallback batch.
    /// Returns true if the body was dynamic and no longer has any constraints associated
    /// with it in the fallback batch, false otherwise.
    pub(crate) fn remove_one_body_reference_from_dynamics_set(
        &mut self,
        body_reference: i32,
        allocation_ids_to_free: &mut QuickList<i32>,
    ) -> bool {
        let mut table_index = 0i32;
        let mut body_references_index = 0i32;
        let mut body_ref = body_reference;
        if !self.dynamic_body_constraint_counts.get_table_indices(
            &mut body_ref,
            &mut table_index,
            &mut body_references_index,
        ) {
            return false;
        }

        let constraint_count = self.dynamic_body_constraint_counts.value_at_mut(body_references_index);
        *constraint_count -= 1;
        if *constraint_count == 0 {
            // No more constraints associated with this body, get rid of the body list.
            *constraint_count = 0;
            self.dynamic_body_constraint_counts
                .fast_remove_at(table_index, body_references_index);
            if self.dynamic_body_constraint_counts.count == 0 {
                // No constraints remain in the fallback batch. Drop the dictionary.
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.keys.id());
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.values.id());
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.table.id());
                self.dynamic_body_constraint_counts = QuickDictionary::default();
            }
            return true;
        }
        false
    }

    /// Removes a body from the fallback batch's dynamic body constraint counts if present.
    /// Returns true if the body was present and was removed.
    pub(crate) fn try_remove_dynamic_body_from_tracking(
        &mut self,
        body_reference: i32,
        allocation_ids_to_free: &mut QuickList<i32>,
    ) -> bool {
        let mut body_ref = body_reference;
        let mut table_index = 0i32;
        let mut body_references_index = 0i32;
        if self.dynamic_body_constraint_counts.keys.allocated()
            && self.dynamic_body_constraint_counts.get_table_indices(
                &mut body_ref,
                &mut table_index,
                &mut body_references_index,
            )
        {
            self.dynamic_body_constraint_counts
                .fast_remove_at(table_index, body_references_index);
            if self.dynamic_body_constraint_counts.count == 0 {
                // No constraints remain in the fallback batch. Drop the dictionary.
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.keys.id());
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.values.id());
                allocation_ids_to_free.add_unsafely(self.dynamic_body_constraint_counts.table.id());
                self.dynamic_body_constraint_counts = QuickDictionary::default();
            }
            return true;
        }
        false
    }

    pub(crate) fn update_for_dynamic_body_memory_move(
        &mut self,
        original_body_index: i32,
        new_body_location: i32,
    ) {
        debug_assert!(
            self.dynamic_body_constraint_counts.keys.allocated()
                && !self.dynamic_body_constraint_counts.contains_key(&new_body_location),
            "If a body is being moved, the target index should not be present."
        );
        let mut table_index = 0i32;
        let mut element_index = 0i32;
        let mut orig = original_body_index;
        self.dynamic_body_constraint_counts.get_table_indices(
            &mut orig,
            &mut table_index,
            &mut element_index,
        );
        let references = *self.dynamic_body_constraint_counts.value_at(element_index);
        self.dynamic_body_constraint_counts
            .fast_remove_at(table_index, element_index);
        self.dynamic_body_constraint_counts
            .add_unsafely(new_body_location, references);
    }

    pub(crate) fn update_for_body_memory_swap(&mut self, a: i32, b: i32) {
        let index_a = self.dynamic_body_constraint_counts.index_of(&a);
        let index_b = self.dynamic_body_constraint_counts.index_of(&b);
        debug_assert!(
            index_a.is_some() && index_b.is_some(),
            "A swap requires that both indices are already present."
        );
        let index_a = index_a.unwrap();
        let index_b = index_b.unwrap();
        unsafe {
            let ptr_a = self.dynamic_body_constraint_counts.value_at_mut(index_a) as *mut i32;
            let ptr_b = self.dynamic_body_constraint_counts.value_at_mut(index_b) as *mut i32;
            std::ptr::swap(ptr_a, ptr_b);
        }
    }

    pub(crate) fn create_from(
        source_batch: &SequentialFallbackBatch,
        pool: &mut BufferPool,
    ) -> SequentialFallbackBatch {
        let mut target = SequentialFallbackBatch::default();
        target.dynamic_body_constraint_counts.count = source_batch.dynamic_body_constraint_counts.count;
        target.dynamic_body_constraint_counts.table_mask = source_batch.dynamic_body_constraint_counts.table_mask;
        target.dynamic_body_constraint_counts.table_power_offset = source_batch.dynamic_body_constraint_counts.table_power_offset;
        target.dynamic_body_constraint_counts.equality_comparer = source_batch.dynamic_body_constraint_counts.equality_comparer;

        target.dynamic_body_constraint_counts.keys =
            pool.take_at_least(source_batch.dynamic_body_constraint_counts.count);
        target.dynamic_body_constraint_counts.values =
            pool.take_at_least(target.dynamic_body_constraint_counts.keys.len());
        target.dynamic_body_constraint_counts.table =
            pool.take_at_least(source_batch.dynamic_body_constraint_counts.table_mask + 1);

        source_batch.dynamic_body_constraint_counts.keys.copy_to(
            0,
            &mut target.dynamic_body_constraint_counts.keys,
            0,
            source_batch.dynamic_body_constraint_counts.count,
        );
        source_batch.dynamic_body_constraint_counts.values.copy_to(
            0,
            &mut target.dynamic_body_constraint_counts.values,
            0,
            source_batch.dynamic_body_constraint_counts.count,
        );
        source_batch.dynamic_body_constraint_counts.table.copy_to(
            0,
            &mut target.dynamic_body_constraint_counts.table,
            0,
            source_batch.dynamic_body_constraint_counts.table_mask + 1,
        );
        target
    }

    pub(crate) fn ensure_capacity(&mut self, body_capacity: i32, pool: &mut BufferPool) {
        if self.dynamic_body_constraint_counts.keys.allocated() {
            // This is conservative since there's no guarantee that we'll actually need to resize
            // if these bodies are already present, but that's fine.
            self.dynamic_body_constraint_counts
                .ensure_capacity(body_capacity, pool);
        } else {
            self.dynamic_body_constraint_counts =
                QuickDictionary::with_capacity(body_capacity, 2, pool, PrimitiveComparer::new());
        }
    }

    pub fn compact(&mut self, pool: &mut BufferPool) {
        if self.dynamic_body_constraint_counts.keys.allocated() {
            self.dynamic_body_constraint_counts.compact(pool);
        }
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        if self.dynamic_body_constraint_counts.keys.allocated() {
            self.dynamic_body_constraint_counts.dispose(pool);
        }
    }
}

impl Default for SequentialFallbackBatch {
    fn default() -> Self {
        Self {
            dynamic_body_constraint_counts: QuickDictionary::default(),
        }
    }
}
