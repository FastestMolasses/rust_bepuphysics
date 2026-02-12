// Translated from BepuPhysics/Constraints/ThreeBodyTypeProcessor.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::body_access_filter::{AccessAll, AccessOnlyVelocity, IBodyAccessFilter};
use crate::physics::constraints::body_references::ThreeBodyReferences;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_batch_alloc;
use crate::physics::constraints::type_processor::ITypeProcessor;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::marker::PhantomData;

/// Prestep, warm start and solve iteration functions for a three-body constraint type.
pub trait IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> {
    fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        position_c: &Vector3Wide,
        orientation_c: &QuaternionWide,
        inertia_c: &BodyInertiaWide,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
    );

    fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        position_c: &Vector3Wide,
        orientation_c: &QuaternionWide,
        inertia_c: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
    );

    fn requires_incremental_substep_updates() -> bool;

    fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        wsv_c: &BodyVelocityWide,
        prestep_data: &mut TPrestepData,
    );
}

/// Generic three-body type processor implementation.
///
/// This corresponds to C#'s `ThreeBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, ...>`.
pub struct ThreeBodyTypeProcessorImpl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
> {
    type_id: i32,
    constrained_degrees_of_freedom: i32,
    _marker: PhantomData<(TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC)>,
}

impl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
> ThreeBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC> {
    pub fn new(type_id: i32, constrained_degrees_of_freedom: i32) -> Self {
        Self {
            type_id,
            constrained_degrees_of_freedom,
            _marker: PhantomData,
        }
    }
}

impl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IThreeBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
> ITypeProcessor for ThreeBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC> {
    #[inline(always)]
    fn bodies_per_constraint(&self) -> i32 {
        3
    }

    #[inline(always)]
    fn constrained_degrees_of_freedom(&self) -> i32 {
        self.constrained_degrees_of_freedom
    }

    #[inline(always)]
    fn type_id(&self) -> i32 {
        self.type_id
    }

    fn requires_incremental_substep_updates(&self) -> bool {
        TConstraintFunctions::requires_incremental_substep_updates()
    }

    fn allocate_in_type_batch(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: &mut BufferPool,
    ) -> i32 {
        unsafe {
            type_batch_alloc::allocate_in_type_batch(
                type_batch,
                handle,
                encoded_body_indices,
                pool,
                3,
                std::mem::size_of::<ThreeBodyReferences>(),
                std::mem::size_of::<TPrestepData>(),
                std::mem::size_of::<TAccumulatedImpulse>(),
            )
        }
    }

    fn allocate_in_type_batch_for_fallback(
        &self,
        type_batch: &mut TypeBatch,
        handle: ConstraintHandle,
        encoded_body_indices: &[i32],
        pool: *mut BufferPool,
    ) -> i32 {
        unsafe {
            type_batch_alloc::allocate_in_type_batch_for_fallback(
                type_batch,
                handle,
                encoded_body_indices,
                &mut *pool,
                3, // bodies_per_constraint
                std::mem::size_of::<ThreeBodyReferences>(),
                std::mem::size_of::<TPrestepData>(),
                std::mem::size_of::<TAccumulatedImpulse>(),
            )
        }
    }

    fn remove(
        &self,
        type_batch: &mut TypeBatch,
        index: i32,
        handles_to_constraints: &mut Buffer<ConstraintLocation>,
        is_fallback: bool,
    ) {
        unsafe {
            if is_fallback {
                type_batch_alloc::remove_from_type_batch_fallback::<
                    ThreeBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 3);
            } else {
                type_batch_alloc::remove_from_type_batch::<
                    ThreeBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 3);
            }
        }
    }

    fn initialize(&self, type_batch: &mut TypeBatch, initial_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::initialize_type_batch(
            type_batch,
            self.type_id,
            initial_capacity,
            pool,
            std::mem::size_of::<ThreeBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
    }

    fn resize(&self, type_batch: &mut TypeBatch, new_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::resize_type_batch(
            type_batch,
            new_capacity,
            pool,
            std::mem::size_of::<ThreeBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
    }

    fn get_bundle_type_sizes(
        &self,
        body_references_bundle_size: &mut i32,
        prestep_bundle_size: &mut i32,
        accumulated_impulse_bundle_size: &mut i32,
    ) {
        *body_references_bundle_size = std::mem::size_of::<ThreeBodyReferences>() as i32;
        *prestep_bundle_size = std::mem::size_of::<TPrestepData>() as i32;
        *accumulated_impulse_bundle_size = std::mem::size_of::<TAccumulatedImpulse>() as i32;
    }

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
    ) {
        unsafe {
            let body_references = type_batch.body_references.as_ptr() as *const ThreeBodyReferences;
            for i in 0..constraint_count {
                *first_source_index.add(i as usize) = local_constraint_start + i;
                let constraint_index = constraint_start + i;
                let bundle_index = (constraint_index as usize) >> crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
                let inner_index = (constraint_index as usize) & crate::utilities::bundle_indexing::VECTOR_MASK;
                let refs = &*body_references.add(bundle_index);
                let a = refs.index_a[inner_index];
                let b = refs.index_b[inner_index];
                let c = refs.index_c[inner_index];
                *first_sort_key.add(i as usize) = a.min(b).min(c);
            }
            // Copy body references to cache
            let ref_size = std::mem::size_of::<ThreeBodyReferences>();
            let src = type_batch.body_references.as_ptr().add(bundle_start as usize * ref_size);
            let dst = body_references_cache.as_mut_ptr().add(local_bundle_start as usize * ref_size);
            std::ptr::copy_nonoverlapping(src, dst, bundle_count as usize * ref_size);
        }
    }

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
    ) {
        unsafe {
            let src_handles = type_batch.index_to_handle.as_ptr().add(constraint_start as usize);
            let dst_handles = index_to_handle_cache.as_mut_ptr().add(local_constraint_start as usize);
            std::ptr::copy_nonoverlapping(src_handles, dst_handles, constraint_count as usize);
            let prestep_size = std::mem::size_of::<TPrestepData>();
            let src_prestep = type_batch.prestep_data.as_ptr().add(prestep_size * bundle_start as usize);
            let dst_prestep = prestep_cache.as_mut_ptr().add(prestep_size * local_bundle_start as usize);
            std::ptr::copy_nonoverlapping(src_prestep, dst_prestep, prestep_size * bundle_count as usize);
            let impulse_size = std::mem::size_of::<TAccumulatedImpulse>();
            let src_impulse = type_batch.accumulated_impulses.as_ptr().add(impulse_size * bundle_start as usize);
            let dst_impulse = accumulated_impulses_cache.as_mut_ptr().add(impulse_size * local_bundle_start as usize);
            std::ptr::copy_nonoverlapping(src_impulse, dst_impulse, impulse_size * bundle_count as usize);
        }
    }

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
    ) {
        unsafe {
            let body_ref_cache = body_references_cache.as_ptr() as *const ThreeBodyReferences;
            let prestep_cache_typed = prestep_cache.as_ptr() as *const TPrestepData;
            let impulse_cache_typed = accumulated_impulses_cache.as_ptr() as *const TAccumulatedImpulse;

            let body_ref_target = type_batch.body_references.as_mut_ptr() as *mut ThreeBodyReferences;
            let prestep_target = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let impulse_target = type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            let vector_shift = crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
            let vector_mask = crate::utilities::bundle_indexing::VECTOR_MASK;

            for i in 0..constraint_count {
                let source_index = *first_source_index.add(i as usize) as usize;
                let target_index = (constraint_start + i) as usize;

                let source_bundle = source_index >> vector_shift;
                let source_inner = source_index & vector_mask;
                let target_bundle = target_index >> vector_shift;
                let target_inner = target_index & vector_mask;

                // Copy body reference lanes
                let src_ref = &*body_ref_cache.add(source_bundle);
                let dst_ref = &mut *body_ref_target.add(target_bundle);
                dst_ref.index_a[target_inner] = src_ref.index_a[source_inner];
                dst_ref.index_b[target_inner] = src_ref.index_b[source_inner];
                dst_ref.index_c[target_inner] = src_ref.index_c[source_inner];

                // Copy prestep data lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TPrestepData>(
                    &*prestep_cache_typed.add(source_bundle), source_inner,
                    &mut *prestep_target.add(target_bundle), target_inner,
                );

                // Copy accumulated impulses lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TAccumulatedImpulse>(
                    &*impulse_cache_typed.add(source_bundle), source_inner,
                    &mut *impulse_target.add(target_bundle), target_inner,
                );

                // Update index to handle and handle-to-constraint mapping
                let handle = *index_to_handle_cache.get(source_index as i32);
                *type_batch.index_to_handle.get_mut(target_index as i32) = handle;
                let location = handles_to_constraints.get_mut(handle.0);
                location.index_in_type_batch = target_index as i32;
            }
        }
    }

    fn scale_accumulated_impulses(&self, type_batch: &mut TypeBatch, scale: f32) {
        let dof_count = std::mem::size_of::<TAccumulatedImpulse>()
            / std::mem::size_of::<crate::utilities::vector::Vector<f32>>();
        let broadcasted_scale = crate::utilities::vector::Vector::<f32>::splat(scale);
        unsafe {
            let impulses_base =
                type_batch.accumulated_impulses.as_mut_ptr() as *mut crate::utilities::vector::Vector<f32>;
            for i in 0..dof_count {
                *impulses_base.add(i) *= broadcasted_scale;
            }
        }
    }

    fn warm_start(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        let _ = (dt, inverse_dt);

        unsafe {
            let prestep_bundles = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut ThreeBodyReferences;
            let impulse_bundles = type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            for i in start_bundle..exclusive_end_bundle {
                let idx = i as usize;
                let prestep = &mut *prestep_bundles.add(idx);
                let accumulated_impulses = &mut *impulse_bundles.add(idx);
                let references = &*body_ref_bundles.add(idx);

                let mut position_a = Vector3Wide::default();
                let mut orientation_a = QuaternionWide::default();
                let mut wsv_a = BodyVelocityWide::default();
                let mut inertia_a = BodyInertiaWide::default();
                bodies.gather_state::<AccessAll>(
                    &references.index_a, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();
                bodies.gather_state::<AccessAll>(
                    &references.index_b, true,
                    &mut position_b, &mut orientation_b, &mut wsv_b, &mut inertia_b,
                );

                let mut position_c = Vector3Wide::default();
                let mut orientation_c = QuaternionWide::default();
                let mut wsv_c = BodyVelocityWide::default();
                let mut inertia_c = BodyInertiaWide::default();
                bodies.gather_state::<AccessAll>(
                    &references.index_c, true,
                    &mut position_c, &mut orientation_c, &mut wsv_c, &mut inertia_c,
                );

                TConstraintFunctions::warm_start(
                    &position_a, &orientation_a, &inertia_a,
                    &position_b, &orientation_b, &inertia_b,
                    &position_c, &orientation_c, &inertia_c,
                    prestep, accumulated_impulses,
                    &mut wsv_a, &mut wsv_b, &mut wsv_c,
                );

                bodies.scatter_velocities::<AccessAll>(&wsv_a, &references.index_a);
                bodies.scatter_velocities::<AccessAll>(&wsv_b, &references.index_b);
                bodies.scatter_velocities::<AccessAll>(&wsv_c, &references.index_c);
            }
        }
    }

    fn solve(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &Bodies,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        unsafe {
            let prestep_bundles = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut ThreeBodyReferences;
            let impulse_bundles = type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            for i in start_bundle..exclusive_end_bundle {
                let idx = i as usize;
                let prestep = &mut *prestep_bundles.add(idx);
                let accumulated_impulses = &mut *impulse_bundles.add(idx);
                let references = &*body_ref_bundles.add(idx);

                let mut position_a = Vector3Wide::default();
                let mut orientation_a = QuaternionWide::default();
                let mut wsv_a = BodyVelocityWide::default();
                let mut inertia_a = BodyInertiaWide::default();
                bodies.gather_state::<TSolveAccessFilterA>(
                    &references.index_a, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();
                bodies.gather_state::<TSolveAccessFilterB>(
                    &references.index_b, true,
                    &mut position_b, &mut orientation_b, &mut wsv_b, &mut inertia_b,
                );

                let mut position_c = Vector3Wide::default();
                let mut orientation_c = QuaternionWide::default();
                let mut wsv_c = BodyVelocityWide::default();
                let mut inertia_c = BodyInertiaWide::default();
                bodies.gather_state::<TSolveAccessFilterC>(
                    &references.index_c, true,
                    &mut position_c, &mut orientation_c, &mut wsv_c, &mut inertia_c,
                );

                TConstraintFunctions::solve(
                    &position_a, &orientation_a, &inertia_a,
                    &position_b, &orientation_b, &inertia_b,
                    &position_c, &orientation_c, &inertia_c,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    &mut wsv_a, &mut wsv_b, &mut wsv_c,
                );

                bodies.scatter_velocities::<TSolveAccessFilterA>(&wsv_a, &references.index_a);
                bodies.scatter_velocities::<TSolveAccessFilterB>(&wsv_b, &references.index_b);
                bodies.scatter_velocities::<TSolveAccessFilterC>(&wsv_c, &references.index_c);
            }
        }
    }

    fn incrementally_update_for_substep(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &Bodies,
        dt: f32,
        _inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
    ) {
        unsafe {
            let prestep_bundles = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut ThreeBodyReferences;
            let dt_wide = Vector::<f32>::splat(dt);

            for i in start_bundle..exclusive_end_bundle {
                let idx = i as usize;
                let prestep = &mut *prestep_bundles.add(idx);
                let references = &*body_ref_bundles.add(idx);

                let mut position_a = Vector3Wide::default();
                let mut orientation_a = QuaternionWide::default();
                let mut wsv_a = BodyVelocityWide::default();
                let mut inertia_a = BodyInertiaWide::default();
                bodies.gather_state::<AccessOnlyVelocity>(
                    &references.index_a, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();
                bodies.gather_state::<AccessOnlyVelocity>(
                    &references.index_b, true,
                    &mut position_b, &mut orientation_b, &mut wsv_b, &mut inertia_b,
                );

                let mut position_c = Vector3Wide::default();
                let mut orientation_c = QuaternionWide::default();
                let mut wsv_c = BodyVelocityWide::default();
                let mut inertia_c = BodyInertiaWide::default();
                bodies.gather_state::<AccessOnlyVelocity>(
                    &references.index_c, true,
                    &mut position_c, &mut orientation_c, &mut wsv_c, &mut inertia_c,
                );

                TConstraintFunctions::incrementally_update_for_substep(
                    &dt_wide, &wsv_a, &wsv_b, &wsv_c, prestep,
                );
            }
        }
    }
}
