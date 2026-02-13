// Translated from BepuPhysics/Constraints/TwoBodyTypeProcessor.cs
//
// The C# TwoBodyTypeProcessor is an abstract generic class. In Rust, we separate the constraint
// function interface (ITwoBodyConstraintFunctions) from the type processor loop logic
// (TwoBodyTypeProcessorImpl). The concrete type processor struct (e.g. BallSocketTypeProcessor)
// is replaced by instantiating TwoBodyTypeProcessorImpl with the appropriate type parameters.

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::body_access_filter::{
    AccessAll, AccessOnlyVelocity, IBodyAccessFilter,
};
use crate::physics::constraints::body_references::TwoBodyReferences;
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

/// Prestep, warm start and solve iteration functions for a two-body constraint type.
pub trait ITwoBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> {
    fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    );

    fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    );

    /// Gets whether this constraint type requires incremental updates for each substep
    /// taken beyond the first.
    fn requires_incremental_substep_updates() -> bool;

    fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        prestep_data: &mut TPrestepData,
    );
}

/// Generic two-body type processor implementation that bridges the concrete constraint functions
/// (which operate on typed prestep/impulse data) to the solver's untyped `ITypeProcessor` interface.
///
/// Type parameters:
/// - `TPrestepData`: The AOSOA-layout prestep data for one bundle of constraints.
/// - `TAccumulatedImpulse`: The AOSOA-layout accumulated impulse data for one bundle.
/// - `TConstraintFunctions`: The struct implementing `ITwoBodyConstraintFunctions`.
/// - `TSolveAccessFilterA/B`: Body access filters used during the solve iteration.
///
/// This corresponds to C#'s `TwoBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, ...>`.
pub struct TwoBodyTypeProcessorImpl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: ITwoBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
> {
    type_id: i32,
    constrained_degrees_of_freedom: i32,
    _marker: PhantomData<(
        TPrestepData,
        TAccumulatedImpulse,
        TConstraintFunctions,
        TSolveAccessFilterA,
        TSolveAccessFilterB,
    )>,
}

impl<
        TPrestepData: Copy + 'static,
        TAccumulatedImpulse: Copy + 'static,
        TConstraintFunctions: ITwoBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
        TSolveAccessFilterA: IBodyAccessFilter + 'static,
        TSolveAccessFilterB: IBodyAccessFilter + 'static,
    >
    TwoBodyTypeProcessorImpl<
        TPrestepData,
        TAccumulatedImpulse,
        TConstraintFunctions,
        TSolveAccessFilterA,
        TSolveAccessFilterB,
    >
{
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
        TConstraintFunctions: ITwoBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
        TSolveAccessFilterA: IBodyAccessFilter + 'static,
        TSolveAccessFilterB: IBodyAccessFilter + 'static,
    > ITypeProcessor
    for TwoBodyTypeProcessorImpl<
        TPrestepData,
        TAccumulatedImpulse,
        TConstraintFunctions,
        TSolveAccessFilterA,
        TSolveAccessFilterB,
    >
{
    #[inline(always)]
    fn bodies_per_constraint(&self) -> i32 {
        2
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
                2, // bodies_per_constraint
                std::mem::size_of::<TwoBodyReferences>(),
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
                2, // bodies_per_constraint
                std::mem::size_of::<TwoBodyReferences>(),
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
                    TwoBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 2);
            } else {
                type_batch_alloc::remove_from_type_batch::<
                    TwoBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 2);
            }
        }
    }

    fn initialize(&self, type_batch: &mut TypeBatch, initial_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::initialize_type_batch(
            type_batch,
            self.type_id,
            initial_capacity,
            pool,
            std::mem::size_of::<TwoBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
    }

    fn resize(&self, type_batch: &mut TypeBatch, new_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::resize_type_batch(
            type_batch,
            new_capacity,
            pool,
            std::mem::size_of::<TwoBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
    }

    fn scale_accumulated_impulses(&self, type_batch: &mut TypeBatch, scale: f32) {
        let dof_count = std::mem::size_of::<TAccumulatedImpulse>()
            / std::mem::size_of::<crate::utilities::vector::Vector<f32>>();
        let broadcasted_scale = crate::utilities::vector::Vector::<f32>::splat(scale);
        unsafe {
            let impulses_base = type_batch.accumulated_impulses.as_mut_ptr()
                as *mut crate::utilities::vector::Vector<f32>;
            for i in 0..dof_count {
                *impulses_base.add(i) *= broadcasted_scale;
            }
        }
    }

    fn warm_start(
        &self,
        type_batch: &mut TypeBatch,
        bodies: &Bodies,
        integration_flags: &Buffer<IndexSet>,
        pose_integrator: Option<&dyn crate::physics::pose_integrator::IPoseIntegrator>,
        batch_integration_mode: crate::physics::constraints::batch_integration_mode::BatchIntegrationMode,
        allow_pose_integration: bool,
        dt: f32,
        inverse_dt: f32,
        start_bundle: i32,
        exclusive_end_bundle: i32,
        worker_index: i32,
    ) {
        let _ = inverse_dt;

        unsafe {
            let prestep_bundles = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let body_ref_bundles =
                type_batch.body_references.as_mut_ptr() as *mut TwoBodyReferences;
            let impulse_bundles =
                type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            // Build integration callback closure if integration is needed.
            let angular_mode = pose_integrator
                .map(|pi| pi.angular_integration_mode())
                .unwrap_or(crate::physics::pose_integration::AngularIntegrationMode::Nonconserving);
            let velocity_fn = |body_indices: Vector<i32>,
                               position: Vector3Wide,
                               orientation: QuaternionWide,
                               local_inertia: BodyInertiaWide,
                               integration_mask: Vector<i32>,
                               w_index: i32,
                               dt_vec: Vector<f32>,
                               velocity: &mut BodyVelocityWide| {
                if let Some(pi) = pose_integrator {
                    pi.integrate_velocity_callback(
                        body_indices,
                        position,
                        orientation,
                        local_inertia,
                        integration_mask,
                        w_index,
                        dt_vec,
                        velocity,
                    );
                }
            };

            for i in start_bundle..exclusive_end_bundle {
                let idx = i as usize;
                let prestep = &mut *prestep_bundles.add(idx);
                let accumulated_impulses = &mut *impulse_bundles.add(idx);
                let references = &*body_ref_bundles.add(idx);

                let mut position_a = Vector3Wide::default();
                let mut orientation_a = QuaternionWide::default();
                let mut wsv_a = BodyVelocityWide::default();
                let mut inertia_a = BodyInertiaWide::default();

                crate::physics::constraints::gather_and_integrate::gather_and_integrate::<AccessAll>(
                    bodies,
                    angular_mode,
                    &velocity_fn,
                    integration_flags,
                    0,
                    batch_integration_mode,
                    allow_pose_integration,
                    dt,
                    worker_index,
                    i,
                    &references.index_a,
                    &mut position_a,
                    &mut orientation_a,
                    &mut wsv_a,
                    &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();

                crate::physics::constraints::gather_and_integrate::gather_and_integrate::<AccessAll>(
                    bodies,
                    angular_mode,
                    &velocity_fn,
                    integration_flags,
                    1,
                    batch_integration_mode,
                    allow_pose_integration,
                    dt,
                    worker_index,
                    i,
                    &references.index_b,
                    &mut position_b,
                    &mut orientation_b,
                    &mut wsv_b,
                    &mut inertia_b,
                );

                TConstraintFunctions::warm_start(
                    &position_a,
                    &orientation_a,
                    &inertia_a,
                    &position_b,
                    &orientation_b,
                    &inertia_b,
                    prestep,
                    accumulated_impulses,
                    &mut wsv_a,
                    &mut wsv_b,
                );

                // When integration happens, all velocities are gathered fully → scatter with AccessAll.
                // When no integration, use the warm start access filter.
                if batch_integration_mode == crate::physics::constraints::batch_integration_mode::BatchIntegrationMode::Never {
                    bodies.scatter_velocities::<AccessAll>(&wsv_a, &references.index_a);
                    bodies.scatter_velocities::<AccessAll>(&wsv_b, &references.index_b);
                } else {
                    bodies.scatter_velocities::<AccessAll>(&wsv_a, &references.index_a);
                    bodies.scatter_velocities::<AccessAll>(&wsv_b, &references.index_b);
                }
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
            let body_ref_bundles =
                type_batch.body_references.as_mut_ptr() as *mut TwoBodyReferences;
            let impulse_bundles =
                type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

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
                    &references.index_a,
                    true,
                    &mut position_a,
                    &mut orientation_a,
                    &mut wsv_a,
                    &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();
                bodies.gather_state::<TSolveAccessFilterB>(
                    &references.index_b,
                    true,
                    &mut position_b,
                    &mut orientation_b,
                    &mut wsv_b,
                    &mut inertia_b,
                );

                TConstraintFunctions::solve(
                    &position_a,
                    &orientation_a,
                    &inertia_a,
                    &position_b,
                    &orientation_b,
                    &inertia_b,
                    dt,
                    inverse_dt,
                    prestep,
                    accumulated_impulses,
                    &mut wsv_a,
                    &mut wsv_b,
                );

                bodies.scatter_velocities::<TSolveAccessFilterA>(&wsv_a, &references.index_a);
                bodies.scatter_velocities::<TSolveAccessFilterB>(&wsv_b, &references.index_b);
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
            let body_ref_bundles =
                type_batch.body_references.as_mut_ptr() as *mut TwoBodyReferences;
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
                    &references.index_a,
                    true,
                    &mut position_a,
                    &mut orientation_a,
                    &mut wsv_a,
                    &mut inertia_a,
                );

                let mut position_b = Vector3Wide::default();
                let mut orientation_b = QuaternionWide::default();
                let mut wsv_b = BodyVelocityWide::default();
                let mut inertia_b = BodyInertiaWide::default();
                bodies.gather_state::<AccessOnlyVelocity>(
                    &references.index_b,
                    true,
                    &mut position_b,
                    &mut orientation_b,
                    &mut wsv_b,
                    &mut inertia_b,
                );

                TConstraintFunctions::incrementally_update_for_substep(
                    &dt_wide, &wsv_a, &wsv_b, prestep,
                );
            }
        }
    }

    fn get_bundle_type_sizes(
        &self,
        body_references_bundle_size: &mut i32,
        prestep_bundle_size: &mut i32,
        accumulated_impulse_bundle_size: &mut i32,
    ) {
        *body_references_bundle_size = std::mem::size_of::<TwoBodyReferences>() as i32;
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
            let body_references = type_batch.body_references.as_ptr() as *const TwoBodyReferences;
            let body_references_buf = std::slice::from_raw_parts(
                body_references,
                type_batch.body_references.len() as usize
                    / std::mem::size_of::<TwoBodyReferences>(),
            );
            for i in 0..constraint_count {
                *first_source_index.add(i as usize) = local_constraint_start + i;
                // Sort key = min(indexA, indexB) for the constraint
                let constraint_index = constraint_start + i;
                let bundle_index = constraint_index
                    >> crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
                let inner_index =
                    constraint_index & crate::utilities::bundle_indexing::VECTOR_MASK as i32;
                let refs = &body_references_buf[bundle_index as usize];
                let index_a = refs.index_a[inner_index as usize];
                let index_b = refs.index_b[inner_index as usize];
                *first_sort_key.add(i as usize) = if index_a < index_b { index_a } else { index_b };
            }
            // Copy body references to cache
            let src = type_batch
                .body_references
                .as_ptr()
                .add(bundle_start as usize * std::mem::size_of::<TwoBodyReferences>());
            let dst = body_references_cache
                .as_mut_ptr()
                .add(local_bundle_start as usize * std::mem::size_of::<TwoBodyReferences>());
            std::ptr::copy_nonoverlapping(
                src,
                dst,
                bundle_count as usize * std::mem::size_of::<TwoBodyReferences>(),
            );
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
            // Copy IndexToHandle
            let src_handles = type_batch
                .index_to_handle
                .as_ptr()
                .add(constraint_start as usize);
            let dst_handles = index_to_handle_cache
                .as_mut_ptr()
                .add(local_constraint_start as usize);
            std::ptr::copy_nonoverlapping(src_handles, dst_handles, constraint_count as usize);
            // Copy prestep data
            let prestep_size = std::mem::size_of::<TPrestepData>();
            let src_prestep = type_batch
                .prestep_data
                .as_ptr()
                .add(prestep_size * bundle_start as usize);
            let dst_prestep = prestep_cache
                .as_mut_ptr()
                .add(prestep_size * local_bundle_start as usize);
            std::ptr::copy_nonoverlapping(
                src_prestep,
                dst_prestep,
                prestep_size * bundle_count as usize,
            );
            // Copy accumulated impulses
            let impulse_size = std::mem::size_of::<TAccumulatedImpulse>();
            let src_impulse = type_batch
                .accumulated_impulses
                .as_ptr()
                .add(impulse_size * bundle_start as usize);
            let dst_impulse = accumulated_impulses_cache
                .as_mut_ptr()
                .add(impulse_size * local_bundle_start as usize);
            std::ptr::copy_nonoverlapping(
                src_impulse,
                dst_impulse,
                impulse_size * bundle_count as usize,
            );
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
            let body_ref_cache = body_references_cache.as_ptr() as *const TwoBodyReferences;
            let prestep_cache_typed = prestep_cache.as_ptr() as *const TPrestepData;
            let impulse_cache_typed =
                accumulated_impulses_cache.as_ptr() as *const TAccumulatedImpulse;

            let body_ref_target = type_batch.body_references.as_mut_ptr() as *mut TwoBodyReferences;
            let prestep_target = type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData;
            let impulse_target =
                type_batch.accumulated_impulses.as_mut_ptr() as *mut TAccumulatedImpulse;

            let vector_shift = crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
            let vector_mask = crate::utilities::bundle_indexing::VECTOR_MASK;

            for i in 0..constraint_count {
                let source_index = *first_source_index.add(i as usize) as usize;
                let target_index = (constraint_start + i) as usize;

                let source_bundle = source_index >> vector_shift;
                let source_inner = source_index & vector_mask;
                let target_bundle = target_index >> vector_shift;
                let target_inner = target_index & vector_mask;

                // Copy body references lane by lane
                let src_ref = &*body_ref_cache.add(source_bundle);
                let dst_ref = &mut *body_ref_target.add(target_bundle);
                dst_ref.index_a[target_inner] = src_ref.index_a[source_inner];
                dst_ref.index_b[target_inner] = src_ref.index_b[source_inner];

                // Copy prestep data lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TPrestepData>(
                    &*prestep_cache_typed.add(source_bundle),
                    source_inner,
                    &mut *prestep_target.add(target_bundle),
                    target_inner,
                );

                // Copy accumulated impulses lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TAccumulatedImpulse>(
                    &*impulse_cache_typed.add(source_bundle),
                    source_inner,
                    &mut *impulse_target.add(target_bundle),
                    target_inner,
                );

                // Update index to handle and handle-to-constraint mapping
                let handle = *index_to_handle_cache.get(source_index as i32);
                *type_batch.index_to_handle.get_mut(target_index as i32) = handle;
                let location = handles_to_constraints.get_mut(handle.0);
                location.index_in_type_batch = target_index as i32;
            }
        }
    }

    fn gather_active_constraints(
        &self,
        bodies: &crate::physics::bodies::Bodies,
        solver: &crate::physics::solver::Solver,
        source_scaffold: &crate::physics::island_scaffold::IslandScaffoldTypeBatch,
        start_index: i32,
        end_index: i32,
        target_type_batch: &mut TypeBatch,
    ) {
        unsafe {
            let active_set = solver.active_set();
            let active_body_set = bodies.active_set();
            let vector_shift = crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
            let vector_mask = crate::utilities::bundle_indexing::VECTOR_MASK;

            for i in start_index..end_index {
                let source_handle_value = *source_scaffold.handles.get(i);
                let source_handle = ConstraintHandle(source_handle_value);
                *target_type_batch.index_to_handle.get_mut(i) = source_handle;
                let location = solver.handle_to_constraint.get(source_handle.0);

                let source_batch = active_set.batches.get(location.batch_index);
                let source_type_batch_index = *source_batch
                    .type_index_to_type_batch_index
                    .get(location.type_id);
                let source_type_batch = source_batch.type_batches.get(source_type_batch_index);

                let source_bundle = (location.index_in_type_batch as usize) >> vector_shift;
                let source_inner = (location.index_in_type_batch as usize) & vector_mask;
                let target_bundle = (i as usize) >> vector_shift;
                let target_inner = (i as usize) & vector_mask;

                // Copy prestep data lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TPrestepData>(
                    &*(source_type_batch.prestep_data.as_ptr() as *const TPrestepData)
                        .add(source_bundle),
                    source_inner,
                    &mut *(target_type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData)
                        .add(target_bundle),
                    target_inner,
                );

                // Copy accumulated impulses lane
                crate::utilities::gather_scatter::GatherScatter::copy_lane::<TAccumulatedImpulse>(
                    &*(source_type_batch.accumulated_impulses.as_ptr()
                        as *const TAccumulatedImpulse)
                        .add(source_bundle),
                    source_inner,
                    &mut *(target_type_batch.accumulated_impulses.as_mut_ptr()
                        as *mut TAccumulatedImpulse)
                        .add(target_bundle),
                    target_inner,
                );

                // Copy body references, converting indices to handles
                let src_refs = &*(source_type_batch.body_references.as_ptr()
                    as *const TwoBodyReferences)
                    .add(source_bundle);
                let dst_refs = &mut *(target_type_batch.body_references.as_mut_ptr()
                    as *mut TwoBodyReferences)
                    .add(target_bundle);

                let encoded_index_a = src_refs.index_a[source_inner];
                let encoded_index_b = src_refs.index_b[source_inner];
                dst_refs.index_a[target_inner] = active_body_set
                    .index_to_handle
                    .get(encoded_index_a & Bodies::BODY_REFERENCE_MASK)
                    .0
                    | (encoded_index_a & (Bodies::KINEMATIC_MASK as i32));
                dst_refs.index_b[target_inner] = active_body_set
                    .index_to_handle
                    .get(encoded_index_b & Bodies::BODY_REFERENCE_MASK)
                    .0
                    | (encoded_index_b & (Bodies::KINEMATIC_MASK as i32));
            }
        }
    }

    fn copy_sleeping_to_active(
        &self,
        source_set: i32,
        batch_index: i32,
        source_type_batch_index: i32,
        target_type_batch_index: i32,
        source_start: i32,
        target_start: i32,
        count: i32,
        bodies: &crate::physics::bodies::Bodies,
        solver: &crate::physics::solver::Solver,
    ) {
        unsafe {
            let source_type_batch = solver
                .sets
                .get(source_set)
                .batches
                .get(batch_index)
                .type_batches
                .get(source_type_batch_index);
            let active_set_ptr =
                solver.sets.as_ptr() as *mut crate::physics::constraint_set::ConstraintSet;
            let target_type_batch = (*active_set_ptr)
                .batches
                .get_mut(batch_index)
                .type_batches
                .get_mut(target_type_batch_index);

            let vector_shift = crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
            let vector_mask = crate::utilities::bundle_indexing::VECTOR_MASK;
            let vector_width = crate::utilities::vector::VECTOR_WIDTH as i32;

            // Check if aligned bulk copy is possible
            if (source_start as usize & vector_mask) == 0
                && (target_start as usize & vector_mask) == 0
                && ((count as usize & vector_mask) == 0
                    || count == target_type_batch.constraint_count)
            {
                let bundle_count =
                    crate::utilities::bundle_indexing::BundleIndexing::get_bundle_count(
                        count as usize,
                    );
                let source_bundle_start = (source_start as usize) >> vector_shift;
                let target_bundle_start = (target_start as usize) >> vector_shift;
                let prestep_size = std::mem::size_of::<TPrestepData>();
                let impulse_size = std::mem::size_of::<TAccumulatedImpulse>();
                std::ptr::copy_nonoverlapping(
                    source_type_batch
                        .prestep_data
                        .as_ptr()
                        .add(prestep_size * source_bundle_start),
                    target_type_batch
                        .prestep_data
                        .as_mut_ptr()
                        .add(prestep_size * target_bundle_start),
                    prestep_size * bundle_count,
                );
                std::ptr::copy_nonoverlapping(
                    source_type_batch
                        .accumulated_impulses
                        .as_ptr()
                        .add(impulse_size * source_bundle_start),
                    target_type_batch
                        .accumulated_impulses
                        .as_mut_ptr()
                        .add(impulse_size * target_bundle_start),
                    impulse_size * bundle_count,
                );
            } else {
                for i in 0..count {
                    let source_index = (source_start + i) as usize;
                    let target_index = (target_start + i) as usize;
                    let source_bundle = source_index >> vector_shift;
                    let source_inner = source_index & vector_mask;
                    let target_bundle = target_index >> vector_shift;
                    let target_inner = target_index & vector_mask;
                    crate::utilities::gather_scatter::GatherScatter::copy_lane::<TPrestepData>(
                        &*(source_type_batch.prestep_data.as_ptr() as *const TPrestepData)
                            .add(source_bundle),
                        source_inner,
                        &mut *(target_type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData)
                            .add(target_bundle),
                        target_inner,
                    );
                    crate::utilities::gather_scatter::GatherScatter::copy_lane::<TAccumulatedImpulse>(
                        &*(source_type_batch.accumulated_impulses.as_ptr()
                            as *const TAccumulatedImpulse)
                            .add(source_bundle),
                        source_inner,
                        &mut *(target_type_batch.accumulated_impulses.as_mut_ptr()
                            as *mut TAccumulatedImpulse)
                            .add(target_bundle),
                        target_inner,
                    );
                }
            }

            // Copy body references (converting handles -> indices) and update handle mappings
            for i in 0..count {
                let source_index = source_start + i;
                let target_index = target_start + i;
                let source_bundle = (source_index as usize) >> vector_shift;
                let source_inner = (source_index as usize) & vector_mask;
                let target_bundle = (target_index as usize) >> vector_shift;
                let target_inner = (target_index as usize) & vector_mask;

                let src_refs = &*(source_type_batch.body_references.as_ptr()
                    as *const TwoBodyReferences)
                    .add(source_bundle);
                let dst_refs = &mut *(target_type_batch.body_references.as_mut_ptr()
                    as *mut TwoBodyReferences)
                    .add(target_bundle);

                let encoded_handle_a = src_refs.index_a[source_inner];
                let encoded_handle_b = src_refs.index_b[source_inner];
                dst_refs.index_a[target_inner] = bodies
                    .handle_to_location
                    .get(encoded_handle_a & Bodies::BODY_REFERENCE_MASK)
                    .index
                    | (encoded_handle_a & (Bodies::KINEMATIC_MASK as i32));
                dst_refs.index_b[target_inner] = bodies
                    .handle_to_location
                    .get(encoded_handle_b & Bodies::BODY_REFERENCE_MASK)
                    .index
                    | (encoded_handle_b & (Bodies::KINEMATIC_MASK as i32));

                let constraint_handle = *source_type_batch.index_to_handle.get(source_index);
                let location = &mut *(solver.handle_to_constraint.as_ptr()
                    as *mut ConstraintLocation)
                    .add(constraint_handle.0 as usize);
                location.set_index = 0;
                location.batch_index = batch_index;
                location.index_in_type_batch = target_index;
                *target_type_batch.index_to_handle.get_mut(target_index) = constraint_handle;
            }
        }
    }

    fn add_sleeping_to_active_for_fallback(
        &self,
        source_set: i32,
        source_type_batch_index: i32,
        target_type_batch_index: i32,
        bodies: &crate::physics::bodies::Bodies,
        solver: &crate::physics::solver::Solver,
    ) {
        unsafe {
            let fallback_batch_index = solver.fallback_batch_threshold();
            let source_type_batch = solver
                .sets
                .get(source_set)
                .batches
                .get(fallback_batch_index)
                .type_batches
                .get(source_type_batch_index);
            let active_set_ptr =
                solver.sets.as_ptr() as *mut crate::physics::constraint_set::ConstraintSet;
            let target_type_batch = (*active_set_ptr)
                .batches
                .get_mut(fallback_batch_index)
                .type_batches
                .get_mut(target_type_batch_index);

            let vector_shift = crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
            let vector_mask = crate::utilities::bundle_indexing::VECTOR_MASK;
            let vector_width = crate::utilities::vector::VECTOR_WIDTH as i32;
            let source_bundle_count =
                crate::utilities::bundle_indexing::BundleIndexing::get_bundle_count(
                    source_type_batch.constraint_count as usize,
                );

            for bundle_index_in_source in 0..source_bundle_count {
                let bundle_start_constraint_index = (bundle_index_in_source as i32) * vector_width;
                let mut count_in_bundle =
                    source_type_batch.constraint_count - bundle_start_constraint_index;
                if count_in_bundle > vector_width {
                    count_in_bundle = vector_width;
                }
                for source_inner_index in 0..count_in_bundle {
                    let source_index = bundle_start_constraint_index + source_inner_index;
                    let src_refs = &*(source_type_batch.body_references.as_ptr()
                        as *const TwoBodyReferences)
                        .add(bundle_index_in_source);

                    let encoded_handle_a = src_refs.index_a[source_inner_index as usize];
                    let encoded_handle_b = src_refs.index_b[source_inner_index as usize];
                    let body_handle_a = encoded_handle_a & Bodies::BODY_REFERENCE_MASK;
                    let body_handle_b = encoded_handle_b & Bodies::BODY_REFERENCE_MASK;
                    let body_indices = [
                        bodies.handle_to_location.get(body_handle_a).index
                            | (encoded_handle_a & (Bodies::KINEMATIC_MASK as i32)),
                        bodies.handle_to_location.get(body_handle_b).index
                            | (encoded_handle_b & (Bodies::KINEMATIC_MASK as i32)),
                    ];

                    let handle = *source_type_batch.index_to_handle.get(source_index);
                    let target_index = self.allocate_in_type_batch_for_fallback(
                        target_type_batch,
                        handle,
                        &body_indices,
                        std::ptr::null_mut(),
                    );
                    let target_bundle = (target_index as usize) >> vector_shift;
                    let target_inner = (target_index as usize) & vector_mask;

                    // Copy accumulated impulses lane
                    crate::utilities::gather_scatter::GatherScatter::copy_lane::<TAccumulatedImpulse>(
                        &*(source_type_batch.accumulated_impulses.as_ptr()
                            as *const TAccumulatedImpulse)
                            .add(bundle_index_in_source),
                        source_inner_index as usize,
                        &mut *(target_type_batch.accumulated_impulses.as_mut_ptr()
                            as *mut TAccumulatedImpulse)
                            .add(target_bundle),
                        target_inner,
                    );

                    // Copy prestep data lane
                    crate::utilities::gather_scatter::GatherScatter::copy_lane::<TPrestepData>(
                        &*(source_type_batch.prestep_data.as_ptr() as *const TPrestepData)
                            .add(bundle_index_in_source),
                        source_inner_index as usize,
                        &mut *(target_type_batch.prestep_data.as_mut_ptr() as *mut TPrestepData)
                            .add(target_bundle),
                        target_inner,
                    );

                    // Update constraint location
                    let location = &mut *(solver.handle_to_constraint.as_ptr()
                        as *mut ConstraintLocation)
                        .add(handle.0 as usize);
                    location.set_index = 0;
                    location.batch_index = fallback_batch_index;
                    location.index_in_type_batch = target_index;
                }
            }
        }
    }

    fn add_waking_body_handles_to_batch_references(
        &self,
        type_batch: &TypeBatch,
        target_batch_referenced_handles: &mut IndexSet,
    ) {
        unsafe {
            let vector_width = crate::utilities::vector::VECTOR_WIDTH;
            let body_refs = type_batch.body_references.as_ptr() as *const TwoBodyReferences;
            for i in 0..type_batch.constraint_count {
                let bundle_index = (i as usize)
                    >> crate::utilities::bundle_indexing::BundleIndexing::vector_shift();
                let inner_index = (i as usize) & crate::utilities::bundle_indexing::VECTOR_MASK;
                let refs = &*body_refs.add(bundle_index);
                let encoded_a = refs.index_a[inner_index];
                if Bodies::is_encoded_dynamic_reference(encoded_a) {
                    target_batch_referenced_handles.set_unsafely(encoded_a);
                }
                let encoded_b = refs.index_b[inner_index];
                if Bodies::is_encoded_dynamic_reference(encoded_b) {
                    target_batch_referenced_handles.set_unsafely(encoded_b);
                }
            }
        }
    }

    fn get_body_reference_count(&self, type_batch: &TypeBatch, body_to_find: i32) -> i32 {
        unsafe {
            let bundle_count = crate::utilities::bundle_indexing::BundleIndexing::get_bundle_count(
                type_batch.constraint_count as usize,
            );
            let body_references = type_batch.body_references.as_ptr() as *const TwoBodyReferences;
            let mut count = 0;
            let vector_width = crate::utilities::vector::VECTOR_WIDTH;
            for bundle_index in 0..bundle_count {
                let bundle_size = std::cmp::min(
                    vector_width,
                    type_batch.constraint_count as usize
                        - (bundle_index
                            << crate::utilities::bundle_indexing::BundleIndexing::vector_shift()),
                );
                let refs = &*body_references.add(bundle_index);
                for inner_index in 0..bundle_size {
                    if refs.index_a[inner_index] == body_to_find {
                        count += 1;
                    }
                    if refs.index_b[inner_index] == body_to_find {
                        count += 1;
                    }
                }
            }
            count
        }
    }
}
