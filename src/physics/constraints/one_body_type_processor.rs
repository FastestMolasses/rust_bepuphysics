// Translated from BepuPhysics/Constraints/OneBodyTypeProcessor.cs
//
// The C# OneBodyTypeProcessor is an abstract generic class. In Rust, we separate the constraint
// function interface (IOneBodyConstraintFunctions) from the type processor loop logic
// (OneBodyTypeProcessorImpl).

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::body_access_filter::{AccessAll, AccessOnlyVelocity, IBodyAccessFilter};
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::ITypeProcessor;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::marker::PhantomData;

/// Prestep, warm start and solve iteration functions for a one-body constraint type.
pub trait IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> {
    fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
    );

    fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
    );

    /// Gets whether this constraint type requires incremental updates for each substep
    /// taken beyond the first.
    fn requires_incremental_substep_updates() -> bool;

    fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity: &BodyVelocityWide,
        prestep_data: &mut TPrestepData,
    );
}

/// Generic one-body type processor implementation that bridges the concrete constraint functions
/// to the solver's untyped `ITypeProcessor` interface.
///
/// This corresponds to C#'s `OneBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, ...>`.
pub struct OneBodyTypeProcessorImpl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
> {
    type_id: i32,
    constrained_degrees_of_freedom: i32,
    _marker: PhantomData<(TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA)>,
}

impl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
> OneBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA> {
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
    TConstraintFunctions: IOneBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
> ITypeProcessor for OneBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA> {
    #[inline(always)]
    fn bodies_per_constraint(&self) -> i32 {
        1
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
        _type_batch: &mut TypeBatch,
        _handle: ConstraintHandle,
        _encoded_body_indices: &[i32],
        _pool: &mut BufferPool,
    ) -> i32 {
        // TODO: implement typed allocation
        0
    }

    fn allocate_in_type_batch_for_fallback(
        &self,
        _type_batch: &mut TypeBatch,
        _handle: ConstraintHandle,
        _encoded_body_indices: &[i32],
        _pool: &mut BufferPool,
    ) -> i32 {
        // TODO: implement typed fallback allocation
        0
    }

    fn remove(
        &self,
        _type_batch: &mut TypeBatch,
        _index: i32,
        _handles_to_constraints: &mut Buffer<ConstraintLocation>,
        _is_fallback: bool,
    ) {
        // TODO: implement typed removal with swap-back
    }

    fn initialize(&self, _type_batch: &mut TypeBatch, _initial_capacity: i32, _pool: &mut BufferPool) {
        // TODO: implement typed TypeBatch initialization
    }

    fn resize(&self, _type_batch: &mut TypeBatch, _new_capacity: i32, _pool: &mut BufferPool) {
        // TODO: implement typed TypeBatch resize
    }

    fn scale_accumulated_impulses(&self, type_batch: &mut TypeBatch, scale: f32) {
        let byte_count = type_batch.accumulated_impulses.len() as usize;
        let float_count = byte_count / std::mem::size_of::<f32>();
        if float_count > 0 {
            let floats = unsafe {
                std::slice::from_raw_parts_mut(
                    type_batch.accumulated_impulses.as_mut_ptr() as *mut f32,
                    float_count,
                )
            };
            for f in floats.iter_mut() {
                *f *= scale;
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut Vector<i32>;
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
                    references, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                TConstraintFunctions::warm_start(
                    &position_a, &orientation_a, &inertia_a,
                    prestep, accumulated_impulses,
                    &mut wsv_a,
                );

                bodies.scatter_velocities::<AccessAll>(&wsv_a, references);
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut Vector<i32>;
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
                    references, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                TConstraintFunctions::solve(
                    &position_a, &orientation_a, &inertia_a,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    &mut wsv_a,
                );

                bodies.scatter_velocities::<TSolveAccessFilterA>(&wsv_a, references);
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut Vector<i32>;
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
                    references, true,
                    &mut position_a, &mut orientation_a, &mut wsv_a, &mut inertia_a,
                );

                TConstraintFunctions::incrementally_update_for_substep(
                    &dt_wide, &wsv_a, prestep,
                );
            }
        }
    }
}
