// Translated from BepuPhysics/Constraints/FourBodyTypeProcessor.cs

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraint_location::ConstraintLocation;
use crate::physics::constraints::body_access_filter::{AccessAll, AccessOnlyVelocity, IBodyAccessFilter};
use crate::physics::constraints::body_references::FourBodyReferences;
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

/// Prestep, warm start and solve iteration functions for a four-body constraint type.
pub trait IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> {
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
        position_d: &Vector3Wide,
        orientation_d: &QuaternionWide,
        inertia_d: &BodyInertiaWide,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
        wsv_d: &mut BodyVelocityWide,
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
        position_d: &Vector3Wide,
        orientation_d: &QuaternionWide,
        inertia_d: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestepData,
        accumulated_impulses: &mut TAccumulatedImpulse,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
        wsv_d: &mut BodyVelocityWide,
    );

    fn requires_incremental_substep_updates() -> bool;

    fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        wsv_c: &BodyVelocityWide,
        wsv_d: &BodyVelocityWide,
        prestep_data: &mut TPrestepData,
    );
}

/// Generic four-body type processor implementation.
///
/// This corresponds to C#'s `FourBodyTypeProcessor<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, ...>`.
pub struct FourBodyTypeProcessorImpl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
    TSolveAccessFilterD: IBodyAccessFilter + 'static,
> {
    type_id: i32,
    constrained_degrees_of_freedom: i32,
    _marker: PhantomData<(TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC, TSolveAccessFilterD)>,
}

impl<
    TPrestepData: Copy + 'static,
    TAccumulatedImpulse: Copy + 'static,
    TConstraintFunctions: IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
    TSolveAccessFilterD: IBodyAccessFilter + 'static,
> FourBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC, TSolveAccessFilterD> {
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
    TConstraintFunctions: IFourBodyConstraintFunctions<TPrestepData, TAccumulatedImpulse> + 'static,
    TSolveAccessFilterA: IBodyAccessFilter + 'static,
    TSolveAccessFilterB: IBodyAccessFilter + 'static,
    TSolveAccessFilterC: IBodyAccessFilter + 'static,
    TSolveAccessFilterD: IBodyAccessFilter + 'static,
> ITypeProcessor for FourBodyTypeProcessorImpl<TPrestepData, TAccumulatedImpulse, TConstraintFunctions, TSolveAccessFilterA, TSolveAccessFilterB, TSolveAccessFilterC, TSolveAccessFilterD> {
    #[inline(always)]
    fn bodies_per_constraint(&self) -> i32 {
        4
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
                4,
                std::mem::size_of::<FourBodyReferences>(),
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
        pool: &mut BufferPool,
    ) -> i32 {
        unsafe {
            type_batch_alloc::allocate_in_type_batch_for_fallback(
                type_batch,
                handle,
                encoded_body_indices,
                pool,
                4, // bodies_per_constraint
                std::mem::size_of::<FourBodyReferences>(),
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
                    FourBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 4);
            } else {
                type_batch_alloc::remove_from_type_batch::<
                    FourBodyReferences,
                    TPrestepData,
                    TAccumulatedImpulse,
                >(type_batch, index, handles_to_constraints, 4);
            }
        }
    }

    fn initialize(&self, type_batch: &mut TypeBatch, initial_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::initialize_type_batch(
            type_batch,
            self.type_id,
            initial_capacity,
            pool,
            std::mem::size_of::<FourBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
    }

    fn resize(&self, type_batch: &mut TypeBatch, new_capacity: i32, pool: &mut BufferPool) {
        type_batch_alloc::resize_type_batch(
            type_batch,
            new_capacity,
            pool,
            std::mem::size_of::<FourBodyReferences>(),
            std::mem::size_of::<TPrestepData>(),
            std::mem::size_of::<TAccumulatedImpulse>(),
        );
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut FourBodyReferences;
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

                let mut position_d = Vector3Wide::default();
                let mut orientation_d = QuaternionWide::default();
                let mut wsv_d = BodyVelocityWide::default();
                let mut inertia_d = BodyInertiaWide::default();
                bodies.gather_state::<AccessAll>(
                    &references.index_d, true,
                    &mut position_d, &mut orientation_d, &mut wsv_d, &mut inertia_d,
                );

                TConstraintFunctions::warm_start(
                    &position_a, &orientation_a, &inertia_a,
                    &position_b, &orientation_b, &inertia_b,
                    &position_c, &orientation_c, &inertia_c,
                    &position_d, &orientation_d, &inertia_d,
                    prestep, accumulated_impulses,
                    &mut wsv_a, &mut wsv_b, &mut wsv_c, &mut wsv_d,
                );

                bodies.scatter_velocities::<AccessAll>(&wsv_a, &references.index_a);
                bodies.scatter_velocities::<AccessAll>(&wsv_b, &references.index_b);
                bodies.scatter_velocities::<AccessAll>(&wsv_c, &references.index_c);
                bodies.scatter_velocities::<AccessAll>(&wsv_d, &references.index_d);
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut FourBodyReferences;
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

                let mut position_d = Vector3Wide::default();
                let mut orientation_d = QuaternionWide::default();
                let mut wsv_d = BodyVelocityWide::default();
                let mut inertia_d = BodyInertiaWide::default();
                bodies.gather_state::<TSolveAccessFilterD>(
                    &references.index_d, true,
                    &mut position_d, &mut orientation_d, &mut wsv_d, &mut inertia_d,
                );

                TConstraintFunctions::solve(
                    &position_a, &orientation_a, &inertia_a,
                    &position_b, &orientation_b, &inertia_b,
                    &position_c, &orientation_c, &inertia_c,
                    &position_d, &orientation_d, &inertia_d,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    &mut wsv_a, &mut wsv_b, &mut wsv_c, &mut wsv_d,
                );

                bodies.scatter_velocities::<TSolveAccessFilterA>(&wsv_a, &references.index_a);
                bodies.scatter_velocities::<TSolveAccessFilterB>(&wsv_b, &references.index_b);
                bodies.scatter_velocities::<TSolveAccessFilterC>(&wsv_c, &references.index_c);
                bodies.scatter_velocities::<TSolveAccessFilterD>(&wsv_d, &references.index_d);
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
            let body_ref_bundles = type_batch.body_references.as_mut_ptr() as *mut FourBodyReferences;
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

                let mut position_d = Vector3Wide::default();
                let mut orientation_d = QuaternionWide::default();
                let mut wsv_d = BodyVelocityWide::default();
                let mut inertia_d = BodyInertiaWide::default();
                bodies.gather_state::<AccessOnlyVelocity>(
                    &references.index_d, true,
                    &mut position_d, &mut orientation_d, &mut wsv_d, &mut inertia_d,
                );

                TConstraintFunctions::incrementally_update_for_substep(
                    &dt_wide, &wsv_a, &wsv_b, &wsv_c, &wsv_d, prestep,
                );
            }
        }
    }
}
