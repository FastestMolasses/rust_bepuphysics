// Translated from BepuPhysics/PoseIntegrator.cs
// Note: The static helper functions (PoseIntegration) are already translated in pose_integration.rs.
// This file contains the PoseIntegrator<TCallbacks> generic struct that handles body integration work.

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{
    BodyActivity, BodyInertiaWide, BodyVelocity, BodyVelocityWide, RigidPose,
};
use crate::physics::collidables::collidable::Collidable;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::constraints::body_access_filter::{AccessAll, AccessNoInertia};
use crate::physics::handles::BodyHandle;
use crate::physics::helpers::Helpers;
use crate::physics::pose_integration::{
    AngularIntegrationMode, IPoseIntegratorCallbacks, PoseIntegration,
};
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;

use crate::physics::bounding_box_batcher::BoundingBoxBatcher;
use crate::physics::collision_detection::broad_phase::BroadPhase;

/// Trait for pose integrator dispatch.
pub trait IPoseIntegrator {
    fn predict_bounding_boxes(
        &mut self,
        dt: f32,
        pool: &mut BufferPool,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    );

    fn integrate_after_substepping(
        &mut self,
        constrained_bodies: &IndexSet,
        dt: f32,
        substep_count: i32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    );

    /// Whether the callbacks want to integrate velocity for kinematics.
    fn integrate_velocity_for_kinematics(&self) -> bool;

    /// Integrates the velocities of kinematic bodies (first substep only).
    unsafe fn integrate_kinematic_velocities_dispatch(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    );

    /// Integrates poses then velocities of kinematic bodies (substeps > 0).
    unsafe fn integrate_kinematic_poses_and_velocities_dispatch(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    );
}

// AngularIntegrationMode is imported from pose_integration above.

/// Handles body integration work that isn't bundled into the solver's execution.
/// Predicts bounding boxes, integrates velocity and poses for unconstrained bodies,
/// and does final post-substepping pose integration for constrained bodies.
pub struct PoseIntegrator<TCallbacks: IPoseIntegratorCallbacks> {
    bodies: *mut Bodies,
    shapes: *mut Shapes,
    broad_phase: *mut BroadPhase,

    pub callbacks: TCallbacks,

    cached_dt: f32,
    job_size: i32,
    substep_count: i32,
    available_job_count: std::cell::UnsafeCell<i32>,
    /// Stored for multi-threaded IntegrateAfterSubstepping dispatch.
    constrained_bodies: Option<IndexSet>,
}

impl<TCallbacks: IPoseIntegratorCallbacks> PoseIntegrator<TCallbacks> {
    pub fn new(
        bodies: *mut Bodies,
        shapes: *mut Shapes,
        broad_phase: *mut BroadPhase,
        callbacks: TCallbacks,
    ) -> Self {
        Self {
            bodies,
            shapes,
            broad_phase,
            callbacks,
            cached_dt: 0.0,
            job_size: 1,
            substep_count: 1,
            available_job_count: std::cell::UnsafeCell::new(0),
            constrained_bodies: None,
        }
    }

    #[inline(always)]
    fn update_sleep_candidacy(velocity_heuristic: f32, activity: &mut BodyActivity) {
        if velocity_heuristic > activity.sleep_threshold {
            activity.timesteps_under_threshold_count = 0;
            activity.sleep_candidate = false;
        } else {
            if activity.timesteps_under_threshold_count < u8::MAX {
                activity.timesteps_under_threshold_count += 1;
                if activity.timesteps_under_threshold_count
                    >= activity.minimum_timesteps_under_threshold
                {
                    activity.sleep_candidate = true;
                }
            }
        }
    }

    fn try_get_job(&self, maximum_job_interval: i32) -> Option<(i32, i32)> {
        let job_index = unsafe {
            use std::sync::atomic::{AtomicI32, Ordering};
            AtomicI32::from_ptr(self.available_job_count.get()).fetch_sub(1, Ordering::AcqRel) - 1
        };
        if job_index < 0 {
            return None;
        }
        let start = job_index * self.job_size;
        let mut exclusive_end = start + self.job_size;
        if exclusive_end > maximum_job_interval {
            exclusive_end = maximum_job_interval;
        }
        Some((start, exclusive_end))
    }

    fn prepare_for_multithreaded_execution(
        &mut self,
        loop_iteration_count: i32,
        dt: f32,
        worker_count: i32,
        substep_count: i32,
    ) {
        self.cached_dt = dt;
        self.substep_count = substep_count;
        const JOBS_PER_WORKER: i32 = 2;
        let target_job_count = worker_count * JOBS_PER_WORKER;
        self.job_size = loop_iteration_count / target_job_count;
        if self.job_size == 0 {
            self.job_size = 1;
        }
        let mut available = loop_iteration_count / self.job_size;
        if self.job_size * available < loop_iteration_count {
            available += 1;
        }
        unsafe {
            *self.available_job_count.get() = available;
        }
    }

    /// Inner per-bundle loop for bounding box prediction.
    /// Integrates velocities (for bounding box prediction only — results are not stored back),
    /// updates sleep candidacy, and feeds collidable bodies into the bounding box batcher.
    unsafe fn predict_bounding_boxes_inner(
        &self,
        start_bundle_index: i32,
        end_bundle_index: i32,
        dt: f32,
        bounding_box_batcher: &mut BoundingBoxBatcher,
        worker_index: i32,
    ) {
        let bodies = &*self.bodies;
        let active_set = bodies.active_set();
        let activities_ptr = active_set.activity.as_ptr() as *mut BodyActivity;
        let collidables = &active_set.collidables;

        let mut lane_index_offsets: Vector<i32> = Simd::splat(0);
        Helpers::fill_vector_with_lane_indices(&mut lane_index_offsets);
        let dt_wide: Vector<f32> = Simd::splat(dt);
        let body_count = active_set.count;
        let lanes = Vector::<i32>::LEN as i32;

        for bundle_index in start_bundle_index..end_bundle_index {
            let bundle_start_body_index = bundle_index * lanes;
            let mut count_in_bundle = body_count - bundle_start_body_index;
            if count_in_bundle > lanes {
                count_in_bundle = lanes;
            }

            let lane_indices = Simd::splat(bundle_start_body_index) + lane_index_offsets;

            let mut position = Vector3Wide::default();
            let mut orientation = QuaternionWide::default();
            let mut velocity = BodyVelocityWide::default();
            let mut inertia = BodyInertiaWide::default();

            bodies.gather_state::<AccessAll>(
                &lane_indices,
                false,
                &mut position,
                &mut orientation,
                &mut velocity,
                &mut inertia,
            );

            let integration_mask;
            if self.callbacks.integrate_velocity_for_kinematics() {
                integration_mask =
                    BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize);
            } else {
                integration_mask =
                    BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize)
                        & !Bodies::is_kinematic_wide(&inertia);
            }

            // Fill empty lanes with -1 for consistent behavior with solver callbacks.
            let lane_indices = lane_indices | !integration_mask;
            let sleep_energy =
                velocity.linear.length_squared() + velocity.angular.length_squared();

            // Only call IntegrateVelocity if any lane is active (has a negative mask value).
            if integration_mask.simd_lt(Simd::splat(0i32)).any() {
                self.callbacks.integrate_velocity(
                    lane_indices,
                    position,
                    orientation,
                    inertia,
                    integration_mask,
                    worker_index,
                    dt_wide,
                    &mut velocity,
                );
            }

            for i in 0..count_in_bundle as usize {
                let body_index = (i as i32) + bundle_start_body_index;
                Self::update_sleep_candidacy(
                    sleep_energy[i],
                    &mut *activities_ptr.add(body_index as usize),
                );

                let mut body_pose = RigidPose {
                    position: glam::Vec3::ZERO,
                    orientation: glam::Quat::IDENTITY,
                };
                Vector3Wide::read_slot(&position, i, &mut body_pose.position);
                QuaternionWide::read_slot(&orientation, i, &mut body_pose.orientation);

                let mut body_velocity = BodyVelocity::new(glam::Vec3::ZERO, glam::Vec3::ZERO);
                Vector3Wide::read_slot(&velocity.linear, i, &mut body_velocity.linear);
                Vector3Wide::read_slot(&velocity.angular, i, &mut body_velocity.angular);

                bounding_box_batcher.add(
                    body_index,
                    &body_pose,
                    &body_velocity,
                    collidables.get(body_index),
                );
            }
        }
    }

    /// Integrates the velocities of kinematic bodies as a prepass to the first substep during solving.
    /// Kinematics have to be integrated ahead of time since they don't block constraint batch membership;
    /// the same kinematic could appear in the batch multiple times.
    pub unsafe fn integrate_kinematic_velocities(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    ) {
        let bodies = &*self.bodies;
        let body_count = body_handles.len();
        let bundle_dt: Vector<f32> = Simd::splat(substep_dt);
        let handle_to_location = &bodies.handle_to_location;
        let zero_inertia = BodyInertiaWide::default();
        let lanes = Vector::<f32>::LEN as i32;

        for bundle_index in bundle_start_index..bundle_end_index {
            let bundle_base_index = bundle_index * lanes;
            let count_in_bundle = (body_count - bundle_base_index).min(lanes);

            let mut body_indices_arr = [0i32; { crate::utilities::vector::optimal_lanes::<i32>() }];
            for i in 0..count_in_bundle as usize {
                body_indices_arr[i] =
                    handle_to_location
                        .get(*body_handles.get(bundle_base_index + i as i32))
                        .index;
            }

            let existing_mask =
                BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize);
            let trailing_mask = !existing_mask;
            let body_indices_vector = Simd::from_array(body_indices_arr) | trailing_mask;

            // Full gather so we can use the vectorized IntegrateVelocity callback.
            let mut position = Vector3Wide::default();
            let mut orientation = QuaternionWide::default();
            let mut velocity = BodyVelocityWide::default();
            let mut dummy_inertia = BodyInertiaWide::default();

            bodies.gather_state::<AccessNoInertia>(
                &body_indices_vector,
                false,
                &mut position,
                &mut orientation,
                &mut velocity,
                &mut dummy_inertia,
            );

            self.callbacks.integrate_velocity(
                body_indices_vector,
                position,
                orientation,
                zero_inertia,
                existing_mask,
                worker_index,
                bundle_dt,
                &mut velocity,
            );

            // Scatter velocities back. Empty lanes won't matter as scatter is masked by index.
            // Kinematic bodies have infinite inertia, so momentum conserving codepaths are skipped.
            bodies.scatter_velocities::<AccessAll>(&velocity, &body_indices_vector);
        }
    }

    /// Integrates the poses *then* velocities of kinematic bodies as a prepass to the second or later substeps during solving.
    /// Kinematics have to be integrated ahead of time since they don't block constraint batch membership;
    /// the same kinematic could appear in the batch multiple times.
    pub unsafe fn integrate_kinematic_poses_and_velocities(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    ) {
        let bodies = &*self.bodies;
        let body_count = body_handles.len();
        let bundle_dt: Vector<f32> = Simd::splat(substep_dt);
        let half_dt = bundle_dt * Simd::splat(0.5f32);
        let handle_to_location = &bodies.handle_to_location;
        let zero_inertia = BodyInertiaWide::default();
        let lanes = Vector::<f32>::LEN as i32;

        for bundle_index in bundle_start_index..bundle_end_index {
            let bundle_base_index = bundle_index * lanes;
            let count_in_bundle = (body_count - bundle_base_index).min(lanes);

            let mut body_indices_arr = [0i32; { crate::utilities::vector::optimal_lanes::<i32>() }];
            for i in 0..count_in_bundle as usize {
                body_indices_arr[i] =
                    handle_to_location
                        .get(*body_handles.get(bundle_base_index + i as i32))
                        .index;
            }

            let existing_mask =
                BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize);
            let trailing_mask = !existing_mask;
            let mut body_indices_vector = Simd::from_array(body_indices_arr);
            body_indices_vector = body_indices_vector | trailing_mask;

            let mut position = Vector3Wide::default();
            let mut orientation = QuaternionWide::default();
            let mut velocity = BodyVelocityWide::default();
            let mut dummy_inertia = BodyInertiaWide::default();

            bodies.gather_state::<AccessNoInertia>(
                &body_indices_vector,
                false,
                &mut position,
                &mut orientation,
                &mut velocity,
                &mut dummy_inertia,
            );

            // Note: we integrate pose THEN velocity. This executes in the context of the second (or later)
            // substep, effectively completing the previous substep's frame. Pose integration completes the
            // last substep; velocity integration prepares for the current substep.
            position += velocity.linear * bundle_dt;
            // Kinematic bodies have infinite inertia, so using the angular momentum conserving
            // codepaths would hit a singularity.
            let prev_orientation = orientation;
            PoseIntegration::integrate_orientation_wide(
                &prev_orientation,
                &velocity.angular,
                &half_dt,
                &mut orientation,
            );
            bodies.scatter_pose(&position, &orientation, &body_indices_vector, &existing_mask);

            if self.callbacks.integrate_velocity_for_kinematics() {
                self.callbacks.integrate_velocity(
                    body_indices_vector,
                    position,
                    orientation,
                    zero_inertia,
                    existing_mask,
                    worker_index,
                    bundle_dt,
                    &mut velocity,
                );
                // Writes to the empty lanes won't matter (scatter is masked), so we don't need to
                // clean them up.
                bodies.scatter_velocities::<AccessAll>(&velocity, &body_indices_vector);
            }
        }
    }

    /// Integrates poses and velocities for the final post-substepping pass.
    /// Constrained bodies get one substep of pose integration.
    /// Unconstrained bodies get full velocity integration (optionally substepped) plus pose integration.
    unsafe fn integrate_bundles_after_substepping(
        &self,
        merged_constrained_body_handles: &IndexSet,
        bundle_start_index: i32,
        bundle_end_index: i32,
        dt: f32,
        substep_dt: f32,
        substep_count: i32,
        worker_index: i32,
    ) {
        let bodies = &*self.bodies;
        let active_set = bodies.active_set();
        let body_count = active_set.count;
        let bundle_dt: Vector<f32> = Simd::splat(dt);
        let bundle_substep_dt: Vector<f32> = Simd::splat(substep_dt);
        let lanes = Vector::<f32>::LEN as i32;

        for i in bundle_start_index..bundle_end_index {
            let bundle_base_index = i * lanes;
            let mut count_in_bundle = body_count - bundle_base_index;
            if count_in_bundle > lanes {
                count_in_bundle = lanes;
            }

            let mut any_body_in_bundle_is_unconstrained = false;
            let mut unconstrained_mask_arr =
                [0i32; { crate::utilities::vector::optimal_lanes::<i32>() }];
            let mut body_indices_arr =
                [0i32; { crate::utilities::vector::optimal_lanes::<i32>() }];

            for inner_index in 0..count_in_bundle as usize {
                let body_index = bundle_base_index + inner_index as i32;
                body_indices_arr[inner_index] = body_index;
                let body_handle = active_set.index_to_handle.get(body_index).0;
                // Use the solver-merged body handles set for less memory bandwidth.
                if merged_constrained_body_handles.contains(body_handle) {
                    unconstrained_mask_arr[inner_index] = 0;
                } else {
                    unconstrained_mask_arr[inner_index] = -1;
                    any_body_in_bundle_is_unconstrained = true;
                }
            }

            let mut unconstrained_mask: Vector<i32> = Simd::from_array(unconstrained_mask_arr);
            let mut body_indices: Vector<i32> = Simd::from_array(body_indices_arr);

            if (count_in_bundle as usize) < Vector::<i32>::LEN {
                // Set empty body index lanes to -1 so inactive lanes are consistent with the
                // active set's storage of body references.
                let trailing_mask = BundleIndexing::create_trailing_mask_for_count_in_bundle(
                    count_in_bundle as usize,
                );
                body_indices = body_indices | trailing_mask;
                // Empty slots should not be considered; clear the mask slot.
                // C#: Vector.AndNot(unconstrained, trailing) => unconstrained & !trailing
                unconstrained_mask = unconstrained_mask & !trailing_mask;
            }

            let bundle_effective_dt: Vector<f32>;
            if self.callbacks.allow_substeps_for_unconstrained_bodies() {
                bundle_effective_dt = bundle_substep_dt;
            } else {
                // Unconstrained bodies use full dt, constrained use substep dt.
                let mask = Mask::from_int(unconstrained_mask);
                bundle_effective_dt = mask.select(bundle_dt, bundle_substep_dt);
            }
            let half_dt = bundle_effective_dt * Simd::splat(0.5f32);

            let mut position = Vector3Wide::default();
            let mut orientation = QuaternionWide::default();
            let mut velocity = BodyVelocityWide::default();
            let mut local_inertia = BodyInertiaWide::default();

            bodies.gather_state::<AccessAll>(
                &body_indices,
                false,
                &mut position,
                &mut orientation,
                &mut velocity,
                &mut local_inertia,
            );

            let unconstrained_velocity_integration_mask: Vector<i32>;
            let any_body_in_bundle_needs_velocity_integration: bool;
            if self.callbacks.integrate_velocity_for_kinematics() {
                unconstrained_velocity_integration_mask = unconstrained_mask;
                any_body_in_bundle_needs_velocity_integration = any_body_in_bundle_is_unconstrained;
            } else {
                let is_kinematic = Bodies::is_kinematic_wide(&local_inertia);
                unconstrained_velocity_integration_mask = unconstrained_mask & !is_kinematic;
                any_body_in_bundle_needs_velocity_integration =
                    unconstrained_velocity_integration_mask
                        .simd_lt(Simd::splat(0i32))
                        .any();
            }

            // Set all bits in non-velocity-integrated lanes so scatter will skip them.
            // This also keeps body indices passed into callbacks consistent (-1 for ignored slots).
            let velocity_masked_body_indices =
                body_indices | !unconstrained_velocity_integration_mask;

            if any_body_in_bundle_is_unconstrained {
                let integration_step_count: i32;
                if self.callbacks.allow_substeps_for_unconstrained_bodies() {
                    integration_step_count = substep_count;
                } else {
                    integration_step_count = 1;
                }

                for step_index in 0..integration_step_count {
                    // Note: integrates velocities, then poses.
                    let previous_velocity = velocity;

                    if any_body_in_bundle_needs_velocity_integration {
                        self.callbacks.integrate_velocity(
                            velocity_masked_body_indices,
                            position,
                            orientation,
                            local_inertia,
                            unconstrained_velocity_integration_mask,
                            worker_index,
                            bundle_effective_dt,
                            &mut velocity,
                        );
                        // Mask velocity writes to only unconstrained lanes so user doesn't have to.
                        velocity.linear = Vector3Wide::conditional_select(
                            &unconstrained_velocity_integration_mask,
                            &velocity.linear,
                            &previous_velocity.linear,
                        );
                        velocity.angular = Vector3Wide::conditional_select(
                            &unconstrained_velocity_integration_mask,
                            &velocity.angular,
                            &previous_velocity.angular,
                        );
                    }

                    position += velocity.linear * bundle_effective_dt;

                    // The full loop for constrained bodies with 3 substeps looks like:
                    // (velocity -> solve) -> (pose -> velocity -> solve) -> (pose -> velocity -> solve) -> pose
                    // For unconstrained bodies, it's a tight loop of:
                    // (velocity -> pose) -> (velocity -> pose) -> (velocity -> pose)
                    match self.callbacks.angular_integration_mode() {
                        AngularIntegrationMode::ConserveMomentum => {
                            let previous_orientation = orientation;
                            PoseIntegration::integrate_orientation_wide(
                                &previous_orientation,
                                &velocity.angular,
                                &half_dt,
                                &mut orientation,
                            );
                            let mut inverse_inertia_tensor = Symmetric3x3Wide::default();
                            PoseIntegration::rotate_inverse_inertia_wide(
                                &local_inertia.inverse_inertia_tensor,
                                &orientation,
                                &mut inverse_inertia_tensor,
                            );
                            PoseIntegration::integrate_angular_velocity_conserve_momentum(
                                &previous_orientation,
                                &local_inertia.inverse_inertia_tensor,
                                &inverse_inertia_tensor,
                                &mut velocity.angular,
                            );
                        }
                        AngularIntegrationMode::ConserveMomentumWithGyroscopicTorque => {
                            let prev_orientation = orientation;
                            PoseIntegration::integrate_orientation_wide(
                                &prev_orientation,
                                &velocity.angular,
                                &half_dt,
                                &mut orientation,
                            );
                            PoseIntegration::integrate_angular_velocity_conserve_momentum_with_gyroscopic_torque(
                                &orientation,
                                &local_inertia.inverse_inertia_tensor,
                                &mut velocity.angular,
                                &bundle_effective_dt,
                            );
                        }
                        _ => {
                            // Nonconserving
                            let prev_orientation = orientation;
                            PoseIntegration::integrate_orientation_wide(
                                &prev_orientation,
                                &velocity.angular,
                                &half_dt,
                                &mut orientation,
                            );
                        }
                    }

                    let mut integrate_pose_mask =
                        BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize);
                    if self.callbacks.allow_substeps_for_unconstrained_bodies() {
                        if step_index > 0 {
                            // Only the first substep should integrate poses for constrained bodies,
                            // so mask them out for later substeps.
                            integrate_pose_mask = integrate_pose_mask & unconstrained_mask;
                        }
                    }
                    bodies.scatter_pose(
                        &position,
                        &orientation,
                        &body_indices,
                        &integrate_pose_mask,
                    );
                    if any_body_in_bundle_needs_velocity_integration {
                        bodies.scatter_velocities::<AccessAll>(
                            &velocity,
                            &velocity_masked_body_indices,
                        );
                    }
                }
            } else {
                // All bodies in the bundle are constrained — no velocity integration needed.
                // Just do one substep of pose integration.
                let prev_orientation = orientation;
                PoseIntegration::integrate_orientation_wide(
                    &prev_orientation,
                    &velocity.angular,
                    &half_dt,
                    &mut orientation,
                );
                position += velocity.linear * bundle_effective_dt;
                let integrate_pose_mask =
                    BundleIndexing::create_mask_for_count_in_bundle(count_in_bundle as usize);
                bodies.scatter_pose(
                    &position,
                    &orientation,
                    &body_indices,
                    &integrate_pose_mask,
                );
            }
        }
    }
}

impl<TCallbacks: IPoseIntegratorCallbacks> IPoseIntegrator for PoseIntegrator<TCallbacks> {
    fn predict_bounding_boxes(
        &mut self,
        dt: f32,
        pool: &mut BufferPool,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        self.callbacks.prepare_for_integration(dt);

        unsafe {
            let bodies = &*self.bodies;
            let bundle_count =
                BundleIndexing::get_bundle_count(bodies.active_set().count as usize) as i32;

            if let Some(td) = thread_dispatcher {
                self.prepare_for_multithreaded_execution(bundle_count, dt, td.thread_count(), 1);
                // NOTE: multi-threaded dispatch via threadDispatcher.DispatchWorkers not yet implemented.
                // For now, fall through to single-threaded path.
            }

            let mut bounding_box_batcher =
                BoundingBoxBatcher::new(self.bodies, self.shapes, self.broad_phase, pool as *mut BufferPool, dt);
            self.predict_bounding_boxes_inner(
                0,
                bundle_count,
                dt,
                &mut bounding_box_batcher,
                0,
            );
            bounding_box_batcher.flush();
        }
    }

    fn integrate_after_substepping(
        &mut self,
        constrained_bodies: &IndexSet,
        dt: f32,
        substep_count: i32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        // The only bodies undergoing *velocity* integration during the post-integration step
        // are unconstrained.
        let substep_dt = dt / substep_count as f32;
        let velocity_integration_timestep =
            if self.callbacks.allow_substeps_for_unconstrained_bodies() {
                substep_dt
            } else {
                dt
            };
        self.callbacks
            .prepare_for_integration(velocity_integration_timestep);

        unsafe {
            let bodies = &*self.bodies;
            let bundle_count =
                BundleIndexing::get_bundle_count(bodies.active_set().count as usize) as i32;

            if let Some(td) = thread_dispatcher {
                if td.thread_count() > 1 {
                    self.prepare_for_multithreaded_execution(
                        bundle_count,
                        dt,
                        td.thread_count(),
                        substep_count,
                    );
                    self.constrained_bodies = Some(*constrained_bodies);
                    // NOTE: multi-threaded dispatch via threadDispatcher.DispatchWorkers not yet implemented.
                    self.constrained_bodies = None;
                }
            }

            self.integrate_bundles_after_substepping(
                constrained_bodies,
                0,
                bundle_count,
                dt,
                substep_dt,
                substep_count,
                0,
            );
        }
    }

    fn integrate_velocity_for_kinematics(&self) -> bool {
        self.callbacks.integrate_velocity_for_kinematics()
    }

    unsafe fn integrate_kinematic_velocities_dispatch(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    ) {
        self.integrate_kinematic_velocities(
            body_handles,
            bundle_start_index,
            bundle_end_index,
            substep_dt,
            worker_index,
        );
    }

    unsafe fn integrate_kinematic_poses_and_velocities_dispatch(
        &self,
        body_handles: &Buffer<i32>,
        bundle_start_index: i32,
        bundle_end_index: i32,
        substep_dt: f32,
        worker_index: i32,
    ) {
        self.integrate_kinematic_poses_and_velocities(
            body_handles,
            bundle_start_index,
            bundle_end_index,
            substep_dt,
            worker_index,
        );
    }
}

unsafe impl<T: IPoseIntegratorCallbacks> Send for PoseIntegrator<T> {}
unsafe impl<T: IPoseIntegratorCallbacks> Sync for PoseIntegrator<T> {}
