// Translated from BepuPhysics/Constraints/TypeProcessor.cs
// Contains the GatherAndIntegrate, BundleShouldIntegrate, IntegratePoseAndVelocity,
// IntegrateVelocity helper functions used during integration-aware warm starts.
//
// In C# these are static methods on the abstract TypeProcessor class.
// In Rust they are free functions since they're shared across all type processor implementations.

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::batch_integration_mode::{
    BatchIntegrationMode, BundleIntegrationMode,
};
use crate::physics::constraints::body_access_filter::{AccessAll, IBodyAccessFilter};
use crate::physics::pose_integration::{AngularIntegrationMode, PoseIntegration};
use crate::utilities::collections::index_set::IndexSet;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Checks whether a bundle of constraints should integrate, returning a per-lane mask.
/// Corresponds to C# `TypeProcessor.BundleShouldIntegrate`.
#[inline(always)]
pub fn bundle_should_integrate(
    bundle_index: i32,
    integration_flags: &IndexSet,
) -> (BundleIntegrationMode, Vector<i32>) {
    let vector_count = Vector::<f32>::LEN;
    let constraint_start_index = bundle_index as usize * vector_count;
    let flag_bundle_index = constraint_start_index >> 6;
    let flag_inner_index = constraint_start_index - (flag_bundle_index << 6);
    let flag_mask = ((1u64 << vector_count) - 1) as i32;
    let scalar_integration_mask =
        ((unsafe { *integration_flags.flags.get(flag_bundle_index as i32) } >> flag_inner_index) as i32) & flag_mask;

    if scalar_integration_mask == flag_mask {
        // All lanes need integration.
        (BundleIntegrationMode::All, Vector::<i32>::splat(-1))
    } else if scalar_integration_mask > 0 {
        // Partial: expand bitstring into a vector mask.
        let integration_mask = Vector::<i32>::from_array(std::array::from_fn(|i| {
            if (scalar_integration_mask & (1 << i)) != 0 {
                -1
            } else {
                0
            }
        }));
        (BundleIntegrationMode::Partial, integration_mask)
    } else {
        // No integration needed.
        (BundleIntegrationMode::None, Vector::<i32>::splat(0))
    }
}

/// Takes body indices that could include metadata like kinematic flags in their upper bits
/// and returns indices with those flags stripped and with any lanes masked out by the
/// integrationMask set to -1.
#[inline(always)]
pub fn decode_body_indices(
    encoded_body_indices: &Vector<i32>,
    integration_mask: &Vector<i32>,
) -> Vector<i32> {
    let mask = Vector::<i32>::splat(Bodies::BODY_REFERENCE_MASK);
    let masked = *encoded_body_indices & mask;
    let ones_complement_mask = !*integration_mask;
    masked | ones_complement_mask
}

/// Integrates pose and velocity for bodies that need integration during warm start.
/// Corresponds to C# `TypeProcessor.IntegratePoseAndVelocity`.
///
/// This is called for substeps > 0 (AllowPoseIntegration). It:
/// 1. Integrates position (linear velocity * dt)
/// 2. Integrates orientation (angular velocity half-step)
/// 3. Recomputes world inverse inertia from new orientation
/// 4. Optionally conserves angular momentum
/// 5. Calls the velocity integration callback
/// 6. Masks results to only affect lanes marked for integration
#[inline(always)]
pub unsafe fn integrate_pose_and_velocity(
    angular_integration_mode: AngularIntegrationMode,
    integrate_velocity_fn: &dyn Fn(
        Vector<i32>,  // body_indices
        Vector3Wide,  // position
        QuaternionWide, // orientation
        BodyInertiaWide, // local_inertia
        Vector<i32>,  // integration_mask
        i32,          // worker_index
        Vector<f32>,  // dt
        &mut BodyVelocityWide, // velocity
    ),
    body_indices: &Vector<i32>,
    local_inertia: &BodyInertiaWide,
    dt: f32,
    integration_mask: &Vector<i32>,
    position: &mut Vector3Wide,
    orientation: &mut QuaternionWide,
    velocity: &mut BodyVelocityWide,
    worker_index: i32,
    inertia_out: &mut BodyInertiaWide,
) {
    let dt_wide = Vector::<f32>::splat(dt);

    // Integrate position: new_position = position + velocity.linear * dt
    let new_position = Vector3Wide {
        x: position.x + velocity.linear.x * dt_wide,
        y: position.y + velocity.linear.y * dt_wide,
        z: position.z + velocity.linear.z * dt_wide,
    };
    // Only apply position update to lanes that need integration
    *position = Vector3Wide::conditional_select(integration_mask, &new_position, position);

    inertia_out.inverse_mass = local_inertia.inverse_mass;
    let previous_velocity = velocity.clone();

    let half_dt_wide = dt_wide * Vector::<f32>::splat(0.5);

    match angular_integration_mode {
        AngularIntegrationMode::ConserveMomentum => {
            let previous_orientation = orientation.clone();
            let mut new_orientation = QuaternionWide::default();
            PoseIntegration::integrate_orientation_wide(
                orientation,
                &velocity.angular,
                &half_dt_wide,
                &mut new_orientation,
            );
            *orientation =
                QuaternionWide::conditional_select(*integration_mask, &new_orientation, orientation);
            PoseIntegration::rotate_inverse_inertia_wide(
                &local_inertia.inverse_inertia_tensor,
                orientation,
                &mut inertia_out.inverse_inertia_tensor,
            );
            PoseIntegration::integrate_angular_velocity_conserve_momentum(
                &previous_orientation,
                &local_inertia.inverse_inertia_tensor,
                &inertia_out.inverse_inertia_tensor,
                &mut velocity.angular,
            );
        }
        AngularIntegrationMode::ConserveMomentumWithGyroscopicTorque => {
            let mut new_orientation = QuaternionWide::default();
            PoseIntegration::integrate_orientation_wide(
                orientation,
                &velocity.angular,
                &half_dt_wide,
                &mut new_orientation,
            );
            *orientation =
                QuaternionWide::conditional_select(*integration_mask, &new_orientation, orientation);
            PoseIntegration::rotate_inverse_inertia_wide(
                &local_inertia.inverse_inertia_tensor,
                orientation,
                &mut inertia_out.inverse_inertia_tensor,
            );
            PoseIntegration::integrate_angular_velocity_conserve_momentum_with_gyroscopic_torque(
                orientation,
                &local_inertia.inverse_inertia_tensor,
                &mut velocity.angular,
                &dt_wide,
            );
        }
        AngularIntegrationMode::Nonconserving => {
            let mut new_orientation = QuaternionWide::default();
            PoseIntegration::integrate_orientation_wide(
                orientation,
                &velocity.angular,
                &half_dt_wide,
                &mut new_orientation,
            );
            *orientation =
                QuaternionWide::conditional_select(*integration_mask, &new_orientation, orientation);
            PoseIntegration::rotate_inverse_inertia_wide(
                &local_inertia.inverse_inertia_tensor,
                orientation,
                &mut inertia_out.inverse_inertia_tensor,
            );
        }
    }

    // Call the velocity integration callback
    integrate_velocity_fn(
        *body_indices,
        position.clone(),
        orientation.clone(),
        local_inertia.clone(),
        *integration_mask,
        worker_index,
        dt_wide,
        velocity,
    );

    // Mask velocity changes to only affect lanes that need integration
    velocity.linear =
        Vector3Wide::conditional_select(integration_mask, &velocity.linear, &previous_velocity.linear);
    velocity.angular =
        Vector3Wide::conditional_select(integration_mask, &velocity.angular, &previous_velocity.angular);
}

/// Integrates velocity only (no pose), used during the first substep (DisallowPoseIntegration).
/// Corresponds to C# `TypeProcessor.IntegrateVelocity`.
#[inline(always)]
pub unsafe fn integrate_velocity(
    angular_integration_mode: AngularIntegrationMode,
    batch_integration_mode: BatchIntegrationMode,
    integrate_velocity_fn: &dyn Fn(
        Vector<i32>,
        Vector3Wide,
        QuaternionWide,
        BodyInertiaWide,
        Vector<i32>,
        i32,
        Vector<f32>,
        &mut BodyVelocityWide,
    ),
    body_indices: &Vector<i32>,
    local_inertia: &BodyInertiaWide,
    dt: f32,
    integration_mask: &Vector<i32>,
    position: &Vector3Wide,
    orientation: &QuaternionWide,
    velocity: &mut BodyVelocityWide,
    worker_index: i32,
    inertia_out: &mut BodyInertiaWide,
) {
    inertia_out.inverse_mass = local_inertia.inverse_mass;
    PoseIntegration::rotate_inverse_inertia_wide(
        &local_inertia.inverse_inertia_tensor,
        orientation,
        &mut inertia_out.inverse_inertia_tensor,
    );

    let dt_wide = Vector::<f32>::splat(dt);

    match angular_integration_mode {
        AngularIntegrationMode::ConserveMomentum => {
            // Integrate backwards to get a previous orientation
            let neg_half_dt = Vector::<f32>::splat(dt * -0.5);
            let mut previous_orientation = QuaternionWide::default();
            PoseIntegration::integrate_orientation_wide(
                orientation,
                &velocity.angular,
                &neg_half_dt,
                &mut previous_orientation,
            );
            PoseIntegration::integrate_angular_velocity_conserve_momentum(
                &previous_orientation,
                &local_inertia.inverse_inertia_tensor,
                &inertia_out.inverse_inertia_tensor,
                &mut velocity.angular,
            );
        }
        AngularIntegrationMode::ConserveMomentumWithGyroscopicTorque => {
            PoseIntegration::integrate_angular_velocity_conserve_momentum_with_gyroscopic_torque(
                orientation,
                &local_inertia.inverse_inertia_tensor,
                &mut velocity.angular,
                &dt_wide,
            );
        }
        AngularIntegrationMode::Nonconserving => {
            // No special angular velocity handling needed
        }
    }

    if batch_integration_mode == BatchIntegrationMode::Conditional {
        // Conditional: must mask velocity changes
        let previous_velocity = velocity.clone();
        integrate_velocity_fn(
            *body_indices,
            position.clone(),
            orientation.clone(),
            local_inertia.clone(),
            *integration_mask,
            worker_index,
            dt_wide,
            velocity,
        );
        velocity.linear = Vector3Wide::conditional_select(
            integration_mask,
            &velocity.linear,
            &previous_velocity.linear,
        );
        velocity.angular = Vector3Wide::conditional_select(
            integration_mask,
            &velocity.angular,
            &previous_velocity.angular,
        );
    } else {
        // Always: no masking needed
        integrate_velocity_fn(
            *body_indices,
            position.clone(),
            orientation.clone(),
            local_inertia.clone(),
            *integration_mask,
            worker_index,
            dt_wide,
            velocity,
        );
    }
}

/// Builds an integration mask from encoded body indices. Lanes with empty (-1)
/// or kinematic body indices are masked out (set to 0). Dynamic lanes are set to -1.
#[inline(always)]
fn build_dynamic_integration_mask(encoded_body_indices: &Vector<i32>) -> Vector<i32> {
    Vector::<i32>::from_array(std::array::from_fn(|i| {
        if (encoded_body_indices[i] as u32) < Bodies::DYNAMIC_LIMIT {
            -1
        } else {
            0
        }
    }))
}

/// The full gather-and-integrate function that handles all 6 integration paths.
/// Corresponds to C# `TypeProcessor.GatherAndIntegrate`.
///
/// The 6 paths are the cross-product of:
/// - allow_pose_integration: true (substep > 0) / false (substep 0)
/// - batch_integration_mode: Always / Never / Conditional
#[inline(always)]
pub unsafe fn gather_and_integrate<TAccessFilter: IBodyAccessFilter>(
    bodies: &Bodies,
    angular_integration_mode: AngularIntegrationMode,
    integrate_velocity_fn: &dyn Fn(
        Vector<i32>,
        Vector3Wide,
        QuaternionWide,
        BodyInertiaWide,
        Vector<i32>,
        i32,
        Vector<f32>,
        &mut BodyVelocityWide,
    ),
    integration_flags: &Buffer<IndexSet>,
    body_index_in_constraint: i32,
    batch_integration_mode: BatchIntegrationMode,
    allow_pose_integration: bool,
    dt: f32,
    worker_index: i32,
    bundle_index: i32,
    encoded_body_indices: &Vector<i32>,
    position: &mut Vector3Wide,
    orientation: &mut QuaternionWide,
    velocity: &mut BodyVelocityWide,
    inertia: &mut BodyInertiaWide,
) {
    if allow_pose_integration {
        // Substeps > 0: integrate pose + velocity
        match batch_integration_mode {
            BatchIntegrationMode::Always => {
                // All bodies in batch 0 are first-seen → always integrate.
                // Check for empty (-1) or kinematic slots.
                let integration_mask = build_dynamic_integration_mask(encoded_body_indices);

                bodies.gather_state::<AccessAll>(
                    encoded_body_indices,
                    false, // local inertia, not world
                    position,
                    orientation,
                    velocity,
                    inertia, // this will hold local inertia
                );
                let local_inertia = inertia.clone();
                let decoded_indices = decode_body_indices(encoded_body_indices, &integration_mask);
                integrate_pose_and_velocity(
                    angular_integration_mode,
                    integrate_velocity_fn,
                    &decoded_indices,
                    &local_inertia,
                    dt,
                    &integration_mask,
                    position,
                    orientation,
                    velocity,
                    worker_index,
                    inertia,
                );
                bodies.scatter_pose(position, orientation, encoded_body_indices, &integration_mask);
                bodies.scatter_inertia(inertia, encoded_body_indices, &integration_mask);
            }
            BatchIntegrationMode::Never => {
                // No integration needed; just gather with filtered access.
                bodies.gather_state::<TAccessFilter>(
                    encoded_body_indices,
                    true, // world inertia
                    position,
                    orientation,
                    velocity,
                    inertia,
                );
            }
            BatchIntegrationMode::Conditional => {
                // Check per-bundle integration flags.
                let flags = &*integration_flags.get(body_index_in_constraint);
                let (bundle_mode, integration_mask) =
                    bundle_should_integrate(bundle_index, flags);

                // Always gather with AccessAll since integration requires full state.
                bodies.gather_state::<AccessAll>(
                    encoded_body_indices,
                    bundle_mode == BundleIntegrationMode::None, // world inertia if no integration
                    position,
                    orientation,
                    velocity,
                    inertia,
                );

                if bundle_mode != BundleIntegrationMode::None {
                    let gathered_inertia = inertia.clone();
                    let decoded_indices = decode_body_indices(encoded_body_indices, &integration_mask);
                    integrate_pose_and_velocity(
                        angular_integration_mode,
                        integrate_velocity_fn,
                        &decoded_indices,
                        &gathered_inertia,
                        dt,
                        &integration_mask,
                        position,
                        orientation,
                        velocity,
                        worker_index,
                        inertia,
                    );
                    bodies.scatter_pose(position, orientation, encoded_body_indices, &integration_mask);
                    bodies.scatter_inertia(inertia, encoded_body_indices, &integration_mask);
                }
            }
        }
    } else {
        // Substep 0: velocity integration only, no pose changes.
        match batch_integration_mode {
            BatchIntegrationMode::Always => {
                let integration_mask = build_dynamic_integration_mask(encoded_body_indices);

                bodies.gather_state::<AccessAll>(
                    encoded_body_indices,
                    false, // local inertia
                    position,
                    orientation,
                    velocity,
                    inertia,
                );
                let local_inertia = inertia.clone();
                let decoded_indices = decode_body_indices(encoded_body_indices, &integration_mask);
                integrate_velocity(
                    angular_integration_mode,
                    BatchIntegrationMode::Always,
                    integrate_velocity_fn,
                    &decoded_indices,
                    &local_inertia,
                    dt,
                    &integration_mask,
                    position,
                    orientation,
                    velocity,
                    worker_index,
                    inertia,
                );
                bodies.scatter_inertia(inertia, encoded_body_indices, &integration_mask);
            }
            BatchIntegrationMode::Never => {
                bodies.gather_state::<TAccessFilter>(
                    encoded_body_indices,
                    true, // world inertia
                    position,
                    orientation,
                    velocity,
                    inertia,
                );
            }
            BatchIntegrationMode::Conditional => {
                let flags = &*integration_flags.get(body_index_in_constraint);
                let (bundle_mode, integration_mask) =
                    bundle_should_integrate(bundle_index, flags);

                bodies.gather_state::<AccessAll>(
                    encoded_body_indices,
                    bundle_mode == BundleIntegrationMode::None,
                    position,
                    orientation,
                    velocity,
                    inertia,
                );

                if bundle_mode != BundleIntegrationMode::None {
                    let gathered_inertia = inertia.clone();
                    let decoded_indices = decode_body_indices(encoded_body_indices, &integration_mask);
                    integrate_velocity(
                        angular_integration_mode,
                        BatchIntegrationMode::Conditional,
                        integrate_velocity_fn,
                        &decoded_indices,
                        &gathered_inertia,
                        dt,
                        &integration_mask,
                        position,
                        orientation,
                        velocity,
                        worker_index,
                        inertia,
                    );
                    bodies.scatter_inertia(inertia, encoded_body_indices, &integration_mask);
                }
            }
        }
    }
}
