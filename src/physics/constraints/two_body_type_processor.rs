// Translated from BepuPhysics/Constraints/TwoBodyTypeProcessor.cs (trait only)
//
// The C# TwoBodyTypeProcessor is an abstract generic class. In Rust, we separate the constraint
// function interface from the loop logic.

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

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
