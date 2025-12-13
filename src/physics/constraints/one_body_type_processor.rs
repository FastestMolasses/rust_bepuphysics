// Translated from BepuPhysics/Constraints/OneBodyTypeProcessor.cs (trait + processor stub)
//
// The C# OneBodyTypeProcessor is an abstract generic class with WarmStart/Solve/IncrementallyUpdateForSubstep.
// In Rust, we separate the constraint function interface (IOneBodyConstraintFunctions) from the
// type processor loop logic. The loop logic is implemented by the concrete Solver and is not
// needed until the Solver is translated.

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

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
