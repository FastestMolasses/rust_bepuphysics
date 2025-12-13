// Translated from BepuPhysics/Constraints/ThreeBodyTypeProcessor.cs (trait only)

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

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
