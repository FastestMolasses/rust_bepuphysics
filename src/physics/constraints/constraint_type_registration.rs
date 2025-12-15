// Centralised constraint-type registration helpers.
//
// This file contains:
//   1.  Macros that generate the `IConstraintDescription` and
//       `ITwoBodyConstraintFunctions`/`IOneBodyConstraintFunctions` trait implementations for each
//       built-in constraint type.
//   2.  One invocation of those macros per constraint type, wiring every concrete `*Functions`
//       struct into the solver's trait-object–based dispatch.
//
// Adding a new constraint type only requires adding a single macro invocation here (plus the
// normal constraint-specific code in its own file).

// ── imports ────────────────────────────────────────────────────────────────────

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::body_access_filter::*;
use crate::physics::constraints::constraint_description::IConstraintDescription;
use crate::physics::constraints::two_body_type_processor::{
    ITwoBodyConstraintFunctions, TwoBodyTypeProcessorImpl,
};
use crate::physics::constraints::one_body_type_processor::{
    IOneBodyConstraintFunctions, OneBodyTypeProcessorImpl,
};
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::TypeProcessor;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

// ── macros ─────────────────────────────────────────────────────────────────────

/// Generates an `ITwoBodyConstraintFunctions` impl that delegates to existing inherent methods on
/// `$funcs`.  The inherent methods are shadowed (by Rust name-resolution rules) so the call inside
/// the trait body resolves to them, not to the trait method being defined.
macro_rules! impl_two_body_functions {
    ($funcs:ty, $prestep:ty, $impulse:ty) => {
        impl ITwoBodyConstraintFunctions<$prestep, $impulse> for $funcs {
            #[inline(always)]
            fn warm_start(
                position_a: &Vector3Wide,
                orientation_a: &QuaternionWide,
                inertia_a: &BodyInertiaWide,
                position_b: &Vector3Wide,
                orientation_b: &QuaternionWide,
                inertia_b: &BodyInertiaWide,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
            ) {
                <$funcs>::warm_start(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b,
                )
            }

            #[inline(always)]
            fn solve(
                position_a: &Vector3Wide,
                orientation_a: &QuaternionWide,
                inertia_a: &BodyInertiaWide,
                position_b: &Vector3Wide,
                orientation_b: &QuaternionWide,
                inertia_b: &BodyInertiaWide,
                dt: f32,
                inverse_dt: f32,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
            ) {
                <$funcs>::solve(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b,
                )
            }

            fn requires_incremental_substep_updates() -> bool {
                false
            }

            fn incrementally_update_for_substep(
                _dt: &Vector<f32>,
                _wsv_a: &BodyVelocityWide,
                _wsv_b: &BodyVelocityWide,
                _prestep_data: &mut $prestep,
            ) {
                // No incremental substep update needed for non-contact constraints.
            }
        }
    };
}

/// Generates an `IOneBodyConstraintFunctions` impl that delegates to existing inherent methods.
macro_rules! impl_one_body_functions {
    ($funcs:ty, $prestep:ty, $impulse:ty) => {
        impl IOneBodyConstraintFunctions<$prestep, $impulse> for $funcs {
            #[inline(always)]
            fn warm_start(
                position_a: &Vector3Wide,
                orientation_a: &QuaternionWide,
                inertia_a: &BodyInertiaWide,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
            ) {
                <$funcs>::warm_start(
                    position_a, orientation_a, inertia_a,
                    prestep, accumulated_impulses,
                    wsv_a,
                )
            }

            #[inline(always)]
            fn solve(
                position_a: &Vector3Wide,
                orientation_a: &QuaternionWide,
                inertia_a: &BodyInertiaWide,
                dt: f32,
                inverse_dt: f32,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
            ) {
                <$funcs>::solve(
                    position_a, orientation_a, inertia_a,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    wsv_a,
                )
            }

            fn requires_incremental_substep_updates() -> bool {
                false
            }

            fn incrementally_update_for_substep(
                _dt: &Vector<f32>,
                _velocity: &BodyVelocityWide,
                _prestep_data: &mut $prestep,
            ) {
            }
        }
    };
}

/// Generates an `IConstraintDescription` impl for a two-body constraint.
macro_rules! impl_two_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr, $solve_a:ty, $solve_b:ty) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, _batch: &mut TypeBatch, _bundle_index: i32, _inner_index: i32) {
                // TODO: implement typed apply (needed for adding constraints at runtime)
            }

            fn build_description(_batch: &TypeBatch, _bundle_index: i32, _inner_index: i32) -> Self {
                // TODO: implement typed build (needed for reading constraint data)
                unsafe { std::mem::zeroed() }
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        TwoBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, $solve_a, $solve_b>::new(
                            $type_id, $dof,
                        ),
                    ),
                )
            }
        }
    };
}

/// Generates an `IConstraintDescription` impl for a one-body constraint.
macro_rules! impl_one_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr, $solve_a:ty) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, _batch: &mut TypeBatch, _bundle_index: i32, _inner_index: i32) {
                // TODO: implement typed apply
            }

            fn build_description(_batch: &TypeBatch, _bundle_index: i32, _inner_index: i32) -> Self {
                unsafe { std::mem::zeroed() }
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        OneBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, $solve_a>::new(
                            $type_id, $dof,
                        ),
                    ),
                )
            }
        }
    };
}

// ── Two-body constraint function impls ─────────────────────────────────────────

use crate::physics::constraints::ball_socket::{BallSocketFunctions, BallSocketPrestepData};
impl_two_body_functions!(BallSocketFunctions, BallSocketPrestepData, Vector3Wide);

use crate::physics::constraints::angular_hinge::{AngularHingeFunctions, AngularHingePrestepData};
use crate::utilities::vector2_wide::Vector2Wide;
impl_two_body_functions!(AngularHingeFunctions, AngularHingePrestepData, Vector2Wide);

use crate::physics::constraints::angular_swivel_hinge::{AngularSwivelHingeFunctions, AngularSwivelHingePrestepData};
impl_two_body_functions!(AngularSwivelHingeFunctions, AngularSwivelHingePrestepData, Vector<f32>);

use crate::physics::constraints::swing_limit::{SwingLimitFunctions, SwingLimitPrestepData};
impl_two_body_functions!(SwingLimitFunctions, SwingLimitPrestepData, Vector<f32>);

use crate::physics::constraints::twist_servo::{TwistServoFunctions, TwistServoPrestepData};
impl_two_body_functions!(TwistServoFunctions, TwistServoPrestepData, Vector<f32>);

use crate::physics::constraints::twist_limit::{TwistLimitFunctions, TwistLimitPrestepData};
impl_two_body_functions!(TwistLimitFunctions, TwistLimitPrestepData, Vector<f32>);

use crate::physics::constraints::twist_motor::{TwistMotorFunctions, TwistMotorPrestepData};
impl_two_body_functions!(TwistMotorFunctions, TwistMotorPrestepData, Vector<f32>);

use crate::physics::constraints::angular_servo::{AngularServoFunctions, AngularServoPrestepData};
impl_two_body_functions!(AngularServoFunctions, AngularServoPrestepData, Vector3Wide);

use crate::physics::constraints::angular_motor::{AngularMotorFunctions, AngularMotorPrestepData};
impl_two_body_functions!(AngularMotorFunctions, AngularMotorPrestepData, Vector3Wide);

use crate::physics::constraints::weld::{WeldFunctions, WeldPrestepData, WeldAccumulatedImpulses};
impl_two_body_functions!(WeldFunctions, WeldPrestepData, WeldAccumulatedImpulses);

use crate::physics::constraints::distance_servo::{DistanceServoFunctions, DistanceServoPrestepData};
impl_two_body_functions!(DistanceServoFunctions, DistanceServoPrestepData, Vector<f32>);

use crate::physics::constraints::distance_limit::{DistanceLimitFunctions, DistanceLimitPrestepData};
impl_two_body_functions!(DistanceLimitFunctions, DistanceLimitPrestepData, Vector<f32>);

use crate::physics::constraints::center_distance_constraint::{CenterDistanceConstraintFunctions, CenterDistancePrestepData};
impl_two_body_functions!(CenterDistanceConstraintFunctions, CenterDistancePrestepData, Vector<f32>);

use crate::physics::constraints::center_distance_limit::{CenterDistanceLimitFunctions, CenterDistanceLimitPrestepData};
impl_two_body_functions!(CenterDistanceLimitFunctions, CenterDistanceLimitPrestepData, Vector<f32>);

use crate::physics::constraints::point_on_line_servo::{PointOnLineServoFunctions, PointOnLineServoPrestepData};
impl_two_body_functions!(PointOnLineServoFunctions, PointOnLineServoPrestepData, Vector2Wide);

use crate::physics::constraints::linear_axis_servo::{LinearAxisServoFunctions, LinearAxisServoPrestepData};
impl_two_body_functions!(LinearAxisServoFunctions, LinearAxisServoPrestepData, Vector<f32>);

use crate::physics::constraints::linear_axis_motor::{LinearAxisMotorFunctions, LinearAxisMotorPrestepData};
impl_two_body_functions!(LinearAxisMotorFunctions, LinearAxisMotorPrestepData, Vector<f32>);

use crate::physics::constraints::linear_axis_limit::{LinearAxisLimitFunctions, LinearAxisLimitPrestepData};
impl_two_body_functions!(LinearAxisLimitFunctions, LinearAxisLimitPrestepData, Vector<f32>);

use crate::physics::constraints::angular_axis_motor::{AngularAxisMotorFunctions, AngularAxisMotorPrestepData};
impl_two_body_functions!(AngularAxisMotorFunctions, AngularAxisMotorPrestepData, Vector<f32>);

use crate::physics::constraints::angular_axis_gear_motor::{AngularAxisGearMotorFunctions, AngularAxisGearMotorPrestepData};
impl_two_body_functions!(AngularAxisGearMotorFunctions, AngularAxisGearMotorPrestepData, Vector<f32>);

use crate::physics::constraints::swivel_hinge::{SwivelHingeFunctions, SwivelHingePrestepData};
use crate::utilities::vector4_wide::Vector4Wide;
impl_two_body_functions!(SwivelHingeFunctions, SwivelHingePrestepData, Vector4Wide);

use crate::physics::constraints::hinge::{HingeFunctions, HingePrestepData, HingeAccumulatedImpulses};
impl_two_body_functions!(HingeFunctions, HingePrestepData, HingeAccumulatedImpulses);

use crate::physics::constraints::ball_socket_servo::{BallSocketServoFunctions, BallSocketServoPrestepData};
impl_two_body_functions!(BallSocketServoFunctions, BallSocketServoPrestepData, Vector3Wide);

use crate::physics::constraints::ball_socket_motor::{BallSocketMotorFunctions, BallSocketMotorPrestepData};
impl_two_body_functions!(BallSocketMotorFunctions, BallSocketMotorPrestepData, Vector3Wide);

// ── One-body constraint function impls ─────────────────────────────────────────

use crate::physics::constraints::one_body_angular_servo::{OneBodyAngularServoFunctions, OneBodyAngularServoPrestepData};
impl_one_body_functions!(OneBodyAngularServoFunctions, OneBodyAngularServoPrestepData, Vector3Wide);

use crate::physics::constraints::one_body_angular_motor::{OneBodyAngularMotorFunctions, OneBodyAngularMotorPrestepData};
impl_one_body_functions!(OneBodyAngularMotorFunctions, OneBodyAngularMotorPrestepData, Vector3Wide);

use crate::physics::constraints::one_body_linear_servo::{OneBodyLinearServoFunctions, OneBodyLinearServoPrestepData};
impl_one_body_functions!(OneBodyLinearServoFunctions, OneBodyLinearServoPrestepData, Vector3Wide);

use crate::physics::constraints::one_body_linear_motor::{OneBodyLinearMotorFunctions, OneBodyLinearMotorPrestepData};
impl_one_body_functions!(OneBodyLinearMotorFunctions, OneBodyLinearMotorPrestepData, Vector3Wide);

// ── Two-body constraint description impls ──────────────────────────────────────

use crate::physics::constraints::ball_socket::BallSocket;
impl_two_body_description!(BallSocket, BallSocketPrestepData, Vector3Wide, BallSocketFunctions, 22, 3, AccessAll, AccessAll);

impl_two_body_description!(
    crate::physics::constraints::angular_hinge::AngularHinge,
    AngularHingePrestepData, Vector2Wide, AngularHingeFunctions,
    23, 2, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::angular_swivel_hinge::AngularSwivelHinge,
    AngularSwivelHingePrestepData, Vector<f32>, AngularSwivelHingeFunctions,
    24, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::swing_limit::SwingLimit,
    SwingLimitPrestepData, Vector<f32>, SwingLimitFunctions,
    25, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::twist_servo::TwistServo,
    TwistServoPrestepData, Vector<f32>, TwistServoFunctions,
    26, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::twist_limit::TwistLimit,
    TwistLimitPrestepData, Vector<f32>, TwistLimitFunctions,
    27, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::twist_motor::TwistMotor,
    TwistMotorPrestepData, Vector<f32>, TwistMotorFunctions,
    28, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::angular_servo::AngularServo,
    AngularServoPrestepData, Vector3Wide, AngularServoFunctions,
    29, 3, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::angular_motor::AngularMotor,
    AngularMotorPrestepData, Vector3Wide, AngularMotorFunctions,
    30, 3, AccessOnlyAngular, AccessOnlyAngularWithoutPose
);

impl_two_body_description!(
    crate::physics::constraints::weld::Weld,
    WeldPrestepData, WeldAccumulatedImpulses, WeldFunctions,
    31, 6, AccessAll, AccessAll
);

// VolumeConstraint (type id 32) is 4-body — skipped; needs FourBodyTypeProcessorImpl.
// AreaConstraint (type id 36) is 3-body — skipped; needs ThreeBodyTypeProcessorImpl.

impl_two_body_description!(
    crate::physics::constraints::distance_servo::DistanceServo,
    DistanceServoPrestepData, Vector<f32>, DistanceServoFunctions,
    33, 1, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::distance_limit::DistanceLimit,
    DistanceLimitPrestepData, Vector<f32>, DistanceLimitFunctions,
    34, 1, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::center_distance_constraint::CenterDistanceConstraint,
    CenterDistancePrestepData, Vector<f32>, CenterDistanceConstraintFunctions,
    35, 1, AccessOnlyLinear, AccessOnlyLinear
);

impl_two_body_description!(
    crate::physics::constraints::point_on_line_servo::PointOnLineServo,
    PointOnLineServoPrestepData, Vector2Wide, PointOnLineServoFunctions,
    37, 2, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::linear_axis_servo::LinearAxisServo,
    LinearAxisServoPrestepData, Vector<f32>, LinearAxisServoFunctions,
    38, 1, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::linear_axis_motor::LinearAxisMotor,
    LinearAxisMotorPrestepData, Vector<f32>, LinearAxisMotorFunctions,
    39, 1, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::linear_axis_limit::LinearAxisLimit,
    LinearAxisLimitPrestepData, Vector<f32>, LinearAxisLimitFunctions,
    40, 1, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::angular_axis_motor::AngularAxisMotor,
    AngularAxisMotorPrestepData, Vector<f32>, AngularAxisMotorFunctions,
    41, 1, AccessOnlyAngular, AccessOnlyAngular
);

impl_two_body_description!(
    crate::physics::constraints::swivel_hinge::SwivelHinge,
    SwivelHingePrestepData, Vector4Wide, SwivelHingeFunctions,
    46, 4, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::hinge::Hinge,
    HingePrestepData, HingeAccumulatedImpulses, HingeFunctions,
    47, 5, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::ball_socket_motor::BallSocketMotor,
    BallSocketMotorPrestepData, Vector3Wide, BallSocketMotorFunctions,
    52, 3, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::ball_socket_servo::BallSocketServo,
    BallSocketServoPrestepData, Vector3Wide, BallSocketServoFunctions,
    53, 3, AccessAll, AccessAll
);

impl_two_body_description!(
    crate::physics::constraints::angular_axis_gear_motor::AngularAxisGearMotor,
    AngularAxisGearMotorPrestepData, Vector<f32>, AngularAxisGearMotorFunctions,
    54, 1, AccessOnlyAngular, AccessOnlyAngularWithoutPose
);

impl_two_body_description!(
    crate::physics::constraints::center_distance_limit::CenterDistanceLimit,
    CenterDistanceLimitPrestepData, Vector<f32>, CenterDistanceLimitFunctions,
    55, 1, AccessOnlyLinear, AccessOnlyLinear
);

// ── One-body constraint description impls ──────────────────────────────────────

impl_one_body_description!(
    crate::physics::constraints::one_body_angular_servo::OneBodyAngularServo,
    OneBodyAngularServoPrestepData, Vector3Wide, OneBodyAngularServoFunctions,
    42, 3, AccessOnlyAngular
);

impl_one_body_description!(
    crate::physics::constraints::one_body_angular_motor::OneBodyAngularMotor,
    OneBodyAngularMotorPrestepData, Vector3Wide, OneBodyAngularMotorFunctions,
    43, 3, AccessOnlyAngular
);

impl_one_body_description!(
    crate::physics::constraints::one_body_linear_servo::OneBodyLinearServo,
    OneBodyLinearServoPrestepData, Vector3Wide, OneBodyLinearServoFunctions,
    44, 3, AccessAll
);

impl_one_body_description!(
    crate::physics::constraints::one_body_linear_motor::OneBodyLinearMotor,
    OneBodyLinearMotorPrestepData, Vector3Wide, OneBodyLinearMotorFunctions,
    45, 3, AccessNoPosition
);

// ── Contact constraints ────────────────────────────────────────────────────────
// Contact constraint registrations (Contact1, Contact2, Contact3, Contact4, etc.) are omitted
// here because the collision detection module is not yet translated. They will be added when
// the contact constraint types are available.
