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
use crate::physics::constraints::three_body_type_processor::{
    IThreeBodyConstraintFunctions, ThreeBodyTypeProcessorImpl,
};
use crate::physics::constraints::four_body_type_processor::{
    IFourBodyConstraintFunctions, FourBodyTypeProcessorImpl,
};
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::TypeProcessor;
use crate::utilities::memory::buffer::Buffer;
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
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
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
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
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

/// Generates an `IThreeBodyConstraintFunctions` impl that delegates to existing inherent methods.
macro_rules! impl_three_body_functions {
    ($funcs:ty, $prestep:ty, $impulse:ty) => {
        impl IThreeBodyConstraintFunctions<$prestep, $impulse> for $funcs {
            #[inline(always)]
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
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
                wsv_c: &mut BodyVelocityWide,
            ) {
                <$funcs>::warm_start(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    position_c, orientation_c, inertia_c,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b, wsv_c,
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
                position_c: &Vector3Wide,
                orientation_c: &QuaternionWide,
                inertia_c: &BodyInertiaWide,
                dt: f32,
                inverse_dt: f32,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
                wsv_c: &mut BodyVelocityWide,
            ) {
                <$funcs>::solve(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    position_c, orientation_c, inertia_c,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b, wsv_c,
                )
            }

            fn requires_incremental_substep_updates() -> bool {
                false
            }

            fn incrementally_update_for_substep(
                _dt: &Vector<f32>,
                _wsv_a: &BodyVelocityWide,
                _wsv_b: &BodyVelocityWide,
                _wsv_c: &BodyVelocityWide,
                _prestep_data: &mut $prestep,
            ) {
            }
        }
    };
}

/// Generates an `IFourBodyConstraintFunctions` impl that delegates to existing inherent methods.
macro_rules! impl_four_body_functions {
    ($funcs:ty, $prestep:ty, $impulse:ty) => {
        impl IFourBodyConstraintFunctions<$prestep, $impulse> for $funcs {
            #[inline(always)]
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
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
                wsv_c: &mut BodyVelocityWide,
                wsv_d: &mut BodyVelocityWide,
            ) {
                <$funcs>::warm_start(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    position_c, orientation_c, inertia_c,
                    position_d, orientation_d, inertia_d,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b, wsv_c, wsv_d,
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
                position_c: &Vector3Wide,
                orientation_c: &QuaternionWide,
                inertia_c: &BodyInertiaWide,
                position_d: &Vector3Wide,
                orientation_d: &QuaternionWide,
                inertia_d: &BodyInertiaWide,
                dt: f32,
                inverse_dt: f32,
                prestep: &mut $prestep,
                accumulated_impulses: &mut $impulse,
                wsv_a: &mut BodyVelocityWide,
                wsv_b: &mut BodyVelocityWide,
                wsv_c: &mut BodyVelocityWide,
                wsv_d: &mut BodyVelocityWide,
            ) {
                <$funcs>::solve(
                    position_a, orientation_a, inertia_a,
                    position_b, orientation_b, inertia_b,
                    position_c, orientation_c, inertia_c,
                    position_d, orientation_d, inertia_d,
                    dt, inverse_dt,
                    prestep, accumulated_impulses,
                    wsv_a, wsv_b, wsv_c, wsv_d,
                )
            }

            fn requires_incremental_substep_updates() -> bool {
                false
            }

            fn incrementally_update_for_substep(
                _dt: &Vector<f32>,
                _wsv_a: &BodyVelocityWide,
                _wsv_b: &BodyVelocityWide,
                _wsv_c: &BodyVelocityWide,
                _wsv_d: &BodyVelocityWide,
                _prestep_data: &mut $prestep,
            ) {
            }
        }
    };
}

/// Generates an `IConstraintDescription` impl for a three-body constraint.
macro_rules! impl_three_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr, $solve_a:ty, $solve_b:ty, $solve_c:ty) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        ThreeBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, $solve_a, $solve_b, $solve_c>::new(
                            $type_id, $dof,
                        ),
                    ),
                )
            }
        }
    };
}

/// Generates an `IConstraintDescription` impl for a four-body constraint.
macro_rules! impl_four_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr, $solve_a:ty, $solve_b:ty, $solve_c:ty, $solve_d:ty) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        FourBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, $solve_a, $solve_b, $solve_c, $solve_d>::new(
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

// ── Three-body constraint function impls ───────────────────────────────────────

use crate::physics::constraints::area_constraint::{AreaConstraintFunctions, AreaConstraintPrestepData};
impl_three_body_functions!(AreaConstraintFunctions, AreaConstraintPrestepData, Vector<f32>);

// ── Four-body constraint function impls ────────────────────────────────────────

use crate::physics::constraints::volume_constraint::{VolumeConstraintFunctions, VolumeConstraintPrestepData};
impl_four_body_functions!(VolumeConstraintFunctions, VolumeConstraintPrestepData, Vector<f32>);

// ── Contact constraint function impls ──────────────────────────────────────────

/// One-body contact constraint functions with incremental substep updates.
macro_rules! impl_contact_one_body_functions {
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
                true
            }

            fn incrementally_update_for_substep(
                dt: &Vector<f32>,
                velocity: &BodyVelocityWide,
                prestep_data: &mut $prestep,
            ) {
                <$funcs>::incrementally_update_for_substep(dt, velocity, prestep_data)
            }
        }
    };
}

/// Two-body contact constraint functions with incremental substep updates.
macro_rules! impl_contact_two_body_functions {
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
                true
            }

            fn incrementally_update_for_substep(
                dt: &Vector<f32>,
                wsv_a: &BodyVelocityWide,
                wsv_b: &BodyVelocityWide,
                prestep_data: &mut $prestep,
            ) {
                <$funcs>::incrementally_update_for_substep(dt, wsv_a, wsv_b, prestep_data)
            }
        }
    };
}

// Convex one-body
impl_contact_one_body_functions!(Contact1OneBodyFunctions, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses);
impl_contact_one_body_functions!(Contact2OneBodyFunctions, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses);
impl_contact_one_body_functions!(Contact3OneBodyFunctions, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses);
impl_contact_one_body_functions!(Contact4OneBodyFunctions, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses);

// Convex two-body
impl_contact_two_body_functions!(Contact1Functions, Contact1PrestepData, Contact1AccumulatedImpulses);
impl_contact_two_body_functions!(Contact2Functions, Contact2PrestepData, Contact2AccumulatedImpulses);
impl_contact_two_body_functions!(Contact3Functions, Contact3PrestepData, Contact3AccumulatedImpulses);
impl_contact_two_body_functions!(Contact4Functions, Contact4PrestepData, Contact4AccumulatedImpulses);

// Nonconvex one-body and two-body: these are generic over their prestep/impulse types, so we
// cannot use the macro directly. Instead we use separate impls for each concrete instantiation.
// The C# code uses a single generic ContactNonconvexOneBodyFunctions/ContactNonconvexTwoBodyFunctions
// struct, but in Rust we need a concrete type per specialization for the type processor.

// For nonconvex contacts, the Functions struct is the same for all contact counts.
// We create thin wrapper types for each instantiation.
pub struct Contact2NonconvexOneBodyFunctionsImpl;
impl Contact2NonconvexOneBodyFunctionsImpl {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;
}

impl IOneBodyConstraintFunctions<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses> for Contact2NonconvexOneBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        prestep: &mut Contact2NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact2NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::warm_start(position_a, orientation_a, inertia_a, prestep, accumulated_impulses, wsv_a);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact2NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact2NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::solve(position_a, orientation_a, inertia_a, dt, inverse_dt, prestep, accumulated_impulses, wsv_a);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, velocity: &BodyVelocityWide, prestep: &mut Contact2NonconvexOneBodyPrestepData) {
        ContactNonconvexOneBodyFunctions::incrementally_update_for_substep(dt, velocity, prestep);
    }
}

pub struct Contact3NonconvexOneBodyFunctionsImpl;
impl IOneBodyConstraintFunctions<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses> for Contact3NonconvexOneBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        prestep: &mut Contact3NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact3NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::warm_start(position_a, orientation_a, inertia_a, prestep, accumulated_impulses, wsv_a);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact3NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact3NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::solve(position_a, orientation_a, inertia_a, dt, inverse_dt, prestep, accumulated_impulses, wsv_a);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, velocity: &BodyVelocityWide, prestep: &mut Contact3NonconvexOneBodyPrestepData) {
        ContactNonconvexOneBodyFunctions::incrementally_update_for_substep(dt, velocity, prestep);
    }
}

pub struct Contact4NonconvexOneBodyFunctionsImpl;
impl IOneBodyConstraintFunctions<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses> for Contact4NonconvexOneBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        prestep: &mut Contact4NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact4NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::warm_start(position_a, orientation_a, inertia_a, prestep, accumulated_impulses, wsv_a);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact4NonconvexOneBodyPrestepData, accumulated_impulses: &mut Contact4NonconvexAccumulatedImpulses, wsv_a: &mut BodyVelocityWide) {
        ContactNonconvexOneBodyFunctions::solve(position_a, orientation_a, inertia_a, dt, inverse_dt, prestep, accumulated_impulses, wsv_a);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, velocity: &BodyVelocityWide, prestep: &mut Contact4NonconvexOneBodyPrestepData) {
        ContactNonconvexOneBodyFunctions::incrementally_update_for_substep(dt, velocity, prestep);
    }
}

pub struct Contact2NonconvexTwoBodyFunctionsImpl;
impl ITwoBodyConstraintFunctions<Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses> for Contact2NonconvexTwoBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        prestep: &mut Contact2NonconvexPrestepData, accumulated_impulses: &mut Contact2NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::warm_start(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact2NonconvexPrestepData, accumulated_impulses: &mut Contact2NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::solve(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, dt, inverse_dt, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, wsv_a: &BodyVelocityWide, wsv_b: &BodyVelocityWide, prestep: &mut Contact2NonconvexPrestepData) {
        ContactNonconvexTwoBodyFunctions::incrementally_update_for_substep(dt, wsv_a, wsv_b, prestep);
    }
}

pub struct Contact3NonconvexTwoBodyFunctionsImpl;
impl ITwoBodyConstraintFunctions<Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses> for Contact3NonconvexTwoBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        prestep: &mut Contact3NonconvexPrestepData, accumulated_impulses: &mut Contact3NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::warm_start(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact3NonconvexPrestepData, accumulated_impulses: &mut Contact3NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::solve(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, dt, inverse_dt, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, wsv_a: &BodyVelocityWide, wsv_b: &BodyVelocityWide, prestep: &mut Contact3NonconvexPrestepData) {
        ContactNonconvexTwoBodyFunctions::incrementally_update_for_substep(dt, wsv_a, wsv_b, prestep);
    }
}

pub struct Contact4NonconvexTwoBodyFunctionsImpl;
impl ITwoBodyConstraintFunctions<Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses> for Contact4NonconvexTwoBodyFunctionsImpl {
    fn warm_start(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        prestep: &mut Contact4NonconvexPrestepData, accumulated_impulses: &mut Contact4NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::warm_start(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn solve(position_a: &Vector3Wide, orientation_a: &QuaternionWide, inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide, orientation_b: &QuaternionWide, inertia_b: &BodyInertiaWide,
        dt: f32, inverse_dt: f32, prestep: &mut Contact4NonconvexPrestepData, accumulated_impulses: &mut Contact4NonconvexAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide, wsv_b: &mut BodyVelocityWide) {
        ContactNonconvexTwoBodyFunctions::solve(position_a, orientation_a, inertia_a, position_b, orientation_b, inertia_b, dt, inverse_dt, prestep, accumulated_impulses, wsv_a, wsv_b);
    }
    fn requires_incremental_substep_updates() -> bool { true }
    fn incrementally_update_for_substep(dt: &Vector<f32>, wsv_a: &BodyVelocityWide, wsv_b: &BodyVelocityWide, prestep: &mut Contact4NonconvexPrestepData) {
        ContactNonconvexTwoBodyFunctions::incrementally_update_for_substep(dt, wsv_a, wsv_b, prestep);
    }
}

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

// VolumeConstraint — 4-body, type id 32
impl_four_body_description!(
    crate::physics::constraints::volume_constraint::VolumeConstraint,
    VolumeConstraintPrestepData, Vector<f32>, VolumeConstraintFunctions,
    32, 1, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear
);

// AreaConstraint — 3-body, type id 36
impl_three_body_description!(
    crate::physics::constraints::area_constraint::AreaConstraint,
    AreaConstraintPrestepData, Vector<f32>, AreaConstraintFunctions,
    36, 1, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear
);

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
// Contact constraint description registrations.

// Macro for contact descriptions that have their own apply_description method on a
// prestep-specific method, not the generic IConstraintDescription. The IConstraintDescription
// apply_description delegates to the description struct's own method.
macro_rules! impl_contact_one_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        OneBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, AccessAll>::new(
                            $type_id, $dof,
                        ),
                    ),
                )
            }
        }
    };
}

macro_rules! impl_contact_two_body_description {
    ($desc:ty, $prestep:ty, $impulse:ty, $funcs:ty, $type_id:expr, $dof:expr) => {
        impl IConstraintDescription for $desc {
            fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32) {
                debug_assert!(batch.type_id == $type_id, "Type batch mismatch");
                let prestep = unsafe {
                    &mut *(batch.prestep_data.as_mut_ptr() as *mut $prestep).add(bundle_index as usize)
                };
                self.apply_description(prestep, bundle_index as usize, inner_index as usize);
            }

            fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self {
                let prestep = unsafe {
                    &*(batch.prestep_data.as_ptr() as *const $prestep).add(bundle_index as usize)
                };
                let mut desc = unsafe { std::mem::zeroed::<Self>() };
                Self::build_description(prestep, bundle_index as usize, inner_index as usize, &mut desc);
                desc
            }

            fn constraint_type_id() -> i32 {
                $type_id
            }

            fn create_type_processor() -> TypeProcessor {
                TypeProcessor::new(
                    $type_id,
                    Box::new(
                        TwoBodyTypeProcessorImpl::<$prestep, $impulse, $funcs, AccessAll, AccessAll>::new(
                            $type_id, $dof,
                        ),
                    ),
                )
            }
        }
    };
}

// Convex one-body contacts (Contact1OneBody..Contact4OneBody)
use crate::physics::constraints::contact::contact_convex_descriptions::*;
use crate::physics::constraints::contact::contact_convex_types::*;
use crate::physics::constraints::contact::contact_nonconvex_common::ContactNonconvexOneBodyFunctions;
use crate::physics::constraints::contact::contact_nonconvex_common::ContactNonconvexTwoBodyFunctions;
use crate::physics::constraints::contact::contact_nonconvex_descriptions::*;
use crate::physics::constraints::contact::contact_nonconvex_types::*;

impl_contact_one_body_description!(
    Contact1OneBody, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses,
    Contact1OneBodyFunctions, 0, 4
);
impl_contact_one_body_description!(
    Contact2OneBody, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses,
    Contact2OneBodyFunctions, 1, 5
);
impl_contact_one_body_description!(
    Contact3OneBody, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses,
    Contact3OneBodyFunctions, 2, 6
);
impl_contact_one_body_description!(
    Contact4OneBody, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses,
    Contact4OneBodyFunctions, 3, 7
);

// Convex two-body contacts (Contact1TwoBody..Contact4TwoBody)
impl_contact_two_body_description!(
    Contact1TwoBody, Contact1PrestepData, Contact1AccumulatedImpulses,
    Contact1Functions, 4, 4
);
impl_contact_two_body_description!(
    Contact2TwoBody, Contact2PrestepData, Contact2AccumulatedImpulses,
    Contact2Functions, 5, 5
);
impl_contact_two_body_description!(
    Contact3TwoBody, Contact3PrestepData, Contact3AccumulatedImpulses,
    Contact3Functions, 6, 6
);
impl_contact_two_body_description!(
    Contact4TwoBody, Contact4PrestepData, Contact4AccumulatedImpulses,
    Contact4Functions, 7, 7
);

// Nonconvex one-body contacts

impl_contact_one_body_description!(
    Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData,
    Contact2NonconvexAccumulatedImpulses, Contact2NonconvexOneBodyFunctionsImpl, 8, 6
);
impl_contact_one_body_description!(
    Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData,
    Contact3NonconvexAccumulatedImpulses, Contact3NonconvexOneBodyFunctionsImpl, 9, 9
);
impl_contact_one_body_description!(
    Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData,
    Contact4NonconvexAccumulatedImpulses, Contact4NonconvexOneBodyFunctionsImpl, 10, 12
);

// Nonconvex two-body contacts
impl_contact_two_body_description!(
    Contact2Nonconvex, Contact2NonconvexPrestepData,
    Contact2NonconvexAccumulatedImpulses, Contact2NonconvexTwoBodyFunctionsImpl, 15, 6
);
impl_contact_two_body_description!(
    Contact3Nonconvex, Contact3NonconvexPrestepData,
    Contact3NonconvexAccumulatedImpulses, Contact3NonconvexTwoBodyFunctionsImpl, 16, 9
);
impl_contact_two_body_description!(
    Contact4Nonconvex, Contact4NonconvexPrestepData,
    Contact4NonconvexAccumulatedImpulses, Contact4NonconvexTwoBodyFunctionsImpl, 17, 12
);
