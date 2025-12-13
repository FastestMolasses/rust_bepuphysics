// Translated from BepuPhysics/Constraints/BallSocketMotor.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::ball_socket_shared::BallSocketShared;
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Controls the relative linear velocity from the center of body A to an attachment point on body B.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BallSocketMotor {
    /// Offset from body B to its attachment in B's local space.
    pub local_offset_b: Vec3,
    /// Target relative linear velocity between A and B, stored in A's local space.
    /// Target world space linear velocity of B is LinearVelocityA + TargetVelocityLocalA * OrientationA.
    pub target_velocity_local_a: Vec3,
    /// Motor control parameters.
    pub settings: MotorSettings,
}

impl BallSocketMotor {
    pub const CONSTRAINT_TYPE_ID: i32 = BallSocketMotorTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut BallSocketMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "BallSocketMotor");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        Vector3Wide::write_first(self.target_velocity_local_a, &mut target.target_velocity_local_a);
        MotorSettingsWide::write_first(&self.settings, &mut target.settings);
    }

    pub fn build_description(
        prestep_data: &BallSocketMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut BallSocketMotor,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&source.target_velocity_local_a, &mut description.target_velocity_local_a);
        MotorSettingsWide::read_first(&source.settings, &mut description.settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct BallSocketMotorPrestepData {
    pub local_offset_b: Vector3Wide,
    pub target_velocity_local_a: Vector3Wide,
    pub settings: MotorSettingsWide,
}

pub struct BallSocketMotorFunctions;

impl BallSocketMotorFunctions {
    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &BallSocketMotorPrestepData,
        accumulated_impulses: &Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut target_offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.local_offset_b,
            orientation_b,
            &mut target_offset_b,
        );
        // offsetA = (positionB - positionA) + targetOffsetB
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut offset_a = Vector3Wide::default();
        Vector3Wide::add(&ab, &target_offset_b, &mut offset_a);
        BallSocketShared::apply_impulse(
            wsv_a,
            wsv_b,
            &offset_a,
            &target_offset_b,
            inertia_a,
            inertia_b,
            accumulated_impulses,
        );
    }

    #[inline(always)]
    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &BallSocketMotorPrestepData,
        accumulated_impulses: &mut Vector3Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut target_offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.local_offset_b,
            orientation_b,
            &mut target_offset_b,
        );
        // offsetA = (positionB - positionA) + targetOffsetB
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut offset_a = Vector3Wide::default();
        Vector3Wide::add(&ab, &target_offset_b, &mut offset_a);

        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        let mut maximum_impulse = Vector::<f32>::splat(0.0);
        MotorSettingsWide::compute_softness(
            &prestep.settings,
            dt,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
            &mut maximum_impulse,
        );
        let mut effective_mass = crate::utilities::symmetric3x3_wide::Symmetric3x3Wide::default();
        BallSocketShared::compute_effective_mass(
            inertia_a,
            inertia_b,
            &offset_a,
            &target_offset_b,
            &effective_mass_cfm_scale,
            &mut effective_mass,
        );

        let bias_velocity = QuaternionWide::transform(&prestep.target_velocity_local_a, orientation_a);
        let mut negated_bias_velocity = Vector3Wide::default();
        Vector3Wide::negate(&bias_velocity, &mut negated_bias_velocity);

        BallSocketShared::solve_with_max_impulse(
            wsv_a,
            wsv_b,
            &offset_a,
            &target_offset_b,
            &negated_bias_velocity,
            &effective_mass,
            &softness_impulse_scale,
            &maximum_impulse,
            accumulated_impulses,
            inertia_a,
            inertia_b,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut BallSocketMotorPrestepData,
    ) {
    }
}

/// Handles the solve iterations of a bunch of ball socket motor constraints.
pub struct BallSocketMotorTypeProcessor;

impl BallSocketMotorTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 52;
}
