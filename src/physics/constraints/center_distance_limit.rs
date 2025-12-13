use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::constraints::inequality_helpers::InequalityHelpers;
use crate::physics::constraints::center_distance_constraint::CenterDistanceConstraintFunctions;
use crate::utilities::vector::Vector;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

/// Constrains the center of two bodies to be separated by a distance within a range.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CenterDistanceLimit {
    /// Minimum distance between the body centers.
    pub minimum_distance: f32,
    /// Maximum distance between the body centers.
    pub maximum_distance: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl CenterDistanceLimit {
    #[inline(always)]
    pub fn new(minimum_distance: f32, maximum_distance: f32, spring_settings: SpringSettings) -> Self {
        Self {
            minimum_distance,
            maximum_distance,
            spring_settings,
        }
    }

    pub fn apply_description(
        &self,
        prestep_data: &mut CenterDistanceLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(self.minimum_distance >= 0.0, "CenterDistanceLimit.MinimumDistance must be nonnegative.");
            debug_assert!(self.maximum_distance >= 0.0, "CenterDistanceLimit.MaximumDistance must be nonnegative.");
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "CenterDistanceLimit");
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        unsafe {
            *GatherScatter::get_first_mut(&mut target.minimum_distance) = self.minimum_distance;
            *GatherScatter::get_first_mut(&mut target.maximum_distance) = self.maximum_distance;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &CenterDistanceLimitPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        description.minimum_distance = unsafe { *GatherScatter::get_first(&source.minimum_distance) };
        description.maximum_distance = unsafe { *GatherScatter::get_first(&source.maximum_distance) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CenterDistanceLimitPrestepData {
    pub minimum_distance: Vector<f32>,
    pub maximum_distance: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct CenterDistanceLimitFunctions;

impl CenterDistanceLimitFunctions {
    #[inline(always)]
    fn compute_jacobian(
        minimum_distance: &Vector<f32>,
        maximum_distance: &Vector<f32>,
        position_a: &Vector3Wide,
        position_b: &Vector3Wide,
        jacobian_a: &mut Vector3Wide,
        distance: &mut Vector<f32>,
        use_minimum: &mut Vector<i32>,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        *distance = ab.length();
        let inverse_distance = math_helper::fast_reciprocal(*distance);
        let use_fallback = distance.simd_lt(Vector::<f32>::splat(1e-5));
        *jacobian_a = Vector3Wide::scale(&ab, &inverse_distance);
        jacobian_a.x = use_fallback.select(Vector::<f32>::splat(1.0), jacobian_a.x);
        jacobian_a.y = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.y);
        jacobian_a.z = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.z);

        // If the current distance is closer to the minimum, calibrate for the minimum. Otherwise, calibrate for the maximum.
        *use_minimum = (*distance - *minimum_distance).abs().simd_lt((*distance - *maximum_distance).abs()).to_int();
        let mut negated_jacobian = Vector3Wide::default();
        Vector3Wide::negate(jacobian_a, &mut negated_jacobian);
        *jacobian_a = Vector3Wide::conditional_select(use_minimum, &negated_jacobian, jacobian_a);
    }

    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        _orientation_a: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        _orientation_b: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &CenterDistanceLimitPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut jacobian_a = Vector3Wide::default();
        let mut distance = Vector::<f32>::splat(0.0);
        let mut use_minimum = Vector::<i32>::splat(0);
        Self::compute_jacobian(
            &prestep.minimum_distance,
            &prestep.maximum_distance,
            position_a,
            position_b,
            &mut jacobian_a,
            &mut distance,
            &mut use_minimum,
        );
        CenterDistanceConstraintFunctions::apply_impulse(
            &jacobian_a,
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            accumulated_impulses,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        position_a: &Vector3Wide,
        _orientation_a: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        _orientation_b: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &CenterDistanceLimitPrestepData,
        accumulated_impulse: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut jacobian_a = Vector3Wide::default();
        let mut distance = Vector::<f32>::splat(0.0);
        let mut use_minimum = Vector::<i32>::splat(0);
        Self::compute_jacobian(
            &prestep.minimum_distance,
            &prestep.maximum_distance,
            position_a,
            position_b,
            &mut jacobian_a,
            &mut distance,
            &mut use_minimum,
        );

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            &prestep.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        // Jacobian is just the unit length direction, so the effective mass is simple:
        let effective_mass = effective_mass_cfm_scale / (inertia_a.inverse_mass + inertia_b.inverse_mass);

        let error = use_minimum.simd_lt(Vector::<i32>::splat(0)).select(
            prestep.minimum_distance - distance,
            distance - prestep.maximum_distance,
        );
        let mut bias_velocity = Vector::<f32>::splat(0.0);
        InequalityHelpers::compute_bias_velocity(error, &position_error_to_velocity, inverse_dt, &mut bias_velocity);
        let csv = Vector3Wide::dot_val(&wsv_a.linear, &jacobian_a) - Vector3Wide::dot_val(&wsv_b.linear, &jacobian_a);
        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csv
        let mut csi = -*accumulated_impulse * softness_impulse_scale - effective_mass * (csv - bias_velocity);
        InequalityHelpers::clamp_positive(accumulated_impulse, &mut csi);

        CenterDistanceConstraintFunctions::apply_impulse(
            &jacobian_a,
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            &csi,
            wsv_a,
            wsv_b,
        );
    }

    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = false;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        _dt: &Vector<f32>,
        _wsv_a: &BodyVelocityWide,
        _wsv_b: &BodyVelocityWide,
        _prestep_data: &mut CenterDistanceLimitPrestepData,
    ) {
    }
}

/// Handles the solve iterations of a bunch of center distance limit constraints.
pub struct CenterDistanceLimitTypeProcessor;

impl CenterDistanceLimitTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 55;
}
