use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::cmp::SimdPartialOrd;

/// Constrains the center of two bodies to be separated by a goal distance.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CenterDistanceConstraint {
    /// Target distance between the body centers.
    pub target_distance: f32,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl CenterDistanceConstraint {
    #[inline(always)]
    pub fn new(target_distance: f32, spring_settings: SpringSettings) -> Self {
        Self {
            target_distance,
            spring_settings,
        }
    }

    pub fn apply_description(
        &self,
        prestep_data: &mut CenterDistancePrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            debug_assert!(
                self.target_distance >= 0.0,
                "CenterDistanceConstraint.TargetDistance must be nonnegative."
            );
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_spring_settings(
                &self.spring_settings,
                "CenterDistanceConstraint",
            );
        }
        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        unsafe {
            *GatherScatter::get_first_mut(&mut target.target_distance) = self.target_distance;
        }
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &CenterDistancePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Self,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        description.target_distance = unsafe { *GatherScatter::get_first(&source.target_distance) };
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct CenterDistancePrestepData {
    pub target_distance: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

pub struct CenterDistanceConstraintFunctions;

impl CenterDistanceConstraintFunctions {
    #[inline(always)]
    pub fn apply_impulse(
        jacobian_a: &Vector3Wide,
        inverse_mass_a: &Vector<f32>,
        inverse_mass_b: &Vector<f32>,
        impulse: &Vector<f32>,
        a: &mut BodyVelocityWide,
        b: &mut BodyVelocityWide,
    ) {
        let mut change_a = Vector3Wide::default();
        Vector3Wide::scale_to(jacobian_a, &(*impulse * *inverse_mass_a), &mut change_a);
        let mut negated_change_b = Vector3Wide::default();
        Vector3Wide::scale_to(
            jacobian_a,
            &(*impulse * *inverse_mass_b),
            &mut negated_change_b,
        );
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&a.linear, &change_a, &mut tmp);
        a.linear = tmp;
        Vector3Wide::subtract(&b.linear, &negated_change_b, &mut tmp);
        b.linear = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        _orientation_a: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        _orientation_b: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_b: &BodyInertiaWide,
        _prestep: &CenterDistancePrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let length_squared = ab.length_squared();
        let inverse_distance = math_helper::fast_reciprocal_square_root(length_squared);
        let use_fallback = length_squared.simd_lt(Vector::<f32>::splat(1e-10));
        let mut jacobian_a = Vector3Wide::scale(&ab, &inverse_distance);
        jacobian_a.x = use_fallback.select(Vector::<f32>::splat(1.0), jacobian_a.x);
        jacobian_a.y = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.y);
        jacobian_a.z = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.z);

        Self::apply_impulse(
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
        _inverse_dt: f32,
        prestep: &CenterDistancePrestepData,
        accumulated_impulse: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // Note that we need the actual length for error calculation.
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let distance = ab.length();
        let inverse_distance = math_helper::fast_reciprocal(distance);
        let use_fallback = distance.simd_lt(Vector::<f32>::splat(1e-5));
        let mut jacobian_a = Vector3Wide::scale(&ab, &inverse_distance);
        jacobian_a.x = use_fallback.select(Vector::<f32>::splat(1.0), jacobian_a.x);
        jacobian_a.y = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.y);
        jacobian_a.z = use_fallback.select(Vector::<f32>::splat(0.0), jacobian_a.z);

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
        let effective_mass =
            effective_mass_cfm_scale / (inertia_a.inverse_mass + inertia_b.inverse_mass);

        // Compute the position error and bias velocities.
        let bias_velocity = (distance - prestep.target_distance) * position_error_to_velocity;

        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csv
        let mut linear_csv_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_a.linear, &jacobian_a, &mut linear_csv_a);
        let mut negated_csv_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&wsv_b.linear, &jacobian_a, &mut negated_csv_b);
        let csi = (bias_velocity - (linear_csv_a - negated_csv_b)) * effective_mass
            - *accumulated_impulse * softness_impulse_scale;
        *accumulated_impulse = *accumulated_impulse + csi;
        Self::apply_impulse(
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
        _prestep_data: &mut CenterDistancePrestepData,
    ) {
    }
}

/// Handles the solve iterations of a bunch of center distance constraints.
pub struct CenterDistanceTypeProcessor;

impl CenterDistanceTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 35;
}
