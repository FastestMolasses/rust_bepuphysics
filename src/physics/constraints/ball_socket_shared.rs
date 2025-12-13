// Translated from BepuPhysics/Constraints/BallSocketShared.cs

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Provides shared functionality for constraints with jacobians similar to the BallSocket.
pub struct BallSocketShared;

impl BallSocketShared {
    #[inline(always)]
    pub fn compute_effective_mass(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        effective_mass_cfm_scale: &Vector<f32>,
        effective_mass: &mut Symmetric3x3Wide,
    ) {
        // Jacobians:
        // LinearA: I, AngularA: skewSymmetric(offsetA), LinearB: -I, AngularB: skewSymmetric(-offsetB)
        let mut inverse_effective_mass = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(
            offset_a,
            &inertia_a.inverse_inertia_tensor,
            &mut inverse_effective_mass,
        );
        // Sign of offsetB doesn't matter due to the sandwich.
        let mut angular_b_contribution = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(
            offset_b,
            &inertia_b.inverse_inertia_tensor,
            &mut angular_b_contribution,
        );
        let mut added = Symmetric3x3Wide::default();
        Symmetric3x3Wide::add(
            &inverse_effective_mass,
            &angular_b_contribution,
            &mut added,
        );
        inverse_effective_mass = added;

        // Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
        let linear_contribution = inertia_a.inverse_mass + inertia_b.inverse_mass;
        inverse_effective_mass.xx += linear_contribution;
        inverse_effective_mass.yy += linear_contribution;
        inverse_effective_mass.zz += linear_contribution;
        Symmetric3x3Wide::invert(&inverse_effective_mass, effective_mass);
        let scaled = {
            let mut tmp = Symmetric3x3Wide::default();
            Symmetric3x3Wide::scale(effective_mass, effective_mass_cfm_scale, &mut tmp);
            tmp
        };
        *effective_mass = scaled;
    }

    #[inline(always)]
    pub fn apply_impulse(
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        constraint_space_impulse: &Vector3Wide,
    ) {
        let mut wsi = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(offset_a, constraint_space_impulse, &mut wsi);
        }
        let mut change = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&wsi, &inertia_a.inverse_inertia_tensor, &mut change);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&velocity_a.angular, &change, &mut tmp);
        velocity_a.angular = tmp;

        Vector3Wide::scale_to(constraint_space_impulse, &inertia_a.inverse_mass, &mut change);
        Vector3Wide::add(&velocity_a.linear, &change, &mut tmp);
        velocity_a.linear = tmp;

        unsafe {
            // Note flip-negation: cross(impulse, offsetB) instead of cross(offsetB, impulse)
            Vector3Wide::cross_without_overlap(constraint_space_impulse, offset_b, &mut wsi);
        }
        Symmetric3x3Wide::transform_without_overlap(&wsi, &inertia_b.inverse_inertia_tensor, &mut change);
        Vector3Wide::add(&velocity_b.angular, &change, &mut tmp);
        velocity_b.angular = tmp;

        Vector3Wide::scale_to(constraint_space_impulse, &inertia_b.inverse_mass, &mut change);
        // Note subtraction; the jacobian is -I.
        Vector3Wide::subtract(&velocity_b.linear, &change, &mut tmp);
        velocity_b.linear = tmp;
    }

    #[inline(always)]
    pub fn compute_corrective_impulse(
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        bias_velocity: &Vector3Wide,
        effective_mass: &Symmetric3x3Wide,
        softness_impulse_scale: &Vector<f32>,
        accumulated_impulse: &Vector3Wide,
        corrective_impulse: &mut Vector3Wide,
    ) {
        // csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale
        //     - (csiaLinear + csiaAngular + csibLinear + csibAngular);
        // Note subtraction; jLinearB = -I.
        let mut csv = Vector3Wide::default();
        Vector3Wide::subtract(&velocity_a.linear, &velocity_b.linear, &mut csv);
        let mut angular_csv = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(&velocity_a.angular, offset_a, &mut angular_csv);
        }
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&csv, &angular_csv, &mut tmp);
        csv = tmp;
        // Note reversed cross order; matches the jacobian -CrossMatrix(offsetB).
        unsafe {
            Vector3Wide::cross_without_overlap(offset_b, &velocity_b.angular, &mut angular_csv);
        }
        Vector3Wide::add(&csv, &angular_csv, &mut tmp);
        csv = tmp;
        Vector3Wide::subtract(bias_velocity, &csv, &mut tmp);
        csv = tmp;

        Symmetric3x3Wide::transform_without_overlap(&csv, effective_mass, corrective_impulse);
        let softness = Vector3Wide::scale(accumulated_impulse, softness_impulse_scale);
        let mut corrective_tmp = Vector3Wide::default();
        Vector3Wide::subtract(corrective_impulse, &softness, &mut corrective_tmp);
        *corrective_impulse = corrective_tmp;
    }

    /// Solves a ball-socket constraint without maximum impulse clamping.
    #[inline(always)]
    pub fn solve(
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        bias_velocity: &Vector3Wide,
        effective_mass: &Symmetric3x3Wide,
        softness_impulse_scale: &Vector<f32>,
        accumulated_impulse: &mut Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
    ) {
        let mut corrective_impulse = Vector3Wide::default();
        Self::compute_corrective_impulse(
            velocity_a,
            velocity_b,
            offset_a,
            offset_b,
            bias_velocity,
            effective_mass,
            softness_impulse_scale,
            accumulated_impulse,
            &mut corrective_impulse,
        );
        // No maximum impulse limit, so no clamping is required.
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(accumulated_impulse, &corrective_impulse, &mut tmp);
        *accumulated_impulse = tmp;

        Self::apply_impulse(
            velocity_a,
            velocity_b,
            offset_a,
            offset_b,
            inertia_a,
            inertia_b,
            &corrective_impulse,
        );
    }

    /// Solves a ball-socket constraint with maximum impulse clamping.
    #[inline(always)]
    pub fn solve_with_max_impulse(
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        bias_velocity: &Vector3Wide,
        effective_mass: &Symmetric3x3Wide,
        softness_impulse_scale: &Vector<f32>,
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
    ) {
        let mut corrective_impulse = Vector3Wide::default();
        Self::compute_corrective_impulse(
            velocity_a,
            velocity_b,
            offset_a,
            offset_b,
            bias_velocity,
            effective_mass,
            softness_impulse_scale,
            accumulated_impulse,
            &mut corrective_impulse,
        );
        // This function DOES have a maximum impulse limit.
        ServoSettingsWide::clamp_impulse_3d(
            maximum_impulse,
            accumulated_impulse,
            &mut corrective_impulse,
        );

        Self::apply_impulse(
            velocity_a,
            velocity_b,
            offset_a,
            offset_b,
            inertia_a,
            inertia_b,
            &corrective_impulse,
        );
    }
}
