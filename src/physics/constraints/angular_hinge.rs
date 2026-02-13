// Translated from BepuPhysics/Constraints/AngularHinge.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::helpers::Helpers;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// Angular component of a hinge. Constrains the angular degrees of freedom of two bodies
/// such that they can only rotate relative to each other around the hinge's axis.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AngularHinge {
    /// Hinge axis in the local space of A.
    pub local_hinge_axis_a: Vec3,
    /// Hinge axis in the local space of B.
    pub local_hinge_axis_b: Vec3,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl AngularHinge {
    pub const CONSTRAINT_TYPE_ID: i32 = AngularHingeTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut AngularHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            ConstraintChecker::assert_unit_length_vec3(
                self.local_hinge_axis_a,
                "AngularHinge",
                "local_hinge_axis_a",
            );
            ConstraintChecker::assert_unit_length_vec3(
                self.local_hinge_axis_b,
                "AngularHinge",
                "local_hinge_axis_b",
            );
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "AngularHinge");
        }

        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_hinge_axis_a, &mut target.local_hinge_axis_a);
        Vector3Wide::write_first(self.local_hinge_axis_b, &mut target.local_hinge_axis_b);
        unsafe {
            *GatherScatter::get_first_mut(&mut target.spring_settings.angular_frequency) =
                self.spring_settings.angular_frequency;
            *GatherScatter::get_first_mut(&mut target.spring_settings.twice_damping_ratio) =
                self.spring_settings.twice_damping_ratio;
        }
    }

    pub fn build_description(
        prestep_data: &AngularHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut AngularHinge,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(
            &source.local_hinge_axis_a,
            &mut description.local_hinge_axis_a,
        );
        Vector3Wide::read_first(
            &source.local_hinge_axis_b,
            &mut description.local_hinge_axis_b,
        );
        description.spring_settings.angular_frequency = source.spring_settings.angular_frequency[0];
        description.spring_settings.twice_damping_ratio =
            source.spring_settings.twice_damping_ratio[0];
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct AngularHingePrestepData {
    pub local_hinge_axis_a: Vector3Wide,
    pub local_hinge_axis_b: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
}

pub struct AngularHingeFunctions;

impl AngularHingeFunctions {
    #[inline(always)]
    pub fn get_error_angles(
        hinge_axis_a: &Vector3Wide,
        hinge_axis_b: &Vector3Wide,
        jacobian_a: &Matrix2x3Wide,
        error_angles: &mut Vector2Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;

        let mut hinge_axis_b_dot_x = Vector::<f32>::splat(0.0);
        let mut hinge_axis_b_dot_y = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(hinge_axis_b, &jacobian_a.x, &mut hinge_axis_b_dot_x);
        Vector3Wide::dot(hinge_axis_b, &jacobian_a.y, &mut hinge_axis_b_dot_y);

        let to_remove_x = Vector3Wide::scale(&jacobian_a.x, &hinge_axis_b_dot_x);
        let to_remove_y = Vector3Wide::scale(&jacobian_a.y, &hinge_axis_b_dot_y);
        let mut hinge_axis_b_on_plane_x = Vector3Wide::default();
        let mut hinge_axis_b_on_plane_y = Vector3Wide::default();
        Vector3Wide::subtract(hinge_axis_b, &to_remove_x, &mut hinge_axis_b_on_plane_x);
        Vector3Wide::subtract(hinge_axis_b, &to_remove_y, &mut hinge_axis_b_on_plane_y);

        let mut x_length = Vector::<f32>::splat(0.0);
        let mut y_length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&hinge_axis_b_on_plane_x, &mut x_length);
        Vector3Wide::length_into(&hinge_axis_b_on_plane_y, &mut y_length);

        let scale_x = Vector::<f32>::splat(1.0) / x_length;
        let scale_y = Vector::<f32>::splat(1.0) / y_length;
        hinge_axis_b_on_plane_x = Vector3Wide::scale(&hinge_axis_b_on_plane_x, &scale_x);
        hinge_axis_b_on_plane_y = Vector3Wide::scale(&hinge_axis_b_on_plane_y, &scale_y);

        // If the axis is parallel with the normal of the plane, just arbitrarily pick 0 angle.
        let epsilon = Vector::<f32>::splat(1e-7);
        let use_fallback_x = x_length.simd_lt(epsilon).to_int();
        let use_fallback_y = y_length.simd_lt(epsilon).to_int();
        hinge_axis_b_on_plane_x = Vector3Wide::conditional_select(
            &use_fallback_x,
            hinge_axis_a,
            &hinge_axis_b_on_plane_x,
        );
        hinge_axis_b_on_plane_y = Vector3Wide::conditional_select(
            &use_fallback_y,
            hinge_axis_a,
            &hinge_axis_b_on_plane_y,
        );

        let hbxha = Vector3Wide::dot_val(&hinge_axis_b_on_plane_x, hinge_axis_a);
        let hbyha = Vector3Wide::dot_val(&hinge_axis_b_on_plane_y, hinge_axis_a);
        error_angles.x = math_helper::acos_simd(hbxha);
        error_angles.y = math_helper::acos_simd(hbyha);

        let hbxay = Vector3Wide::dot_val(&hinge_axis_b_on_plane_x, &jacobian_a.y);
        let hbyax = Vector3Wide::dot_val(&hinge_axis_b_on_plane_y, &jacobian_a.x);
        let zero = Vector::<f32>::splat(0.0);
        let hbxay_neg = hbxay.simd_lt(zero);
        let hbyax_neg = hbyax.simd_lt(zero);
        error_angles.x = hbxay_neg.select(error_angles.x, -error_angles.x);
        error_angles.y = hbyax_neg.select(-error_angles.y, error_angles.y);
    }

    #[inline(always)]
    fn apply_impulse(
        impulse_to_velocity_a: &Matrix2x3Wide,
        negated_impulse_to_velocity_b: &Matrix2x3Wide,
        csi: &Vector2Wide,
        angular_velocity_a: &mut Vector3Wide,
        angular_velocity_b: &mut Vector3Wide,
    ) {
        let mut velocity_change_a = Vector3Wide::default();
        Matrix2x3Wide::transform(csi, impulse_to_velocity_a, &mut velocity_change_a);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(angular_velocity_a, &velocity_change_a, &mut tmp);
        *angular_velocity_a = tmp;

        let mut negated_velocity_change_b = Vector3Wide::default();
        Matrix2x3Wide::transform(
            csi,
            negated_impulse_to_velocity_b,
            &mut negated_velocity_change_b,
        );
        Vector3Wide::subtract(angular_velocity_b, &negated_velocity_change_b, &mut tmp);
        *angular_velocity_b = tmp;
    }

    #[inline(always)]
    fn compute_jacobians(
        local_hinge_axis_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        hinge_axis_a: &mut Vector3Wide,
        jacobian_a: &mut Matrix2x3Wide,
    ) {
        // Note that we build the tangents in local space first to avoid inconsistencies.
        let mut local_ax = Vector3Wide::default();
        let mut local_ay = Vector3Wide::default();
        Helpers::build_orthonormal_basis(local_hinge_axis_a, &mut local_ax, &mut local_ay);
        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        Matrix3x3Wide::transform_without_overlap(
            local_hinge_axis_a,
            &orientation_matrix_a,
            hinge_axis_a,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_ax,
            &orientation_matrix_a,
            &mut jacobian_a.x,
        );
        Matrix3x3Wide::transform_without_overlap(
            &local_ay,
            &orientation_matrix_a,
            &mut jacobian_a.y,
        );
    }

    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &AngularHingePrestepData,
        accumulated_impulses: &Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut _hinge_axis_a = Vector3Wide::default();
        let mut jacobian_a = Matrix2x3Wide::default();
        Self::compute_jacobians(
            &prestep.local_hinge_axis_a,
            orientation_a,
            &mut _hinge_axis_a,
            &mut jacobian_a,
        );
        let mut impulse_to_velocity_a = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(
            &jacobian_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut negated_impulse_to_velocity_b = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(
            &jacobian_a,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            accumulated_impulses,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
        );
    }

    pub fn solve(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &AngularHingePrestepData,
        accumulated_impulses: &mut Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut hinge_axis_a = Vector3Wide::default();
        let mut jacobian_a = Matrix2x3Wide::default();
        Self::compute_jacobians(
            &prestep.local_hinge_axis_a,
            orientation_a,
            &mut hinge_axis_a,
            &mut jacobian_a,
        );
        let mut hinge_axis_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(
            &prestep.local_hinge_axis_b,
            orientation_b,
            &mut hinge_axis_b,
        );

        // Note that JA = -JB. Compute J * M^-1 for impulse->velocity transforms.
        let mut impulse_to_velocity_a = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(
            &jacobian_a,
            &inertia_a.inverse_inertia_tensor,
            &mut impulse_to_velocity_a,
        );
        let mut negated_impulse_to_velocity_b = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(
            &jacobian_a,
            &inertia_b.inverse_inertia_tensor,
            &mut negated_impulse_to_velocity_b,
        );
        let mut angular_a = Symmetric2x2Wide::default();
        Symmetric2x2Wide::complete_matrix_sandwich(
            &impulse_to_velocity_a,
            &jacobian_a,
            &mut angular_a,
        );
        let mut angular_b = Symmetric2x2Wide::default();
        Symmetric2x2Wide::complete_matrix_sandwich(
            &negated_impulse_to_velocity_b,
            &jacobian_a,
            &mut angular_b,
        );
        let mut inverse_effective_mass = Symmetric2x2Wide::default();
        Symmetric2x2Wide::add(&angular_a, &angular_b, &mut inverse_effective_mass);
        let mut effective_mass = Symmetric2x2Wide::default();
        Symmetric2x2Wide::invert_without_overlap(&inverse_effective_mass, &mut effective_mass);

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

        // Compute error angles.
        let mut error_angle = Vector2Wide::default();
        Self::get_error_angles(&hinge_axis_a, &hinge_axis_b, &jacobian_a, &mut error_angle);

        // Note the negation: we want to oppose the separation.
        let mut bias_velocity = Vector2Wide::default();
        let neg_pev = -position_error_to_velocity;
        Vector2Wide::scale(&error_angle, &neg_pev, &mut bias_velocity);
        let mut bias_impulse = Vector2Wide::default();
        Symmetric2x2Wide::transform_without_overlap(
            &bias_velocity,
            &effective_mass,
            &mut bias_impulse,
        );

        // JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass
        let mut difference = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.angular, &wsv_b.angular, &mut difference);
        let mut csv = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&difference, &jacobian_a, &mut csv);
        let mut csi = Vector2Wide::default();
        Symmetric2x2Wide::transform_without_overlap(&csv, &effective_mass, &mut csi);
        // Scale by effectiveMassCFMScale
        let mut csi_scaled = Vector2Wide::default();
        Vector2Wide::scale(&csi, &effective_mass_cfm_scale, &mut csi_scaled);
        csi = csi_scaled;
        // csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular)
        let mut softness_contribution = Vector2Wide::default();
        Vector2Wide::scale(
            accumulated_impulses,
            &softness_impulse_scale,
            &mut softness_contribution,
        );
        let mut tmp = Vector2Wide::default();
        Vector2Wide::add(&softness_contribution, &csi, &mut tmp);
        Vector2Wide::subtract(&bias_impulse, &tmp, &mut csi);

        let mut new_accumulated = Vector2Wide::default();
        Vector2Wide::add(accumulated_impulses, &csi, &mut new_accumulated);
        *accumulated_impulses = new_accumulated;

        Self::apply_impulse(
            &impulse_to_velocity_a,
            &negated_impulse_to_velocity_b,
            &csi,
            &mut wsv_a.angular,
            &mut wsv_b.angular,
        );
    }
}

pub struct AngularHingeTypeProcessor;

impl AngularHingeTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 23;
}
