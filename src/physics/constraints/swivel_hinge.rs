// Translated from BepuPhysics/Constraints/SwivelHinge.cs

use glam::Vec3;

use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::constraint_checker::ConstraintChecker;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::symmetric4x4_wide::Symmetric4x4Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector4_wide::Vector4Wide;

/// Constrains two bodies with a swivel hinge that allows rotation around two axes,
/// like a laptop monitor hinge that allows flipping the screen.
/// Equivalent to a BallSocket constraint and an AngularSwivelHinge constraint solved together.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SwivelHinge {
    /// Local offset from the center of body A to its attachment point.
    pub local_offset_a: Vec3,
    /// Swivel axis in the local space of body A.
    pub local_swivel_axis_a: Vec3,
    /// Local offset from the center of body B to its attachment point.
    pub local_offset_b: Vec3,
    /// Hinge axis in the local space of body B.
    pub local_hinge_axis_b: Vec3,
    /// Spring frequency and damping parameters.
    pub spring_settings: SpringSettings,
}

impl SwivelHinge {
    pub const CONSTRAINT_TYPE_ID: i32 = SwivelHingeTypeProcessor::BATCH_TYPE_ID;

    pub fn apply_description(
        &self,
        prestep_data: &mut SwivelHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            ConstraintChecker::assert_unit_length_vec3(
                self.local_swivel_axis_a,
                "SwivelHinge",
                "local_swivel_axis_a",
            );
            ConstraintChecker::assert_unit_length_vec3(
                self.local_hinge_axis_b,
                "SwivelHinge",
                "local_hinge_axis_b",
            );
            ConstraintChecker::assert_valid_spring_settings(&self.spring_settings, "SwivelHinge");
        }

        let target = unsafe { GatherScatter::get_offset_instance_mut(prestep_data, inner_index) };
        Vector3Wide::write_first(self.local_offset_a, &mut target.local_offset_a);
        Vector3Wide::write_first(self.local_swivel_axis_a, &mut target.local_swivel_axis_a);
        Vector3Wide::write_first(self.local_offset_b, &mut target.local_offset_b);
        Vector3Wide::write_first(self.local_hinge_axis_b, &mut target.local_hinge_axis_b);
        SpringSettingsWide::write_first(&self.spring_settings, &mut target.spring_settings);
    }

    pub fn build_description(
        prestep_data: &SwivelHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut SwivelHinge,
    ) {
        let source = unsafe { GatherScatter::get_offset_instance(prestep_data, inner_index) };
        Vector3Wide::read_first(&source.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(
            &source.local_swivel_axis_a,
            &mut description.local_swivel_axis_a,
        );
        Vector3Wide::read_first(&source.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(
            &source.local_hinge_axis_b,
            &mut description.local_hinge_axis_b,
        );
        SpringSettingsWide::read_first(&source.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SwivelHingePrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_swivel_axis_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_hinge_axis_b: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
}

pub struct SwivelHingeFunctions;

impl SwivelHingeFunctions {
    #[inline(always)]
    fn apply_impulse(
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        swivel_hinge_jacobian: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        csi: &mut Vector4Wide,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
    ) {
        // Reinterpret the first 3 components of csi as a Vector3Wide (ball socket impulse).
        let ball_socket_csi = unsafe { &*(core::ptr::from_ref(&csi.x) as *const Vector3Wide) };

        // Apply linear impulse to A.
        let linear_change_a = Vector3Wide::scale(ball_socket_csi, &inertia_a.inverse_mass);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::add(&velocity_a.linear, &linear_change_a, &mut tmp);
        velocity_a.linear = tmp;

        // Compute angular impulse for A: cross(offsetA, ballSocketCSI) + swivelHingeJacobian * csi.W
        let mut ball_socket_angular_impulse_a = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                offset_a,
                ball_socket_csi,
                &mut ball_socket_angular_impulse_a,
            );
        }
        let swivel_hinge_angular_impulse_a = *swivel_hinge_jacobian * csi.w;
        let mut angular_impulse_a = Vector3Wide::default();
        Vector3Wide::add(
            &ball_socket_angular_impulse_a,
            &swivel_hinge_angular_impulse_a,
            &mut angular_impulse_a,
        );
        let mut angular_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_a,
            &inertia_a.inverse_inertia_tensor,
            &mut angular_change_a,
        );
        Vector3Wide::add(&velocity_a.angular, &angular_change_a, &mut tmp);
        velocity_a.angular = tmp;

        // Note cross order flip for negation for B's linear.
        let negated_linear_change_b = Vector3Wide::scale(ball_socket_csi, &inertia_b.inverse_mass);
        Vector3Wide::subtract(&velocity_b.linear, &negated_linear_change_b, &mut tmp);
        velocity_b.linear = tmp;

        // Angular for B: cross(ballSocketCSI, offsetB) - swivelHingeAngularImpulseA
        let mut ball_socket_angular_impulse_b = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                ball_socket_csi,
                offset_b,
                &mut ball_socket_angular_impulse_b,
            );
        }
        let mut angular_impulse_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &ball_socket_angular_impulse_b,
            &swivel_hinge_angular_impulse_a,
            &mut angular_impulse_b,
        );
        let mut angular_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_b,
            &inertia_b.inverse_inertia_tensor,
            &mut angular_change_b,
        );
        Vector3Wide::add(&velocity_b.angular, &angular_change_b, &mut tmp);
        velocity_b.angular = tmp;
    }

    #[inline(always)]
    fn compute_jacobian(
        local_offset_a: &Vector3Wide,
        local_swivel_axis_a: &Vector3Wide,
        local_offset_b: &Vector3Wide,
        local_hinge_axis_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        swivel_axis: &mut Vector3Wide,
        hinge_axis: &mut Vector3Wide,
        offset_a: &mut Vector3Wide,
        offset_b: &mut Vector3Wide,
        swivel_hinge_jacobian: &mut Vector3Wide,
    ) {
        use std::simd::cmp::SimdPartialOrd;

        let mut orientation_matrix_a = Matrix3x3Wide::default();
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        Matrix3x3Wide::transform_without_overlap(local_offset_a, &orientation_matrix_a, offset_a);
        Matrix3x3Wide::transform_without_overlap(
            local_swivel_axis_a,
            &orientation_matrix_a,
            swivel_axis,
        );
        Matrix3x3Wide::transform_without_overlap(local_offset_b, &orientation_matrix_b, offset_b);
        Matrix3x3Wide::transform_without_overlap(
            local_hinge_axis_b,
            &orientation_matrix_b,
            hinge_axis,
        );
        unsafe {
            Vector3Wide::cross_without_overlap(swivel_axis, hinge_axis, swivel_hinge_jacobian);
        }
        // If the axes are aligned, it'll be zero length and the effective mass can get NaNsploded.
        let length_squared = swivel_hinge_jacobian.length_squared();
        let use_fallback = length_squared.simd_lt(Vector::<f32>::splat(1e-3)).to_int();
        // This causes a discontinuity, but a discontinuity is better than a NaNsplode.
        *swivel_hinge_jacobian =
            Vector3Wide::conditional_select(&use_fallback, hinge_axis, swivel_hinge_jacobian);
    }

    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &SwivelHingePrestepData,
        accumulated_impulses: &mut Vector4Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut _swivel_axis = Vector3Wide::default();
        let mut _hinge_axis = Vector3Wide::default();
        let mut offset_a = Vector3Wide::default();
        let mut offset_b = Vector3Wide::default();
        let mut swivel_hinge_jacobian = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.local_offset_a,
            &prestep.local_swivel_axis_a,
            &prestep.local_offset_b,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            &mut _swivel_axis,
            &mut _hinge_axis,
            &mut offset_a,
            &mut offset_b,
            &mut swivel_hinge_jacobian,
        );
        Self::apply_impulse(
            &offset_a,
            &offset_b,
            &swivel_hinge_jacobian,
            inertia_a,
            inertia_b,
            accumulated_impulses,
            wsv_a,
            wsv_b,
        );
    }

    pub fn solve(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &SwivelHingePrestepData,
        accumulated_impulses: &mut Vector4Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut swivel_axis = Vector3Wide::default();
        let mut hinge_axis = Vector3Wide::default();
        let mut offset_a = Vector3Wide::default();
        let mut offset_b = Vector3Wide::default();
        let mut swivel_hinge_jacobian = Vector3Wide::default();
        Self::compute_jacobian(
            &prestep.local_offset_a,
            &prestep.local_swivel_axis_a,
            &prestep.local_offset_b,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            &mut swivel_axis,
            &mut hinge_axis,
            &mut offset_a,
            &mut offset_b,
            &mut swivel_hinge_jacobian,
        );

        // Upper left 3x3 block: ball socket contribution.
        let mut ball_socket_contribution_angular_a = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(
            &offset_a,
            &inertia_a.inverse_inertia_tensor,
            &mut ball_socket_contribution_angular_a,
        );
        let mut ball_socket_contribution_angular_b = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(
            &offset_b,
            &inertia_b.inverse_inertia_tensor,
            &mut ball_socket_contribution_angular_b,
        );
        let mut inverse_effective_mass = Symmetric4x4Wide::default();
        {
            let upper_left =
                Symmetric4x4Wide::get_upper_left_3x3_block_mut(&mut inverse_effective_mass);
            Symmetric3x3Wide::add(
                &ball_socket_contribution_angular_a,
                &ball_socket_contribution_angular_b,
                upper_left,
            );
            let linear_contribution = inertia_a.inverse_mass + inertia_b.inverse_mass;
            upper_left.xx += linear_contribution;
            upper_left.yy += linear_contribution;
            upper_left.zz += linear_contribution;
        }

        // Lower right 1x1 block: AngularSwivelHinge.
        let mut swivel_hinge_inertia_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &swivel_hinge_jacobian,
            &inertia_a.inverse_inertia_tensor,
            &mut swivel_hinge_inertia_a,
        );
        let mut swivel_hinge_inertia_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(
            &swivel_hinge_jacobian,
            &inertia_b.inverse_inertia_tensor,
            &mut swivel_hinge_inertia_b,
        );
        let swivel_hinge_contribution_angular_a =
            Vector3Wide::dot_val(&swivel_hinge_inertia_a, &swivel_hinge_jacobian);
        let swivel_hinge_contribution_angular_b =
            Vector3Wide::dot_val(&swivel_hinge_inertia_b, &swivel_hinge_jacobian);
        inverse_effective_mass.ww =
            swivel_hinge_contribution_angular_a + swivel_hinge_contribution_angular_b;

        // Off-diagonal: (Ia^-1 * swivelHingeJ) x offsetA + (Ib^-1 * swivelHingeJ) x offsetB
        {
            let mut off_diagonal_contribution_a = Vector3Wide::default();
            unsafe {
                Vector3Wide::cross_without_overlap(
                    &swivel_hinge_inertia_a,
                    &offset_a,
                    &mut off_diagonal_contribution_a,
                );
            }
            let mut off_diagonal_contribution_b = Vector3Wide::default();
            unsafe {
                Vector3Wide::cross_without_overlap(
                    &swivel_hinge_inertia_b,
                    &offset_b,
                    &mut off_diagonal_contribution_b,
                );
            }
            let upper_right =
                Symmetric4x4Wide::get_upper_right_3x1_block_mut(&mut inverse_effective_mass);
            Vector3Wide::add(
                &off_diagonal_contribution_a,
                &off_diagonal_contribution_b,
                upper_right,
            );
        }

        let mut effective_mass = Symmetric4x4Wide::default();
        Symmetric4x4Wide::invert_without_overlap(&inverse_effective_mass, &mut effective_mass);
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

        // Compute position error and bias velocities.
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab);
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(&ab, &offset_b, &mut anchor_b);
        let mut ball_socket_error = Vector3Wide::default();
        Vector3Wide::subtract(&anchor_b, &offset_a, &mut ball_socket_error);

        let mut bias_velocity = Vector4Wide {
            x: ball_socket_error.x * position_error_to_velocity,
            y: ball_socket_error.y * position_error_to_velocity,
            z: ball_socket_error.z * position_error_to_velocity,
            w: Vector::<f32>::splat(0.0),
        };
        let mut swivel_hinge_error = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&hinge_axis, &swivel_axis, &mut swivel_hinge_error);
        // Note the negation: we want to oppose the separation.
        bias_velocity.w = position_error_to_velocity * -swivel_hinge_error;

        // CSV computation
        let mut ball_socket_angular_csv_a = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                &wsv_a.angular,
                &offset_a,
                &mut ball_socket_angular_csv_a,
            );
        }
        let swivel_hinge_csv_a = Vector3Wide::dot_val(&swivel_hinge_jacobian, &wsv_a.angular);
        let mut ball_socket_angular_csv_b = Vector3Wide::default();
        unsafe {
            Vector3Wide::cross_without_overlap(
                &offset_b,
                &wsv_b.angular,
                &mut ball_socket_angular_csv_b,
            );
        }
        let negated_swivel_hinge_csv_b =
            Vector3Wide::dot_val(&swivel_hinge_jacobian, &wsv_b.angular);

        let mut ball_socket_angular_csv = Vector3Wide::default();
        Vector3Wide::add(
            &ball_socket_angular_csv_a,
            &ball_socket_angular_csv_b,
            &mut ball_socket_angular_csv,
        );
        let mut ball_socket_linear_csv = Vector3Wide::default();
        Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear, &mut ball_socket_linear_csv);

        let csv = Vector4Wide {
            x: ball_socket_angular_csv.x + ball_socket_linear_csv.x,
            y: ball_socket_angular_csv.y + ball_socket_linear_csv.y,
            z: ball_socket_angular_csv.z + ball_socket_linear_csv.z,
            w: swivel_hinge_csv_a - negated_swivel_hinge_csv_b,
        };

        let mut bias_minus_csv = Vector4Wide::default();
        Vector4Wide::subtract_to(&bias_velocity, &csv, &mut bias_minus_csv);

        let mut csi = Vector4Wide::default();
        Symmetric4x4Wide::transform_without_overlap(&bias_minus_csv, &effective_mass, &mut csi);
        let mut csi_scaled = Vector4Wide::default();
        Vector4Wide::scale(&csi, effective_mass_cfm_scale, &mut csi_scaled);
        let mut softness_contribution = Vector4Wide::default();
        Vector4Wide::scale(
            accumulated_impulses,
            softness_impulse_scale,
            &mut softness_contribution,
        );
        Vector4Wide::subtract_to(&csi_scaled, &softness_contribution, &mut csi);

        // accumulatedImpulses += csi
        let new_accumulated = *accumulated_impulses + csi;
        *accumulated_impulses = new_accumulated;

        Self::apply_impulse(
            &offset_a,
            &offset_b,
            &swivel_hinge_jacobian,
            inertia_a,
            inertia_b,
            &mut csi,
            wsv_a,
            wsv_b,
        );
    }
}

pub struct SwivelHingeTypeProcessor;

impl SwivelHingeTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 46;
}
