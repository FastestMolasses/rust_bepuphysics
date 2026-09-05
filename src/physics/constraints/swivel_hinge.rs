// Translated from BepuPhysics/Constraints/SwivelHinge.cs

use glam::Vec3;

use crate::out;
use crate::out_unsafe;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
#[cfg(debug_assertions)]
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
use std::mem::MaybeUninit;

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

        Vector3Wide::write_slot(
            self.local_offset_a,
            inner_index,
            &mut prestep_data.local_offset_a,
        );
        Vector3Wide::write_slot(
            self.local_swivel_axis_a,
            inner_index,
            &mut prestep_data.local_swivel_axis_a,
        );
        Vector3Wide::write_slot(
            self.local_offset_b,
            inner_index,
            &mut prestep_data.local_offset_b,
        );
        Vector3Wide::write_slot(
            self.local_hinge_axis_b,
            inner_index,
            &mut prestep_data.local_hinge_axis_b,
        );
        unsafe {
            *GatherScatter::get_mut(
                &mut prestep_data.spring_settings.angular_frequency,
                inner_index,
            ) = self.spring_settings.angular_frequency;
            *GatherScatter::get_mut(
                &mut prestep_data.spring_settings.twice_damping_ratio,
                inner_index,
            ) = self.spring_settings.twice_damping_ratio;
        }
    }

    pub fn build_description(
        prestep_data: &SwivelHingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut SwivelHinge,
    ) {
        Vector3Wide::read_slot(
            &prestep_data.local_offset_a,
            inner_index,
            &mut description.local_offset_a,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_swivel_axis_a,
            inner_index,
            &mut description.local_swivel_axis_a,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_offset_b,
            inner_index,
            &mut description.local_offset_b,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_hinge_axis_b,
            inner_index,
            &mut description.local_hinge_axis_b,
        );
        unsafe {
            description.spring_settings.angular_frequency =
                *GatherScatter::get(&prestep_data.spring_settings.angular_frequency, inner_index);
            description.spring_settings.twice_damping_ratio = *GatherScatter::get(
                &prestep_data.spring_settings.twice_damping_ratio,
                inner_index,
            );
        }
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
        velocity_a.linear = out!(Vector3Wide::add(&velocity_a.linear, &linear_change_a));

        // Compute angular impulse for A: cross(offsetA, ballSocketCSI) + swivelHingeJacobian * csi.W
        let ball_socket_angular_impulse_a =
            out_unsafe!(Vector3Wide::cross_without_overlap(offset_a, ball_socket_csi));
        let swivel_hinge_angular_impulse_a = *swivel_hinge_jacobian * csi.w;
        let angular_impulse_a = out!(Vector3Wide::add(
            &ball_socket_angular_impulse_a,
            &swivel_hinge_angular_impulse_a
        ));
        let angular_change_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_a,
            &inertia_a.inverse_inertia_tensor
        ));
        velocity_a.angular = out!(Vector3Wide::add(&velocity_a.angular, &angular_change_a));

        // Note cross order flip for negation for B's linear.
        let negated_linear_change_b = Vector3Wide::scale(ball_socket_csi, &inertia_b.inverse_mass);
        velocity_b.linear = out!(Vector3Wide::subtract(
            &velocity_b.linear,
            &negated_linear_change_b
        ));

        // Angular for B: cross(ballSocketCSI, offsetB) - swivelHingeAngularImpulseA
        let ball_socket_angular_impulse_b =
            out_unsafe!(Vector3Wide::cross_without_overlap(ball_socket_csi, offset_b));
        let angular_impulse_b = out!(Vector3Wide::subtract(
            &ball_socket_angular_impulse_b,
            &swivel_hinge_angular_impulse_a
        ));
        let angular_change_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_b,
            &inertia_b.inverse_inertia_tensor
        ));
        velocity_b.angular = out!(Vector3Wide::add(&velocity_b.angular, &angular_change_b));
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

        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        let orientation_matrix_b = out!(Matrix3x3Wide::create_from_quaternion(orientation_b));
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
        let use_fallback = length_squared.simd_lt(Vector::<f32>::splat(1e-3)).to_simd();
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
        let mut _swivel_axis = MaybeUninit::<Vector3Wide>::uninit();
        let mut _hinge_axis = MaybeUninit::<Vector3Wide>::uninit();
        let mut offset_a = MaybeUninit::<Vector3Wide>::uninit();
        let mut offset_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut swivel_hinge_jacobian = MaybeUninit::<Vector3Wide>::uninit();
        Self::compute_jacobian(
            &prestep.local_offset_a,
            &prestep.local_swivel_axis_a,
            &prestep.local_offset_b,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            unsafe { &mut *_swivel_axis.as_mut_ptr() },
            unsafe { &mut *_hinge_axis.as_mut_ptr() },
            unsafe { &mut *offset_a.as_mut_ptr() },
            unsafe { &mut *offset_b.as_mut_ptr() },
            unsafe { &mut *swivel_hinge_jacobian.as_mut_ptr() },
        );
        let offset_a = unsafe { offset_a.assume_init() };
        let offset_b = unsafe { offset_b.assume_init() };
        let swivel_hinge_jacobian = unsafe { swivel_hinge_jacobian.assume_init() };
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
        let mut swivel_axis = MaybeUninit::<Vector3Wide>::uninit();
        let mut hinge_axis = MaybeUninit::<Vector3Wide>::uninit();
        let mut offset_a = MaybeUninit::<Vector3Wide>::uninit();
        let mut offset_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut swivel_hinge_jacobian = MaybeUninit::<Vector3Wide>::uninit();
        Self::compute_jacobian(
            &prestep.local_offset_a,
            &prestep.local_swivel_axis_a,
            &prestep.local_offset_b,
            &prestep.local_hinge_axis_b,
            orientation_a,
            orientation_b,
            unsafe { &mut *swivel_axis.as_mut_ptr() },
            unsafe { &mut *hinge_axis.as_mut_ptr() },
            unsafe { &mut *offset_a.as_mut_ptr() },
            unsafe { &mut *offset_b.as_mut_ptr() },
            unsafe { &mut *swivel_hinge_jacobian.as_mut_ptr() },
        );
        let swivel_axis = unsafe { swivel_axis.assume_init() };
        let hinge_axis = unsafe { hinge_axis.assume_init() };
        let offset_a = unsafe { offset_a.assume_init() };
        let offset_b = unsafe { offset_b.assume_init() };
        let swivel_hinge_jacobian = unsafe { swivel_hinge_jacobian.assume_init() };

        // Upper left 3x3 block: ball socket contribution.
        let ball_socket_contribution_angular_a = out!(Symmetric3x3Wide::skew_sandwich_without_overlap(
            &offset_a,
            &inertia_a.inverse_inertia_tensor
        ));
        let ball_socket_contribution_angular_b = out!(Symmetric3x3Wide::skew_sandwich_without_overlap(
            &offset_b,
            &inertia_b.inverse_inertia_tensor
        ));

        // Lower right 1x1 block: AngularSwivelHinge.
        let swivel_hinge_inertia_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &swivel_hinge_jacobian,
            &inertia_a.inverse_inertia_tensor
        ));
        let swivel_hinge_inertia_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &swivel_hinge_jacobian,
            &inertia_b.inverse_inertia_tensor
        ));
        let swivel_hinge_contribution_angular_a =
            Vector3Wide::dot_val(&swivel_hinge_inertia_a, &swivel_hinge_jacobian);
        let swivel_hinge_contribution_angular_b =
            Vector3Wide::dot_val(&swivel_hinge_inertia_b, &swivel_hinge_jacobian);

        // Off-diagonal: (Ia^-1 * swivelHingeJ) x offsetA + (Ib^-1 * swivelHingeJ) x offsetB
        let off_diagonal_contribution_a = out_unsafe!(Vector3Wide::cross_without_overlap(
            &swivel_hinge_inertia_a,
            &offset_a
        ));
        let off_diagonal_contribution_b = out_unsafe!(Vector3Wide::cross_without_overlap(
            &swivel_hinge_inertia_b,
            &offset_b
        ));

        // inverseEffectiveMass is fully written (upper-left 3x3, ww, upper-right 3x1) below before it is read by Invert.
        let mut inverse_effective_mass = MaybeUninit::<Symmetric4x4Wide>::uninit();
        unsafe {
            let iem = inverse_effective_mass.as_mut_ptr();
            let upper_left = Symmetric4x4Wide::get_upper_left_3x3_block_mut(&mut *iem);
            Symmetric3x3Wide::add(
                &ball_socket_contribution_angular_a,
                &ball_socket_contribution_angular_b,
                upper_left,
            );
            let linear_contribution = inertia_a.inverse_mass + inertia_b.inverse_mass;
            upper_left.xx += linear_contribution;
            upper_left.yy += linear_contribution;
            upper_left.zz += linear_contribution;

            (*iem).ww = swivel_hinge_contribution_angular_a + swivel_hinge_contribution_angular_b;

            let upper_right = Symmetric4x4Wide::get_upper_right_3x1_block_mut(&mut *iem);
            Vector3Wide::add(
                &off_diagonal_contribution_a,
                &off_diagonal_contribution_b,
                upper_right,
            );
        }
        let inverse_effective_mass = unsafe { inverse_effective_mass.assume_init() };

        let effective_mass = out!(Symmetric4x4Wide::invert_without_overlap(
            &inverse_effective_mass
        ));
        let mut position_error_to_velocity = MaybeUninit::<Vector<f32>>::uninit();
        let mut effective_mass_cfm_scale = MaybeUninit::<Vector<f32>>::uninit();
        let mut softness_impulse_scale = MaybeUninit::<Vector<f32>>::uninit();
        SpringSettingsWide::compute_springiness(
            &prestep.spring_settings,
            dt,
            unsafe { &mut *position_error_to_velocity.as_mut_ptr() },
            unsafe { &mut *effective_mass_cfm_scale.as_mut_ptr() },
            unsafe { &mut *softness_impulse_scale.as_mut_ptr() },
        );
        let position_error_to_velocity = unsafe { position_error_to_velocity.assume_init() };
        let effective_mass_cfm_scale = unsafe { effective_mass_cfm_scale.assume_init() };
        let softness_impulse_scale = unsafe { softness_impulse_scale.assume_init() };

        // Compute position error and bias velocities.
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let anchor_b = out!(Vector3Wide::add(&ab, &offset_b));
        let ball_socket_error = out!(Vector3Wide::subtract(&anchor_b, &offset_a));

        let swivel_hinge_error = out!(Vector3Wide::dot(&hinge_axis, &swivel_axis));
        // Note the negation: we want to oppose the separation.
        let bias_velocity = Vector4Wide {
            x: ball_socket_error.x * position_error_to_velocity,
            y: ball_socket_error.y * position_error_to_velocity,
            z: ball_socket_error.z * position_error_to_velocity,
            w: position_error_to_velocity * -swivel_hinge_error,
        };

        // CSV computation
        let ball_socket_angular_csv_a =
            out_unsafe!(Vector3Wide::cross_without_overlap(&wsv_a.angular, &offset_a));
        let swivel_hinge_csv_a = Vector3Wide::dot_val(&swivel_hinge_jacobian, &wsv_a.angular);
        let ball_socket_angular_csv_b =
            out_unsafe!(Vector3Wide::cross_without_overlap(&offset_b, &wsv_b.angular));
        let negated_swivel_hinge_csv_b =
            Vector3Wide::dot_val(&swivel_hinge_jacobian, &wsv_b.angular);

        let ball_socket_angular_csv = out!(Vector3Wide::add(
            &ball_socket_angular_csv_a,
            &ball_socket_angular_csv_b
        ));
        let ball_socket_linear_csv = out!(Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear));

        let csv = Vector4Wide {
            x: ball_socket_angular_csv.x + ball_socket_linear_csv.x,
            y: ball_socket_angular_csv.y + ball_socket_linear_csv.y,
            z: ball_socket_angular_csv.z + ball_socket_linear_csv.z,
            w: swivel_hinge_csv_a - negated_swivel_hinge_csv_b,
        };

        let bias_minus_csv = out!(Vector4Wide::subtract_to(&bias_velocity, &csv));

        let mut csi = out!(Symmetric4x4Wide::transform_without_overlap(
            &bias_minus_csv,
            &effective_mass
        ));
        let csi_scaled = out!(Vector4Wide::scale(&csi, effective_mass_cfm_scale));
        let softness_contribution = out!(Vector4Wide::scale(
            accumulated_impulses,
            softness_impulse_scale
        ));
        csi = out!(Vector4Wide::subtract_to(&csi_scaled, &softness_contribution));

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
