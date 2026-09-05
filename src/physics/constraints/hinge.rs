use crate::out;
use crate::out_unsafe;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::angular_hinge::AngularHingeFunctions;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::physics::helpers::Helpers;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::symmetric5x5_wide::Symmetric5x5Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::mem::MaybeUninit;

pub const BATCH_TYPE_ID: i32 = 47;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Hinge {
    pub local_offset_a: Vec3,
    pub local_hinge_axis_a: Vec3,
    pub local_offset_b: Vec3,
    pub local_hinge_axis_b: Vec3,
    pub spring_settings: SpringSettings,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct HingePrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_hinge_axis_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_hinge_axis_b: Vector3Wide,
    pub spring_settings: SpringSettingsWide,
}

impl Hinge {
    pub fn apply_description(
        &self,
        prestep_data: &mut HingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        Vector3Wide::write_slot(
            self.local_offset_a,
            inner_index,
            &mut prestep_data.local_offset_a,
        );
        Vector3Wide::write_slot(
            self.local_hinge_axis_a,
            inner_index,
            &mut prestep_data.local_hinge_axis_a,
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
        prestep_data: &HingePrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut Hinge,
    ) {
        Vector3Wide::read_slot(
            &prestep_data.local_offset_a,
            inner_index,
            &mut description.local_offset_a,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_hinge_axis_a,
            inner_index,
            &mut description.local_hinge_axis_a,
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

impl HingePrestepData {
    /// Legacy build_description on PrestepData (prefer Hinge::build_description).
    #[inline(always)]
    pub fn build_description_from_prestep(&self, description: &mut Hinge, _bundle_index: usize) {
        Vector3Wide::read_first(&self.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(
            &self.local_hinge_axis_a,
            &mut description.local_hinge_axis_a,
        );
        Vector3Wide::read_first(&self.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(
            &self.local_hinge_axis_b,
            &mut description.local_hinge_axis_b,
        );
        SpringSettingsWide::read_first(&self.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct HingeAccumulatedImpulses {
    pub ball_socket: Vector3Wide,
    pub hinge: Vector2Wide,
}

pub struct HingeFunctions;

impl HingeFunctions {
    #[inline(always)]
    fn apply_impulse(
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        hinge_jacobian: &Matrix2x3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        csi: &HingeAccumulatedImpulses,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
    ) {
        // [ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
        //           [ 0, constraintAxisAX, 0, -constraintAxisAX ]
        //           [ 0, constraintAxisAY, 0, -constraintAxisAY ]
        let linear_change_a = Vector3Wide::scale(&csi.ball_socket, &inertia_a.inverse_mass);
        velocity_a.linear = velocity_a.linear + linear_change_a;

        let ball_socket_angular_impulse_a =
            out_unsafe!(Vector3Wide::cross_without_overlap(offset_a, &csi.ball_socket));
        let hinge_angular_impulse_a = out!(Matrix2x3Wide::transform(&csi.hinge, hinge_jacobian));
        let angular_impulse_a = ball_socket_angular_impulse_a + hinge_angular_impulse_a;
        let angular_change_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_a,
            &inertia_a.inverse_inertia_tensor
        ));
        velocity_a.angular = velocity_a.angular + angular_change_a;

        // Note cross order flip for negation
        let negated_linear_change_b = Vector3Wide::scale(&csi.ball_socket, &inertia_b.inverse_mass);
        velocity_b.linear = out!(Vector3Wide::subtract(
            &velocity_b.linear,
            &negated_linear_change_b
        ));
        let ball_socket_angular_impulse_b =
            out_unsafe!(Vector3Wide::cross_without_overlap(&csi.ball_socket, offset_b));
        let angular_impulse_b = out!(Vector3Wide::subtract(
            &ball_socket_angular_impulse_b,
            &hinge_angular_impulse_a
        ));
        let angular_change_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_b,
            &inertia_b.inverse_inertia_tensor
        ));
        velocity_b.angular = velocity_b.angular + angular_change_b;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &HingePrestepData,
        accumulated_impulses: &HingeAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        let offset_a = out!(Matrix3x3Wide::transform_without_overlap(
            &prestep.local_offset_a,
            &orientation_matrix_a
        ));
        let offset_b = out!(QuaternionWide::transform_without_overlap(
            &prestep.local_offset_b,
            orientation_b
        ));
        let (local_ax, local_ay) = out_unsafe!(
            Helpers::build_orthonormal_basis(&prestep.local_hinge_axis_a),
            2
        );
        let mut hinge_jacobian = MaybeUninit::<Matrix2x3Wide>::uninit();
        unsafe {
            Matrix3x3Wide::transform_without_overlap(
                &local_ax,
                &orientation_matrix_a,
                &mut (*hinge_jacobian.as_mut_ptr()).x,
            );
            Matrix3x3Wide::transform_without_overlap(
                &local_ay,
                &orientation_matrix_a,
                &mut (*hinge_jacobian.as_mut_ptr()).y,
            );
        }
        let hinge_jacobian = unsafe { hinge_jacobian.assume_init() };
        Self::apply_impulse(
            &offset_a,
            &offset_b,
            &hinge_jacobian,
            inertia_a,
            inertia_b,
            accumulated_impulses,
            wsv_a,
            wsv_b,
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
        prestep: &HingePrestepData,
        accumulated_impulses: &mut HingeAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // 5x12 jacobians from BallSocket + AngularHinge:
        // [ I, skew(offsetA),   -I, -skew(offsetB)    ]
        // [ 0, constraintAxisAX, 0, -constraintAxisAX ]
        // [ 0, constraintAxisAY, 0, -constraintAxisAY ]

        let orientation_matrix_a = out!(Matrix3x3Wide::create_from_quaternion(orientation_a));
        let orientation_matrix_b = out!(Matrix3x3Wide::create_from_quaternion(orientation_b));
        let offset_a = out!(Matrix3x3Wide::transform_without_overlap(
            &prestep.local_offset_a,
            &orientation_matrix_a
        ));
        let hinge_axis_a = out!(Matrix3x3Wide::transform_without_overlap(
            &prestep.local_hinge_axis_a,
            &orientation_matrix_a
        ));
        let offset_b = out!(Matrix3x3Wide::transform_without_overlap(
            &prestep.local_offset_b,
            &orientation_matrix_b
        ));
        let hinge_axis_b = out!(Matrix3x3Wide::transform_without_overlap(
            &prestep.local_hinge_axis_b,
            &orientation_matrix_b
        ));
        let (local_ax, local_ay) = out_unsafe!(
            Helpers::build_orthonormal_basis(&prestep.local_hinge_axis_a),
            2
        );
        let mut hinge_jacobian = MaybeUninit::<Matrix2x3Wide>::uninit();
        unsafe {
            Matrix3x3Wide::transform_without_overlap(
                &local_ax,
                &orientation_matrix_a,
                &mut (*hinge_jacobian.as_mut_ptr()).x,
            );
            Matrix3x3Wide::transform_without_overlap(
                &local_ay,
                &orientation_matrix_a,
                &mut (*hinge_jacobian.as_mut_ptr()).y,
            );
        }
        let hinge_jacobian = unsafe { hinge_jacobian.assume_init() };

        // Upper left 3x3 block: ball socket contribution
        let ball_socket_contribution_angular_a =
            out!(Symmetric3x3Wide::skew_sandwich_without_overlap(
                &offset_a,
                &inertia_a.inverse_inertia_tensor
            ));
        let ball_socket_contribution_angular_b =
            out!(Symmetric3x3Wide::skew_sandwich_without_overlap(
                &offset_b,
                &inertia_b.inverse_inertia_tensor
            ));

        // Lower right 2x2 block: angular hinge contribution
        let hinge_inertia_a = out!(Symmetric3x3Wide::multiply_without_overlap_2x3(
            &hinge_jacobian,
            &inertia_a.inverse_inertia_tensor
        ));
        let hinge_inertia_b = out!(Symmetric3x3Wide::multiply_without_overlap_2x3(
            &hinge_jacobian,
            &inertia_b.inverse_inertia_tensor
        ));
        let hinge_contribution_angular_a = out!(Symmetric2x2Wide::complete_matrix_sandwich(
            &hinge_inertia_a,
            &hinge_jacobian
        ));
        let hinge_contribution_angular_b = out!(Symmetric2x2Wide::complete_matrix_sandwich(
            &hinge_inertia_b,
            &hinge_jacobian
        ));

        // Off-diagonal 2x3 block: coupling between ball socket and angular hinge
        // skew(offsetA) * (Ia^-1 * hingeJacobian) + skew(offsetB) * (Ib^-1 * hingeJacobian)
        let off_diagonal_contribution_ax = out_unsafe!(Vector3Wide::cross_without_overlap(
            &hinge_inertia_a.x,
            &offset_a
        ));
        let off_diagonal_contribution_ay = out_unsafe!(Vector3Wide::cross_without_overlap(
            &hinge_inertia_a.y,
            &offset_a
        ));
        let off_diagonal_contribution_bx = out_unsafe!(Vector3Wide::cross_without_overlap(
            &hinge_inertia_b.x,
            &offset_b
        ));
        let off_diagonal_contribution_by = out_unsafe!(Vector3Wide::cross_without_overlap(
            &hinge_inertia_b.y,
            &offset_b
        ));

        // inverseEffectiveMass is fully written (a, b, d) below before it is read by Invert.
        let mut inverse_effective_mass = MaybeUninit::<Symmetric5x5Wide>::uninit();
        unsafe {
            let iem = inverse_effective_mass.as_mut_ptr();
            Symmetric3x3Wide::add(
                &ball_socket_contribution_angular_a,
                &ball_socket_contribution_angular_b,
                &mut (*iem).a,
            );
            let linear_contribution = inertia_a.inverse_mass + inertia_b.inverse_mass;
            (*iem).a.xx += linear_contribution;
            (*iem).a.yy += linear_contribution;
            (*iem).a.zz += linear_contribution;

            Symmetric2x2Wide::add(
                &hinge_contribution_angular_a,
                &hinge_contribution_angular_b,
                &mut (*iem).d,
            );

            Vector3Wide::add(
                &off_diagonal_contribution_ax,
                &off_diagonal_contribution_bx,
                &mut (*iem).b.x,
            );
            Vector3Wide::add(
                &off_diagonal_contribution_ay,
                &off_diagonal_contribution_by,
                &mut (*iem).b.y,
            );
        }
        let inverse_effective_mass = unsafe { inverse_effective_mass.assume_init() };

        // Invert the 5x5 to get effective mass
        let effective_mass = out!(Symmetric5x5Wide::invert_without_overlap(
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

        // Position error: anchorB - offsetA, where anchorB = (positionB - positionA) + offsetB
        let pb_minus_pa = *position_b - *position_a;
        let anchor_b = out!(Vector3Wide::add(&pb_minus_pa, &offset_b));
        let ball_socket_error = out!(Vector3Wide::subtract(&anchor_b, &offset_a));
        let ball_socket_bias_velocity =
            Vector3Wide::scale(&ball_socket_error, &position_error_to_velocity);

        // Angular hinge error
        let error_angles = out!(AngularHingeFunctions::get_error_angles(
            &hinge_axis_a,
            &hinge_axis_b,
            &hinge_jacobian
        ));
        // Negate: we want to oppose the separation
        let hinge_bias_velocity = out!(Vector2Wide::scale(
            &error_angles,
            &(-position_error_to_velocity)
        ));

        // CSV computation
        // J = [ I, skew(offsetA),   -I, -skew(offsetB)    ]
        //     [ 0, constraintAxisAX, 0, -constraintAxisAX ]
        //     [ 0, constraintAxisAY, 0, -constraintAxisAY ]
        let ball_socket_angular_csv_a =
            out_unsafe!(Vector3Wide::cross_without_overlap(&wsv_a.angular, &offset_a));
        let hinge_csv_a = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_a.angular,
            &hinge_jacobian
        ));
        let ball_socket_angular_csv_b =
            out_unsafe!(Vector3Wide::cross_without_overlap(&offset_b, &wsv_b.angular));
        let negated_hinge_csv_b = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_b.angular,
            &hinge_jacobian
        ));

        let ball_socket_angular_csv = ball_socket_angular_csv_a + ball_socket_angular_csv_b;
        let ball_socket_linear_csv = wsv_a.linear - wsv_b.linear;
        let ball_socket_csv = ball_socket_angular_csv + ball_socket_linear_csv;
        let ball_socket_csv_biased = out!(Vector3Wide::subtract(
            &ball_socket_bias_velocity,
            &ball_socket_csv
        ));
        let hinge_csv = hinge_csv_a - negated_hinge_csv_b;
        let hinge_csv_biased = hinge_bias_velocity - hinge_csv;

        let (csi_ball_socket, csi_hinge) = out_unsafe!(
            Symmetric5x5Wide::transform_without_overlap(
                &ball_socket_csv_biased,
                &hinge_csv_biased,
                &effective_mass
            ),
            2
        );
        let mut csi = HingeAccumulatedImpulses {
            ball_socket: csi_ball_socket,
            hinge: csi_hinge,
        };
        csi.ball_socket *= effective_mass_cfm_scale;
        csi.hinge *= effective_mass_cfm_scale;
        let ball_socket_softness =
            Vector3Wide::scale(&accumulated_impulses.ball_socket, &softness_impulse_scale);
        csi.ball_socket = out!(Vector3Wide::subtract(
            &csi.ball_socket,
            &ball_socket_softness
        ));
        let hinge_softness = out!(Vector2Wide::scale(
            &accumulated_impulses.hinge,
            &softness_impulse_scale
        ));
        csi.hinge = out!(Vector2Wide::subtract(&csi.hinge, &hinge_softness));

        accumulated_impulses.ball_socket += csi.ball_socket;
        accumulated_impulses.hinge += csi.hinge;

        Self::apply_impulse(
            &offset_a,
            &offset_b,
            &hinge_jacobian,
            inertia_a,
            inertia_b,
            &csi,
            wsv_a,
            wsv_b,
        );
    }
}
