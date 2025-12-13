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
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;
use glam::Vec3;

pub const BATCH_TYPE_ID: i32 = 47;

#[repr(C)]
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

impl HingePrestepData {
    #[inline(always)]
    pub fn build_description(&self, description: &mut Hinge, _bundle_index: usize) {
        Vector3Wide::read_first(&self.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&self.local_hinge_axis_a, &mut description.local_hinge_axis_a);
        Vector3Wide::read_first(&self.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&self.local_hinge_axis_b, &mut description.local_hinge_axis_b);
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

        let mut ball_socket_angular_impulse_a = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(offset_a, &csi.ball_socket, &mut ball_socket_angular_impulse_a); }
        let mut hinge_angular_impulse_a = Vector3Wide::default();
        Matrix2x3Wide::transform(&csi.hinge, hinge_jacobian, &mut hinge_angular_impulse_a);
        let angular_impulse_a = ball_socket_angular_impulse_a + hinge_angular_impulse_a;
        let mut angular_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_a, &inertia_a.inverse_inertia_tensor, &mut angular_change_a);
        velocity_a.angular = velocity_a.angular + angular_change_a;

        // Note cross order flip for negation
        let negated_linear_change_b = Vector3Wide::scale(&csi.ball_socket, &inertia_b.inverse_mass);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&velocity_b.linear, &negated_linear_change_b, &mut tmp);
        velocity_b.linear = tmp;
        let mut ball_socket_angular_impulse_b = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&csi.ball_socket, offset_b, &mut ball_socket_angular_impulse_b); }
        let mut angular_impulse_b = Vector3Wide::default();
        Vector3Wide::subtract(&ball_socket_angular_impulse_b, &hinge_angular_impulse_a, &mut angular_impulse_b);
        let mut angular_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_b, &inertia_b.inverse_inertia_tensor, &mut angular_change_b);
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
        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        let mut offset_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&prestep.local_offset_a, &orientation_matrix_a, &mut offset_a);
        let mut offset_b = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset_b, orientation_b, &mut offset_b);
        let mut local_ax = Vector3Wide::default();
        let mut local_ay = Vector3Wide::default();
        Helpers::build_orthonormal_basis(&prestep.local_hinge_axis_a, &mut local_ax, &mut local_ay);
        let mut hinge_jacobian = Matrix2x3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&local_ax, &orientation_matrix_a, &mut hinge_jacobian.x);
        Matrix3x3Wide::transform_without_overlap(&local_ay, &orientation_matrix_a, &mut hinge_jacobian.y);
        Self::apply_impulse(&offset_a, &offset_b, &hinge_jacobian, inertia_a, inertia_b, accumulated_impulses, wsv_a, wsv_b);
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

        let mut orientation_matrix_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut orientation_matrix_a);
        let mut orientation_matrix_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut orientation_matrix_b);
        let mut offset_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&prestep.local_offset_a, &orientation_matrix_a, &mut offset_a);
        let mut hinge_axis_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&prestep.local_hinge_axis_a, &orientation_matrix_a, &mut hinge_axis_a);
        let mut offset_b = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&prestep.local_offset_b, &orientation_matrix_b, &mut offset_b);
        let mut hinge_axis_b = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&prestep.local_hinge_axis_b, &orientation_matrix_b, &mut hinge_axis_b);
        let mut local_ax = Vector3Wide::default();
        let mut local_ay = Vector3Wide::default();
        Helpers::build_orthonormal_basis(&prestep.local_hinge_axis_a, &mut local_ax, &mut local_ay);
        let mut hinge_jacobian = Matrix2x3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&local_ax, &orientation_matrix_a, &mut hinge_jacobian.x);
        Matrix3x3Wide::transform_without_overlap(&local_ay, &orientation_matrix_a, &mut hinge_jacobian.y);

        // Upper left 3x3 block: ball socket contribution
        let mut ball_socket_contribution_angular_a = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(&offset_a, &inertia_a.inverse_inertia_tensor, &mut ball_socket_contribution_angular_a);
        let mut ball_socket_contribution_angular_b = Symmetric3x3Wide::default();
        Symmetric3x3Wide::skew_sandwich_without_overlap(&offset_b, &inertia_b.inverse_inertia_tensor, &mut ball_socket_contribution_angular_b);
        let mut inverse_effective_mass = Symmetric5x5Wide {
            a: Symmetric3x3Wide::default(),
            b: Matrix2x3Wide::default(),
            d: Symmetric2x2Wide::default(),
        };
        Symmetric3x3Wide::add(&ball_socket_contribution_angular_a, &ball_socket_contribution_angular_b, &mut inverse_effective_mass.a);
        let linear_contribution = inertia_a.inverse_mass + inertia_b.inverse_mass;
        inverse_effective_mass.a.xx += linear_contribution;
        inverse_effective_mass.a.yy += linear_contribution;
        inverse_effective_mass.a.zz += linear_contribution;

        // Lower right 2x2 block: angular hinge contribution
        let mut hinge_inertia_a = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(&hinge_jacobian, &inertia_a.inverse_inertia_tensor, &mut hinge_inertia_a);
        let mut hinge_inertia_b = Matrix2x3Wide::default();
        Symmetric3x3Wide::multiply_without_overlap_2x3(&hinge_jacobian, &inertia_b.inverse_inertia_tensor, &mut hinge_inertia_b);
        let mut hinge_contribution_angular_a = Symmetric2x2Wide::default();
        Symmetric2x2Wide::complete_matrix_sandwich(&hinge_inertia_a, &hinge_jacobian, &mut hinge_contribution_angular_a);
        let mut hinge_contribution_angular_b = Symmetric2x2Wide::default();
        Symmetric2x2Wide::complete_matrix_sandwich(&hinge_inertia_b, &hinge_jacobian, &mut hinge_contribution_angular_b);
        Symmetric2x2Wide::add(&hinge_contribution_angular_a, &hinge_contribution_angular_b, &mut inverse_effective_mass.d);

        // Off-diagonal 2x3 block: coupling between ball socket and angular hinge
        // skew(offsetA) * (Ia^-1 * hingeJacobian) + skew(offsetB) * (Ib^-1 * hingeJacobian)
        let mut off_diagonal_contribution_ax = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&hinge_inertia_a.x, &offset_a, &mut off_diagonal_contribution_ax); }
        let mut off_diagonal_contribution_ay = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&hinge_inertia_a.y, &offset_a, &mut off_diagonal_contribution_ay); }
        let mut off_diagonal_contribution_bx = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&hinge_inertia_b.x, &offset_b, &mut off_diagonal_contribution_bx); }
        let mut off_diagonal_contribution_by = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&hinge_inertia_b.y, &offset_b, &mut off_diagonal_contribution_by); }
        Vector3Wide::add(&off_diagonal_contribution_ax, &off_diagonal_contribution_bx, &mut inverse_effective_mass.b.x);
        Vector3Wide::add(&off_diagonal_contribution_ay, &off_diagonal_contribution_by, &mut inverse_effective_mass.b.y);

        // Invert the 5x5 to get effective mass
        let mut effective_mass = Symmetric5x5Wide {
            a: Symmetric3x3Wide::default(),
            b: Matrix2x3Wide::default(),
            d: Symmetric2x2Wide::default(),
        };
        Symmetric5x5Wide::invert_without_overlap(&inverse_effective_mass, &mut effective_mass);
        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            &prestep.spring_settings, dt,
            &mut position_error_to_velocity, &mut effective_mass_cfm_scale, &mut softness_impulse_scale,
        );

        // Position error: anchorB - offsetA, where anchorB = (positionB - positionA) + offsetB
        let mut ball_socket_error = Vector3Wide::default();
        let pb_minus_pa = *position_b - *position_a;
        let mut anchor_b = Vector3Wide::default();
        Vector3Wide::add(&pb_minus_pa, &offset_b, &mut anchor_b);
        Vector3Wide::subtract(&anchor_b, &offset_a, &mut ball_socket_error);
        let ball_socket_bias_velocity = Vector3Wide::scale(&ball_socket_error, &position_error_to_velocity);

        // Angular hinge error
        let mut error_angles = Vector2Wide::default();
        AngularHingeFunctions::get_error_angles(&hinge_axis_a, &hinge_axis_b, &hinge_jacobian, &mut error_angles);
        // Negate: we want to oppose the separation
        let mut hinge_bias_velocity = Vector2Wide::default();
        Vector2Wide::scale(&error_angles, &(-position_error_to_velocity), &mut hinge_bias_velocity);

        // CSV computation
        // J = [ I, skew(offsetA),   -I, -skew(offsetB)    ]
        //     [ 0, constraintAxisAX, 0, -constraintAxisAX ]
        //     [ 0, constraintAxisAY, 0, -constraintAxisAY ]
        let mut ball_socket_angular_csv_a = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&wsv_a.angular, &offset_a, &mut ball_socket_angular_csv_a); }
        let mut hinge_csv_a = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_a.angular, &hinge_jacobian, &mut hinge_csv_a);
        let mut ball_socket_angular_csv_b = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(&offset_b, &wsv_b.angular, &mut ball_socket_angular_csv_b); }
        let mut negated_hinge_csv_b = Vector2Wide::default();
        Matrix2x3Wide::transform_by_transpose_without_overlap(&wsv_b.angular, &hinge_jacobian, &mut negated_hinge_csv_b);

        let ball_socket_angular_csv = ball_socket_angular_csv_a + ball_socket_angular_csv_b;
        let ball_socket_linear_csv = wsv_a.linear - wsv_b.linear;
        let ball_socket_csv = ball_socket_angular_csv + ball_socket_linear_csv;
        let mut ball_socket_csv_biased = Vector3Wide::default();
        Vector3Wide::subtract(&ball_socket_bias_velocity, &ball_socket_csv, &mut ball_socket_csv_biased);
        let hinge_csv = hinge_csv_a - negated_hinge_csv_b;
        let hinge_csv_biased = hinge_bias_velocity - hinge_csv;

        let mut csi = HingeAccumulatedImpulses {
            ball_socket: Vector3Wide::default(),
            hinge: Vector2Wide::default(),
        };
        Symmetric5x5Wide::transform_without_overlap(
            &ball_socket_csv_biased, &hinge_csv_biased,
            &effective_mass,
            &mut csi.ball_socket, &mut csi.hinge,
        );
        csi.ball_socket *= effective_mass_cfm_scale;
        csi.hinge *= effective_mass_cfm_scale;
        let ball_socket_softness = Vector3Wide::scale(&accumulated_impulses.ball_socket, &softness_impulse_scale);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&csi.ball_socket, &ball_socket_softness, &mut tmp);
        csi.ball_socket = tmp;
        let mut hinge_softness = Vector2Wide::default();
        Vector2Wide::scale(&accumulated_impulses.hinge, &softness_impulse_scale, &mut hinge_softness);
        let mut csi_hinge_tmp = Vector2Wide::default();
        Vector2Wide::subtract(&csi.hinge, &hinge_softness, &mut csi_hinge_tmp);
        csi.hinge = csi_hinge_tmp;

        accumulated_impulses.ball_socket += csi.ball_socket;
        accumulated_impulses.hinge += csi.hinge;

        Self::apply_impulse(&offset_a, &offset_b, &hinge_jacobian, inertia_a, inertia_b, &csi, wsv_a, wsv_b);
    }
}
