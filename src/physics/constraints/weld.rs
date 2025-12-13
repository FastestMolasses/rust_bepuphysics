use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::symmetric6x6_wide::Symmetric6x6Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector::Vector;
use glam::{Quat, Vec3};

pub const BATCH_TYPE_ID: i32 = 31;

#[repr(C)]
pub struct Weld {
    pub local_offset: Vec3,
    pub local_orientation: Quat,
    pub spring_settings: SpringSettings,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct WeldPrestepData {
    pub local_offset: Vector3Wide,
    pub local_orientation: QuaternionWide,
    pub spring_settings: SpringSettingsWide,
}

impl WeldPrestepData {
    #[inline(always)]
    pub fn build_description(&self, description: &mut Weld, _bundle_index: usize) {
        Vector3Wide::read_first(&self.local_offset, &mut description.local_offset);
        QuaternionWide::read_first(&self.local_orientation, &mut description.local_orientation);
        SpringSettingsWide::read_first(&self.spring_settings, &mut description.spring_settings);
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct WeldAccumulatedImpulses {
    pub orientation: Vector3Wide,
    pub offset: Vector3Wide,
}

pub struct WeldFunctions;

impl WeldFunctions {
    #[inline(always)]
    fn apply_impulse(
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        offset: &Vector3Wide,
        orientation_csi: &Vector3Wide,
        offset_csi: &Vector3Wide,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
    ) {
        // J = [ 0, I,                                          0, -I ]
        //     [ I, skewSymmetric(localOffset * orientationA), -I,  0 ]
        // linearImpulseA = offsetCSI
        let linear_change_a = Vector3Wide::scale(offset_csi, &inertia_a.inverse_mass);
        velocity_a.linear = velocity_a.linear + linear_change_a;

        // angularImpulseA = orientationCSI + worldOffset x offsetCSI
        let mut offset_world_impulse = Vector3Wide::default();
        unsafe { Vector3Wide::cross_without_overlap(offset, offset_csi, &mut offset_world_impulse); }
        let angular_impulse_a = offset_world_impulse + *orientation_csi;
        let mut angular_change_a = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(&angular_impulse_a, &inertia_a.inverse_inertia_tensor, &mut angular_change_a);
        velocity_a.angular = velocity_a.angular + angular_change_a;

        // linearImpulseB = -offsetCSI
        let negated_linear_change_b = Vector3Wide::scale(offset_csi, &inertia_b.inverse_mass);
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&velocity_b.linear, &negated_linear_change_b, &mut tmp);
        velocity_b.linear = tmp;

        // angularImpulseB = -orientationCSI
        let mut negated_angular_change_b = Vector3Wide::default();
        Symmetric3x3Wide::transform_without_overlap(orientation_csi, &inertia_b.inverse_inertia_tensor, &mut negated_angular_change_b);
        Vector3Wide::subtract(&velocity_b.angular, &negated_angular_change_b, &mut tmp);
        velocity_b.angular = tmp;
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &WeldPrestepData,
        accumulated_impulses: &WeldAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset, orientation_a, &mut offset);
        Self::apply_impulse(inertia_a, inertia_b, &offset, &accumulated_impulses.orientation, &accumulated_impulses.offset, wsv_a, wsv_b);
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
        prestep: &WeldPrestepData,
        accumulated_impulses: &mut WeldAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        // Compute world-space offset
        let mut offset = Vector3Wide::default();
        QuaternionWide::transform_without_overlap(&prestep.local_offset, orientation_a, &mut offset);

        // Effective mass = (J * M^-1 * JT)^-1, a 6x6 matrix decomposed into blocks:
        // [A, B^T]   where A = Ia^-1 + Ib^-1 (3x3 symmetric),
        // [B,   D]   B = skew(offset) * Ia^-1, D = Ma^-1+Mb^-1 + skew(offset)*Ia^-1*skew(offset)^T
        let mut jmjt_a = Symmetric3x3Wide::default();
        Symmetric3x3Wide::add(&inertia_a.inverse_inertia_tensor, &inertia_b.inverse_inertia_tensor, &mut jmjt_a);
        let x_ab = Matrix3x3Wide::create_cross_product(&offset);
        let mut jmjt_b = Matrix3x3Wide::default();
        Symmetric3x3Wide::multiply(&inertia_a.inverse_inertia_tensor, &x_ab, &mut jmjt_b);
        let mut jmjt_d = Symmetric3x3Wide::default();
        Symmetric3x3Wide::complete_matrix_sandwich_transpose(&x_ab, &jmjt_b, &mut jmjt_d);
        let diagonal_add = inertia_a.inverse_mass + inertia_b.inverse_mass;
        jmjt_d.xx += diagonal_add;
        jmjt_d.yy += diagonal_add;
        jmjt_d.zz += diagonal_add;

        // Position error
        let mut position_error = Vector3Wide::default();
        let mut ab_plus_offset_b = Vector3Wide::default();
        Vector3Wide::subtract(position_b, position_a, &mut ab_plus_offset_b);
        Vector3Wide::subtract(&ab_plus_offset_b, &offset, &mut position_error);
        // The above is positionB - positionA - offset, which is equivalent to (positionB + offsetB) - (positionA + offsetA)
        // but since offsetB = 0 for weld and offset = offsetA rotated, same thing.

        // Orientation error
        let mut target_orientation_b = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(&prestep.local_orientation, orientation_a, &mut target_orientation_b);
        let conjugated_target = QuaternionWide::conjugate(&target_orientation_b);
        let mut rotation_error = QuaternionWide::default();
        QuaternionWide::concatenate_without_overlap(&conjugated_target, orientation_b, &mut rotation_error);
        let mut rotation_error_axis = Vector3Wide::default();
        let mut rotation_error_length = Vector::<f32>::splat(0.0);
        QuaternionWide::get_axis_angle_from_quaternion(&rotation_error, &mut rotation_error_axis, &mut rotation_error_length);

        let mut position_error_to_velocity = Vector::<f32>::splat(0.0);
        let mut effective_mass_cfm_scale = Vector::<f32>::splat(0.0);
        let mut softness_impulse_scale = Vector::<f32>::splat(0.0);
        SpringSettingsWide::compute_springiness(
            &prestep.spring_settings, dt,
            &mut position_error_to_velocity, &mut effective_mass_cfm_scale, &mut softness_impulse_scale,
        );
        let orientation_bias_velocity = rotation_error_axis * (rotation_error_length * position_error_to_velocity);
        let offset_bias_velocity = position_error * position_error_to_velocity;

        // CSV computation — manually inlined for performance
        let mut orientation_csv = Vector3Wide {
            x: orientation_bias_velocity.x - wsv_a.angular.x + wsv_b.angular.x,
            y: orientation_bias_velocity.y - wsv_a.angular.y + wsv_b.angular.y,
            z: orientation_bias_velocity.z - wsv_a.angular.z + wsv_b.angular.z,
        };

        let mut offset_csv = Vector3Wide {
            x: offset_bias_velocity.x - wsv_a.linear.x + wsv_b.linear.x - (wsv_a.angular.y * offset.z - wsv_a.angular.z * offset.y),
            y: offset_bias_velocity.y - wsv_a.linear.y + wsv_b.linear.y - (wsv_a.angular.z * offset.x - wsv_a.angular.x * offset.z),
            z: offset_bias_velocity.z - wsv_a.linear.z + wsv_b.linear.z - (wsv_a.angular.x * offset.y - wsv_a.angular.y * offset.x),
        };

        // Solve using LDLT decomposition of the 6x6 system
        let mut orientation_csi = Vector3Wide::default();
        let mut offset_csi = Vector3Wide::default();
        Symmetric6x6Wide::ldlt_solve(
            &orientation_csv, &offset_csv,
            &jmjt_a, &jmjt_b, &jmjt_d,
            &mut orientation_csi, &mut offset_csi,
        );

        // Apply CFM scale and softness
        orientation_csi.x = orientation_csi.x * effective_mass_cfm_scale - accumulated_impulses.orientation.x * softness_impulse_scale;
        orientation_csi.y = orientation_csi.y * effective_mass_cfm_scale - accumulated_impulses.orientation.y * softness_impulse_scale;
        orientation_csi.z = orientation_csi.z * effective_mass_cfm_scale - accumulated_impulses.orientation.z * softness_impulse_scale;
        accumulated_impulses.orientation.x += orientation_csi.x;
        accumulated_impulses.orientation.y += orientation_csi.y;
        accumulated_impulses.orientation.z += orientation_csi.z;

        offset_csi.x = offset_csi.x * effective_mass_cfm_scale - accumulated_impulses.offset.x * softness_impulse_scale;
        offset_csi.y = offset_csi.y * effective_mass_cfm_scale - accumulated_impulses.offset.y * softness_impulse_scale;
        offset_csi.z = offset_csi.z * effective_mass_cfm_scale - accumulated_impulses.offset.z * softness_impulse_scale;
        accumulated_impulses.offset.x += offset_csi.x;
        accumulated_impulses.offset.y += offset_csi.y;
        accumulated_impulses.offset.z += offset_csi.z;

        Self::apply_impulse(inertia_a, inertia_b, &offset, &orientation_csi, &offset_csi, wsv_a, wsv_b);
    }
}
