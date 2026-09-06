use crate::out;
use crate::physics::constraints::linear_axis_servo::LinearAxisServoFunctions;
use crate::physics::constraints::motor_settings::{MotorSettings, MotorSettingsWide};
use crate::physics::constraints::servo_settings::ServoSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::mem::MaybeUninit;

pub const BATCH_TYPE_ID: i32 = 39;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisMotor {
    pub local_offset_a: Vec3,
    pub local_offset_b: Vec3,
    pub local_axis: Vec3,
    pub target_velocity: f32,
    pub settings: MotorSettings,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct LinearAxisMotorPrestepData {
    pub local_offset_a: Vector3Wide,
    pub local_offset_b: Vector3Wide,
    pub local_plane_normal: Vector3Wide,
    pub target_velocity: Vector<f32>,
    pub settings: MotorSettingsWide,
}

impl LinearAxisMotor {
    pub fn apply_description(
        &self,
        prestep_data: &mut LinearAxisMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_unit_length_vec3(
                self.local_axis,
                "LinearAxisMotor",
                "local_axis",
            );
            ConstraintChecker::assert_valid_motor_settings(&self.settings, "LinearAxisMotor");
        }
        Vector3Wide::write_slot(
            self.local_offset_a,
            inner_index,
            &mut prestep_data.local_offset_a,
        );
        Vector3Wide::write_slot(
            self.local_offset_b,
            inner_index,
            &mut prestep_data.local_offset_b,
        );
        Vector3Wide::write_slot(
            self.local_axis,
            inner_index,
            &mut prestep_data.local_plane_normal,
        );
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.target_velocity, inner_index) =
                self.target_velocity;
            *GatherScatter::get_mut(&mut prestep_data.settings.maximum_force, inner_index) =
                self.settings.maximum_force;
            *GatherScatter::get_mut(&mut prestep_data.settings.damping, inner_index) =
                self.settings.damping;
        }
    }

    pub fn build_description(
        prestep_data: &LinearAxisMotorPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut LinearAxisMotor,
    ) {
        Vector3Wide::read_slot(
            &prestep_data.local_offset_a,
            inner_index,
            &mut description.local_offset_a,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_offset_b,
            inner_index,
            &mut description.local_offset_b,
        );
        Vector3Wide::read_slot(
            &prestep_data.local_plane_normal,
            inner_index,
            &mut description.local_axis,
        );
        unsafe {
            description.target_velocity =
                *GatherScatter::get(&prestep_data.target_velocity, inner_index);
            description.settings.maximum_force =
                *GatherScatter::get(&prestep_data.settings.maximum_force, inner_index);
            description.settings.damping =
                *GatherScatter::get(&prestep_data.settings.damping, inner_index);
        }
    }
}

impl LinearAxisMotorPrestepData {
    /// Legacy build_description on PrestepData (prefer LinearAxisMotor::build_description).
    #[inline(always)]
    pub fn build_description_from_prestep(
        &self,
        description: &mut LinearAxisMotor,
        _bundle_index: usize,
    ) {
        Vector3Wide::read_first(&self.local_offset_a, &mut description.local_offset_a);
        Vector3Wide::read_first(&self.local_offset_b, &mut description.local_offset_b);
        Vector3Wide::read_first(&self.local_plane_normal, &mut description.local_axis);
        description.target_velocity = unsafe { *GatherScatter::get_first(&self.target_velocity) };
        MotorSettingsWide::read_first(&self.settings, &mut description.settings);
    }
}

pub struct LinearAxisMotorFunctions;

impl LinearAxisMotorFunctions {
    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        orientation_a: &QuaternionWide,
        inertia_a: &crate::physics::body_properties::BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &crate::physics::body_properties::BodyInertiaWide,
        prestep: &LinearAxisMotorPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut crate::physics::body_properties::BodyVelocityWide,
        wsv_b: &mut crate::physics::body_properties::BodyVelocityWide,
    ) {
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut plane_normal_dot = MaybeUninit::<Vector<f32>>::uninit();
        let mut normal = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Vector3Wide>::uninit();
        unsafe {
            LinearAxisServoFunctions::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_plane_normal,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &mut *plane_normal_dot.as_mut_ptr(),
                &mut *normal.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let normal = unsafe { normal.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };
        let angular_impulse_to_velocity_a = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_ja,
            &inertia_a.inverse_inertia_tensor
        ));
        let angular_impulse_to_velocity_b = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_jb,
            &inertia_b.inverse_inertia_tensor
        ));
        LinearAxisServoFunctions::apply_impulse(
            &normal,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
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
        inertia_a: &crate::physics::body_properties::BodyInertiaWide,
        position_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        inertia_b: &crate::physics::body_properties::BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &LinearAxisMotorPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut crate::physics::body_properties::BodyVelocityWide,
        wsv_b: &mut crate::physics::body_properties::BodyVelocityWide,
    ) {
        let ab = out!(Vector3Wide::subtract(position_b, position_a));
        let mut plane_normal_dot = MaybeUninit::<Vector<f32>>::uninit();
        let mut normal = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_jb = MaybeUninit::<Vector3Wide>::uninit();
        unsafe {
            LinearAxisServoFunctions::compute_jacobians(
                &ab,
                orientation_a,
                orientation_b,
                &prestep.local_plane_normal,
                &prestep.local_offset_a,
                &prestep.local_offset_b,
                &mut *plane_normal_dot.as_mut_ptr(),
                &mut *normal.as_mut_ptr(),
                &mut *angular_ja.as_mut_ptr(),
                &mut *angular_jb.as_mut_ptr(),
            );
        }
        let normal = unsafe { normal.assume_init() };
        let angular_ja = unsafe { angular_ja.assume_init() };
        let angular_jb = unsafe { angular_jb.assume_init() };
        let mut effective_mass_cfm_scale = MaybeUninit::<Vector<f32>>::uninit();
        let mut softness_impulse_scale = MaybeUninit::<Vector<f32>>::uninit();
        let mut maximum_impulse = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            MotorSettingsWide::compute_softness(
                &prestep.settings,
                dt,
                &mut *effective_mass_cfm_scale.as_mut_ptr(),
                &mut *softness_impulse_scale.as_mut_ptr(),
                &mut *maximum_impulse.as_mut_ptr(),
            );
        }
        let effective_mass_cfm_scale = unsafe { effective_mass_cfm_scale.assume_init() };
        let softness_impulse_scale = unsafe { softness_impulse_scale.assume_init() };
        let maximum_impulse = unsafe { maximum_impulse.assume_init() };
        let mut angular_impulse_to_velocity_a = MaybeUninit::<Vector3Wide>::uninit();
        let mut angular_impulse_to_velocity_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut effective_mass = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            LinearAxisServoFunctions::compute_effective_mass(
                &angular_ja,
                &angular_jb,
                inertia_a,
                inertia_b,
                &effective_mass_cfm_scale,
                &mut *angular_impulse_to_velocity_a.as_mut_ptr(),
                &mut *angular_impulse_to_velocity_b.as_mut_ptr(),
                &mut *effective_mass.as_mut_ptr(),
            );
        }
        let angular_impulse_to_velocity_a = unsafe { angular_impulse_to_velocity_a.assume_init() };
        let angular_impulse_to_velocity_b = unsafe { angular_impulse_to_velocity_b.assume_init() };
        let effective_mass = unsafe { effective_mass.assume_init() };

        // csv = dot(wsvA.Linear - wsvB.Linear, normal) + dot(wsvA.Angular, angularJA) + dot(wsvB.Angular, angularJB)
        let linear_diff = out!(Vector3Wide::subtract(&wsv_a.linear, &wsv_b.linear));
        let csv = Vector3Wide::dot_val(&linear_diff, &normal)
            + Vector3Wide::dot_val(&wsv_a.angular, &angular_ja)
            + Vector3Wide::dot_val(&wsv_b.angular, &angular_jb);

        let mut csi = effective_mass * (-prestep.target_velocity - csv)
            - *accumulated_impulses * softness_impulse_scale;

        ServoSettingsWide::clamp_impulse_1d(&maximum_impulse, accumulated_impulses, &mut csi);
        LinearAxisServoFunctions::apply_impulse(
            &normal,
            &angular_impulse_to_velocity_a,
            &angular_impulse_to_velocity_b,
            inertia_a,
            inertia_b,
            &csi,
            wsv_a,
            wsv_b,
        );
    }
}
