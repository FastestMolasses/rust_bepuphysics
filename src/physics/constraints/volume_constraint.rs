use crate::out;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::vector::{HwMinMax, Vector};
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
use std::mem::MaybeUninit;

pub const BATCH_TYPE_ID: i32 = 32;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VolumeConstraint {
    pub target_scaled_volume: f32,
    pub spring_settings: SpringSettings,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VolumeConstraintPrestepData {
    pub target_scaled_volume: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
}

impl VolumeConstraint {
    /// Creates a new volume constraint, initializing the target volume using a set of initial positions.
    #[inline(always)]
    pub fn new(a: Vec3, b: Vec3, c: Vec3, d: Vec3, spring_settings: SpringSettings) -> Self {
        Self {
            target_scaled_volume: (b - a).cross(c - a).dot(d - a),
            spring_settings,
        }
    }

    pub fn apply_description(
        &self,
        prestep_data: &mut VolumeConstraintPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
        #[cfg(debug_assertions)]
        {
            use crate::physics::constraints::constraint_checker::ConstraintChecker;
            ConstraintChecker::assert_valid_spring_settings(
                &self.spring_settings,
                "VolumeConstraint",
            );
        }
        unsafe {
            *GatherScatter::get_mut(&mut prestep_data.target_scaled_volume, inner_index) =
                self.target_scaled_volume;
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
        prestep_data: &VolumeConstraintPrestepData,
        _bundle_index: usize,
        inner_index: usize,
        description: &mut VolumeConstraint,
    ) {
        unsafe {
            description.target_scaled_volume =
                *GatherScatter::get(&prestep_data.target_scaled_volume, inner_index);
            description.spring_settings.angular_frequency =
                *GatherScatter::get(&prestep_data.spring_settings.angular_frequency, inner_index);
            description.spring_settings.twice_damping_ratio = *GatherScatter::get(
                &prestep_data.spring_settings.twice_damping_ratio,
                inner_index,
            );
        }
    }
}

impl VolumeConstraintPrestepData {
    /// Legacy build_description on PrestepData (prefer VolumeConstraint::build_description).
    #[inline(always)]
    pub fn build_description_from_prestep(
        &self,
        description: &mut VolumeConstraint,
        _bundle_index: usize,
    ) {
        description.target_scaled_volume =
            unsafe { *GatherScatter::get_first(&self.target_scaled_volume) };
        SpringSettingsWide::read_first(&self.spring_settings, &mut description.spring_settings);
    }
}

pub struct VolumeConstraintFunctions;

impl VolumeConstraintFunctions {
    #[inline(always)]
    fn apply_impulse(
        inverse_mass_a: &Vector<f32>,
        inverse_mass_b: &Vector<f32>,
        inverse_mass_c: &Vector<f32>,
        inverse_mass_d: &Vector<f32>,
        negated_jacobian_a: &Vector3Wide,
        jacobian_b: &Vector3Wide,
        jacobian_c: &Vector3Wide,
        jacobian_d: &Vector3Wide,
        impulse: &Vector<f32>,
        velocity_a: &mut BodyVelocityWide,
        velocity_b: &mut BodyVelocityWide,
        velocity_c: &mut BodyVelocityWide,
        velocity_d: &mut BodyVelocityWide,
    ) {
        let negative_velocity_change_a = out!(Vector3Wide::scale_to(
            negated_jacobian_a,
            &(*inverse_mass_a * *impulse)
        ));
        let velocity_change_b =
            out!(Vector3Wide::scale_to(jacobian_b, &(*inverse_mass_b * *impulse)));
        let velocity_change_c =
            out!(Vector3Wide::scale_to(jacobian_c, &(*inverse_mass_c * *impulse)));
        let velocity_change_d =
            out!(Vector3Wide::scale_to(jacobian_d, &(*inverse_mass_d * *impulse)));
        velocity_a.linear = out!(Vector3Wide::subtract(
            &velocity_a.linear,
            &negative_velocity_change_a
        ));
        velocity_b.linear = out!(Vector3Wide::add(&velocity_b.linear, &velocity_change_b));
        velocity_c.linear = out!(Vector3Wide::add(&velocity_c.linear, &velocity_change_c));
        velocity_d.linear = out!(Vector3Wide::add(&velocity_d.linear, &velocity_change_d));
    }

    #[inline(always)]
    fn compute_jacobian(
        position_a: &Vector3Wide,
        position_b: &Vector3Wide,
        position_c: &Vector3Wide,
        position_d: &Vector3Wide,
        ad: &mut Vector3Wide,
        negated_ja: &mut Vector3Wide,
        jacobian_b: &mut Vector3Wide,
        jacobian_c: &mut Vector3Wide,
        jacobian_d: &mut Vector3Wide,
        contribution_a: &mut Vector<f32>,
        contribution_b: &mut Vector<f32>,
        contribution_c: &mut Vector<f32>,
        contribution_d: &mut Vector<f32>,
        inverse_jacobian_length: &mut Vector<f32>,
    ) {
        let ab = *position_b - *position_a;
        let ac = *position_c - *position_a;
        *ad = *position_d - *position_a;
        unsafe {
            Vector3Wide::cross_without_overlap(&ac, ad, jacobian_b);
            Vector3Wide::cross_without_overlap(ad, &ab, jacobian_c);
            Vector3Wide::cross_without_overlap(&ab, &ac, jacobian_d);
        }
        Vector3Wide::add(jacobian_b, jacobian_c, negated_ja);
        let tmp = *negated_ja;
        Vector3Wide::add(jacobian_d, &tmp, negated_ja);
        // Normalize the jacobian to unit length so the inverse effective mass is a bounded weighted average of
        // inverse masses regardless of tetrahedron size; the scale factor cancels in the solve, so the impulse is unchanged.
        Vector3Wide::dot(negated_ja, negated_ja, contribution_a);
        Vector3Wide::dot(jacobian_b, jacobian_b, contribution_b);
        Vector3Wide::dot(jacobian_c, jacobian_c, contribution_c);
        Vector3Wide::dot(jacobian_d, jacobian_d, contribution_d);
        let jacobian_length_squared =
            *contribution_a + *contribution_b + *contribution_c + *contribution_d;
        // Guard against the collinear degeneracy (all cross products vanish; generic coplanar cases stay nonzero).
        let jacobian_length_squared = Vector::<f32>::splat(1e-14).hw_max(jacobian_length_squared);
        *inverse_jacobian_length = math_helper::fast_reciprocal_square_root(jacobian_length_squared);
    }

    #[inline(always)]
    pub fn warm_start(
        position_a: &Vector3Wide,
        _orientation_a: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_a: &BodyInertiaWide,
        position_b: &Vector3Wide,
        _orientation_b: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_b: &BodyInertiaWide,
        position_c: &Vector3Wide,
        _orientation_c: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_c: &BodyInertiaWide,
        position_d: &Vector3Wide,
        _orientation_d: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_d: &BodyInertiaWide,
        _prestep: &VolumeConstraintPrestepData,
        accumulated_impulses: &Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
        wsv_d: &mut BodyVelocityWide,
    ) {
        let mut ad = MaybeUninit::<Vector3Wide>::uninit();
        let mut negated_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_c = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_d = MaybeUninit::<Vector3Wide>::uninit();
        let mut contribution_a = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_b = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_c = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_d = MaybeUninit::<Vector<f32>>::uninit();
        let mut inverse_jacobian_length = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            Self::compute_jacobian(
                position_a,
                position_b,
                position_c,
                position_d,
                &mut *ad.as_mut_ptr(),
                &mut *negated_ja.as_mut_ptr(),
                &mut *jacobian_b.as_mut_ptr(),
                &mut *jacobian_c.as_mut_ptr(),
                &mut *jacobian_d.as_mut_ptr(),
                &mut *contribution_a.as_mut_ptr(),
                &mut *contribution_b.as_mut_ptr(),
                &mut *contribution_c.as_mut_ptr(),
                &mut *contribution_d.as_mut_ptr(),
                &mut *inverse_jacobian_length.as_mut_ptr(),
            );
        }
        let negated_ja = unsafe { negated_ja.assume_init() };
        let jacobian_b = unsafe { jacobian_b.assume_init() };
        let jacobian_c = unsafe { jacobian_c.assume_init() };
        let jacobian_d = unsafe { jacobian_d.assume_init() };
        let inverse_jacobian_length = unsafe { inverse_jacobian_length.assume_init() };
        // The accumulated impulse is in unit-jacobian space; replay it through inverseJacobianLength * J_raw.
        Self::apply_impulse(
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            &inertia_c.inverse_mass,
            &inertia_d.inverse_mass,
            &negated_ja,
            &jacobian_b,
            &jacobian_c,
            &jacobian_d,
            &(inverse_jacobian_length * *accumulated_impulses),
            wsv_a,
            wsv_b,
            wsv_c,
            wsv_d,
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
        position_c: &Vector3Wide,
        _orientation_c: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_c: &BodyInertiaWide,
        position_d: &Vector3Wide,
        _orientation_d: &crate::utilities::quaternion_wide::QuaternionWide,
        inertia_d: &BodyInertiaWide,
        dt: f32,
        _inverse_dt: f32,
        prestep: &VolumeConstraintPrestepData,
        accumulated_impulses: &mut Vector<f32>,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
        wsv_c: &mut BodyVelocityWide,
        wsv_d: &mut BodyVelocityWide,
    ) {
        let mut ad = MaybeUninit::<Vector3Wide>::uninit();
        let mut negated_ja = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_b = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_c = MaybeUninit::<Vector3Wide>::uninit();
        let mut jacobian_d = MaybeUninit::<Vector3Wide>::uninit();
        let mut contribution_a = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_b = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_c = MaybeUninit::<Vector<f32>>::uninit();
        let mut contribution_d = MaybeUninit::<Vector<f32>>::uninit();
        let mut inverse_jacobian_length = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            Self::compute_jacobian(
                position_a,
                position_b,
                position_c,
                position_d,
                &mut *ad.as_mut_ptr(),
                &mut *negated_ja.as_mut_ptr(),
                &mut *jacobian_b.as_mut_ptr(),
                &mut *jacobian_c.as_mut_ptr(),
                &mut *jacobian_d.as_mut_ptr(),
                &mut *contribution_a.as_mut_ptr(),
                &mut *contribution_b.as_mut_ptr(),
                &mut *contribution_c.as_mut_ptr(),
                &mut *contribution_d.as_mut_ptr(),
                &mut *inverse_jacobian_length.as_mut_ptr(),
            );
        }
        let ad = unsafe { ad.assume_init() };
        let negated_ja = unsafe { negated_ja.assume_init() };
        let jacobian_b = unsafe { jacobian_b.assume_init() };
        let jacobian_c = unsafe { jacobian_c.assume_init() };
        let jacobian_d = unsafe { jacobian_d.assume_init() };
        let contribution_a = unsafe { contribution_a.assume_init() };
        let contribution_b = unsafe { contribution_b.assume_init() };
        let contribution_c = unsafe { contribution_c.assume_init() };
        let contribution_d = unsafe { contribution_d.assume_init() };
        let inverse_jacobian_length = unsafe { inverse_jacobian_length.assume_init() };
        let inverse_jacobian_length_squared = inverse_jacobian_length * inverse_jacobian_length;

        // Guard against degenerate configurations (e.g. all points collinear) where all contributions are zero.
        let inverse_effective_mass = Vector::<f32>::splat(1e-14).hw_max(
            inverse_jacobian_length_squared
                * (contribution_a * inertia_a.inverse_mass
                    + contribution_b * inertia_b.inverse_mass
                    + contribution_c * inertia_c.inverse_mass
                    + contribution_d * inertia_d.inverse_mass),
        );

        let mut position_error_to_velocity = MaybeUninit::<Vector<f32>>::uninit();
        let mut effective_mass_cfm_scale = MaybeUninit::<Vector<f32>>::uninit();
        let mut softness_impulse_scale = MaybeUninit::<Vector<f32>>::uninit();
        unsafe {
            SpringSettingsWide::compute_springiness(
                &prestep.spring_settings,
                dt,
                &mut *position_error_to_velocity.as_mut_ptr(),
                &mut *effective_mass_cfm_scale.as_mut_ptr(),
                &mut *softness_impulse_scale.as_mut_ptr(),
            );
        }
        let position_error_to_velocity = unsafe { position_error_to_velocity.assume_init() };
        let effective_mass_cfm_scale = unsafe { effective_mass_cfm_scale.assume_init() };
        let softness_impulse_scale = unsafe { softness_impulse_scale.assume_init() };

        let effective_mass = effective_mass_cfm_scale / inverse_effective_mass;
        // Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
        let volume = out!(Vector3Wide::dot(&jacobian_d, &ad));
        let bias_velocity =
            (prestep.target_scaled_volume - volume) * inverse_jacobian_length * position_error_to_velocity;

        let negated_velocity_contribution_a = out!(Vector3Wide::dot(&negated_ja, &wsv_a.linear));
        let velocity_contribution_b = out!(Vector3Wide::dot(&jacobian_b, &wsv_b.linear));
        let velocity_contribution_c = out!(Vector3Wide::dot(&jacobian_c, &wsv_c.linear));
        let velocity_contribution_d = out!(Vector3Wide::dot(&jacobian_d, &wsv_d.linear));
        let csv = inverse_jacobian_length
            * (velocity_contribution_b + velocity_contribution_c + velocity_contribution_d
                - negated_velocity_contribution_a);
        let csi =
            (bias_velocity - csv) * effective_mass - *accumulated_impulses * softness_impulse_scale;
        *accumulated_impulses += csi;

        Self::apply_impulse(
            &inertia_a.inverse_mass,
            &inertia_b.inverse_mass,
            &inertia_c.inverse_mass,
            &inertia_d.inverse_mass,
            &negated_ja,
            &jacobian_b,
            &jacobian_c,
            &jacobian_d,
            &(inverse_jacobian_length * csi),
            wsv_a,
            wsv_b,
            wsv_c,
            wsv_d,
        );
    }
}
