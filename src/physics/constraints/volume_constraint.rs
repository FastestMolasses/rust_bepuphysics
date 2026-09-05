use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::num::SimdFloat;

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
    pub fn apply_description(
        &self,
        prestep_data: &mut VolumeConstraintPrestepData,
        _bundle_index: usize,
        inner_index: usize,
    ) {
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
        let mut negative_velocity_change_a = Vector3Wide::default();
        Vector3Wide::scale_to(
            negated_jacobian_a,
            &(*inverse_mass_a * *impulse),
            &mut negative_velocity_change_a,
        );
        let mut velocity_change_b = Vector3Wide::default();
        Vector3Wide::scale_to(
            jacobian_b,
            &(*inverse_mass_b * *impulse),
            &mut velocity_change_b,
        );
        let mut velocity_change_c = Vector3Wide::default();
        Vector3Wide::scale_to(
            jacobian_c,
            &(*inverse_mass_c * *impulse),
            &mut velocity_change_c,
        );
        let mut velocity_change_d = Vector3Wide::default();
        Vector3Wide::scale_to(
            jacobian_d,
            &(*inverse_mass_d * *impulse),
            &mut velocity_change_d,
        );
        let mut tmp = Vector3Wide::default();
        Vector3Wide::subtract(&velocity_a.linear, &negative_velocity_change_a, &mut tmp);
        velocity_a.linear = tmp;
        Vector3Wide::add(&velocity_b.linear, &velocity_change_b, &mut tmp);
        velocity_b.linear = tmp;
        Vector3Wide::add(&velocity_c.linear, &velocity_change_c, &mut tmp);
        velocity_c.linear = tmp;
        Vector3Wide::add(&velocity_d.linear, &velocity_change_d, &mut tmp);
        velocity_d.linear = tmp;
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
        let jacobian_length_squared = jacobian_length_squared.simd_max(Vector::<f32>::splat(1e-14));
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
        let mut ad = Vector3Wide::default();
        let mut negated_ja = Vector3Wide::default();
        let mut jacobian_b = Vector3Wide::default();
        let mut jacobian_c = Vector3Wide::default();
        let mut jacobian_d = Vector3Wide::default();
        let mut _contribution_a = Vector::<f32>::splat(0.0);
        let mut _contribution_b = Vector::<f32>::splat(0.0);
        let mut _contribution_c = Vector::<f32>::splat(0.0);
        let mut _contribution_d = Vector::<f32>::splat(0.0);
        let mut inverse_jacobian_length = Vector::<f32>::splat(0.0);
        Self::compute_jacobian(
            position_a,
            position_b,
            position_c,
            position_d,
            &mut ad,
            &mut negated_ja,
            &mut jacobian_b,
            &mut jacobian_c,
            &mut jacobian_d,
            &mut _contribution_a,
            &mut _contribution_b,
            &mut _contribution_c,
            &mut _contribution_d,
            &mut inverse_jacobian_length,
        );
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
        let mut ad = Vector3Wide::default();
        let mut negated_ja = Vector3Wide::default();
        let mut jacobian_b = Vector3Wide::default();
        let mut jacobian_c = Vector3Wide::default();
        let mut jacobian_d = Vector3Wide::default();
        let mut contribution_a = Vector::<f32>::splat(0.0);
        let mut contribution_b = Vector::<f32>::splat(0.0);
        let mut contribution_c = Vector::<f32>::splat(0.0);
        let mut contribution_d = Vector::<f32>::splat(0.0);
        let mut inverse_jacobian_length = Vector::<f32>::splat(0.0);
        Self::compute_jacobian(
            position_a,
            position_b,
            position_c,
            position_d,
            &mut ad,
            &mut negated_ja,
            &mut jacobian_b,
            &mut jacobian_c,
            &mut jacobian_d,
            &mut contribution_a,
            &mut contribution_b,
            &mut contribution_c,
            &mut contribution_d,
            &mut inverse_jacobian_length,
        );
        let inverse_jacobian_length_squared = inverse_jacobian_length * inverse_jacobian_length;

        // Guard against degenerate configurations (e.g. all points collinear) where all contributions are zero.
        let inverse_effective_mass = (inverse_jacobian_length_squared
            * (contribution_a * inertia_a.inverse_mass
                + contribution_b * inertia_b.inverse_mass
                + contribution_c * inertia_c.inverse_mass
                + contribution_d * inertia_d.inverse_mass))
            .simd_max(Vector::<f32>::splat(1e-14));

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

        let effective_mass = effective_mass_cfm_scale / inverse_effective_mass;
        // Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
        let mut volume = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&jacobian_d, &ad, &mut volume);
        let bias_velocity =
            (prestep.target_scaled_volume - volume) * inverse_jacobian_length * position_error_to_velocity;

        let mut negated_velocity_contribution_a = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(
            &negated_ja,
            &wsv_a.linear,
            &mut negated_velocity_contribution_a,
        );
        let mut velocity_contribution_b = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&jacobian_b, &wsv_b.linear, &mut velocity_contribution_b);
        let mut velocity_contribution_c = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&jacobian_c, &wsv_c.linear, &mut velocity_contribution_c);
        let mut velocity_contribution_d = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&jacobian_d, &wsv_d.linear, &mut velocity_contribution_d);
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
