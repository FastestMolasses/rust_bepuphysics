// Translated from BepuPhysics/Constraints/Contact/ContactNonconvexCommon.cs

use crate::out;
use crate::out_unsafe;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::contact::contact_convex_common::*;
use crate::physics::constraints::contact::penetration_limit::PenetrationLimit;
use crate::physics::constraints::contact::penetration_limit_one_body::PenetrationLimitOneBody;
use crate::physics::constraints::contact::tangent_friction::TangentFriction;
use crate::physics::constraints::contact::tangent_friction_one_body::TangentFrictionOneBody;
use crate::physics::constraints::spring_settings::SpringSettingsWide;
use crate::physics::helpers::Helpers;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// Per-contact prestep data for nonconvex constraints.
/// Each contact has its own normal (unlike convex contacts which share a single normal).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NonconvexContactPrestepData {
    pub offset: Vector3Wide,
    pub depth: Vector<f32>,
    pub normal: Vector3Wide,
}

/// Trait for nonconvex contact prestep data (one body or two body).
pub trait INonconvexContactPrestep: IContactPrestep {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData;
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData;
}

/// Trait for two-body nonconvex contact prestep data with an offset_b.
pub trait ITwoBodyNonconvexContactPrestep: INonconvexContactPrestep {
    fn get_offset_b(&self) -> &Vector3Wide;
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide;
}

/// Accumulated impulses for a single nonconvex contact.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NonconvexAccumulatedImpulses {
    pub tangent: Vector2Wide,
    pub penetration: Vector<f32>,
}

/// Trait for nonconvex contact accumulated impulses.
pub trait INonconvexContactAccumulatedImpulses {
    fn contact_count() -> i32;
    fn get_impulses_for_contact(&self, index: usize) -> &NonconvexAccumulatedImpulses;
    fn get_impulses_for_contact_mut(&mut self, index: usize) -> &mut NonconvexAccumulatedImpulses;
}

/// Common solver functions for nonconvex one-body contact constraints.
/// In C# this was a generic struct implementing IOneBodyConstraintFunctions.
/// In Rust we implement the logic for any nonconvex one-body prestep + impulse pair.
pub struct ContactNonconvexOneBodyFunctions;

impl ContactNonconvexOneBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep<TPrestep: INonconvexContactPrestep>(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        prestep: &mut TPrestep,
    ) {
        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact_mut(i);
            PenetrationLimitOneBody::update_penetration_depth(
                dt,
                &contact.offset,
                &contact.normal,
                velocity_a,
                &mut contact.depth,
            );
        }
    }

    #[inline(always)]
    pub fn warm_start<TPrestep: INonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut TPrestep,
        accumulated_impulses: &mut TAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact(i);
            let normal = contact.normal;
            let offset = contact.offset;
            let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&normal), 2);
            let contact_impulse = accumulated_impulses.get_impulses_for_contact(i);
            let tangent = contact_impulse.tangent;
            let penetration = contact_impulse.penetration;
            TangentFrictionOneBody::warm_start(&x, &z, &offset, inertia_a, &tangent, wsv_a);
            PenetrationLimitOneBody::warm_start(inertia_a, &normal, &offset, &penetration, wsv_a);
        }
    }

    #[inline(always)]
    pub fn solve<TPrestep: INonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestep,
        accumulated_impulses: &mut TAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        //Note that, unlike convex manifolds, we simply solve every contact in sequence rather than tangent->penetration.
        //This is not for any principled reason- only simplicity. May want to reconsider later, but remember the significant change in access pattern.
        let material = prestep.get_material_properties();
        let (mut position_error_to_velocity, mut effective_mass_cfm_scale, mut softness_impulse_scale) = Default::default();
        SpringSettingsWide::compute_springiness(
            &material.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let friction_coefficient = prestep.get_material_properties().friction_coefficient;
        let maximum_recovery_velocity = prestep.get_material_properties().maximum_recovery_velocity;

        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact(i);
            let normal = contact.normal;
            let offset = contact.offset;
            let depth = contact.depth;
            let contact_impulse = accumulated_impulses.get_impulses_for_contact_mut(i);
            PenetrationLimitOneBody::solve(
                inertia_a,
                &normal,
                &offset,
                &depth,
                &position_error_to_velocity,
                &effective_mass_cfm_scale,
                &maximum_recovery_velocity,
                &inverse_dt_wide,
                &softness_impulse_scale,
                &mut contact_impulse.penetration,
                wsv_a,
            );
            let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&normal), 2);
            let maximum_tangent_impulse = friction_coefficient * contact_impulse.penetration;
            TangentFrictionOneBody::solve(
                &x,
                &z,
                &offset,
                inertia_a,
                &maximum_tangent_impulse,
                &mut contact_impulse.tangent,
                wsv_a,
            );
        }
    }
}

/// Common solver functions for nonconvex two-body contact constraints.
pub struct ContactNonconvexTwoBodyFunctions;

impl ContactNonconvexTwoBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep<TPrestep: ITwoBodyNonconvexContactPrestep>(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        prestep: &mut TPrestep,
    ) {
        let offset_b = *prestep.get_offset_b();
        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact_mut(i);
            PenetrationLimit::update_penetration_depth(
                dt,
                &contact.offset,
                &offset_b,
                &contact.normal,
                velocity_a,
                velocity_b,
                &mut contact.depth,
            );
        }
    }

    #[inline(always)]
    pub fn warm_start<TPrestep: ITwoBodyNonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut TPrestep,
        accumulated_impulses: &mut TAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let offset_b = *prestep.get_offset_b();
        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact(i);
            let normal = contact.normal;
            let offset = contact.offset;
            let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&normal), 2);
            let mut contact_offset_b = Vector3Wide::default();
            Vector3Wide::subtract(&offset, &offset_b, &mut contact_offset_b);
            let contact_impulse = accumulated_impulses.get_impulses_for_contact(i);
            let tangent = contact_impulse.tangent;
            let penetration = contact_impulse.penetration;
            TangentFriction::warm_start(
                &x,
                &z,
                &offset,
                &contact_offset_b,
                inertia_a,
                inertia_b,
                &tangent,
                wsv_a,
                wsv_b,
            );
            PenetrationLimit::warm_start(
                inertia_a,
                inertia_b,
                &normal,
                &offset,
                &contact_offset_b,
                &penetration,
                wsv_a,
                wsv_b,
            );
        }
    }

    #[inline(always)]
    pub fn solve<TPrestep: ITwoBodyNonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut TPrestep,
        accumulated_impulses: &mut TAccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        //Note that, unlike convex manifolds, we simply solve every contact in sequence rather than tangent->penetration.
        let offset_b = *prestep.get_offset_b();
        let material = prestep.get_material_properties();
        let (mut position_error_to_velocity, mut effective_mass_cfm_scale, mut softness_impulse_scale) = Default::default();
        SpringSettingsWide::compute_springiness(
            &material.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let friction_coefficient = prestep.get_material_properties().friction_coefficient;
        let maximum_recovery_velocity = prestep.get_material_properties().maximum_recovery_velocity;

        let contact_count = TPrestep::contact_count();
        for i in 0..contact_count as usize {
            let contact = prestep.get_contact(i);
            let normal = contact.normal;
            let offset = contact.offset;
            let depth = contact.depth;
            let mut contact_offset_b = Vector3Wide::default();
            Vector3Wide::subtract(&offset, &offset_b, &mut contact_offset_b);
            let contact_impulse = accumulated_impulses.get_impulses_for_contact_mut(i);
            PenetrationLimit::solve(
                inertia_a,
                inertia_b,
                &normal,
                &offset,
                &contact_offset_b,
                &depth,
                &position_error_to_velocity,
                &effective_mass_cfm_scale,
                &maximum_recovery_velocity,
                &inverse_dt_wide,
                &softness_impulse_scale,
                &mut contact_impulse.penetration,
                wsv_a,
                wsv_b,
            );
            let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&normal), 2);
            let maximum_tangent_impulse = friction_coefficient * contact_impulse.penetration;
            TangentFriction::solve(
                &x,
                &z,
                &offset,
                &contact_offset_b,
                inertia_a,
                inertia_b,
                &maximum_tangent_impulse,
                &mut contact_impulse.tangent,
                wsv_a,
                wsv_b,
            );
        }
    }
}
