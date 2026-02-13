// Translated from BepuPhysics/Constraints/Contact/ContactConvexTypes.cs

use crate::out_unsafe;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::physics::constraints::contact::contact_convex_common::*;
use crate::physics::constraints::contact::penetration_limit::PenetrationLimit;
use crate::physics::constraints::contact::penetration_limit_one_body::PenetrationLimitOneBody;
use crate::physics::constraints::contact::tangent_friction::TangentFriction;
use crate::physics::constraints::contact::tangent_friction_one_body::TangentFrictionOneBody;
use crate::physics::constraints::contact::twist_friction::TwistFriction;
use crate::physics::constraints::contact::twist_friction_one_body::TwistFrictionOneBody;
use crate::physics::constraints::spring_settings::SpringSettingsWide;
use crate::physics::helpers::Helpers;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::cmp::SimdPartialEq;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;

// ============================================================================
// Accumulated Impulse types
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact1AccumulatedImpulses {
    pub tangent: Vector2Wide,
    pub penetration0: Vector<f32>,
    pub twist: Vector<f32>,
}

impl IContactAccumulatedImpulses for Contact1AccumulatedImpulses {
    fn contact_count() -> i32 {
        1
    }
}

impl IConvexContactAccumulatedImpulses for Contact1AccumulatedImpulses {
    fn get_tangent_friction(&self) -> &Vector2Wide {
        &self.tangent
    }
    fn get_tangent_friction_mut(&mut self) -> &mut Vector2Wide {
        &mut self.tangent
    }
    fn get_twist_friction(&self) -> &Vector<f32> {
        &self.twist
    }
    fn get_twist_friction_mut(&mut self) -> &mut Vector<f32> {
        &mut self.twist
    }
    fn get_penetration_impulse_for_contact(&self, index: usize) -> &Vector<f32> {
        debug_assert!(index < 1);
        &self.penetration0
    }
    fn get_penetration_impulse_for_contact_mut(&mut self, index: usize) -> &mut Vector<f32> {
        debug_assert!(index < 1);
        &mut self.penetration0
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2AccumulatedImpulses {
    pub tangent: Vector2Wide,
    pub penetration0: Vector<f32>,
    pub penetration1: Vector<f32>,
    pub twist: Vector<f32>,
}

impl IContactAccumulatedImpulses for Contact2AccumulatedImpulses {
    fn contact_count() -> i32 {
        2
    }
}

impl IConvexContactAccumulatedImpulses for Contact2AccumulatedImpulses {
    fn get_tangent_friction(&self) -> &Vector2Wide {
        &self.tangent
    }
    fn get_tangent_friction_mut(&mut self) -> &mut Vector2Wide {
        &mut self.tangent
    }
    fn get_twist_friction(&self) -> &Vector<f32> {
        &self.twist
    }
    fn get_twist_friction_mut(&mut self) -> &mut Vector<f32> {
        &mut self.twist
    }
    fn get_penetration_impulse_for_contact(&self, index: usize) -> &Vector<f32> {
        debug_assert!(index < 2);
        unsafe { &*(&self.penetration0 as *const Vector<f32>).add(index) }
    }
    fn get_penetration_impulse_for_contact_mut(&mut self, index: usize) -> &mut Vector<f32> {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.penetration0 as *mut Vector<f32>).add(index) }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3AccumulatedImpulses {
    pub tangent: Vector2Wide,
    pub penetration0: Vector<f32>,
    pub penetration1: Vector<f32>,
    pub penetration2: Vector<f32>,
    pub twist: Vector<f32>,
}

impl IContactAccumulatedImpulses for Contact3AccumulatedImpulses {
    fn contact_count() -> i32 {
        3
    }
}

impl IConvexContactAccumulatedImpulses for Contact3AccumulatedImpulses {
    fn get_tangent_friction(&self) -> &Vector2Wide {
        &self.tangent
    }
    fn get_tangent_friction_mut(&mut self) -> &mut Vector2Wide {
        &mut self.tangent
    }
    fn get_twist_friction(&self) -> &Vector<f32> {
        &self.twist
    }
    fn get_twist_friction_mut(&mut self) -> &mut Vector<f32> {
        &mut self.twist
    }
    fn get_penetration_impulse_for_contact(&self, index: usize) -> &Vector<f32> {
        debug_assert!(index < 3);
        unsafe { &*(&self.penetration0 as *const Vector<f32>).add(index) }
    }
    fn get_penetration_impulse_for_contact_mut(&mut self, index: usize) -> &mut Vector<f32> {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.penetration0 as *mut Vector<f32>).add(index) }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4AccumulatedImpulses {
    pub tangent: Vector2Wide,
    pub penetration0: Vector<f32>,
    pub penetration1: Vector<f32>,
    pub penetration2: Vector<f32>,
    pub penetration3: Vector<f32>,
    pub twist: Vector<f32>,
}

impl IContactAccumulatedImpulses for Contact4AccumulatedImpulses {
    fn contact_count() -> i32 {
        4
    }
}

impl IConvexContactAccumulatedImpulses for Contact4AccumulatedImpulses {
    fn get_tangent_friction(&self) -> &Vector2Wide {
        &self.tangent
    }
    fn get_tangent_friction_mut(&mut self) -> &mut Vector2Wide {
        &mut self.tangent
    }
    fn get_twist_friction(&self) -> &Vector<f32> {
        &self.twist
    }
    fn get_twist_friction_mut(&mut self) -> &mut Vector<f32> {
        &mut self.twist
    }
    fn get_penetration_impulse_for_contact(&self, index: usize) -> &Vector<f32> {
        debug_assert!(index < 4);
        unsafe { &*(&self.penetration0 as *const Vector<f32>).add(index) }
    }
    fn get_penetration_impulse_for_contact_mut(&mut self, index: usize) -> &mut Vector<f32> {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.penetration0 as *mut Vector<f32>).add(index) }
    }
}

// ============================================================================
// FrictionHelpers
// ============================================================================

pub struct FrictionHelpers;

impl FrictionHelpers {
    #[inline(always)]
    pub fn compute_friction_center_2(
        offset_a0: &Vector3Wide,
        offset_a1: &Vector3Wide,
        depth0: &Vector<f32>,
        depth1: &Vector<f32>,
        center: &mut Vector3Wide,
    ) {
        //This can sometimes cause a weird center of friction. That's a bit strange, but the alternative is often stranger.
        let zero = Vector::<f32>::splat(0.0);
        let one = Vector::<f32>::splat(1.0);
        let depth0_negative = depth0.simd_lt(zero);
        let depth1_negative = depth1.simd_lt(zero);
        let mut weight0 = depth0_negative.select(zero, one);
        let mut weight1 = depth1_negative.select(zero, one);
        let weight_sum = weight0 + weight1;
        let use_fallback = weight_sum.simd_eq(zero);
        let weight_sum = use_fallback.select(Vector::<f32>::splat(2.0), weight_sum);
        let inverse_weight_sum = one / weight_sum;
        weight0 = use_fallback.select(inverse_weight_sum, weight0 * inverse_weight_sum);
        weight1 = use_fallback.select(inverse_weight_sum, weight1 * inverse_weight_sum);
        let a0_contribution = Vector3Wide::scale(offset_a0, &weight0);
        let a1_contribution = Vector3Wide::scale(offset_a1, &weight1);
        Vector3Wide::add(&a0_contribution, &a1_contribution, center);
    }

    #[inline(always)]
    pub fn compute_friction_center_3(
        offset_a0: &Vector3Wide,
        offset_a1: &Vector3Wide,
        offset_a2: &Vector3Wide,
        depth0: &Vector<f32>,
        depth1: &Vector<f32>,
        depth2: &Vector<f32>,
        center: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        let one = Vector::<f32>::splat(1.0);
        let depth0_negative = depth0.simd_lt(zero);
        let depth1_negative = depth1.simd_lt(zero);
        let depth2_negative = depth2.simd_lt(zero);
        let mut weight0 = depth0_negative.select(zero, one);
        let mut weight1 = depth1_negative.select(zero, one);
        let mut weight2 = depth2_negative.select(zero, one);
        let weight_sum = weight0 + weight1 + weight2;
        let use_fallback = weight_sum.simd_eq(zero);
        let weight_sum = use_fallback.select(Vector::<f32>::splat(3.0), weight_sum);
        let inverse_weight_sum = one / weight_sum;
        weight0 = use_fallback.select(inverse_weight_sum, weight0 * inverse_weight_sum);
        weight1 = use_fallback.select(inverse_weight_sum, weight1 * inverse_weight_sum);
        weight2 = use_fallback.select(inverse_weight_sum, weight2 * inverse_weight_sum);
        let a0_contribution = Vector3Wide::scale(offset_a0, &weight0);
        let a1_contribution = Vector3Wide::scale(offset_a1, &weight1);
        let a2_contribution = Vector3Wide::scale(offset_a2, &weight2);
        let mut a0a1 = Vector3Wide::default();
        Vector3Wide::add(&a0_contribution, &a1_contribution, &mut a0a1);
        Vector3Wide::add(&a0a1, &a2_contribution, center);
    }

    #[inline(always)]
    pub fn compute_friction_center_4(
        offset_a0: &Vector3Wide,
        offset_a1: &Vector3Wide,
        offset_a2: &Vector3Wide,
        offset_a3: &Vector3Wide,
        depth0: &Vector<f32>,
        depth1: &Vector<f32>,
        depth2: &Vector<f32>,
        depth3: &Vector<f32>,
        center: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        let one = Vector::<f32>::splat(1.0);
        let depth0_negative = depth0.simd_lt(zero);
        let depth1_negative = depth1.simd_lt(zero);
        let depth2_negative = depth2.simd_lt(zero);
        let depth3_negative = depth3.simd_lt(zero);
        let mut weight0 = depth0_negative.select(zero, one);
        let mut weight1 = depth1_negative.select(zero, one);
        let mut weight2 = depth2_negative.select(zero, one);
        let mut weight3 = depth3_negative.select(zero, one);
        let weight_sum = weight0 + weight1 + weight2 + weight3;
        let use_fallback = weight_sum.simd_eq(zero);
        let weight_sum = use_fallback.select(Vector::<f32>::splat(4.0), weight_sum);
        let inverse_weight_sum = one / weight_sum;
        weight0 = use_fallback.select(inverse_weight_sum, weight0 * inverse_weight_sum);
        weight1 = use_fallback.select(inverse_weight_sum, weight1 * inverse_weight_sum);
        weight2 = use_fallback.select(inverse_weight_sum, weight2 * inverse_weight_sum);
        weight3 = use_fallback.select(inverse_weight_sum, weight3 * inverse_weight_sum);
        let a0_contribution = Vector3Wide::scale(offset_a0, &weight0);
        let a1_contribution = Vector3Wide::scale(offset_a1, &weight1);
        let a2_contribution = Vector3Wide::scale(offset_a2, &weight2);
        let a3_contribution = Vector3Wide::scale(offset_a3, &weight3);
        let mut a0a1 = Vector3Wide::default();
        Vector3Wide::add(&a0_contribution, &a1_contribution, &mut a0a1);
        let mut a2a3 = Vector3Wide::default();
        Vector3Wide::add(&a2_contribution, &a3_contribution, &mut a2a3);
        Vector3Wide::add(&a0a1, &a2a3, center);
    }
}

// ============================================================================
// Contact1OneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact1OneBodyPrestepData {
    //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
    //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
    pub contact0: ConvexContactWide,
    //In a convex manifold, all contacts share the same normal and tangents.
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact1OneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        1
    }
    fn body_count() -> i32 {
        1
    }
}

impl IConvexContactPrestep for Contact1OneBodyPrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 1);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 1);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

pub struct Contact1OneBodyFunctions;

impl Contact1OneBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        prestep: &mut Contact1OneBodyPrestepData,
    ) {
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact0.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut Contact1OneBodyPrestepData,
        accumulated_impulses: &mut Contact1AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        TangentFrictionOneBody::warm_start(
            &x,
            &z,
            &prestep.contact0.offset_a,
            inertia_a,
            &accumulated_impulses.tangent,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &accumulated_impulses.penetration0,
            wsv_a,
        );
        TwistFrictionOneBody::warm_start(
            &prestep.normal,
            inertia_a,
            &accumulated_impulses.twist,
            wsv_a,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact1OneBodyPrestepData,
        accumulated_impulses: &mut Contact1AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        //Note that we solve the penetration constraints before the friction constraints.
        //This makes the friction constraints more authoritative, since they happen last.
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let maximum_tangent_impulse =
            prestep.material_properties.friction_coefficient * accumulated_impulses.penetration0;
        TangentFrictionOneBody::solve(
            &x,
            &z,
            &prestep.contact0.offset_a,
            inertia_a,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
        );
        //If there's only one contact, then the contact patch as determined by contact distance would be zero.
        //That can cause some subtle behavioral issues sometimes, so we approximate lever arm with the contact depth, assuming that the contact surface area will increase as the depth increases.
        let maximum_twist_impulse = prestep.material_properties.friction_coefficient
            * accumulated_impulses.penetration0
            * prestep.contact0.depth.simd_max(Vector::<f32>::splat(0.0));
        TwistFrictionOneBody::solve(
            &prestep.normal,
            inertia_a,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
        );
    }
}

/// Handles the solve iterations of a bunch of 1-contact one body manifold constraints.
pub struct Contact1OneBodyTypeProcessor;
impl Contact1OneBodyTypeProcessor {
    //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
    pub const BATCH_TYPE_ID: i32 = 0;
}

// ============================================================================
// Contact2OneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2OneBodyPrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact2OneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        2
    }
    fn body_count() -> i32 {
        1
    }
}

impl IConvexContactPrestep for Contact2OneBodyPrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 2);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

pub struct Contact2OneBodyFunctions;

impl Contact2OneBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        prestep: &mut Contact2OneBodyPrestepData,
    ) {
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact0.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact1.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut Contact2OneBodyPrestepData,
        accumulated_impulses: &mut Contact2AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_2(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &accumulated_impulses.tangent,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &accumulated_impulses.penetration1,
            wsv_a,
        );
        TwistFrictionOneBody::warm_start(
            &prestep.normal,
            inertia_a,
            &accumulated_impulses.twist,
            wsv_a,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact2OneBodyPrestepData,
        accumulated_impulses: &mut Contact2AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 2.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0 + accumulated_impulses.penetration1);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_2(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    ));
        TwistFrictionOneBody::solve(
            &prestep.normal,
            inertia_a,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
        );
    }
}

/// Handles the solve iterations of a bunch of 2-contact one body manifold constraints.
pub struct Contact2OneBodyTypeProcessor;
impl Contact2OneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 1;
}

// ============================================================================
// Contact3OneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3OneBodyPrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub contact2: ConvexContactWide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact3OneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        3
    }
    fn body_count() -> i32 {
        1
    }
}

impl IConvexContactPrestep for Contact3OneBodyPrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 3);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

pub struct Contact3OneBodyFunctions;

impl Contact3OneBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        prestep: &mut Contact3OneBodyPrestepData,
    ) {
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact0.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact1.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact2.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact2.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut Contact3OneBodyPrestepData,
        accumulated_impulses: &mut Contact3AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_3(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &accumulated_impulses.tangent,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &accumulated_impulses.penetration1,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &accumulated_impulses.penetration2,
            wsv_a,
        );
        TwistFrictionOneBody::warm_start(
            &prestep.normal,
            inertia_a,
            &accumulated_impulses.twist,
            wsv_a,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact3OneBodyPrestepData,
        accumulated_impulses: &mut Contact3AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &prestep.contact2.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration2,
            wsv_a,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 3.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                + accumulated_impulses.penetration1
                + accumulated_impulses.penetration2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_3(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    )
                + accumulated_impulses.penetration2
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact2.offset_a,
                    ));
        TwistFrictionOneBody::solve(
            &prestep.normal,
            inertia_a,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
        );
    }
}

/// Handles the solve iterations of a bunch of 3-contact one body manifold constraints.
pub struct Contact3OneBodyTypeProcessor;
impl Contact3OneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 2;
}

// ============================================================================
// Contact4OneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4OneBodyPrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub contact2: ConvexContactWide,
    pub contact3: ConvexContactWide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact4OneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        4
    }
    fn body_count() -> i32 {
        1
    }
}

impl IConvexContactPrestep for Contact4OneBodyPrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 4);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

pub struct Contact4OneBodyFunctions;

impl Contact4OneBodyFunctions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        prestep: &mut Contact4OneBodyPrestepData,
    ) {
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact0.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact1.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact2.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact2.depth,
        );
        PenetrationLimitOneBody::update_penetration_depth(
            dt,
            &prestep.contact3.offset_a,
            &prestep.normal,
            velocity_a,
            &mut prestep.contact3.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        prestep: &mut Contact4OneBodyPrestepData,
        accumulated_impulses: &mut Contact4AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_4(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact3.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &prestep.contact3.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &accumulated_impulses.tangent,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &accumulated_impulses.penetration1,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &accumulated_impulses.penetration2,
            wsv_a,
        );
        PenetrationLimitOneBody::warm_start(
            inertia_a,
            &prestep.normal,
            &prestep.contact3.offset_a,
            &accumulated_impulses.penetration3,
            wsv_a,
        );
        TwistFrictionOneBody::warm_start(
            &prestep.normal,
            inertia_a,
            &accumulated_impulses.twist,
            wsv_a,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact4OneBodyPrestepData,
        accumulated_impulses: &mut Contact4AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &prestep.contact2.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration2,
            wsv_a,
        );
        PenetrationLimitOneBody::solve(
            inertia_a,
            &prestep.normal,
            &prestep.contact3.offset_a,
            &prestep.contact3.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration3,
            wsv_a,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 4.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                + accumulated_impulses.penetration1
                + accumulated_impulses.penetration2
                + accumulated_impulses.penetration3);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_4(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact3.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &prestep.contact3.depth,
            &mut offset_to_manifold_center_a,
        );
        TangentFrictionOneBody::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            inertia_a,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    )
                + accumulated_impulses.penetration2
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact2.offset_a,
                    )
                + accumulated_impulses.penetration3
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact3.offset_a,
                    ));
        TwistFrictionOneBody::solve(
            &prestep.normal,
            inertia_a,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
        );
    }
}

/// Handles the solve iterations of a bunch of 4-contact one body manifold constraints.
pub struct Contact4OneBodyTypeProcessor;
impl Contact4OneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 3;
}

// ============================================================================
// Contact1 (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact1PrestepData {
    pub contact0: ConvexContactWide,
    pub offset_b: Vector3Wide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact1PrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        1
    }
    fn body_count() -> i32 {
        2
    }
}

impl IConvexContactPrestep for Contact1PrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 1);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 1);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

impl ITwoBodyConvexContactPrestep for Contact1PrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

pub struct Contact1Functions;

impl Contact1Functions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        prestep: &mut Contact1PrestepData,
    ) {
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact0.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut Contact1PrestepData,
        accumulated_impulses: &mut Contact1AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::warm_start(
            &x,
            &z,
            &prestep.contact0.offset_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b,
            &accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        TwistFriction::warm_start(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact1PrestepData,
        accumulated_impulses: &mut Contact1AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let maximum_tangent_impulse =
            prestep.material_properties.friction_coefficient * accumulated_impulses.penetration0;
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::solve(
            &x,
            &z,
            &prestep.contact0.offset_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        //If there's only one contact, then the contact patch as determined by contact distance would be zero.
        let maximum_twist_impulse = prestep.material_properties.friction_coefficient
            * accumulated_impulses.penetration0
            * prestep.contact0.depth.simd_max(Vector::<f32>::splat(0.0));
        TwistFriction::solve(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }
}

/// Handles the solve iterations of a bunch of 1-contact two body manifold constraints.
pub struct Contact1TypeProcessor;
impl Contact1TypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 4;
}

// ============================================================================
// Contact2 (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2PrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub offset_b: Vector3Wide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact2PrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        2
    }
    fn body_count() -> i32 {
        2
    }
}

impl IConvexContactPrestep for Contact2PrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 2);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

impl ITwoBodyConvexContactPrestep for Contact2PrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

pub struct Contact2Functions;

impl Contact2Functions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        prestep: &mut Contact2PrestepData,
    ) {
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact0.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact1.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut Contact2PrestepData,
        accumulated_impulses: &mut Contact2AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_2(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        TwistFriction::warm_start(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact2PrestepData,
        accumulated_impulses: &mut Contact2AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 2.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0 + accumulated_impulses.penetration1);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_2(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    ));
        TwistFriction::solve(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }
}

/// Handles the solve iterations of a bunch of 2-contact two body manifold constraints.
pub struct Contact2TypeProcessor;
impl Contact2TypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 5;
}

// ============================================================================
// Contact3 (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3PrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub contact2: ConvexContactWide,
    pub offset_b: Vector3Wide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact3PrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        3
    }
    fn body_count() -> i32 {
        2
    }
}

impl IConvexContactPrestep for Contact3PrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 3);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

impl ITwoBodyConvexContactPrestep for Contact3PrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

pub struct Contact3Functions;

impl Contact3Functions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        prestep: &mut Contact3PrestepData,
    ) {
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact0.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact1.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact2.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact2.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut Contact3PrestepData,
        accumulated_impulses: &mut Contact3AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_3(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b2 = prestep.contact2.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &contact_offset_b2,
            &accumulated_impulses.penetration2,
            wsv_a,
            wsv_b,
        );
        TwistFriction::warm_start(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact3PrestepData,
        accumulated_impulses: &mut Contact3AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b2 = prestep.contact2.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &contact_offset_b2,
            &prestep.contact2.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration2,
            wsv_a,
            wsv_b,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 3.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                + accumulated_impulses.penetration1
                + accumulated_impulses.penetration2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_3(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    )
                + accumulated_impulses.penetration2
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact2.offset_a,
                    ));
        TwistFriction::solve(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }
}

/// Handles the solve iterations of a bunch of 3-contact two body manifold constraints.
pub struct Contact3TypeProcessor;
impl Contact3TypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 6;
}

// ============================================================================
// Contact4 (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4PrestepData {
    pub contact0: ConvexContactWide,
    pub contact1: ConvexContactWide,
    pub contact2: ConvexContactWide,
    pub contact3: ConvexContactWide,
    pub offset_b: Vector3Wide,
    pub normal: Vector3Wide,
    pub material_properties: MaterialPropertiesWide,
}

impl IContactPrestep for Contact4PrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        4
    }
    fn body_count() -> i32 {
        2
    }
}

impl IConvexContactPrestep for Contact4PrestepData {
    fn get_normal(&self) -> &Vector3Wide {
        &self.normal
    }
    fn get_normal_mut(&mut self) -> &mut Vector3Wide {
        &mut self.normal
    }
    fn get_contact(&self, index: usize) -> &ConvexContactWide {
        debug_assert!(index < 4);
        unsafe { &*(&self.contact0 as *const ConvexContactWide).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.contact0 as *mut ConvexContactWide).add(index) }
    }
}

impl ITwoBodyConvexContactPrestep for Contact4PrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

pub struct Contact4Functions;

impl Contact4Functions {
    pub const REQUIRES_INCREMENTAL_SUBSTEP_UPDATES: bool = true;

    #[inline(always)]
    pub fn incrementally_update_for_substep(
        dt: &Vector<f32>,
        velocity_a: &BodyVelocityWide,
        velocity_b: &BodyVelocityWide,
        prestep: &mut Contact4PrestepData,
    ) {
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact0.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact0.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact1.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact1.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact2.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact2.depth,
        );
        PenetrationLimit::update_penetration_depth(
            dt,
            &prestep.contact3.offset_a,
            &prestep.offset_b,
            &prestep.normal,
            velocity_a,
            velocity_b,
            &mut prestep.contact3.depth,
        );
    }

    #[inline(always)]
    pub fn warm_start(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        prestep: &mut Contact4PrestepData,
        accumulated_impulses: &mut Contact4AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_4(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact3.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &prestep.contact3.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::warm_start(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b2 = prestep.contact2.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &contact_offset_b2,
            &accumulated_impulses.penetration2,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b3 = prestep.contact3.offset_a - prestep.offset_b;
        PenetrationLimit::warm_start(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact3.offset_a,
            &contact_offset_b3,
            &accumulated_impulses.penetration3,
            wsv_a,
            wsv_b,
        );
        TwistFriction::warm_start(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        _position_a: &Vector3Wide,
        _orientation_a: &QuaternionWide,
        inertia_a: &BodyInertiaWide,
        _position_b: &Vector3Wide,
        _orientation_b: &QuaternionWide,
        inertia_b: &BodyInertiaWide,
        dt: f32,
        inverse_dt: f32,
        prestep: &mut Contact4PrestepData,
        accumulated_impulses: &mut Contact4AccumulatedImpulses,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let (
            mut position_error_to_velocity,
            mut effective_mass_cfm_scale,
            mut softness_impulse_scale,
        ) = Default::default();
        SpringSettingsWide::compute_springiness(
            &prestep.material_properties.spring_settings,
            dt,
            &mut position_error_to_velocity,
            &mut effective_mass_cfm_scale,
            &mut softness_impulse_scale,
        );
        let inverse_dt_wide = Vector::<f32>::splat(inverse_dt);
        let contact_offset_b0 = prestep.contact0.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact0.offset_a,
            &contact_offset_b0,
            &prestep.contact0.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration0,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b1 = prestep.contact1.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact1.offset_a,
            &contact_offset_b1,
            &prestep.contact1.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration1,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b2 = prestep.contact2.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact2.offset_a,
            &contact_offset_b2,
            &prestep.contact2.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration2,
            wsv_a,
            wsv_b,
        );
        let contact_offset_b3 = prestep.contact3.offset_a - prestep.offset_b;
        PenetrationLimit::solve(
            inertia_a,
            inertia_b,
            &prestep.normal,
            &prestep.contact3.offset_a,
            &contact_offset_b3,
            &prestep.contact3.depth,
            &position_error_to_velocity,
            &effective_mass_cfm_scale,
            &prestep.material_properties.maximum_recovery_velocity,
            &inverse_dt_wide,
            &softness_impulse_scale,
            &mut accumulated_impulses.penetration3,
            wsv_a,
            wsv_b,
        );
        let (x, z) = out_unsafe!(Helpers::build_orthonormal_basis(&prestep.normal), 2);
        let premultiplied_friction_coefficient =
            Vector::<f32>::splat(1.0 / 4.0) * prestep.material_properties.friction_coefficient;
        let maximum_tangent_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                + accumulated_impulses.penetration1
                + accumulated_impulses.penetration2
                + accumulated_impulses.penetration3);
        let mut offset_to_manifold_center_a = Vector3Wide::default();
        FrictionHelpers::compute_friction_center_4(
            &prestep.contact0.offset_a,
            &prestep.contact1.offset_a,
            &prestep.contact2.offset_a,
            &prestep.contact3.offset_a,
            &prestep.contact0.depth,
            &prestep.contact1.depth,
            &prestep.contact2.depth,
            &prestep.contact3.depth,
            &mut offset_to_manifold_center_a,
        );
        let mut offset_to_manifold_center_b = Vector3Wide::default();
        Vector3Wide::subtract(
            &offset_to_manifold_center_a,
            &prestep.offset_b,
            &mut offset_to_manifold_center_b,
        );
        TangentFriction::solve(
            &x,
            &z,
            &offset_to_manifold_center_a,
            &offset_to_manifold_center_b,
            inertia_a,
            inertia_b,
            &maximum_tangent_impulse,
            &mut accumulated_impulses.tangent,
            wsv_a,
            wsv_b,
        );
        let maximum_twist_impulse = premultiplied_friction_coefficient
            * (accumulated_impulses.penetration0
                * Vector3Wide::distance(&offset_to_manifold_center_a, &prestep.contact0.offset_a)
                + accumulated_impulses.penetration1
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact1.offset_a,
                    )
                + accumulated_impulses.penetration2
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact2.offset_a,
                    )
                + accumulated_impulses.penetration3
                    * Vector3Wide::distance(
                        &offset_to_manifold_center_a,
                        &prestep.contact3.offset_a,
                    ));
        TwistFriction::solve(
            &prestep.normal,
            inertia_a,
            inertia_b,
            &maximum_twist_impulse,
            &mut accumulated_impulses.twist,
            wsv_a,
            wsv_b,
        );
    }
}

/// Handles the solve iterations of a bunch of 4-contact two body manifold constraints.
pub struct Contact4TypeProcessor;
impl Contact4TypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 7;
}
