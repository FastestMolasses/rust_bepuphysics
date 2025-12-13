// Translated from BepuPhysics/Constraints/Contact/ContactConvexCommon.cs

use crate::physics::constraints::spring_settings::SpringSettingsWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ConvexContactWide {
    pub offset_a: Vector3Wide,
    pub depth: Vector<f32>,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MaterialPropertiesWide {
    pub friction_coefficient: Vector<f32>,
    pub spring_settings: SpringSettingsWide,
    pub maximum_recovery_velocity: Vector<f32>,
}

pub trait IContactPrestep {
    fn get_material_properties(&self) -> &MaterialPropertiesWide;
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide;
    fn contact_count() -> i32;
    fn body_count() -> i32;
}

pub trait IConvexContactPrestep: IContactPrestep {
    fn get_normal(&self) -> &Vector3Wide;
    fn get_normal_mut(&mut self) -> &mut Vector3Wide;
    fn get_contact(&self, index: usize) -> &ConvexContactWide;
    fn get_contact_mut(&mut self, index: usize) -> &mut ConvexContactWide;
}

pub trait ITwoBodyConvexContactPrestep: IConvexContactPrestep {
    fn get_offset_b(&self) -> &Vector3Wide;
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide;
}

pub trait IContactAccumulatedImpulses {
    fn contact_count() -> i32;
}

pub trait IConvexContactAccumulatedImpulses: IContactAccumulatedImpulses {
    fn get_tangent_friction(&self) -> &Vector2Wide;
    fn get_tangent_friction_mut(&mut self) -> &mut Vector2Wide;
    fn get_twist_friction(&self) -> &Vector<f32>;
    fn get_twist_friction_mut(&mut self) -> &mut Vector<f32>;
    fn get_penetration_impulse_for_contact(&self, index: usize) -> &Vector<f32>;
    fn get_penetration_impulse_for_contact_mut(&mut self, index: usize) -> &mut Vector<f32>;
}
