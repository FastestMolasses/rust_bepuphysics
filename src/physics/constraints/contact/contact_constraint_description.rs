// Translated from BepuPhysics/Constraints/Contact/IContactConstraintDescription.cs

use crate::physics::constraints::spring_settings::SpringSettings;
use glam::Vec3;

/// Per-contact data for convex constraint descriptions (narrow/scalar form).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ConstraintContactData {
    pub offset_a: Vec3,
    pub penetration_depth: f32,
}

/// Per-contact data for nonconvex constraint descriptions (narrow/scalar form).
/// Each contact has its own normal.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NonconvexConstraintContactData {
    pub offset_a: Vec3,
    pub normal: Vec3,
    pub penetration_depth: f32,
}

/// Common properties for nonconvex two-body manifold constraint descriptions.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NonconvexTwoBodyManifoldConstraintProperties {
    pub offset_b: Vec3,
    //Note that the positioning of the friction coefficient, spring settings, and maximum recovery velocity are used by the UnsafeManifoldViewer. Careful about moving these.
    pub friction_coefficient: f32,
    pub spring_settings: SpringSettings,
    pub maximum_recovery_velocity: f32,
}

/// Common properties for nonconvex one-body manifold constraint descriptions.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NonconvexOneBodyManifoldConstraintProperties {
    //Note that the positioning of the friction coefficient, spring settings, and maximum recovery velocity are used by the UnsafeManifoldViewer. Careful about moving these.
    pub friction_coefficient: f32,
    pub spring_settings: SpringSettings,
    pub maximum_recovery_velocity: f32,
}

/// Trait for convex one-body contact constraint descriptions.
pub trait IConvexOneBodyContactConstraintDescription {
    fn get_first_contact(&self) -> &ConstraintContactData;
    fn get_first_contact_mut(&mut self) -> &mut ConstraintContactData;
}

/// Trait for convex two-body contact constraint descriptions.
pub trait IConvexTwoBodyContactConstraintDescription {
    fn get_first_contact(&self) -> &ConstraintContactData;
    fn get_first_contact_mut(&mut self) -> &mut ConstraintContactData;
}

/// Trait for nonconvex one-body contact constraint descriptions.
pub trait INonconvexOneBodyContactConstraintDescription {
    fn contact_count() -> i32;
    fn get_common_properties(&self) -> &NonconvexOneBodyManifoldConstraintProperties;
    fn get_common_properties_mut(&mut self) -> &mut NonconvexOneBodyManifoldConstraintProperties;
    fn get_first_contact(&self) -> &NonconvexConstraintContactData;
    fn get_first_contact_mut(&mut self) -> &mut NonconvexConstraintContactData;
}

/// Trait for nonconvex two-body contact constraint descriptions.
pub trait INonconvexTwoBodyContactConstraintDescription {
    fn contact_count() -> i32;
    fn get_common_properties(&self) -> &NonconvexTwoBodyManifoldConstraintProperties;
    fn get_common_properties_mut(&mut self) -> &mut NonconvexTwoBodyManifoldConstraintProperties;
    fn get_first_contact(&self) -> &NonconvexConstraintContactData;
    fn get_first_contact_mut(&mut self) -> &mut NonconvexConstraintContactData;
}
