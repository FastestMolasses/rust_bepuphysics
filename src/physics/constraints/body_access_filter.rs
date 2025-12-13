// Translated from BepuPhysics/Constraints/IBodyAccessFilter.cs

/// Constrains which body properties should be accessed in a body during constraint data
/// gathering/scattering.
pub trait IBodyAccessFilter {
    /// Gets whether position is loaded by the constraint.
    fn gather_position() -> bool;
    /// Gets whether orientation is loaded by the constraint.
    fn gather_orientation() -> bool;
    /// Gets whether body mass is loaded by this constraint.
    fn gather_mass() -> bool;
    /// Gets whether body inertia tensor is loaded by this constraint.
    fn gather_inertia_tensor() -> bool;
    /// Gets whether to load or store body linear velocity in this constraint.
    fn access_linear_velocity() -> bool;
    /// Gets whether to load or store body angular velocity in this constraint.
    fn access_angular_velocity() -> bool;
}

/// Marks all body properties as necessary for gather/scatter.
pub struct AccessAll;
impl IBodyAccessFilter for AccessAll {
    fn gather_position() -> bool { true }
    fn gather_orientation() -> bool { true }
    fn gather_mass() -> bool { true }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

/// Used for kinematic integration; the inertias are known ahead of time and there's no reason to
/// gather them.
pub struct AccessNoInertia;
impl IBodyAccessFilter for AccessNoInertia {
    fn gather_position() -> bool { true }
    fn gather_orientation() -> bool { true }
    fn gather_mass() -> bool { false }
    fn gather_inertia_tensor() -> bool { false }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessNoPose;
impl IBodyAccessFilter for AccessNoPose {
    fn gather_position() -> bool { false }
    fn gather_orientation() -> bool { false }
    fn gather_mass() -> bool { true }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessNoPosition;
impl IBodyAccessFilter for AccessNoPosition {
    fn gather_position() -> bool { false }
    fn gather_orientation() -> bool { true }
    fn gather_mass() -> bool { true }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessNoOrientation;
impl IBodyAccessFilter for AccessNoOrientation {
    fn gather_position() -> bool { true }
    fn gather_orientation() -> bool { false }
    fn gather_mass() -> bool { true }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessOnlyVelocity;
impl IBodyAccessFilter for AccessOnlyVelocity {
    fn gather_position() -> bool { false }
    fn gather_orientation() -> bool { false }
    fn gather_mass() -> bool { false }
    fn gather_inertia_tensor() -> bool { false }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessOnlyAngular;
impl IBodyAccessFilter for AccessOnlyAngular {
    fn gather_position() -> bool { false }
    fn gather_orientation() -> bool { true }
    fn gather_mass() -> bool { false }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { false }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessOnlyAngularWithoutPose;
impl IBodyAccessFilter for AccessOnlyAngularWithoutPose {
    fn gather_position() -> bool { false }
    fn gather_orientation() -> bool { false }
    fn gather_mass() -> bool { false }
    fn gather_inertia_tensor() -> bool { true }
    fn access_linear_velocity() -> bool { false }
    fn access_angular_velocity() -> bool { true }
}

pub struct AccessOnlyLinear;
impl IBodyAccessFilter for AccessOnlyLinear {
    fn gather_position() -> bool { true }
    fn gather_orientation() -> bool { false }
    fn gather_mass() -> bool { true }
    fn gather_inertia_tensor() -> bool { false }
    fn access_linear_velocity() -> bool { true }
    fn access_angular_velocity() -> bool { false }
}
