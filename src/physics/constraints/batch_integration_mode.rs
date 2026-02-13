// Translated from BepuPhysics/Constraints/IBatchIntegrationMode.cs

/// Marks a type as determining the integration mode for a solver batch.
pub trait IBatchIntegrationMode {}

/// The batch was determined to have only constraints with integration responsibilities,
/// so there's no need to check.
pub struct BatchShouldAlwaysIntegrate;
impl IBatchIntegrationMode for BatchShouldAlwaysIntegrate {}

/// The batch was determined to have no constraints with integration responsibilities,
/// so there's no need to check.
pub struct BatchShouldNeverIntegrate;
impl IBatchIntegrationMode for BatchShouldNeverIntegrate {}

/// The batch was determined to have some constraints with integration responsibilities.
pub struct BatchShouldConditionallyIntegrate;
impl IBatchIntegrationMode for BatchShouldConditionallyIntegrate {}

/// Marks a type as determining whether pose integration should be performed on bodies
/// within the constraint batch.
pub trait IBatchPoseIntegrationAllowed {}

/// Marks a batch as integrating poses for any bodies with integration responsibility
/// within the constraint batch. Constraints which need to be updated in response to pose
/// integration will also have their UpdateForNewPose function called.
pub struct AllowPoseIntegration;
impl IBatchPoseIntegrationAllowed for AllowPoseIntegration {}

/// Marks a batch as not integrating poses for any bodies within the constraint batch.
pub struct DisallowPoseIntegration;
impl IBatchPoseIntegrationAllowed for DisallowPoseIntegration {}

/// Runtime enum equivalent of the compile-time IBatchIntegrationMode marker types.
/// Used for dynamic dispatch through trait objects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum BatchIntegrationMode {
    /// Batch 0: all bodies first-seen here, always integrate.
    Always = 0,
    /// No constraints in this type batch have integration responsibilities.
    Never = 1,
    /// Some constraints have integration responsibilities; check per-bundle flags.
    Conditional = 2,
}

/// Result of checking whether a bundle should integrate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum BundleIntegrationMode {
    None = 0,
    Partial = 1,
    All = 2,
}
