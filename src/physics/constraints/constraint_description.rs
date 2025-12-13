// Translated from BepuPhysics/Constraints/IConstraintDescription.cs
//
// In C#, these are generic interfaces with static abstract members. In Rust, we model them as
// traits. The static abstract members (BuildDescription, ConstraintTypeId, TypeProcessorType,
// CreateTypeProcessor) become associated functions/constants.

use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::constraints::type_processor::TypeProcessor;

/// Marks a type as a description of a constraint associated with a particular batch.
///
/// Note that one batch may have multiple description types associated with it, each one
/// potentially offering a different subset of properties or translation logic.
pub trait IConstraintDescription: Copy {
    /// Changes the batch-held memory at a given location to match the given description.
    fn apply_description(&self, batch: &mut TypeBatch, bundle_index: i32, inner_index: i32);

    /// Creates a description from the batch-held memory at a given location.
    fn build_description(batch: &TypeBatch, bundle_index: i32, inner_index: i32) -> Self;

    /// Gets the type id of the constraint that this is a description of.
    fn constraint_type_id() -> i32;

    /// Creates a type processor for this constraint type.
    fn create_type_processor() -> TypeProcessor;
}

/// Marks a type as a one body constraint description.
pub trait IOneBodyConstraintDescription: IConstraintDescription {}

/// Marks a type as a two body constraint description.
pub trait ITwoBodyConstraintDescription: IConstraintDescription {}

/// Marks a type as a three body constraint description.
pub trait IThreeBodyConstraintDescription: IConstraintDescription {}

/// Marks a type as a four body constraint description.
pub trait IFourBodyConstraintDescription: IConstraintDescription {}
