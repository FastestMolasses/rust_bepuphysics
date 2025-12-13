// Translated from BepuPhysics/Constraints/Contact/ISolverContactDataExtractor.cs

use crate::physics::constraints::contact::contact_convex_common::*;
use crate::physics::constraints::contact::contact_nonconvex_common::*;
use crate::physics::handles::BodyHandle;

/// Callbacks for direct references to the solver's contact constraint data.
pub trait ISolverContactDataExtractor {
    /// Provides a reference to a convex one body contact constraint.
    /// Constraint data is in the first lane of the direct reference (slot 0 of vectors).
    fn convex_one_body<TPrestep: IConvexContactPrestep, TAccumulatedImpulses: IConvexContactAccumulatedImpulses>(
        &mut self,
        body_handle: BodyHandle,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a convex two body contact constraint.
    /// Constraint data is in the first lane of the direct reference (slot 0 of vectors).
    fn convex_two_body<TPrestep: ITwoBodyConvexContactPrestep, TAccumulatedImpulses: IConvexContactAccumulatedImpulses>(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a nonconvex one body contact constraint.
    /// Constraint data is in the first lane of the direct reference (slot 0 of vectors).
    fn nonconvex_one_body<TPrestep: INonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        &mut self,
        body_handle: BodyHandle,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a nonconvex two body contact constraint.
    /// Constraint data is in the first lane of the direct reference (slot 0 of vectors).
    fn nonconvex_two_body<TPrestep: ITwoBodyNonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        &mut self,
        body_handle_a: BodyHandle,
        body_handle_b: BodyHandle,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );
}

/// Callbacks for direct references to the solver's contact constraint data.
/// Includes only prestep and impulse data (no body references).
pub trait ISolverContactPrestepAndImpulsesExtractor {
    /// Provides a reference to a convex one body contact constraint.
    fn convex_one_body<TPrestep: IConvexContactPrestep, TAccumulatedImpulses: IConvexContactAccumulatedImpulses>(
        &mut self,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a convex two body contact constraint.
    fn convex_two_body<TPrestep: ITwoBodyConvexContactPrestep, TAccumulatedImpulses: IConvexContactAccumulatedImpulses>(
        &mut self,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a nonconvex one body contact constraint.
    fn nonconvex_one_body<TPrestep: INonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        &mut self,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );

    /// Provides a reference to a nonconvex two body contact constraint.
    fn nonconvex_two_body<TPrestep: ITwoBodyNonconvexContactPrestep, TAccumulatedImpulses: INonconvexContactAccumulatedImpulses>(
        &mut self,
        prestep: &mut TPrestep,
        impulses: &mut TAccumulatedImpulses,
    );
}
