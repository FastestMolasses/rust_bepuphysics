// Translated from BepuPhysics/DefaultTypes.cs

use crate::physics::solver::Solver;
use crate::physics::collision_detection::narrow_phase::NarrowPhase;

// Re-import all the constraint description types.
use crate::physics::constraints::ball_socket::BallSocket;
use crate::physics::constraints::angular_hinge::AngularHinge;
use crate::physics::constraints::angular_swivel_hinge::AngularSwivelHinge;
use crate::physics::constraints::swing_limit::SwingLimit;
use crate::physics::constraints::twist_servo::TwistServo;
use crate::physics::constraints::twist_limit::TwistLimit;
use crate::physics::constraints::twist_motor::TwistMotor;
use crate::physics::constraints::angular_servo::AngularServo;
use crate::physics::constraints::angular_motor::AngularMotor;
use crate::physics::constraints::weld::Weld;
use crate::physics::constraints::distance_servo::DistanceServo;
use crate::physics::constraints::distance_limit::DistanceLimit;
use crate::physics::constraints::center_distance_constraint::CenterDistanceConstraint;
use crate::physics::constraints::point_on_line_servo::PointOnLineServo;
use crate::physics::constraints::linear_axis_servo::LinearAxisServo;
use crate::physics::constraints::linear_axis_motor::LinearAxisMotor;
use crate::physics::constraints::linear_axis_limit::LinearAxisLimit;
use crate::physics::constraints::angular_axis_motor::AngularAxisMotor;
use crate::physics::constraints::one_body_angular_servo::OneBodyAngularServo;
use crate::physics::constraints::one_body_angular_motor::OneBodyAngularMotor;
use crate::physics::constraints::one_body_linear_servo::OneBodyLinearServo;
use crate::physics::constraints::one_body_linear_motor::OneBodyLinearMotor;
use crate::physics::constraints::swivel_hinge::SwivelHinge;
use crate::physics::constraints::hinge::Hinge;
use crate::physics::constraints::ball_socket_motor::BallSocketMotor;
use crate::physics::constraints::ball_socket_servo::BallSocketServo;
use crate::physics::constraints::angular_axis_gear_motor::AngularAxisGearMotor;
use crate::physics::constraints::center_distance_limit::CenterDistanceLimit;
use crate::physics::constraints::volume_constraint::VolumeConstraint;
use crate::physics::constraints::area_constraint::AreaConstraint;

// Contact constraint description types (for solver registration).
use crate::physics::constraints::contact::contact_convex_descriptions::{
    Contact1OneBody, Contact2OneBody, Contact3OneBody, Contact4OneBody,
    Contact1TwoBody, Contact2TwoBody, Contact3TwoBody, Contact4TwoBody,
};
use crate::physics::constraints::contact::contact_nonconvex_descriptions::{
    Contact2NonconvexOneBody, Contact3NonconvexOneBody, Contact4NonconvexOneBody,
    Contact2Nonconvex, Contact3Nonconvex, Contact4Nonconvex,
};

// Contact constraint accessor types (for narrow phase registration).
use crate::physics::collision_detection::contact_constraint_accessor::{
    Contact1OneBodyAccessor, Contact2OneBodyAccessor, Contact3OneBodyAccessor, Contact4OneBodyAccessor,
    Contact1TwoBodyAccessor, Contact2TwoBodyAccessor, Contact3TwoBodyAccessor, Contact4TwoBodyAccessor,
    Contact2NonconvexOneBodyAccessor, Contact3NonconvexOneBodyAccessor, Contact4NonconvexOneBodyAccessor,
    Contact2NonconvexTwoBodyAccessor, Contact3NonconvexTwoBodyAccessor, Contact4NonconvexTwoBodyAccessor,
};

/// Helper to register the default types within a simulation instance.
pub struct DefaultTypes;

impl DefaultTypes {
    /// Registers the set of constraints that are packaged in the engine.
    pub fn register_defaults(solver: &mut Solver, narrow_phase: &mut NarrowPhase) {
        solver.register::<BallSocket>();
        solver.register::<AngularHinge>();
        solver.register::<AngularSwivelHinge>();
        solver.register::<SwingLimit>();
        solver.register::<TwistServo>();
        solver.register::<TwistLimit>();
        solver.register::<TwistMotor>();
        solver.register::<AngularServo>();
        solver.register::<AngularMotor>();
        solver.register::<Weld>();
        solver.register::<VolumeConstraint>();
        solver.register::<DistanceServo>();
        solver.register::<DistanceLimit>();
        solver.register::<CenterDistanceConstraint>();
        solver.register::<AreaConstraint>();
        solver.register::<PointOnLineServo>();
        solver.register::<LinearAxisServo>();
        solver.register::<LinearAxisMotor>();
        solver.register::<LinearAxisLimit>();
        solver.register::<AngularAxisMotor>();
        solver.register::<OneBodyAngularServo>();
        solver.register::<OneBodyAngularMotor>();
        solver.register::<OneBodyLinearServo>();
        solver.register::<OneBodyLinearMotor>();
        solver.register::<SwivelHinge>();
        solver.register::<Hinge>();
        solver.register::<BallSocketMotor>();
        solver.register::<BallSocketServo>();
        solver.register::<AngularAxisGearMotor>();
        solver.register::<CenterDistanceLimit>();

        // Contact constraint types.
        solver.register::<Contact1OneBody>();
        solver.register::<Contact2OneBody>();
        solver.register::<Contact3OneBody>();
        solver.register::<Contact4OneBody>();
        solver.register::<Contact1TwoBody>();
        solver.register::<Contact2TwoBody>();
        solver.register::<Contact3TwoBody>();
        solver.register::<Contact4TwoBody>();
        solver.register::<Contact2NonconvexOneBody>();
        solver.register::<Contact3NonconvexOneBody>();
        solver.register::<Contact4NonconvexOneBody>();
        solver.register::<Contact2Nonconvex>();
        solver.register::<Contact3Nonconvex>();
        solver.register::<Contact4Nonconvex>();

        // Contact constraint accessors — wire NarrowPhase to dispatch each
        // contact manifold type through the correct type-erased path.
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4NonconvexTwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3NonconvexTwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2NonconvexTwoBodyAccessor));

        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4NonconvexOneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3NonconvexOneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2NonconvexOneBodyAccessor));

        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact1TwoBodyAccessor));

        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact1OneBodyAccessor));
    }

    // TODO: CreateDefaultCollisionTaskRegistry
    // TODO: CreateDefaultSweepTaskRegistry
}