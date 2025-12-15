// Translated from BepuPhysics/DefaultTypes.cs

use crate::physics::solver::Solver;

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

/// Helper to register the default types within a simulation instance.
pub struct DefaultTypes;

impl DefaultTypes {
    /// Registers the set of constraints that are packaged in the engine.
    ///
    /// Note: Contact constraints (Contact1–Contact4, Contact*OneBody, Contact*Nonconvex) and
    /// NarrowPhase constraint accessors are not yet registered because the collision detection
    /// module has not been translated. VolumeConstraint (4-body) and AreaConstraint (3-body) are
    /// also deferred until their respective type processor impls are added.
    pub fn register_defaults(solver: &mut Solver, _narrow_phase: &mut ()) {
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
        // solver.register::<VolumeConstraint>();  // TODO: 4-body type processor
        solver.register::<DistanceServo>();
        solver.register::<DistanceLimit>();
        solver.register::<CenterDistanceConstraint>();
        // solver.register::<AreaConstraint>();     // TODO: 3-body type processor
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

        // Contact constraint registrations (collision detection module not yet translated):
        // solver.register::<Contact1OneBody>();
        // solver.register::<Contact2OneBody>();
        // solver.register::<Contact3OneBody>();
        // solver.register::<Contact4OneBody>();
        // solver.register::<Contact1>();
        // solver.register::<Contact2>();
        // solver.register::<Contact3>();
        // solver.register::<Contact4>();
        // solver.register::<Contact2NonconvexOneBody>();
        // solver.register::<Contact3NonconvexOneBody>();
        // solver.register::<Contact4NonconvexOneBody>();
        // solver.register::<Contact2Nonconvex>();
        // solver.register::<Contact3Nonconvex>();
        // solver.register::<Contact4Nonconvex>();
        //
        // narrow_phase.register_contact_constraint_accessor(...)
    }

    // TODO: CreateDefaultCollisionTaskRegistry
    // TODO: CreateDefaultSweepTaskRegistry
}

