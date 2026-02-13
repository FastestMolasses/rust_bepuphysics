// Translated from BepuPhysics/ITimestepper.cs

use crate::physics::simulation::Simulation;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

/// Type alias for timestep stage handler callbacks.
/// Corresponds to `delegate void TimestepperStageHandler(float dt, IThreadDispatcher threadDispatcher)`.
pub type TimestepperStageHandler = Box<dyn Fn(f32, Option<&dyn IThreadDispatcher>)>;

/// Defines a type capable of updating the simulation state for a given elapsed time.
/// Corresponds to `ITimestepper` in C#.
pub trait ITimestepper {
    /// Performs one timestep of the given length.
    ///
    /// # Safety
    /// The caller must ensure that `simulation` is a valid pointer.
    unsafe fn timestep(
        &mut self,
        simulation: *mut Simulation,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    );

    /// Gets a mutable reference to the before-collision-detection event handler.
    /// Corresponds to C# `event TimestepperStageHandler BeforeCollisionDetection`.
    fn before_collision_detection_mut(&mut self) -> &mut Option<TimestepperStageHandler>;

    /// Gets a mutable reference to the collisions-detected event handler.
    /// Corresponds to C# `event TimestepperStageHandler CollisionsDetected`.
    fn collisions_detected_mut(&mut self) -> &mut Option<TimestepperStageHandler>;
}
