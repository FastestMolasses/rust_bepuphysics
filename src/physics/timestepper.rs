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
}
