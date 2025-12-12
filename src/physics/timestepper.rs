// Translated from BepuPhysics/ITimestepper.cs

/// Type alias for timestep stage handler callbacks.
/// Corresponds to `delegate void TimestepperStageHandler(float dt, IThreadDispatcher threadDispatcher)`.
///
/// In practice, these callbacks are invoked during the timestep to allow user code to run
/// at specific points (e.g., before/after collision detection).
///
/// The `thread_dispatcher` parameter is an optional raw pointer to the thread dispatcher,
/// which may be null if single-threaded.
pub type TimestepperStageHandler = Box<dyn Fn(f32, *mut u8)>;

/// Defines a type capable of updating the simulation state for a given elapsed time.
///
/// Corresponds to `ITimestepper` in C#.
///
/// Note: In the C# version, `ITimestepper` has event fields (`BeforeCollisionDetection`, `CollisionsDetected`)
/// and a `Timestep` method that takes a `Simulation` parameter. Since `Simulation` is not yet translated,
/// this trait uses opaque pointers for now.
pub trait ITimestepper {
    /// Performs one timestep of the given length.
    ///
    /// # Parameters
    /// * `simulation` - Pointer to the simulation to be stepped forward in time.
    /// * `dt` - Duration of the time step.
    /// * `thread_dispatcher` - Optional pointer to the thread dispatcher for parallel execution.
    ///
    /// # Safety
    /// The caller must ensure that `simulation` and `thread_dispatcher` (if non-null) are valid pointers.
    unsafe fn timestep(
        &mut self,
        simulation: *mut u8,
        dt: f32,
        thread_dispatcher: *mut u8,
    );
}
