// Translated from BepuPhysics/DefaultTimestepper.cs
//
// Updates the simulation in the order of:
// sleeper -> predict body bounding boxes -> collision detection -> substepping solve ->
// data structure optimization.
// Each substep of the solve simulates and integrates a sub-timestep of length dt/substepCount.

use crate::physics::timestepper::{ITimestepper, TimestepperStageHandler};

/// Default timestepper that executes:
/// Sleep -> PredictBBs -> CollisionDetection -> Solve -> OptimizeDataStructures.
pub struct DefaultTimestepper {
    /// Fires after the sleeper completes and before bodies are integrated.
    pub slept: Option<TimestepperStageHandler>,
    /// Fires after bodies have their bounding boxes updated for predicted motion
    /// and before collision detection.
    pub before_collision_detection: Option<TimestepperStageHandler>,
    /// Fires after all collisions have been identified but before the substep loop begins.
    pub collisions_detected: Option<TimestepperStageHandler>,
    /// Fires after the solver executes and before the final integration step.
    pub constraints_solved: Option<TimestepperStageHandler>,
}

impl Default for DefaultTimestepper {
    fn default() -> Self {
        Self {
            slept: None,
            before_collision_detection: None,
            collisions_detected: None,
            constraints_solved: None,
        }
    }
}

impl ITimestepper for DefaultTimestepper {
    unsafe fn timestep(
        &mut self,
        simulation: *mut u8,
        dt: f32,
        thread_dispatcher: *mut u8,
    ) {
        // TODO: Call simulation methods once Simulation is translated:
        // (*simulation).sleep(thread_dispatcher);
        if let Some(ref handler) = self.slept {
            handler(dt, thread_dispatcher);
        }

        // (*simulation).predict_bounding_boxes(dt, thread_dispatcher);
        if let Some(ref handler) = self.before_collision_detection {
            handler(dt, thread_dispatcher);
        }

        // (*simulation).collision_detection(dt, thread_dispatcher);
        if let Some(ref handler) = self.collisions_detected {
            handler(dt, thread_dispatcher);
        }

        // (*simulation).solve(dt, thread_dispatcher);
        if let Some(ref handler) = self.constraints_solved {
            handler(dt, thread_dispatcher);
        }

        // (*simulation).incrementally_optimize_data_structures(thread_dispatcher);
    }
}
