// Translated from BepuPhysics/SimulationProfiler.cs

#[cfg(feature = "profile")]
use std::collections::HashMap;
#[cfg(feature = "profile")]
use std::time::Instant;

/// Stores profiling information for the previous simulation execution.
///
/// Profiling is only active when the `profile` feature is enabled.
/// When inactive, all methods are no-ops.
pub struct SimulationProfiler {
    #[cfg(feature = "profile")]
    stages: HashMap<&'static str, f64>,
    #[cfg(feature = "profile")]
    start_timestamps: HashMap<&'static str, Instant>,
}

impl SimulationProfiler {
    /// Creates a new simulation profiler.
    pub fn new(_initial_stage_count: usize) -> Self {
        Self {
            #[cfg(feature = "profile")]
            stages: HashMap::with_capacity(_initial_stage_count),
            #[cfg(feature = "profile")]
            start_timestamps: HashMap::with_capacity(_initial_stage_count),
        }
    }

    /// Gets the time it took to complete the last execution of the given stage.
    /// If no stage matching the given key ran, returns -1.
    pub fn get(&self, _stage: &'static str) -> f64 {
        #[cfg(feature = "profile")]
        {
            if let Some(&time) = self.stages.get(_stage) {
                return time;
            }
        }
        -1.0
    }

    /// Starts timing a stage.
    pub fn start(&mut self, _stage: &'static str) {
        #[cfg(feature = "profile")]
        {
            debug_assert!(
                !self.start_timestamps.contains_key(_stage),
                "Cannot start a stage that has already been started."
            );
            self.start_timestamps.insert(_stage, Instant::now());
        }
    }

    /// Ends timing a stage and accumulates the elapsed time.
    pub fn end(&mut self, _stage: &'static str) {
        #[cfg(feature = "profile")]
        {
            let end_time = Instant::now();
            debug_assert!(
                self.start_timestamps.contains_key(_stage),
                "To end a stage, it must currently be active (started and not already stopped)."
            );
            let start_time = self.start_timestamps.remove(_stage).unwrap();
            let elapsed = end_time.duration_since(start_time).as_secs_f64();
            let accumulated = self.stages.get(_stage).copied().unwrap_or(0.0);
            self.stages.insert(_stage, accumulated + elapsed);
        }
    }

    /// Clears all accumulated stage times.
    pub fn clear(&mut self) {
        #[cfg(feature = "profile")]
        {
            debug_assert!(
                self.start_timestamps.is_empty(),
                "It's likely that some stage was left unended from the previous frame."
            );
            self.stages.clear();
        }
    }
}
