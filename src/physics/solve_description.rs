/// Callback executed to determine how many velocity iterations should be used for a given substep.
///
/// Returns the number of velocity iterations to use for the substep. If nonpositive,
/// `SolveDescription::velocity_iteration_count` will be used for the substep instead.
pub type SubstepVelocityIterationScheduler = Box<dyn Fn(i32) -> i32>;

/// Describes how the solver should schedule substeps and velocity iterations.
pub struct SolveDescription {
    /// Number of velocity iterations to use in the solver if there is no `velocity_iteration_scheduler`
    /// or if it returns a non-positive value for a substep.
    pub velocity_iteration_count: i32,
    /// Number of substeps to execute each time the solver runs.
    pub substep_count: i32,
    /// Number of synchronized constraint batches to use before using a fallback approach.
    pub fallback_batch_threshold: i32,
    /// Callback executed to determine how many velocity iterations should be used for a given substep.
    /// If `None`, or if it returns a non-positive value, the `velocity_iteration_count` will be used instead.
    pub velocity_iteration_scheduler: Option<SubstepVelocityIterationScheduler>,
}

impl SolveDescription {
    /// Default number of synchronized constraint batches to use before falling back to an alternative solving method.
    pub const DEFAULT_FALLBACK_BATCH_THRESHOLD: i32 = 64;

    fn validate_description(&self) {
        assert!(self.substep_count >= 1, "Substep count must be positive.");
        assert!(
            self.velocity_iteration_count >= 1,
            "Velocity iteration count must be positive."
        );
        assert!(
            self.fallback_batch_threshold >= 1,
            "Fallback batch threshold must be positive."
        );
    }

    /// Creates a solve description.
    ///
    /// # Arguments
    /// * `velocity_iteration_count` - Number of velocity iterations per substep.
    /// * `substep_count` - Number of substeps in the solve.
    /// * `fallback_batch_threshold` - Number of synchronized constraint batches to use before using a fallback approach.
    pub fn new(
        velocity_iteration_count: i32,
        substep_count: i32,
        fallback_batch_threshold: i32,
    ) -> Self {
        let desc = Self {
            substep_count,
            velocity_iteration_count,
            fallback_batch_threshold,
            velocity_iteration_scheduler: None,
        };
        desc.validate_description();
        desc
    }

    /// Creates a solve description with the default fallback batch threshold.
    pub fn with_defaults(velocity_iteration_count: i32, substep_count: i32) -> Self {
        Self::new(
            velocity_iteration_count,
            substep_count,
            Self::DEFAULT_FALLBACK_BATCH_THRESHOLD,
        )
    }

    /// Creates a solve description with a velocity iteration scheduler.
    pub fn with_scheduler(
        substep_count: i32,
        velocity_iteration_scheduler: SubstepVelocityIterationScheduler,
        fallback_velocity_iteration_count: i32,
        fallback_batch_threshold: i32,
    ) -> Self {
        let desc = Self {
            substep_count,
            velocity_iteration_count: fallback_velocity_iteration_count,
            fallback_batch_threshold,
            velocity_iteration_scheduler: Some(velocity_iteration_scheduler),
        };
        desc.validate_description();
        desc
    }

    /// Creates a solve description from a slice of per-substep velocity iterations.
    pub fn from_substep_iterations(
        substep_velocity_iterations: &[i32],
        fallback_velocity_iteration_count: i32,
        fallback_batch_threshold: i32,
    ) -> Self {
        let copy = substep_velocity_iterations.to_vec();
        let desc = Self {
            substep_count: copy.len() as i32,
            velocity_iteration_count: fallback_velocity_iteration_count,
            fallback_batch_threshold,
            velocity_iteration_scheduler: Some(Box::new(move |substep_index| {
                copy[substep_index as usize]
            })),
        };
        desc.validate_description();
        desc
    }
}

/// Creates a solve description with the given number of velocity iterations and a single substep.
impl From<i32> for SolveDescription {
    fn from(velocity_iteration_count: i32) -> Self {
        SolveDescription::with_defaults(velocity_iteration_count, 1)
    }
}

/// Creates a solve description from (iterations_per_substep, substep_count).
impl From<(i32, i32)> for SolveDescription {
    fn from((iterations_per_substep, substep_count): (i32, i32)) -> Self {
        SolveDescription::with_defaults(iterations_per_substep, substep_count)
    }
}
