use super::typed_index::TypedIndex;

/// Defines how a collidable will handle collision detection in the presence of velocity.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContinuousDetectionMode {
    /// No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.
    /// The collidable's bounding box will not be expanded by velocity beyond the speculative margin.
    Discrete = 0,
    /// No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.
    /// The collidable's bounding box will be expanded by velocity without being limited by the speculative margin.
    Passive = 1,
    /// Collision detection will start with a sweep test to identify a likely time of impact.
    /// Speculative contacts will be generated for the predicted collision.
    Continuous = 2,
}

/// Defines how a collidable handles collisions with significant velocity.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ContinuousDetection {
    /// The continuous collision detection mode.
    pub mode: ContinuousDetectionMode,
    /// If using Continuous mode, this defines the minimum progress that the sweep test will make
    /// when searching for the first time of impact.
    pub minimum_sweep_timestep: f32,
    /// If using Continuous mode, sweep tests will terminate if the time of impact region
    /// has been refined to be smaller than this threshold.
    pub sweep_convergence_threshold: f32,
}

impl ContinuousDetection {
    /// Gets whether the continuous collision detection configuration will permit
    /// bounding box expansion beyond the calculated speculative margin.
    #[inline(always)]
    pub fn allow_expansion_beyond_speculative_margin(&self) -> bool {
        (self.mode as u32) > 0
    }

    /// No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.
    /// The collidable's bounding box will not be expanded by velocity beyond the speculative margin.
    #[inline(always)]
    pub fn discrete() -> Self {
        Self {
            mode: ContinuousDetectionMode::Discrete,
            minimum_sweep_timestep: 0.0,
            sweep_convergence_threshold: 0.0,
        }
    }

    /// No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.
    /// The collidable's bounding box and speculative margin will be expanded by velocity.
    #[inline(always)]
    pub fn passive() -> Self {
        Self {
            mode: ContinuousDetectionMode::Passive,
            minimum_sweep_timestep: 0.0,
            sweep_convergence_threshold: 0.0,
        }
    }

    /// Collision detection will start with a sweep test to identify a likely time of impact.
    /// Speculative contacts will be generated for the predicted collision.
    #[inline(always)]
    pub fn continuous(minimum_sweep_timestep: f32, sweep_convergence_threshold: f32) -> Self {
        Self {
            mode: ContinuousDetectionMode::Continuous,
            minimum_sweep_timestep,
            sweep_convergence_threshold,
        }
    }

    /// Continuous mode with default parameters (1e-3 for both thresholds).
    #[inline(always)]
    pub fn continuous_default() -> Self {
        Self::continuous(1e-3, 1e-3)
    }
}

impl Default for ContinuousDetection {
    fn default() -> Self {
        Self::discrete()
    }
}

/// Description of a collidable used by a body living in the broad phase and able to generate collision pairs.
/// Collidables with a ShapeIndex that points to nothing (a default constructed TypedIndex) are not capable
/// of colliding with anything.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Collidable {
    /// Index of the shape used by the body.
    pub shape: TypedIndex,
    /// Continuous collision detection settings for this collidable.
    pub continuity: ContinuousDetection,
    /// Lower bound on the value of the speculative margin used by the collidable.
    pub minimum_speculative_margin: f32,
    /// Upper bound on the value of the speculative margin used by the collidable.
    pub maximum_speculative_margin: f32,
    /// Automatically computed size of the margin around the surface of the shape
    /// in which contacts can be generated.
    pub speculative_margin: f32,
    /// Index of the collidable in the broad phase.
    pub broad_phase_index: i32,
}

impl Default for Collidable {
    fn default() -> Self {
        Self {
            shape: TypedIndex::default(),
            continuity: ContinuousDetection::default(),
            minimum_speculative_margin: 0.0,
            maximum_speculative_margin: f32::MAX,
            speculative_margin: 0.0,
            broad_phase_index: 0,
        }
    }
}
