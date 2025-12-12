use super::collidable::ContinuousDetection;
use super::typed_index::TypedIndex;

/// Describes a collidable and how it should handle continuous collision detection.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CollidableDescription {
    /// Shape of the collidable.
    pub shape: TypedIndex,
    /// Continuous collision detection settings used by the collidable.
    pub continuity: ContinuousDetection,
    /// Lower bound on the value of the speculative margin used by the collidable.
    pub minimum_speculative_margin: f32,
    /// Upper bound on the value of the speculative margin used by the collidable.
    pub maximum_speculative_margin: f32,
}

impl CollidableDescription {
    /// Constructs a new collidable description.
    #[inline(always)]
    pub fn new(
        shape: TypedIndex,
        minimum_speculative_margin: f32,
        maximum_speculative_margin: f32,
        continuity: ContinuousDetection,
    ) -> Self {
        Self {
            shape,
            minimum_speculative_margin,
            maximum_speculative_margin,
            continuity,
        }
    }

    /// Constructs a new collidable description with Discrete mode.
    #[inline(always)]
    pub fn with_discrete(
        shape: TypedIndex,
        minimum_speculative_margin: f32,
        maximum_speculative_margin: f32,
    ) -> Self {
        Self {
            shape,
            minimum_speculative_margin,
            maximum_speculative_margin,
            continuity: ContinuousDetection::discrete(),
        }
    }

    /// Constructs a new collidable description with 0 minimum speculative margin.
    #[inline(always)]
    pub fn with_max_margin(
        shape: TypedIndex,
        maximum_speculative_margin: f32,
        continuity: ContinuousDetection,
    ) -> Self {
        Self {
            shape,
            minimum_speculative_margin: 0.0,
            maximum_speculative_margin,
            continuity,
        }
    }

    /// Constructs a new collidable description with 0 minimum and f32::MAX maximum speculative margin.
    #[inline(always)]
    pub fn with_continuity(shape: TypedIndex, continuity: ContinuousDetection) -> Self {
        Self {
            shape,
            minimum_speculative_margin: 0.0,
            maximum_speculative_margin: f32::MAX,
            continuity,
        }
    }

    /// Constructs a new collidable description with Passive mode, 0 minimum and f32::MAX maximum margin.
    #[inline(always)]
    pub fn from_shape(shape: TypedIndex) -> Self {
        Self {
            shape,
            minimum_speculative_margin: 0.0,
            maximum_speculative_margin: f32::MAX,
            continuity: ContinuousDetection::passive(),
        }
    }

    /// Constructs a new collidable description with Discrete mode, 0 minimum margin.
    #[inline(always)]
    pub fn with_discrete_margin(shape: TypedIndex, maximum_speculative_margin: f32) -> Self {
        Self {
            shape,
            minimum_speculative_margin: 0.0,
            maximum_speculative_margin,
            continuity: ContinuousDetection::discrete(),
        }
    }
}

impl From<TypedIndex> for CollidableDescription {
    fn from(shape: TypedIndex) -> Self {
        Self::from_shape(shape)
    }
}
