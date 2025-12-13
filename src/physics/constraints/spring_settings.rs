// Translated from BepuPhysics/Constraints/SpringSettings.cs

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;

/// SIMD-wide spring settings, aligned with execution order.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SpringSettingsWide {
    pub angular_frequency: Vector<f32>,
    pub twice_damping_ratio: Vector<f32>,
}

impl SpringSettingsWide {
    #[inline(always)]
    pub fn write_first(source: &SpringSettings, target: &mut SpringSettingsWide) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target.angular_frequency) = source.angular_frequency;
            *GatherScatter::get_first_mut(&mut target.twice_damping_ratio) =
                source.twice_damping_ratio;
        }
    }

    #[inline(always)]
    pub fn read_first(source: &SpringSettingsWide, target: &mut SpringSettings) {
        target.angular_frequency = source.angular_frequency[0];
        target.twice_damping_ratio = source.twice_damping_ratio[0];
    }

    /// Computes springiness values for a set of constraints.
    #[inline(always)]
    pub fn compute_springiness(
        settings: &SpringSettingsWide,
        dt: f32,
        position_error_to_velocity: &mut Vector<f32>,
        effective_mass_cfm_scale: &mut Vector<f32>,
        softness_impulse_scale: &mut Vector<f32>,
    ) {
        let dt_wide = Vector::<f32>::splat(dt);
        let angular_frequency_dt = settings.angular_frequency * dt_wide;
        *position_error_to_velocity =
            settings.angular_frequency / (angular_frequency_dt + settings.twice_damping_ratio);
        let extra = Vector::<f32>::splat(1.0)
            / (angular_frequency_dt * (angular_frequency_dt + settings.twice_damping_ratio));
        *effective_mass_cfm_scale = Vector::<f32>::splat(1.0) / (Vector::<f32>::splat(1.0) + extra);
        *softness_impulse_scale = extra * *effective_mass_cfm_scale;
    }
}

/// Scalar spring settings describing the frequency and damping of a springy constraint.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct SpringSettings {
    /// Target number of undamped oscillations per unit of time, scaled by 2 * PI.
    pub angular_frequency: f32,
    /// Twice the ratio of the spring's actual damping to its critical damping.
    pub twice_damping_ratio: f32,
}

impl SpringSettings {
    /// Gets the target number of undamped oscillations per unit of time.
    #[inline(always)]
    pub fn frequency(&self) -> f32 {
        self.angular_frequency / (2.0 * std::f32::consts::PI)
    }

    /// Sets the target number of undamped oscillations per unit of time.
    #[inline(always)]
    pub fn set_frequency(&mut self, value: f32) {
        self.angular_frequency = value * (2.0 * std::f32::consts::PI);
    }

    /// Gets the ratio of the spring's actual damping to its critical damping.
    #[inline(always)]
    pub fn damping_ratio(&self) -> f32 {
        self.twice_damping_ratio / 2.0
    }

    /// Sets the ratio of the spring's actual damping to its critical damping.
    #[inline(always)]
    pub fn set_damping_ratio(&mut self, value: f32) {
        self.twice_damping_ratio = value * 2.0;
    }

    /// Checks if a spring settings instance contains valid values.
    #[inline(always)]
    pub fn validate(settings: &SpringSettings) -> bool {
        super::constraint_checker::ConstraintChecker::is_positive_number(settings.angular_frequency)
            && super::constraint_checker::ConstraintChecker::is_nonnegative_number(
                settings.twice_damping_ratio,
            )
    }

    /// Constructs a new spring settings instance.
    ///
    /// * `frequency` — Target number of undamped oscillations per unit of time.
    /// * `damping_ratio` — Ratio of the spring's actual damping to its critical damping.
    ///   0 is undamped, 1 is critically damped, and higher values are overdamped.
    pub fn new(frequency: f32, damping_ratio: f32) -> Self {
        let settings = Self {
            angular_frequency: frequency * (2.0 * std::f32::consts::PI),
            twice_damping_ratio: damping_ratio * 2.0,
        };
        debug_assert!(
            Self::validate(&settings),
            "Spring settings must have positive frequency and nonnegative damping ratio."
        );
        settings
    }
}
