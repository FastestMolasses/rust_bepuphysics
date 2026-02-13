// Translated from BepuPhysics/Constraints/MotorSettings.cs

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;

/// Defines some of the shared behavior across motor constraints.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MotorSettings {
    /// Maximum amount of force the motor can apply in one unit of time.
    pub maximum_force: f32,
    /// Mass-scaled damping constant. If you want to simulate a viscous damping coefficient of D
    /// with an object of mass M, set this damping value to D / M.
    pub damping: f32,
}

impl MotorSettings {
    /// Gets how soft the constraint is. Values range from 0 to infinity.
    /// Softness is inverse damping; 0 is perfectly rigid, 1 is very soft.
    #[inline(always)]
    pub fn softness(&self) -> f32 {
        1.0 / self.damping
    }

    /// Sets how soft the constraint is.
    #[inline(always)]
    pub fn set_softness(&mut self, value: f32) {
        self.damping = if value <= 0.0 { f32::MAX } else { 1.0 / value };
    }

    /// Checks if a settings instance has valid nonnegative values.
    #[inline(always)]
    pub fn validate(settings: &MotorSettings) -> bool {
        super::constraint_checker::ConstraintChecker::is_nonnegative_number(settings.maximum_force)
            && super::constraint_checker::ConstraintChecker::is_nonnegative_number(settings.damping)
    }

    /// Defines settings for a motor constraint.
    ///
    /// * `maximum_force` — Maximum amount of force the motor can apply in one unit of time.
    /// * `softness` — How soft the constraint is. 0 is perfectly rigid, 1 is very soft.
    #[inline(always)]
    pub fn new(maximum_force: f32, softness: f32) -> Self {
        let mut settings = Self {
            maximum_force,
            damping: 0.0,
        };
        settings.set_softness(softness);
        debug_assert!(
            Self::validate(&settings),
            "Motor settings must have nonnegative maximum force and nonnegative damping."
        );
        settings
    }
}

/// SIMD-wide motor settings.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MotorSettingsWide {
    pub maximum_force: Vector<f32>,
    pub damping: Vector<f32>,
}

impl MotorSettingsWide {
    #[inline(always)]
    pub fn write_first(source: &MotorSettings, target: &mut MotorSettingsWide) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target.maximum_force) = source.maximum_force;
            *GatherScatter::get_first_mut(&mut target.damping) = source.damping;
        }
    }

    #[inline(always)]
    pub fn read_first(source: &MotorSettingsWide, target: &mut MotorSettings) {
        target.maximum_force = source.maximum_force[0];
        target.damping = source.damping[0];
    }

    #[inline(always)]
    pub fn compute_softness(
        settings: &MotorSettingsWide,
        dt: f32,
        effective_mass_cfm_scale: &mut Vector<f32>,
        softness_impulse_scale: &mut Vector<f32>,
        maximum_impulse: &mut Vector<f32>,
    ) {
        let dt_wide = Vector::<f32>::splat(dt);
        let dtd = dt_wide * settings.damping;
        *maximum_impulse = settings.maximum_force * dt_wide;
        *softness_impulse_scale = Vector::<f32>::splat(1.0) / (dtd + Vector::<f32>::splat(1.0));
        *effective_mass_cfm_scale = dtd * *softness_impulse_scale;
    }
}
