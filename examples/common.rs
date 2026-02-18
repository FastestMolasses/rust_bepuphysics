//! Shared utilities for BepuPhysics examples.
//!
//! This module contains common components and systems used across multiple examples,
//! including FPS display, object counters, and camera orbit controls.

use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

// ============================================================================
// FPS Counter
// ============================================================================

/// Component marker for the FPS display text.
#[derive(Component)]
pub struct FpsText;

/// Spawns an FPS counter in the top-right corner.
pub fn setup_fps(mut commands: Commands) {
    commands.spawn((
        Text::new("FPS: --"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            right: Val::Px(12.0),
            ..default()
        },
        FpsText,
    ));
}

/// Updates the FPS counter text.
pub fn update_fps(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    if let Ok(mut text) = query.single_mut() {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(val) = fps.smoothed() {
                **text = format!("FPS: {:.0}", val);
            }
        }
    }
}

// ============================================================================
// Object Counter
// ============================================================================

/// Component marker for the object count display text.
#[derive(Component)]
pub struct CounterText;

/// Spawns an object counter below the FPS counter.
pub fn setup_counter(mut commands: Commands) {
    commands.spawn((
        Text::new("Objects: 0"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(36.0),
            right: Val::Px(12.0),
            ..default()
        },
        CounterText,
    ));
}

/// Updates the object counter text based on a value resource.
pub fn update_counter<T: Resource>(counter: Res<T>, mut query: Query<&mut Text, With<CounterText>>)
where
    T: ObjectCounter,
{
    if let Ok(mut text) = query.single_mut() {
        **text = format!("Objects: {}", counter.count());
    }
}

/// Trait for resources that provide an object count.
pub trait ObjectCounter {
    fn count(&self) -> usize;
}

// ============================================================================
// Orbit Camera
// ============================================================================

/// Component for controlling an orbiting camera.
#[derive(Component)]
pub struct OrbitCamera {
    /// Distance from the target point.
    pub distance: f32,
    /// Horizontal rotation angle (radians).
    pub yaw: f32,
    /// Vertical angle (radians, clamped to prevent gimbal lock).
    pub pitch: f32,
    /// Point in world space to orbit around.
    pub target: Vec3,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            distance: 25.0,
            yaw: 0.0,
            pitch: 0.5,
            target: Vec3::new(0.0, 5.0, 0.0),
        }
    }
}

/// System that handles mouse input for the orbit camera.
///
/// - Left-click drag to orbit
/// - Scroll to zoom in/out
pub fn orbit_camera(
    mouse_button: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let Ok((mut orbit, mut transform)) = query.single_mut() else {
        return;
    };

    // Left-drag to orbit
    if mouse_button.pressed(MouseButton::Left) {
        let delta = mouse_motion.delta;
        orbit.yaw -= delta.x * 0.005;
        orbit.pitch = (orbit.pitch + delta.y * 0.005).clamp(-1.4, 1.4);
    }

    // Scroll to zoom
    let scroll = mouse_scroll.delta.y;
    if scroll != 0.0 {
        orbit.distance = (orbit.distance - scroll * 2.0).clamp(5.0, 200.0);
    }

    // Update camera position and look direction
    let x = orbit.distance * orbit.pitch.cos() * orbit.yaw.sin();
    let y = orbit.distance * orbit.pitch.sin();
    let z = orbit.distance * orbit.pitch.cos() * orbit.yaw.cos();
    transform.translation = orbit.target + Vec3::new(x, y, z);
    transform.look_at(orbit.target, Vec3::Y);
}
