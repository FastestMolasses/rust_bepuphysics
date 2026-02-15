//! Rain (Avian3D) — Hold spacebar to shower random physics objects onto a floor.
//!
//! Uses avian3d (Bevy-native ECS physics) with the same scene setup as the
//! bepu `rain` example for comparison.
//!
//! Controls:
//!   Hold SPACE  — spawn random objects
//!   R           — clear all objects
//!   Mouse drag  — orbit camera
//!   Scroll      — zoom

use avian3d::prelude::*;
use bevy::color::palettes::css;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use rand::Rng;

// ============================================================================
// Bevy components / markers
// ============================================================================

/// Marker for spawned dynamic entities (so we can despawn on reset).
#[derive(Component)]
struct Spawned;

/// Camera orbit state.
#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    yaw: f32,
    pitch: f32,
    target: Vec3,
}

/// Marker for the static scene (ground, walls, ramps) so we can keep them.
#[derive(Component)]
struct SceneStatic;

/// Tracks count of spawned objects.
#[derive(Resource, Default)]
struct SpawnedBodies {
    count: usize,
}

// ============================================================================
// Setup
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ---- Ground ----
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(200.0, 1.0, 200.0),
        Friction::new(0.7),
        Restitution::new(0.3),
        Mesh3d(meshes.add(Cuboid::new(200.0, 1.0, 200.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.27, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.5, 0.0),
        SceneStatic,
    ));

    // ---- Perimeter walls ----
    let wall_height = 2.0;
    let wall_thickness = 1.0;
    let ground_size = 200.0;
    let wall_data: [(Vec3, f32, f32, f32); 4] = [
        (
            Vec3::new(0.0, wall_height / 2.0, ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        (
            Vec3::new(0.0, wall_height / 2.0, -ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        (
            Vec3::new(ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
        (
            Vec3::new(-ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
    ];
    for (pos, width, height, depth) in &wall_data {
        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(*width, *height, *depth),
            Friction::new(0.7),
            Restitution::new(0.3),
            Mesh3d(meshes.add(Cuboid::new(*width, *height, *depth))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.35, 0.4),
                ..default()
            })),
            Transform::from_translation(*pos),
            SceneStatic,
        ));
    }

    // ---- Angled ramps ----
    let ramps: [(Vec3, Quat); 3] = [
        (Vec3::new(-8.0, 3.0, 0.0), Quat::from_rotation_z(0.3)),
        (Vec3::new(8.0, 5.0, 4.0), Quat::from_rotation_z(-0.25)),
        (Vec3::new(0.0, 7.0, -6.0), Quat::from_rotation_x(0.2)),
    ];
    for (pos, rot) in &ramps {
        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(12.0, 0.5, 6.0),
            Friction::new(0.7),
            Restitution::new(0.3),
            Mesh3d(meshes.add(Cuboid::new(12.0, 0.5, 6.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.4, 0.45),
                ..default()
            })),
            Transform {
                translation: *pos,
                rotation: *rot,
                ..default()
            },
            SceneStatic,
        ));
    }

    // ---- Camera ----
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(30.0, 25.0, 30.0).looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
        OrbitCamera {
            distance: 50.0,
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: 0.45,
            target: Vec3::new(0.0, 5.0, 0.0),
        },
    ));

    // ---- Lights ----
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.3, 0.0)),
    ));

    // ---- UI ----
    commands.spawn((
        Text::new(
            "Hold SPACE to rain objects | R to clear | Mouse drag to orbit, scroll to zoom\n[Avian3D]",
        ),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// ============================================================================
// Spawning system
// ============================================================================

fn spawn_objects(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut spawned: ResMut<SpawnedBodies>,
) {
    if !keyboard.pressed(KeyCode::Space) {
        return;
    }

    let mut rng = rand::rng();
    let batch_size = 10;

    for _ in 0..batch_size {
        let x = rng.random_range(-15.0..15.0f32);
        let z = rng.random_range(-15.0..15.0f32);
        let y = rng.random_range(20.0..40.0f32);
        let pos = Vec3::new(x, y, z);

        let rot = Quat::from_euler(
            EulerRot::XYZ,
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
        );

        let linvel = Vec3::new(
            rng.random_range(-2.0..2.0),
            rng.random_range(-5.0..-1.0),
            rng.random_range(-2.0..2.0),
        );
        let angvel = Vec3::new(
            rng.random_range(-3.0..3.0),
            rng.random_range(-3.0..3.0),
            rng.random_range(-3.0..3.0),
        );

        // Random color
        let color: Color = match rng.random_range(0..8u32) {
            0 => css::TOMATO.into(),
            1 => css::GOLD.into(),
            2 => css::LIME.into(),
            3 => css::DODGER_BLUE.into(),
            4 => css::HOT_PINK.into(),
            5 => css::ORANGE.into(),
            6 => css::AQUA.into(),
            _ => css::WHITE.into(),
        };

        // Random shape
        let shape_kind: u32 = rng.random_range(0..4);
        let (collider, mesh_handle) = match shape_kind {
            0 => {
                let w = rng.random_range(0.6..2.4f32);
                let h = rng.random_range(0.6..2.4f32);
                let d = rng.random_range(0.6..2.4f32);
                (
                    Collider::cuboid(w, h, d),
                    meshes.add(Cuboid::new(w, h, d)),
                )
            }
            1 => {
                let r = rng.random_range(0.3..1.0f32);
                (
                    Collider::sphere(r),
                    meshes.add(bevy::math::primitives::Sphere::new(r)),
                )
            }
            2 => {
                let r = rng.random_range(0.2..0.6f32);
                let l = rng.random_range(0.5..2.0f32);
                (
                    Collider::capsule(r, l),
                    meshes.add(bevy::math::primitives::Capsule3d::new(r, l)),
                )
            }
            _ => {
                let r = rng.random_range(0.3..0.8f32);
                let l = rng.random_range(0.4..1.5f32);
                (
                    Collider::cylinder(r, l),
                    meshes.add(bevy::math::primitives::Cylinder::new(r, l)),
                )
            }
        };

        commands.spawn((
            RigidBody::Dynamic,
            collider,
            Friction::new(0.7),
            Restitution::new(0.3),
            LinearVelocity(linvel),
            AngularVelocity(angvel),
            Mesh3d(mesh_handle),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform {
                translation: pos,
                rotation: rot,
                ..default()
            },
            Spawned,
        ));

        spawned.count += 1;
    }
}

// ============================================================================
// Clear system
// ============================================================================

fn clear_objects(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut spawned: ResMut<SpawnedBodies>,
    query: Query<Entity, With<Spawned>>,
) {
    if !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    spawned.count = 0;

    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    info!("Reset simulation (Avian)");
}

// ============================================================================
// HUD
// ============================================================================

#[derive(Component)]
struct CounterText;

#[derive(Component)]
struct FpsText;

fn setup_counter(mut commands: Commands) {
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

fn update_counter(spawned: Res<SpawnedBodies>, mut query: Query<&mut Text, With<CounterText>>) {
    if let Ok(mut text) = query.single_mut() {
        **text = format!("Objects: {}", spawned.count);
    }
}

fn update_fps(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    if let Ok(mut text) = query.single_mut() {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(val) = fps.smoothed() {
                **text = format!("FPS: {:.0}", val);
            }
        }
    }
}

// ============================================================================
// Camera orbit
// ============================================================================

fn orbit_camera(
    mouse_button: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let Ok((mut orbit, mut transform)) = query.single_mut() else {
        return;
    };

    if mouse_button.pressed(MouseButton::Left) {
        let delta = mouse_motion.delta;
        orbit.yaw -= delta.x * 0.005;
        orbit.pitch = (orbit.pitch + delta.y * 0.005).clamp(-1.4, 1.4);
    }

    let scroll = mouse_scroll.delta.y;
    if scroll != 0.0 {
        orbit.distance = (orbit.distance - scroll * 2.0).clamp(5.0, 200.0);
    }

    let x = orbit.distance * orbit.pitch.cos() * orbit.yaw.sin();
    let y = orbit.distance * orbit.pitch.sin();
    let z = orbit.distance * orbit.pitch.cos() * orbit.yaw.cos();
    transform.translation = orbit.target + Vec3::new(x, y, z);
    transform.look_at(orbit.target, Vec3::Y);
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Physics Rain (Avian3D)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(Gravity(Vec3::new(0.0, -20.0, 0.0)))
        .insert_resource(SpawnedBodies::default())
        .add_systems(Startup, (setup, setup_counter))
        .add_systems(Update, (orbit_camera, update_counter, update_fps))
        .add_systems(FixedUpdate, (spawn_objects, clear_objects).chain())
        .run();
}
