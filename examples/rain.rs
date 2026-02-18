//! Rain — Hold spacebar to shower random physics objects onto a floor.
//!
//! Objects include boxes, spheres, capsules, and cylinders with randomized sizes,
//! colors, and bounce. Release spacebar to stop spawning; objects stay and pile up.
//!
//! Controls:
//!   Hold SPACE     — spawn random objects
//!   R              — clear all objects
//!   Right-click    — create explosion at cursor
//!   Left drag      — orbit camera
//!   Scroll         — zoom
//!
//! NOTE: Run with --release for best performance!

#![feature(portable_simd)]
#![feature(generic_const_exprs)]

use bevy::color::palettes::css;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use rand::Rng;
use rust_bepuphysics::bevy_bepu::prelude::*;

mod common;
use common::*;

/// Tracks the count of spawned dynamic objects.
#[derive(Resource, Default)]
struct SpawnedObjects {
    count: usize,
}

impl ObjectCounter for SpawnedObjects {
    fn count(&self) -> usize {
        self.count
    }
}

/// Marker for spawned dynamic entities (so we can despawn on reset).
#[derive(Component)]
struct Spawned;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ---- Ground plane ----
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(200.0, 1.0, 200.0),
        Mesh3d(meshes.add(Cuboid::new(200.0, 1.0, 200.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.27, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    // ---- Perimeter walls ----
    let wall_height = 2.0;
    let wall_thickness = 1.0;
    let ground_size = 200.0;

    // North wall (along X axis, at +Z)
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(ground_size, wall_height, wall_thickness),
        Mesh3d(meshes.add(Cuboid::new(ground_size, wall_height, wall_thickness))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.4),
            ..default()
        })),
        Transform::from_xyz(0.0, wall_height / 2.0, ground_size / 2.0),
    ));

    // South wall
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(ground_size, wall_height, wall_thickness),
        Mesh3d(meshes.add(Cuboid::new(ground_size, wall_height, wall_thickness))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.4),
            ..default()
        })),
        Transform::from_xyz(0.0, wall_height / 2.0, -ground_size / 2.0),
    ));

    // East wall
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(wall_thickness, wall_height, ground_size),
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_height, ground_size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.4),
            ..default()
        })),
        Transform::from_xyz(ground_size / 2.0, wall_height / 2.0, 0.0),
    ));

    // West wall
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(wall_thickness, wall_height, ground_size),
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_height, ground_size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.4),
            ..default()
        })),
        Transform::from_xyz(-ground_size / 2.0, wall_height / 2.0, 0.0),
    ));

    // ---- Some angled ramps for fun bouncing ----
    let ramp_positions_rotations: [(Vec3, Quat); 3] = [
        (Vec3::new(-8.0, 3.0, 0.0), Quat::from_rotation_z(0.3)),
        (Vec3::new(8.0, 5.0, 4.0), Quat::from_rotation_z(-0.25)),
        (Vec3::new(0.0, 7.0, -6.0), Quat::from_rotation_x(0.2)),
    ];

    for (pos, rot) in &ramp_positions_rotations {
        commands.spawn((
            RigidBody::Static,
            BepuCollider::cuboid(12.0, 0.5, 6.0),
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

    // ---- Light ----
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
        Text::new("Hold SPACE to rain objects\nR to clear\nRight-click for explosion\nLeft drag to orbit, scroll to zoom"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

fn spawn_objects(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut spawned: ResMut<SpawnedObjects>,
) {
    if !keyboard.pressed(KeyCode::Space) {
        return;
    }

    let mut rng = rand::thread_rng();

    // Spawn 10 objects per tick
    let batch_size = 10;
    for _ in 0..batch_size {
        // Random position in a region above the scene
        let pos = Vec3::new(
            rng.random_range(-15.0..15.0),
            rng.random_range(20.0..40.0),
            rng.random_range(-15.0..15.0),
        );

        // Random orientation
        let rotation = Quat::from_euler(
            EulerRot::XYZ,
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
        );

        // Small random downward + lateral velocity
        let vel = Vec3::new(
            rng.random_range(-2.0..2.0),
            rng.random_range(-5.0..-1.0),
            rng.random_range(-2.0..2.0),
        );

        let angular_vel = Vec3::new(
            rng.random_range(-3.0..3.0),
            rng.random_range(-3.0..3.0),
            rng.random_range(-3.0..3.0),
        );

        // Pick a random shape
        let shape_kind: u32 = rng.random_range(0..4);
        let (collider, mesh, mass, restitution) = match shape_kind {
            0 => {
                // Box
                let hw = rng.random_range(0.3..1.2f32);
                let hh = rng.random_range(0.3..1.2f32);
                let hd = rng.random_range(0.3..1.2f32);
                let bounce = if rng.random::<f32>() < 0.3 {
                    rng.random_range(0.6..0.95)
                } else {
                    0.1
                };
                (
                    BepuCollider::cuboid(hw * 2.0, hh * 2.0, hd * 2.0),
                    meshes.add(Cuboid::new(hw * 2.0, hh * 2.0, hd * 2.0)),
                    hw * hh * hd * 4.0,
                    bounce,
                )
            }
            1 => {
                // Sphere
                let r = rng.random_range(0.3..1.0f32);
                let bounce = if rng.random::<f32>() < 0.3 {
                    rng.random_range(0.6..0.95)
                } else {
                    0.1
                };
                (
                    BepuCollider::sphere(r),
                    meshes.add(bevy::math::primitives::Sphere::new(r)),
                    r * r * r * 6.0,
                    bounce,
                )
            }
            2 => {
                // Capsule
                let r = rng.random_range(0.2..0.6f32);
                let l = rng.random_range(0.5..2.0f32);
                let bounce = if rng.random::<f32>() < 0.3 {
                    rng.random_range(0.6..0.95)
                } else {
                    0.1
                };
                (
                    BepuCollider::capsule(r, l),
                    meshes.add(bevy::math::primitives::Capsule3d::new(r, l)),
                    r * r * l * 8.0,
                    bounce,
                )
            }
            _ => {
                // Cylinder
                let r = rng.random_range(0.3..0.8f32);
                let l = rng.random_range(0.4..1.5f32);
                let bounce = if rng.random::<f32>() < 0.3 {
                    rng.random_range(0.6..0.95)
                } else {
                    0.1
                };
                (
                    BepuCollider::cylinder(r, l),
                    meshes.add(bevy::math::primitives::Cylinder::new(r, l)),
                    r * r * l * 6.0,
                    bounce,
                )
            }
        };

        // Random color
        let color = match rng.random_range(0..8u32) {
            0 => css::TOMATO.into(),
            1 => css::GOLD.into(),
            2 => css::LIME.into(),
            3 => css::DODGER_BLUE.into(),
            4 => css::HOT_PINK.into(),
            5 => css::ORANGE.into(),
            6 => css::AQUA.into(),
            _ => css::WHITE.into(),
        };

        commands.spawn((
            RigidBody::Dynamic,
            collider,
            Mass(mass),
            Restitution(restitution),
            Friction(0.5),
            LinearVelocity(vel),
            AngularVelocity(angular_vel),
            Mesh3d(mesh),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform {
                translation: pos,
                rotation,
                ..default()
            },
            Spawned,
        ));

        spawned.count += 1;
    }
}
fn clear_objects(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    query: Query<Entity, With<Spawned>>,
    mut spawned: ResMut<SpawnedObjects>,
) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        for entity in query.iter() {
            commands.entity(entity).despawn();
        }
        spawned.count = 0;
    }
}

fn handle_explosion_input(
    mouse_button: Res<ButtonInput<MouseButton>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    window_query: Query<&Window, With<PrimaryWindow>>,
    mut bodies_query: Query<(&Transform, &mut LinearVelocity)>,
) {
    if !mouse_button.just_pressed(MouseButton::Right) {
        return;
    }

    // Raycast from camera through mouse cursor
    let Ok(window) = window_query.single() else {
        return;
    };
    let Some(cursor_position) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = camera_query.single() else {
        return;
    };

    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        return;
    };

    // Find intersection with ground plane (y = 0)
    let t = -ray.origin.y / ray.direction.y;
    if t <= 0.0 {
        return;
    }
    let explosion_pos = ray.origin + ray.direction * t;

    // Apply explosion force to nearby bodies
    let explosion_force = 500.0;
    let explosion_radius = 15.0;

    for (transform, mut velocity) in bodies_query.iter_mut() {
        let offset = transform.translation - explosion_pos;
        let distance = offset.length();
        if distance < explosion_radius && distance > 0.01 {
            let direction = offset / distance;
            let falloff = 1.0 - (distance / explosion_radius);
            let impulse = direction * explosion_force * falloff;
            velocity.0 += impulse;
        }
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "BepuPhysics Rain Demo (Bevy Plugin)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(BepuPhysicsPlugin)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(Gravity(Vec3::new(0.0, -20.0, 0.0)))
        .insert_resource(BepuConfig {
            deterministic: false,
            thread_count: None, // Use all cores
            ..default()
        })
        .init_resource::<SpawnedObjects>()
        .add_systems(Startup, (setup, setup_fps, setup_counter))
        .add_systems(
            Update,
            (
                orbit_camera,
                update_fps,
                update_counter::<SpawnedObjects>,
                handle_explosion_input,
            ),
        )
        .add_systems(FixedUpdate, (spawn_objects, clear_objects))
        .run();
}
