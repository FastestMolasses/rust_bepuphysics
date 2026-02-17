//! Minimal example: a sphere falling onto a static floor.
//!
//! Demonstrates the BepuPhysics Bevy plugin with the simplest possible setup.

#![feature(portable_simd)]
#![feature(generic_const_exprs)]

use bevy::prelude::*;
use rust_bepuphysics::bevy_bepu::prelude::*;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, BepuPhysicsPlugin))
        .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // --- Static ground ---
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(100.0, 1.0, 100.0),
        Mesh3d(meshes.add(Cuboid::new(100.0, 1.0, 100.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.35),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    // --- Dynamic sphere ---
    commands.spawn((
        RigidBody::Dynamic,
        BepuCollider::sphere(0.5),
        Mass(1.0),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            ..default()
        })),
        Transform::from_xyz(0.0, 5.0, 0.0),
    ));

    // --- A few more shapes ---
    commands.spawn((
        RigidBody::Dynamic,
        BepuCollider::cuboid(1.0, 1.0, 1.0),
        Mass(2.0),
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.6, 0.2),
            ..default()
        })),
        Transform::from_xyz(2.0, 8.0, 0.0),
    ));

    commands.spawn((
        RigidBody::Dynamic,
        BepuCollider::capsule(0.3, 1.0),
        Mass(1.5),
        Mesh3d(meshes.add(Capsule3d::new(0.3, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.8),
            ..default()
        })),
        Transform::from_xyz(-2.0, 10.0, 1.0),
    ));

    // --- Camera ---
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 8.0, 10.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));

    // --- Light ---
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.8, 0.3, 0.0)),
    ));
}
