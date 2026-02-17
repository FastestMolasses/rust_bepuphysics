# BepuPhysics for Rust

A line-by-line translation of [BepuPhysics v2](https://github.com/bepu/bepuphysics2), an extremely high performance physics engine.

## Features

- **High Performance**: SIMD-optimized collision detection and constraint solving
- **Multithreaded**: Lock-free task scheduling with automatic work distribution
- **Bevy Integration**: Optional plugin for seamless integration with the Bevy game engine

## Requirements

- **Rust nightly** — requires `#![feature(portable_simd)]` and `#![feature(generic_const_exprs)]`
- macOS (NEON SIMD) or x86_64 (SSE/AVX2)

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
rust_bepuphysics = "0.1.0"

# For Bevy integration (optional):
rust_bepuphysics = { version = "0.1.0", features = ["bevy"] }
```

## Quick Start with Bevy

The simplest way to use BepuPhysics is through the Bevy plugin:

```rust
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
    // Static ground
    commands.spawn((
        RigidBody::Static,
        BepuCollider::cuboid(100.0, 1.0, 100.0),
        Transform::from_xyz(0.0, -0.5, 0.0),
        Mesh3d(meshes.add(Cuboid::new(200.0, 2.0, 200.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.3, 0.35))),
    ));

    // Dynamic sphere
    commands.spawn((
        RigidBody::Dynamic,
        BepuCollider::sphere(0.5),
        Mass(1.0),
        Transform::from_xyz(0.0, 5.0, 0.0),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.2, 0.2))),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 8.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
```

### Bevy Components

**Body Types:**
- `RigidBody::Dynamic` — moved by physics solver
- `RigidBody::Static` — immovable (infinite mass)
- `RigidBody::Kinematic` — moved by user, affects other bodies

**Colliders:**
- `BepuCollider::sphere(radius)`
- `BepuCollider::cuboid(half_width, half_height, half_depth)`
- `BepuCollider::capsule(radius, length)`
- `BepuCollider::cylinder(radius, length)`

**Material Properties:**
- `Mass(f32)` — default: 1.0
- `Friction(f32)` — default: 0.5
- `Restitution(f32)` — bounciness (0.0 = no bounce, 1.0 = perfect bounce)
- `LinearDamping(f32)` / `AngularDamping(f32)` — velocity decay

**Motion:**
- `LinearVelocity(Vec3)` — bidirectional sync with physics
- `AngularVelocity(Vec3)` — bidirectional sync with physics

**Resources:**
- `Gravity(Vec3)` — global gravity (default: `Vec3::new(0.0, -9.81, 0.0)`)
- `BepuConfig` — simulation settings (timestep, substeps, thread count, determinism)

### Configuration

```rust
.insert_resource(BepuConfig {
    velocity_iterations: 4,  // More = more accurate but slower
    substep_count: 1,        // Substeps per timestep
    deterministic: false,    // Enable for replay/networking
    thread_count: None,      // None = use all cores, Some(n) = use n threads
    ..default()
})
```

## Running Examples

**⚠️ IMPORTANT: Always run examples in release mode!** Debug builds are 10-100x slower due to disabled SIMD optimizations.

```bash
# Rain demo - press SPACE to spawn objects, R to clear, right-click for explosions
cargo +nightly run --release --features bevy --example rain

# Tower demo - stable stack of boxes
cargo +nightly run --release --features bevy --example tower

# Simple hello world
cargo +nightly run --release --features bevy --example hello_physics

cargo +nightly run --release --example rain_csharp  # Using C# FFI (experimental)
cargo +nightly run --release --example rain_avian  # Avian physics comparison
```

## License

Licensed under the Apache License, Version 2.0, matching the original [BepuPhysics v2](https://github.com/bepu/bepuphysics2) license.

See [LICENSE.md](LICENSE.md) for the full license text.

## Credits

- **Original BepuPhysics** by Ross Nordby ([https://github.com/bepu/bepuphysics2](https://github.com/bepu/bepuphysics2))
- **Bevy plugin** — ECS-friendly wrapper inspired by the [Avian](https://github.com/Jondolf/avian) physics engine design

## Documentation

- [BepuPhysics v2 Documentation](https://github.com/bepu/bepuphysics2/tree/master/Documentation)
