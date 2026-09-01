# BepuPhysics for Rust

A line-by-line translation of [BepuPhysics v2](https://github.com/bepu/bepuphysics2), an extremely high performance physics engine.

## Features

- **High Performance**: SIMD-optimized collision detection and constraint solving
- **Multithreaded**: Lock-free task scheduling with automatic work distribution
- **Bevy Integration**: Optional plugin for seamless integration with the Bevy game engine
- **Spatial Queries**: filtered ray casts and shape sweeps

## Requirements

- **Rust nightly v1.95.0+** — requires `#![feature(portable_simd)]` and `#![feature(generic_const_exprs)]`
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
        BepuCollider::cuboid(200.0, 2.0, 200.0),
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

**Body tuning:**
- `LockedRotation` — locks rotation about chosen body-local axes by zeroing the matching rows and
  columns of the inverse inertia tensor. `LockedRotation::{X, Y, Z}` combine with `|`;
  `LockedRotation::UPRIGHT` (pitch + roll, yaw free) suits any prop that must not tip over, and
  `LockedRotation::ALL` locks rotation entirely.
- `SpeculativeMargin { minimum, maximum }` — how far ahead of a surface contacts are created
- `SleepThreshold(f32)` — squared-velocity sleep threshold; `SleepThreshold::NEVER` disables sleeping
- `QueryLayers(u32)` — layer mask used by spatial queries (query filtering only, not collision response)

**Resources:**
- `Gravity(Vec3)` — global gravity (default: `Vec3::new(0.0, -9.81, 0.0)`)
- `BepuConfig` — simulation settings (timestep, substeps, thread count, determinism)

## Spatial Queries

`BepuSpatialQuery` is a system parameter. It borrows the simulation *shared*, so several query
systems can run in parallel; scratch memory comes from a per-system `Local`. Schedule queries
outside `BepuSet::Step`.

```rust
fn shoot(
    mut spatial: BepuSpatialQuery,
    shooter: Single<(Entity, &Transform), With<Player>>,
) {
    let (entity, transform) = *shooter;

    // Self-exclusion matters: a ray starting inside your own collider hits you first, every time.
    let filter = QueryFilter::default().exclude(&[entity]);

    if let Some(hit) = spatial.ray_cast(transform.translation, *transform.forward(), 100.0, filter) {
        // hit.entity, hit.t, hit.point, hit.normal, hit.child_index, hit.is_dynamic
    }

    // Every hit along the ray, not just the closest.
    spatial.ray_cast_all(origin, direction, 100.0, filter, |hit| { /* ... */ });

    // Shape sweep.
    match spatial.sweep_shape(QueryShape::capsule(0.3, 1.0), position, Vec3::NEG_Y, 2.0, filter) {
        SweepResult::Hit(hit)                  => { /* hit.t, hit.point, hit.normal */ }
        SweepResult::StartPenetrating { .. }   => { /* already overlapping: no t, no normal */ }
        SweepResult::Miss                      => {}
    }
}
```

**`SweepResult::StartPenetrating` is not an edge case to fold away.** Bepu reports a sweep that
begins in contact through a separate callback that supplies no time of impact, no contact point,
and no normal, because none of them are defined. Treating it as a miss makes solid geometry
occasionally transparent; treating it as a hit with a zero normal pushes things in random
directions. It is a distinct variant here so you have to decide.

**Filtering:**
- `QueryFilter::default()` — hits everything
- `.exclude(&[entity, ...])` — never hit these entities
- `.layers(mask)` — only collidables whose `QueryLayers` mask shares a bit. Entities without a
  `QueryLayers` component belong to layer 0.
- `.statics_only()` / `.bodies_only()` — filter by mobility

Entity resolution branches on `CollidableReference::mobility()` before touching the handle: a
`BodyHandle` and a `StaticHandle` with the same raw value are unrelated objects.

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
```

The comparison benchmarks live in the separate `bepu-benchmarks` workspace member, so that their
heavy and platform-specific dependencies stay out of the library:

```bash
# Avian physics comparison
cargo +nightly run --release -p bepu-benchmarks --example rain_avian

# The same scene against the original C# BepuPhysics through FFI. Behind the `csharp` feature
# because its `bepuvy-sys` dependency links a prebuilt native library that fails to link on MSVC.
cargo +nightly run --release -p bepu-benchmarks --features csharp --example rain_csharp
```

## License

Licensed under the Apache License, Version 2.0, matching the original [BepuPhysics v2](https://github.com/bepu/bepuphysics2) license.

See [LICENSE.md](LICENSE.md) for the full license text.

## Credits

- **Original BepuPhysics** by Ross Nordby ([https://github.com/bepu/bepuphysics2](https://github.com/bepu/bepuphysics2))
- **Bevy plugin** — ECS-friendly wrapper inspired by the [Avian](https://github.com/Jondolf/avian) physics engine design

## Documentation

- [BepuPhysics v2 Documentation](https://github.com/bepu/bepuphysics2/tree/master/Documentation)
