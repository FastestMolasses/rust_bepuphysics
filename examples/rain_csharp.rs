//! Rain (C# BepuPhysics) — Same scene as `rain.rs` but using the original C#
//! BepuPhysics library through FFI bindings (bepuvy-sys).
//!
//! Use this to compare physics performance between the Rust port and the
//! original C# engine.
//!
//! Controls:
//!   Hold SPACE  — spawn random objects
//!   R           — clear all objects
//!   Mouse drag  — orbit camera
//!   Scroll      — zoom

use bepuvy_sys::bepu::{
    bodies::*, collisions::*, constraints::SpringSettings, functions::*, handles::*,
    interop_math::*, pose_integration::*, shapes::Box as BepuBox, shapes::Capsule as BepuCapsule,
    shapes::Cylinder as BepuCylinder, shapes::Sphere as BepuSphere, statics::*,
    SimulationAllocationSizes, SolveDescription,
};
use bevy::color::palettes::css;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use rand::Rng;

// ============================================================================
// Quaternion helpers (bepuvy-sys only exposes Quaternion::new / identity)
// ============================================================================

/// Quaternion from rotation around the Z axis (matches glam's from_rotation_z).
fn quat_from_rotation_z(angle: f32) -> Quaternion {
    let (s, c) = (angle * 0.5).sin_cos();
    Quaternion::new(0.0, 0.0, s, c)
}

/// Quaternion from rotation around the X axis (matches glam's from_rotation_x).
fn quat_from_rotation_x(angle: f32) -> Quaternion {
    let (s, c) = (angle * 0.5).sin_cos();
    Quaternion::new(s, 0.0, 0.0, c)
}

/// Quaternion from Euler angles XYZ (matches glam's from_euler(XYZ, x, y, z)).
fn quat_from_euler(x: f32, y: f32, z: f32) -> Quaternion {
    let (sx, cx) = (x * 0.5).sin_cos();
    let (sy, cy) = (y * 0.5).sin_cos();
    let (sz, cz) = (z * 0.5).sin_cos();
    Quaternion::new(
        sx * cy * cz + cx * sy * sz,
        cx * sy * cz - sx * cy * sz,
        cx * cy * sz + sx * sy * cz,
        cx * cy * cz - sx * sy * sz,
    )
}

// ============================================================================
// Physics callbacks
// ============================================================================

unsafe extern "C" fn integrate_velocity_cb(
    _simulation: SimulationHandle,
    _body_index: i32,
    _position: Vector3,
    _orientation: Quaternion,
    _local_inertia: BodyInertia,
    _worker_index: i32,
    dt: f32,
    velocity: *mut BodyVelocity,
) {
    // Gravity = (0, -20, 0) — same as the Rust version
    (*velocity).linear.x += 0.0;
    (*velocity).linear.y += -20.0 * dt;
    (*velocity).linear.z += 0.0;
}

fn create_narrow_phase_callbacks() -> NarrowPhaseCallbacks {
    unsafe extern "C" fn allow_contact_generation(
        _sim: SimulationHandle,
        _worker: i32,
        _a: CollidableReference,
        _b: CollidableReference,
        _margin: *mut f32,
    ) -> bool {
        true
    }

    unsafe extern "C" fn allow_children(
        _sim: SimulationHandle,
        _worker: i32,
        _pair: CollidablePair,
        _a: i32,
        _b: i32,
    ) -> bool {
        true
    }

    unsafe extern "C" fn configure_convex(
        _sim: SimulationHandle,
        _worker: i32,
        _pair: CollidablePair,
        _manifold: *mut ConvexContactManifold,
        material: *mut PairMaterialProperties,
    ) -> bool {
        (*material).friction_coefficient = 0.7;
        (*material).maximum_recovery_velocity = 10.0;
        (*material).contact_spring_settings = SpringSettings::new(30.0, 1.0);
        true
    }

    unsafe extern "C" fn configure_nonconvex(
        _sim: SimulationHandle,
        _worker: i32,
        _pair: CollidablePair,
        _manifold: *mut NonconvexContactManifold,
        material: *mut PairMaterialProperties,
    ) -> bool {
        (*material).friction_coefficient = 0.7;
        (*material).maximum_recovery_velocity = 10.0;
        (*material).contact_spring_settings = SpringSettings::new(30.0, 1.0);
        true
    }

    unsafe extern "C" fn configure_child(
        _sim: SimulationHandle,
        _worker: i32,
        _pair: CollidablePair,
        _a: i32,
        _b: i32,
        _manifold: *mut ConvexContactManifold,
    ) -> bool {
        true
    }

    NarrowPhaseCallbacks {
        initialize_function: None,
        dispose_function: None,
        allow_contact_generation_function: Some(allow_contact_generation),
        allow_contact_generation_between_children_function: Some(allow_children),
        configure_convex_contact_manifold_function: Some(configure_convex),
        configure_nonconvex_contact_manifold_function: Some(configure_nonconvex),
        configure_child_contact_manifold_function: Some(configure_child),
    }
}

fn create_pose_integrator_callbacks() -> PoseIntegratorCallbacks {
    PoseIntegratorCallbacks {
        angular_integration_mode: AngularIntegrationMode::Nonconserving,
        allow_substeps_for_unconstrained_bodies: false,
        integrate_velocity_for_kinematics: false,
        use_scalar_callback: true,
        initialize: None,
        prepare_for_integration: None,
        integrate_velocity_scalar: Some(integrate_velocity_cb),
        integrate_velocity_simd128: None,
        integrate_velocity_simd256: None,
    }
}

// ============================================================================
// Resources and components
// ============================================================================

#[derive(Resource)]
struct PhysicsWorld {
    buffer_pool: BufferPoolHandle,
    thread_dispatcher: ThreadDispatcherHandle,
    simulation: SimulationHandle,
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        unsafe {
            DestroyThreadDispatcher(self.thread_dispatcher);
            DestroySimulation(self.simulation);
            DestroyBufferPool(self.buffer_pool);
            Destroy();
        }
    }
}

#[derive(Resource, Default)]
struct SpawnedBodies {
    handles: Vec<BodyHandle>,
    count: usize,
}

#[derive(Component)]
struct RigidBody {
    handle: BodyHandle,
}

#[derive(Component)]
struct Spawned;

#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    yaw: f32,
    pitch: f32,
    target: Vec3,
}

#[derive(Clone, Copy)]
enum ShapeKind {
    Box { hw: f32, hh: f32, hd: f32 },
    Sphere { radius: f32 },
    Capsule { radius: f32, length: f32 },
    Cylinder { radius: f32, length: f32 },
}

// ============================================================================
// Scene constants (must match rain.rs exactly)
// ============================================================================

const GROUND_SIZE: f32 = 200.0;
const WALL_HEIGHT: f32 = 2.0;
const WALL_THICKNESS: f32 = 1.0;

struct RampDef {
    pos: Vector3,
    rot: Quaternion,
}

fn ramp_defs() -> [RampDef; 3] {
    [
        RampDef {
            pos: Vector3::new(-8.0, 3.0, 0.0),
            rot: quat_from_rotation_z(0.3),
        },
        RampDef {
            pos: Vector3::new(8.0, 5.0, 4.0),
            rot: quat_from_rotation_z(-0.25),
        },
        RampDef {
            pos: Vector3::new(0.0, 7.0, -6.0),
            rot: quat_from_rotation_x(0.2),
        },
    ]
}

fn wall_defs() -> [(Vector3, f32, f32, f32); 4] {
    [
        (
            Vector3::new(0.0, WALL_HEIGHT / 2.0, GROUND_SIZE / 2.0),
            GROUND_SIZE,
            WALL_HEIGHT,
            WALL_THICKNESS,
        ),
        (
            Vector3::new(0.0, WALL_HEIGHT / 2.0, -GROUND_SIZE / 2.0),
            GROUND_SIZE,
            WALL_HEIGHT,
            WALL_THICKNESS,
        ),
        (
            Vector3::new(GROUND_SIZE / 2.0, WALL_HEIGHT / 2.0, 0.0),
            WALL_THICKNESS,
            WALL_HEIGHT,
            GROUND_SIZE,
        ),
        (
            Vector3::new(-GROUND_SIZE / 2.0, WALL_HEIGHT / 2.0, 0.0),
            WALL_THICKNESS,
            WALL_HEIGHT,
            GROUND_SIZE,
        ),
    ]
}

// ============================================================================
// Helper: create a new C# simulation
// ============================================================================

fn create_csharp_simulation() -> PhysicsWorld {
    unsafe { Initialize() };

    let buffer_pool = unsafe { CreateBufferPool(131072, 16) };
    let thread_dispatcher = unsafe { CreateThreadDispatcher(6, 16384) };

    let simulation = unsafe {
        CreateSimulation(
            buffer_pool,
            create_narrow_phase_callbacks(),
            create_pose_integrator_callbacks(),
            SolveDescription {
                velocity_iteration_count: 4,
                substep_count: 1,
                fallback_batch_threshold: 128,
                velocity_iteration_scheduler: None,
            },
            SimulationAllocationSizes {
                bodies: 8192,
                statics: 128,
                islands: 2048,
                shapes_per_type: 128,
                constraints: 4096,
                constraints_per_type_batch: 128,
                constraint_count_per_body_estimate: 8,
            },
        )
    };

    PhysicsWorld {
        buffer_pool,
        thread_dispatcher,
        simulation,
    }
}

/// Add statics (ground, walls, ramps) to the C# simulation.
fn add_statics(sim: SimulationHandle) {
    // Ground
    let ground_shape = unsafe { AddBox(sim, BepuBox::new(GROUND_SIZE, 1.0, GROUND_SIZE)) };
    let ground_desc = StaticDescription::create_with_position_orientation_discrete(
        Vector3::new(0.0, -0.5, 0.0),
        Quaternion::identity(),
        ground_shape,
    );
    unsafe { AddStatic(sim, ground_desc) };

    // Walls
    for (pos, w, h, d) in wall_defs() {
        let shape = unsafe { AddBox(sim, BepuBox::new(w, h, d)) };
        let desc = StaticDescription::create_with_position_orientation_discrete(
            pos,
            Quaternion::identity(),
            shape,
        );
        unsafe { AddStatic(sim, desc) };
    }

    // Ramps
    for ramp in ramp_defs() {
        let shape = unsafe { AddBox(sim, BepuBox::new(12.0, 0.5, 6.0)) };
        let desc = StaticDescription::create_with_position_orientation_discrete(
            ramp.pos,
            ramp.rot,
            shape,
        );
        unsafe { AddStatic(sim, desc) };
    }
}

// ============================================================================
// Setup
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ---- Physics world ----
    let physics = create_csharp_simulation();
    add_statics(physics.simulation);

    // ---- Visuals: ground ----
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(GROUND_SIZE, 1.0, GROUND_SIZE))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.27, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    // ---- Visuals: walls ----
    for (pos, w, h, d) in wall_defs() {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(w, h, d))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.35, 0.4),
                ..default()
            })),
            Transform::from_xyz(pos.x, pos.y, pos.z),
        ));
    }

    // ---- Visuals: ramps ----
    for ramp in ramp_defs() {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(12.0, 0.5, 6.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.4, 0.45),
                ..default()
            })),
            Transform {
                translation: Vec3::new(ramp.pos.x, ramp.pos.y, ramp.pos.z),
                rotation: Quat::from_xyzw(ramp.rot.x, ramp.rot.y, ramp.rot.z, ramp.rot.w),
                ..default()
            },
        ));
    }

    commands.insert_resource(physics);
    commands.insert_resource(SpawnedBodies::default());

    // ---- Camera (same as rain.rs) ----
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
            "Hold SPACE to rain objects\nR to clear\nMouse drag to orbit, scroll to zoom\n\n[C# BepuPhysics via FFI]",
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
    physics: Res<PhysicsWorld>,
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

        // Random orientation
        let euler_x = rng.random_range(0.0..std::f32::consts::TAU);
        let euler_y = rng.random_range(0.0..std::f32::consts::TAU);
        let euler_z = rng.random_range(0.0..std::f32::consts::TAU);
        let rot = quat_from_euler(euler_x, euler_y, euler_z);

        let vel = BodyVelocity::new(
            Vector3::new(
                rng.random_range(-2.0..2.0),
                rng.random_range(-5.0..-1.0),
                rng.random_range(-2.0..2.0),
            ),
            Vector3::new(
                rng.random_range(-3.0..3.0),
                rng.random_range(-3.0..3.0),
                rng.random_range(-3.0..3.0),
            ),
        );

        // Pick random shape (same ranges as rain.rs)
        let shape_kind: u32 = rng.random_range(0..4);
        let (shape_kind_data, shape_index, mass) = match shape_kind {
            0 => {
                let hw = rng.random_range(0.3..1.2f32);
                let hh = rng.random_range(0.3..1.2f32);
                let hd = rng.random_range(0.3..1.2f32);
                let shape =
                    unsafe { AddBox(physics.simulation, BepuBox::new(hw * 2.0, hh * 2.0, hd * 2.0)) };
                (ShapeKind::Box { hw, hh, hd }, shape, hw * hh * hd * 4.0)
            }
            1 => {
                let r = rng.random_range(0.3..1.0f32);
                let shape = unsafe { AddSphere(physics.simulation, BepuSphere { radius: r }) };
                (ShapeKind::Sphere { radius: r }, shape, r * r * r * 6.0)
            }
            2 => {
                let r = rng.random_range(0.2..0.6f32);
                let l = rng.random_range(0.5..2.0f32);
                let shape = unsafe {
                    AddCapsule(
                        physics.simulation,
                        BepuCapsule {
                            radius: r,
                            half_length: l * 0.5,
                        },
                    )
                };
                (
                    ShapeKind::Capsule {
                        radius: r,
                        length: l,
                    },
                    shape,
                    r * r * l * 8.0,
                )
            }
            _ => {
                let r = rng.random_range(0.3..0.8f32);
                let l = rng.random_range(0.4..1.5f32);
                let shape = unsafe {
                    AddCylinder(
                        physics.simulation,
                        BepuCylinder {
                            radius: r,
                            half_length: l * 0.5,
                        },
                    )
                };
                (
                    ShapeKind::Cylinder {
                        radius: r,
                        length: l,
                    },
                    shape,
                    r * r * l * 6.0,
                )
            }
        };

        let inertia = unsafe {
            match shape_kind_data {
                ShapeKind::Box { hw, hh, hd } => {
                    ComputeBoxInertia(BepuBox::new(hw * 2.0, hh * 2.0, hd * 2.0), mass)
                }
                ShapeKind::Sphere { radius } => {
                    ComputeSphereInertia(BepuSphere { radius }, mass)
                }
                ShapeKind::Capsule { radius, length } => ComputeCapsuleInertia(
                    BepuCapsule {
                        radius,
                        half_length: length * 0.5,
                    },
                    mass,
                ),
                ShapeKind::Cylinder { radius, length } => ComputeCylinderInertia(
                    BepuCylinder {
                        radius,
                        half_length: length * 0.5,
                    },
                    mass,
                ),
            }
        };

        let rot_x = rot.x;
        let rot_y = rot.y;
        let rot_z = rot.z;
        let rot_w = rot.w;

        let pose = RigidPose::new(Vector3::new(x, y, z), rot);
        let collidable = CollidableDescription::passive(shape_index);
        let activity = BodyActivityDescription::new(0.01, 32);
        let body_desc = BodyDescription::create_dynamic(pose, vel, inertia, collidable, activity);
        let handle = unsafe { AddBody(physics.simulation, body_desc) };

        // Random color (same palette as rain.rs)
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

        let mesh_handle = match shape_kind_data {
            ShapeKind::Box { hw, hh, hd } => meshes.add(Cuboid::new(hw * 2.0, hh * 2.0, hd * 2.0)),
            ShapeKind::Sphere { radius } => meshes.add(bevy::math::primitives::Sphere::new(radius)),
            ShapeKind::Capsule { radius, length } => {
                meshes.add(bevy::math::primitives::Capsule3d::new(radius, length))
            }
            ShapeKind::Cylinder { radius, length } => {
                meshes.add(bevy::math::primitives::Cylinder::new(radius, length))
            }
        };

        commands.spawn((
            Mesh3d(mesh_handle),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform {
                translation: Vec3::new(x, y, z),
                rotation: Quat::from_xyzw(rot_x, rot_y, rot_z, rot_w),
                ..default()
            },
            RigidBody { handle },
            Spawned,
        ));

        spawned.handles.push(handle);
        spawned.count += 1;
    }
}

// ============================================================================
// Clear system — press R to remove all spawned objects
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

    // Remove all spawned body handles from the C# simulation.
    // Dropping the resource calls Destroy* via Drop impl, then we recreate.
    commands.remove_resource::<PhysicsWorld>();

    spawned.handles.clear();
    spawned.count = 0;

    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    // Recreate the physics world with the same statics
    let new_physics = create_csharp_simulation();
    add_statics(new_physics.simulation);
    commands.insert_resource(new_physics);

    info!("Reset simulation (disposed old, created new)");
}

// ============================================================================
// Physics step + transform sync
// ============================================================================

fn physics_step(physics: Res<PhysicsWorld>) {
    let dt = 1.0 / 60.0;
    unsafe {
        Timestep(physics.simulation, dt, physics.thread_dispatcher);
    }
}

fn sync_transforms(physics: Res<PhysicsWorld>, mut query: Query<(&RigidBody, &mut Transform)>) {
    for (body, mut transform) in query.iter_mut() {
        let dynamics = unsafe { GetBodyDynamics(physics.simulation, body.handle) };
        if dynamics.is_null() {
            continue;
        }
        unsafe {
            transform.translation = Vec3::new(
                (*dynamics).motion.pose.position.x,
                (*dynamics).motion.pose.position.y,
                (*dynamics).motion.pose.position.z,
            );
            transform.rotation = Quat::from_xyzw(
                (*dynamics).motion.pose.orientation.x,
                (*dynamics).motion.pose.orientation.y,
                (*dynamics).motion.pose.orientation.z,
                (*dynamics).motion.pose.orientation.w,
            );
        }
    }
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
// Camera orbit (identical to rain.rs)
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
                title: "Physics Rain [C# BepuPhysics via FFI]".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .add_systems(Startup, (setup, setup_counter))
        .add_systems(Update, (orbit_camera, update_counter, update_fps))
        .add_systems(
            FixedUpdate,
            (spawn_objects, clear_objects, physics_step, sync_transforms).chain(),
        )
        .run();
}
