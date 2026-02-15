//! Rain — Hold spacebar to shower random physics objects onto a floor.
//!
//! Objects include boxes, spheres, capsules, and cylinders with randomized sizes,
//! colors, and bounce. Release spacebar to stop spawning; objects stay and pile up.
//!
//! Controls:
//!   Hold SPACE  — spawn random objects
//!   R           — clear all objects
//!   Mouse drag  — orbit camera
//!   Scroll      — zoom

#![feature(portable_simd)]
#![feature(generic_const_exprs)]

use bevy::color::palettes::css;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use rand::Rng;

use rust_bepuphysics::glam as phys_glam;

// --- Physics crate imports ---
use rust_bepuphysics::physics::body_description::BodyDescription;
use rust_bepuphysics::physics::body_properties::{
    BodyInertiaWide, BodyVelocity, BodyVelocityWide, RigidPose,
};
use rust_bepuphysics::physics::collidables::box_shape::Box as PhysicsBox;
use rust_bepuphysics::physics::collidables::capsule::Capsule;
use rust_bepuphysics::physics::collidables::collidable_reference::CollidableReference;
use rust_bepuphysics::physics::collidables::cylinder::Cylinder;
use rust_bepuphysics::physics::collidables::shapes::Shapes;
use rust_bepuphysics::physics::collidables::sphere::Sphere;
use rust_bepuphysics::physics::collision_detection::contact_manifold::{
    ConvexContactManifold, IContactManifold,
};
use rust_bepuphysics::physics::collision_detection::narrow_phase_callbacks::{
    INarrowPhaseCallbacks, PairMaterialProperties,
};
use rust_bepuphysics::physics::collision_detection::pair_cache::CollidablePair;
use rust_bepuphysics::physics::constraints::spring_settings::SpringSettings;
use rust_bepuphysics::physics::handles::BodyHandle;
use rust_bepuphysics::physics::pose_integration::{
    AngularIntegrationMode, IPoseIntegratorCallbacks,
};
use rust_bepuphysics::physics::simulation::Simulation;
use rust_bepuphysics::physics::solve_description::SolveDescription;
use rust_bepuphysics::physics::static_description::StaticDescription;
use rust_bepuphysics::utilities::memory::buffer_pool::BufferPool;
use rust_bepuphysics::utilities::quaternion_wide::QuaternionWide;
use rust_bepuphysics::utilities::thread_dispatcher::ThreadDispatcher;
use rust_bepuphysics::utilities::vector::Vector;
use rust_bepuphysics::utilities::vector3_wide::Vector3Wide;

// ============================================================================
// Glam bridge
// ============================================================================

fn to_bevy_vec3(v: phys_glam::Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

fn to_bevy_quat(q: phys_glam::Quat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

// ============================================================================
// Physics callbacks
// ============================================================================

struct GravityCallbacks {
    gravity: phys_glam::Vec3,
}

impl IPoseIntegratorCallbacks for GravityCallbacks {
    fn angular_integration_mode(&self) -> AngularIntegrationMode {
        AngularIntegrationMode::Nonconserving
    }
    fn allow_substeps_for_unconstrained_bodies(&self) -> bool {
        false
    }
    fn integrate_velocity_for_kinematics(&self) -> bool {
        false
    }
    unsafe fn initialize(&mut self, _simulation: *mut u8) {}
    fn prepare_for_integration(&mut self, _dt: f32) {}

    fn integrate_velocity(
        &self,
        _body_indices: Vector<i32>,
        _position: Vector3Wide,
        _orientation: QuaternionWide,
        _local_inertia: BodyInertiaWide,
        _integration_mask: Vector<i32>,
        _worker_index: i32,
        dt: Vector<f32>,
        velocity: &mut BodyVelocityWide,
    ) {
        let gx = Vector::<f32>::splat(self.gravity.x);
        let gy = Vector::<f32>::splat(self.gravity.y);
        let gz = Vector::<f32>::splat(self.gravity.z);
        velocity.linear.x += gx * dt;
        velocity.linear.y += gy * dt;
        velocity.linear.z += gz * dt;
    }
}

/// Narrow phase callbacks with per-pair bounce variation.
/// Objects tagged as "bouncy" get high restitution.
struct BouncyNarrowPhaseCallbacks;

impl INarrowPhaseCallbacks for BouncyNarrowPhaseCallbacks {
    fn initialize(&mut self, _simulation: *mut Simulation) {}

    fn allow_contact_generation(
        &self,
        _worker_index: i32,
        _a: CollidableReference,
        _b: CollidableReference,
        _speculative_margin: &mut f32,
    ) -> bool {
        true
    }

    fn configure_contact_manifold<TManifold: IContactManifold>(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _manifold: &mut TManifold,
        pair_material: &mut PairMaterialProperties,
    ) -> bool {
        pair_material.friction_coefficient = 0.7;
        pair_material.maximum_recovery_velocity = 10.0;
        pair_material.spring_settings = SpringSettings::new(30.0, 1.0);
        true
    }

    fn allow_contact_generation_for_children(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
    ) -> bool {
        true
    }

    fn configure_child_contact_manifold(
        &self,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
        _manifold: &mut ConvexContactManifold,
    ) -> bool {
        true
    }

    fn dispose(&mut self) {}
}

// ============================================================================
// Bevy resources and components
// ============================================================================

#[derive(Resource)]
struct PhysicsWorld {
    simulation: Box<Simulation>,
    _buffer_pool: Box<BufferPool>,
}

impl Drop for PhysicsWorld {
    fn drop(&mut self) {
        unsafe {
            self.simulation.dispose();
            self._buffer_pool.clear();
        }
    }
}

unsafe impl Send for PhysicsWorld {}
unsafe impl Sync for PhysicsWorld {}

/// Separate resource for the thread dispatcher so we can borrow it independently
/// from PhysicsWorld (avoids split-borrow issues with ResMut).
#[derive(Resource)]
struct PhysicsDispatcher {
    dispatcher: ThreadDispatcher,
}

unsafe impl Send for PhysicsDispatcher {}
unsafe impl Sync for PhysicsDispatcher {}

/// Tracks spawned rigid bodies so we can clear them.
#[derive(Resource, Default)]
struct SpawnedBodies {
    handles: Vec<BodyHandle>,
    count: usize,
}

/// Component tagging a Bevy entity as a dynamic rigid body.
#[derive(Component)]
struct RigidBody {
    handle: BodyHandle,
}

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

/// Which shape type was spawned (for display).
#[derive(Clone, Copy)]
enum ShapeKind {
    Box { hw: f32, hh: f32, hd: f32 },
    Sphere { radius: f32 },
    Capsule { radius: f32, length: f32 },
    Cylinder { radius: f32, length: f32 },
}

// ============================================================================
// Setup
// ============================================================================

/// Creates a new physics world with default settings.
fn create_physics_world() -> PhysicsWorld {
    let mut buffer_pool = Box::new(BufferPool::new(131072, 16));
    let pool_ptr: *mut BufferPool = &mut *buffer_pool;

    let simulation = unsafe {
        Simulation::create(
            pool_ptr,
            BouncyNarrowPhaseCallbacks,
            GravityCallbacks {
                gravity: phys_glam::Vec3::new(0.0, -20.0, 0.0),
            },
            SolveDescription::with_defaults(4, 1),
            None,
            None,
            None,
        )
    };

    PhysicsWorld {
        simulation,
        _buffer_pool: buffer_pool,
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ---- Physics world ----
    let physics = create_physics_world();

    // ---- Ground plane ----
    let shapes = unsafe { &mut *(physics.simulation.shapes as *mut Shapes) };
    let ground_shape = shapes.add(&PhysicsBox::new(200.0, 1.0, 200.0));
    let ground_desc = StaticDescription::with_discrete(
        RigidPose::from_position(phys_glam::Vec3::new(0.0, -0.5, 0.0)),
        ground_shape,
    );
    unsafe {
        (&mut *physics.simulation.statics).add(&ground_desc);
    }

    // Ground visual — a slightly dark checkerboard-ish floor
    commands.spawn((
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
    let wall_positions_sizes: [(phys_glam::Vec3, f32, f32, f32); 4] = [
        // North wall (along X axis, at +Z)
        (
            phys_glam::Vec3::new(0.0, wall_height / 2.0, ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        // South wall (along X axis, at -Z)
        (
            phys_glam::Vec3::new(0.0, wall_height / 2.0, -ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        // East wall (along Z axis, at +X)
        (
            phys_glam::Vec3::new(ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
        // West wall (along Z axis, at -X)
        (
            phys_glam::Vec3::new(-ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
    ];

    for (pos, width, height, depth) in &wall_positions_sizes {
        let wall_shape = shapes.add(&PhysicsBox::new(*width, *height, *depth));
        let wall_desc =
            StaticDescription::with_discrete(RigidPose::from_position(*pos), wall_shape);
        unsafe {
            (&mut *physics.simulation.statics).add(&wall_desc);
        }

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(*width, *height, *depth))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.35, 0.4),
                ..default()
            })),
            Transform::from_xyz(pos.x, pos.y, pos.z),
        ));
    }

    // ---- Some angled ramps for fun bouncing ----
    let ramp_positions_rotations: [(phys_glam::Vec3, phys_glam::Quat); 3] = [
        (
            phys_glam::Vec3::new(-8.0, 3.0, 0.0),
            phys_glam::Quat::from_rotation_z(0.3),
        ),
        (
            phys_glam::Vec3::new(8.0, 5.0, 4.0),
            phys_glam::Quat::from_rotation_z(-0.25),
        ),
        (
            phys_glam::Vec3::new(0.0, 7.0, -6.0),
            phys_glam::Quat::from_rotation_x(0.2),
        ),
    ];
    for (pos, rot) in &ramp_positions_rotations {
        let ramp_shape = shapes.add(&PhysicsBox::new(12.0, 0.5, 6.0));
        let ramp_desc = StaticDescription::with_discrete(RigidPose::new(*pos, *rot), ramp_shape);
        unsafe {
            (&mut *physics.simulation.statics).add(&ramp_desc);
        }

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(12.0, 0.5, 6.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.4, 0.45),
                ..default()
            })),
            Transform {
                translation: to_bevy_vec3(*pos),
                rotation: to_bevy_quat(*rot),
                ..default()
            },
        ));
    }

    commands.insert_resource(physics);
    commands.insert_resource(SpawnedBodies::default());

    // ---- Thread dispatcher (match C# FFI example's thread count) ----
    let thread_count = std::thread::available_parallelism()
        .map(|n| n.get() as i32)
        .unwrap_or(6);
    commands.insert_resource(PhysicsDispatcher {
        dispatcher: ThreadDispatcher::new(thread_count, 16384),
    });

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
        Text::new("Hold SPACE to rain objects\nR to clear\nMouse drag to orbit, scroll to zoom"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// ============================================================================
// Spawning system — runs every fixed tick while space is held
// ============================================================================

/// Spawn a batch of random objects per physics tick while spacebar is held.
fn spawn_objects(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics: ResMut<PhysicsWorld>,
    mut spawned: ResMut<SpawnedBodies>,
) {
    if !keyboard.pressed(KeyCode::Space) {
        return;
    }

    let mut rng = rand::rng();
    let shapes = unsafe { &mut *(physics.simulation.shapes as *mut Shapes) };
    let bodies = unsafe { &mut *physics.simulation.bodies };

    // Spawn 10 objects per tick
    let batch_size = 10;
    for _ in 0..batch_size {
        // Random position in a region above the scene
        let x = rng.random_range(-15.0..15.0f32);
        let z = rng.random_range(-15.0..15.0f32);
        let y = rng.random_range(20.0..40.0f32);
        let pos = phys_glam::Vec3::new(x, y, z);

        // Random orientation
        let rot = phys_glam::Quat::from_euler(
            phys_glam::EulerRot::XYZ,
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
            rng.random_range(0.0..std::f32::consts::TAU),
        );

        // Small random downward + lateral velocity
        let vel = BodyVelocity::new(
            phys_glam::Vec3::new(
                rng.random_range(-2.0..2.0),
                rng.random_range(-5.0..-1.0),
                rng.random_range(-2.0..2.0),
            ),
            phys_glam::Vec3::new(
                rng.random_range(-3.0..3.0),
                rng.random_range(-3.0..3.0),
                rng.random_range(-3.0..3.0),
            ),
        );

        // Pick a random shape
        let shape_kind: u32 = rng.random_range(0..4);
        let (shape_kind_data, mass) = match shape_kind {
            0 => {
                // Box
                let hw = rng.random_range(0.3..1.2f32);
                let hh = rng.random_range(0.3..1.2f32);
                let hd = rng.random_range(0.3..1.2f32);
                (ShapeKind::Box { hw, hh, hd }, hw * hh * hd * 4.0)
            }
            1 => {
                // Sphere
                let r = rng.random_range(0.3..1.0f32);
                (ShapeKind::Sphere { radius: r }, r * r * r * 6.0)
            }
            2 => {
                // Capsule
                let r = rng.random_range(0.2..0.6f32);
                let l = rng.random_range(0.5..2.0f32);
                (
                    ShapeKind::Capsule {
                        radius: r,
                        length: l,
                    },
                    r * r * l * 8.0,
                )
            }
            _ => {
                // Cylinder
                let r = rng.random_range(0.3..0.8f32);
                let l = rng.random_range(0.4..1.5f32);
                (
                    ShapeKind::Cylinder {
                        radius: r,
                        length: l,
                    },
                    r * r * l * 6.0,
                )
            }
        };

        let pose = RigidPose::new(pos, rot);

        let handle = match shape_kind_data {
            ShapeKind::Box { hw, hh, hd } => {
                let shape = PhysicsBox::new(hw * 2.0, hh * 2.0, hd * 2.0);
                let desc = BodyDescription::create_convex_dynamic(pose, vel, mass, shapes, &shape);
                bodies.add(&desc)
            }
            ShapeKind::Sphere { radius } => {
                let shape = Sphere::new(radius);
                let desc = BodyDescription::create_convex_dynamic(pose, vel, mass, shapes, &shape);
                bodies.add(&desc)
            }
            ShapeKind::Capsule { radius, length } => {
                let shape = Capsule::new(radius, length);
                let desc = BodyDescription::create_convex_dynamic(pose, vel, mass, shapes, &shape);
                bodies.add(&desc)
            }
            ShapeKind::Cylinder { radius, length } => {
                let shape = Cylinder::new(radius, length);
                let desc = BodyDescription::create_convex_dynamic(pose, vel, mass, shapes, &shape);
                bodies.add(&desc)
            }
        };

        // Random color — some are bright "bouncy" colors
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
                translation: to_bevy_vec3(pos),
                rotation: to_bevy_quat(rot),
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

    spawned.handles.clear();
    spawned.count = 0;

    // Despawn Bevy entities for spawned objects.
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    // Properly dispose and recreate the physics world
    let physics = create_physics_world();
    let shapes = unsafe { &mut *(physics.simulation.shapes as *mut Shapes) };

    let ground_shape = shapes.add(&PhysicsBox::new(200.0, 1.0, 200.0));
    let ground_desc = StaticDescription::with_discrete(
        RigidPose::from_position(phys_glam::Vec3::new(0.0, -0.5, 0.0)),
        ground_shape,
    );
    unsafe {
        (&mut *physics.simulation.statics).add(&ground_desc);
    }

    // Re-add perimeter walls
    let wall_height = 2.0;
    let wall_thickness = 1.0;
    let ground_size = 200.0;
    let wall_positions_sizes: [(phys_glam::Vec3, f32, f32, f32); 4] = [
        (
            phys_glam::Vec3::new(0.0, wall_height / 2.0, ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        (
            phys_glam::Vec3::new(0.0, wall_height / 2.0, -ground_size / 2.0),
            ground_size,
            wall_height,
            wall_thickness,
        ),
        (
            phys_glam::Vec3::new(ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
        (
            phys_glam::Vec3::new(-ground_size / 2.0, wall_height / 2.0, 0.0),
            wall_thickness,
            wall_height,
            ground_size,
        ),
    ];

    for (pos, width, height, depth) in &wall_positions_sizes {
        let wall_shape = shapes.add(&PhysicsBox::new(*width, *height, *depth));
        let wall_desc =
            StaticDescription::with_discrete(RigidPose::from_position(*pos), wall_shape);
        unsafe {
            (&mut *physics.simulation.statics).add(&wall_desc);
        }
    }

    let ramp_positions_rotations: [(phys_glam::Vec3, phys_glam::Quat); 3] = [
        (
            phys_glam::Vec3::new(-8.0, 3.0, 0.0),
            phys_glam::Quat::from_rotation_z(0.3),
        ),
        (
            phys_glam::Vec3::new(8.0, 5.0, 4.0),
            phys_glam::Quat::from_rotation_z(-0.25),
        ),
        (
            phys_glam::Vec3::new(0.0, 7.0, -6.0),
            phys_glam::Quat::from_rotation_x(0.2),
        ),
    ];
    for (pos, rot) in &ramp_positions_rotations {
        let ramp_shape = shapes.add(&PhysicsBox::new(12.0, 0.5, 6.0));
        let ramp_desc = StaticDescription::with_discrete(RigidPose::new(*pos, *rot), ramp_shape);
        unsafe {
            (&mut *physics.simulation.statics).add(&ramp_desc);
        }
    }

    commands.insert_resource(physics);
    info!("Reset simulation (disposed old, created new)");
}

// ============================================================================
// Physics step + transform sync
// ============================================================================

fn physics_step(mut physics: ResMut<PhysicsWorld>, dispatcher: Res<PhysicsDispatcher>) {
    let dt = 1.0 / 60.0;
    physics
        .simulation
        .timestep(dt, Some(&dispatcher.dispatcher));
}

fn sync_transforms(physics: Res<PhysicsWorld>, mut query: Query<(&RigidBody, &mut Transform)>) {
    let bodies = unsafe { &*physics.simulation.bodies };
    for (body, mut transform) in query.iter_mut() {
        if !bodies.body_exists(body.handle) {
            continue;
        }
        let mut desc = std::mem::MaybeUninit::<BodyDescription>::uninit();
        unsafe {
            bodies.get_description(body.handle, &mut *desc.as_mut_ptr());
            let desc = desc.assume_init();
            transform.translation = to_bevy_vec3(desc.pose.position);
            transform.rotation = to_bevy_quat(desc.pose.orientation);
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
                title: "Physics Rain".into(),
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
