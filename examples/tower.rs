//! Tower — a circular tower of cubes that you can demolish by throwing balls.
//!
//! Click (left mouse button) to throw a ball from the camera toward the tower.
//!
//! Controls:
//!   Left click       — throw a ball
//!   Space            — start / pause physics
//!   R                — reset the scene
//!   1-5              — select preset (100, 500, 1k, 2k, 5k cubes)
//!   Right-drag       — orbit camera
//!   Scroll           — zoom

#![feature(portable_simd)]
#![feature(generic_const_exprs)]

use bevy::color::palettes::css;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use rust_bepuphysics::glam as phys_glam;

// --- Physics crate imports ---
use rust_bepuphysics::physics::body_description::BodyDescription;
use rust_bepuphysics::physics::body_properties::{
    BodyInertiaWide, BodyVelocity, BodyVelocityWide, RigidPose,
};
use rust_bepuphysics::physics::collidables::box_shape::Box as PhysicsBox;
use rust_bepuphysics::physics::collidables::collidable_reference::CollidableReference;
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

/// Convert physics glam Vec3 -> Bevy Vec3
fn to_bevy_vec3(v: phys_glam::Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Convert physics glam Quat -> Bevy Quat
fn to_bevy_quat(q: phys_glam::Quat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

// ============================================================================
// Physics callbacks
// ============================================================================

struct GravityCallbacks {
    gravity: phys_glam::Vec3,
    linear_damping: f32,
    angular_damping: f32,
    linear_damping_dt: f32,
    angular_damping_dt: f32,
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
    fn prepare_for_integration(&mut self, dt: f32) {
        self.linear_damping_dt = (1.0 - self.linear_damping).clamp(0.0, 1.0).powf(dt);
        self.angular_damping_dt = (1.0 - self.angular_damping).clamp(0.0, 1.0).powf(dt);
    }

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
        let lin_damp = Vector::<f32>::splat(self.linear_damping_dt);
        let ang_damp = Vector::<f32>::splat(self.angular_damping_dt);
        velocity.linear.x = (velocity.linear.x + gx * dt) * lin_damp;
        velocity.linear.y = (velocity.linear.y + gy * dt) * lin_damp;
        velocity.linear.z = (velocity.linear.z + gz * dt) * lin_damp;
        velocity.angular.x = velocity.angular.x * ang_damp;
        velocity.angular.y = velocity.angular.y * ang_damp;
        velocity.angular.z = velocity.angular.z * ang_damp;
    }
}

struct DemoNarrowPhaseCallbacks;

impl INarrowPhaseCallbacks for DemoNarrowPhaseCallbacks {
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
        pair_material.friction_coefficient = 1.0;
        pair_material.maximum_recovery_velocity = 2.0;
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

/// Separate resource for the thread dispatcher so we can borrow it independently.
#[derive(Resource)]
struct PhysicsDispatcher {
    dispatcher: ThreadDispatcher,
}

unsafe impl Send for PhysicsDispatcher {}
unsafe impl Sync for PhysicsDispatcher {}

#[derive(Component)]
struct RigidBody {
    handle: BodyHandle,
}

/// Marker for entities that should be despawned on reset.
#[derive(Component)]
struct PhysicsEntity;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Default, States)]
enum PhysicsState {
    Running,
    #[default]
    Paused,
}

#[derive(Resource)]
struct TowerConfig {
    cube_count: usize,
}

impl Default for TowerConfig {
    fn default() -> Self {
        Self { cube_count: 1_000 }
    }
}

#[derive(Component)]
struct OrbitCamera {
    distance: f32,
    yaw: f32,
    pitch: f32,
    target: Vec3,
}

#[derive(Component)]
struct StatsText;

#[derive(Component)]
struct FpsText;

// ============================================================================
// Physics world creation
// ============================================================================

fn create_physics_world() -> PhysicsWorld {
    let mut buffer_pool = Box::new(BufferPool::new(131072, 16));
    let pool_ptr: *mut BufferPool = &mut *buffer_pool;

    let simulation = unsafe {
        Simulation::create(
            pool_ptr,
            DemoNarrowPhaseCallbacks,
            GravityCallbacks {
                gravity: phys_glam::Vec3::new(0.0, -20.0, 0.0),
                linear_damping: 0.03,
                angular_damping: 0.03,
                linear_damping_dt: 1.0,
                angular_damping_dt: 1.0,
            },
            SolveDescription::with_defaults(8, 1),
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

// ============================================================================
// Circular tower spawning
// ============================================================================

fn spawn_circular_tower(
    commands: &mut Commands,
    physics: &mut PhysicsWorld,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    cube_count: usize,
) {
    let shapes = unsafe { &mut *(physics.simulation.shapes as *mut Shapes) };

    // --- Ground ---
    let ground_shape = shapes.add(&PhysicsBox::new(200.0, 1.0, 200.0));
    let ground_desc = StaticDescription::with_discrete(
        RigidPose::from_position(phys_glam::Vec3::new(0.0, -0.5, 0.0)),
        ground_shape,
    );
    unsafe {
        (&mut *physics.simulation.statics).add(&ground_desc);
    }

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(200.0, 1.0, 200.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: css::DARK_GREEN.into(),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.5, 0.0),
        PhysicsEntity,
    ));

    // --- Circular tower ---
    let cube_size = 1.0f32;
    let cube_mesh = meshes.add(Cuboid::new(cube_size, cube_size, cube_size));
    let box_shape = PhysicsBox::new(cube_size, cube_size, cube_size);

    // Determine tower dimensions from cube_count.
    // Build rings of cubes; each ring has `cubes_per_ring` cubes.
    let radius = 8.0f32;
    let circumference = 2.0 * std::f32::consts::PI * radius;
    let cubes_per_ring = (circumference / (cube_size * 1.05)).floor().max(8.0) as usize;
    let num_layers = (cube_count as f32 / cubes_per_ring as f32).ceil().max(1.0) as usize;

    // Alternate layer colors for visual distinction.
    let color_a = materials.add(StandardMaterial {
        base_color: css::TOMATO.into(),
        ..default()
    });
    let color_b = materials.add(StandardMaterial {
        base_color: css::SANDY_BROWN.into(),
        ..default()
    });

    let mut placed = 0usize;
    let layer_height = cube_size * 1.02;

    for layer in 0..num_layers {
        if placed >= cube_count {
            break;
        }
        let y = layer as f32 * layer_height + cube_size * 0.5;
        // Stagger alternate layers by half a cube angle for stability.
        let angle_offset = if layer % 2 == 0 {
            0.0
        } else {
            std::f32::consts::PI / cubes_per_ring as f32
        };
        let mat = if layer % 2 == 0 {
            color_a.clone()
        } else {
            color_b.clone()
        };

        for i in 0..cubes_per_ring {
            if placed >= cube_count {
                break;
            }
            let angle =
                angle_offset + 2.0 * std::f32::consts::PI * i as f32 / cubes_per_ring as f32;
            let x = radius * angle.cos();
            let z = radius * angle.sin();

            // Orient cube to face center (tangent to ring).
            let rot = phys_glam::Quat::from_rotation_y(-angle);

            let desc = BodyDescription::create_convex_dynamic_no_velocity(
                RigidPose::new(phys_glam::Vec3::new(x, y, z), rot),
                1.0,
                shapes,
                &box_shape,
            );
            let handle = unsafe { (&mut *physics.simulation.bodies).add(&desc) };

            commands.spawn((
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(mat.clone()),
                Transform::from_xyz(x, y, z).with_rotation(to_bevy_quat(rot)),
                RigidBody { handle },
                PhysicsEntity,
            ));
            placed += 1;
        }
    }

    info!(
        "Spawned {} cubes in circular tower ({} per ring, {} layers)",
        placed, cubes_per_ring, num_layers
    );
}

// ============================================================================
// Setup
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<TowerConfig>,
) {
    let mut physics = create_physics_world();
    spawn_circular_tower(
        &mut commands,
        &mut physics,
        &mut meshes,
        &mut materials,
        config.cube_count,
    );
    commands.insert_resource(physics);

    // Thread dispatcher
    let thread_count = std::thread::available_parallelism()
        .map(|n| n.get() as i32)
        .unwrap_or(6);
    commands.insert_resource(PhysicsDispatcher {
        dispatcher: ThreadDispatcher::new(thread_count, 16384),
    });

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 25.0, 30.0).looking_at(Vec3::new(0.0, 8.0, 0.0), Vec3::Y),
        OrbitCamera {
            distance: 45.0,
            yaw: std::f32::consts::FRAC_PI_4 * 5.0,
            pitch: 0.45,
            target: Vec3::new(0.0, 8.0, 0.0),
        },
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.3, 0.0)),
    ));

    // HUD
    commands.spawn((
        Text::new(format!(
            "SPACE=start/pause  R=reset  1-5=preset  LClick=throw ball\nCubes: {}  |  Physics: paused",
            config.cube_count
        )),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        StatsText,
    ));
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

// ============================================================================
// Ball throwing
// ============================================================================

fn throw_ball(
    mouse_button: Res<ButtonInput<MouseButton>>,
    camera_query: Query<&Transform, With<Camera3d>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut physics: ResMut<PhysicsWorld>,
) {
    if !mouse_button.just_pressed(MouseButton::Left) {
        return;
    }

    let Ok(camera_tf) = camera_query.single() else {
        return;
    };

    let shapes = unsafe { &mut *(physics.simulation.shapes as *mut Shapes) };
    let ball_radius = 1.5f32;
    let ball_shape = Sphere::new(ball_radius);

    let spawn_pos = camera_tf.translation;
    let direction = camera_tf.forward().as_vec3();
    let speed = 80.0f32;

    let desc = BodyDescription::create_convex_dynamic(
        RigidPose::from_position(phys_glam::Vec3::new(spawn_pos.x, spawn_pos.y, spawn_pos.z)),
        BodyVelocity::new(
            phys_glam::Vec3::new(
                direction.x * speed,
                direction.y * speed,
                direction.z * speed,
            ),
            phys_glam::Vec3::ZERO,
        ),
        5.0,
        shapes,
        &ball_shape,
    );
    let handle = unsafe { (&mut *physics.simulation.bodies).add(&desc) };

    commands.spawn((
        Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(ball_radius))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: css::DODGER_BLUE.into(),
            ..default()
        })),
        Transform::from_translation(spawn_pos),
        RigidBody { handle },
        PhysicsEntity,
    ));
}

// ============================================================================
// Physics + sync
// ============================================================================

fn physics_step(mut physics: ResMut<PhysicsWorld>, dispatcher: Res<PhysicsDispatcher>) {
    physics
        .simulation
        .timestep(1.0 / 60.0, Some(&dispatcher.dispatcher));
}

fn sync_transforms(physics: Res<PhysicsWorld>, mut query: Query<(&RigidBody, &mut Transform)>) {
    for (body, mut transform) in query.iter_mut() {
        let mut desc = std::mem::MaybeUninit::<BodyDescription>::uninit();
        unsafe {
            let bodies = &*physics.simulation.bodies;
            if !bodies.body_exists(body.handle) {
                continue;
            }
            bodies.get_description(body.handle, &mut *desc.as_mut_ptr());
            let desc = desc.assume_init();
            transform.translation = to_bevy_vec3(desc.pose.position);
            transform.rotation = to_bevy_quat(desc.pose.orientation);
        }
    }
}

// ============================================================================
// UI + controls
// ============================================================================

fn toggle_physics(
    keyboard: Res<ButtonInput<KeyCode>>,
    state: Res<State<PhysicsState>>,
    mut next_state: ResMut<NextState<PhysicsState>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        match state.get() {
            PhysicsState::Running => next_state.set(PhysicsState::Paused),
            PhysicsState::Paused => next_state.set(PhysicsState::Running),
        }
    }
}

fn select_preset(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut config: ResMut<TowerConfig>,
    mut reset_events: MessageWriter<ResetEvent>,
) {
    let presets = [
        (KeyCode::Digit1, 100),
        (KeyCode::Digit2, 500),
        (KeyCode::Digit3, 1_000),
        (KeyCode::Digit4, 2_000),
        (KeyCode::Digit5, 5_000),
    ];
    for (key, count) in presets {
        if keyboard.just_pressed(key) {
            config.cube_count = count;
            reset_events.write(ResetEvent);
            return;
        }
    }
    if keyboard.just_pressed(KeyCode::KeyR) {
        reset_events.write(ResetEvent);
    }
}

#[derive(Message)]
struct ResetEvent;

fn handle_reset(
    mut commands: Commands,
    mut reset_events: MessageReader<ResetEvent>,
    existing: Query<Entity, With<PhysicsEntity>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<TowerConfig>,
    mut next_state: ResMut<NextState<PhysicsState>>,
) {
    if reset_events.read().next().is_none() {
        return;
    }
    for entity in existing.iter() {
        commands.entity(entity).despawn();
    }

    let mut physics = create_physics_world();
    spawn_circular_tower(
        &mut commands,
        &mut physics,
        &mut meshes,
        &mut materials,
        config.cube_count,
    );
    commands.insert_resource(physics);
    next_state.set(PhysicsState::Paused);
}

fn update_stats_text(
    config: Res<TowerConfig>,
    state: Res<State<PhysicsState>>,
    mut query: Query<&mut Text, With<StatsText>>,
) {
    let status = match state.get() {
        PhysicsState::Running => "running",
        PhysicsState::Paused => "paused",
    };
    for mut text in query.iter_mut() {
        **text = format!(
            "SPACE=start/pause  R=reset  1-5=preset  LClick=throw ball\nCubes: {}  |  Physics: {}",
            config.cube_count, status
        );
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

fn orbit_camera(
    mouse_button: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_scroll: Res<AccumulatedMouseScroll>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let Ok((mut orbit, mut transform)) = query.single_mut() else {
        return;
    };

    // Use right-drag for orbit so left-click can throw balls.
    if mouse_button.pressed(MouseButton::Right) {
        let delta = mouse_motion.delta;
        orbit.yaw -= delta.x * 0.005;
        orbit.pitch = (orbit.pitch - delta.y * 0.005).clamp(-1.4, 1.4);
    }

    let scroll = mouse_scroll.delta.y;
    if scroll != 0.0 {
        orbit.distance = (orbit.distance - scroll * 3.0).clamp(5.0, 200.0);
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
                title: "Circular Tower — Click to Throw Balls".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .init_state::<PhysicsState>()
        .init_resource::<TowerConfig>()
        .add_message::<ResetEvent>()
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                toggle_physics,
                select_preset,
                orbit_camera,
                update_stats_text,
                update_fps,
                handle_reset,
                throw_ball,
            ),
        )
        .add_systems(
            FixedUpdate,
            (physics_step, sync_transforms)
                .chain()
                .run_if(in_state(PhysicsState::Running)),
        )
        .run();
}
