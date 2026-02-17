//! Bevy systems that bridge ECS components and the BepuPhysics simulation.

use bevy::prelude::*;

use crate::physics::body_description::BodyDescription;
use crate::physics::body_properties::{BodyVelocity, RigidPose};
use crate::physics::body_reference::BodyReference;
use crate::physics::collidables::box_shape::Box as PhysicsBox;
use crate::physics::collidables::capsule::Capsule;
use crate::physics::collidables::cylinder::Cylinder;
use crate::physics::collidables::sphere::Sphere;
use crate::physics::static_description::StaticDescription;

use super::callbacks::{DefaultNarrowPhaseCallbacks, DefaultPoseCallbacks};
use super::components::*;
use super::resources::*;

use crate::physics::simulation::Simulation;
use crate::physics::solve_description::SolveDescription;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::ThreadDispatcher;

// ---------------------------------------------------------------------------
// Startup: create the simulation
// ---------------------------------------------------------------------------

/// Creates the [`BepuSimulation`] resource from [`BepuConfig`] and [`Gravity`].
pub(crate) fn initialize_simulation(mut commands: Commands, config: Res<BepuConfig>, gravity: Res<Gravity>) {
    let thread_count = config.thread_count.unwrap_or_else(|| {
        std::thread::available_parallelism()
            .map(|n| n.get() as u32)
            .unwrap_or(4)
    });

    let mut buffer_pool = Box::new(BufferPool::new(131072, 16));
    let pool_ptr: *mut BufferPool = &mut *buffer_pool;

    let pose_callbacks = DefaultPoseCallbacks::new(
        glam::Vec3::new(gravity.0.x, gravity.0.y, gravity.0.z),
        config.linear_damping,
        config.angular_damping,
    );

    let narrow_callbacks = DefaultNarrowPhaseCallbacks::new(
        config.default_friction,
        config.max_recovery_velocity,
        config.spring_frequency,
        config.spring_damping_ratio,
    );

    let mut simulation = unsafe {
        Simulation::create(
            pool_ptr,
            narrow_callbacks,
            pose_callbacks,
            SolveDescription::with_defaults(config.velocity_iterations as i32, config.substep_count as i32),
            None,
            None,
            None,
        )
    };

    simulation.deterministic = config.deterministic;

    let dispatcher = ThreadDispatcher::new(thread_count as i32, 16384);

    commands.insert_resource(BepuSimulation {
        simulation,
        buffer_pool,
        dispatcher,
        entity_to_body: Default::default(),
        body_to_entity: Default::default(),
        entity_to_static: Default::default(),
        static_to_entity: Default::default(),
        config: config.clone(),
    });
}

// ---------------------------------------------------------------------------
// Body addition
// ---------------------------------------------------------------------------

/// Adds newly spawned entities with [`RigidBody`] + [`BepuCollider`] to the simulation.
pub(crate) fn add_new_bodies(
    mut sim: ResMut<BepuSimulation>,
    query: Query<
        (
            Entity,
            &RigidBody,
            &BepuCollider,
            &Transform,
            Option<&Mass>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        (Added<RigidBody>, Without<BepuBodyHandle>, Without<BepuStaticHandle>),
    >,
    mut commands: Commands,
) {
    for (entity, rb, collider, transform, mass, lin_vel, ang_vel) in query.iter() {
        let pos = glam::Vec3::new(transform.translation.x, transform.translation.y, transform.translation.z);
        let rot = glam::Quat::from_xyzw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        );
        let pose = RigidPose::new(pos, rot);
        let mass_val = mass.map(|m| m.0).unwrap_or(1.0);

        let lin = lin_vel.map(|v| glam::Vec3::new(v.0.x, v.0.y, v.0.z)).unwrap_or(glam::Vec3::ZERO);
        let ang = ang_vel.map(|v| glam::Vec3::new(v.0.x, v.0.y, v.0.z)).unwrap_or(glam::Vec3::ZERO);
        let velocity = BodyVelocity::new(lin, ang);

        unsafe {
            let shapes = sim.shapes_mut();

            match rb {
                RigidBody::Dynamic => {
                    let handle = match collider {
                        BepuCollider::Sphere { radius } => {
                            let shape = Sphere::new(*radius);
                            let desc = BodyDescription::create_convex_dynamic(pose, velocity, mass_val, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Box { width, height, depth } => {
                            let shape = PhysicsBox::new(*width, *height, *depth);
                            let desc = BodyDescription::create_convex_dynamic(pose, velocity, mass_val, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Capsule { radius, length } => {
                            let shape = Capsule::new(*radius, *length);
                            let desc = BodyDescription::create_convex_dynamic(pose, velocity, mass_val, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Cylinder { radius, length } => {
                            let shape = Cylinder::new(*radius, *length);
                            let desc = BodyDescription::create_convex_dynamic(pose, velocity, mass_val, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                    };

                    sim.entity_to_body.insert(entity, handle);
                    sim.body_to_entity.insert(handle, entity);
                    commands.entity(entity).insert(BepuBodyHandle(handle));
                }
                RigidBody::Kinematic => {
                    let handle = match collider {
                        BepuCollider::Sphere { radius } => {
                            let shape = Sphere::new(*radius);
                            let desc = BodyDescription::create_convex_kinematic(pose, velocity, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Box { width, height, depth } => {
                            let shape = PhysicsBox::new(*width, *height, *depth);
                            let desc = BodyDescription::create_convex_kinematic(pose, velocity, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Capsule { radius, length } => {
                            let shape = Capsule::new(*radius, *length);
                            let desc = BodyDescription::create_convex_kinematic(pose, velocity, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                        BepuCollider::Cylinder { radius, length } => {
                            let shape = Cylinder::new(*radius, *length);
                            let desc = BodyDescription::create_convex_kinematic(pose, velocity, shapes, &shape);
                            sim.bodies_mut().add(&desc)
                        }
                    };

                    sim.entity_to_body.insert(entity, handle);
                    sim.body_to_entity.insert(handle, entity);
                    commands.entity(entity).insert(BepuBodyHandle(handle));
                }
                RigidBody::Static => {
                    let shape_index = match collider {
                        BepuCollider::Sphere { radius } => shapes.add(&Sphere::new(*radius)),
                        BepuCollider::Box { width, height, depth } => {
                            shapes.add(&PhysicsBox::new(*width, *height, *depth))
                        }
                        BepuCollider::Capsule { radius, length } => {
                            shapes.add(&Capsule::new(*radius, *length))
                        }
                        BepuCollider::Cylinder { radius, length } => {
                            shapes.add(&Cylinder::new(*radius, *length))
                        }
                    };

                    let desc = StaticDescription::with_discrete(pose, shape_index);
                    let handle = sim.statics_mut().add(&desc);

                    sim.entity_to_static.insert(entity, handle);
                    sim.static_to_entity.insert(handle, entity);
                    commands.entity(entity).insert(BepuStaticHandle(handle));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Body removal
// ---------------------------------------------------------------------------

/// Removes bodies from the simulation when their entities are despawned.
pub(crate) fn remove_despawned_bodies(
    mut sim: ResMut<BepuSimulation>,
    mut removed_bodies: RemovedComponents<BepuBodyHandle>,
    mut removed_statics: RemovedComponents<BepuStaticHandle>,
) {
    for entity in removed_bodies.read() {
        if let Some(handle) = sim.entity_to_body.remove(&entity) {
            unsafe {
                if sim.bodies().body_exists(handle) {
                    sim.bodies_mut().remove(handle);
                }
            }
            sim.body_to_entity.remove(&handle);
        }
    }

    for entity in removed_statics.read() {
        if let Some(handle) = sim.entity_to_static.remove(&entity) {
            unsafe {
                if sim.statics().static_exists(handle) {
                    sim.statics_mut().remove(handle);
                }
            }
            sim.static_to_entity.remove(&handle);
        }
    }
}

// ---------------------------------------------------------------------------
// Pre-step: sync user changes → simulation
// ---------------------------------------------------------------------------

/// Pushes user-side velocity changes into the Bepu simulation before stepping.
pub(crate) fn sync_velocities_to_bepu(
    mut sim: ResMut<BepuSimulation>,
    query: Query<
        (&BepuBodyHandle, &LinearVelocity, &AngularVelocity),
        Or<(Changed<LinearVelocity>, Changed<AngularVelocity>)>,
    >,
) {
    for (bh, lin_vel, ang_vel) in query.iter() {
        unsafe {
            let bodies = &mut *sim.simulation.bodies;
            if !bodies.body_exists(bh.0) {
                continue;
            }
            let body_ref = BodyReference::new(bh.0, bodies);
            let vel = body_ref.velocity_mut();
            vel.linear = glam::Vec3::new(lin_vel.0.x, lin_vel.0.y, lin_vel.0.z);
            vel.angular = glam::Vec3::new(ang_vel.0.x, ang_vel.0.y, ang_vel.0.z);
        }
    }
}

/// Pushes user-side Transform changes into Bepu poses for kinematic bodies.
pub(crate) fn sync_transforms_to_bepu(
    mut sim: ResMut<BepuSimulation>,
    query: Query<
        (&BepuBodyHandle, &RigidBody, &Transform),
        Changed<Transform>,
    >,
) {
    for (bh, rb, transform) in query.iter() {
        // Only push transform changes for kinematic bodies.
        // Dynamic body transforms are written by the sim, so we shouldn't fight it.
        if *rb != RigidBody::Kinematic {
            continue;
        }
        unsafe {
            let bodies = &mut *sim.simulation.bodies;
            if !bodies.body_exists(bh.0) {
                continue;
            }
            let body_ref = BodyReference::new(bh.0, bodies);
            let pose = body_ref.pose_mut();
            pose.position = glam::Vec3::new(transform.translation.x, transform.translation.y, transform.translation.z);
            pose.orientation = glam::Quat::from_xyzw(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            );
        }
    }
}

/// Copies current Gravity resource and damping config into the pose integrator
/// callbacks before stepping.
pub(crate) fn update_callback_data(
    sim: ResMut<BepuSimulation>,
    gravity: Res<Gravity>,
) {
    // The pose integrator is a PoseIntegrator<DefaultPoseCallbacks> behind a
    // dyn IPoseIntegrator. We stored the gravity/damping in DefaultPoseCallbacks
    // at creation. To update it we need to access the concrete type.
    //
    // For now we rely on the fact that the simulation was created with
    // DefaultPoseCallbacks and access the callbacks through the pose integrator's
    // raw pointer.
    //
    // This is the "default callbacks" path. When users provide custom callbacks
    // via the escape hatch, this system is not registered.
    unsafe {
        use crate::physics::pose_integrator::PoseIntegrator;
        let pi = sim.simulation.pose_integrator as *mut PoseIntegrator<DefaultPoseCallbacks>;
        let callbacks = &mut (*pi).callbacks;
        callbacks.gravity = glam::Vec3::new(gravity.0.x, gravity.0.y, gravity.0.z);
        callbacks.linear_damping = sim.config.linear_damping;
        callbacks.angular_damping = sim.config.angular_damping;
    }
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

/// Advances the simulation by the fixed timestep.
pub(crate) fn step_simulation(mut sim: ResMut<BepuSimulation>, time: Res<Time<Fixed>>) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    // We need to split-borrow: simulation + dispatcher separately.
    // Since dispatcher is inside BepuSimulation, we use a raw pointer trick.
    let dispatcher_ptr: *const ThreadDispatcher = &sim.dispatcher;
    let dispatcher_ref: &ThreadDispatcher = unsafe { &*dispatcher_ptr };
    sim.simulation.timestep(dt, Some(dispatcher_ref));
}

// ---------------------------------------------------------------------------
// Post-step: simulation → ECS writeback
// ---------------------------------------------------------------------------

/// Reads dynamic body poses from the simulation and writes them back to [`Transform`].
pub(crate) fn sync_bepu_to_transforms(
    sim: Res<BepuSimulation>,
    mut query: Query<(&BepuBodyHandle, &RigidBody, &mut Transform)>,
) {
    let bodies = unsafe { &*sim.simulation.bodies };

    for (bh, rb, mut transform) in query.iter_mut() {
        // Don't overwrite kinematic transforms — users control those.
        if *rb == RigidBody::Kinematic {
            continue;
        }

        if !bodies.body_exists(bh.0) {
            continue;
        }

        let mut desc = std::mem::MaybeUninit::<BodyDescription>::uninit();
        unsafe {
            bodies.get_description(bh.0, &mut *desc.as_mut_ptr());
            let desc = desc.assume_init();
            transform.translation.x = desc.pose.position.x;
            transform.translation.y = desc.pose.position.y;
            transform.translation.z = desc.pose.position.z;
            transform.rotation.x = desc.pose.orientation.x;
            transform.rotation.y = desc.pose.orientation.y;
            transform.rotation.z = desc.pose.orientation.z;
            transform.rotation.w = desc.pose.orientation.w;
        }
    }
}

/// Writes simulated velocities back to [`LinearVelocity`] and [`AngularVelocity`] components.
pub(crate) fn sync_bepu_to_velocities(
    sim: Res<BepuSimulation>,
    mut query: Query<(&BepuBodyHandle, &mut LinearVelocity, &mut AngularVelocity)>,
) {
    let bodies = unsafe { &*sim.simulation.bodies };

    for (bh, mut lin_vel, mut ang_vel) in query.iter_mut() {
        if !bodies.body_exists(bh.0) {
            continue;
        }

        let body_ref = BodyReference::new(bh.0, sim.simulation.bodies);
        let vel = body_ref.velocity();
        lin_vel.0 = Vec3::new(vel.linear.x, vel.linear.y, vel.linear.z);
        ang_vel.0 = Vec3::new(vel.angular.x, vel.angular.y, vel.angular.z);
    }
}
