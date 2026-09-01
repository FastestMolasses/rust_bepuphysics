//! Bevy systems that bridge ECS components and the BepuPhysics simulation.

use bevy::prelude::*;

use crate::physics::body_description::{BodyActivityDescription, BodyDescription};
use crate::physics::body_properties::{BodyInertia, BodyVelocity, RigidPose};
use crate::physics::body_reference::BodyReference;
use crate::physics::collidables::box_shape::Box as PhysicsBox;
use crate::physics::collidables::capsule::Capsule;
use crate::physics::collidables::collidable_description::CollidableDescription;
use crate::physics::collidables::cylinder::Cylinder;
use crate::physics::collidables::shape::IConvexShape;
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
pub(crate) fn initialize_simulation(
    mut commands: Commands,
    config: Res<BepuConfig>,
    gravity: Res<Gravity>,
) {
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
            SolveDescription::with_defaults(
                config.velocity_iterations as i32,
                config.substep_count as i32,
            ),
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
        body_layers: Default::default(),
        static_layers: Default::default(),
        shape_registry: Default::default(),
        config: config.clone(),
    });
}

// ---------------------------------------------------------------------------
// Body addition
// ---------------------------------------------------------------------------

/// Adds newly spawned entities with [`RigidBody`] + [`BepuCollider`] to the simulation.
#[allow(clippy::type_complexity)]
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
            Option<&LockedRotation>,
            Option<&SpeculativeMargin>,
            Option<&SleepThreshold>,
            Option<&QueryLayers>,
        ),
        // Deliberately *not* `Added<RigidBody>`. `Added` is a one-shot: an entity that has a
        // RigidBody but is not yet addable — no BepuCollider yet, no Transform yet, spawned before
        // the simulation existed — consumes the signal on the tick it appears and is then never
        // reconsidered, leaving a permanently inert entity with no diagnostic. The absence of a
        // handle component is the real predicate, it is archetype-filtered (so this query is empty
        // in the steady state and costs nothing to iterate), and it makes body creation
        // self-healing: whatever was missing gets picked up on the tick it arrives.
        (Without<BepuBodyHandle>, Without<BepuStaticHandle>),
    >,
    mut commands: Commands,
) {
    for (
        entity,
        rb,
        collider,
        transform,
        mass,
        lin_vel,
        ang_vel,
        locked_rotation,
        margin,
        sleep_threshold,
        layers,
    ) in query.iter()
    {
        // Belt and braces: the handle components are inserted through `Commands`, so within a single
        // run of this system the query filter alone cannot tell us whether we already added this
        // entity. The maps can.
        if sim.entity_to_body.contains_key(&entity) || sim.entity_to_static.contains_key(&entity) {
            continue;
        }

        let pos = glam::Vec3::new(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        );
        let rot = glam::Quat::from_xyzw(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        );
        let pose = RigidPose::new(pos, rot);
        let mass_val = mass.map(|m| m.0).unwrap_or(1.0);

        let lin = lin_vel
            .map(|v| glam::Vec3::new(v.0.x, v.0.y, v.0.z))
            .unwrap_or(glam::Vec3::ZERO);
        let ang = ang_vel
            .map(|v| glam::Vec3::new(v.0.x, v.0.y, v.0.z))
            .unwrap_or(glam::Vec3::ZERO);
        let velocity = BodyVelocity::new(lin, ang);

        // One shared, reference-counted `Shapes` entry per distinct collider description. The
        // `BepuShapeIndex` inserted below is what releases it again when the entity goes away;
        // without it every spawned collider leaked a shape slot forever.
        let shape_index = sim.acquire_shape(collider);

        unsafe {
            match rb {
                RigidBody::Dynamic | RigidBody::Kinematic => {
                    let kinematic = *rb == RigidBody::Kinematic;
                    // `BodyDescription::create_convex_*` would add a *fresh* shape per body, which is
                    // exactly the leak being fixed, so the shape-derived defaults are computed here
                    // against the already-shared index instead. The resulting description is
                    // identical to what those helpers produce.
                    let (local_inertia, activity) = shape_derived_body_defaults(
                        collider,
                        if kinematic { None } else { Some(mass_val) },
                    );
                    let mut desc = BodyDescription {
                        pose,
                        velocity,
                        local_inertia,
                        collidable: CollidableDescription::from(shape_index),
                        activity,
                    };

                    // Per-entity overrides applied on top of the shape-derived defaults.
                    if let Some(locked) = locked_rotation {
                        // A zeroed row/column pair is an infinite inertia about that axis: no
                        // torque can rotate the body there. The mask is in the body's local frame,
                        // which is the frame `local_inertia` is expressed in, so it applies
                        // directly.
                        locked.apply_to_inverse_inertia(
                            &mut desc.local_inertia.inverse_inertia_tensor,
                        );
                        // The tensor only governs the *response* to torque. A spin handed to the
                        // body up front would survive it untouched, so project that too.
                        desc.velocity.angular =
                            locked.apply_to_angular_velocity(pose.orientation, desc.velocity.angular);
                    }
                    if let Some(margin) = margin {
                        desc.collidable.minimum_speculative_margin = margin.minimum;
                        desc.collidable.maximum_speculative_margin = margin.maximum;
                    }
                    if let Some(sleep) = sleep_threshold {
                        desc.activity.sleep_threshold = sleep.0;
                    }

                    let handle = sim.bodies_mut().add(&desc);

                    sim.entity_to_body.insert(entity, handle);
                    sim.body_to_entity.insert(handle, entity);
                    if let Some(layers) = layers {
                        sim.body_layers.insert(handle, layers.0);
                    }
                    commands
                        .entity(entity)
                        .insert((BepuBodyHandle(handle), BepuShapeIndex(shape_index)));
                }
                RigidBody::Static => {
                    let desc = StaticDescription::with_discrete(pose, shape_index);
                    let handle = sim.statics_mut().add(&desc);

                    sim.entity_to_static.insert(entity, handle);
                    sim.static_to_entity.insert(handle, entity);
                    if let Some(layers) = layers {
                        sim.static_layers.insert(handle, layers.0);
                    }
                    commands
                        .entity(entity)
                        .insert((BepuStaticHandle(handle), BepuShapeIndex(shape_index)));
                }
            }
        }
    }
}

/// Inertia and sleep thresholds Bepu derives from a shape, computed without adding the shape.
///
/// `mass` is `None` for kinematic bodies, which get a zeroed (infinite) inertia.
fn shape_derived_body_defaults(
    collider: &BepuCollider,
    mass: Option<f32>,
) -> (BodyInertia, BodyActivityDescription) {
    #[inline]
    fn defaults<TShape: IConvexShape>(
        shape: &TShape,
        mass: Option<f32>,
    ) -> (BodyInertia, BodyActivityDescription) {
        let inertia = match mass {
            Some(mass) => shape.compute_inertia(mass),
            None => BodyInertia::default(),
        };
        (inertia, BodyDescription::get_default_activity(shape))
    }

    match collider {
        BepuCollider::Sphere { radius } => defaults(&Sphere::new(*radius), mass),
        BepuCollider::Box {
            width,
            height,
            depth,
        } => defaults(&PhysicsBox::new(*width, *height, *depth), mass),
        BepuCollider::Capsule { radius, length } => defaults(&Capsule::new(*radius, *length), mass),
        BepuCollider::Cylinder { radius, length } => {
            defaults(&Cylinder::new(*radius, *length), mass)
        }
    }
}

// ---------------------------------------------------------------------------
// Body removal
// ---------------------------------------------------------------------------

/// Tears down everything queued by the removal hooks: bodies, statics, and shapes.
///
/// Ordering inside this single system is deliberate and is the reason it is one system rather than
/// two: bodies and statics are removed first, and shapes are released last so nothing in the
/// simulation still references a freed shape.
///
/// The queue is drained unconditionally. It must never be gated on a run condition, on app state, or
/// on the simulation existing, because a skipped drain is a permanent leak rather than a delayed
/// one — the entries simply accumulate until the next drain.
pub(crate) fn apply_pending_removals(
    sim: Option<ResMut<BepuSimulation>>,
    mut queue: ResMut<BepuRemovalQueue>,
) {
    if queue.is_empty() {
        return;
    }

    let Some(mut sim) = sim else {
        // No simulation yet (or already torn down). Nothing was ever registered, so drop the queued
        // work rather than letting it pile up.
        queue.bodies.clear();
        queue.statics.clear();
        queue.shapes.clear();
        return;
    };

    // 1. Bodies and statics.
    for (entity, handle) in queue.bodies.drain(..) {
        // The hook recorded the handle, so removal works even if the entity is long gone. The map
        // is still authoritative for "did this plugin create that body".
        if sim.entity_to_body.remove(&entity).is_some() {
            unsafe {
                if sim.bodies().body_exists(handle) {
                    sim.bodies_mut().remove(handle);
                }
            }
            sim.body_to_entity.remove(&handle);
            sim.body_layers.remove(&handle);
        }
    }

    for (entity, handle) in queue.statics.drain(..) {
        if sim.entity_to_static.remove(&entity).is_some() {
            unsafe {
                if sim.statics().static_exists(handle) {
                    sim.statics_mut().remove(handle);
                }
            }
            sim.static_to_entity.remove(&handle);
            sim.static_layers.remove(&handle);
        }
    }

    // 2. Shapes last: releasing one may free the underlying `Shapes` entry, which must not happen
    //    while a body or static still points at it.
    for index in queue.shapes.drain(..) {
        sim.release_shape(index);
    }
}

// ---------------------------------------------------------------------------
// Pre-step: sync user changes → simulation
// ---------------------------------------------------------------------------

/// Pushes user-side velocity changes into the Bepu simulation before stepping.
pub(crate) fn sync_velocities_to_bepu(
    mut sim: ResMut<BepuSimulation>,
    query: Query<
        (
            &BepuBodyHandle,
            &LinearVelocity,
            &AngularVelocity,
            Option<&LockedRotation>,
        ),
        Or<(Changed<LinearVelocity>, Changed<AngularVelocity>)>,
    >,
) {
    for (bh, lin_vel, ang_vel, locked_rotation) in query.iter() {
        unsafe {
            let bodies = &mut *sim.simulation.bodies;
            if !bodies.body_exists(bh.0) {
                continue;
            }
            let body_ref = BodyReference::new(bh.0, bodies);

            let target_linear = glam::Vec3::new(lin_vel.0.x, lin_vel.0.y, lin_vel.0.z);
            let mut target_angular = glam::Vec3::new(ang_vel.0.x, ang_vel.0.y, ang_vel.0.z);

            if let Some(locked) = locked_rotation {
                target_angular =
                    locked.apply_to_angular_velocity(body_ref.pose().orientation, target_angular);
            }

            // The post-step writeback (`sync_bepu_to_velocities`) rewrites these
            // components every frame, so Bevy's change detection marks *every*
            // body as changed every frame — `Changed` alone can't tell a genuine
            // user edit from our own writeback. Compare against the body's current
            // velocity and skip when nothing actually changed; otherwise we'd wake
            // every body every frame below and sleeping would never happen.
            {
                let current = body_ref.velocity();
                if current.linear == target_linear && current.angular == target_angular {
                    continue;
                }
            }

            if !body_ref.awake() {
                body_ref.set_awake(true);
            }

            let vel = body_ref.velocity_mut();
            vel.linear = target_linear;
            vel.angular = target_angular;
        }
    }
}

/// Pushes user-side Transform changes into Bepu poses.
///
/// # Dynamic bodies are included, and the equality guard is what makes that safe
///
/// This used to skip everything that was not [`RigidBody::Kinematic`], on the reasoning that the
/// simulation writes dynamic transforms and we should not fight it. The consequence was that a
/// dynamic body could not be TELEPORTED at all: a `Transform` written from `FixedUpdate` was
/// silently discarded, with no warning and no error. That is not a narrow gap — it is every
/// hand-written step-up, every "put the player back on the checkpoint", and every effect that wants
/// to resolve a position rather than a velocity, and each one fails by doing precisely nothing.
///
/// The reason to skip dynamics was never really about dynamics. It was that
/// [`sync_bepu_to_transforms`] rewrites `Transform` on every body every tick, so Bevy marks all of
/// them `Changed` and the filter alone cannot tell our own writeback from a genuine edit. The fix is
/// the one [`sync_velocities_to_bepu`] already uses: compare against the body's current pose and
/// skip when nothing actually moved. A writeback round-trip compares equal and costs two float
/// compares; a real edit differs and is applied.
pub(crate) fn sync_transforms_to_bepu(
    mut sim: ResMut<BepuSimulation>,
    query: Query<(&BepuBodyHandle, &RigidBody, &Transform), Changed<Transform>>,
) {
    for (bh, rb, transform) in query.iter() {
        if *rb == RigidBody::Static {
            continue;
        }
        unsafe {
            let bodies = &mut *sim.simulation.bodies;
            if !bodies.body_exists(bh.0) {
                continue;
            }
            let body_ref = BodyReference::new(bh.0, bodies);

            let target_position = glam::Vec3::new(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            );
            let target_orientation = glam::Quat::from_xyzw(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            );

            {
                let current = body_ref.pose();
                if current.position == target_position && current.orientation == target_orientation
                {
                    continue;
                }
            }

            // A body that has been moved has to be awake, or the change sits in the sleeping set
            // unobserved and the next writeback quietly reverts it.
            if !body_ref.awake() {
                body_ref.set_awake(true);
            }

            let pose = body_ref.pose_mut();
            pose.position = target_position;
            pose.orientation = target_orientation;
        }
    }
}

/// Copies current Gravity resource and damping config into the pose integrator
/// callbacks before stepping.
pub(crate) fn update_callback_data(sim: ResMut<BepuSimulation>, gravity: Res<Gravity>) {
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
///
/// Two things this deliberately does *not* do, both of which used to make every physics entity's
/// `Transform` register as `Changed` on every single tick — which then cascaded into transform
/// propagation and render extraction for the whole scene, settled or not:
///
/// * It does not call `Bodies::get_description`, which copies an entire [`BodyDescription`]
///   (inertia, collidable, activity) to read two fields. The pose is read directly instead.
/// * It does not write unconditionally. Sleeping bodies are skipped outright, and an awake body
///   whose pose is bit-identical to the component leaves the component untouched, so change
///   detection stays clean for anything that has come to rest but not yet fallen asleep.
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

        let body_ref = BodyReference::new(bh.0, sim.simulation.bodies);
        // A sleeping body cannot have moved since the last tick, so its Transform is already
        // correct. Bepu only wakes a body when something can change its pose.
        if !body_ref.awake() {
            continue;
        }

        let pose = body_ref.pose();
        let translation = Vec3::new(pose.position.x, pose.position.y, pose.position.z);
        let rotation = Quat::from_xyzw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        );

        // Reading through `Mut` does not mark the component changed; only `DerefMut` does.
        if transform.translation != translation {
            transform.translation = translation;
        }
        if transform.rotation != rotation {
            transform.rotation = rotation;
        }
    }
}

/// Writes simulated velocities back to [`LinearVelocity`] and [`AngularVelocity`] components.
///
/// Only when the value actually differs: writing unconditionally marked every physics entity's
/// velocity components as `Changed` every tick, which made the `Changed` filter on
/// [`sync_velocities_to_bepu`] match every body every tick and defeated its purpose.
///
/// Unlike [`sync_bepu_to_transforms`] this does not skip sleeping bodies: a body's velocity is not
/// necessarily what it was when the body fell asleep (Bepu leaves the last integrated value in
/// place), and callers use these components to decide whether something is moving.
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
        let linear = LinearVelocity(Vec3::new(vel.linear.x, vel.linear.y, vel.linear.z));
        let angular = AngularVelocity(Vec3::new(vel.angular.x, vel.angular.y, vel.angular.z));
        lin_vel.set_if_neq(linear);
        ang_vel.set_if_neq(angular);
    }
}
