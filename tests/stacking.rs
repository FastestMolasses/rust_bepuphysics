//! Scenario (a): a static ground plus a stack of boxes and spheres.
//!
//! Steps the simulation long enough for the stack to settle, then checks the three things that
//! catch nearly every solver / integrator / contact regression: no NaNs, nothing tunneled through
//! the ground, and the pile actually came to rest instead of jittering or exploding.

#![feature(portable_simd)]
#![feature(generic_const_exprs)]

mod common;

use common::*;
use glam::Vec3;
use rust_bepuphysics::physics::collidables::box_shape::Box as BoxShape;
use rust_bepuphysics::physics::collidables::sphere::Sphere;
use rust_bepuphysics::physics::solve_description::SolveDescription;

const DT: f32 = 1.0 / 60.0;

/// Ground top surface sits at y = 0: a 1-unit-thick slab centered at y = -0.5.
fn build_stack(sim: &mut TestSim) {
    sim.add_static_box(60.0, 1.0, 60.0, Vec3::new(0.0, -0.5, 0.0));

    // A tower of boxes, each resting on the one below with a small gap so they settle rather
    // than start interpenetrating.
    let box_shape = BoxShape::new(1.0, 1.0, 1.0);
    for i in 0..6 {
        sim.add_dynamic(
            &box_shape,
            Vec3::new(0.0, 0.55 + i as f32 * 1.02, 0.0),
            1.0,
        );
    }

    // Spheres dropped alongside, so sphere-box and sphere-sphere paths are exercised too.
    let sphere = Sphere::new(0.5);
    for i in 0..4 {
        sim.add_dynamic(
            &sphere,
            Vec3::new(3.0, 0.55 + i as f32 * 1.05, 0.0),
            1.0,
        );
    }

    // A second, offset tower so the broad phase has more than one island to deal with.
    for i in 0..4 {
        sim.add_dynamic(
            &box_shape,
            Vec3::new(-3.0, 0.55 + i as f32 * 1.02, 1.5),
            1.0,
        );
    }
}

#[test]
fn stack_settles_and_stays_finite() {
    let mut sim = TestSim::with_config(
        Vec3::new(0.0, -10.0, 0.0),
        SolveDescription::with_defaults(8, 1),
        1.0,
    );
    build_stack(&mut sim);

    // Check finiteness as we go, not just at the end: a NaN that appears early and is then
    // "washed out" by clamping would otherwise be invisible.
    for step in 0..240 {
        sim.step(DT);
        if step % 40 == 0 {
            assert_all_finite(&sim.snapshot(), &format!("step {step}"));
        }
    }

    let states = sim.snapshot();
    assert_all_finite(&states, "final");

    // The ground slab's top is at y = 0 and every dynamic body has a half-extent of 0.5, so a
    // resting body's center cannot legitimately go below ~0.5. -0.2 leaves generous room for
    // speculative-margin penetration while still catching real tunneling.
    assert_above(&states, -0.2, "final");

    // Settled: the whole pile should be essentially motionless after 4 seconds. Sleeping bodies
    // have their velocity zeroed, so this holds whether or not the islands slept.
    assert_at_rest(&states, 0.05, 0.15, "final");

    // The towers must still be towers. Boxes are 1 unit tall, so the topmost of the 6-box stack
    // has to be somewhere near y = 5.5; if the stack collapsed or sank it will be far below.
    let top_box = states[5];
    assert!(
        top_box.position.y > 4.5,
        "6-box tower collapsed or sank: top box at y = {} (expected > 4.5)",
        top_box.position.y
    );
    // And it should not have wandered off laterally.
    assert!(
        top_box.position.x.abs() < 1.0 && top_box.position.z.abs() < 1.0,
        "top box drifted laterally to {:?}",
        top_box.position
    );

    sim.dispose_and_check_pool();
}

#[test]
fn substepped_solver_also_settles() {
    // Same scene through the substepping path, which is a different code path in the solver
    // (`prepare_constraint_integration_responsibilities` / `integrate_after_substepping`).
    let mut sim = TestSim::with_config(
        Vec3::new(0.0, -10.0, 0.0),
        SolveDescription::with_defaults(4, 4),
        1.0,
    );
    build_stack(&mut sim);

    sim.step_n(240, DT);

    let states = sim.snapshot();
    assert_all_finite(&states, "substepped final");
    assert_above(&states, -0.2, "substepped final");
    assert_at_rest(&states, 0.05, 0.15, "substepped final");

    sim.dispose_and_check_pool();
}

#[test]
fn single_sphere_falls_and_rests_on_ground() {
    // The narrowest possible check on the contact constraint path: one sphere, one plane, a
    // known resting height.
    let mut sim = TestSim::new();
    sim.add_static_box(20.0, 1.0, 20.0, Vec3::new(0.0, -0.5, 0.0));
    let sphere = Sphere::new(0.5);
    let handle = sim.add_dynamic(&sphere, Vec3::new(0.0, 4.0, 0.0), 1.0);

    sim.step_n(180, DT);

    let s = sim.body_state(handle);
    assert!(s.is_finite(), "sphere state not finite: {s:?}");
    // Resting on a ground surface at y = 0 with radius 0.5, the center belongs at y ~ 0.5.
    assert!(
        (s.position.y - 0.5).abs() < 0.05,
        "sphere did not come to rest on the ground: y = {} (expected ~0.5)",
        s.position.y
    );
    assert!(
        s.speed() < 0.05,
        "sphere still moving at |v| = {}",
        s.speed()
    );

    sim.dispose_and_check_pool();
}
