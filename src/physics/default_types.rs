// Translated from BepuPhysics/DefaultTypes.cs

use crate::physics::collision_detection::narrow_phase::NarrowPhase;
use crate::physics::collision_detection::sweep_task_registry::SweepTaskRegistry;
use crate::physics::solver::Solver;

// Re-import all the constraint description types.
use crate::physics::constraints::angular_axis_gear_motor::AngularAxisGearMotor;
use crate::physics::constraints::angular_axis_motor::AngularAxisMotor;
use crate::physics::constraints::angular_hinge::AngularHinge;
use crate::physics::constraints::angular_motor::AngularMotor;
use crate::physics::constraints::angular_servo::AngularServo;
use crate::physics::constraints::angular_swivel_hinge::AngularSwivelHinge;
use crate::physics::constraints::area_constraint::AreaConstraint;
use crate::physics::constraints::ball_socket::BallSocket;
use crate::physics::constraints::ball_socket_motor::BallSocketMotor;
use crate::physics::constraints::ball_socket_servo::BallSocketServo;
use crate::physics::constraints::center_distance_constraint::CenterDistanceConstraint;
use crate::physics::constraints::center_distance_limit::CenterDistanceLimit;
use crate::physics::constraints::distance_limit::DistanceLimit;
use crate::physics::constraints::distance_servo::DistanceServo;
use crate::physics::constraints::hinge::Hinge;
use crate::physics::constraints::linear_axis_limit::LinearAxisLimit;
use crate::physics::constraints::linear_axis_motor::LinearAxisMotor;
use crate::physics::constraints::linear_axis_servo::LinearAxisServo;
use crate::physics::constraints::one_body_angular_motor::OneBodyAngularMotor;
use crate::physics::constraints::one_body_angular_servo::OneBodyAngularServo;
use crate::physics::constraints::one_body_linear_motor::OneBodyLinearMotor;
use crate::physics::constraints::one_body_linear_servo::OneBodyLinearServo;
use crate::physics::constraints::point_on_line_servo::PointOnLineServo;
use crate::physics::constraints::swing_limit::SwingLimit;
use crate::physics::constraints::swivel_hinge::SwivelHinge;
use crate::physics::constraints::twist_limit::TwistLimit;
use crate::physics::constraints::twist_motor::TwistMotor;
use crate::physics::constraints::twist_servo::TwistServo;
use crate::physics::constraints::volume_constraint::VolumeConstraint;
use crate::physics::constraints::weld::Weld;

// Contact constraint description types (for solver registration).
use crate::physics::constraints::contact::contact_convex_descriptions::{
    Contact1OneBody, Contact1TwoBody, Contact2OneBody, Contact2TwoBody, Contact3OneBody,
    Contact3TwoBody, Contact4OneBody, Contact4TwoBody,
};
use crate::physics::constraints::contact::contact_nonconvex_descriptions::{
    Contact2Nonconvex, Contact2NonconvexOneBody, Contact3Nonconvex, Contact3NonconvexOneBody,
    Contact4Nonconvex, Contact4NonconvexOneBody,
};

// Contact constraint accessor types (for narrow phase registration).
use crate::physics::collision_detection::contact_constraint_accessor::{
    Contact1OneBodyAccessor, Contact1TwoBodyAccessor, Contact2NonconvexOneBodyAccessor,
    Contact2NonconvexTwoBodyAccessor, Contact2OneBodyAccessor, Contact2TwoBodyAccessor,
    Contact3NonconvexOneBodyAccessor, Contact3NonconvexTwoBodyAccessor, Contact3OneBodyAccessor,
    Contact3TwoBodyAccessor, Contact4NonconvexOneBodyAccessor, Contact4NonconvexTwoBodyAccessor,
    Contact4OneBodyAccessor, Contact4TwoBodyAccessor,
};

/// Helper to register the default types within a simulation instance.
pub struct DefaultTypes;

impl DefaultTypes {
    /// Registers the set of constraints that are packaged in the engine.
    pub fn register_defaults(solver: &mut Solver, narrow_phase: &mut NarrowPhase) {
        solver.register::<BallSocket>();
        solver.register::<AngularHinge>();
        solver.register::<AngularSwivelHinge>();
        solver.register::<SwingLimit>();
        solver.register::<TwistServo>();
        solver.register::<TwistLimit>();
        solver.register::<TwistMotor>();
        solver.register::<AngularServo>();
        solver.register::<AngularMotor>();
        solver.register::<Weld>();
        solver.register::<VolumeConstraint>();
        solver.register::<DistanceServo>();
        solver.register::<DistanceLimit>();
        solver.register::<CenterDistanceConstraint>();
        solver.register::<AreaConstraint>();
        solver.register::<PointOnLineServo>();
        solver.register::<LinearAxisServo>();
        solver.register::<LinearAxisMotor>();
        solver.register::<LinearAxisLimit>();
        solver.register::<AngularAxisMotor>();
        solver.register::<OneBodyAngularServo>();
        solver.register::<OneBodyAngularMotor>();
        solver.register::<OneBodyLinearServo>();
        solver.register::<OneBodyLinearMotor>();
        solver.register::<SwivelHinge>();
        solver.register::<Hinge>();
        solver.register::<BallSocketMotor>();
        solver.register::<BallSocketServo>();
        solver.register::<AngularAxisGearMotor>();
        solver.register::<CenterDistanceLimit>();

        // Contact constraint types.
        solver.register::<Contact1OneBody>();
        solver.register::<Contact2OneBody>();
        solver.register::<Contact3OneBody>();
        solver.register::<Contact4OneBody>();
        solver.register::<Contact1TwoBody>();
        solver.register::<Contact2TwoBody>();
        solver.register::<Contact3TwoBody>();
        solver.register::<Contact4TwoBody>();
        solver.register::<Contact2NonconvexOneBody>();
        solver.register::<Contact3NonconvexOneBody>();
        solver.register::<Contact4NonconvexOneBody>();
        solver.register::<Contact2Nonconvex>();
        solver.register::<Contact3Nonconvex>();
        solver.register::<Contact4Nonconvex>();

        // Contact constraint accessors — wire NarrowPhase to dispatch each
        // contact manifold type through the correct type-erased path.
        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact4NonconvexTwoBodyAccessor));
        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact3NonconvexTwoBodyAccessor));
        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact2NonconvexTwoBodyAccessor));

        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact4NonconvexOneBodyAccessor));
        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact3NonconvexOneBodyAccessor));
        narrow_phase
            .register_contact_constraint_accessor(Box::new(Contact2NonconvexOneBodyAccessor));

        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2TwoBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact1TwoBodyAccessor));

        narrow_phase.register_contact_constraint_accessor(Box::new(Contact4OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact3OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact2OneBodyAccessor));
        narrow_phase.register_contact_constraint_accessor(Box::new(Contact1OneBodyAccessor));
    }

    /// Creates a task registry containing the default collision pair types.
    pub fn create_default_collision_task_registry(
    ) -> crate::physics::collision_detection::collision_task_registry::CollisionTaskRegistry {
        use crate::physics::collidables::big_compound::BigCompound;
        use crate::physics::collidables::box_shape::{Box as BoxShape, BoxWide};
        use crate::physics::collidables::capsule::{Capsule, CapsuleWide};
        use crate::physics::collidables::compound::Compound;
        use crate::physics::collidables::convex_hull::{ConvexHull, ConvexHullWide};
        use crate::physics::collidables::cylinder::{Cylinder, CylinderWide};
        use crate::physics::collidables::mesh::Mesh;
        use crate::physics::collidables::sphere::{Sphere, SphereWide};
        use crate::physics::collidables::triangle::{Triangle, TriangleWide};

        use crate::physics::collision_detection::collision_task_registry::CollisionTaskRegistry;
        use crate::physics::collision_detection::collision_tasks::convex_collision_task::create_convex_collision_task_with_ids;

        // Pair types
        use crate::physics::collision_detection::collision_tasks::pair_types::{
            CollisionPair, ConvexPairWide, FliplessPair, FliplessPairWide, SphereIncludingPair,
            SphereIncludingPairWide, SpherePair, SpherePairWide,
        };

        // Manifold wide types
        use crate::physics::collision_detection::convex_contact_manifold_wide::{
            Convex1ContactManifoldWide, Convex2ContactManifoldWide, Convex4ContactManifoldWide,
        };

        // Pair testers
        use crate::physics::collision_detection::collision_tasks::box_convex_hull_tester::BoxConvexHullTester;
        use crate::physics::collision_detection::collision_tasks::box_cylinder_tester::BoxCylinderTester;
        use crate::physics::collision_detection::collision_tasks::box_pair_tester::BoxPairTester;
        use crate::physics::collision_detection::collision_tasks::box_triangle_tester::BoxTriangleTester;
        use crate::physics::collision_detection::collision_tasks::capsule_box_tester::CapsuleBoxTester;
        use crate::physics::collision_detection::collision_tasks::capsule_convex_hull_tester::CapsuleConvexHullTester;
        use crate::physics::collision_detection::collision_tasks::capsule_cylinder_tester::CapsuleCylinderTester;
        use crate::physics::collision_detection::collision_tasks::capsule_pair_tester::CapsulePairTester;
        use crate::physics::collision_detection::collision_tasks::capsule_triangle_tester::CapsuleTriangleTester;
        use crate::physics::collision_detection::collision_tasks::convex_hull_pair_tester::ConvexHullPairTester;
        use crate::physics::collision_detection::collision_tasks::cylinder_convex_hull_tester::CylinderConvexHullTester;
        use crate::physics::collision_detection::collision_tasks::cylinder_pair_tester::CylinderPairTester;
        use crate::physics::collision_detection::collision_tasks::sphere_box_tester::SphereBoxTester;
        use crate::physics::collision_detection::collision_tasks::sphere_capsule_tester::SphereCapsuleTester;
        use crate::physics::collision_detection::collision_tasks::sphere_convex_hull_tester::SphereConvexHullTester;
        use crate::physics::collision_detection::collision_tasks::sphere_cylinder_tester::SphereCylinderTester;
        use crate::physics::collision_detection::collision_tasks::sphere_pair_tester::SpherePairTester;
        use crate::physics::collision_detection::collision_tasks::sphere_triangle_tester::SphereTriangleTester;
        use crate::physics::collision_detection::collision_tasks::triangle_convex_hull_tester::TriangleConvexHullTester;
        use crate::physics::collision_detection::collision_tasks::triangle_cylinder_tester::TriangleCylinderTester;
        use crate::physics::collision_detection::collision_tasks::triangle_pair_tester::TrianglePairTester;

        // Compound collision task types
        use crate::physics::collision_detection::collision_tasks::compound_mesh_continuations::CompoundMeshContinuations;
        use crate::physics::collision_detection::collision_tasks::compound_pair_collision_task::CompoundPairCollisionTask;
        use crate::physics::collision_detection::collision_tasks::compound_pair_continuations::CompoundPairContinuations;
        use crate::physics::collision_detection::collision_tasks::compound_pair_overlap_finder::CompoundPairOverlapFinder;
        use crate::physics::collision_detection::collision_tasks::convex_compound_collision_task::ConvexCompoundCollisionTask;
        use crate::physics::collision_detection::collision_tasks::convex_compound_continuations::ConvexCompoundContinuations;
        use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::ConvexCompoundOverlapFinder;
        use crate::physics::collision_detection::collision_tasks::convex_mesh_continuations::ConvexMeshContinuations;
        use crate::physics::collision_detection::collision_tasks::mesh_pair_continuations::MeshPairContinuations;
        use crate::physics::collision_detection::collision_tasks::mesh_pair_overlap_finder::MeshPairOverlapFinder;
        use crate::physics::collision_detection::compound_mesh_reduction::CompoundMeshReduction;
        use crate::physics::collision_detection::mesh_reduction::MeshReduction;
        use crate::physics::collision_detection::nonconvex_reduction::NonconvexReduction;

        let mut registry = CollisionTaskRegistry::new(9);

        // ── Sphere vs convex ──────────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SpherePair,
            SpherePairWide,
            SphereWide,
            SphereWide,
            Convex1ContactManifoldWide,
            SpherePairTester,
        >(Sphere::ID, Sphere::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SphereIncludingPair,
            SphereIncludingPairWide<CapsuleWide>,
            SphereWide,
            CapsuleWide,
            Convex1ContactManifoldWide,
            SphereCapsuleTester,
        >(Sphere::ID, Capsule::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SphereIncludingPair,
            SphereIncludingPairWide<BoxWide>,
            SphereWide,
            BoxWide,
            Convex1ContactManifoldWide,
            SphereBoxTester,
        >(Sphere::ID, BoxShape::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SphereIncludingPair,
            SphereIncludingPairWide<TriangleWide>,
            SphereWide,
            TriangleWide,
            Convex1ContactManifoldWide,
            SphereTriangleTester,
        >(Sphere::ID, Triangle::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SphereIncludingPair,
            SphereIncludingPairWide<CylinderWide>,
            SphereWide,
            CylinderWide,
            Convex1ContactManifoldWide,
            SphereCylinderTester,
        >(Sphere::ID, Cylinder::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            SphereIncludingPair,
            SphereIncludingPairWide<ConvexHullWide>,
            SphereWide,
            ConvexHullWide,
            Convex1ContactManifoldWide,
            SphereConvexHullTester,
        >(Sphere::ID, ConvexHull::ID)));

        // Sphere vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(Sphere::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(Sphere::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(Sphere::ID, Mesh::ID)));

        // ── Capsule vs convex ─────────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            FliplessPair,
            FliplessPairWide<CapsuleWide>,
            CapsuleWide,
            CapsuleWide,
            Convex2ContactManifoldWide,
            CapsulePairTester,
        >(Capsule::ID, Capsule::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<CapsuleWide, BoxWide>,
            CapsuleWide,
            BoxWide,
            Convex2ContactManifoldWide,
            CapsuleBoxTester,
        >(Capsule::ID, BoxShape::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<CapsuleWide, TriangleWide>,
            CapsuleWide,
            TriangleWide,
            Convex2ContactManifoldWide,
            CapsuleTriangleTester,
        >(Capsule::ID, Triangle::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<CapsuleWide, CylinderWide>,
            CapsuleWide,
            CylinderWide,
            Convex2ContactManifoldWide,
            CapsuleCylinderTester,
        >(Capsule::ID, Cylinder::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<CapsuleWide, ConvexHullWide>,
            CapsuleWide,
            ConvexHullWide,
            Convex2ContactManifoldWide,
            CapsuleConvexHullTester,
        >(Capsule::ID, ConvexHull::ID)));

        // Capsule vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(Capsule::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(Capsule::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(Capsule::ID, Mesh::ID)));

        // ── Box vs convex ─────────────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            FliplessPair,
            FliplessPairWide<BoxWide>,
            BoxWide,
            BoxWide,
            Convex4ContactManifoldWide,
            BoxPairTester,
        >(BoxShape::ID, BoxShape::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<BoxWide, TriangleWide>,
            BoxWide,
            TriangleWide,
            Convex4ContactManifoldWide,
            BoxTriangleTester,
        >(BoxShape::ID, Triangle::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<BoxWide, CylinderWide>,
            BoxWide,
            CylinderWide,
            Convex4ContactManifoldWide,
            BoxCylinderTester,
        >(BoxShape::ID, Cylinder::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<BoxWide, ConvexHullWide>,
            BoxWide,
            ConvexHullWide,
            Convex4ContactManifoldWide,
            BoxConvexHullTester,
        >(BoxShape::ID, ConvexHull::ID)));

        // Box vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(BoxShape::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(BoxShape::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(BoxShape::ID, Mesh::ID)));

        // ── Triangle vs convex ────────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            FliplessPair,
            FliplessPairWide<TriangleWide>,
            TriangleWide,
            TriangleWide,
            Convex4ContactManifoldWide,
            TrianglePairTester,
        >(Triangle::ID, Triangle::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<TriangleWide, CylinderWide>,
            TriangleWide,
            CylinderWide,
            Convex4ContactManifoldWide,
            TriangleCylinderTester,
        >(Triangle::ID, Cylinder::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<TriangleWide, ConvexHullWide>,
            TriangleWide,
            ConvexHullWide,
            Convex4ContactManifoldWide,
            TriangleConvexHullTester,
        >(Triangle::ID, ConvexHull::ID)));

        // Triangle vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(Triangle::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(Triangle::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(Triangle::ID, Mesh::ID)));

        // ── Cylinder vs convex ────────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            FliplessPair,
            FliplessPairWide<CylinderWide>,
            CylinderWide,
            CylinderWide,
            Convex4ContactManifoldWide,
            CylinderPairTester,
        >(Cylinder::ID, Cylinder::ID)));

        registry.register(Box::new(create_convex_collision_task_with_ids::<
            CollisionPair,
            ConvexPairWide<CylinderWide, ConvexHullWide>,
            CylinderWide,
            ConvexHullWide,
            Convex4ContactManifoldWide,
            CylinderConvexHullTester,
        >(Cylinder::ID, ConvexHull::ID)));

        // Cylinder vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(Cylinder::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(Cylinder::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(Cylinder::ID, Mesh::ID)));

        // ── ConvexHull vs convex ──────────────────────────────────────
        registry.register(Box::new(create_convex_collision_task_with_ids::<
            FliplessPair,
            FliplessPairWide<ConvexHullWide>,
            ConvexHullWide,
            ConvexHullWide,
            Convex4ContactManifoldWide,
            ConvexHullPairTester,
        >(ConvexHull::ID, ConvexHull::ID)));

        // ConvexHull vs compound/mesh
        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Compound>,
            ConvexCompoundContinuations<Compound>,
            NonconvexReduction,
        >::new(ConvexHull::ID, Compound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<BigCompound>,
            ConvexCompoundContinuations<BigCompound>,
            NonconvexReduction,
        >::new(ConvexHull::ID, BigCompound::ID)));

        registry.register(Box::new(ConvexCompoundCollisionTask::<
            ConvexCompoundOverlapFinder<Mesh>,
            ConvexMeshContinuations<Mesh>,
            MeshReduction,
        >::new(ConvexHull::ID, Mesh::ID)));

        // ── Compound-pair tasks ───────────────────────────────────────
        registry.register(Box::new(CompoundPairCollisionTask::<
            CompoundPairOverlapFinder<Compound, Compound>,
            CompoundPairContinuations<Compound, Compound>,
            NonconvexReduction,
        >::new(Compound::ID, Compound::ID)));

        registry.register(Box::new(CompoundPairCollisionTask::<
            CompoundPairOverlapFinder<Compound, BigCompound>,
            CompoundPairContinuations<Compound, BigCompound>,
            NonconvexReduction,
        >::new(Compound::ID, BigCompound::ID)));

        registry.register(Box::new(CompoundPairCollisionTask::<
            CompoundPairOverlapFinder<Compound, Mesh>,
            CompoundMeshContinuations<Compound, Mesh>,
            CompoundMeshReduction,
        >::new(Compound::ID, Mesh::ID)));

        registry.register(Box::new(CompoundPairCollisionTask::<
            CompoundPairOverlapFinder<BigCompound, BigCompound>,
            CompoundPairContinuations<BigCompound, BigCompound>,
            NonconvexReduction,
        >::new(BigCompound::ID, BigCompound::ID)));

        registry.register(Box::new(CompoundPairCollisionTask::<
            CompoundPairOverlapFinder<BigCompound, Mesh>,
            CompoundMeshContinuations<BigCompound, Mesh>,
            CompoundMeshReduction,
        >::new(BigCompound::ID, Mesh::ID)));

        registry.register(Box::new(CompoundPairCollisionTask::<
            MeshPairOverlapFinder<Mesh, Mesh>,
            MeshPairContinuations<Mesh, Mesh>,
            CompoundMeshReduction,
        >::new(Mesh::ID, Mesh::ID)));

        registry
    }

    /// Creates the default sweep task registry with all built-in shape pair sweep tasks.
    pub fn create_default_sweep_task_registry() -> SweepTaskRegistry {
        use crate::physics::collidables::big_compound::BigCompound;
        use crate::physics::collidables::box_shape::{Box as BoxShape, BoxSupportFinder, BoxWide};
        use crate::physics::collidables::capsule::{Capsule, CapsuleSupportFinder, CapsuleWide};
        use crate::physics::collidables::compound::Compound;
        use crate::physics::collidables::convex_hull::{
            ConvexHull, ConvexHullSupportFinder, ConvexHullWide,
        };
        use crate::physics::collidables::cylinder::{
            Cylinder, CylinderSupportFinder, CylinderWide,
        };
        use crate::physics::collidables::mesh::Mesh;
        use crate::physics::collidables::sphere::{Sphere, SphereSupportFinder, SphereWide};
        use crate::physics::collidables::triangle::{
            Triangle, TriangleSupportFinder, TriangleWide,
        };

        use crate::physics::collision_detection::sweep_tasks::convex_pair_sweep_task::ConvexPairSweepTask;
        use crate::physics::collision_detection::sweep_tasks::convex_compound_sweep_task::ConvexCompoundSweepTask;
        use crate::physics::collision_detection::sweep_tasks::convex_homogeneous_compound_sweep_task::ConvexHomogeneousCompoundSweepTask;
        use crate::physics::collision_detection::sweep_tasks::compound_pair_sweep_task::CompoundPairSweepTask;
        use crate::physics::collision_detection::sweep_tasks::compound_homogeneous_compound_sweep_task::CompoundHomogeneousCompoundSweepTask;

        use crate::physics::collision_detection::sweep_tasks::capsule_box_distance_tester::CapsuleBoxDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::capsule_pair_distance_tester::CapsulePairDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::gjk_distance_tester::GJKDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::sphere_box_distance_tester::SphereBoxDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::sphere_capsule_distance_tester::SphereCapsuleDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::sphere_cylinder_distance_tester::SphereCylinderDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::sphere_pair_distance_tester::SpherePairDistanceTester;
        use crate::physics::collision_detection::sweep_tasks::sphere_triangle_distance_tester::SphereTriangleDistanceTester;

        use crate::physics::collision_detection::sweep_tasks::convex_compound_sweep_overlap_finder::ConvexCompoundSweepOverlapFinder;
        use crate::physics::collision_detection::sweep_tasks::compound_pair_sweep_overlap_finder::CompoundPairSweepOverlapFinder;

        let mut registry = SweepTaskRegistry::new(9);

        // Sphere vs everything
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            Sphere,
            SphereWide,
            SpherePairDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            Capsule,
            CapsuleWide,
            SphereCapsuleDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            Cylinder,
            CylinderWide,
            SphereCylinderDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            BoxShape,
            BoxWide,
            SphereBoxDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            Triangle,
            TriangleWide,
            SphereTriangleDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Sphere,
            SphereWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                Sphere,
                SphereWide,
                SphereSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Sphere,
            SphereWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<Sphere, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Sphere,
            SphereWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<Sphere, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            Sphere,
            SphereWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<Sphere, Mesh>,
        >::new()));

        // Capsule vs everything (excluding sphere, already registered above)
        registry.register(Box::new(ConvexPairSweepTask::<
            Capsule,
            CapsuleWide,
            Capsule,
            CapsuleWide,
            CapsulePairDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Capsule,
            CapsuleWide,
            Cylinder,
            CylinderWide,
            GJKDistanceTester<
                Capsule,
                CapsuleWide,
                CapsuleSupportFinder,
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Capsule,
            CapsuleWide,
            BoxShape,
            BoxWide,
            CapsuleBoxDistanceTester,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Capsule,
            CapsuleWide,
            Triangle,
            TriangleWide,
            GJKDistanceTester<
                Capsule,
                CapsuleWide,
                CapsuleSupportFinder,
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Capsule,
            CapsuleWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                Capsule,
                CapsuleWide,
                CapsuleSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Capsule,
            CapsuleWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<Capsule, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Capsule,
            CapsuleWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<Capsule, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            Capsule,
            CapsuleWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<Capsule, Mesh>,
        >::new()));

        // Cylinder vs everything
        registry.register(Box::new(ConvexPairSweepTask::<
            Cylinder,
            CylinderWide,
            Cylinder,
            CylinderWide,
            GJKDistanceTester<
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Cylinder,
            CylinderWide,
            BoxShape,
            BoxWide,
            GJKDistanceTester<
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
                BoxShape,
                BoxWide,
                BoxSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Cylinder,
            CylinderWide,
            Triangle,
            TriangleWide,
            GJKDistanceTester<
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Cylinder,
            CylinderWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                Cylinder,
                CylinderWide,
                CylinderSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Cylinder,
            CylinderWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<Cylinder, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Cylinder,
            CylinderWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<Cylinder, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            Cylinder,
            CylinderWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<Cylinder, Mesh>,
        >::new()));

        // Box vs everything
        registry.register(Box::new(ConvexPairSweepTask::<
            BoxShape,
            BoxWide,
            BoxShape,
            BoxWide,
            GJKDistanceTester<
                BoxShape,
                BoxWide,
                BoxSupportFinder,
                BoxShape,
                BoxWide,
                BoxSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            BoxShape,
            BoxWide,
            Triangle,
            TriangleWide,
            GJKDistanceTester<
                BoxShape,
                BoxWide,
                BoxSupportFinder,
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            BoxShape,
            BoxWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                BoxShape,
                BoxWide,
                BoxSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            BoxShape,
            BoxWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<BoxShape, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            BoxShape,
            BoxWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<BoxShape, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            BoxShape,
            BoxWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<BoxShape, Mesh>,
        >::new()));

        // Triangle vs everything
        registry.register(Box::new(ConvexPairSweepTask::<
            Triangle,
            TriangleWide,
            Triangle,
            TriangleWide,
            GJKDistanceTester<
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexPairSweepTask::<
            Triangle,
            TriangleWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                Triangle,
                TriangleWide,
                TriangleSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Triangle,
            TriangleWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<Triangle, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            Triangle,
            TriangleWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<Triangle, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            Triangle,
            TriangleWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<Triangle, Mesh>,
        >::new()));

        // ConvexHull vs everything
        registry.register(Box::new(ConvexPairSweepTask::<
            ConvexHull,
            ConvexHullWide,
            ConvexHull,
            ConvexHullWide,
            GJKDistanceTester<
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
                ConvexHull,
                ConvexHullWide,
                ConvexHullSupportFinder,
            >,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            ConvexHull,
            ConvexHullWide,
            Compound,
            ConvexCompoundSweepOverlapFinder<ConvexHull, Compound>,
        >::new()));
        registry.register(Box::new(ConvexCompoundSweepTask::<
            ConvexHull,
            ConvexHullWide,
            BigCompound,
            ConvexCompoundSweepOverlapFinder<ConvexHull, BigCompound>,
        >::new()));
        registry.register(Box::new(ConvexHomogeneousCompoundSweepTask::<
            ConvexHull,
            ConvexHullWide,
            Mesh,
            Triangle,
            TriangleWide,
            ConvexCompoundSweepOverlapFinder<ConvexHull, Mesh>,
        >::new()));

        // Compound vs Compound/BigCompound/Mesh
        registry.register(Box::new(CompoundPairSweepTask::<
            Compound,
            Compound,
            CompoundPairSweepOverlapFinder<Compound, Compound>,
        >::new()));
        registry.register(Box::new(CompoundPairSweepTask::<
            Compound,
            BigCompound,
            CompoundPairSweepOverlapFinder<Compound, BigCompound>,
        >::new()));
        registry.register(Box::new(CompoundHomogeneousCompoundSweepTask::<
            Compound,
            Mesh,
            Triangle,
            TriangleWide,
            CompoundPairSweepOverlapFinder<Compound, Mesh>,
        >::new()));

        // BigCompound vs BigCompound/Mesh
        registry.register(Box::new(CompoundPairSweepTask::<
            BigCompound,
            BigCompound,
            CompoundPairSweepOverlapFinder<BigCompound, BigCompound>,
        >::new()));
        registry.register(Box::new(CompoundHomogeneousCompoundSweepTask::<
            BigCompound,
            Mesh,
            Triangle,
            TriangleWide,
            CompoundPairSweepOverlapFinder<BigCompound, Mesh>,
        >::new()));

        //TODO: No mesh-mesh at the moment.
        registry
    }
}
