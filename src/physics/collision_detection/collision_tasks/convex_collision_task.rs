// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCollisionTask.cs

use crate::physics::collision_detection::collision_batcher::{CollisionBatcher, ICollisionCallbacks};
use crate::physics::collision_detection::collision_task_registry::CollisionTask;
use crate::physics::collision_detection::contact_manifold::ConvexContactManifold;
use crate::physics::collision_detection::convex_contact_manifold_wide::IContactManifoldWide;
use crate::physics::collision_detection::untyped_list::UntypedList;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use super::pair_types::{ICollisionPair, ICollisionPairWide};

/// Trait for pair testers that perform SIMD-wide collision tests between two shapes.
pub trait IPairTester<TShapeWideA, TShapeWideB, TManifoldWide> {
    /// Gets the number of pairs which would ideally be gathered together before executing a wide test.
    fn batch_size() -> i32;

    /// Tests with two orientations (standard convex pair).
    fn test_two_orientations(
        a: &TShapeWideA,
        b: &TShapeWideB,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut TManifoldWide,
    );

    /// Tests with one orientation (sphere-including pair).
    fn test_one_orientation(
        a: &TShapeWideA,
        b: &TShapeWideB,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        orientation_b: &QuaternionWide,
        pair_count: i32,
        manifold: &mut TManifoldWide,
    );

    /// Tests with no orientation (sphere pair).
    fn test_no_orientation(
        a: &TShapeWideA,
        b: &TShapeWideB,
        speculative_margin: &Vector<f32>,
        offset_b: &Vector3Wide,
        pair_count: i32,
        manifold: &mut TManifoldWide,
    );
}

// NOTE: The C# ConvexCollisionTask is a strongly-typed generic class that encapsulates
// the entire batch execution loop with compile-time monomorphization.
// In Rust, this would be achieved through monomorphized function calls rather than a class hierarchy.
// The actual collision task dispatch is handled through function pointers in the CollisionTaskRegistry.
//
// The core loop logic from ConvexCollisionTask.ExecuteBatch is:
// 1. Iterate through pairs in SIMD-width bundles
// 2. Gather scalar pairs into wide SIMD bundles (WriteSlot)
// 3. Run the appropriate Test variant based on OrientationCount
// 4. Apply flip mask if needed
// 5. Extract per-lane results (ReadFirst) and dispatch to callbacks

/// Concrete convex collision task for registration.
pub struct ConvexCollisionTaskInstance {
    pub batch_size: i32,
    pub shape_type_index_a: i32,
    pub shape_type_index_b: i32,
}

impl crate::physics::collision_detection::collision_task_registry::CollisionTask for ConvexCollisionTaskInstance {
    fn batch_size(&self) -> i32 { self.batch_size }
    fn shape_type_index_a(&self) -> i32 { self.shape_type_index_a }
    fn shape_type_index_b(&self) -> i32 { self.shape_type_index_b }
    fn subtask_generator(&self) -> bool { false }
    fn pair_type(&self) -> crate::physics::collision_detection::collision_task_registry::CollisionTaskPairType {
        crate::physics::collision_detection::collision_task_registry::CollisionTaskPairType::StandardPair
    }
    fn execute_batch(&self, _batch: &mut crate::physics::collision_detection::untyped_list::UntypedList, _batcher: *mut u8) {
        // TODO: Implement the actual batched test execution using TPairTester
    }
}

/// Helper to create a convex collision task for registration.
pub fn create_convex_collision_task<TPairTester, TShapeWideA, TShapeWideB, TManifoldWide>(
    shape_type_a: i32,
    shape_type_b: i32,
    batch_size: i32,
) -> ConvexCollisionTaskInstance
where
    TPairTester: IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>,
{
    ConvexCollisionTaskInstance {
        batch_size,
        shape_type_index_a: shape_type_a,
        shape_type_index_b: shape_type_b,
    }
}
