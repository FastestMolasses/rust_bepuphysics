// Translated from BepuPhysics/CollisionDetection/CollisionTasks/ConvexCollisionTask.cs

use crate::physics::collidables::shape::IShapeWideAllocation;
use crate::physics::collision_detection::collision_batcher::{CollisionBatcher, ICollisionCallbacks};
use crate::physics::collision_detection::collision_batcher_continuations::PairContinuation;
use crate::physics::collision_detection::collision_task_registry::{BatcherVtable, CollisionTask};
use crate::physics::collision_detection::contact_manifold::ConvexContactManifold;
use crate::physics::collision_detection::convex_contact_manifold_wide::IContactManifoldWide;
use crate::physics::collision_detection::untyped_list::UntypedList;
use crate::utilities::gather_scatter::GatherScatter;
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

/// The type-erased function pointer for executing a convex collision batch.
/// Parameters: (batch, vtable)
type ConvexExecuteFn = unsafe fn(
    &mut UntypedList,
    &BatcherVtable,
);

/// Monomorphized execution loop for convex collision tasks.
/// This function is instantiated per concrete pair type combination and stored as a function pointer.
unsafe fn execute_batch_inner<TPair, TPairWide, TShapeWideA, TShapeWideB, TManifoldWide, TPairTester>(
    batch: &mut UntypedList,
    vtable: &BatcherVtable,
)
where
    TPair: ICollisionPair + Copy,
    TPairWide: ICollisionPairWide<TShapeWideA, TShapeWideB, TPair> + Default,
    TManifoldWide: IContactManifoldWide + Default,
    TPairTester: IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>,
    TShapeWideA: IShapeWideAllocation,
    TShapeWideB: IShapeWideAllocation,
{
    let vector_width = crate::utilities::vector::VECTOR_WIDTH;
    let start = batch.buffer.as_ptr() as *const TPair;
    let mut pair_wide = TPairWide::default();

    // Handle IShapeWide internal allocation (e.g. ConvexHullWide).
    // These allocations must outlive pair_wide usage, so we hold them here.
    let _alloc_a;
    let _alloc_b;
    {
        let a_wide = pair_wide.get_shape_a_mut();
        let alloc_size = a_wide.internal_allocation_size_of();
        if alloc_size > 0 {
            _alloc_a = vec![0u8; alloc_size];
            let buf = crate::utilities::memory::buffer::Buffer::new(
                _alloc_a.as_ptr() as *mut u8, alloc_size as i32, -1,
            );
            a_wide.initialize_allocation(&buf);
        } else {
            _alloc_a = Vec::new();
        }
    }
    {
        let b_wide = pair_wide.get_shape_b_mut();
        let alloc_size = b_wide.internal_allocation_size_of();
        if alloc_size > 0 {
            _alloc_b = vec![0u8; alloc_size];
            let buf = crate::utilities::memory::buffer::Buffer::new(
                _alloc_b.as_ptr() as *mut u8, alloc_size as i32, -1,
            );
            b_wide.initialize_allocation(&buf);
        } else {
            _alloc_b = Vec::new();
        }
    }

    let mut manifold_wide = TManifoldWide::default();
    let mut manifold = ConvexContactManifold::default();

    let mut i = 0i32;
    while (i as usize) < batch.count as usize {
        let bundle_start = start.add(i as usize);
        let mut count_in_bundle = batch.count - i;
        if count_in_bundle > vector_width as i32 {
            count_in_bundle = vector_width as i32;
        }

        // Gather scalar pairs into SIMD-wide bundle
        for j in 0..count_in_bundle {
            pair_wide.write_slot(j, &*bundle_start.add(j as usize));
        }

        // Dispatch to the appropriate test variant based on orientation count
        if TPairWide::ORIENTATION_COUNT == 2 {
            TPairTester::test_two_orientations(
                pair_wide.get_shape_a(),
                pair_wide.get_shape_b(),
                pair_wide.get_speculative_margin(),
                pair_wide.get_offset_b(),
                pair_wide.get_orientation_a(),
                pair_wide.get_orientation_b(),
                count_in_bundle,
                &mut manifold_wide,
            );
        } else if TPairWide::ORIENTATION_COUNT == 1 {
            TPairTester::test_one_orientation(
                pair_wide.get_shape_a(),
                pair_wide.get_shape_b(),
                pair_wide.get_speculative_margin(),
                pair_wide.get_offset_b(),
                pair_wide.get_orientation_b(),
                count_in_bundle,
                &mut manifold_wide,
            );
        } else {
            debug_assert!(TPairWide::ORIENTATION_COUNT == 0);
            TPairTester::test_no_orientation(
                pair_wide.get_shape_a(),
                pair_wide.get_shape_b(),
                pair_wide.get_speculative_margin(),
                pair_wide.get_offset_b(),
                count_in_bundle,
                &mut manifold_wide,
            );
        }

        // Apply flip mask for pairs that were reordered for shape type normalization
        if TPairWide::HAS_FLIP_MASK {
            let flip_mask = *pair_wide.get_flip_mask();
            manifold_wide.apply_flip_mask(pair_wide.get_offset_b_mut(), &flip_mask);
        }

        // Extract per-lane results and dispatch to callbacks
        let offset_b_ptr = pair_wide.get_offset_b() as *const Vector3Wide;
        for j in 0..count_in_bundle {
            // Use raw pointer offset to get lane-offset views (like C#'s GetOffsetInstance).
            // This adds j * sizeof(f32) bytes to the struct pointer, giving a "view" into lane j.
            let manifold_source = (&manifold_wide as *const TManifoldWide as *const u8)
                .add(j as usize * std::mem::size_of::<f32>()) as *const TManifoldWide;
            let offset_source = (offset_b_ptr as *const u8)
                .add(j as usize * std::mem::size_of::<f32>()) as *const Vector3Wide;
            (*manifold_source).read_first(&*offset_source, &mut manifold);
            let pair = &*bundle_start.add(j as usize);
            (vtable.process_convex_result)(vtable.batcher, &mut manifold, pair.get_continuation());
        }

        i += vector_width as i32;
    }
}

/// Concrete convex collision task for registration.
pub struct ConvexCollisionTaskInstance {
    pub batch_size: i32,
    pub shape_type_index_a: i32,
    pub shape_type_index_b: i32,
    pub pair_type: crate::physics::collision_detection::collision_task_registry::CollisionTaskPairType,
    execute_fn: ConvexExecuteFn,
}

impl crate::physics::collision_detection::collision_task_registry::CollisionTask for ConvexCollisionTaskInstance {
    fn batch_size(&self) -> i32 { self.batch_size }
    fn shape_type_index_a(&self) -> i32 { self.shape_type_index_a }
    fn shape_type_index_b(&self) -> i32 { self.shape_type_index_b }
    fn subtask_generator(&self) -> bool { false }
    fn pair_type(&self) -> crate::physics::collision_detection::collision_task_registry::CollisionTaskPairType {
        self.pair_type
    }
    fn execute_batch(
        &self,
        batch: &mut crate::physics::collision_detection::untyped_list::UntypedList,
        vtable: &BatcherVtable,
    ) {
        unsafe { (self.execute_fn)(batch, vtable) }
    }
}

/// Creates a convex collision task instance with a monomorphized execution loop.
pub fn create_convex_collision_task<TPair, TPairWide, TShapeWideA, TShapeWideB, TManifoldWide, TPairTester>(
) -> ConvexCollisionTaskInstance
where
    TPair: ICollisionPair + Copy + 'static,
    TPairWide: ICollisionPairWide<TShapeWideA, TShapeWideB, TPair> + Default + 'static,
    TManifoldWide: IContactManifoldWide + Default + 'static,
    TPairTester: IPairTester<TShapeWideA, TShapeWideB, TManifoldWide> + 'static,
    TShapeWideA: IShapeWideAllocation + 'static,
    TShapeWideB: IShapeWideAllocation + 'static,
{
    ConvexCollisionTaskInstance {
        batch_size: TPairTester::batch_size(),
        shape_type_index_a: TPair::pair_type() as i32, // Will be overridden by shape TypeId
        shape_type_index_b: -1,
        pair_type: TPair::pair_type(),
        execute_fn: execute_batch_inner::<TPair, TPairWide, TShapeWideA, TShapeWideB, TManifoldWide, TPairTester>,
    }
}

/// Creates a convex collision task instance with explicit shape type indices.
pub fn create_convex_collision_task_with_ids<TPair, TPairWide, TShapeWideA, TShapeWideB, TManifoldWide, TPairTester>(
    shape_type_a: i32,
    shape_type_b: i32,
) -> ConvexCollisionTaskInstance
where
    TPair: ICollisionPair + Copy + 'static,
    TPairWide: ICollisionPairWide<TShapeWideA, TShapeWideB, TPair> + Default + 'static,
    TManifoldWide: IContactManifoldWide + Default + 'static,
    TPairTester: IPairTester<TShapeWideA, TShapeWideB, TManifoldWide> + 'static,
    TShapeWideA: IShapeWideAllocation + 'static,
    TShapeWideB: IShapeWideAllocation + 'static,
{
    ConvexCollisionTaskInstance {
        batch_size: TPairTester::batch_size(),
        shape_type_index_a: shape_type_a,
        shape_type_index_b: shape_type_b,
        pair_type: TPair::pair_type(),
        execute_fn: execute_batch_inner::<TPair, TPairWide, TShapeWideA, TShapeWideB, TManifoldWide, TPairTester>,
    }
}
