// Translated from BepuPhysics/CollisionDetection/CollisionTaskRegistry.cs

use super::collision_batcher_continuations::{BatcherContinuations, PairContinuation};
use super::compound_mesh_reduction::CompoundMeshReduction;
use super::contact_manifold::ConvexContactManifold;
use super::mesh_reduction::MeshReduction;
use super::nonconvex_reduction::NonconvexReduction;
use super::untyped_list::UntypedList;
use crate::physics::body_properties::BodyVelocity;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::{Quat, Vec3};
// ICollisionCallbacks is defined in collision_batcher.rs

/// Type-erased function pointer table for collision batcher operations.
///
/// This enables compound collision tasks (which generate subtasks back into the batcher)
/// to call batcher methods without knowing the concrete `TCallbacks` type.
/// In C#, this is handled via the generic virtual method `ExecuteBatch<TCallbacks>`;
/// in Rust, we provide an explicit vtable since trait objects cannot have generic methods.
#[repr(C)]
pub struct BatcherVtable {
    /// Type-erased pointer to the `CollisionBatcher<TCallbacks>`.
    pub batcher: *mut u8,
    /// Memory pool.
    pub pool: *mut BufferPool,
    /// Shapes collection.
    pub shapes: *mut crate::physics::collidables::shapes::Shapes,
    /// Timestep duration.
    pub dt: f32,
    /// NonconvexReduction continuations manager.
    pub nonconvex_reductions: *mut BatcherContinuations<NonconvexReduction>,
    /// MeshReduction continuations manager.
    pub mesh_reductions: *mut BatcherContinuations<MeshReduction>,
    /// CompoundMeshReduction continuations manager.
    pub compound_mesh_reductions: *mut BatcherContinuations<CompoundMeshReduction>,

    /// Process a convex collision result (used by convex tasks).
    pub process_convex_result: unsafe fn(*mut u8, &mut ConvexContactManifold, &PairContinuation),
    /// Add a child pair directly to the batcher for testing (used by compound tasks).
    pub add_directly: unsafe fn(
        batcher: *mut u8,
        shape_type_a: i32,
        shape_type_b: i32,
        shape_a: *const u8,
        shape_b: *const u8,
        offset_b: Vec3,
        orientation_a: Quat,
        orientation_b: Quat,
        velocity_a: &BodyVelocity,
        velocity_b: &BodyVelocity,
        speculative_margin: f32,
        maximum_expansion: f32,
        pair_continuation: &PairContinuation,
    ),
    /// Process an empty result (no overlaps found for a pair).
    pub process_empty_result: unsafe fn(batcher: *mut u8, continuation: &PairContinuation),
    /// Process an untested subpair result (testing was blocked by callback).
    pub process_untested_subpair_convex_result:
        unsafe fn(batcher: *mut u8, continuation: &PairContinuation),
    /// Check if collision testing is allowed for a pair.
    pub allow_collision_testing:
        unsafe fn(batcher: *mut u8, pair_id: i32, child_a: i32, child_b: i32) -> bool,
}

/// Describes the data requirements for a collision pair type in a CollisionBatcher.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionTaskPairType {
    /// General pair for two shapes with full pose and flip mask, but no bounds related data.
    StandardPair = 0,
    /// Pair specialized for convex pairs between two shapes of the same type.
    FliplessPair = 1,
    /// Pair specialized for two spheres, requiring no flip mask or orientations.
    SpherePair = 2,
    /// Pair specialized for convex pairs that involve one sphere which requires no orientation.
    SphereIncludingPair = 3,
    /// Pair that requires computing local bounding boxes, and so requires extra information like velocity.
    BoundsTestedPair = 4,
}

/// Parent type of tasks which handle collision tests between batches of shapes of a particular type.
pub trait CollisionTask {
    /// Gets the number of tasks to batch together before executing this task.
    fn batch_size(&self) -> i32;
    /// Gets the first shape type index associated with the task.
    fn shape_type_index_a(&self) -> i32;
    /// Gets the second shape type index associated with the task.
    fn shape_type_index_b(&self) -> i32;
    /// Gets whether the task is capable of generating subtasks.
    fn subtask_generator(&self) -> bool;
    /// Gets the pair type that the execute_batch call requires.
    fn pair_type(&self) -> CollisionTaskPairType;
    /// Executes the task on the given input.
    fn execute_batch(
        &self,
        batch: &mut UntypedList,
        vtable: &BatcherVtable,
    );
}

/// Metadata about a collision task.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CollisionTaskReference {
    /// Index of the task in the registry.
    pub task_index: i32,
    /// Number of pairs to accumulate in a batch before dispatching tests.
    pub batch_size: i32,
    /// The type id that is expected to come first in the collision pair.
    pub expected_first_type_id: i32,
    /// Data requirements for the collision pair type.
    pub pair_type: CollisionTaskPairType,
}

impl Default for CollisionTaskReference {
    fn default() -> Self {
        Self {
            task_index: -1,
            batch_size: 0,
            expected_first_type_id: 0,
            pair_type: CollisionTaskPairType::StandardPair,
        }
    }
}

/// Registry of collision tasks used to handle various shape pair types.
pub struct CollisionTaskRegistry {
    top_level_matrix: Vec<Vec<CollisionTaskReference>>,
    pub(crate) tasks: Vec<Box<dyn CollisionTask>>,
    count: usize,
}

impl CollisionTaskRegistry {
    /// Creates a new collision task registry.
    pub fn new(initial_shape_count: usize) -> Self {
        let mut registry = Self {
            top_level_matrix: Vec::new(),
            tasks: Vec::new(),
            count: 0,
        };
        registry.resize_matrix(initial_shape_count);
        registry
    }

    fn resize_matrix(&mut self, new_size: usize) {
        let old_size = self.top_level_matrix.len();
        self.top_level_matrix.resize_with(new_size, Vec::new);
        for i in 0..new_size {
            self.top_level_matrix[i].resize_with(new_size, CollisionTaskReference::default);
            for j in old_size..new_size {
                self.top_level_matrix[i][j] = CollisionTaskReference { task_index: -1, ..Default::default() };
            }
        }
    }

    /// Registers a collision task. Returns the index of the task in the registry.
    pub fn register(&mut self, task: Box<dyn CollisionTask>) -> usize {
        // Some tasks can generate tasks. Note that this can only be one level deep; nesting compounds is not allowed.
        // All such generators will be placed at the beginning.
        let index = if task.subtask_generator() { 0 } else { self.count };

        let new_count = self.count + 1;
        if self.tasks.len() < new_count {
            self.tasks.reserve(new_count - self.tasks.len());
        }

        if index < self.count {
            // Insert at the beginning, shift everything else
            self.tasks.insert(index, task);
            // Every task in the array that got moved needs to have its top level index bumped.
            for i in 0..self.top_level_matrix.len() {
                for j in 0..self.top_level_matrix.len() {
                    if self.top_level_matrix[i][j].task_index >= index as i32 {
                        self.top_level_matrix[i][j].task_index += 1;
                    }
                }
            }
        } else {
            self.tasks.push(task);
        }

        self.count = new_count;

        let a = self.tasks[index].shape_type_index_a();
        let b = self.tasks[index].shape_type_index_b();
        let highest_shape_index = if a > b { a } else { b };
        if highest_shape_index >= 0 {
            // This only handles top level pairs (those associated with shape type pairs).
            let highest = highest_shape_index as usize;
            if highest >= self.top_level_matrix.len() {
                self.resize_matrix(highest + 1);
            }
            let task_info = CollisionTaskReference {
                task_index: index as i32,
                batch_size: self.tasks[index].batch_size(),
                expected_first_type_id: self.tasks[index].shape_type_index_a(),
                pair_type: self.tasks[index].pair_type(),
            };
            self.top_level_matrix[a as usize][b as usize] = task_info;
            self.top_level_matrix[b as usize][a as usize] = task_info;
        }

        #[cfg(debug_assertions)]
        {
            // Ensure that no task dependency cycles exist.
            let mut encountered_nongenerator = false;
            for i in 0..self.count {
                if encountered_nongenerator {
                    debug_assert!(
                        !self.tasks[i].subtask_generator(),
                        "To avoid cycles, the tasks list should be partitioned into two contiguous groups: subtask generators, followed by non-subtask generators."
                    );
                } else if !self.tasks[i].subtask_generator() {
                    encountered_nongenerator = true;
                }
            }
        }

        index
    }

    /// Gets metadata about the task associated with a shape type pair.
    #[inline(always)]
    pub fn get_task_reference(&self, top_level_type_a: i32, top_level_type_b: i32) -> &CollisionTaskReference {
        &self.top_level_matrix[top_level_type_a as usize][top_level_type_b as usize]
    }

    /// Gets a mutable reference to the metadata about the task associated with a shape type pair.
    #[inline(always)]
    pub fn get_task_reference_mut(&mut self, top_level_type_a: i32, top_level_type_b: i32) -> &mut CollisionTaskReference {
        &mut self.top_level_matrix[top_level_type_a as usize][top_level_type_b as usize]
    }

    /// Gets the collision task at the given index.
    #[inline(always)]
    pub fn get_task(&self, task_index: i32) -> &dyn CollisionTask {
        &*self.tasks[task_index as usize]
    }
}
