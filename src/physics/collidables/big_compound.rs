use glam::{Quat, Vec3};

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::compound::{Compound, CompoundChild};
use super::shape::IShape;
use super::shapes::Shapes;

// Forward reference: Tree type from physics::trees module
// use crate::physics::trees::Tree;

/// Compound shape containing a bunch of shapes accessible through a tree acceleration structure.
/// Useful for compounds with lots of children.
#[repr(C)]
pub struct BigCompound {
    /// Acceleration structure for the compound children.
    /// TODO: Tree type needs to be wired up from physics::trees.
    // pub tree: Tree,
    /// Buffer of children within this compound.
    pub children: Buffer<CompoundChild>,
}

impl BigCompound {
    /// Type id of big compound shapes.
    pub const ID: i32 = 7;

    /// Gets the number of children in the compound.
    pub fn child_count(&self) -> usize {
        self.children.len() as usize
    }

    /// Gets a reference to a child by index.
    pub fn get_child(&self, compound_child_index: usize) -> &CompoundChild {
        &self.children[compound_child_index]
    }

    /// Gets a mutable reference to a child by index.
    pub fn get_child_mut(&mut self, compound_child_index: usize) -> &mut CompoundChild {
        &mut self.children[compound_child_index]
    }

    /// Disposes resources.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.children);
        // TODO: self.tree.dispose(pool);
    }

    /// Computes the bounding box for the big compound given an orientation.
    /// Delegates to Compound::compute_child_bounds for each child.
    pub fn compute_bounds(
        &self,
        orientation: Quat,
        shape_batches: &Shapes,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        Compound::compute_child_bounds(&self.children[0], orientation, shape_batches, min, max);
        for i in 1..self.children.len() as usize {
            let mut child_min = Vec3::ZERO;
            let mut child_max = Vec3::ZERO;
            Compound::compute_child_bounds(
                &self.children[i],
                orientation,
                shape_batches,
                &mut child_min,
                &mut child_max,
            );
            *min = min.min(child_min);
            *max = max.max(child_max);
        }
    }

    // TODO: The following methods depend on Tree and collision detection:
    // - create_without_tree_build
    // - fill_subtrees_for_children
    // - create_with_sweep_build
    // - new (BigCompound constructor with binned build — requires Tree)
    // - add_child_bounds_to_batcher
    // - ray_test (both single and batched — requires Tree)
    // - find_local_overlaps
    // - compute_inertia (delegates to CompoundBuilder)
    // - add / remove_at
}

impl IShape for BigCompound {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
