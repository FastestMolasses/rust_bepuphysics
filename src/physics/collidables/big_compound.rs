use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::compound::CompoundChild;
use super::shape::IShape;

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

    // TODO: The following methods depend on Shapes, BoundingBoxBatcher, Tree, and collision detection:
    // - create_without_tree_build
    // - fill_subtrees_for_children
    // - create_with_sweep_build
    // - new (BigCompound constructor with binned build)
    // - compute_bounds
    // - add_child_bounds_to_batcher
    // - ray_test (both single and batched)
    // - find_local_overlaps
    // - compute_inertia
    // - add / remove_at
}

impl IShape for BigCompound {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
