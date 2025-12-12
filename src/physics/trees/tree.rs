// Translated from BepuPhysics/Trees/Tree.cs

use crate::utilities::bounding_box::{BoundingBox, BoundingBox4};
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;
use std::mem::MaybeUninit;

use super::leaf::Leaf;
use super::node::{Metanode, Node, NodeChild, CostOrFlag};

/// A binary bounding volume tree (BVH).
///
/// Note: In C#, Tree is a partial struct spread across many files.
/// In Rust, the struct is defined here and extended with `impl Tree` blocks in sibling modules.
#[repr(C)]
pub struct Tree {
    /// Buffer of nodes in the tree.
    pub nodes: Buffer<Node>,
    /// Buffer of metanodes in the tree. Metanodes contain metadata that aren't read during most
    /// query operations but are useful for bookkeeping.
    pub metanodes: Buffer<Metanode>,
    /// Buffer of leaves in the tree.
    pub leaves: Buffer<Leaf>,
    /// Number of nodes in the tree.
    pub node_count: i32,
    /// Number of leaves in the tree.
    pub leaf_count: i32,
}

/// Maximum stack depth for iterative tree traversals.
pub const TRAVERSAL_STACK_CAPACITY: usize = 256;

impl Tree {
    /// Encodes a leaf index into the negative-index form used by the tree.
    #[inline(always)]
    pub fn encode(index: i32) -> i32 {
        -1 - index
    }

    #[inline(always)]
    pub(crate) fn allocate_node(&mut self) -> i32 {
        debug_assert!(
            self.nodes.len() > self.node_count && self.metanodes.len() > self.node_count,
            "Any attempt to allocate a node should not overrun the allocated nodes."
        );
        let index = self.node_count;
        self.node_count += 1;
        index
    }

    #[inline(always)]
    pub(crate) fn add_leaf(&mut self, node_index: i32, child_index: i32) -> i32 {
        debug_assert!(
            self.leaf_count < self.leaves.len(),
            "Any attempt to allocate a leaf should not overrun the allocated leaves."
        );
        *self.leaves.get_mut(self.leaf_count) = Leaf::new(node_index, child_index);
        let index = self.leaf_count;
        self.leaf_count += 1;
        index
    }

    fn allocate_leaf(&mut self) -> i32 {
        debug_assert!(
            self.leaf_count < self.leaves.len(),
            "Any attempt to allocate a leaf should not overrun the allocated leaves."
        );
        let index = self.leaf_count;
        self.leaf_count += 1;
        index
    }

    /// Gets bounds pointers for a leaf in the tree.
    #[inline(always)]
    pub unsafe fn get_bounds_pointers(&self, leaf_index: i32) -> (*mut Vec3, *mut Vec3) {
        let leaf = *self.leaves.get(leaf_index);
        let node_child = (&*self.nodes.get(leaf.node_index()) as *const Node as *const NodeChild)
            .add(leaf.child_index() as usize) as *mut NodeChild;
        (&mut (*node_child).min as *mut Vec3, &mut (*node_child).max as *mut Vec3)
    }

    /// Applies updated bounds to the given leaf index in the tree, refitting the tree to match.
    #[inline(always)]
    pub fn update_bounds(&self, leaf_index: i32, min: Vec3, max: Vec3) {
        unsafe {
            let (min_ptr, max_ptr) = self.get_bounds_pointers(leaf_index);
            *min_ptr = min;
            *max_ptr = max;
            self.refit_for_node_bounds_change(self.leaves.get(leaf_index).node_index());
        }
    }

    /// Constructs an empty tree.
    pub fn new(pool: &mut BufferPool, initial_leaf_capacity: i32) -> Self {
        assert!(initial_leaf_capacity > 0, "Initial leaf capacity must be positive.");
        let mut tree = Self {
            nodes: Buffer::default(),
            metanodes: Buffer::default(),
            leaves: Buffer::default(),
            node_count: 0,
            leaf_count: 0,
        };
        tree.resize(pool, initial_leaf_capacity);
        tree
    }

    /// Creates an empty/default tree (not allocated).
    pub fn empty() -> Self {
        Self {
            nodes: Buffer::default(),
            metanodes: Buffer::default(),
            leaves: Buffer::default(),
            node_count: 0,
            leaf_count: 0,
        }
    }

    /// Gets the number of bytes required to store the tree in serialized form.
    pub fn get_serialized_byte_count(&self) -> usize {
        4 + std::mem::size_of::<Leaf>() * self.leaf_count as usize
            + (std::mem::size_of::<Node>() + std::mem::size_of::<Metanode>()) * self.node_count as usize
    }

    /// Serializes the tree into a byte buffer.
    pub unsafe fn serialize(&self, bytes: &mut [u8]) {
        let required = self.get_serialized_byte_count();
        assert!(bytes.len() >= required, "Target buffer too small.");

        *(bytes.as_mut_ptr() as *mut i32) = self.leaf_count;
        let leaf_byte_count = self.leaf_count as usize * std::mem::size_of::<Leaf>();
        let node_byte_count = self.node_count as usize * std::mem::size_of::<Node>();
        let metanode_byte_count = self.node_count as usize * std::mem::size_of::<Metanode>();

        let leaves_start = 4usize;
        let nodes_start = leaves_start + leaf_byte_count;
        let metanodes_start = nodes_start + node_byte_count;

        std::ptr::copy_nonoverlapping(
            self.leaves.as_ptr() as *const u8,
            bytes.as_mut_ptr().add(leaves_start),
            leaf_byte_count,
        );
        std::ptr::copy_nonoverlapping(
            self.nodes.as_ptr() as *const u8,
            bytes.as_mut_ptr().add(nodes_start),
            node_byte_count,
        );
        std::ptr::copy_nonoverlapping(
            self.metanodes.as_ptr() as *const u8,
            bytes.as_mut_ptr().add(metanodes_start),
            metanode_byte_count,
        );
    }

    /// Deserializes a tree from a byte buffer.
    pub unsafe fn from_bytes(data: &[u8], pool: &mut BufferPool) -> Self {
        assert!(data.len() > 4, "Data too small for header.");
        let leaf_count = *(data.as_ptr() as *const i32);
        let node_count = leaf_count - 1;
        let leaf_byte_count = leaf_count as usize * std::mem::size_of::<Leaf>();
        let node_byte_count = node_count as usize * std::mem::size_of::<Node>();
        let metanode_byte_count = node_count as usize * std::mem::size_of::<Metanode>();
        let leaves_start = 4usize;
        let nodes_start = leaves_start + leaf_byte_count;
        let metanodes_start = nodes_start + node_byte_count;
        assert!(
            data.len() >= leaves_start + leaf_byte_count + node_byte_count + metanode_byte_count,
            "Not enough data for the specified leaf count."
        );

        let leaves = pool.take_at_least::<Leaf>(leaf_count);
        let nodes = pool.take_at_least::<Node>(node_count);
        let metanodes = pool.take_at_least::<Metanode>(node_count);

        std::ptr::copy_nonoverlapping(
            data.as_ptr().add(leaves_start),
            leaves.as_ptr() as *mut u8,
            leaf_byte_count,
        );
        std::ptr::copy_nonoverlapping(
            data.as_ptr().add(nodes_start),
            nodes.as_ptr() as *mut u8,
            node_byte_count,
        );
        std::ptr::copy_nonoverlapping(
            data.as_ptr().add(metanodes_start),
            metanodes.as_ptr() as *mut u8,
            metanode_byte_count,
        );

        Self {
            nodes,
            metanodes,
            leaves,
            node_count,
            leaf_count,
        }
    }

    #[inline(always)]
    fn initialize_root(&mut self) {
        // The root always exists, even if there are no children in it. Makes some bookkeeping simpler.
        self.node_count = 1;
        let root_metanode = self.metanodes.get_mut(0);
        root_metanode.parent = -1;
        root_metanode.index_in_parent = -1;
    }

    /// Resizes the buffers backing the tree's nodes and leaves.
    /// Will not shrink the buffers below the size needed by the currently resident nodes and leaves.
    pub fn resize(&mut self, pool: &mut BufferPool, target_leaf_slot_count: i32) {
        let leaf_capacity_for_target =
            BufferPool::get_capacity_for_count::<Leaf>(self.leaf_count.max(target_leaf_slot_count));
        let node_capacity_for_target = BufferPool::get_capacity_for_count::<Node>(
            self.node_count.max(leaf_capacity_for_target - 1),
        );
        let metanode_capacity_for_target = BufferPool::get_capacity_for_count::<Metanode>(
            self.node_count.max(leaf_capacity_for_target - 1),
        );
        let was_allocated = self.leaves.allocated();
        debug_assert!(self.leaves.allocated() == self.nodes.allocated());

        if leaf_capacity_for_target != self.leaves.len() {
            pool.resize_to_at_least(&mut self.leaves, leaf_capacity_for_target, self.leaf_count);
        }
        if node_capacity_for_target != self.nodes.len() {
            pool.resize_to_at_least(&mut self.nodes, node_capacity_for_target, self.node_count);
        }
        if metanode_capacity_for_target != self.metanodes.len() {
            pool.resize_to_at_least(
                &mut self.metanodes,
                metanode_capacity_for_target,
                self.node_count,
            );
            // A node's RefineFlag must be 0, so clear out the unused portion.
            unsafe {
                let clear_start = self.node_count as usize;
                let clear_count = self.nodes.len() as usize - clear_start;
                if clear_count > 0 {
                    std::ptr::write_bytes(
                        (self.metanodes.as_mut_ptr()).add(clear_start),
                        0,
                        clear_count,
                    );
                }
            }
        }
        if !was_allocated {
            self.initialize_root();
        }
    }

    /// Resets the tree to a fresh post-construction state, clearing out leaves and nodes
    /// but leaving the backing resources intact.
    pub fn clear(&mut self) {
        self.leaf_count = 0;
        self.initialize_root();
    }

    /// Disposes the tree's backing resources, returning them to the pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        debug_assert!(
            self.nodes.allocated() == self.leaves.allocated()
                && self.nodes.allocated() == self.metanodes.allocated(),
            "Nodes and leaves should have consistent lifetimes."
        );
        if self.nodes.allocated() {
            pool.return_buffer(&mut self.nodes);
            pool.return_buffer(&mut self.metanodes);
            pool.return_buffer(&mut self.leaves);
        }
    }

    /// Tests if two tree references point to the same data.
    #[inline(always)]
    pub fn equals(a: &Tree, b: &Tree) -> bool {
        a.nodes.as_ptr() == b.nodes.as_ptr() && a.node_count == b.node_count
    }

    /// Computes the SAH (surface area heuristic) metric for a bounding box.
    #[inline(never)]
    pub fn compute_bounds_metric_bb(bounds: &BoundingBox) -> f32 {
        Self::compute_bounds_metric_vecs(&bounds.min, &bounds.max)
    }

    /// Computes the SAH (surface area heuristic) metric for a bounding box specified by min/max vectors.
    #[inline(never)]
    pub fn compute_bounds_metric_vecs(min: &Vec3, max: &Vec3) -> f32 {
        // Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
        let offset = *max - *min;
        offset.x * offset.y + offset.y * offset.z + offset.x * offset.z
    }

    /// Computes SAH metric for a type with the same memory layout as BoundingBox4 (32 bytes: min4 + max4).
    #[inline(never)]
    pub fn compute_bounds_metric<T>(bounds: &T) -> f32 {
        debug_assert!(std::mem::size_of::<T>() == 32);
        unsafe {
            let ptr = bounds as *const T as *const f32;
            let min = Vec3::new(*ptr, *ptr.add(1), *ptr.add(2));
            let max = Vec3::new(*ptr.add(4), *ptr.add(5), *ptr.add(6));
            Self::compute_bounds_metric_vecs(&min, &max)
        }
    }

    /// Gets a reference to the node child for a given child index (0 = a, 1 = b).
    #[inline(always)]
    pub unsafe fn node_child(node: &Node, index: i32) -> &NodeChild {
        &*(&node.a as *const NodeChild).add(index as usize)
    }

    /// Gets a mutable reference to the node child for a given child index (0 = a, 1 = b).
    #[inline(always)]
    pub unsafe fn node_child_mut(node: &mut Node, index: i32) -> &mut NodeChild {
        &mut *(&mut node.a as *mut NodeChild).add(index as usize)
    }

    /// Gets a reference to the node child representing the leaf within the tree.
    pub fn get_node_child_for_leaf_ref(&self, leaf: &Leaf) -> &NodeChild {
        debug_assert!(leaf.child_index() == 0 || leaf.child_index() == 1);
        unsafe { Self::node_child(self.nodes.get(leaf.node_index()), leaf.child_index()) }
    }

    /// Gets a reference to the node child representing the leaf at `leaf_index` within the tree.
    pub fn get_node_child_for_leaf(&self, leaf_index: i32) -> &NodeChild {
        self.get_node_child_for_leaf_ref(self.leaves.get(leaf_index))
    }
}
