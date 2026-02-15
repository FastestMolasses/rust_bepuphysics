// Translated from BepuPhysics/Trees/Tree_VolumeQuery.cs

use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::for_each_ref::IBreakableForEach;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

impl Tree {
    unsafe fn get_overlaps_internal<TEnumerator: IBreakableForEach<i32>>(
        &self,
        mut node_index: i32,
        bounding_box: BoundingBox,
        mut stack: Buffer<i32>,
        pool: &mut BufferPool,
        leaf_enumerator: &mut TEnumerator,
    ) {
        debug_assert!(
            (node_index >= 0 && node_index < self.node_count)
                || (Self::encode(node_index) >= 0 && Self::encode(node_index) < self.leaf_count)
        );
        debug_assert!(
            self.leaf_count >= 2,
            "This implementation assumes all nodes are filled."
        );

        let mut stack_end: i32 = 0;
        loop {
            if node_index < 0 {
                // This is actually a leaf node.
                let leaf_index = Self::encode(node_index);
                if !leaf_enumerator.loop_body(leaf_index) {
                    break;
                }
                // Leaves have no children; pull from the stack.
                if stack_end == 0 {
                    break;
                }
                stack_end -= 1;
                node_index = *stack.get(stack_end);
            } else {
                let node = self.nodes.get(node_index);
                let a_intersected = BoundingBox::intersects_unsafe(&node.a, &bounding_box);
                let b_intersected = BoundingBox::intersects_unsafe(&node.b, &bounding_box);

                if a_intersected {
                    node_index = node.a.index;
                    if b_intersected {
                        if stack_end == stack.len() {
                            if stack.len() == TRAVERSAL_STACK_CAPACITY as i32 {
                                // First allocation is on the stack.
                                let mut new_stack: Buffer<i32> =
                                    pool.take_at_least(TRAVERSAL_STACK_CAPACITY as i32 * 2);
                                stack.copy_to(0, &mut new_stack, 0, TRAVERSAL_STACK_CAPACITY as i32);
                                stack = new_stack;
                            } else {
                                pool.resize(&mut stack, stack_end * 2, stack_end);
                            }
                        }
                        *stack.get_mut(stack_end) = node.b.index;
                        stack_end += 1;
                    }
                } else if b_intersected {
                    node_index = node.b.index;
                } else {
                    // No intersection. Pull from stack.
                    if stack_end == 0 {
                        break;
                    }
                    stack_end -= 1;
                    node_index = *stack.get(stack_end);
                }
            }
        }
        if stack.len() > TRAVERSAL_STACK_CAPACITY as i32 {
            // We rented a larger stack at some point. Return it.
            pool.return_buffer(&mut stack);
        }
    }

    /// Finds and processes all leaves with bounding boxes that overlap the specified axis-aligned bounding box.
    /// The leaf_enumerator is invoked for each overlapping element.
    ///
    /// # Arguments
    /// * `bounding_box` - Query to test against the bounding volume hierarchy.
    /// * `pool` - Buffer pool used for temporary allocations if the tree is pathologically deep; stack memory is used preferentially.
    /// * `leaf_enumerator` - A reference to the enumerator that processes the indices of overlapping elements.
    #[inline(always)]
    pub fn get_overlaps<TEnumerator: IBreakableForEach<i32>>(
        &self,
        bounding_box: BoundingBox,
        pool: &mut BufferPool,
        leaf_enumerator: &mut TEnumerator,
    ) {
        unsafe {
            if self.leaf_count > 1 {
                let mut stack_memory = [0i32; TRAVERSAL_STACK_CAPACITY];
                let stack = Buffer::new(
                    stack_memory.as_mut_ptr(),
                    TRAVERSAL_STACK_CAPACITY as i32,
                    -1,
                );
                self.get_overlaps_internal(0, bounding_box, stack, pool, leaf_enumerator);
            } else if self.leaf_count == 1 {
                debug_assert!(
                    self.nodes.get(0).a.index < 0,
                    "If the root only has one child, it must be a leaf."
                );
                if BoundingBox::intersects_unsafe(&bounding_box, &self.nodes.get(0).a) {
                    leaf_enumerator.loop_body(Self::encode(self.nodes.get(0).a.index));
                }
                return;
            }
            // If the leaf count is zero, there's nothing to test against.
        }
    }

    /// Gets all leaf indices whose bounding boxes overlap the given min/max bounds.
    #[inline(always)]
    pub fn get_overlaps_minmax<TEnumerator: IBreakableForEach<i32>>(
        &self,
        min: Vec3,
        max: Vec3,
        pool: &mut BufferPool,
        leaf_enumerator: &mut TEnumerator,
    ) {
        self.get_overlaps(BoundingBox::new(min, max), pool, leaf_enumerator);
    }
}
