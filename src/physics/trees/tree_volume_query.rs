// Translated from BepuPhysics/Trees/Tree_VolumeQuery.cs

use super::tree::{Tree, TRAVERSAL_STACK_CAPACITY};
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::for_each_ref::IBreakableForEach;
use glam::Vec3;

impl Tree {
    unsafe fn get_overlaps_internal<TEnumerator: IBreakableForEach<i32>>(
        &self,
        mut node_index: i32,
        bounding_box: BoundingBox,
        stack: *mut i32,
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
                    return;
                }
                // Leaves have no children; pull from the stack.
                if stack_end == 0 {
                    return;
                }
                stack_end -= 1;
                node_index = *stack.add(stack_end as usize);
            } else {
                let node = self.nodes.get(node_index);
                let a_intersected = BoundingBox::intersects_unsafe(&node.a, &bounding_box);
                let b_intersected = BoundingBox::intersects_unsafe(&node.b, &bounding_box);

                if a_intersected {
                    node_index = node.a.index;
                    if b_intersected {
                        debug_assert!(
                            (stack_end as usize) < TRAVERSAL_STACK_CAPACITY - 1,
                            "Fixed size stack overflow."
                        );
                        *stack.add(stack_end as usize) = node.b.index;
                        stack_end += 1;
                    }
                } else if b_intersected {
                    node_index = node.b.index;
                } else {
                    // No intersection. Pull from stack.
                    if stack_end == 0 {
                        return;
                    }
                    stack_end -= 1;
                    node_index = *stack.add(stack_end as usize);
                }
            }
        }
    }

    /// Gets all leaf indices whose bounding boxes overlap the given bounding box.
    #[inline(always)]
    pub fn get_overlaps<TEnumerator: IBreakableForEach<i32>>(
        &self,
        bounding_box: BoundingBox,
        leaf_enumerator: &mut TEnumerator,
    ) {
        unsafe {
            if self.leaf_count > 1 {
                let mut stack = [0i32; TRAVERSAL_STACK_CAPACITY];
                self.get_overlaps_internal(0, bounding_box, stack.as_mut_ptr(), leaf_enumerator);
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
        leaf_enumerator: &mut TEnumerator,
    ) {
        self.get_overlaps(BoundingBox::new(min, max), leaf_enumerator);
    }
}
