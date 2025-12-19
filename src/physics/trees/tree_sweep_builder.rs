// Translated from BepuPhysics/Trees/Tree_SweepBuilder.cs

use super::leaf::Leaf;
use super::node::NodeChild;
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::quicksort::Quicksort;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;
use std::cmp::Ordering;

#[repr(C)]
pub(crate) struct SweepResources {
    pub bounds: *mut BoundingBox,
    pub index_map: *mut i32,
    pub index_map_x: *mut i32,
    pub index_map_y: *mut i32,
    pub index_map_z: *mut i32,
    pub centroids_x: *mut f32,
    pub centroids_y: *mut f32,
    pub centroids_z: *mut f32,
    pub merged: *mut BoundingBox,
}

struct IndexMapComparer {
    centroids: *const f32,
}

impl RefComparer<i32> for IndexMapComparer {
    #[inline(always)]
    fn compare(&self, a: &i32, b: &i32) -> Ordering {
        unsafe {
            let centroid_a = *self.centroids.add(*a as usize);
            let centroid_b = *self.centroids.add(*b as usize);
            centroid_a.partial_cmp(&centroid_b).unwrap_or(Ordering::Equal)
        }
    }
}

impl Tree {
    unsafe fn find_partition_for_axis(
        &self,
        bounding_boxes: *mut BoundingBox,
        a_merged: *mut BoundingBox,
        centroids: *mut f32,
        index_map: *mut i32,
        count: i32,
        split_index: &mut i32,
        cost: &mut f32,
        a: &mut BoundingBox,
        b: &mut BoundingBox,
        leaf_count_a: &mut i32,
        leaf_count_b: &mut i32,
    ) {
        debug_assert!(count > 1);

        let comparer = IndexMapComparer {
            centroids: centroids as *const f32,
        };
        Quicksort::sort_keys(std::slice::from_raw_parts_mut(index_map, count as usize), 0, count - 1, &comparer);

        // Search for the best split.
        // Sweep across from low to high, caching the merged size at each point.
        let last_index = count - 1;

        *a_merged = *bounding_boxes.add(*index_map as usize);
        for i in 1..last_index {
            let index = *index_map.add(i as usize);
            BoundingBox::create_merged_boxes(
                *a_merged.add((i - 1) as usize),
                *bounding_boxes.add(index as usize),
                &mut *a_merged.add(i as usize),
            );
        }

        // Sweep from high to low.
        let mut b_merged = BoundingBox {
            min: Vec3::splat(f32::MAX),
            _pad0: 0.0,
            max: Vec3::splat(f32::MIN),
            _pad1: 0.0,
        };
        *cost = f32::MAX;
        *split_index = 0;
        *a = b_merged;
        *b = b_merged;
        *leaf_count_a = 0;
        *leaf_count_b = 0;

        const NORMAL_EPSILON: f32 = 1.1754943508e-38;

        for i in (1..=last_index).rev() {
            let a_index = i - 1;
            let subtree_index = *index_map.add(i as usize);
            BoundingBox::create_merged_boxes(
                b_merged,
                *bounding_boxes.add(subtree_index as usize),
                &mut b_merged,
            );

            let i_f = i as f32;
            let a_cost = i_f
                * (1.0 + i_f * 0.001)
                * (NORMAL_EPSILON
                    + Tree::compute_bounds_metric_bb(&*a_merged.add(a_index as usize)));
            let b_count = count - i;
            let b_count_f = b_count as f32;
            let b_cost = b_count_f
                * (1.0 + b_count_f * 0.001)
                * (NORMAL_EPSILON + Tree::compute_bounds_metric_bb(&b_merged));

            let total_cost = a_cost + b_cost;
            if total_cost < *cost {
                *cost = total_cost;
                *split_index = i;
                *a = *a_merged.add(a_index as usize);
                *b = b_merged;
                *leaf_count_a = i;
                *leaf_count_b = count - i;
            }
        }
    }

    unsafe fn find_partition(
        &self,
        leaves: &mut SweepResources,
        start: i32,
        count: i32,
        split_index: &mut i32,
        a: &mut BoundingBox,
        b: &mut BoundingBox,
        leaf_count_a: &mut i32,
        leaf_count_b: &mut i32,
    ) {
        // Initialize the per-axis candidate maps.
        for i in 0..count {
            let original_value = *leaves.index_map.add((i + start) as usize);
            *leaves.index_map_x.add(i as usize) = original_value;
            *leaves.index_map_y.add(i as usize) = original_value;
            *leaves.index_map_z.add(i as usize) = original_value;
        }

        let (mut x_split, mut x_cost) = (0i32, 0.0f32);
        let (mut x_a, mut x_b) = (BoundingBox::default(), BoundingBox::default());
        let (mut x_leaf_count_a, mut x_leaf_count_b) = (0i32, 0i32);
        self.find_partition_for_axis(
            leaves.bounds,
            leaves.merged,
            leaves.centroids_x,
            leaves.index_map_x,
            count,
            &mut x_split,
            &mut x_cost,
            &mut x_a,
            &mut x_b,
            &mut x_leaf_count_a,
            &mut x_leaf_count_b,
        );

        let (mut y_split, mut y_cost) = (0i32, 0.0f32);
        let (mut y_a, mut y_b) = (BoundingBox::default(), BoundingBox::default());
        let (mut y_leaf_count_a, mut y_leaf_count_b) = (0i32, 0i32);
        self.find_partition_for_axis(
            leaves.bounds,
            leaves.merged,
            leaves.centroids_y,
            leaves.index_map_y,
            count,
            &mut y_split,
            &mut y_cost,
            &mut y_a,
            &mut y_b,
            &mut y_leaf_count_a,
            &mut y_leaf_count_b,
        );

        let (mut z_split, mut z_cost) = (0i32, 0.0f32);
        let (mut z_a, mut z_b) = (BoundingBox::default(), BoundingBox::default());
        let (mut z_leaf_count_a, mut z_leaf_count_b) = (0i32, 0i32);
        self.find_partition_for_axis(
            leaves.bounds,
            leaves.merged,
            leaves.centroids_z,
            leaves.index_map_z,
            count,
            &mut z_split,
            &mut z_cost,
            &mut z_a,
            &mut z_b,
            &mut z_leaf_count_a,
            &mut z_leaf_count_b,
        );

        let best_index_map: *mut i32;
        if x_cost <= y_cost && x_cost <= z_cost {
            *split_index = x_split;
            *a = x_a;
            *b = x_b;
            *leaf_count_a = x_leaf_count_a;
            *leaf_count_b = x_leaf_count_b;
            best_index_map = leaves.index_map_x;
        } else if y_cost <= z_cost {
            *split_index = y_split;
            *a = y_a;
            *b = y_b;
            *leaf_count_a = y_leaf_count_a;
            *leaf_count_b = y_leaf_count_b;
            best_index_map = leaves.index_map_y;
        } else {
            *split_index = z_split;
            *a = z_a;
            *b = z_b;
            *leaf_count_a = z_leaf_count_a;
            *leaf_count_b = z_leaf_count_b;
            best_index_map = leaves.index_map_z;
        }
        for i in 0..count {
            *leaves.index_map.add((i + start) as usize) = *best_index_map.add(i as usize);
        }

        *split_index += start;
    }

    unsafe fn split_leaves_into_children(
        &mut self,
        leaves: &mut SweepResources,
        start: i32,
        count: i32,
        node_index: i32,
    ) {
        debug_assert!(count >= 2);
        let mut split_index = 0i32;
        let mut a_bounds = BoundingBox::default();
        let mut b_bounds = BoundingBox::default();
        let mut leaf_count_a = 0i32;
        let mut leaf_count_b = 0i32;
        self.find_partition(
            leaves,
            start,
            count,
            &mut split_index,
            &mut a_bounds,
            &mut b_bounds,
            &mut leaf_count_a,
            &mut leaf_count_b,
        );

        let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node).add(node_index as usize);

        node.a.min = a_bounds.min;
        node.a.max = a_bounds.max;
        node.b.min = b_bounds.min;
        node.b.max = b_bounds.max;
        node.a.leaf_count = leaf_count_a;
        node.b.leaf_count = leaf_count_b;

        if leaf_count_a > 1 {
            node.a.index =
                self.create_sweep_builder_node(node_index, 0, leaves, start, leaf_count_a);
        } else {
            debug_assert_eq!(leaf_count_a, 1);
            let leaf_index = *leaves.index_map.add(start as usize);
            *self.leaves.get_mut(leaf_index) = Leaf::new(node_index, 0);
            node.a.index = Self::encode(leaf_index);
        }

        // Re-borrow node after potential recursive calls
        let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node).add(node_index as usize);
        if leaf_count_b > 1 {
            node.b.index =
                self.create_sweep_builder_node(-1, 1, leaves, split_index, leaf_count_b);
        } else {
            debug_assert_eq!(leaf_count_b, 1);
            let leaf_index = *leaves.index_map.add(split_index as usize);
            *self.leaves.get_mut(leaf_index) = Leaf::new(node_index, 1);
            node.b.index = Self::encode(leaf_index);
        }
    }

    unsafe fn create_sweep_builder_node(
        &mut self,
        parent_index: i32,
        index_in_parent: i32,
        leaves: &mut SweepResources,
        start: i32,
        count: i32,
    ) -> i32 {
        let node_index = self.allocate_node();
        {
            let metanode = self.metanodes.get_mut(node_index);
            metanode.parent = parent_index;
            metanode.index_in_parent = index_in_parent;
            metanode.cost_or_flag.refine_flag = 0;
        }

        if count <= 2 {
            // No need to do any sorting. This node can fit every remaining subtree.
            let children = &mut (*(self.nodes.as_ptr() as *mut super::node::Node)
                .add(node_index as usize))
                .a as *mut NodeChild;
            for i in 0..count {
                let leaf_index = *leaves.index_map.add((i + start) as usize);
                *self.leaves.get_mut(leaf_index) = Leaf::new(node_index, i);
                let child = &mut *children.add(i as usize);
                let bounds = &*leaves.bounds.add(leaf_index as usize);
                child.min = bounds.min;
                child.max = bounds.max;
                child.index = Self::encode(leaf_index);
                child.leaf_count = 1;
            }
            return node_index;
        }

        self.split_leaves_into_children(leaves, start, count, node_index);

        node_index
    }

    /// Builds the tree using a top-down sweep approach with SAH cost function.
    pub fn sweep_build(&mut self, pool: &mut BufferPool, leaf_bounds: &Buffer<BoundingBox>) {
        assert!(leaf_bounds.len() > 0, "Length must be positive.");

        // Clear node count for fresh build.
        self.node_count = 0;

        // Guarantee that no resizes will occur during the build.
        if self.leaves.len() < leaf_bounds.len() {
            self.resize(pool, leaf_bounds.len());
        }
        self.leaf_count = leaf_bounds.len();

        let mut index_map: Buffer<i32> = pool.take_at_least(leaf_bounds.len());
        let mut index_map_x: Buffer<i32> = pool.take_at_least(leaf_bounds.len());
        let mut index_map_y: Buffer<i32> = pool.take_at_least(leaf_bounds.len());
        let mut index_map_z: Buffer<i32> = pool.take_at_least(leaf_bounds.len());
        let mut centroids_x: Buffer<f32> = pool.take_at_least(leaf_bounds.len());
        let mut centroids_y: Buffer<f32> = pool.take_at_least(leaf_bounds.len());
        let mut centroids_z: Buffer<f32> = pool.take_at_least(leaf_bounds.len());
        let mut merged: Buffer<BoundingBox> = pool.take_at_least(leaf_bounds.len());

        unsafe {
            let mut leaves = SweepResources {
                bounds: leaf_bounds.as_ptr() as *mut BoundingBox,
                index_map: index_map.as_mut_ptr(),
                index_map_x: index_map_x.as_mut_ptr(),
                index_map_y: index_map_y.as_mut_ptr(),
                index_map_z: index_map_z.as_mut_ptr(),
                centroids_x: centroids_x.as_mut_ptr(),
                centroids_y: centroids_y.as_mut_ptr(),
                centroids_z: centroids_z.as_mut_ptr(),
                merged: merged.as_mut_ptr(),
            };

            for i in 0..leaf_bounds.len() {
                let bounds = &*leaves.bounds.add(i as usize);
                *leaves.index_map.add(i as usize) = i;

                let centroid = bounds.min + bounds.max;
                *centroids_x.get_mut(i) = centroid.x;
                *centroids_y.get_mut(i) = centroid.y;
                *centroids_z.get_mut(i) = centroid.z;
            }

            // Now perform a top-down sweep build.
            self.create_sweep_builder_node(-1, -1, &mut leaves, 0, leaf_bounds.len());
        }

        // Return resources.
        pool.return_buffer(&mut centroids_x);
        pool.return_buffer(&mut centroids_y);
        pool.return_buffer(&mut centroids_z);
        pool.return_buffer(&mut index_map);
        pool.return_buffer(&mut index_map_x);
        pool.return_buffer(&mut index_map_y);
        pool.return_buffer(&mut index_map_z);
        pool.return_buffer(&mut merged);
    }
}
