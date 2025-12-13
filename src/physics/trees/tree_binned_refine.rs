// Translated from BepuPhysics/Trees/Tree_BinnedRefine.cs

use super::leaf::Leaf;
use super::node::{Node, NodeChild};
use super::tree::Tree;
use super::tree_refine_common::SubtreeHeapEntry;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;
use std::mem;

/// Maximum number of bins used in binned partitioning.
const MAXIMUM_BIN_COUNT: i32 = 64;

/// Pre-allocated resources for binned refinement operations.
/// All pointers are suballocated from a single buffer to minimize pool overhead.
#[repr(C)]
pub struct BinnedResources {
    pub bounding_boxes: *mut BoundingBox,
    pub leaf_counts: *mut i32,
    pub index_map: *mut i32,
    pub centroids: *mut Vec3,

    pub subtree_heap_entries: *mut SubtreeHeapEntry,
    pub staging_nodes: *mut Node,
    pub refine_flags: *mut i32,

    // Subtree related reusable resources.
    pub subtree_bin_indices_x: *mut i32,
    pub subtree_bin_indices_y: *mut i32,
    pub subtree_bin_indices_z: *mut i32,
    pub temp_index_map: *mut i32,

    pub a_leaf_counts_x: *mut i32,
    pub a_leaf_counts_y: *mut i32,
    pub a_leaf_counts_z: *mut i32,
    pub a_merged_x: *mut BoundingBox,
    pub a_merged_y: *mut BoundingBox,
    pub a_merged_z: *mut BoundingBox,

    // Bin-space reusable resources.
    pub bin_bounding_boxes_x: *mut BoundingBox,
    pub bin_bounding_boxes_y: *mut BoundingBox,
    pub bin_bounding_boxes_z: *mut BoundingBox,
    pub bin_leaf_counts_x: *mut i32,
    pub bin_leaf_counts_y: *mut i32,
    pub bin_leaf_counts_z: *mut i32,
    pub bin_subtree_counts_x: *mut i32,
    pub bin_subtree_counts_y: *mut i32,
    pub bin_subtree_counts_z: *mut i32,

    pub bin_start_indices: *mut i32,
    pub bin_subtree_counts_second_pass: *mut i32,
}

impl Tree {
    #[inline(always)]
    unsafe fn suballocate(memory: *mut u8, memory_allocated: &mut i32, byte_count: i32) -> *mut u8 {
        let new_size = *memory_allocated + (byte_count + 15) & !0xF; // 16-byte alignment
        let to_return = memory.add(*memory_allocated as usize);
        *memory_allocated = new_size;
        to_return
    }

    pub fn create_binned_resources(
        &self,
        pool: &mut BufferPool,
        maximum_subtree_count: i32,
    ) -> (Buffer<u8>, BinnedResources) {
        let node_count = maximum_subtree_count - 1;
        let bytes_required = 16 * (3 + 3 + 1)
            + mem::size_of::<BoundingBox>() as i32
                * (maximum_subtree_count + 3 * node_count + 3 * MAXIMUM_BIN_COUNT)
            + 16 * (6 + 3 + 8)
            + mem::size_of::<i32>() as i32
                * (maximum_subtree_count * 6 + node_count * 3 + MAXIMUM_BIN_COUNT * 8)
            + 16 * 1
            + mem::size_of::<Vec3>() as i32 * maximum_subtree_count
            + 16 * 1
            + mem::size_of::<SubtreeHeapEntry>() as i32 * maximum_subtree_count
            + 16 * 1
            + mem::size_of::<Node>() as i32 * node_count
            + 16 * 1
            + mem::size_of::<i32>() as i32 * node_count;

        let buffer: Buffer<u8> = pool.take_at_least(bytes_required);
        let memory = buffer.as_ptr() as *mut u8;
        let mut memory_allocated = 0i32;

        unsafe {
            let resources = BinnedResources {
                bounding_boxes: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * maximum_subtree_count,
                ) as *mut BoundingBox,
                leaf_counts: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                index_map: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                centroids: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<Vec3>() as i32 * maximum_subtree_count,
                ) as *mut Vec3,
                subtree_heap_entries: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<SubtreeHeapEntry>() as i32 * maximum_subtree_count,
                ) as *mut SubtreeHeapEntry,
                staging_nodes: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<Node>() as i32 * node_count,
                ) as *mut Node,
                refine_flags: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * node_count,
                ) as *mut i32,
                subtree_bin_indices_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                subtree_bin_indices_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                subtree_bin_indices_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                temp_index_map: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * maximum_subtree_count,
                ) as *mut i32,
                a_leaf_counts_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * node_count,
                ) as *mut i32,
                a_leaf_counts_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * node_count,
                ) as *mut i32,
                a_leaf_counts_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * node_count,
                ) as *mut i32,
                a_merged_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * node_count,
                ) as *mut BoundingBox,
                a_merged_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * node_count,
                ) as *mut BoundingBox,
                a_merged_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * node_count,
                ) as *mut BoundingBox,
                bin_bounding_boxes_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut BoundingBox,
                bin_bounding_boxes_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut BoundingBox,
                bin_bounding_boxes_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<BoundingBox>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut BoundingBox,
                bin_leaf_counts_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_leaf_counts_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_leaf_counts_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_subtree_counts_x: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_subtree_counts_y: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_subtree_counts_z: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_start_indices: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
                bin_subtree_counts_second_pass: Self::suballocate(
                    memory,
                    &mut memory_allocated,
                    mem::size_of::<i32>() as i32 * MAXIMUM_BIN_COUNT,
                ) as *mut i32,
            };

            debug_assert!(
                memory_allocated <= buffer.len(),
                "The allocated buffer should be large enough for all the suballocations."
            );

            (buffer, resources)
        }
    }

    unsafe fn find_partition_binned(
        &self,
        resources: &mut BinnedResources,
        start: i32,
        count: i32,
        split_index: &mut i32,
        a: &mut BoundingBox,
        b: &mut BoundingBox,
        leaf_count_a: &mut i32,
        leaf_count_b: &mut i32,
    ) {
        let local_index_map = resources.index_map.add(start as usize);

        let mut centroid_bounding_box = BoundingBox {
            min: *resources.centroids.add(*local_index_map as usize),
            max: Vec3::ZERO,
        };
        centroid_bounding_box.max = centroid_bounding_box.min;

        for i in 1..count {
            let centroid = &*resources.centroids.add(*local_index_map.add(i as usize) as usize);
            centroid_bounding_box.min = centroid_bounding_box.min.min(*centroid);
            centroid_bounding_box.max = centroid_bounding_box.max.max(*centroid);
        }

        let null_bounding_box = BoundingBox {
            min: Vec3::splat(f32::MAX),
            max: Vec3::splat(f32::MIN),
        };
        let span = centroid_bounding_box.max - centroid_bounding_box.min;
        const EPSILON: f32 = 1e-12;
        if span.x < EPSILON && span.y < EPSILON && span.z < EPSILON {
            // All axes are degenerate. Short circuit with midpoint split.
            *split_index = count / 2;
            *a = null_bounding_box;
            *b = null_bounding_box;
            *leaf_count_a = 0;
            *leaf_count_b = 0;
            for i in 0..*split_index {
                let idx = *local_index_map.add(i as usize);
                BoundingBox::create_merged_boxes(*a, *resources.bounding_boxes.add(idx as usize), a);
                *leaf_count_a += *resources.leaf_counts.add(idx as usize);
            }
            for i in *split_index..count {
                let idx = *local_index_map.add(i as usize);
                BoundingBox::create_merged_boxes(*b, *resources.bounding_boxes.add(idx as usize), b);
                *leaf_count_b += *resources.leaf_counts.add(idx as usize);
            }
            *split_index += start;
            return;
        }

        let bin_count = MAXIMUM_BIN_COUNT.min((count as f32 * 0.25).max(2.0) as i32);

        let inverse_bin_size = Vec3::new(
            if span.x > EPSILON { bin_count as f32 / span.x } else { 0.0 },
            if span.y > EPSILON { bin_count as f32 / span.y } else { 0.0 },
            if span.z > EPSILON { bin_count as f32 / span.z } else { 0.0 },
        );
        let maximum_bin_index = Vec3::splat((bin_count - 1) as f32);

        // Initialize bin information.
        for i in 0..bin_count {
            *resources.bin_bounding_boxes_x.add(i as usize) = null_bounding_box;
            *resources.bin_bounding_boxes_y.add(i as usize) = null_bounding_box;
            *resources.bin_bounding_boxes_z.add(i as usize) = null_bounding_box;
            *resources.bin_subtree_counts_x.add(i as usize) = 0;
            *resources.bin_subtree_counts_y.add(i as usize) = 0;
            *resources.bin_subtree_counts_z.add(i as usize) = 0;
            *resources.bin_leaf_counts_x.add(i as usize) = 0;
            *resources.bin_leaf_counts_y.add(i as usize) = 0;
            *resources.bin_leaf_counts_z.add(i as usize) = 0;
        }

        // Allocate subtrees to bins for all axes simultaneously.
        for i in 0..count {
            let subtree_index = *local_index_map.add(i as usize);
            let bin_indices = ((*resources.centroids.add(subtree_index as usize) - centroid_bounding_box.min)
                * inverse_bin_size)
                .min(maximum_bin_index);
            let x = bin_indices.x as i32;
            let y = bin_indices.y as i32;
            let z = bin_indices.z as i32;

            *resources.subtree_bin_indices_x.add(i as usize) = x;
            *resources.subtree_bin_indices_y.add(i as usize) = y;
            *resources.subtree_bin_indices_z.add(i as usize) = z;

            let leaf_count = *resources.leaf_counts.add(subtree_index as usize);
            let subtree_bb = &*resources.bounding_boxes.add(subtree_index as usize);

            *resources.bin_leaf_counts_x.add(x as usize) += leaf_count;
            *resources.bin_leaf_counts_y.add(y as usize) += leaf_count;
            *resources.bin_leaf_counts_z.add(z as usize) += leaf_count;

            *resources.bin_subtree_counts_x.add(x as usize) += 1;
            *resources.bin_subtree_counts_y.add(y as usize) += 1;
            *resources.bin_subtree_counts_z.add(z as usize) += 1;

            let bb_x = &mut *resources.bin_bounding_boxes_x.add(x as usize);
            BoundingBox::create_merged_boxes(*bb_x, *subtree_bb, bb_x);
            let bb_y = &mut *resources.bin_bounding_boxes_y.add(y as usize);
            BoundingBox::create_merged_boxes(*bb_y, *subtree_bb, bb_y);
            let bb_z = &mut *resources.bin_bounding_boxes_z.add(z as usize);
            BoundingBox::create_merged_boxes(*bb_z, *subtree_bb, bb_z);
        }

        // Sweep from low to high.
        let last_index = bin_count - 1;

        *resources.a_leaf_counts_x = *resources.bin_leaf_counts_x;
        *resources.a_leaf_counts_y = *resources.bin_leaf_counts_y;
        *resources.a_leaf_counts_z = *resources.bin_leaf_counts_z;
        *resources.a_merged_x = *resources.bin_bounding_boxes_x;
        *resources.a_merged_y = *resources.bin_bounding_boxes_y;
        *resources.a_merged_z = *resources.bin_bounding_boxes_z;

        for i in 1..last_index {
            let prev = (i - 1) as usize;
            let cur = i as usize;
            *resources.a_leaf_counts_x.add(cur) =
                *resources.bin_leaf_counts_x.add(cur) + *resources.a_leaf_counts_x.add(prev);
            *resources.a_leaf_counts_y.add(cur) =
                *resources.bin_leaf_counts_y.add(cur) + *resources.a_leaf_counts_y.add(prev);
            *resources.a_leaf_counts_z.add(cur) =
                *resources.bin_leaf_counts_z.add(cur) + *resources.a_leaf_counts_z.add(prev);
            BoundingBox::create_merged_boxes(
                *resources.a_merged_x.add(prev),
                *resources.bin_bounding_boxes_x.add(cur),
                &mut *resources.a_merged_x.add(cur),
            );
            BoundingBox::create_merged_boxes(
                *resources.a_merged_y.add(prev),
                *resources.bin_bounding_boxes_y.add(cur),
                &mut *resources.a_merged_y.add(cur),
            );
            BoundingBox::create_merged_boxes(
                *resources.a_merged_z.add(prev),
                *resources.bin_bounding_boxes_z.add(cur),
                &mut *resources.a_merged_z.add(cur),
            );
        }

        // Sweep from high to low.
        let mut b_merged_x = null_bounding_box;
        let mut b_merged_y = null_bounding_box;
        let mut b_merged_z = null_bounding_box;
        let mut b_leaf_count_x = 0i32;
        let mut b_leaf_count_y = 0i32;
        let mut b_leaf_count_z = 0i32;

        let mut best_axis = 0i32;
        let mut cost = f32::MAX;
        let mut bin_split_index = 0i32;
        *a = null_bounding_box;
        *b = null_bounding_box;
        *leaf_count_a = 0;
        *leaf_count_b = 0;

        for i in (1..=last_index).rev() {
            let a_index = (i - 1) as usize;
            let iu = i as usize;
            BoundingBox::create_merged_boxes(b_merged_x, *resources.bin_bounding_boxes_x.add(iu), &mut b_merged_x);
            BoundingBox::create_merged_boxes(b_merged_y, *resources.bin_bounding_boxes_y.add(iu), &mut b_merged_y);
            BoundingBox::create_merged_boxes(b_merged_z, *resources.bin_bounding_boxes_z.add(iu), &mut b_merged_z);
            b_leaf_count_x += *resources.bin_leaf_counts_x.add(iu);
            b_leaf_count_y += *resources.bin_leaf_counts_y.add(iu);
            b_leaf_count_z += *resources.bin_leaf_counts_z.add(iu);

            let cost_candidate_x = if b_leaf_count_x > 0 && *resources.a_leaf_counts_x.add(a_index) > 0 {
                let metric_a = Self::compute_bounds_metric_bb(&*resources.a_merged_x.add(a_index));
                let metric_b = Self::compute_bounds_metric_bb(&b_merged_x);
                *resources.a_leaf_counts_x.add(a_index) as f32 * metric_a + b_leaf_count_x as f32 * metric_b
            } else {
                f32::MAX
            };
            let cost_candidate_y = if b_leaf_count_y > 0 && *resources.a_leaf_counts_y.add(a_index) > 0 {
                let metric_a = Self::compute_bounds_metric_bb(&*resources.a_merged_y.add(a_index));
                let metric_b = Self::compute_bounds_metric_bb(&b_merged_y);
                *resources.a_leaf_counts_y.add(a_index) as f32 * metric_a + b_leaf_count_y as f32 * metric_b
            } else {
                f32::MAX
            };
            let cost_candidate_z = if b_leaf_count_z > 0 && *resources.a_leaf_counts_z.add(a_index) > 0 {
                let metric_a = Self::compute_bounds_metric_bb(&*resources.a_merged_z.add(a_index));
                let metric_b = Self::compute_bounds_metric_bb(&b_merged_z);
                *resources.a_leaf_counts_z.add(a_index) as f32 * metric_a + b_leaf_count_z as f32 * metric_b
            } else {
                f32::MAX
            };

            if cost_candidate_x < cost_candidate_y && cost_candidate_x < cost_candidate_z {
                if cost_candidate_x < cost {
                    best_axis = 0;
                    cost = cost_candidate_x;
                    bin_split_index = i;
                    *a = *resources.a_merged_x.add(a_index);
                    *b = b_merged_x;
                    *leaf_count_a = *resources.a_leaf_counts_x.add(a_index);
                    *leaf_count_b = b_leaf_count_x;
                }
            } else if cost_candidate_y < cost_candidate_z {
                if cost_candidate_y < cost {
                    best_axis = 1;
                    cost = cost_candidate_y;
                    bin_split_index = i;
                    *a = *resources.a_merged_y.add(a_index);
                    *b = b_merged_y;
                    *leaf_count_a = *resources.a_leaf_counts_y.add(a_index);
                    *leaf_count_b = b_leaf_count_y;
                }
            } else {
                if cost_candidate_z < cost {
                    best_axis = 2;
                    cost = cost_candidate_z;
                    bin_split_index = i;
                    *a = *resources.a_merged_z.add(a_index);
                    *b = b_merged_z;
                    *leaf_count_a = *resources.a_leaf_counts_z.add(a_index);
                    *leaf_count_b = b_leaf_count_z;
                }
            }
        }

        let (best_bin_subtree_counts, best_subtree_bin_indices) = match best_axis {
            0 => (resources.bin_subtree_counts_x, resources.subtree_bin_indices_x),
            1 => (resources.bin_subtree_counts_y, resources.subtree_bin_indices_y),
            _ => (resources.bin_subtree_counts_z, resources.subtree_bin_indices_z),
        };

        // Rebuild the index map.
        *resources.bin_start_indices = 0;
        *resources.bin_subtree_counts_second_pass = 0;
        for i in 1..bin_count {
            *resources.bin_start_indices.add(i as usize) =
                *resources.bin_start_indices.add((i - 1) as usize)
                    + *best_bin_subtree_counts.add((i - 1) as usize);
            *resources.bin_subtree_counts_second_pass.add(i as usize) = 0;
        }

        for i in 0..count {
            let index = *best_subtree_bin_indices.add(i as usize);
            let second_pass = &mut *resources.bin_subtree_counts_second_pass.add(index as usize);
            *resources.temp_index_map.add(
                (*resources.bin_start_indices.add(index as usize) + *second_pass) as usize,
            ) = *local_index_map.add(i as usize);
            *second_pass += 1;
        }

        // Update the real index map.
        for i in 0..count {
            *local_index_map.add(i as usize) = *resources.temp_index_map.add(i as usize);
        }
        *split_index = *resources.bin_start_indices.add(bin_split_index as usize) + start;
    }

    unsafe fn split_subtrees_into_children_binned(
        &self,
        resources: &mut BinnedResources,
        start: i32,
        count: i32,
        staging_node_index: i32,
        staging_nodes_count: &mut i32,
        children_treelets_cost: &mut f32,
    ) {
        debug_assert!(count > 2);
        let mut split_index = 0i32;
        let mut a_bounds = BoundingBox::default();
        let mut b_bounds = BoundingBox::default();
        let mut leaf_count_a = 0i32;
        let mut leaf_count_b = 0i32;
        self.find_partition_binned(
            resources,
            start,
            count,
            &mut split_index,
            &mut a_bounds,
            &mut b_bounds,
            &mut leaf_count_a,
            &mut leaf_count_b,
        );

        let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
        staging_node.a.min = a_bounds.min;
        staging_node.a.max = a_bounds.max;
        staging_node.b.min = b_bounds.min;
        staging_node.b.max = b_bounds.max;
        staging_node.a.leaf_count = leaf_count_a;
        staging_node.b.leaf_count = leaf_count_b;

        let subtree_count_a = split_index - start;
        let subtree_count_b = start + count - split_index;

        let cost_a;
        if subtree_count_a > 1 {
            let idx = self.create_staging_node_binned(
                resources,
                start,
                subtree_count_a,
                staging_nodes_count,
                &mut 0.0,
            );
            // Re-borrow staging node
            let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
            staging_node.a.index = idx;
            cost_a = Self::compute_bounds_metric_bb(&a_bounds);
        } else {
            debug_assert_eq!(subtree_count_a, 1);
            let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
            staging_node.a.index = Self::encode(*resources.index_map.add(start as usize));
            cost_a = 0.0;
        }

        let cost_b;
        if subtree_count_b > 1 {
            let idx = self.create_staging_node_binned(
                resources,
                split_index,
                subtree_count_b,
                staging_nodes_count,
                &mut 0.0,
            );
            let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
            staging_node.b.index = idx;
            cost_b = Self::compute_bounds_metric_bb(&b_bounds);
        } else {
            debug_assert_eq!(subtree_count_b, 1);
            let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
            staging_node.b.index = Self::encode(*resources.index_map.add(split_index as usize));
            cost_b = 0.0;
        }
        *children_treelets_cost = cost_a + cost_b;
    }

    unsafe fn create_staging_node_binned(
        &self,
        resources: &mut BinnedResources,
        start: i32,
        count: i32,
        staging_node_count: &mut i32,
        child_treelets_cost: &mut f32,
    ) -> i32 {
        let staging_node_index = *staging_node_count;
        *staging_node_count += 1;

        if count <= 2 {
            let local_index_map = resources.index_map.add(start as usize);
            let staging_node = &mut *resources.staging_nodes.add(staging_node_index as usize);
            let staging_children = &mut staging_node.a as *mut NodeChild;
            for i in 0..count {
                let subtree_index = *local_index_map.add(i as usize);
                let child = &mut *staging_children.add(i as usize);
                let bounds = &*resources.bounding_boxes.add(subtree_index as usize);
                child.min = bounds.min;
                child.max = bounds.max;
                child.leaf_count = *resources.leaf_counts.add(subtree_index as usize);
                child.index = Self::encode(subtree_index);
            }
            *child_treelets_cost = 0.0;
            return staging_node_index;
        }

        self.split_subtrees_into_children_binned(
            resources,
            start,
            count,
            staging_node_index,
            staging_node_count,
            child_treelets_cost,
        );

        staging_node_index
    }

    #[inline(always)]
    unsafe fn reify_children(
        &mut self,
        internal_node_index: i32,
        staging_nodes: *mut Node,
        subtrees: &mut QuickList<i32>,
        treelet_internal_nodes: &mut QuickList<i32>,
        next_internal_node_index_to_use: &mut i32,
    ) {
        debug_assert!(subtrees.count > 1);
        let internal_node = &mut *(self.nodes.as_ptr() as *mut Node).add(internal_node_index as usize);
        let internal_node_children = &mut internal_node.a as *mut NodeChild;
        for i in 0..2i32 {
            let child = &mut *internal_node_children.add(i as usize);
            if child.index >= 0 {
                child.index = self.reify_staging_node(
                    internal_node_index,
                    i,
                    staging_nodes,
                    child.index,
                    subtrees,
                    treelet_internal_nodes,
                    next_internal_node_index_to_use,
                );
            } else {
                // It's a subtree. Update its pointers.
                let subtree_index = subtrees[Self::encode(child.index)];
                child.index = subtree_index;
                if subtree_index >= 0 {
                    debug_assert!(subtree_index >= 0 && subtree_index < self.node_count);
                    let metanode = self.metanodes.get_mut(subtree_index);
                    metanode.index_in_parent = i;
                    metanode.parent = internal_node_index;
                } else {
                    let leaf_index = Self::encode(subtree_index);
                    debug_assert!(leaf_index >= 0 && leaf_index < self.leaf_count);
                    *self.leaves.get_mut(leaf_index) = Leaf::new(internal_node_index, i);
                }
            }
        }
    }

    unsafe fn reify_staging_node(
        &mut self,
        parent: i32,
        index_in_parent: i32,
        staging_nodes: *mut Node,
        staging_node_index: i32,
        subtrees: &mut QuickList<i32>,
        treelet_internal_nodes: &mut QuickList<i32>,
        next_internal_node_index_to_use: &mut i32,
    ) -> i32 {
        debug_assert!(
            *next_internal_node_index_to_use < treelet_internal_nodes.count,
            "Binary trees should never run out of available internal nodes when reifying staging nodes."
        );

        let internal_node_index = treelet_internal_nodes[*next_internal_node_index_to_use];
        *next_internal_node_index_to_use += 1;

        // Copy staging node into real tree.
        let staging_node = &*staging_nodes.add(staging_node_index as usize);
        let internal_node =
            &mut *(self.nodes.as_ptr() as *mut Node).add(internal_node_index as usize);
        *internal_node = *staging_node;
        let metanode = self.metanodes.get_mut(internal_node_index);
        metanode.cost_or_flag.refine_flag = 0;
        metanode.parent = parent;
        metanode.index_in_parent = index_in_parent;

        self.reify_children(
            internal_node_index,
            staging_nodes,
            subtrees,
            treelet_internal_nodes,
            next_internal_node_index_to_use,
        );
        internal_node_index
    }

    unsafe fn reify_staging_nodes(
        &mut self,
        treelet_root_index: i32,
        staging_nodes: *mut Node,
        subtrees: &mut QuickList<i32>,
        treelet_internal_nodes: &mut QuickList<i32>,
        next_internal_node_index_to_use: &mut i32,
    ) {
        let staging_root = &*staging_nodes;
        let internal_node =
            &mut *(self.nodes.as_ptr() as *mut Node).add(treelet_root_index as usize);
        internal_node.a = staging_root.a;
        internal_node.b = staging_root.b;
        self.reify_children(
            treelet_root_index,
            staging_nodes,
            subtrees,
            treelet_internal_nodes,
            next_internal_node_index_to_use,
        );
    }

    pub fn binned_refine(
        &mut self,
        node_index: i32,
        subtree_references: &mut QuickList<i32>,
        maximum_subtrees: i32,
        treelet_internal_nodes: &mut QuickList<i32>,
        resources: &mut BinnedResources,
        pool: &mut BufferPool,
    ) {
        debug_assert_eq!(
            subtree_references.count, 0,
            "The subtree references list should be empty."
        );
        debug_assert_eq!(
            treelet_internal_nodes.count, 0,
            "The treelet internal nodes list should be empty."
        );

        unsafe {
            let mut original_treelet_cost = 0.0f32;
            self.collect_subtrees(
                node_index,
                maximum_subtrees,
                resources.subtree_heap_entries,
                subtree_references,
                treelet_internal_nodes,
                &mut original_treelet_cost,
            );

            debug_assert_eq!(
                treelet_internal_nodes.count,
                subtree_references.count - 2,
                "Binary tree internal node count must match subtree references minus two."
            );
            debug_assert!(subtree_references.count <= maximum_subtrees);

            // Gather necessary information from nodes.
            for i in 0..subtree_references.count {
                *resources.index_map.add(i as usize) = i;
                let subtree_ref = subtree_references[i];
                if subtree_ref >= 0 {
                    // Internal node.
                    let metanode = &*self.metanodes.get(subtree_ref);
                    let parent_node = &*self.nodes.get(metanode.parent);
                    let owning_child = Self::node_child(parent_node, metanode.index_in_parent);
                    let target_bounds = &mut *resources.bounding_boxes.add(i as usize);
                    target_bounds.min = owning_child.min;
                    target_bounds.max = owning_child.max;
                    *resources.centroids.add(i as usize) = owning_child.min + owning_child.max;
                    *resources.leaf_counts.add(i as usize) = owning_child.leaf_count;
                } else {
                    // Leaf node.
                    let leaf = &*self.leaves.get(Self::encode(subtree_ref));
                    let owning_child = Self::node_child(
                        &*self.nodes.get(leaf.node_index()),
                        leaf.child_index(),
                    );
                    let target_bounds = &mut *resources.bounding_boxes.add(i as usize);
                    target_bounds.min = owning_child.min;
                    target_bounds.max = owning_child.max;
                    *resources.centroids.add(i as usize) = owning_child.min + owning_child.max;
                    *resources.leaf_counts.add(i as usize) = 1;
                }
            }

            // Perform top-down binned build.
            let mut staging_node_count = 0i32;
            let mut new_treelet_cost = 0.0f32;
            self.create_staging_node_binned(
                resources,
                0,
                subtree_references.count,
                &mut staging_node_count,
                &mut new_treelet_cost,
            );

            // Copy refine flag from the treelet root so it persists.
            *resources.refine_flags = self.metanodes.get(node_index).cost_or_flag.refine_flag;

            // Apply staged nodes to real nodes.
            let mut next_internal_node_index_to_use = 0i32;
            self.reify_staging_nodes(
                node_index,
                resources.staging_nodes,
                subtree_references,
                treelet_internal_nodes,
                &mut next_internal_node_index_to_use,
            );
        }
    }
}
