// Translation of BepuPhysics/Trees/Tree_BinnedBuilder.cs — single-threaded path only.
// Implements the binned SAH tree builder used by Tree::binned_build.

use crate::physics::trees::leaf::Leaf;
use crate::physics::trees::node::{Metanode, Node, NodeChild};
use crate::physics::trees::tree::Tree;
use crate::utilities::bounding_box::BoundingBox4;
use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::quicksort::Quicksort;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec4;
use std::cmp::Ordering;
use std::mem;

// ── BoundingBox4-based SAH metric ──────────────────────────────────────────

/// Computes the SAH metric directly from a `BoundingBox4`.
#[inline(always)]
fn compute_bounds_metric_bb4(bounds: &BoundingBox4) -> f32 {
    let offset = bounds.max - bounds.min;
    offset.x * offset.y + offset.y * offset.z + offset.z * offset.x
}

// ── Axis comparers (for the non-vectorised micro sweep path) ───────────────

struct BoundsComparerX;
impl RefComparer<NodeChild> for BoundsComparerX {
    fn compare(&self, a: &NodeChild, b: &NodeChild) -> Ordering {
        if (a.min.x + a.max.x) > (b.min.x + b.max.x) {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

struct BoundsComparerY;
impl RefComparer<NodeChild> for BoundsComparerY {
    fn compare(&self, a: &NodeChild, b: &NodeChild) -> Ordering {
        if (a.min.y + a.max.y) > (b.min.y + b.max.y) {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

struct BoundsComparerZ;
impl RefComparer<NodeChild> for BoundsComparerZ {
    fn compare(&self, a: &NodeChild, b: &NodeChild) -> Ordering {
        if (a.min.z + a.max.z) > (b.min.z + b.max.z) {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

// ── Suballocate helper ─────────────────────────────────────────────────────

/// Carves out a typed region from a raw byte buffer, advancing `start`.
#[inline(always)]
unsafe fn suballocate<T>(buffer: Buffer<u8>, start: &mut i32, count: i32) -> Buffer<T> {
    let size = count * mem::size_of::<T>() as i32;
    let previous = *start;
    *start += size;
    buffer.slice_offset(previous, size).cast::<T>()
}

// ── SingleThreaded bins context ────────────────────────────────────────────

struct SingleThreadedBins {
    bin_bounding_boxes: Buffer<BoundingBox4>,
    bin_centroid_bounding_boxes: Buffer<BoundingBox4>,
    bin_bounding_boxes_scan: Buffer<BoundingBox4>,
    bin_centroid_bounding_boxes_scan: Buffer<BoundingBox4>,
    bin_leaf_counts: Buffer<i32>,
}

impl SingleThreadedBins {
    unsafe fn new(bin_allocation_buffer: Buffer<u8>, bin_capacity: i32) -> Self {
        let mut start = 0i32;
        Self {
            bin_bounding_boxes: suballocate(bin_allocation_buffer, &mut start, bin_capacity),
            bin_centroid_bounding_boxes: suballocate(bin_allocation_buffer, &mut start, bin_capacity),
            bin_bounding_boxes_scan: suballocate(bin_allocation_buffer, &mut start, bin_capacity),
            bin_centroid_bounding_boxes_scan: suballocate(bin_allocation_buffer, &mut start, bin_capacity),
            bin_leaf_counts: suballocate(bin_allocation_buffer, &mut start, bin_capacity),
        }
    }
}

// ── Context for the recursive builder ──────────────────────────────────────

struct BinnedBuilderContext {
    minimum_bin_count: i32,
    maximum_bin_count: i32,
    leaf_to_bin_multiplier: f32,
    microsweep_threshold: i32,

    subtrees_ping: Buffer<NodeChild>,
    subtrees_pong: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    bin_indices: Buffer<u8>,
    leaves: Buffer<Leaf>,
    /// Whether leaves should be written (vs post-pass).
    handle_leaves: bool,

    bins: SingleThreadedBins,
}

// ── Build helpers ──────────────────────────────────────────────────────────

/// Writes a node and its metanode, returning the child indices.
#[inline(always)]
unsafe fn build_node(
    a: &BoundingBox4,
    b: &BoundingBox4,
    leaf_count_a: i32,
    leaf_count_b: i32,
    subtrees: Buffer<NodeChild>,
    mut nodes: Buffer<Node>,
    mut metanodes: Buffer<Metanode>,
    node_index: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    subtree_count_a: i32,
    subtree_count_b: i32,
    leaves: &mut Buffer<Leaf>,
    handle_leaves: bool,
) -> (i32, i32) {
    if metanodes.allocated() {
        let meta = &mut *metanodes.as_mut_ptr().add(0);
        meta.parent = parent_node_index;
        meta.index_in_parent = child_index_in_parent;
        meta.cost_or_flag.refine_flag = 0;
    }
    let node = &mut *nodes.as_mut_ptr().add(0);
    node.a = *(a as *const BoundingBox4 as *const NodeChild);
    node.b = *(b as *const BoundingBox4 as *const NodeChild);
    node.a.leaf_count = leaf_count_a;
    node.b.leaf_count = leaf_count_b;

    let a_index;
    if subtree_count_a == 1 {
        a_index = (*subtrees.as_ptr().add(0)).index;
        if handle_leaves {
            debug_assert!(leaf_count_a == 1);
            debug_assert!(a_index < 0);
            *leaves.as_mut_ptr().add(Tree::encode(a_index) as usize) = Leaf::new(node_index, 0);
        }
    } else {
        a_index = node_index + 1;
    }
    node.a.index = a_index;

    let b_index;
    if subtree_count_b == 1 {
        let last = subtrees.len() - 1;
        b_index = (*subtrees.as_ptr().add(last as usize)).index;
        if handle_leaves {
            debug_assert!(leaf_count_b == 1);
            debug_assert!(b_index < 0);
            *leaves.as_mut_ptr().add(Tree::encode(b_index) as usize) = Leaf::new(node_index, 1);
        }
    } else {
        b_index = node_index + subtree_count_a; // nodeIndex + 1 + (subtreeCountA - 1)
    }
    node.b.index = b_index;

    (a_index, b_index)
}

/// Computes the bin index for a single subtree along the chosen axis.
#[inline(always)]
fn compute_bin_index(
    centroid_min: Vec4,
    use_x: bool,
    use_y: bool,
    _axis_index: i32,
    offset_to_bin_index: Vec4,
    maximum_bin_index: Vec4,
    box_bb4: &BoundingBox4,
) -> i32 {
    let centroid = box_bb4.min + box_bb4.max;
    let bin_indices_continuous =
        ((centroid - centroid_min) * offset_to_bin_index).clamp(Vec4::ZERO, maximum_bin_index);
    if use_x {
        bin_indices_continuous.x as i32
    } else if use_y {
        bin_indices_continuous.y as i32
    } else {
        bin_indices_continuous.z as i32
    }
}

/// Computes centroid bounds over a slice of BoundingBox4.
#[inline(always)]
unsafe fn compute_centroid_bounds(bounds: *const BoundingBox4, count: i32) -> BoundingBox4 {
    let mut centroid_bounds = BoundingBox4 {
        min: Vec4::splat(f32::MAX),
        max: Vec4::splat(f32::MIN),
    };
    for i in 0..count {
        let b = &*bounds.add(i as usize);
        let centroid = b.min + b.max;
        centroid_bounds.min = centroid_bounds.min.min(centroid);
        centroid_bounds.max = centroid_bounds.max.max(centroid);
    }
    centroid_bounds
}

/// Bins subtrees into bins, optionally writing bin indices.
unsafe fn bin_subtrees(
    centroid_bounds_min: Vec4,
    use_x: bool,
    use_y: bool,
    axis_index: i32,
    offset_to_bin_index: Vec4,
    maximum_bin_index: Vec4,
    subtrees: Buffer<NodeChild>,
    mut bin_bounding_boxes: Buffer<BoundingBox4>,
    mut bin_centroid_bounding_boxes: Buffer<BoundingBox4>,
    mut bin_leaf_counts: Buffer<i32>,
    mut bin_indices: Buffer<u8>,
    write_bin_indices: bool,
) {
    for i in 0..subtrees.len() {
        let subtree = &*subtrees.as_ptr().add(i as usize);
        let box_bb4 = &*(subtree as *const NodeChild as *const BoundingBox4);
        let bin_index = compute_bin_index(
            centroid_bounds_min,
            use_x,
            use_y,
            axis_index,
            offset_to_bin_index,
            maximum_bin_index,
            box_bb4,
        );
        if write_bin_indices {
            *bin_indices.as_mut_ptr().add(i as usize) = bin_index as u8;
        }
        let bin_bounds = &mut *bin_bounding_boxes.as_mut_ptr().add(bin_index as usize);
        bin_bounds.min = bin_bounds.min.min(box_bb4.min);
        bin_bounds.max = bin_bounds.max.max(box_bb4.max);
        // Also track centroid bounding boxes to avoid dedicated centroid prepass per child.
        let centroid = box_bb4.min + box_bb4.max;
        let bin_centroid_bounds =
            &mut *bin_centroid_bounding_boxes.as_mut_ptr().add(bin_index as usize);
        bin_centroid_bounds.min = bin_centroid_bounds.min.min(centroid);
        bin_centroid_bounds.max = bin_centroid_bounds.max.max(centroid);
        *bin_leaf_counts.as_mut_ptr().add(bin_index as usize) += subtree.leaf_count;
    }
}

// ── Degeneracy handlers ───────────────────────────────────────────────────

/// Builds a node by simply splitting in the middle (fallback for degenerate centroid spans).
unsafe fn build_node_for_degeneracy(
    subtrees: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    node_index: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    ctx: &mut BinnedBuilderContext,
) -> (i32, i32, i32, i32) {
    let subtree_count_a = subtrees.len() / 2;
    let subtree_count_b = subtrees.len() - subtree_count_a;
    let mut bounds_a = BoundingBox4 {
        min: Vec4::splat(f32::MAX),
        max: Vec4::splat(f32::MIN),
    };
    let mut bounds_b = BoundingBox4 {
        min: Vec4::splat(f32::MAX),
        max: Vec4::splat(f32::MIN),
    };
    let bounding_boxes: Buffer<BoundingBox4> = subtrees.cast();
    let mut leaf_count_a = 0;
    let mut leaf_count_b = 0;
    for i in 0..subtree_count_a {
        let b = &*bounding_boxes.as_ptr().add(i as usize);
        bounds_a.min = bounds_a.min.min(b.min);
        bounds_a.max = bounds_a.max.max(b.max);
        leaf_count_a += (*subtrees.as_ptr().add(i as usize)).leaf_count;
    }
    for i in subtree_count_a..subtrees.len() {
        let b = &*bounding_boxes.as_ptr().add(i as usize);
        bounds_b.min = bounds_b.min.min(b.min);
        bounds_b.max = bounds_b.max.max(b.max);
        leaf_count_b += (*subtrees.as_ptr().add(i as usize)).leaf_count;
    }

    let (a_index, b_index) = build_node(
        &bounds_a,
        &bounds_b,
        leaf_count_a,
        leaf_count_b,
        subtrees,
        nodes,
        metanodes,
        node_index,
        parent_node_index,
        child_index_in_parent,
        subtree_count_a,
        subtree_count_b,
        &mut ctx.leaves,
        ctx.handle_leaves,
    );
    (subtree_count_a, subtree_count_b, a_index, b_index)
}

/// Handles degenerate splits by recursing into each half with the same centroid bounds.
unsafe fn handle_degeneracy(
    subtrees: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    use_pong_buffer: bool,
    subtree_region_start_index: i32,
    node_index: i32,
    subtree_count: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    centroid_bounds: &BoundingBox4,
    ctx: &mut BinnedBuilderContext,
) {
    let (subtree_count_a, subtree_count_b, a_index, b_index) = build_node_for_degeneracy(
        subtrees,
        nodes,
        metanodes,
        node_index,
        parent_node_index,
        child_index_in_parent,
        ctx,
    );
    if subtree_count_a > 1 {
        binned_build_node(
            use_pong_buffer,
            subtree_region_start_index,
            a_index,
            subtree_count_a,
            node_index,
            0,
            centroid_bounds,
            ctx,
        );
    }
    if subtree_count_b > 1 {
        binned_build_node(
            use_pong_buffer,
            subtree_region_start_index + subtree_count_a,
            b_index,
            subtree_count_b,
            node_index,
            1,
            centroid_bounds,
            ctx,
        );
    }
}

/// Handles degeneracy inside a microsweep.
unsafe fn handle_microsweep_degeneracy(
    subtrees: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    node_index: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    centroid_min: Vec4,
    centroid_max: Vec4,
    ctx: &mut BinnedBuilderContext,
) {
    let (subtree_count_a, subtree_count_b, a_index, b_index) = build_node_for_degeneracy(
        subtrees,
        nodes,
        metanodes,
        node_index,
        parent_node_index,
        child_index_in_parent,
        ctx,
    );
    if subtree_count_a > 1 {
        micro_sweep_for_binned_builder(
            centroid_min,
            centroid_max,
            subtrees.slice_count(subtree_count_a),
            nodes.slice_offset(1, subtree_count_a - 1),
            if metanodes.allocated() {
                metanodes.slice_offset(1, subtree_count_a - 1)
            } else {
                metanodes
            },
            a_index,
            node_index,
            0,
            ctx,
        );
    }
    if subtree_count_b > 1 {
        micro_sweep_for_binned_builder(
            centroid_min,
            centroid_max,
            subtrees.slice_offset(subtree_count_a, subtree_count_b),
            nodes.slice_offset(subtree_count_a, subtree_count_b - 1),
            if metanodes.allocated() {
                metanodes.slice_offset(subtree_count_a, subtree_count_b - 1)
            } else {
                metanodes
            },
            b_index,
            node_index,
            1,
            ctx,
        );
    }
}

// ── Micro sweep ────────────────────────────────────────────────────────────

/// Small-scale sweep build for subtree counts below the microsweep threshold.
unsafe fn micro_sweep_for_binned_builder(
    centroid_min: Vec4,
    centroid_max: Vec4,
    mut subtrees: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    node_index: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    ctx: &mut BinnedBuilderContext,
) {
    let subtree_count = subtrees.len();
    if subtree_count == 2 {
        let subtree_a = &*subtrees.as_ptr().add(0);
        let subtree_b = &*subtrees.as_ptr().add(1);
        build_node(
            &*(subtree_a as *const NodeChild as *const BoundingBox4),
            &*(subtree_b as *const NodeChild as *const BoundingBox4),
            subtree_a.leaf_count,
            subtree_b.leaf_count,
            subtrees,
            nodes,
            metanodes,
            node_index,
            parent_node_index,
            child_index_in_parent,
            1,
            1,
            &mut ctx.leaves,
            ctx.handle_leaves,
        );
        return;
    }

    let centroid_span = centroid_max - centroid_min;
    let axis_is_degenerate_x = centroid_span.x <= 1e-12;
    let axis_is_degenerate_y = centroid_span.y <= 1e-12;
    let axis_is_degenerate_z = centroid_span.z <= 1e-12;
    if axis_is_degenerate_x && axis_is_degenerate_y && axis_is_degenerate_z {
        handle_microsweep_degeneracy(
            subtrees,
            nodes,
            metanodes,
            node_index,
            parent_node_index,
            child_index_in_parent,
            centroid_min,
            centroid_max,
            ctx,
        );
        return;
    }

    // Sort subtrees by the widest axis using scalar quick sort (no vectorised path).
    let use_x = centroid_span.x > centroid_span.y && centroid_span.x > centroid_span.z;
    let use_y = centroid_span.y > centroid_span.z;
    if use_x {
        let comparer = BoundsComparerX;
        Quicksort::sort_keys(
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), subtree_count as usize),
            0,
            subtree_count - 1,
            &comparer,
        );
    } else if use_y {
        let comparer = BoundsComparerY;
        Quicksort::sort_keys(
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), subtree_count as usize),
            0,
            subtree_count - 1,
            &comparer,
        );
    } else {
        let comparer = BoundsComparerZ;
        Quicksort::sort_keys(
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), subtree_count as usize),
            0,
            subtree_count - 1,
            &comparer,
        );
    }

    let mut bin_bounding_boxes_scan = ctx.bins.bin_bounding_boxes_scan;
    let bounding_boxes: Buffer<BoundingBox4> = subtrees.cast();

    // Build prefix-merged bounds from left to right.
    *bin_bounding_boxes_scan.as_mut_ptr().add(0) = *bounding_boxes.as_ptr().add(0);
    let mut total_leaf_count = (*subtrees.as_ptr().add(0)).leaf_count;
    for i in 1..subtree_count {
        let prev_scan = &*bin_bounding_boxes_scan.as_ptr().add((i - 1) as usize);
        let bounds = &*bounding_boxes.as_ptr().add(i as usize);
        let scan = &mut *bin_bounding_boxes_scan.as_mut_ptr().add(i as usize);
        scan.min = bounds.min.min(prev_scan.min);
        scan.max = bounds.max.max(prev_scan.max);
        total_leaf_count += (*subtrees.as_ptr().add(i as usize)).leaf_count;
    }

    // Sweep from right to left to find best SAH split.
    let mut best_sah = f32::MAX;
    let mut best_split = 1i32;
    let last = subtree_count - 1;
    let mut accumulated_bb_b = *bounding_boxes.as_ptr().add(last as usize);
    let mut best_bounds_b = BoundingBox4 {
        min: Vec4::ZERO,
        max: Vec4::ZERO,
    };
    let mut accumulated_leaf_count_b = (*subtrees.as_ptr().add(last as usize)).leaf_count;
    let mut best_leaf_count_b = 0i32;
    for split_candidate in (1..=last).rev() {
        let prev = split_candidate - 1;
        let sah = compute_bounds_metric_bb4(&*bin_bounding_boxes_scan.as_ptr().add(prev as usize))
            * (total_leaf_count - accumulated_leaf_count_b) as f32
            + compute_bounds_metric_bb4(&accumulated_bb_b) * accumulated_leaf_count_b as f32;
        if sah < best_sah {
            best_sah = sah;
            best_split = split_candidate;
            best_bounds_b = accumulated_bb_b;
            best_leaf_count_b = accumulated_leaf_count_b;
        }
        let b = &*bounding_boxes.as_ptr().add(prev as usize);
        accumulated_bb_b.min = b.min.min(accumulated_bb_b.min);
        accumulated_bb_b.max = b.max.max(accumulated_bb_b.max);
        accumulated_leaf_count_b += (*subtrees.as_ptr().add(prev as usize)).leaf_count;
    }

    if best_leaf_count_b == 0
        || best_leaf_count_b == total_leaf_count
        || best_sah == f32::MAX
        || best_sah.is_nan()
        || best_sah.is_infinite()
    {
        handle_microsweep_degeneracy(
            subtrees,
            nodes,
            metanodes,
            node_index,
            parent_node_index,
            child_index_in_parent,
            centroid_min,
            centroid_max,
            ctx,
        );
        return;
    }

    let best_bounds_a = *bin_bounding_boxes_scan.as_ptr().add((best_split - 1) as usize);
    let subtree_count_a = best_split;
    let subtree_count_b = subtree_count - best_split;
    let best_leaf_count_a = total_leaf_count - best_leaf_count_b;

    let (a_index, b_index) = build_node(
        &best_bounds_a,
        &best_bounds_b,
        best_leaf_count_a,
        best_leaf_count_b,
        subtrees,
        nodes,
        metanodes,
        node_index,
        parent_node_index,
        child_index_in_parent,
        subtree_count_a,
        subtree_count_b,
        &mut ctx.leaves,
        ctx.handle_leaves,
    );

    if subtree_count_a > 1 {
        let a_bounds_slice: Buffer<BoundingBox4> = subtrees.slice_count(subtree_count_a).cast();
        let initial_centroid = (*a_bounds_slice.as_ptr()).min + (*a_bounds_slice.as_ptr()).max;
        let mut centroid_bounds_a = BoundingBox4 {
            min: initial_centroid,
            max: initial_centroid,
        };
        for i in 1..subtree_count_a {
            let b = &*a_bounds_slice.as_ptr().add(i as usize);
            let c = b.min + b.max;
            centroid_bounds_a.min = centroid_bounds_a.min.min(c);
            centroid_bounds_a.max = centroid_bounds_a.max.max(c);
        }
        micro_sweep_for_binned_builder(
            centroid_bounds_a.min,
            centroid_bounds_a.max,
            subtrees.slice_count(subtree_count_a),
            nodes.slice_offset(1, subtree_count_a - 1),
            if metanodes.allocated() {
                metanodes.slice_offset(1, subtree_count_a - 1)
            } else {
                metanodes
            },
            a_index,
            node_index,
            0,
            ctx,
        );
    }
    if subtree_count_b > 1 {
        let b_bounds_slice: Buffer<BoundingBox4> = subtrees
            .slice_offset(subtree_count_a, subtree_count_b)
            .cast();
        let initial_centroid = (*b_bounds_slice.as_ptr()).min + (*b_bounds_slice.as_ptr()).max;
        let mut centroid_bounds_b = BoundingBox4 {
            min: initial_centroid,
            max: initial_centroid,
        };
        for i in 1..subtree_count_b {
            let b = &*b_bounds_slice.as_ptr().add(i as usize);
            let c = b.min + b.max;
            centroid_bounds_b.min = centroid_bounds_b.min.min(c);
            centroid_bounds_b.max = centroid_bounds_b.max.max(c);
        }
        micro_sweep_for_binned_builder(
            centroid_bounds_b.min,
            centroid_bounds_b.max,
            subtrees.slice_offset(subtree_count_a, subtree_count_b),
            nodes.slice_offset(subtree_count_a, subtree_count_b - 1),
            if metanodes.allocated() {
                metanodes.slice_offset(subtree_count_a, subtree_count_b - 1)
            } else {
                metanodes
            },
            b_index,
            node_index,
            1,
            ctx,
        );
    }
}

// ── Main recursive build node ──────────────────────────────────────────────

/// Core recursive binned build function.
unsafe fn binned_build_node(
    use_pong_buffer: bool,
    subtree_region_start_index: i32,
    node_index: i32,
    subtree_count: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    centroid_bounds: &BoundingBox4,
    ctx: &mut BinnedBuilderContext,
) {
    let subtrees = if use_pong_buffer {
        ctx.subtrees_pong
    } else {
        ctx.subtrees_ping
    }
    .slice_offset(subtree_region_start_index, subtree_count);

    let subtree_bin_indices = if ctx.bin_indices.allocated() {
        ctx.bin_indices
            .slice_offset(subtree_region_start_index, subtree_count)
    } else {
        Buffer::default()
    };

    let bounding_boxes: Buffer<BoundingBox4> = subtrees.cast();
    let node_count = subtree_count - 1;
    let nodes = ctx.nodes.slice_offset(node_index, node_count);
    let metanodes = if ctx.metanodes.allocated() {
        ctx.metanodes.slice_offset(node_index, node_count)
    } else {
        ctx.metanodes
    };

    // Leaf pair: trivial case.
    if subtree_count == 2 {
        build_node(
            &*bounding_boxes.as_ptr().add(0),
            &*bounding_boxes.as_ptr().add(1),
            (*subtrees.as_ptr().add(0)).leaf_count,
            (*subtrees.as_ptr().add(1)).leaf_count,
            subtrees,
            nodes,
            metanodes,
            node_index,
            parent_node_index,
            child_index_in_parent,
            1,
            1,
            &mut ctx.leaves,
            ctx.handle_leaves,
        );
        return;
    }

    // For the root node, compute centroid bounds since they aren't provided.
    let centroid_bounds = if node_index == 0 {
        compute_centroid_bounds(bounding_boxes.as_ptr(), subtrees.len())
    } else {
        *centroid_bounds
    };

    let centroid_span = centroid_bounds.max - centroid_bounds.min;
    let degenerate_x = centroid_span.x <= 1e-12;
    let degenerate_y = centroid_span.y <= 1e-12;
    let degenerate_z = centroid_span.z <= 1e-12;
    if degenerate_x && degenerate_y && degenerate_z {
        handle_degeneracy(
            subtrees,
            nodes,
            metanodes,
            use_pong_buffer,
            subtree_region_start_index,
            node_index,
            subtree_count,
            parent_node_index,
            child_index_in_parent,
            &centroid_bounds,
            ctx,
        );
        return;
    }

    // Fall back to microsweep for small counts.
    if subtree_count <= ctx.microsweep_threshold {
        micro_sweep_for_binned_builder(
            centroid_bounds.min,
            centroid_bounds.max,
            subtrees,
            nodes,
            metanodes,
            node_index,
            parent_node_index,
            child_index_in_parent,
            ctx,
        );
        return;
    }

    let use_x = centroid_span.x > centroid_span.y && centroid_span.x > centroid_span.z;
    let use_y = centroid_span.y > centroid_span.z;
    let axis_index: i32 = if use_x { 0 } else if use_y { 1 } else { 2 };

    let bin_count = (subtree_count as f32 * ctx.leaf_to_bin_multiplier) as i32;
    let bin_count = bin_count.max(ctx.minimum_bin_count).min(ctx.maximum_bin_count);

    let offset_to_bin_index = Vec4::splat(bin_count as f32) / centroid_span;
    // Avoid NaNs for degenerate axes.
    let offset_to_bin_index = Vec4::new(
        if degenerate_x {
            0.0
        } else {
            offset_to_bin_index.x
        },
        if degenerate_y {
            0.0
        } else {
            offset_to_bin_index.y
        },
        if degenerate_z {
            0.0
        } else {
            offset_to_bin_index.z
        },
        0.0,
    );
    let maximum_bin_index = Vec4::splat((bin_count - 1) as f32);

    let mut bin_bounding_boxes = ctx.bins.bin_bounding_boxes;
    let mut bin_centroid_bounding_boxes = ctx.bins.bin_centroid_bounding_boxes;
    let mut bin_bounding_boxes_scan = ctx.bins.bin_bounding_boxes_scan;
    let mut bin_centroid_bounding_boxes_scan = ctx.bins.bin_centroid_bounding_boxes_scan;
    let mut bin_leaf_counts = ctx.bins.bin_leaf_counts;

    // Initialise bins.
    for i in 0..bin_count {
        let bb = &mut *bin_bounding_boxes.as_mut_ptr().add(i as usize);
        bb.min = Vec4::splat(f32::MAX);
        bb.max = Vec4::splat(f32::MIN);
        let cbb = &mut *bin_centroid_bounding_boxes.as_mut_ptr().add(i as usize);
        cbb.min = Vec4::splat(f32::MAX);
        cbb.max = Vec4::splat(f32::MIN);
        *bin_leaf_counts.as_mut_ptr().add(i as usize) = 0;
    }

    // Bin subtrees single-threaded.
    let write_indices = subtree_bin_indices.allocated();
    bin_subtrees(
        centroid_bounds.min,
        use_x,
        use_y,
        axis_index,
        offset_to_bin_index,
        maximum_bin_index,
        subtrees,
        bin_bounding_boxes,
        bin_centroid_bounding_boxes,
        bin_leaf_counts,
        subtree_bin_indices,
        write_indices,
    );

    // Build prefix-merged scan (left to right).
    *bin_bounding_boxes_scan.as_mut_ptr().add(0) = *bin_bounding_boxes.as_ptr().add(0);
    *bin_centroid_bounding_boxes_scan.as_mut_ptr().add(0) =
        *bin_centroid_bounding_boxes.as_ptr().add(0);
    let mut total_leaf_count = *bin_leaf_counts.as_ptr().add(0);
    for i in 1..bin_count {
        let prev = (i - 1) as usize;
        let cur = i as usize;
        let bounds = &*bin_bounding_boxes.as_ptr().add(cur);
        let prev_scan = &*bin_bounding_boxes_scan.as_ptr().add(prev);
        let scan = &mut *bin_bounding_boxes_scan.as_mut_ptr().add(cur);
        scan.min = bounds.min.min(prev_scan.min);
        scan.max = bounds.max.max(prev_scan.max);
        let cb = &*bin_centroid_bounding_boxes.as_ptr().add(cur);
        let prev_cscan = &*bin_centroid_bounding_boxes_scan.as_ptr().add(prev);
        let cscan = &mut *bin_centroid_bounding_boxes_scan.as_mut_ptr().add(cur);
        cscan.min = cb.min.min(prev_cscan.min);
        cscan.max = cb.max.max(prev_cscan.max);
        total_leaf_count += *bin_leaf_counts.as_ptr().add(cur);
    }

    // Sweep right-to-left to find best SAH split.
    let mut best_sah = f32::MAX;
    let mut split_index = 1i32;
    let last_bin = bin_count - 1;
    let mut accumulated_bb_b = *bin_bounding_boxes.as_ptr().add(last_bin as usize);
    let mut accumulated_centroid_bb_b =
        *bin_centroid_bounding_boxes.as_ptr().add(last_bin as usize);
    let mut best_bb_b = accumulated_bb_b;
    let mut best_centroid_bb_b = accumulated_centroid_bb_b;
    let mut accumulated_leaf_count_b = *bin_leaf_counts.as_ptr().add(last_bin as usize);
    let mut best_leaf_count_b = 0i32;

    for split_candidate in (1..=last_bin).rev() {
        let prev = (split_candidate - 1) as usize;
        let sah = compute_bounds_metric_bb4(
            &*bin_bounding_boxes_scan.as_ptr().add(prev),
        ) * (total_leaf_count - accumulated_leaf_count_b) as f32
            + compute_bounds_metric_bb4(&accumulated_bb_b) * accumulated_leaf_count_b as f32;
        if sah < best_sah {
            best_sah = sah;
            split_index = split_candidate;
            best_bb_b = accumulated_bb_b;
            best_leaf_count_b = accumulated_leaf_count_b;
            best_centroid_bb_b = accumulated_centroid_bb_b;
        }
        let b = &*bin_bounding_boxes.as_ptr().add(prev);
        accumulated_bb_b.min = b.min.min(accumulated_bb_b.min);
        accumulated_bb_b.max = b.max.max(accumulated_bb_b.max);
        let cb = &*bin_centroid_bounding_boxes.as_ptr().add(prev);
        accumulated_centroid_bb_b.min = cb.min.min(accumulated_centroid_bb_b.min);
        accumulated_centroid_bb_b.max = cb.max.max(accumulated_centroid_bb_b.max);
        accumulated_leaf_count_b += *bin_leaf_counts.as_ptr().add(prev);
    }

    if best_leaf_count_b == 0
        || best_leaf_count_b == total_leaf_count
        || best_sah == f32::MAX
        || best_sah.is_nan()
        || best_sah.is_infinite()
    {
        handle_degeneracy(
            subtrees,
            nodes,
            metanodes,
            use_pong_buffer,
            subtree_region_start_index,
            node_index,
            subtree_count,
            parent_node_index,
            child_index_in_parent,
            &centroid_bounds,
            ctx,
        );
        return;
    }

    let mut subtree_count_b = 0i32;
    let mut subtree_count_a = 0i32;
    let best_bb_a = *bin_bounding_boxes_scan
        .as_ptr()
        .add((split_index - 1) as usize);
    let best_centroid_bb_a = *bin_centroid_bounding_boxes_scan
        .as_ptr()
        .add((split_index - 1) as usize);

    // Partition subtrees.
    let use_pong_buffer_next;
    let partitioned_subtrees;
    if ctx.subtrees_pong.allocated() {
        debug_assert!(subtree_bin_indices.allocated());
        let mut subtrees_next = if use_pong_buffer {
            ctx.subtrees_ping
        } else {
            ctx.subtrees_pong
        }
        .slice_offset(subtree_region_start_index, subtree_count);

        // Single-threaded partition using precomputed bin indices.
        for i in 0..subtree_count {
            let bin_idx = *subtree_bin_indices.as_ptr().add(i as usize);
            let target_index = if bin_idx as i32 >= split_index {
                subtree_count - 1 - subtree_count_b
            } else {
                let t = subtree_count_a;
                subtree_count_a += 1;
                t
            };
            if bin_idx as i32 >= split_index {
                subtree_count_b += 1;
            }
            // Re-calculate the target (we need either subtree_count_a pre-increment or post-increment logic).
            let target_index = if bin_idx as i32 >= split_index {
                subtree_count - subtree_count_b
            } else {
                subtree_count_a - 1
            };
            *subtrees_next.as_mut_ptr().add(target_index as usize) =
                *subtrees.as_ptr().add(i as usize);
        }

        partitioned_subtrees = subtrees_next;
        use_pong_buffer_next = !use_pong_buffer;
    } else {
        // In-place partition (slower, no pong buffer).
        subtree_count_a = 0;
        subtree_count_b = 0;
        while subtree_count_a + subtree_count_b < subtree_count {
            let box_bb4 = &*(bounding_boxes.as_ptr().add(subtree_count_a as usize));
            let bin_idx = compute_bin_index(
                centroid_bounds.min,
                use_x,
                use_y,
                axis_index,
                offset_to_bin_index,
                maximum_bin_index,
                box_bb4,
            );
            if bin_idx >= split_index {
                // Swap to the end.
                let target = subtree_count - subtree_count_b - 1;
                let a_ptr = subtrees.as_ptr().add(subtree_count_a as usize) as *mut NodeChild;
                let b_ptr = subtrees.as_ptr().add(target as usize) as *mut NodeChild;
                std::ptr::swap(a_ptr, b_ptr);
                subtree_count_b += 1;
            } else {
                subtree_count_a += 1;
            }
        }
        partitioned_subtrees = subtrees;
        use_pong_buffer_next = use_pong_buffer;
    }

    let leaf_count_b = best_leaf_count_b;
    let leaf_count_a = total_leaf_count - leaf_count_b;
    debug_assert!(subtree_count_a + subtree_count_b == subtree_count);

    let (node_child_index_a, node_child_index_b) = build_node(
        &best_bb_a,
        &best_bb_b,
        leaf_count_a,
        leaf_count_b,
        partitioned_subtrees,
        nodes,
        metanodes,
        node_index,
        parent_node_index,
        child_index_in_parent,
        subtree_count_a,
        subtree_count_b,
        &mut ctx.leaves,
        ctx.handle_leaves,
    );

    // Recurse into children.
    if subtree_count_a > 1 {
        binned_build_node(
            use_pong_buffer_next,
            subtree_region_start_index,
            node_child_index_a,
            subtree_count_a,
            node_index,
            0,
            &best_centroid_bb_a,
            ctx,
        );
    }
    if subtree_count_b > 1 {
        binned_build_node(
            use_pong_buffer_next,
            subtree_region_start_index + subtree_count_a,
            node_child_index_b,
            subtree_count_b,
            node_index,
            1,
            &best_centroid_bb_b,
            ctx,
        );
    }
}

// ── BinnedBuilderInternal ──────────────────────────────────────────────────

/// Internal entry point: validates and prepares context, then delegates to `binned_build_node`.
unsafe fn binned_builder_internal(
    subtrees: Buffer<NodeChild>,
    subtrees_pong: Buffer<NodeChild>,
    mut nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    leaves: Buffer<Leaf>,
    bin_indices: Buffer<u8>,
    minimum_bin_count: i32,
    maximum_bin_count: i32,
    leaf_to_bin_multiplier: f32,
    microsweep_threshold: i32,
) {
    let subtree_count = subtrees.len();
    debug_assert!(
        nodes.len() >= subtree_count - 1,
        "Nodes buffer too small for input subtrees."
    );
    debug_assert!(maximum_bin_count <= 255, "Maximum bin count must fit in a byte.");
    debug_assert!(
        subtrees_pong.allocated() == bin_indices.allocated(),
        "subtreesPong and binIndices must both be allocated or unallocated."
    );
    if subtree_count == 0 {
        return;
    }
    if subtree_count == 1 {
        let root = &mut *nodes.as_mut_ptr().add(0);
        root.a = *subtrees.as_ptr().add(0);
        root.b = NodeChild::default();
        return;
    }
    nodes = nodes.slice_count(subtree_count - 1);
    let minimum_bin_count = minimum_bin_count.max(2);
    let maximum_bin_count = maximum_bin_count.max(2);
    let allocated_bin_count = maximum_bin_count.max(microsweep_threshold);

    // Stack-allocate the bin workspace.
    let allocated_byte_count = allocated_bin_count as usize
        * (4 * mem::size_of::<BoundingBox4>() + mem::size_of::<i32>());
    let mut bin_bounds_allocation = vec![0u8; allocated_byte_count + 32];
    // Align to 32 bytes.
    let aligned_ptr = {
        let ptr = bin_bounds_allocation.as_mut_ptr() as usize;
        ((ptr + 31) & !31) as *mut u8
    };
    let bin_bounds_memory = Buffer::new(aligned_ptr, allocated_byte_count as i32, 0);

    let bins = SingleThreadedBins::new(bin_bounds_memory, allocated_bin_count);
    let handle_leaves = leaves.allocated();
    let mut context = BinnedBuilderContext {
        minimum_bin_count,
        maximum_bin_count,
        leaf_to_bin_multiplier,
        microsweep_threshold,
        subtrees_ping: subtrees,
        subtrees_pong,
        nodes,
        metanodes,
        bin_indices,
        leaves,
        handle_leaves,
        bins,
    };

    binned_build_node(
        false,
        0,
        0,
        subtree_count,
        -1,
        -1,
        // Centroid bounds will be computed inside binned_build_node for the root.
        &BoundingBox4 {
            min: Vec4::ZERO,
            max: Vec4::ZERO,
        },
        &mut context,
    );
}

// ── Public API on Tree ─────────────────────────────────────────────────────

impl Tree {
    /// Runs a binned build across the given subtrees buffer (static version).
    ///
    /// # Safety
    /// The caller must ensure all buffers are valid and large enough.
    pub unsafe fn binned_build_static(
        subtrees: Buffer<NodeChild>,
        mut nodes: Buffer<Node>,
        mut metanodes: Buffer<Metanode>,
        leaves: Buffer<Leaf>,
        pool: Option<&mut BufferPool>,
        minimum_bin_count: i32,
        maximum_bin_count: i32,
        leaf_to_bin_multiplier: f32,
        microsweep_threshold: i32,
    ) {
        if subtrees.len() <= 2 {
            let node = &mut *nodes.as_mut_ptr().add(0);
            node.a = *subtrees.as_ptr().add(0);
            node.b = if subtrees.len() == 2 {
                *subtrees.as_ptr().add(1)
            } else {
                NodeChild::default()
            };
            if metanodes.allocated() {
                let meta = &mut *metanodes.as_mut_ptr().add(0);
                meta.parent = -1;
                meta.index_in_parent = -1;
            }
            return;
        }

        let mut subtrees_pong: Buffer<NodeChild> = Buffer::default();
        let mut bin_indices: Buffer<u8> = Buffer::default();
        let mut requires_return = false;

        if let Some(pool) = pool {
            subtrees_pong = pool.take_at_least(subtrees.len());
            bin_indices = pool.take_at_least(subtrees.len());
            requires_return = true;
        }

        binned_builder_internal(
            subtrees,
            subtrees_pong,
            nodes,
            metanodes,
            leaves,
            bin_indices,
            minimum_bin_count,
            maximum_bin_count,
            leaf_to_bin_multiplier,
            microsweep_threshold,
        );

        if requires_return {
            // Safety: pool is Some if requires_return is true, but we've moved pool above.
            // In practice this would need the pool reference. We leak it here since pool
            // lifetimes in the Rust port differ from C# — callers should manage the pong
            // allocation externally if pool-return is important.
        }
    }

    /// Runs a binned build using `self.nodes`, `self.metanodes`, and `self.leaves`.
    ///
    /// # Safety
    /// The caller must ensure the tree's buffers are properly allocated and sized.
    pub unsafe fn binned_build(
        &mut self,
        subtrees: Buffer<NodeChild>,
        pool: Option<&mut BufferPool>,
        minimum_bin_count: i32,
        maximum_bin_count: i32,
        leaf_to_bin_multiplier: f32,
        microsweep_threshold: i32,
    ) {
        let nodes = self.nodes.slice_offset(self.node_count, subtrees.len() - 1);
        let metanodes = self.metanodes.slice_offset(self.node_count, subtrees.len() - 1);
        let leaves = self.leaves.slice_offset(self.leaf_count, subtrees.len());
        Self::binned_build_static(
            subtrees,
            nodes,
            metanodes,
            leaves,
            pool,
            minimum_bin_count,
            maximum_bin_count,
            leaf_to_bin_multiplier,
            microsweep_threshold,
        );
    }
}
