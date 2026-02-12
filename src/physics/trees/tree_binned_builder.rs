// Translation of BepuPhysics/Trees/Tree_BinnedBuilder.cs
// Implements the binned SAH tree builder used by Tree::binned_build, including MT paths.

use crate::physics::trees::leaf::Leaf;
use crate::physics::trees::node::{Metanode, Node, NodeChild};
use crate::physics::trees::tree::Tree;
use crate::utilities::bounding_box::BoundingBox4;
use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::quicksort::Quicksort;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::task_scheduling::{ContinuationHandle, EqualTagFilter, IJobFilter, Task, TaskStack};
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use glam::Vec4;
use std::cmp::Ordering;
use std::ffi::c_void;
use std::mem;
use std::sync::atomic::{AtomicI32, Ordering as AtomicOrdering};

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

// ── MT structs and constants ───────────────────────────────────────────────

const MINIMUM_SUBTREES_PER_THREAD_FOR_CENTROID_PREPASS: i32 = 1024;
const MINIMUM_SUBTREES_PER_THREAD_FOR_BINNING: i32 = 1024;
const MINIMUM_SUBTREES_PER_THREAD_FOR_PARTITIONING: i32 = 1024;
const MINIMUM_SUBTREES_PER_THREAD_FOR_NODE_JOB: i32 = 256;
const TARGET_TASK_COUNT_MULTIPLIER_FOR_NODE_PUSH_OVER_INNER_LOOP: i32 = 8;
/// Random value stored in the upper 32 bits of the job tag for internal multithreading.
const JOB_FILTER_TAG_HEADER: u64 = 0xB0A1BF32u64 << 32;

/// Per-worker bin storage persisting across the build.
#[derive(Clone, Copy)]
struct BinnedBuildWorkerContext {
    bin_bounding_boxes: Buffer<BoundingBox4>,
    bin_centroid_bounding_boxes: Buffer<BoundingBox4>,
    bin_bounding_boxes_scan: Buffer<BoundingBox4>,
    bin_centroid_bounding_boxes_scan: Buffer<BoundingBox4>,
    bin_leaf_counts: Buffer<i32>,
}

impl BinnedBuildWorkerContext {
    unsafe fn new(bin_allocation_buffer: Buffer<u8>, start: &mut i32, bin_capacity: i32) -> Self {
        Self {
            bin_bounding_boxes: suballocate(bin_allocation_buffer, start, bin_capacity),
            bin_centroid_bounding_boxes: suballocate(bin_allocation_buffer, start, bin_capacity),
            bin_bounding_boxes_scan: suballocate(bin_allocation_buffer, start, bin_capacity),
            bin_centroid_bounding_boxes_scan: suballocate(bin_allocation_buffer, start, bin_capacity),
            bin_leaf_counts: suballocate(bin_allocation_buffer, start, bin_capacity),
        }
    }
}

/// Multithreaded binned build context implementing per-worker bin dispatch.
struct MultithreadBinnedBuildContext {
    task_stack: *mut TaskStack,
    original_subtree_count: i32,
    top_level_target_task_count: i32,
    workers: Buffer<BinnedBuildWorkerContext>,
}

impl MultithreadBinnedBuildContext {
    #[inline(always)]
    unsafe fn get_bins(
        &self,
        worker_index: i32,
    ) -> (
        Buffer<BoundingBox4>,
        Buffer<BoundingBox4>,
        Buffer<BoundingBox4>,
        Buffer<BoundingBox4>,
        Buffer<i32>,
    ) {
        let worker = &*self.workers.get(worker_index);
        (
            worker.bin_bounding_boxes,
            worker.bin_centroid_bounding_boxes,
            worker.bin_bounding_boxes_scan,
            worker.bin_centroid_bounding_boxes_scan,
            worker.bin_leaf_counts,
        )
    }

    #[inline(always)]
    fn get_target_task_count_for_inner_loop(&self, subtree_count: i32) -> i32 {
        (self.top_level_target_task_count as f32 * subtree_count as f32
            / self.original_subtree_count as f32)
            .ceil() as i32
    }

    #[inline(always)]
    fn get_target_task_count_for_nodes(&self, subtree_count: i32) -> i32 {
        (TARGET_TASK_COUNT_MULTIPLIER_FOR_NODE_PUSH_OVER_INNER_LOOP as f32
            * self.top_level_target_task_count as f32
            * subtree_count as f32
            / self.original_subtree_count as f32)
            .ceil() as i32
    }
}

/// Shared data for distributing subtree slots across tasks.
struct SharedTaskData {
    worker_count: i32,
    task_count: i32,
    subtree_start_index: i32,
    subtree_count: i32,
    slots_per_task_base: i32,
    slot_remainder: i32,
    task_count_fits_in_worker_count: bool,
}

impl SharedTaskData {
    fn new(
        worker_count: i32,
        subtree_start_index: i32,
        slot_count: i32,
        minimum_slots_per_task: i32,
        target_task_count: i32,
    ) -> Self {
        let task_size = minimum_slots_per_task.max(slot_count / target_task_count);
        let task_count = (slot_count + task_size - 1) / task_size;
        let slots_per_task_base = slot_count / task_count;
        let slot_remainder = slot_count - task_count * slots_per_task_base;
        Self {
            worker_count,
            task_count,
            subtree_start_index,
            subtree_count: slot_count,
            slots_per_task_base,
            slot_remainder,
            task_count_fits_in_worker_count: task_count <= worker_count,
        }
    }

    #[inline(always)]
    fn get_slot_interval(&self, task_id: i64) -> (i32, i32) {
        let remaindered_task_count = self.slot_remainder.min(task_id as i32);
        let early_slot_count =
            (self.slots_per_task_base + 1) as i64 * remaindered_task_count as i64;
        let late_slot_count =
            self.slots_per_task_base as i64 * (task_id - remaindered_task_count as i64);
        let start = self.subtree_start_index + (early_slot_count + late_slot_count) as i32;
        let count = if task_id >= self.slot_remainder as i64 {
            self.slots_per_task_base
        } else {
            self.slots_per_task_base + 1
        };
        (start, count)
    }
}

/// Context for multithreaded centroid prepass.
struct CentroidPrepassTaskContext {
    task_data: SharedTaskData,
    prepass_workers: Buffer<BoundingBox4>,
    bounds: Buffer<BoundingBox4>,
}

/// Per-worker context for bin subtrees MT.
#[derive(Clone, Copy)]
struct BinSubtreesWorkerContext {
    bin_bounding_boxes: Buffer<BoundingBox4>,
    bin_centroid_bounding_boxes: Buffer<BoundingBox4>,
    bin_leaf_counts: Buffer<i32>,
}

/// Context for multithreaded subtree binning.
struct BinSubtreesTaskContext {
    task_data: SharedTaskData,
    bin_subtrees_workers: Buffer<BinSubtreesWorkerContext>,
    worker_helped_with_binning: Buffer<bool>,
    subtrees: Buffer<NodeChild>,
    bin_indices: Buffer<u8>,
    bin_count: i32,
    use_x: bool,
    use_y: bool,
    axis_index: i32,
    centroid_bounds_min: Vec4,
    offset_to_bin_index: Vec4,
    maximum_bin_index: Vec4,
}

/// Partition counters with cache-line padding to prevent false sharing.
#[repr(C)]
struct PartitionCounters {
    _pad_a: [u8; 128],
    subtree_count_a: i32,
    _pad_mid: [u8; 2], // C# has 6 byte gap (offset 128 for A, 134 for B)
    subtree_count_b: i32,
    _pad_b: [u8; 126],
}

/// Context for multithreaded partition.
struct PartitionTaskContext {
    task_data: SharedTaskData,
    subtrees: Buffer<NodeChild>,
    subtrees_next: Buffer<NodeChild>,
    bin_indices: Buffer<u8>,
    bin_split_index: i32,
    counters: PartitionCounters,
}

/// Context for pushing child B node work onto the task stack.
#[repr(C)]
struct NodePushTaskContext {
    context: *mut BinnedBuilderContext,
    node_index: i32,
    parent_node_index: i32,
    centroid_bounds: BoundingBox4,
}

// ── MT worker functions ────────────────────────────────────────────────────

unsafe fn centroid_prepass_worker(
    task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    _dispatcher: &dyn IThreadDispatcher,
) {
    let context = &mut *(untyped_context as *mut CentroidPrepassTaskContext);
    let (start, count) = context.task_data.get_slot_interval(task_id);
    let bounds_slice_ptr = context.bounds.as_ptr().add(start as usize);
    let centroid_bounds = compute_centroid_bounds(bounds_slice_ptr, count);
    if context.task_data.task_count_fits_in_worker_count {
        *context.prepass_workers.get_mut(task_id as i32) = centroid_bounds;
    } else {
        let worker_bounds = context.prepass_workers.get_mut(worker_index);
        worker_bounds.min = worker_bounds.min.min(centroid_bounds.min);
        worker_bounds.max = worker_bounds.max.max(centroid_bounds.max);
    }
}

unsafe fn multithreaded_centroid_prepass(
    mt_context: *mut MultithreadBinnedBuildContext,
    bounds: Buffer<BoundingBox4>,
    task_data: &SharedTaskData,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) -> BoundingBox4 {
    let worker_pool = &mut *dispatcher.worker_pool_ptr(worker_index);
    let prepass_workers: Buffer<BoundingBox4> =
        worker_pool.take_at_least(task_data.worker_count.min(task_data.task_count));
    let task_count = task_data.task_count;
    let active_worker_count = task_data.worker_count.min(task_count);

    let mut task_context = CentroidPrepassTaskContext {
        task_data: SharedTaskData::new(
            task_data.worker_count,
            task_data.subtree_start_index,
            task_data.subtree_count,
            MINIMUM_SUBTREES_PER_THREAD_FOR_CENTROID_PREPASS,
            task_count,
        ),
        prepass_workers,
        bounds,
    };
    // Re-share the computed task_count
    task_context.task_data.task_count = task_count;

    if task_count > task_data.worker_count {
        for i in 0..active_worker_count {
            let wb = task_context.prepass_workers.get_mut(i);
            wb.min = Vec4::splat(f32::MAX);
            wb.max = Vec4::splat(f32::MIN);
        }
    }

    let tag_value = (worker_index as u64) | JOB_FILTER_TAG_HEADER;
    let job_filter = EqualTagFilter::new(tag_value);
    (*(*mt_context).task_stack).for_loop(
        centroid_prepass_worker,
        &mut task_context as *mut CentroidPrepassTaskContext as *mut c_void,
        0,
        task_count,
        worker_index,
        dispatcher,
        &job_filter,
        tag_value,
    );

    let mut result = *task_context.prepass_workers.get(0);
    for i in 1..active_worker_count {
        let wb = task_context.prepass_workers.get(i);
        result.min = result.min.min(wb.min);
        result.max = result.max.max(wb.max);
    }
    let mut pw = task_context.prepass_workers;
    worker_pool.return_buffer(&mut pw);
    result
}

unsafe fn bin_subtrees_worker(
    task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    _dispatcher: &dyn IThreadDispatcher,
) {
    let context = &mut *(untyped_context as *mut BinSubtreesTaskContext);
    let effective_worker_index = if context.task_data.task_count_fits_in_worker_count {
        task_id as i32
    } else {
        worker_index
    };
    let worker = context
        .bin_subtrees_workers
        .get_mut(effective_worker_index);
    *context
        .worker_helped_with_binning
        .get_mut(effective_worker_index) = true;

    if context.task_data.task_count_fits_in_worker_count {
        for i in 0..context.bin_count {
            let bb = worker.bin_bounding_boxes.get_mut(i);
            bb.min = Vec4::splat(f32::MAX);
            bb.max = Vec4::splat(f32::MIN);
            let cbb = worker.bin_centroid_bounding_boxes.get_mut(i);
            cbb.min = Vec4::splat(f32::MAX);
            cbb.max = Vec4::splat(f32::MIN);
            *worker.bin_leaf_counts.get_mut(i) = 0;
        }
    }

    let (start, count) = context.task_data.get_slot_interval(task_id);
    bin_subtrees(
        context.centroid_bounds_min,
        context.use_x,
        context.use_y,
        context.axis_index,
        context.offset_to_bin_index,
        context.maximum_bin_index,
        context.subtrees.slice_offset(start, count),
        worker.bin_bounding_boxes,
        worker.bin_centroid_bounding_boxes,
        worker.bin_leaf_counts,
        context.bin_indices.slice_offset(start, count),
        true, // Always write bin indices in MT path
    );
}

#[allow(clippy::too_many_arguments)]
unsafe fn multithreaded_bin_subtrees(
    mt_context: *mut MultithreadBinnedBuildContext,
    centroid_bounds_min: Vec4,
    use_x: bool,
    use_y: bool,
    axis_index: i32,
    offset_to_bin_index: Vec4,
    maximum_bin_index: Vec4,
    subtrees: Buffer<NodeChild>,
    subtree_bin_indices: Buffer<u8>,
    bin_count: i32,
    task_data: &SharedTaskData,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let worker_pool = &mut *dispatcher.worker_pool_ptr(worker_index);
    let effective_worker_count = (*mt_context).workers.len().min(task_data.task_count);

    // Bulk allocation for worker contexts
    let alloc_size = (mem::size_of::<BinSubtreesWorkerContext>() as i32
        + (mem::size_of::<BoundingBox4>() as i32 * 2 + mem::size_of::<i32>() as i32) * bin_count
        + mem::size_of::<bool>() as i32 * task_data.worker_count)
        * effective_worker_count;
    let allocation: Buffer<u8> = worker_pool.take_at_least(alloc_size);
    let mut alloc_start = 0i32;
    let bin_subtrees_workers: Buffer<BinSubtreesWorkerContext> =
        suballocate(allocation, &mut alloc_start, effective_worker_count);
    for i in 0..effective_worker_count {
        let w = &mut *bin_subtrees_workers.as_ptr().add(i as usize).cast_mut();
        w.bin_bounding_boxes = suballocate(allocation, &mut alloc_start, bin_count);
        w.bin_centroid_bounding_boxes = suballocate(allocation, &mut alloc_start, bin_count);
        w.bin_leaf_counts = suballocate(allocation, &mut alloc_start, bin_count);
    }
    let worker_helped: Buffer<bool> =
        suballocate(allocation, &mut alloc_start, effective_worker_count);
    for i in 0..effective_worker_count {
        *worker_helped.as_ptr().add(i as usize).cast_mut() = false;
    }

    let mut task_context = BinSubtreesTaskContext {
        task_data: SharedTaskData::new(
            (*mt_context).workers.len(),
            0,
            subtrees.len(),
            MINIMUM_SUBTREES_PER_THREAD_FOR_BINNING,
            (*mt_context).get_target_task_count_for_inner_loop(subtrees.len()),
        ),
        bin_subtrees_workers,
        worker_helped_with_binning: worker_helped,
        subtrees,
        bin_indices: subtree_bin_indices,
        bin_count,
        use_x,
        use_y,
        axis_index,
        centroid_bounds_min,
        offset_to_bin_index,
        maximum_bin_index,
    };
    task_context.task_data.task_count = task_data.task_count;
    let active_worker_count = effective_worker_count.min(task_context.task_data.task_count);

    if !task_context.task_data.task_count_fits_in_worker_count {
        for cache_index in 0..active_worker_count {
            let cache = task_context.bin_subtrees_workers.get_mut(cache_index);
            for i in 0..bin_count {
                let bb = cache.bin_bounding_boxes.get_mut(i);
                bb.min = Vec4::splat(f32::MAX);
                bb.max = Vec4::splat(f32::MIN);
                let cbb = cache.bin_centroid_bounding_boxes.get_mut(i);
                cbb.min = Vec4::splat(f32::MAX);
                cbb.max = Vec4::splat(f32::MIN);
                *cache.bin_leaf_counts.get_mut(i) = 0;
            }
        }
    }

    let tag_value = (worker_index as u64) | JOB_FILTER_TAG_HEADER;
    let job_filter = EqualTagFilter::new(tag_value);
    (*(*mt_context).task_stack).for_loop(
        bin_subtrees_worker,
        &mut task_context as *mut BinSubtreesTaskContext as *mut c_void,
        0,
        task_context.task_data.task_count,
        worker_index,
        dispatcher,
        &job_filter,
        tag_value,
    );

    // Merge worker results into the dispatching worker's bins.
    let main_worker = (*mt_context).workers.get_mut(worker_index);
    let cache0 = task_context.bin_subtrees_workers.get(0);
    cache0
        .bin_bounding_boxes
        .copy_to(0, &mut main_worker.bin_bounding_boxes, 0, bin_count);
    cache0
        .bin_centroid_bounding_boxes
        .copy_to(0, &mut main_worker.bin_centroid_bounding_boxes, 0, bin_count);
    cache0
        .bin_leaf_counts
        .copy_to(0, &mut main_worker.bin_leaf_counts, 0, bin_count);

    for cache_index in 1..active_worker_count {
        if *task_context.worker_helped_with_binning.get(cache_index) {
            let cache = task_context.bin_subtrees_workers.get(cache_index);
            for bin_index in 0..bin_count {
                let b0 = main_worker.bin_bounding_boxes.get_mut(bin_index);
                let bi = cache.bin_bounding_boxes.get(bin_index);
                b0.min = b0.min.min(bi.min);
                b0.max = b0.max.max(bi.max);
                let bc0 = main_worker.bin_centroid_bounding_boxes.get_mut(bin_index);
                let bci = cache.bin_centroid_bounding_boxes.get(bin_index);
                bc0.min = bc0.min.min(bci.min);
                bc0.max = bc0.max.max(bci.max);
                *main_worker.bin_leaf_counts.get_mut(bin_index) +=
                    *cache.bin_leaf_counts.get(bin_index);
            }
        }
    }

    let mut alloc_buf = allocation;
    worker_pool.return_buffer(&mut alloc_buf);
}

unsafe fn partition_subtrees_worker(
    task_id: i64,
    untyped_context: *mut c_void,
    _worker_index: i32,
    _dispatcher: &dyn IThreadDispatcher,
) {
    let context = &mut *(untyped_context as *mut PartitionTaskContext);
    let (start, count) = context.task_data.get_slot_interval(task_id);

    const BATCH_SIZE: i32 = 16384;
    let mut slot_belongs_to_a = vec![0u8; BATCH_SIZE as usize];

    let batch_count = (count + BATCH_SIZE - 1) / BATCH_SIZE;
    for batch_index in 0..batch_count {
        let mut local_count_a = 0i32;
        let batch_start = start + batch_index * BATCH_SIZE;
        let count_in_batch = (start + count - batch_start).min(BATCH_SIZE);

        for index_in_batch in 0..count_in_batch {
            let subtree_index = (index_in_batch + batch_start) as usize;
            let bin_index = *context.bin_indices.as_ptr().add(subtree_index);
            let belongs_to_a = (bin_index as i32) < context.bin_split_index;
            slot_belongs_to_a[index_in_batch as usize] = if belongs_to_a { 0xFF } else { 0 };
            if belongs_to_a {
                local_count_a += 1;
            }
        }

        let local_count_b = count_in_batch - local_count_a;
        let counter_a_ptr = &context.counters.subtree_count_a as *const i32 as *mut i32;
        let counter_b_ptr = &context.counters.subtree_count_b as *const i32 as *mut i32;
        let start_index_a = AtomicI32::from_ptr(counter_a_ptr)
            .fetch_add(local_count_a, AtomicOrdering::AcqRel);
        let start_index_b = context.subtrees.len()
            - AtomicI32::from_ptr(counter_b_ptr)
                .fetch_add(local_count_b, AtomicOrdering::AcqRel)
            - local_count_b;

        let mut recount_a = 0;
        let mut recount_b = 0;
        for index_in_batch in 0..count_in_batch {
            let target_index = if slot_belongs_to_a[index_in_batch as usize] != 0 {
                let t = start_index_a + recount_a;
                recount_a += 1;
                t
            } else {
                let t = start_index_b + recount_b;
                recount_b += 1;
                t
            };
            *context
                .subtrees_next
                .as_ptr()
                .add(target_index as usize)
                .cast_mut() =
                *context
                    .subtrees
                    .as_ptr()
                    .add((batch_start + index_in_batch) as usize);
        }
    }
}

unsafe fn multithreaded_partition(
    mt_context: *mut MultithreadBinnedBuildContext,
    subtrees: Buffer<NodeChild>,
    subtrees_next: Buffer<NodeChild>,
    bin_indices: Buffer<u8>,
    bin_split_index: i32,
    task_data: &SharedTaskData,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) -> (i32, i32) {
    let mut task_context = PartitionTaskContext {
        task_data: SharedTaskData::new(
            (*mt_context).workers.len(),
            0,
            subtrees.len(),
            MINIMUM_SUBTREES_PER_THREAD_FOR_PARTITIONING,
            (*mt_context).get_target_task_count_for_inner_loop(subtrees.len()),
        ),
        subtrees,
        subtrees_next,
        bin_indices,
        bin_split_index,
        counters: PartitionCounters {
            _pad_a: [0; 128],
            subtree_count_a: 0,
            _pad_mid: [0; 2],
            subtree_count_b: 0,
            _pad_b: [0; 126],
        },
    };
    task_context.task_data.task_count = task_data.task_count;

    let tag_value = (worker_index as u64) | JOB_FILTER_TAG_HEADER;
    let job_filter = EqualTagFilter::new(tag_value);
    (*(*mt_context).task_stack).for_loop(
        partition_subtrees_worker,
        &mut task_context as *mut PartitionTaskContext as *mut c_void,
        0,
        task_context.task_data.task_count,
        worker_index,
        dispatcher,
        &job_filter,
        tag_value,
    );

    (
        task_context.counters.subtree_count_a,
        task_context.counters.subtree_count_b,
    )
}

unsafe fn binned_builder_node_worker(
    task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let subtree_region_start_index = task_id as i32;
    let subtree_count = ((task_id >> 32) & 0x7FFF_FFFF) as i32;
    let use_pong_buffer = (task_id as u64) >= (1u64 << 63);
    let node_push_context = &*(untyped_context as *const NodePushTaskContext);
    // Child index is always 1 because we only ever push child B.
    binned_build_node(
        use_pong_buffer,
        subtree_region_start_index,
        node_push_context.node_index,
        subtree_count,
        node_push_context.parent_node_index,
        1,
        &node_push_context.centroid_bounds,
        node_push_context.context,
        worker_index,
        Some(dispatcher),
    );
}

unsafe fn binned_builder_worker_entry(
    _task_id: i64,
    untyped_context: *mut c_void,
    worker_index: i32,
    dispatcher: &dyn IThreadDispatcher,
) {
    let context = untyped_context as *mut BinnedBuilderContext;
    binned_build_node(
        false,
        0,
        0,
        (*context).subtrees_ping.len(),
        -1,
        -1,
        &BoundingBox4 {
            min: Vec4::ZERO,
            max: Vec4::ZERO,
        },
        context,
        worker_index,
        Some(dispatcher),
    );
    // Once the entry point returns, all workers should stop.
    (*(*(*context).mt_threading.unwrap()).task_stack).request_stop();
}

// ── Context for the recursive builder ──────────────────────────────────────

struct BinnedBuilderContext {
    minimum_bin_count: i32,
    maximum_bin_count: i32,
    leaf_to_bin_multiplier: f32,
    microsweep_threshold: i32,
    deterministic: bool,

    subtrees_ping: Buffer<NodeChild>,
    subtrees_pong: Buffer<NodeChild>,
    nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    bin_indices: Buffer<u8>,
    leaves: Buffer<Leaf>,
    /// Whether leaves should be written (vs post-pass).
    handle_leaves: bool,

    bins: SingleThreadedBins,
    /// If Some, the builder uses multithreaded dispatch.
    mt_threading: Option<*mut MultithreadBinnedBuildContext>,
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
    ctx: *mut BinnedBuilderContext,
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
        &mut (*ctx).leaves,
        (*ctx).handle_leaves,
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
    ctx: *mut BinnedBuilderContext,
    worker_index: i32,
    dispatcher: Option<&dyn IThreadDispatcher>,
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
            worker_index,
            dispatcher,
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
            worker_index,
            dispatcher,
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
    ctx: *mut BinnedBuilderContext,
    worker_index: i32,
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
            worker_index,
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
            worker_index,
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
    ctx: *mut BinnedBuilderContext,
    worker_index: i32,
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
            &mut (*ctx).leaves,
            (*ctx).handle_leaves,
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
            worker_index,
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

    // Get the scan buffer from the appropriate bins (MT per-worker or ST).
    let mut bin_bounding_boxes_scan = if let Some(mt_ctx) = (*ctx).mt_threading {
        (*mt_ctx).get_bins(worker_index).2
    } else {
        (*ctx).bins.bin_bounding_boxes_scan
    };
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
            worker_index,
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
        &mut (*ctx).leaves,
        (*ctx).handle_leaves,
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
            worker_index,
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
            worker_index,
        );
    }
}

// ── Main recursive build node ──────────────────────────────────────────────

/// Core recursive binned build function.
#[allow(clippy::too_many_arguments)]
unsafe fn binned_build_node(
    mut use_pong_buffer: bool,
    subtree_region_start_index: i32,
    node_index: i32,
    subtree_count: i32,
    parent_node_index: i32,
    child_index_in_parent: i32,
    centroid_bounds: &BoundingBox4,
    ctx: *mut BinnedBuilderContext,
    worker_index: i32,
    dispatcher: Option<&dyn IThreadDispatcher>,
) {
    let subtrees = if use_pong_buffer {
        (*ctx).subtrees_pong
    } else {
        (*ctx).subtrees_ping
    }
    .slice_offset(subtree_region_start_index, subtree_count);

    let subtree_bin_indices = if (*ctx).bin_indices.allocated() {
        (*ctx).bin_indices
            .slice_offset(subtree_region_start_index, subtree_count)
    } else {
        Buffer::default()
    };

    let bounding_boxes: Buffer<BoundingBox4> = subtrees.cast();
    let node_count = subtree_count - 1;
    let nodes = (*ctx).nodes.slice_offset(node_index, node_count);
    let metanodes = if (*ctx).metanodes.allocated() {
        (*ctx).metanodes.slice_offset(node_index, node_count)
    } else {
        (*ctx).metanodes
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
            &mut (*ctx).leaves,
            (*ctx).handle_leaves,
        );
        return;
    }

    let mt_context_ptr = (*ctx).mt_threading;
    let target_task_count = if let Some(mt_ctx) = mt_context_ptr {
        (*mt_ctx).get_target_task_count_for_inner_loop(subtree_count)
    } else {
        1
    };

    // For the root node, compute centroid bounds since they aren't provided.
    let centroid_bounds = if node_index == 0 {
        let mut use_st = true;
        let mut mt_centroid_bounds = BoundingBox4 {
            min: Vec4::ZERO,
            max: Vec4::ZERO,
        };
        if let Some(mt_ctx) = mt_context_ptr {
            let task_data = SharedTaskData::new(
                (*mt_ctx).workers.len(),
                0,
                subtrees.len(),
                MINIMUM_SUBTREES_PER_THREAD_FOR_CENTROID_PREPASS,
                (*mt_ctx).get_target_task_count_for_inner_loop(subtree_count),
            );
            if task_data.task_count > 1 {
                mt_centroid_bounds = multithreaded_centroid_prepass(
                    mt_ctx,
                    bounding_boxes,
                    &task_data,
                    worker_index,
                    dispatcher.unwrap(),
                );
                use_st = false;
            }
        }
        if use_st {
            compute_centroid_bounds(bounding_boxes.as_ptr(), subtrees.len())
        } else {
            mt_centroid_bounds
        }
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
            worker_index,
            dispatcher,
        );
        return;
    }

    // Fall back to microsweep for small counts.
    if subtree_count <= (*ctx).microsweep_threshold {
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
            worker_index,
        );
        return;
    }

    let use_x = centroid_span.x > centroid_span.y && centroid_span.x > centroid_span.z;
    let use_y = centroid_span.y > centroid_span.z;
    let axis_index: i32 = if use_x { 0 } else if use_y { 1 } else { 2 };

    let bin_count = (subtree_count as f32 * (*ctx).leaf_to_bin_multiplier) as i32;
    let bin_count = bin_count.max((*ctx).minimum_bin_count).min((*ctx).maximum_bin_count);

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

    // Get bins from the appropriate source (MT per-worker or ST).
    let (mut bin_bounding_boxes, mut bin_centroid_bounding_boxes, mut bin_bounding_boxes_scan, mut bin_centroid_bounding_boxes_scan, mut bin_leaf_counts) =
        if let Some(mt_ctx) = mt_context_ptr {
            (*mt_ctx).get_bins(worker_index)
        } else {
            (
                (*ctx).bins.bin_bounding_boxes,
                (*ctx).bins.bin_centroid_bounding_boxes,
                (*ctx).bins.bin_bounding_boxes_scan,
                (*ctx).bins.bin_centroid_bounding_boxes_scan,
                (*ctx).bins.bin_leaf_counts,
            )
        };

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

    // Bin subtrees — potentially multithreaded.
    let mut use_st_for_binning = true;
    if let Some(mt_ctx) = mt_context_ptr {
        let task_data = SharedTaskData::new(
            (*mt_ctx).workers.len(),
            0,
            subtrees.len(),
            MINIMUM_SUBTREES_PER_THREAD_FOR_BINNING,
            (*mt_ctx).get_target_task_count_for_inner_loop(subtree_count),
        );
        if task_data.task_count > 1 {
            multithreaded_bin_subtrees(
                mt_ctx,
                centroid_bounds.min,
                use_x,
                use_y,
                axis_index,
                offset_to_bin_index,
                maximum_bin_index,
                subtrees,
                subtree_bin_indices,
                bin_count,
                &task_data,
                worker_index,
                dispatcher.unwrap(),
            );
            use_st_for_binning = false;
        }
    }
    if use_st_for_binning {
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
    }

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
            worker_index,
            dispatcher,
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
    let partitioned_subtrees;
    if (*ctx).subtrees_pong.allocated() {
        debug_assert!(subtree_bin_indices.allocated());
        let mut subtrees_next = if use_pong_buffer {
            (*ctx).subtrees_ping
        } else {
            (*ctx).subtrees_pong
        }
        .slice_offset(subtree_region_start_index, subtree_count);

        let mut use_st_for_partitioning = true;
        // MT partition is nondeterministic; skip when deterministic mode requested.
        if let Some(mt_ctx) = mt_context_ptr {
            if !(*ctx).deterministic {
                let task_data = SharedTaskData::new(
                    (*mt_ctx).workers.len(),
                    0,
                    subtrees.len(),
                    MINIMUM_SUBTREES_PER_THREAD_FOR_PARTITIONING,
                    (*mt_ctx).get_target_task_count_for_inner_loop(subtree_count),
                );
                if task_data.task_count > 1 {
                    let (ca, cb) = multithreaded_partition(
                        mt_ctx,
                        subtrees,
                        subtrees_next,
                        subtree_bin_indices,
                        split_index,
                        &task_data,
                        worker_index,
                        dispatcher.unwrap(),
                    );
                    subtree_count_a = ca;
                    subtree_count_b = cb;
                    use_st_for_partitioning = false;
                }
            }
        }
        if use_st_for_partitioning {
            // Single-threaded partition using precomputed bin indices.
            for i in 0..subtree_count {
                let bin_idx = *subtree_bin_indices.as_ptr().add(i as usize);
                let target_index = if bin_idx as i32 >= split_index {
                    subtree_count_b += 1;
                    subtree_count - subtree_count_b
                } else {
                    let t = subtree_count_a;
                    subtree_count_a += 1;
                    t
                };
                *subtrees_next.as_mut_ptr().add(target_index as usize) =
                    *subtrees.as_ptr().add(i as usize);
            }
        }

        partitioned_subtrees = subtrees_next;
        use_pong_buffer = !use_pong_buffer;
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
        &mut (*ctx).leaves,
        (*ctx).handle_leaves,
    );

    // Determine whether to push child B onto the MT task stack.
    let target_node_task_count = if let Some(mt_ctx) = mt_context_ptr {
        (*mt_ctx).get_target_task_count_for_nodes(subtree_count)
    } else {
        1
    };
    let should_push_b_onto_mt_queue = target_node_task_count > 1
        && subtree_count_a >= MINIMUM_SUBTREES_PER_THREAD_FOR_NODE_JOB
        && subtree_count_b >= MINIMUM_SUBTREES_PER_THREAD_FOR_NODE_JOB;

    let mut node_b_continuation = ContinuationHandle::default();
    if should_push_b_onto_mt_queue {
        // Both children are large. Push child B onto the task stack for parallel execution.
        debug_assert!(MINIMUM_SUBTREES_PER_THREAD_FOR_NODE_JOB > 1);
        let mt_ctx = mt_context_ptr.unwrap();
        // Allocate params on the local stack — we must preserve the stack frame until WaitForCompletion.
        let mut node_push_context = NodePushTaskContext {
            context: ctx,
            node_index: node_child_index_b,
            parent_node_index: node_index,
            centroid_bounds: best_centroid_bb_b,
        };
        // Encode subtree start, count, and pong flag into the task id.
        debug_assert!((subtree_count_b as u32) < (1u32 << 31));
        let task_id = (subtree_region_start_index + subtree_count_a) as i64
            | ((subtree_count_b as i64) << 32)
            | if use_pong_buffer { 1i64 << 63 } else { 0 };
        let mut task = Task::with_context(
            binned_builder_node_worker,
            &mut node_push_context as *mut NodePushTaskContext as *mut c_void,
            task_id,
        );
        let disp = dispatcher.unwrap();
        node_b_continuation = (*(*mt_ctx).task_stack).allocate_continuation_and_push(
            std::slice::from_mut(&mut task),
            worker_index,
            disp,
            0,
            Task::default(),
        );
    }

    if subtree_count_a > 1 {
        binned_build_node(
            use_pong_buffer,
            subtree_region_start_index,
            node_child_index_a,
            subtree_count_a,
            node_index,
            0,
            &best_centroid_bb_a,
            ctx,
            worker_index,
            dispatcher,
        );
    }
    if !should_push_b_onto_mt_queue && subtree_count_b > 1 {
        binned_build_node(
            use_pong_buffer,
            subtree_region_start_index + subtree_count_a,
            node_child_index_b,
            subtree_count_b,
            node_index,
            1,
            &best_centroid_bb_b,
            ctx,
            worker_index,
            dispatcher,
        );
    }
    if should_push_b_onto_mt_queue {
        // Keep the stack alive until the node push task completes. WaitForCompletion processes pending work.
        debug_assert!(node_b_continuation.initialized());
        let mt_ctx = mt_context_ptr.unwrap();
        (*(*mt_ctx).task_stack).wait_for_completion_unfiltered(
            node_b_continuation,
            worker_index,
            dispatcher.unwrap(),
        );
    }
}

// ── BinnedBuilderInternal ──────────────────────────────────────────────────

/// Internal entry point: validates and prepares context, then delegates to `binned_build_node`.
#[allow(clippy::too_many_arguments)]
unsafe fn binned_builder_internal(
    subtrees: Buffer<NodeChild>,
    subtrees_pong: Buffer<NodeChild>,
    mut nodes: Buffer<Node>,
    metanodes: Buffer<Metanode>,
    leaves: Buffer<Leaf>,
    bin_indices: Buffer<u8>,
    dispatcher: Option<&dyn IThreadDispatcher>,
    task_stack_pointer: Option<*mut TaskStack>,
    worker_index: i32,
    worker_count: i32,
    target_task_count: i32,
    pool: Option<*mut BufferPool>,
    minimum_bin_count: i32,
    maximum_bin_count: i32,
    leaf_to_bin_multiplier: f32,
    microsweep_threshold: i32,
    deterministic: bool,
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

    if dispatcher.is_none() && task_stack_pointer.is_none() {
        // Single-threaded path.
        let allocated_byte_count = allocated_bin_count as usize
            * (4 * mem::size_of::<BoundingBox4>() + mem::size_of::<i32>());
        let mut bin_bounds_allocation = vec![0u8; allocated_byte_count + 32];
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
            deterministic,
            subtrees_ping: subtrees,
            subtrees_pong,
            nodes,
            metanodes,
            bin_indices,
            leaves,
            handle_leaves,
            bins,
            mt_threading: None,
        };

        binned_build_node(
            false,
            0,
            0,
            subtree_count,
            -1,
            -1,
            &BoundingBox4 {
                min: Vec4::ZERO,
                max: Vec4::ZERO,
            },
            &mut context as *mut BinnedBuilderContext,
            worker_index,
            None,
        );
    } else {
        // Multithreaded dispatch.
        let disp = dispatcher.unwrap();
        let pool_ptr = pool.unwrap();

        // Allocate per-worker bin storage from the pool.
        let worker_bins_byte_count = allocated_bin_count * worker_count
            * (mem::size_of::<BoundingBox4>() as i32 * 4 + mem::size_of::<i32>() as i32);
        let mut worker_bins_allocation: Buffer<u8> =
            (*pool_ptr).take_at_least(worker_bins_byte_count);

        let mut worker_contexts_vec =
            vec![std::mem::zeroed::<BinnedBuildWorkerContext>(); worker_count as usize];
        let mut bin_alloc_start = 0i32;
        for i in 0..worker_count as usize {
            worker_contexts_vec[i] = BinnedBuildWorkerContext::new(
                worker_bins_allocation,
                &mut bin_alloc_start,
                allocated_bin_count,
            );
        }
        let worker_contexts = Buffer::new(
            worker_contexts_vec.as_mut_ptr(),
            worker_count,
            0,
        );

        let mut task_stack_storage: Option<TaskStack> = None;
        let actual_task_stack_ptr = if let Some(ts_ptr) = task_stack_pointer {
            ts_ptr
        } else {
            task_stack_storage = Some(TaskStack::new(&mut *pool_ptr, disp, worker_count, 128, 128));
            task_stack_storage.as_mut().unwrap() as *mut TaskStack
        };
        let dispatch_internally = task_stack_pointer.is_none();

        let mut threading = MultithreadBinnedBuildContext {
            top_level_target_task_count: target_task_count,
            original_subtree_count: subtrees.len(),
            task_stack: actual_task_stack_ptr,
            workers: worker_contexts,
        };

        // Use a dummy ST bins (won't be used since MT threading is active).
        let dummy_bins = SingleThreadedBins {
            bin_bounding_boxes: Buffer::default(),
            bin_centroid_bounding_boxes: Buffer::default(),
            bin_bounding_boxes_scan: Buffer::default(),
            bin_centroid_bounding_boxes_scan: Buffer::default(),
            bin_leaf_counts: Buffer::default(),
        };
        let handle_leaves = leaves.allocated();
        let mut context = BinnedBuilderContext {
            minimum_bin_count,
            maximum_bin_count,
            leaf_to_bin_multiplier,
            microsweep_threshold,
            deterministic,
            subtrees_ping: subtrees,
            subtrees_pong,
            nodes,
            metanodes,
            bin_indices,
            leaves,
            handle_leaves,
            bins: dummy_bins,
            mt_threading: Some(&mut threading as *mut MultithreadBinnedBuildContext),
        };

        if dispatch_internally {
            debug_assert!(worker_index == 0);
            (*actual_task_stack_ptr).push_unsafely_single(
                Task::with_context(
                    binned_builder_worker_entry,
                    &mut context as *mut BinnedBuilderContext as *mut c_void,
                    0,
                ),
                0,
                disp,
                0,
            );
            TaskStack::dispatch_workers(disp, actual_task_stack_ptr, worker_count);
        } else {
            binned_build_node(
                false,
                0,
                0,
                context.subtrees_ping.len(),
                -1,
                -1,
                &BoundingBox4 {
                    min: Vec4::ZERO,
                    max: Vec4::ZERO,
                },
                &mut context as *mut BinnedBuilderContext,
                worker_index,
                Some(disp),
            );
        }

        if dispatch_internally {
            if let Some(ref mut ts) = task_stack_storage {
                ts.dispose(&mut *pool_ptr, disp);
            }
        }
        (*pool_ptr).return_buffer(&mut worker_bins_allocation);
    }
}

// ── Public API on Tree ─────────────────────────────────────────────────────

impl Tree {
    /// Runs a binned build across the given subtrees buffer (static version).
    ///
    /// Supports both single-threaded and multithreaded execution.
    /// If `dispatcher` is provided, `pool` must also be provided.
    ///
    /// # Safety
    /// The caller must ensure all buffers are valid and large enough.
    #[allow(clippy::too_many_arguments)]
    pub unsafe fn binned_build_static(
        subtrees: Buffer<NodeChild>,
        mut nodes: Buffer<Node>,
        mut metanodes: Buffer<Metanode>,
        leaves: Buffer<Leaf>,
        pool: Option<*mut BufferPool>,
        dispatcher: Option<&dyn IThreadDispatcher>,
        task_stack_pointer: Option<*mut TaskStack>,
        worker_index: i32,
        worker_count: i32,
        target_task_count: i32,
        minimum_bin_count: i32,
        maximum_bin_count: i32,
        leaf_to_bin_multiplier: f32,
        microsweep_threshold: i32,
        deterministic: bool,
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

        if let Some(pool_ptr) = pool {
            subtrees_pong = (*pool_ptr).take_at_least(subtrees.len());
            bin_indices = (*pool_ptr).take_at_least(subtrees.len());
            requires_return = true;
        }

        let effective_worker_count = if dispatcher.is_none() {
            0
        } else if worker_count < 0 {
            dispatcher.unwrap().thread_count()
        } else {
            worker_count
        };

        let effective_target_task_count = if dispatcher.is_none() {
            0
        } else if target_task_count < 0 {
            dispatcher.unwrap().thread_count()
        } else {
            target_task_count
        };

        binned_builder_internal(
            subtrees,
            subtrees_pong,
            nodes,
            metanodes,
            leaves,
            bin_indices,
            dispatcher,
            task_stack_pointer,
            worker_index,
            effective_worker_count,
            effective_target_task_count,
            pool,
            minimum_bin_count,
            maximum_bin_count,
            leaf_to_bin_multiplier,
            microsweep_threshold,
            deterministic,
        );

        if requires_return {
            if let Some(pool_ptr) = pool {
                (*pool_ptr).return_buffer(&mut bin_indices);
                (*pool_ptr).return_buffer(&mut subtrees_pong);
            }
        }
    }

    /// Runs a binned build using `self.nodes`, `self.metanodes`, and `self.leaves`.
    ///
    /// # Safety
    /// The caller must ensure the tree's buffers are properly allocated and sized.
    #[allow(clippy::too_many_arguments)]
    pub unsafe fn binned_build(
        &mut self,
        subtrees: Buffer<NodeChild>,
        pool: Option<*mut BufferPool>,
        dispatcher: Option<&dyn IThreadDispatcher>,
        task_stack_pointer: Option<*mut TaskStack>,
        worker_index: i32,
        worker_count: i32,
        target_task_count: i32,
        minimum_bin_count: i32,
        maximum_bin_count: i32,
        leaf_to_bin_multiplier: f32,
        microsweep_threshold: i32,
        deterministic: bool,
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
            dispatcher,
            task_stack_pointer,
            worker_index,
            worker_count,
            target_task_count,
            minimum_bin_count,
            maximum_bin_count,
            leaf_to_bin_multiplier,
            microsweep_threshold,
            deterministic,
        );
    }
}
