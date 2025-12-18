// Translated from BepuPhysics/CollisionDetection/FreshnessChecker.cs

use crate::physics::collision_detection::pair_cache::PairCache;
use crate::physics::handles::ConstraintHandle;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

/// Identifies a type of preflush job.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PreflushJobType {
    CheckFreshness,
    // Other types will be added by NarrowPhase
}

/// Describes a preflush job to execute, including its type and work range.
#[derive(Clone, Copy, Debug)]
pub struct PreflushJob {
    pub job_type: PreflushJobType,
    pub start: i32,
    pub end: i32,
}

/// Checks for stale collision pairs by reading freshness bytes in bulk (8 at a time)
/// and enqueuing removals for any pair that was not refreshed during the current frame.
pub(crate) struct FreshnessChecker {
    freshness_job_count: i32,
    pair_cache: *mut PairCache,
    constraint_remover: *mut ConstraintRemover,
    pub(crate) cached_dispatcher: Option<*mut dyn IThreadDispatcher>,
}

/// Placeholder for ConstraintRemover — defined in constraint_remover.rs
pub(crate) use super::constraint_remover::ConstraintRemover;

impl FreshnessChecker {
    pub fn new(pair_cache: *mut PairCache, constraint_remover: *mut ConstraintRemover) -> Self {
        Self {
            freshness_job_count: 0,
            pair_cache,
            constraint_remover,
            cached_dispatcher: None,
        }
    }

    pub fn create_jobs(
        &mut self,
        dispatcher: Option<*mut dyn IThreadDispatcher>,
        thread_count: i32,
        jobs: &mut Vec<PreflushJob>,
        _pool: &mut BufferPool,
        mapping_count: i32,
    ) {
        self.cached_dispatcher = dispatcher;
        if mapping_count > 0 {
            if thread_count > 1 {
                const JOBS_PER_THREAD: i32 = 2;
                self.freshness_job_count =
                    i32::min(thread_count * JOBS_PER_THREAD, mapping_count);
                let pairs_per_job = mapping_count / self.freshness_job_count;
                let remainder = mapping_count - pairs_per_job * self.freshness_job_count;
                let mut previous_end = 0i32;
                let mut job_index = 0i32;
                while previous_end < mapping_count {
                    let pairs_in_job = if job_index < remainder {
                        pairs_per_job + 1
                    } else {
                        pairs_per_job
                    };
                    let mut next_end = ((previous_end + pairs_in_job + 7) >> 3) << 3;
                    if next_end > mapping_count {
                        next_end = mapping_count;
                    }
                    jobs.push(PreflushJob {
                        job_type: PreflushJobType::CheckFreshness,
                        start: previous_end,
                        end: next_end,
                    });
                    previous_end = next_end;
                    job_index += 1;
                }
            } else {
                jobs.push(PreflushJob {
                    job_type: PreflushJobType::CheckFreshness,
                    start: 0,
                    end: mapping_count,
                });
            }
        }
    }

    /// Checks freshness in a specific region, enqueuing removals for stale pairs.
    /// Uses 8-byte-wide reads for bulk checking.
    pub unsafe fn check_freshness_in_region(
        &self,
        worker_index: i32,
        start_index: i32,
        end_index: i32,
    ) {
        let pair_cache = &*self.pair_cache;
        let count = end_index - start_index;
        let wide_count = count >> 3;
        let remainder = count - (wide_count << 3);
        debug_assert!(
            (start_index & 7) == 0 || start_index == end_index,
            "Either this job is empty or the start should be 8 byte aligned for quick reading."
        );

        // We check 8 pairs simultaneously by reading a u64.
        let freshness_ptr = pair_cache.pair_freshness.as_ptr() as *const u8;
        let start = freshness_ptr.add(start_index as usize) as *const u64;

        for i in 0..wide_count {
            let freshness_batch = *start.add(i as usize);
            // Perform a binary search for all stale bytes.
            if freshness_batch < 0xFFFF_FFFF_FFFF_FFFF {
                let start_of_wide = start_index + (i << 3);
                if (freshness_batch & 0x0000_0000_FFFF_FFFF) < 0x0000_0000_FFFF_FFFF {
                    if (freshness_batch & 0x0000_0000_0000_FFFF) < 0x0000_0000_0000_FFFF {
                        if (freshness_batch & 0x0000_0000_0000_00FF) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 0);
                        }
                        if (freshness_batch & 0x0000_0000_0000_FF00) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 1);
                        }
                    }
                    if (freshness_batch & 0x0000_0000_FFFF_0000) < 0x0000_0000_FFFF_0000 {
                        if (freshness_batch & 0x0000_0000_00FF_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 2);
                        }
                        if (freshness_batch & 0x0000_0000_FF00_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 3);
                        }
                    }
                }
                if (freshness_batch & 0xFFFF_FFFF_0000_0000) < 0xFFFF_FFFF_0000_0000 {
                    if (freshness_batch & 0x0000_FFFF_0000_0000) < 0x0000_FFFF_0000_0000 {
                        if (freshness_batch & 0x0000_00FF_0000_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 4);
                        }
                        if (freshness_batch & 0x0000_FF00_0000_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 5);
                        }
                    }
                    if (freshness_batch & 0xFFFF_0000_0000_0000) < 0xFFFF_0000_0000_0000 {
                        if (freshness_batch & 0x00FF_0000_0000_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 6);
                        }
                        if (freshness_batch & 0xFF00_0000_0000_0000) == 0 {
                            self.enqueue_stale_removal(worker_index, start_of_wide + 7);
                        }
                    }
                }
            }
        }
        // Check the remainder of the bytes one by one.
        for i in (end_index - remainder)..end_index {
            if *pair_cache.pair_freshness.get(i) == 0 {
                self.enqueue_stale_removal(worker_index, i);
            }
        }
    }

    #[inline(always)]
    unsafe fn enqueue_stale_removal(&self, worker_index: i32, pair_index: i32) {
        let pair_cache = &mut *self.pair_cache;
        let constraint_remover = &mut *self.constraint_remover;

        // Note: we grab the *old* handle, because the current frame's constraint caches do not contain this pair.
        let constraint_handle = pair_cache.get_old_constraint_handle(pair_index);
        constraint_remover.enqueue_removal(worker_index, constraint_handle);

        let pool = if self.cached_dispatcher.is_none() {
            pair_cache.pool
        } else {
            // TODO: Use dispatcher.worker_pools[worker_index] when thread dispatcher is fully hooked up
            pair_cache.pool
        };

        let pending_changes = &mut pair_cache.worker_pending_changes[worker_index as usize];
        let key = unsafe { *pair_cache.mapping.keys.get(pair_index) };
        let pool_ref = unsafe { &mut *pool };
        pending_changes.pending_removes.add(key, pool_ref);
    }
}
