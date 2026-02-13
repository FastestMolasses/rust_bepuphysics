// Translated from BepuPhysics/BatchCompressor.cs

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

use crate::physics::bodies::Bodies;
use crate::physics::constraints::type_batch::TypeBatch;
use crate::physics::handles::ConstraintHandle;
use crate::physics::handy_enumerators::ActiveConstraintDynamicBodyHandleCollector;
use crate::physics::solver::Solver;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

/// Handles the movement of constraints from higher indexed batches into lower indexed batches
/// to avoid accumulating unnecessary ConstraintBatches.
pub struct BatchCompressor {
    solver: *mut Solver,
    bodies: *mut Bodies,
    target_candidate_fraction: f32,
    maximum_compression_fraction: f32,

    /// Index of the constraint batch to optimize.
    next_batch_index: i32,
    next_type_batch_index: i32,

    // MT fields — only valid during `compress`.
    worker_compressions: Buffer<QuickList<Compression>>,
    analysis_job_index: UnsafeCell<i32>,
    analysis_jobs: QuickList<AnalysisRegion>,
}

#[derive(Clone, Copy)]
struct Compression {
    constraint_handle: ConstraintHandle,
    target_batch: i32,
}

#[derive(Clone, Copy)]
struct AnalysisRegion {
    type_batch_index: i32,
    start_index_in_type_batch: i32,
    end_index_in_type_batch: i32,
}

impl BatchCompressor {
    pub fn new(
        solver: *mut Solver,
        bodies: *mut Bodies,
        target_candidate_fraction: f32,
        maximum_compression_fraction: f32,
    ) -> Self {
        Self {
            solver,
            bodies,
            target_candidate_fraction: target_candidate_fraction.clamp(0.0, 1.0),
            maximum_compression_fraction: maximum_compression_fraction.clamp(0.0, 1.0),
            next_batch_index: 0,
            next_type_batch_index: 0,
            worker_compressions: Buffer::default(),
            analysis_job_index: UnsafeCell::new(0),
            analysis_jobs: QuickList::default(),
        }
    }

    pub fn with_defaults(solver: *mut Solver, bodies: *mut Bodies) -> Self {
        Self::new(solver, bodies, 0.005, 0.0005)
    }

    pub fn target_candidate_fraction(&self) -> f32 {
        self.target_candidate_fraction
    }

    pub fn set_target_candidate_fraction(&mut self, value: f32) {
        debug_assert!(value >= 0.0 && value <= 1.0, "Fraction must be from 0 to 1.");
        self.target_candidate_fraction = value;
    }

    pub fn maximum_compression_fraction(&self) -> f32 {
        self.maximum_compression_fraction
    }

    pub fn set_maximum_compression_fraction(&mut self, value: f32) {
        debug_assert!(value >= 0.0 && value <= 1.0, "Fraction must be from 0 to 1.");
        self.maximum_compression_fraction = value;
    }

    fn solver(&self) -> &Solver {
        unsafe { &*self.solver }
    }

    fn solver_mut(&self) -> &mut Solver {
        unsafe { &mut *self.solver }
    }

    fn bodies(&self) -> &Bodies {
        unsafe { &*self.bodies }
    }

    unsafe fn try_to_find_better_batch_for_constraint(
        &self,
        pool: &mut BufferPool,
        compressions: &mut QuickList<Compression>,
        type_batch: &TypeBatch,
        constraint_index: i32,
    ) {
        let solver = self.solver();
        let bodies = self.bodies();
        let type_processor = solver.type_processors[type_batch.type_id as usize].as_ref().unwrap();
        let bodies_per_constraint = type_processor.bodies_per_constraint;

        // Collect dynamic body handles for this constraint (stack alloc — matches C# stackalloc).
        let mut handle_buf = [0i32; 8]; // max bodies per constraint
        debug_assert!(bodies_per_constraint <= 8, "Bodies per constraint exceeds stack buffer size");
        let mut collector = ActiveConstraintDynamicBodyHandleCollector::new(bodies, handle_buf.as_mut_ptr());
        solver.enumerate_connected_raw_body_references_from_type_batch(
            type_batch,
            constraint_index,
            &mut collector,
        );
        let dynamic_body_handles = &handle_buf[..collector.count as usize];

        for batch_index in (0..self.next_batch_index).rev() {
            if solver.batch_referenced_handles.get(batch_index).can_fit(dynamic_body_handles) {
                compressions.add(
                    Compression {
                        constraint_handle: *type_batch.index_to_handle.get(constraint_index),
                        target_batch: batch_index,
                    },
                    pool,
                );
                return;
            }
        }
    }

    unsafe fn do_job(&self, region: &AnalysisRegion, worker_index: i32, pool: &mut BufferPool) {
        let compressions = &mut *(self.worker_compressions.as_ptr().add(worker_index as usize) as *mut QuickList<Compression>);
        let solver = self.solver();
        let batch = solver.active_set().batches.get(self.next_batch_index);
        let type_batch = batch.type_batches.get(region.type_batch_index);

        if self.next_batch_index == solver.fallback_batch_threshold() {
            for i in region.start_index_in_type_batch..region.end_index_in_type_batch {
                // Fallback batch might have empty slots.
                if type_batch.index_to_handle.get(i).0 >= 0 {
                    self.try_to_find_better_batch_for_constraint(pool, compressions, type_batch, i);
                }
            }
        } else {
            for i in region.start_index_in_type_batch..region.end_index_in_type_batch {
                self.try_to_find_better_batch_for_constraint(pool, compressions, type_batch, i);
            }
        }
    }

    fn analysis_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
        unsafe {
            let bc = &*(dispatcher.unmanaged_context() as *const BatchCompressor);
            let pool = &mut *dispatcher.worker_pool_ptr(worker_index);
            loop {
                let job_index = AtomicI32::from_ptr(bc.analysis_job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
                if job_index >= bc.analysis_jobs.count {
                    break;
                }
                bc.do_job(bc.analysis_jobs.get(job_index), worker_index, pool);
            }
        }
    }

    unsafe fn apply_compression(&self, source_batch_index: i32, compression: &Compression) {
        let solver = self.solver_mut();
        let location = *solver.handle_to_constraint.get(compression.constraint_handle.0);
        let type_processor = solver.type_processors[location.type_id as usize].as_ref().unwrap();

        if source_batch_index == solver.fallback_batch_threshold() {
            // Fallback batch: check if target batch can still hold the constraint
            // (another compression may have interfered).
            let mut handle_buf = [0i32; 8]; // stack alloc — matches C# stackalloc
            let bodies = self.bodies();
            let mut collector = ActiveConstraintDynamicBodyHandleCollector::new(bodies, handle_buf.as_mut_ptr());
            solver.enumerate_connected_raw_body_references(compression.constraint_handle, &mut collector);
            let dynamic_handles = &handle_buf[..collector.count as usize];
            if !solver.batch_referenced_handles.get(compression.target_batch).can_fit(dynamic_handles) {
                return;
            }
        }

        let solver_ptr = self.solver;
        let bodies_ptr = self.bodies;
        let batch = (*solver_ptr).sets.get_mut(0).batches.get_mut(location.batch_index);
        let type_batch_index = *batch.type_index_to_type_batch_index.get(location.type_id);
        let type_batch = batch.type_batches.get_mut(type_batch_index);
        let type_proc = (&(*solver_ptr).type_processors)[location.type_id as usize].as_ref().unwrap();
        type_proc.inner().transfer_constraint_auto_collect(
            type_batch,
            source_batch_index,
            location.index_in_type_batch,
            solver_ptr,
            bodies_ptr,
            compression.target_batch,
        );
    }

    /// Incrementally finds and applies a set of compressions.
    /// Constraints in higher index batches try to move to lower index batches whenever possible.
    pub unsafe fn compress(
        &mut self,
        pool: &mut BufferPool,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
        deterministic: bool,
    ) {
        let solver = &*self.solver;
        let worker_count = thread_dispatcher.map_or(1, |td| td.thread_count());
        let constraint_count = solver.active_set().constraint_count();
        if constraint_count == 0 {
            return;
        }

        let maximum_compression_count = (1.0f32).max((self.maximum_compression_fraction * constraint_count as f32).round()) as i32;
        let target_candidate_count = (1.0f32).max((self.target_candidate_fraction * constraint_count as f32).round()) as i32;

        // Allocate per-worker compression lists.
        self.worker_compressions = pool.take::<QuickList<Compression>>(worker_count);
        for i in 0..worker_count {
            let worker_pool = if let Some(td) = thread_dispatcher {
                &mut *td.worker_pool_ptr(i)
            } else {
                &mut *pool
            };
            *self.worker_compressions.get_mut(i) = QuickList::with_capacity(maximum_compression_count.max(8), worker_pool);
        }

        let batch_count = solver.active_set().batches.count;
        debug_assert!(batch_count > 0);

        // Validate indices.
        if self.next_batch_index >= batch_count {
            self.next_batch_index = 0;
            self.next_type_batch_index = 0;
        }
        while self.next_type_batch_index >= solver.active_set().batches.get(self.next_batch_index).type_batches.count {
            self.next_batch_index += 1;
            if self.next_batch_index >= batch_count {
                self.next_batch_index = 0;
            }
            self.next_type_batch_index = 0;
        }

        let batch = solver.active_set().batches.get(self.next_batch_index);
        self.analysis_jobs = QuickList::with_capacity(512.min(batch.type_batches.count * 4 + 1), pool);

        const TARGET_CONSTRAINTS_PER_JOB: i32 = 64;
        let mut total_constraints_scheduled = 0;

        let mut tbi = self.next_type_batch_index;
        while tbi < batch.type_batches.count && total_constraints_scheduled < target_candidate_count {
            let type_batch = batch.type_batches.get(tbi);
            let job_count = 1 + type_batch.constraint_count / TARGET_CONSTRAINTS_PER_JOB;
            let base_per_job = type_batch.constraint_count / job_count;
            let remainder = type_batch.constraint_count - base_per_job * job_count;

            let mut previous_end = 0;
            self.analysis_jobs.ensure_capacity(self.analysis_jobs.count + job_count, pool);
            for j in 0..job_count {
                let constraints_in_job = if j < remainder { base_per_job + 1 } else { base_per_job };
                let job = self.analysis_jobs.allocate_unsafely();
                job.type_batch_index = tbi;
                job.start_index_in_type_batch = previous_end;
                previous_end += constraints_in_job;
                job.end_index_in_type_batch = previous_end;
            }

            total_constraints_scheduled += type_batch.constraint_count;
            tbi += 1;
        }
        self.next_type_batch_index = tbi;

        // Run analysis.
        if let Some(td) = thread_dispatcher {
            *self.analysis_job_index.get() = -1;
            let self_ptr = self as *mut BatchCompressor as *mut ();
            td.dispatch_workers(
                Self::analysis_worker,
                self.analysis_jobs.count,
                self_ptr,
                None,
            );
        } else {
            for i in 0..self.analysis_jobs.count {
                self.do_job(self.analysis_jobs.get(i), 0, pool);
            }
        }

        self.analysis_jobs.dispose(pool);

        // Apply compressions.
        let mut compressions_applied = 0;
        if deterministic {
            // In deterministic mode, gather and sort all compressions by constraint handle.
            let mut total_compression_count = 0;
            for i in 0..worker_count {
                total_compression_count += self.worker_compressions.get(i).count;
            }
            if total_compression_count > 0 {
                let mut sorted: QuickList<(u16, u16, ConstraintHandle)> = QuickList::with_capacity(total_compression_count, pool);
                for i in 0..worker_count {
                    let wc = self.worker_compressions.get(i);
                    for j in 0..wc.count {
                        let c = wc.get(j);
                        *sorted.allocate_unsafely() = (i as u16, j as u16, c.constraint_handle);
                    }
                }
                // Sort by constraint handle for determinism.
                let slice = std::slice::from_raw_parts_mut(sorted.span.get_mut(0) as *mut (u16, u16, ConstraintHandle), sorted.count as usize);
                slice.sort_unstable_by_key(|&(_, _, h)| h.0);

                let limit = sorted.count.min(maximum_compression_count);
                for i in 0..limit {
                    let (wi, ci, _) = *sorted.get(i);
                    let c = *self.worker_compressions.get(wi as i32).get(ci as i32);
                    self.apply_compression(self.next_batch_index, &c);
                }
                sorted.dispose(pool);
            }
        } else {
            // In nondeterministic mode, walk worker results in reverse order.
            for i in (0..worker_count).rev() {
                let wc = self.worker_compressions.get(i);
                for j in (0..wc.count).rev() {
                    if compressions_applied >= maximum_compression_count {
                        break;
                    }
                    let c = *wc.get(j);
                    self.apply_compression(self.next_batch_index, &c);
                    compressions_applied += 1;
                }
                if compressions_applied >= maximum_compression_count {
                    break;
                }
            }
        }

        // Clean up per-worker compression lists.
        if thread_dispatcher.is_none() {
            self.worker_compressions.get_mut(0).dispose(pool);
        } else {
            let td = thread_dispatcher.unwrap();
            for i in 0..worker_count {
                self.worker_compressions.get_mut(i).dispose(&mut *td.worker_pool_ptr(i));
            }
        }
        pool.return_buffer(&mut self.worker_compressions);
    }
}

unsafe impl Send for BatchCompressor {}
unsafe impl Sync for BatchCompressor {}
