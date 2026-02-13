// Translated from BepuPhysics/CollisionDetection/CollidableOverlapFinder.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::collision_detection::narrow_phase::NarrowPhaseGeneric;
use crate::physics::collision_detection::narrow_phase_callbacks::INarrowPhaseCallbacks;
use crate::physics::trees::tree_intertree_queries_mt::MultithreadedIntertreeTest;
use crate::physics::trees::tree_self_queries_mt::MultithreadedSelfTest;
use crate::physics::trees::IOverlapHandler;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::thread_dispatcher::IThreadDispatcher;
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicI32, Ordering};

/// Trait for types that can dispatch broad phase overlap testing.
/// Matches C#'s abstract `CollidableOverlapFinder` base class.

struct SelfOverlapHandler<TCallbacks: INarrowPhaseCallbacks> {
    narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    leaves: Buffer<CollidableReference>,
    worker_index: i32,
}

impl<TCallbacks: INarrowPhaseCallbacks> IOverlapHandler for SelfOverlapHandler<TCallbacks> {
    #[inline(always)]
    fn handle(&mut self, index_a: i32, index_b: i32) {
        unsafe {
            let a = *self.leaves.get(index_a);
            let b = *self.leaves.get(index_b);
            (*self.narrow_phase).handle_overlap(self.worker_index, a, b);
        }
    }
}

struct IntertreeOverlapHandler<TCallbacks: INarrowPhaseCallbacks> {
    narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
    leaves_a: Buffer<CollidableReference>,
    leaves_b: Buffer<CollidableReference>,
    worker_index: i32,
}

impl<TCallbacks: INarrowPhaseCallbacks> IOverlapHandler for IntertreeOverlapHandler<TCallbacks> {
    #[inline(always)]
    fn handle(&mut self, index_a: i32, index_b: i32) {
        unsafe {
            let a = *self.leaves_a.get(index_a);
            let b = *self.leaves_b.get(index_b);
            (*self.narrow_phase).handle_overlap(self.worker_index, a, b);
        }
    }
}

// ============================================================================
// Monomorphized dispatch function — captures TCallbacks at construction time.
// ============================================================================

/// Type-erased dispatch function signature.
type DispatchFn = unsafe fn(
    narrow_phase: *mut u8,
    broad_phase: *mut BroadPhase,
    dt: f32,
    thread_dispatcher: Option<&dyn IThreadDispatcher>,
);

/// Context for the MT overlap worker function.
struct OverlapWorkerContext {
    self_test: *mut u8,      // *mut MultithreadedSelfTest<SelfOverlapHandler>
    intertree_test: *mut u8, // *mut MultithreadedIntertreeTest<IntertreeOverlapHandler>
    narrow_phase: *mut u8,
    self_job_count: i32,
    total_job_count: i32,
    next_job_index: UnsafeCell<i32>,
    // Type-erased function pointers for execute_job calls
    execute_self_job: unsafe fn(*mut u8, i32, i32),
    execute_intertree_job: unsafe fn(*mut u8, i32, i32),
    flush_worker: unsafe fn(*mut u8, i32),
}

unsafe fn execute_self_job_impl<TCallbacks: INarrowPhaseCallbacks>(
    ctx: *mut u8,
    job_index: i32,
    worker_index: i32,
) {
    let test = &mut *(ctx as *mut MultithreadedSelfTest<SelfOverlapHandler<TCallbacks>>);
    test.execute_job(job_index, worker_index);
}

unsafe fn execute_intertree_job_impl<TCallbacks: INarrowPhaseCallbacks>(
    ctx: *mut u8,
    job_index: i32,
    worker_index: i32,
) {
    let test = &mut *(ctx as *mut MultithreadedIntertreeTest<IntertreeOverlapHandler<TCallbacks>>);
    test.execute_job(job_index, worker_index);
}

unsafe fn flush_worker_impl<TCallbacks: INarrowPhaseCallbacks>(np: *mut u8, worker_index: i32) {
    let np = &mut *(np as *mut NarrowPhaseGeneric<TCallbacks>);
    np.overlap_workers[worker_index as usize].batcher.flush();
}

/// Worker function dispatched by the thread dispatcher.
fn overlap_worker(worker_index: i32, dispatcher: &dyn IThreadDispatcher) {
    unsafe {
        let ctx = &*(dispatcher.unmanaged_context() as *const OverlapWorkerContext);
        loop {
            let job_index =
                AtomicI32::from_ptr(ctx.next_job_index.get()).fetch_add(1, Ordering::AcqRel) + 1;
            if job_index < ctx.self_job_count {
                (ctx.execute_self_job)(ctx.self_test, job_index, worker_index);
            } else if job_index < ctx.total_job_count {
                (ctx.execute_intertree_job)(
                    ctx.intertree_test,
                    job_index - ctx.self_job_count,
                    worker_index,
                );
            } else {
                break;
            }
        }
        (ctx.flush_worker)(ctx.narrow_phase, worker_index);
    }
}

unsafe fn dispatch_overlaps_impl<TCallbacks: INarrowPhaseCallbacks>(
    narrow_phase_ptr: *mut u8,
    broad_phase_ptr: *mut BroadPhase,
    dt: f32,
    thread_dispatcher: Option<&dyn IThreadDispatcher>,
) {
    let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
    let bp = &mut *broad_phase_ptr;

    if let Some(td) = thread_dispatcher {
        if td.thread_count() > 1 {
            // Multi-threaded path
            np.prepare(dt, Some(td));
            let thread_count = td.thread_count();

            // Create per-worker overlap handlers
            let mut self_handlers = Vec::with_capacity(thread_count as usize);
            let mut intertree_handlers = Vec::with_capacity(thread_count as usize);
            for i in 0..thread_count {
                self_handlers.push(SelfOverlapHandler::<TCallbacks> {
                    narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                    leaves: bp.active_leaves,
                    worker_index: i,
                });
                intertree_handlers.push(IntertreeOverlapHandler::<TCallbacks> {
                    narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                    leaves_a: bp.active_leaves,
                    leaves_b: bp.static_leaves,
                    worker_index: i,
                });
            }

            // Prepare MT overlap testing jobs
            let pool = np.base.pool;
            let mut self_test = MultithreadedSelfTest::new(pool);
            self_test.prepare_jobs(&bp.active_tree, self_handlers, thread_count);
            let mut intertree_test = MultithreadedIntertreeTest::new(pool);
            intertree_test.prepare_jobs(
                &bp.active_tree,
                &bp.static_tree,
                intertree_handlers,
                thread_count,
            );

            let self_job_count = self_test.job_count();
            let intertree_job_count = intertree_test.job_count();
            let total_job_count = self_job_count + intertree_job_count;

            let mut ctx = OverlapWorkerContext {
                self_test: &mut self_test
                    as *mut MultithreadedSelfTest<SelfOverlapHandler<TCallbacks>>
                    as *mut u8,
                intertree_test: &mut intertree_test
                    as *mut MultithreadedIntertreeTest<IntertreeOverlapHandler<TCallbacks>>
                    as *mut u8,
                narrow_phase: narrow_phase_ptr,
                self_job_count,
                total_job_count,
                next_job_index: UnsafeCell::new(-1),
                execute_self_job: execute_self_job_impl::<TCallbacks>,
                execute_intertree_job: execute_intertree_job_impl::<TCallbacks>,
                flush_worker: flush_worker_impl::<TCallbacks>,
            };

            td.dispatch_workers(
                overlap_worker,
                total_job_count,
                &mut ctx as *mut OverlapWorkerContext as *mut (),
                None,
            );

            // If total job count was zero, flush worker 0 (tree was tiny)
            if total_job_count == 0 {
                let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
                np.overlap_workers[0].batcher.flush();
            }
            // Flush any workers that were allocated but not used due to lack of jobs
            for i in (1.max(total_job_count))..thread_count {
                let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
                np.overlap_workers[i as usize].batcher.flush();
            }

            self_test.complete_self_test();
            intertree_test.complete_test();
            return;
        }
    }

    // Single-threaded path
    np.prepare(dt, None);

    {
        let mut self_handler = SelfOverlapHandler::<TCallbacks> {
            narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
            leaves: bp.active_leaves,
            worker_index: 0,
        };
        bp.active_tree.get_self_overlaps(&mut self_handler);
    }

    {
        let mut intertree_handler = IntertreeOverlapHandler::<TCallbacks> {
            narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
            leaves_a: bp.active_leaves,
            leaves_b: bp.static_leaves,
            worker_index: 0,
        };
        bp.active_tree
            .get_overlaps_with_tree(&bp.static_tree, &mut intertree_handler);
    }

    let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
    np.overlap_workers[0].batcher.flush();
}

// ============================================================================
// CollidableOverlapFinder — type-erased dispatch between broad phase trees.
// ============================================================================

/// Dispatches overlap testing between broad phase trees.
pub struct CollidableOverlapFinder {
    narrow_phase: *mut u8,
    broad_phase: *mut BroadPhase,
    dispatch_fn: DispatchFn,
}

impl CollidableOverlapFinder {
    /// Creates a new overlap finder. The generic parameter is captured at construction
    /// and erased — subsequent calls to `dispatch_overlaps` do not require it.
    pub fn new<TCallbacks: INarrowPhaseCallbacks>(
        narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
        broad_phase: *mut BroadPhase,
    ) -> Self {
        Self {
            narrow_phase: narrow_phase as *mut u8,
            broad_phase,
            dispatch_fn: dispatch_overlaps_impl::<TCallbacks>,
        }
    }

    /// Dispatches overlap finding between active and static trees,
    /// forwarding all overlaps to the narrow phase's handle_overlap.
    pub fn dispatch_overlaps(
        &mut self,
        dt: f32,
        thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        unsafe {
            (self.dispatch_fn)(self.narrow_phase, self.broad_phase, dt, thread_dispatcher);
        }
    }
}
