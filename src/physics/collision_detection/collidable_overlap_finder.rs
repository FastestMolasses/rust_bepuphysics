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

/// Per-`TCallbacks` MT test contexts and handler buffers, built once in `new` and reused every frame.
struct OverlapFinderState<TCallbacks: INarrowPhaseCallbacks> {
    self_test: MultithreadedSelfTest<SelfOverlapHandler<TCallbacks>>,
    intertree_test: MultithreadedIntertreeTest<IntertreeOverlapHandler<TCallbacks>>,
    self_handlers: Vec<SelfOverlapHandler<TCallbacks>>,
    intertree_handlers: Vec<IntertreeOverlapHandler<TCallbacks>>,
}

// ============================================================================
// Monomorphized dispatch function — captures TCallbacks at construction time.
// ============================================================================

/// Type-erased dispatch function signature.
type DispatchFn = unsafe fn(
    narrow_phase: *mut u8,
    broad_phase: *mut BroadPhase,
    state: *mut u8,
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
    // Workers only touch their own overlap_handlers slot, so a shared reference is sound here.
    let test = &*(ctx as *const MultithreadedSelfTest<SelfOverlapHandler<TCallbacks>>);
    test.execute_job(job_index, worker_index);
}

unsafe fn execute_intertree_job_impl<TCallbacks: INarrowPhaseCallbacks>(
    ctx: *mut u8,
    job_index: i32,
    worker_index: i32,
) {
    // Workers only touch their own overlap_handlers slot, so a shared reference is sound here.
    let test = &*(ctx as *const MultithreadedIntertreeTest<IntertreeOverlapHandler<TCallbacks>>);
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
    state_ptr: *mut u8,
    dt: f32,
    thread_dispatcher: Option<&dyn IThreadDispatcher>,
) {
    let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
    let bp = &mut *broad_phase_ptr;
    let state = &mut *(state_ptr as *mut OverlapFinderState<TCallbacks>);

    if let Some(td) = thread_dispatcher {
        if td.thread_count() > 1 {
            // Multi-threaded path
            np.prepare(dt, Some(td));
            let thread_count = td.thread_count();

            if state.intertree_handlers.len() < thread_count as usize {
                // This initialization/resize should occur extremely rarely.
                while state.self_handlers.len() < thread_count as usize {
                    let i = state.self_handlers.len() as i32;
                    state.self_handlers.push(SelfOverlapHandler::<TCallbacks> {
                        narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                        leaves: bp.active_leaves,
                        worker_index: i,
                    });
                }
                while state.intertree_handlers.len() < thread_count as usize {
                    let i = state.intertree_handlers.len() as i32;
                    state
                        .intertree_handlers
                        .push(IntertreeOverlapHandler::<TCallbacks> {
                            narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                            leaves_a: bp.active_leaves,
                            leaves_b: bp.static_leaves,
                            worker_index: i,
                        });
                }
            }
            // Note that the overlap handlers are reinitialized regardless of whether the thread
            // count changed. This is just a simple way to guarantee that the most recent broad
            // phase buffers are used- caching the buffers outside of this execution would be
            // invalid because they may get resized, invalidating the pointers.
            for (i, handler) in state.self_handlers.iter_mut().enumerate() {
                *handler = SelfOverlapHandler::<TCallbacks> {
                    narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                    leaves: bp.active_leaves,
                    worker_index: i as i32,
                };
            }
            for (i, handler) in state.intertree_handlers.iter_mut().enumerate() {
                *handler = IntertreeOverlapHandler::<TCallbacks> {
                    narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
                    leaves_a: bp.active_leaves,
                    leaves_b: bp.static_leaves,
                    worker_index: i as i32,
                };
            }
            debug_assert!(state.intertree_handlers.len() >= thread_count as usize);

            // Handler buffers are moved into the test contexts here and back out below; they only ever grow.
            state.self_test.prepare_jobs(
                &bp.active_tree,
                std::mem::take(&mut state.self_handlers),
                thread_count,
            );
            state.intertree_test.prepare_jobs(
                &bp.active_tree,
                &bp.static_tree,
                std::mem::take(&mut state.intertree_handlers),
                thread_count,
            );

            let self_job_count = state.self_test.job_count();
            let intertree_job_count = state.intertree_test.job_count();
            let total_job_count = self_job_count + intertree_job_count;

            let mut ctx = OverlapWorkerContext {
                self_test: &state.self_test
                    as *const MultithreadedSelfTest<SelfOverlapHandler<TCallbacks>>
                    as *mut u8,
                intertree_test: &state.intertree_test
                    as *const MultithreadedIntertreeTest<IntertreeOverlapHandler<TCallbacks>>
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

            state.self_test.complete_self_test();
            state.intertree_test.complete_test();

            // Move the handler buffers back into persistent storage for reuse next frame.
            state.self_handlers = std::mem::take(&mut state.self_test.overlap_handlers);
            state.intertree_handlers = std::mem::take(&mut state.intertree_test.overlap_handlers);
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
    /// Type-erased `*mut OverlapFinderState<TCallbacks>`; allocated once in `new`, never freed.
    state: *mut u8,
    dispatch_fn: DispatchFn,
}

impl CollidableOverlapFinder {
    /// Creates a new overlap finder. The generic parameter is captured at construction
    /// and erased — subsequent calls to `dispatch_overlaps` do not require it.
    pub fn new<TCallbacks: INarrowPhaseCallbacks>(
        narrow_phase: *mut NarrowPhaseGeneric<TCallbacks>,
        broad_phase: *mut BroadPhase,
    ) -> Self {
        let pool = unsafe { (*narrow_phase).base.pool };
        let state = Box::new(OverlapFinderState::<TCallbacks> {
            self_test: MultithreadedSelfTest::new(pool),
            intertree_test: MultithreadedIntertreeTest::new(pool),
            self_handlers: Vec::new(),
            intertree_handlers: Vec::new(),
        });
        Self {
            narrow_phase: narrow_phase as *mut u8,
            broad_phase,
            state: Box::into_raw(state) as *mut u8,
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
            (self.dispatch_fn)(
                self.narrow_phase,
                self.broad_phase,
                self.state,
                dt,
                thread_dispatcher,
            );
        }
    }
}
