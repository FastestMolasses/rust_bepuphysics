// Translated from BepuPhysics/CollisionDetection/CollidableOverlapFinder.cs

use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::physics::collision_detection::narrow_phase::NarrowPhaseGeneric;
use crate::physics::collision_detection::narrow_phase_callbacks::INarrowPhaseCallbacks;
use crate::physics::trees::IOverlapHandler;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::thread_dispatcher::IThreadDispatcher;

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
);

unsafe fn dispatch_overlaps_impl<TCallbacks: INarrowPhaseCallbacks>(
    narrow_phase_ptr: *mut u8,
    broad_phase_ptr: *mut BroadPhase,
    dt: f32,
) {
    let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
    let bp = &mut *broad_phase_ptr;

    // TODO: Multi-threaded overlap dispatch using MultithreadedSelfTest/MultithreadedIntertreeTest.
    // These C# types are in the missing multi-threaded tree files (tree_self_queries_mt.rs,
    // tree_intertree_queries_mt.rs). When available, the thread_dispatcher would be passed
    // to prepare() and used to create per-worker handlers + dispatch.

    // Single-threaded path: prepare with no dispatcher (creates 1 overlap worker).
    np.prepare(dt, None);

    // Single-threaded path:
    // 1) Self-test the active tree for body-body overlaps
    {
        let mut self_handler = SelfOverlapHandler::<TCallbacks> {
            narrow_phase: narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>,
            leaves: bp.active_leaves,
            worker_index: 0,
        };
        bp.active_tree.get_self_overlaps(&mut self_handler);
    }

    // 2) Inter-tree test active vs static for body-static overlaps
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

    // 3) Flush the collision batcher for worker 0
    let np = &mut *(narrow_phase_ptr as *mut NarrowPhaseGeneric<TCallbacks>);
    np.overlap_workers[0].batcher.flush();
}

// ============================================================================
// CollidableOverlapFinder — type-erased dispatch between broad phase trees.
// Uses a monomorphized function pointer captured at construction time to
// perform overlap testing without requiring generic type parameters.
// ============================================================================

/// Dispatches overlap testing between broad phase trees.
/// Stores type-erased pointers to NarrowPhaseGeneric and BroadPhase,
/// with a monomorphized dispatch function that knows the concrete TCallbacks type.
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
        _thread_dispatcher: Option<&dyn IThreadDispatcher>,
    ) {
        // TODO: When multi-threaded overlap dispatch is implemented (requires
        // MultithreadedSelfTest/MultithreadedIntertreeTest tree types), pass
        // thread_dispatcher through to prepare and create per-worker handlers.
        unsafe {
            (self.dispatch_fn)(self.narrow_phase, self.broad_phase, dt);
        }
    }
}
