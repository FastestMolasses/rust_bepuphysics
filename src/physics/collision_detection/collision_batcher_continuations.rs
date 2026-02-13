// Translated from BepuPhysics/CollisionDetection/CollisionBatcherContinuations.cs

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;

/// Describes the flow control to apply to a convex-convex pair report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CollisionContinuationType {
    /// Requires no further processing before being reported to user callbacks.
    Direct = 0,
    /// Part of a set of a higher (potentially multi-manifold) pair, potentially requiring contact reduction.
    NonconvexReduction = 1,
    /// Part of a set of mesh-convex collisions, potentially requiring mesh boundary smoothing.
    MeshReduction = 2,
    /// Part of mesh-convex collisions spawned by a mesh-compound pair.
    CompoundMeshReduction = 3,
}

/// Describes the continuation of a collision pair after initial processing.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct PairContinuation {
    pub pair_id: i32,
    pub child_a: i32,
    pub child_b: i32,
    pub packed: u32,
}

impl PairContinuation {
    pub const CHILD_INDEX_BITS: u32 = 20;
    pub const CONTINUATION_INDEX_BITS: u32 = 10;
    pub const CONTINUATION_TYPE_BITS: u32 = 2;

    pub const EXCLUSIVE_MAXIMUM_CHILD_INDEX: u32 = 1 << Self::CHILD_INDEX_BITS;
    pub const EXCLUSIVE_MAXIMUM_CONTINUATION_INDEX: u32 = 1 << Self::CONTINUATION_INDEX_BITS;
    pub const EXCLUSIVE_MAXIMUM_CONTINUATION_TYPE: u32 = 1 << Self::CONTINUATION_TYPE_BITS;

    const TYPE_SHIFT: u32 = Self::CHILD_INDEX_BITS + Self::CONTINUATION_INDEX_BITS;
    const INDEX_SHIFT: u32 = Self::CHILD_INDEX_BITS;
    const INDEX_MASK: u32 = (1 << Self::CONTINUATION_INDEX_BITS) - 1;
    const CHILD_INDEX_MASK: u32 = (1 << Self::CHILD_INDEX_BITS) - 1;

    #[inline(always)]
    pub fn new(
        pair_id: i32,
        child_a: i32,
        child_b: i32,
        continuation_type: CollisionContinuationType,
        continuation_index: i32,
        continuation_child_index: i32,
    ) -> Self {
        debug_assert!((continuation_child_index as u32) < Self::EXCLUSIVE_MAXIMUM_CHILD_INDEX);
        debug_assert!((continuation_index as u32) < Self::EXCLUSIVE_MAXIMUM_CONTINUATION_INDEX);
        debug_assert!((continuation_type as u32) < Self::EXCLUSIVE_MAXIMUM_CONTINUATION_TYPE);
        Self {
            pair_id,
            child_a,
            child_b,
            packed: ((continuation_type as u32) << Self::TYPE_SHIFT)
                | ((continuation_index as u32) << Self::INDEX_SHIFT)
                | (continuation_child_index as u32),
        }
    }

    #[inline(always)]
    pub fn direct(pair_id: i32) -> Self {
        Self {
            pair_id,
            child_a: 0,
            child_b: 0,
            packed: 0,
        }
    }

    #[inline(always)]
    pub fn continuation_type(&self) -> CollisionContinuationType {
        unsafe {
            std::mem::transmute::<u8, CollisionContinuationType>(
                (self.packed >> Self::TYPE_SHIFT) as u8,
            )
        }
    }

    #[inline(always)]
    pub fn index(&self) -> i32 {
        ((self.packed >> Self::INDEX_SHIFT) & Self::INDEX_MASK) as i32
    }

    #[inline(always)]
    pub fn child_index(&self) -> i32 {
        (self.packed & Self::CHILD_INDEX_MASK) as i32
    }
}

impl Default for PairContinuation {
    fn default() -> Self {
        Self {
            pair_id: 0,
            child_a: 0,
            child_b: 0,
            packed: 0,
        }
    }
}

/// Defines a type which includes information necessary to apply some form of post processing to a collision test result.
pub trait ICollisionTestContinuation: Sized {
    /// Creates a collision test continuation with the given number of slots for subpairs.
    fn create(&mut self, slots: i32, pool: &mut BufferPool);

    // NOTE: OnChildCompleted, OnUntestedChildCompleted, and TryFlush require generic
    // CollisionBatcher<TCallbacks> which can't be expressed as trait methods without GATs.
    // These will be implemented via direct method calls on concrete types.
}

/// Manages a pool of collision test continuations.
pub struct BatcherContinuations<T: ICollisionTestContinuation> {
    pub continuations: Buffer<T>,
    pub id_pool: IdPool,
}

const INITIAL_CAPACITY: i32 = 64;

impl<T: ICollisionTestContinuation + Default + Copy> BatcherContinuations<T> {
    /// Creates a new empty BatcherContinuations. Lazy-initializes on first use.
    pub fn new() -> Self {
        Self {
            continuations: Buffer::default(),
            id_pool: IdPool::default(),
        }
    }

    pub fn create_continuation(
        &mut self,
        slots_in_continuation: i32,
        pool: &mut BufferPool,
    ) -> (i32, &mut T) {
        if !self.continuations.allocated() {
            self.continuations = pool.take_at_least(INITIAL_CAPACITY);
            self.id_pool = IdPool::new(INITIAL_CAPACITY, pool);
        }
        let index = self.id_pool.take();
        if index >= self.continuations.len() {
            pool.resize_to_at_least(&mut self.continuations, index + 1, index);
        }
        unsafe {
            let continuation = &mut *self.continuations.get_mut(index);
            continuation.create(slots_in_continuation, pool);
            (index, continuation)
        }
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        if self.continuations.allocated() {
            pool.return_buffer(&mut self.continuations);
            self.id_pool.dispose(pool);
        }
    }
}
