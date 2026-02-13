// Translated from BepuPhysics/BoundingBoxBatcher.cs (namespace BepuPhysics)

use crate::physics::body_properties::{BodyVelocity, MotionState, RigidPose};
use crate::physics::bodies::Bodies;
use crate::physics::bounding_box_helpers::BoundingBoxHelpers;
use crate::physics::collidables::collidable::Collidable;
use crate::physics::collidables::shape::{IConvexShape, IDisposableShape, INonConvexBounds, ICompoundShape};
use crate::physics::collidables::shapes::{ConvexShapeBatch, HomogeneousCompoundShapeBatch, CompoundShapeBatch, ShapeBatch, Shapes};
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::collision_detection::broad_phase::BroadPhase;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::vector::Vector;
#[allow(unused_imports)]
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;
#[allow(unused_imports)]
use std::simd::prelude::*;

const VECTOR_WIDTH: usize = crate::utilities::vector::VECTOR_WIDTH;

/// Continuation data for a bounding box computation.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BoundsContinuation {
    packed: u32,
}

impl BoundsContinuation {
    /// Gets the body index from the continuation.
    #[inline(always)]
    pub fn body_index(&self) -> i32 {
        (self.packed & 0x7FFFFFFF) as i32
    }

    /// Returns true if this continuation is for a compound child shape.
    #[inline(always)]
    pub fn compound_child(&self) -> bool {
        (self.packed & (1u32 << 31)) > 0
    }

    /// Creates a continuation for a top-level body.
    pub fn create_continuation(body_index: i32) -> Self {
        debug_assert!(body_index >= 0);
        Self {
            packed: body_index as u32,
        }
    }

    /// Creates a continuation for a compound child shape.
    pub fn create_compound_child_continuation(compound_body_index: i32) -> Self {
        debug_assert!(compound_body_index >= 0);
        Self {
            packed: (1u32 << 31) | (compound_body_index as u32),
        }
    }
}

impl Default for BoundsContinuation {
    fn default() -> Self {
        Self { packed: 0 }
    }
}

/// A single bounding box computation instance.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BoundingBoxInstance {
    pub shape_index: i32,
    pub continuation: BoundsContinuation,
}

/// A batch of bounding box computations for a single shape type.
pub struct BoundingBoxBatch {
    pub shape_indices: Buffer<i32>,
    pub continuations: Buffer<BoundsContinuation>,
    pub motion_states: Buffer<MotionState>,
    pub count: i32,
}

impl BoundingBoxBatch {
    pub fn allocated(&self) -> bool {
        self.shape_indices.allocated()
    }

    pub fn new(pool: &mut BufferPool, initial_capacity: i32) -> Self {
        let shape_indices = pool.take(initial_capacity);
        let continuations = pool.take(initial_capacity);
        let motion_states = pool.take(initial_capacity);
        Self {
            shape_indices,
            continuations,
            motion_states,
            count: 0,
        }
    }

    pub(crate) fn add(
        &mut self,
        shape_index: i32,
        pose: RigidPose,
        velocity: BodyVelocity,
        continuation: BoundsContinuation,
    ) {
        *self.shape_indices.get_mut(self.count) = shape_index;
        *self.continuations.get_mut(self.count) = continuation;
        let motion_state = self.motion_states.get_mut(self.count);
        motion_state.pose = pose;
        motion_state.velocity = velocity;
        self.count += 1;
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        if self.allocated() {
            pool.return_buffer(&mut self.shape_indices);
            pool.return_buffer(&mut self.continuations);
            pool.return_buffer(&mut self.motion_states);
            self.count = 0;
        }
    }
}

impl Default for BoundingBoxBatch {
    fn default() -> Self {
        Self {
            shape_indices: Buffer::default(),
            continuations: Buffer::default(),
            motion_states: Buffer::default(),
            count: 0,
        }
    }
}

/// Batches bounding box computations across shape types and flushes them to the broad phase.
pub struct BoundingBoxBatcher {
    pub(crate) pool: *mut BufferPool,
    pub(crate) shapes: *mut Shapes,
    pub(crate) bodies: *mut Bodies,
    pub(crate) broad_phase: *mut BroadPhase,
    pub(crate) dt: f32,

    minimum_batch_index: i32,
    maximum_batch_index: i32,
    batches: Buffer<BoundingBoxBatch>,
}

pub const COLLIDABLES_PER_FLUSH: i32 = 16;

impl BoundingBoxBatcher {
    pub fn new(
        bodies: *mut Bodies,
        shapes: *mut Shapes,
        broad_phase: *mut BroadPhase,
        pool: *mut BufferPool,
        dt: f32,
    ) -> Self {
        let pool_ref = unsafe { &mut *pool };
        let shapes_ref = unsafe { &*shapes };
        let span = shapes_ref.registered_type_span() as i32;
        let mut batches: Buffer<BoundingBoxBatch> = pool_ref.take_at_least(span);
        batches.clear(0, span);
        Self {
            pool,
            shapes,
            bodies,
            broad_phase,
            dt,
            minimum_batch_index: span,
            maximum_batch_index: -1,
            batches,
        }
    }

    /// Executes a batch of convex shape bounding box computations.
    pub unsafe fn execute_convex_batch<TShape: IConvexShape + Copy + Default + 'static>(
        &mut self,
        shape_batch: &ConvexShapeBatch<TShape>,
    ) {
        let batch = self.batches.get(shape_batch.type_id() as i32);
        let shape_indices = &batch.shape_indices;
        let continuations = &batch.continuations;
        let bodies = &mut *self.bodies;
        let active_set = bodies.active_set_mut();
        let _dt_wide = Vector::<f32>::splat(self.dt);
        let broad_phase = &mut *self.broad_phase;

        let mut bundle_start_index = 0i32;
        while bundle_start_index < batch.count {
            let mut count_in_bundle = batch.count - bundle_start_index;
            if count_in_bundle > VECTOR_WIDTH as i32 {
                count_in_bundle = VECTOR_WIDTH as i32;
            }

            // NOTE: Full wide batch processing with TransposeMotionStates, shapeWide, etc. not yet implemented.
            // For now, do scalar per-shape processing (functionally correct, not yet SIMD-optimized).
            for inner_index in 0..count_in_bundle {
                let index_in_batch = bundle_start_index + inner_index;
                let shape_index = *shape_indices.get(index_in_batch);
                let continuation = *continuations.get(index_in_batch);
                let motion_state = batch.motion_states.get(index_in_batch);

                let shape = shape_batch.get(shape_index as usize);
                let mut min = Vec3::ZERO;
                let mut max = Vec3::ZERO;
                shape.compute_bounds(motion_state.pose.orientation, &mut min, &mut max);

                let mut maximum_radius = 0.0f32;
                let mut maximum_angular_expansion = 0.0f32;
                shape.compute_angular_expansion_data(&mut maximum_radius, &mut maximum_angular_expansion);

                let angular_bounds_expansion = BoundingBoxHelpers::get_angular_bounds_expansion(
                    motion_state.velocity.angular.length(),
                    self.dt,
                    maximum_radius,
                    maximum_angular_expansion,
                );
                let mut speculative_margin = motion_state.velocity.linear.length() * self.dt + angular_bounds_expansion;

                let collidable = active_set.collidables.get_mut(continuation.body_index());
                speculative_margin = speculative_margin
                    .max(collidable.minimum_speculative_margin)
                    .min(collidable.maximum_speculative_margin);

                let maximum_allowed_expansion = if collidable.continuity.allow_expansion_beyond_speculative_margin() {
                    f32::MAX
                } else {
                    speculative_margin
                };

                let mut min_expansion = Vec3::ZERO;
                let mut max_expansion = Vec3::ZERO;
                BoundingBoxHelpers::get_bounds_expansion_scalar(
                    motion_state.velocity.linear,
                    self.dt,
                    angular_bounds_expansion,
                    &mut min_expansion,
                    &mut max_expansion,
                );
                let max_exp_vec = Vec3::splat(maximum_allowed_expansion);
                min_expansion = min_expansion.max(-max_exp_vec);
                max_expansion = max_expansion.min(max_exp_vec);

                let (min_ptr, max_ptr) = broad_phase.get_active_bounds_pointers(collidable.broad_phase_index);

                if continuation.compound_child() {
                    collidable.speculative_margin = collidable.speculative_margin.max(speculative_margin);
                    let new_min = motion_state.pose.position + (min + min_expansion);
                    let new_max = motion_state.pose.position + (max + max_expansion);
                    BoundingBox::create_merged(*min_ptr, *max_ptr, new_min, new_max, &mut *min_ptr, &mut *max_ptr);
                } else {
                    collidable.speculative_margin = speculative_margin;
                    *min_ptr = motion_state.pose.position + (min + min_expansion);
                    *max_ptr = motion_state.pose.position + (max + max_expansion);
                }
            }

            bundle_start_index += VECTOR_WIDTH as i32;
        }
    }

    /// Executes a batch of homogeneous compound shape bounding box computations.
    /// For shapes like Mesh where all children are the same type and the shape computes
    /// its own overall bounds directly.
    pub unsafe fn execute_homogeneous_compound_batch<TShape>(
        &mut self,
        shape_batch: &HomogeneousCompoundShapeBatch<TShape>,
    )
    where
        TShape: IDisposableShape + INonConvexBounds + crate::physics::collidables::shape::IShape + Copy + Default + 'static,
    {
        let batch = self.batches.get(shape_batch.type_id() as i32);
        let bodies = &mut *self.bodies;
        let active_set = bodies.active_set_mut();
        let broad_phase = &mut *self.broad_phase;

        for i in 0..batch.count {
            let shape_index = *batch.shape_indices.get(i);
            let motion_state = batch.motion_states.get(i);
            let continuation = *batch.continuations.get(i);
            let body_index = continuation.body_index();
            let collidable = active_set.collidables.get_mut(body_index);

            let shape = shape_batch.get(shape_index as usize);
            let mut min = Vec3::ZERO;
            let mut max = Vec3::ZERO;
            shape.compute_bounds_by_orientation(motion_state.pose.orientation, &mut min, &mut max);

            // Compute angular expansion from bounding box extents (simplified for non-convex).
            let abs_min = min.abs();
            let abs_max = max.abs();
            let max_components = abs_min.max(abs_max);
            let maximum_radius = max_components.length();
            let min_components = abs_min.min(abs_max);
            let minimum_radius = min_components.x.min(min_components.y.min(min_components.z));
            let maximum_angular_expansion = maximum_radius - minimum_radius;

            let angular_bounds_expansion = BoundingBoxHelpers::get_angular_bounds_expansion(
                motion_state.velocity.angular.length(),
                self.dt,
                maximum_radius,
                maximum_angular_expansion,
            );
            let mut speculative_margin = motion_state.velocity.linear.length() * self.dt + angular_bounds_expansion;
            speculative_margin = speculative_margin
                .max(collidable.minimum_speculative_margin)
                .min(collidable.maximum_speculative_margin);
            collidable.speculative_margin = speculative_margin;

            let maximum_allowed_expansion = if collidable.continuity.allow_expansion_beyond_speculative_margin() {
                f32::MAX
            } else {
                speculative_margin
            };

            let mut min_expansion = Vec3::ZERO;
            let mut max_expansion = Vec3::ZERO;
            BoundingBoxHelpers::get_bounds_expansion_scalar(
                motion_state.velocity.linear,
                self.dt,
                angular_bounds_expansion,
                &mut min_expansion,
                &mut max_expansion,
            );
            let max_exp_vec = Vec3::splat(maximum_allowed_expansion);
            min_expansion = min_expansion.max(-max_exp_vec);
            max_expansion = max_expansion.min(max_exp_vec);

            let (min_ptr, max_ptr) = broad_phase.get_active_bounds_pointers(collidable.broad_phase_index);
            *min_ptr = motion_state.pose.position + (min + min_expansion);
            *max_ptr = motion_state.pose.position + (max + max_expansion);
        }
    }

    /// Executes a batch of compound shape bounding box computations.
    /// For shapes like Compound and BigCompound where children are different types.
    /// Initializes bounds to empty then dispatches each child back through the batcher.
    pub unsafe fn execute_compound_batch<TShape>(
        &mut self,
        shape_batch: &CompoundShapeBatch<TShape>,
    )
    where
        TShape: ICompoundShape + crate::physics::collidables::shape::IShape + Copy + Default + 'static,
    {
        let batcher_ptr = self as *mut BoundingBoxBatcher;
        let batch = self.batches.get(shape_batch.type_id() as i32);
        let bodies = &mut *self.bodies;
        let active_set = bodies.active_set_mut();
        let broad_phase = &mut *self.broad_phase;

        for i in 0..batch.count {
            let shape_index = *batch.shape_indices.get(i);
            let body_index = batch.continuations.get(i).body_index();
            let motion_state = *batch.motion_states.get(i);
            let collidable = active_set.collidables.get_mut(body_index);

            // Initialize bounds to empty and speculative margin to 0 — children will merge into these.
            collidable.speculative_margin = 0.0;
            let (min_ptr, max_ptr) = broad_phase.get_active_bounds_pointers(collidable.broad_phase_index);
            *min_ptr = Vec3::splat(f32::MAX);
            *max_ptr = Vec3::splat(-f32::MAX);

            // Dispatch each child shape back through the batcher.
            let shape = shape_batch.get(shape_index as usize);
            shape.add_child_bounds_to_batcher(
                &mut *batcher_ptr,
                &motion_state.pose,
                &motion_state.velocity,
                body_index,
            );
        }
    }

    fn add_internal(
        &mut self,
        shape_index: TypedIndex,
        pose: RigidPose,
        velocity: BodyVelocity,
        continuation: BoundsContinuation,
    ) {
        let type_index = shape_index.type_id();
        debug_assert!(type_index >= 0 && type_index < self.batches.len());
        let batch_slot = self.batches.get_mut(type_index);
        if !batch_slot.allocated() {
            let pool = unsafe { &mut *self.pool };
            *batch_slot = BoundingBoxBatch::new(pool, COLLIDABLES_PER_FLUSH);
            if type_index < self.minimum_batch_index {
                self.minimum_batch_index = type_index;
            }
            if type_index > self.maximum_batch_index {
                self.maximum_batch_index = type_index;
            }
        }
        batch_slot.add(shape_index.index(), pose, velocity, continuation);
        if batch_slot.count == COLLIDABLES_PER_FLUSH {
            unsafe {
                let shapes = &*self.shapes;
                let batcher_ptr = self as *mut BoundingBoxBatcher;
                if let Some(shape_batch) = shapes.get_batch(type_index as usize) {
                    shape_batch.compute_bounds_for_batcher(&mut *batcher_ptr);
                }
                (*batcher_ptr).batches.get_mut(type_index).count = 0;
            }
        }
    }

    /// Adds a body to be processed for bounding box computation.
    #[inline(always)]
    pub fn add(
        &mut self,
        body_index: i32,
        pose: &RigidPose,
        velocity: &BodyVelocity,
        collidable: &Collidable,
    ) {
        if collidable.shape.exists() {
            self.add_internal(
                collidable.shape,
                *pose,
                *velocity,
                BoundsContinuation::create_continuation(body_index),
            );
        }
    }

    /// Adds a compound child shape for bounding box computation.
    pub fn add_compound_child(
        &mut self,
        body_index: i32,
        shape_index: TypedIndex,
        pose: &RigidPose,
        velocity: &BodyVelocity,
    ) {
        self.add_internal(
            shape_index,
            *pose,
            *velocity,
            BoundsContinuation::create_compound_child_continuation(body_index),
        );
    }

    /// Flushes all remaining batched bounding box computations.
    #[inline(always)]
    pub fn flush(&mut self) {
        let pool = unsafe { &mut *self.pool };
        // Process from highest to lowest to handle compound shapes first.
        let mut i = self.maximum_batch_index;
        while i >= self.minimum_batch_index {
            let batch_count = self.batches.get(i).count;
            if batch_count > 0 {
                unsafe {
                    let shapes = &*self.shapes;
                    let batcher_ptr = self as *mut BoundingBoxBatcher;
                    if let Some(shape_batch) = shapes.get_batch(i as usize) {
                        shape_batch.compute_bounds_for_batcher(&mut *batcher_ptr);
                    }
                }
            }
            self.batches.get_mut(i).dispose(pool);
            i -= 1;
        }
        pool.return_buffer(&mut self.batches);
    }
}
