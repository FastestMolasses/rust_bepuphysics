use glam::{Quat, Vec3};

use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::memory::buffer_pool::BufferPool;

use crate::physics::body_properties::{BodyInertia, RigidPose};
use super::typed_index::TypedIndex;

/// Abstract base trait for shape batches. Each shape type gets its own batch.
pub trait ShapeBatch {
    /// Gets the number of shapes this batch can currently hold without resizing.
    fn capacity(&self) -> usize;

    /// Gets the type id of the shape type in this batch.
    fn type_id(&self) -> i32;

    /// Gets whether this batch's shape type potentially contains children requiring other batches.
    fn compound(&self) -> bool;

    /// Gets the size of one shape instance in bytes.
    fn shape_data_size(&self) -> usize;

    /// Removes a shape from the batch (frees the slot but does not dispose resources).
    fn remove(&mut self, index: usize);

    /// Removes a shape and disposes its internal resources.
    fn remove_and_dispose(&mut self, index: usize, pool: &mut BufferPool);

    /// Computes bounds for a shape at the given index with the given orientation.
    fn compute_bounds_by_orientation(
        &self,
        shape_index: usize,
        orientation: Quat,
        min: &mut Vec3,
        max: &mut Vec3,
    );

    /// Computes bounds for a shape at the given index with the given pose.
    fn compute_bounds_by_pose(
        &self,
        shape_index: usize,
        pose: &RigidPose,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        self.compute_bounds_by_orientation(shape_index, pose.orientation, min, max);
        *min += pose.position;
        *max += pose.position;
    }

    /// Frees all shape slots without returning resources to the pool.
    fn clear(&mut self);

    /// Ensures the batch can hold at least `shape_capacity` shapes.
    fn ensure_capacity(&mut self, shape_capacity: usize);

    /// Resizes the batch to the target capacity (conservatively).
    fn resize(&mut self, shape_capacity: usize);

    /// Returns all backing resources to the pool.
    fn dispose(&mut self);
}

/// Trait for convex shape batches that support inertia computation.
pub trait IConvexShapeBatch: ShapeBatch {
    /// Computes the inertia of a shape.
    fn compute_inertia(&self, shape_index: usize, mass: f32) -> BodyInertia;
}

/// The central shape storage. Manages batches of shapes indexed by type id.
pub struct Shapes {
    /// Batches indexed by type id. Not all slots may be filled.
    batches: Vec<Option<Box<dyn ShapeBatch>>>,
    registered_type_span: usize,
    initial_capacity_per_type_batch: usize,
    pool: *mut BufferPool,
}

impl Shapes {
    /// Creates a new Shapes collection.
    pub fn new(pool: &mut BufferPool, initial_capacity_per_type_batch: usize) -> Self {
        Self {
            batches: Vec::with_capacity(16),
            registered_type_span: 0,
            initial_capacity_per_type_batch,
            pool: pool as *mut BufferPool,
        }
    }

    /// Gets the span of registered types.
    pub fn registered_type_span(&self) -> usize {
        self.registered_type_span
    }

    /// Gets a reference to a shape batch by type index.
    pub fn get_batch(&self, type_index: usize) -> Option<&dyn ShapeBatch> {
        self.batches.get(type_index).and_then(|b| b.as_deref())
    }

    /// Gets a mutable reference to a shape batch by type index.
    pub fn get_batch_mut(&mut self, type_index: usize) -> Option<&mut dyn ShapeBatch> {
        match self.batches.get_mut(type_index) {
            Some(Some(batch)) => Some(batch.as_mut()),
            _ => None,
        }
    }

    /// Computes a bounding box for a single shape.
    pub fn update_bounds(
        &self,
        pose: &RigidPose,
        shape_index: &TypedIndex,
        bounds: &mut BoundingBox,
    ) {
        if let Some(batch) = self.get_batch(shape_index.type_id() as usize) {
            batch.compute_bounds_by_pose(shape_index.index() as usize, pose, &mut bounds.min, &mut bounds.max);
        }
    }

    /// Removes a shape from the shapes collection and returns its resources to the pool.
    pub fn remove_and_dispose(&mut self, shape_index: &TypedIndex, pool: &mut BufferPool) {
        if shape_index.exists() {
            let type_id = shape_index.type_id() as usize;
            debug_assert!(self.registered_type_span > type_id);
            if let Some(batch) = &mut self.batches[type_id] {
                batch.remove_and_dispose(shape_index.index() as usize, pool);
            }
        }
    }

    /// Removes a shape without removing children or disposing resources.
    pub fn remove(&mut self, shape_index: &TypedIndex) {
        if shape_index.exists() {
            let type_id = shape_index.type_id() as usize;
            debug_assert!(self.registered_type_span > type_id);
            if let Some(batch) = &mut self.batches[type_id] {
                batch.remove(shape_index.index() as usize);
            }
        }
    }

    /// Clears all shapes from existing batches without releasing memory.
    pub fn clear(&mut self) {
        for i in 0..self.registered_type_span {
            if let Some(batch) = &mut self.batches[i] {
                batch.clear();
            }
        }
    }

    /// Ensures a minimum capacity for all existing shape batches.
    pub fn ensure_batch_capacities(&mut self, shape_capacity: usize) {
        for i in 0..self.registered_type_span {
            if let Some(batch) = &mut self.batches[i] {
                batch.ensure_capacity(shape_capacity);
            }
        }
    }

    /// Resizes all existing batches for a target capacity.
    pub fn resize_batches(&mut self, shape_capacity: usize) {
        for i in 0..self.registered_type_span {
            if let Some(batch) = &mut self.batches[i] {
                batch.resize(shape_capacity);
            }
        }
    }

    /// Releases all memory from existing batches.
    pub fn dispose(&mut self) {
        for i in 0..self.registered_type_span {
            if let Some(batch) = &mut self.batches[i] {
                batch.dispose();
            }
        }
    }

    // TODO: The following require generic registration infrastructure:
    // - add<TShape>(shape) -> TypedIndex
    // - get_shape<TShape>(index) -> &TShape
    // - recursively_remove_and_dispose(index, pool)
}
