use glam::{Quat, Vec3};

use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::memory::id_pool::IdPool;

use crate::physics::body_properties::{BodyInertia, RigidPose};
use super::shape::{IConvexShape, IShape};
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

    /// Recursively removes and disposes a shape and its children.
    fn recursively_remove_and_dispose(&mut self, index: usize, shapes: &mut Shapes, pool: &mut BufferPool) {
        // Default: no children, just remove_and_dispose.
        self.remove_and_dispose(index, pool);
    }

    /// Computes bounds for a shape at the given index with the given orientation.
    fn compute_bounds_by_orientation(
        &self,
        shape_index: usize,
        orientation: Quat,
        min: &mut Vec3,
        max: &mut Vec3,
    );

    /// Computes bounds for a shape at the given index with the given orientation,
    /// also returning maximum radius and angular expansion data.
    /// Only valid for convex shape batches; nonconvex shapes will panic.
    fn compute_bounds_with_angular_data(
        &self,
        shape_index: usize,
        orientation: Quat,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        let _ = (shape_index, orientation, maximum_radius, maximum_angular_expansion, min, max);
        panic!("Nonconvex shapes are not required to have a maximum radius or angular expansion implementation. This should only ever be called on convexes.");
    }

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

    /// Dispatches batched bounding box computations to the given batcher.
    /// Each shape batch type knows how to invoke the correct batcher method.
    ///
    /// # Safety
    /// The batcher must be properly initialized and the shape data must be valid.
    unsafe fn compute_bounds_for_batcher(
        &self,
        batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher,
    );

    /// Frees all shape slots without returning resources to the pool.
    fn clear(&mut self);

    /// Ensures the batch can hold at least `shape_capacity` shapes.
    fn ensure_capacity(&mut self, shape_capacity: usize);

    /// Resizes the batch to the target capacity (conservatively).
    fn resize(&mut self, shape_capacity: usize);

    /// Returns all backing resources to the pool.
    fn dispose(&mut self);

    /// Gets a raw pointer to the shape data at the given index plus its size.
    ///
    /// # Safety
    /// Caller must ensure `shape_index` is valid.
    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize);

    /// Adds a shape from raw bytes and returns the index.
    ///
    /// # Safety
    /// Caller must provide valid shape data of the correct type and size.
    unsafe fn add_raw(&mut self, data: *const u8) -> usize;

    /// Tests a ray against a shape at the given index.
    ///
    /// # Safety
    /// Caller must ensure shape_index is valid and pose/ray data are valid.
    unsafe fn ray_test(
        &self,
        shape_index: usize,
        pose: &RigidPose,
        ray: &crate::physics::collision_detection::ray_batchers::RayData,
        maximum_t: &mut f32,
        hit_handler: &mut dyn crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler,
    );

    /// Tries to compute inertia for a shape. Returns `Some` for convex shape batches,
    /// `None` for non-convex (compound/mesh) batches.
    fn try_compute_inertia(&self, _shape_index: usize, _mass: f32) -> Option<BodyInertia> {
        None
    }
}

/// Trait for convex shape batches that support inertia computation.
pub trait IConvexShapeBatch: ShapeBatch {
    /// Computes the inertia of a shape.
    fn compute_inertia(&self, shape_index: usize, mass: f32) -> BodyInertia;
}

/// A concrete typed shape batch for convex shapes.
pub struct ConvexShapeBatch<TShape: IConvexShape + Copy + Default> {
    shapes: Vec<TShape>,
    id_pool: IdPool,
    type_id_val: i32,
    pool: *mut BufferPool,
}

impl<TShape: IConvexShape + Copy + Default + 'static> ConvexShapeBatch<TShape> {
    pub fn new(pool: &mut BufferPool, initial_capacity: usize) -> Self {
        Self {
            shapes: vec![TShape::default(); initial_capacity],
            id_pool: IdPool::new(initial_capacity as i32, pool),
            type_id_val: TShape::type_id(),
            pool: pool as *mut BufferPool,
        }
    }

    /// Adds a shape and returns the index assigned to it.
    pub fn add(&mut self, shape: TShape) -> usize {
        let pool = unsafe { &mut *self.pool };
        let index = self.id_pool.take() as usize;
        if self.shapes.len() <= index {
            self.shapes.resize(index + 1, TShape::default());
        }
        self.shapes[index] = shape;
        index
    }

    /// Gets a reference to the shape at the given index.
    pub fn get(&self, index: usize) -> &TShape {
        &self.shapes[index]
    }

    /// Gets a mutable reference to the shape at the given index.
    pub fn get_mut(&mut self, index: usize) -> &mut TShape {
        &mut self.shapes[index]
    }
}

impl<TShape: IConvexShape + Copy + Default + 'static> ShapeBatch for ConvexShapeBatch<TShape> {
    fn capacity(&self) -> usize {
        self.shapes.len()
    }

    fn type_id(&self) -> i32 {
        self.type_id_val
    }

    fn compound(&self) -> bool {
        false
    }

    fn shape_data_size(&self) -> usize {
        std::mem::size_of::<TShape>()
    }

    fn remove(&mut self, index: usize) {
        let pool = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool);
    }

    fn remove_and_dispose(&mut self, index: usize, _pool: &mut BufferPool) {
        // Convex shapes typically have no internal resources to dispose.
        let pool = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool);
    }

    fn compute_bounds_by_orientation(
        &self,
        shape_index: usize,
        orientation: Quat,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        self.shapes[shape_index].compute_bounds(orientation, min, max);
    }

    fn compute_bounds_with_angular_data(
        &self,
        shape_index: usize,
        orientation: Quat,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        let shape = &self.shapes[shape_index];
        shape.compute_bounds(orientation, min, max);
        shape.compute_angular_expansion_data(maximum_radius, maximum_angular_expansion);
    }

    unsafe fn compute_bounds_for_batcher(
        &self,
        batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher,
    ) {
        batcher.execute_convex_batch(self);
    }

    fn clear(&mut self) {
        self.id_pool.clear();
    }

    fn ensure_capacity(&mut self, shape_capacity: usize) {
        if self.shapes.len() < shape_capacity {
            self.shapes.resize(shape_capacity, TShape::default());
        }
    }

    fn resize(&mut self, shape_capacity: usize) {
        let min_cap = (self.id_pool.highest_possibly_claimed_id() + 1) as usize;
        let target = shape_capacity.max(min_cap);
        self.shapes.resize(target, TShape::default());
    }

    fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        self.id_pool.dispose(pool);
        self.shapes.clear();
    }

    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize) {
        let ptr = &self.shapes[shape_index] as *const TShape as *const u8;
        (ptr, std::mem::size_of::<TShape>())
    }

    unsafe fn add_raw(&mut self, data: *const u8) -> usize {
        let shape = *(data as *const TShape);
        self.add(shape)
    }

    unsafe fn ray_test(
        &self,
        shape_index: usize,
        pose: &RigidPose,
        ray: &crate::physics::collision_detection::ray_batchers::RayData,
        maximum_t: &mut f32,
        hit_handler: &mut dyn crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler,
    ) {
        let mut t = 0.0f32;
        let mut normal = Vec3::ZERO;
        if self.shapes[shape_index].ray_test(pose, ray.origin, ray.direction, &mut t, &mut normal) && t <= *maximum_t {
            hit_handler.on_ray_hit(ray, maximum_t, t, normal, 0);
        }
    }

    fn try_compute_inertia(&self, shape_index: usize, mass: f32) -> Option<BodyInertia> {
        Some(self.shapes[shape_index].compute_inertia(mass))
    }
}

impl<TShape: IConvexShape + Copy + Default + 'static> IConvexShapeBatch for ConvexShapeBatch<TShape> {
    fn compute_inertia(&self, shape_index: usize, mass: f32) -> BodyInertia {
        self.shapes[shape_index].compute_inertia(mass)
    }
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

    /// Adds a convex shape to the shapes collection.
    /// The shape type must have been registered via its `IShape::type_id()` before calling this,
    /// or this method will create a new batch automatically.
    pub fn add<TShape: IConvexShape + Copy + Default + 'static>(&mut self, shape: &TShape) -> TypedIndex {
        let type_id = TShape::type_id() as usize;

        if self.registered_type_span <= type_id {
            self.registered_type_span = type_id + 1;
            if self.batches.len() <= type_id {
                self.batches.resize_with(type_id + 1, || None);
            }
        }

        if self.batches[type_id].is_none() {
            let pool = unsafe { &mut *self.pool };
            self.batches[type_id] = Some(Box::new(
                ConvexShapeBatch::<TShape>::new(pool, self.initial_capacity_per_type_batch),
            ));
        }

        let batch = self.batches[type_id].as_mut().unwrap();
        let index = unsafe { batch.add_raw(shape as *const TShape as *const u8) };
        TypedIndex::new(type_id as i32, index as i32)
    }

    /// Gets a reference to a specific convex shape by type and index.
    ///
    /// # Safety
    /// The caller must ensure that the batch at `type_id` contains shapes of type `TShape`.
    pub unsafe fn get_shape<TShape: IConvexShape + Copy + Default + 'static>(&self, shape_index: i32) -> &TShape {
        let type_id = TShape::type_id() as usize;
        debug_assert!(self.registered_type_span > type_id);
        let batch = self.batches[type_id].as_ref().unwrap();
        let (ptr, _) = batch.get_shape_data(shape_index as usize);
        &*(ptr as *const TShape)
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

    /// Recursively removes a shape and any existing children from the shapes collection.
    pub fn recursively_remove_and_dispose(&mut self, shape_index: &TypedIndex, pool: &mut BufferPool) {
        if shape_index.exists() {
            let type_id = shape_index.type_id() as usize;
            debug_assert!(self.registered_type_span > type_id);
            if let Some(batch) = &mut self.batches[type_id] {
                // Note: recursive children removal requires passing self, which is tricky.
                // For now, just remove and dispose the shape itself.
                batch.remove_and_dispose(shape_index.index() as usize, pool);
            }
        }
    }

    /// Removes a shape and returns its resources to the pool. Does not remove or dispose children.
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
}
