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
    shapes: Buffer<TShape>,
    shapes_data: Buffer<u8>,
    id_pool: IdPool,
    type_id_val: i32,
    pool: *mut BufferPool,
}

impl<TShape: IConvexShape + Copy + Default + 'static> ConvexShapeBatch<TShape> {
    pub fn new(pool: &mut BufferPool, initial_capacity: usize) -> Self {
        let mut batch = Self {
            shapes: Buffer::default(),
            shapes_data: Buffer::default(),
            id_pool: IdPool::new(initial_capacity as i32, pool),
            type_id_val: TShape::type_id(),
            pool: pool as *mut BufferPool,
        };
        batch.internal_resize(initial_capacity as i32, 0);
        batch
    }

    fn internal_resize(&mut self, shape_count: i32, old_copy_length: i32) {
        let pool = unsafe { &mut *self.pool };
        let required_size_in_bytes = shape_count * std::mem::size_of::<TShape>() as i32;
        let new_shapes_data: Buffer<u8> = pool.take_at_least(required_size_in_bytes);
        let mut new_shapes: Buffer<TShape> = new_shapes_data.cast();
        if self.shapes_data.allocated() {
            self.shapes.copy_to(0, &mut new_shapes, 0, old_copy_length);
            pool.return_buffer(&mut self.shapes_data);
        }
        self.shapes = new_shapes;
        self.shapes_data = new_shapes_data;
    }

    /// Adds a shape and returns the index assigned to it.
    pub fn add(&mut self, shape: TShape) -> usize {
        let pool = unsafe { &mut *self.pool };
        let index = self.id_pool.take() as usize;
        if (self.shapes.len() as usize) <= index {
            let old_len = self.shapes.len();
            self.internal_resize(index as i32 + 1, old_len);
        }
        *self.shapes.get_mut(index as i32) = shape;
        index
    }

    /// Gets a reference to the shape at the given index.
    pub fn get(&self, index: usize) -> &TShape {
        self.shapes.get(index as i32)
    }

    /// Gets a mutable reference to the shape at the given index.
    pub fn get_mut(&mut self, index: usize) -> &mut TShape {
        self.shapes.get_mut(index as i32)
    }
}

impl<TShape: IConvexShape + Copy + Default + 'static> ShapeBatch for ConvexShapeBatch<TShape> {
    fn capacity(&self) -> usize {
        self.shapes.len() as usize
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
        self.shapes.get(shape_index as i32).compute_bounds(orientation, min, max);
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
        let shape = self.shapes.get(shape_index as i32);
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
        if (self.shapes.len() as usize) < shape_capacity {
            let old_len = self.shapes.len();
            self.internal_resize(shape_capacity as i32, old_len);
        }
    }

    fn resize(&mut self, shape_capacity: usize) {
        let min_cap = (self.id_pool.highest_possibly_claimed_id() + 1) as usize;
        let target = shape_capacity.max(min_cap);
        let pool = unsafe { &mut *self.pool };
        let target = BufferPool::get_capacity_for_count::<TShape>(target as i32);
        if target != self.shapes.len() {
            let old_copy_len = (self.id_pool.highest_possibly_claimed_id() + 1).min(self.shapes.len());
            self.internal_resize(target, old_copy_len);
        }
    }

    fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        pool.return_buffer(&mut self.shapes_data);
        self.shapes = Buffer::default();
        self.id_pool.dispose(pool);
    }

    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize) {
        let ptr = self.shapes.get(shape_index as i32) as *const TShape as *const u8;
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
        if self.shapes.get(shape_index as i32).ray_test(pose, ray.origin, ray.direction, &mut t, &mut normal) && t <= *maximum_t {
            hit_handler.on_ray_hit(ray, maximum_t, t, normal, 0);
        }
    }

    fn try_compute_inertia(&self, shape_index: usize, mass: f32) -> Option<BodyInertia> {
        Some(self.shapes.get(shape_index as i32).compute_inertia(mass))
    }
}

impl<TShape: IConvexShape + Copy + Default + 'static> IConvexShapeBatch for ConvexShapeBatch<TShape> {
    fn compute_inertia(&self, shape_index: usize, mass: f32) -> BodyInertia {
        self.shapes.get(shape_index as i32).compute_inertia(mass)
    }
}

// ─── ConvexHullShapeBatch ────────────────────────────────────────────────────

use super::convex_hull::ConvexHull;

/// Specialized convex shape batch for ConvexHull that disposes internal buffers on removal.
/// Equivalent to C# `ConvexHullShapeBatch`.
pub struct ConvexHullShapeBatch {
    inner: ConvexShapeBatch<ConvexHull>,
}

impl ConvexHullShapeBatch {
    pub fn new(pool: &mut BufferPool, initial_capacity: usize) -> Self {
        Self {
            inner: ConvexShapeBatch::new(pool, initial_capacity),
        }
    }
}

impl ShapeBatch for ConvexHullShapeBatch {
    fn capacity(&self) -> usize { self.inner.capacity() }
    fn type_id(&self) -> i32 { self.inner.type_id() }
    fn compound(&self) -> bool { false }
    fn shape_data_size(&self) -> usize { self.inner.shape_data_size() }

    fn remove(&mut self, index: usize) { self.inner.remove(index); }

    fn remove_and_dispose(&mut self, index: usize, pool: &mut BufferPool) {
        // ConvexHull has internal buffers that must be returned to the pool.
        self.inner.shapes.get_mut(index as i32).dispose(pool);
        self.inner.remove(index);
    }

    fn compute_bounds_by_orientation(&self, shape_index: usize, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        self.inner.compute_bounds_by_orientation(shape_index, orientation, min, max);
    }

    fn compute_bounds_with_angular_data(&self, shape_index: usize, orientation: Quat,
        maximum_radius: &mut f32, maximum_angular_expansion: &mut f32, min: &mut Vec3, max: &mut Vec3) {
        self.inner.compute_bounds_with_angular_data(shape_index, orientation, maximum_radius, maximum_angular_expansion, min, max);
    }

    unsafe fn compute_bounds_for_batcher(&self, batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher) {
        self.inner.compute_bounds_for_batcher(batcher);
    }

    fn clear(&mut self) { self.inner.clear(); }
    fn ensure_capacity(&mut self, shape_capacity: usize) { self.inner.ensure_capacity(shape_capacity); }
    fn resize(&mut self, shape_capacity: usize) { self.inner.resize(shape_capacity); }
    fn dispose(&mut self) { self.inner.dispose(); }

    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize) {
        self.inner.get_shape_data(shape_index)
    }

    unsafe fn add_raw(&mut self, data: *const u8) -> usize {
        self.inner.add_raw(data)
    }

    unsafe fn ray_test(&self, shape_index: usize, pose: &RigidPose,
        ray: &crate::physics::collision_detection::ray_batchers::RayData,
        maximum_t: &mut f32,
        hit_handler: &mut dyn crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler) {
        self.inner.ray_test(shape_index, pose, ray, maximum_t, hit_handler);
    }

    fn try_compute_inertia(&self, shape_index: usize, mass: f32) -> Option<BodyInertia> {
        self.inner.try_compute_inertia(shape_index, mass)
    }
}

impl IConvexShapeBatch for ConvexHullShapeBatch {
    fn compute_inertia(&self, shape_index: usize, mass: f32) -> BodyInertia {
        self.inner.compute_inertia(shape_index, mass)
    }
}

// ─── HomogeneousCompoundShapeBatch ───────────────────────────────────────────

use super::shape::{IDisposableShape, INonConvexBounds};

/// Shape batch for homogeneous compound shapes (all children are the same type, e.g. Mesh).
/// Equivalent to C# `HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide>`.
pub struct HomogeneousCompoundShapeBatch<TShape: IDisposableShape + INonConvexBounds + IShape + Copy + Default> {
    shapes: Buffer<TShape>,
    shapes_data: Buffer<u8>,
    id_pool: IdPool,
    type_id_val: i32,
    pool: *mut BufferPool,
}

impl<TShape: IDisposableShape + INonConvexBounds + IShape + Copy + Default + 'static> HomogeneousCompoundShapeBatch<TShape> {
    pub fn new(pool: &mut BufferPool, initial_capacity: usize) -> Self {
        let mut batch = Self {
            shapes: Buffer::default(),
            shapes_data: Buffer::default(),
            id_pool: IdPool::new(initial_capacity as i32, pool),
            type_id_val: TShape::type_id(),
            pool: pool as *mut BufferPool,
        };
        batch.internal_resize(initial_capacity as i32, 0);
        batch
    }

    fn internal_resize(&mut self, shape_count: i32, old_copy_length: i32) {
        let pool = unsafe { &mut *self.pool };
        let required_size_in_bytes = shape_count * std::mem::size_of::<TShape>() as i32;
        let new_shapes_data: Buffer<u8> = pool.take_at_least(required_size_in_bytes);
        let mut new_shapes: Buffer<TShape> = new_shapes_data.cast();
        if self.shapes_data.allocated() {
            self.shapes.copy_to(0, &mut new_shapes, 0, old_copy_length);
            pool.return_buffer(&mut self.shapes_data);
        }
        self.shapes = new_shapes;
        self.shapes_data = new_shapes_data;
    }

    /// Gets a reference to the shape at the given index.
    pub fn get(&self, index: usize) -> &TShape {
        self.shapes.get(index as i32)
    }

    /// Gets a mutable reference to the shape at the given index.
    pub fn get_mut(&mut self, index: usize) -> &mut TShape {
        self.shapes.get_mut(index as i32)
    }
}

impl<TShape: IDisposableShape + INonConvexBounds + IShape + Copy + Default + 'static> ShapeBatch for HomogeneousCompoundShapeBatch<TShape> {
    fn capacity(&self) -> usize { self.shapes.len() as usize }
    fn type_id(&self) -> i32 { self.type_id_val }
    fn compound(&self) -> bool { true }
    fn shape_data_size(&self) -> usize { std::mem::size_of::<TShape>() }

    fn remove(&mut self, index: usize) {
        let pool = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool);
    }

    fn remove_and_dispose(&mut self, index: usize, pool: &mut BufferPool) {
        self.shapes.get_mut(index as i32).dispose(pool);
        let pool_ref = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool_ref);
    }

    fn recursively_remove_and_dispose(&mut self, index: usize, _shapes: &mut Shapes, pool: &mut BufferPool) {
        // Homogeneous compounds don't have shape-registered children, just internal data.
        self.remove_and_dispose(index, pool);
    }

    fn compute_bounds_by_orientation(&self, shape_index: usize, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        self.shapes.get(shape_index as i32).compute_bounds_by_orientation(orientation, min, max);
    }

    unsafe fn compute_bounds_for_batcher(&self, batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher) {
        batcher.execute_homogeneous_compound_batch(self);
    }

    fn clear(&mut self) { self.id_pool.clear(); }

    fn ensure_capacity(&mut self, shape_capacity: usize) {
        if (self.shapes.len() as usize) < shape_capacity {
            let old_len = self.shapes.len();
            self.internal_resize(shape_capacity as i32, old_len);
        }
    }

    fn resize(&mut self, shape_capacity: usize) {
        let min_cap = (self.id_pool.highest_possibly_claimed_id() + 1) as usize;
        let target = shape_capacity.max(min_cap);
        let pool = unsafe { &mut *self.pool };
        let target = BufferPool::get_capacity_for_count::<TShape>(target as i32);
        if target != self.shapes.len() {
            let old_copy_len = (self.id_pool.highest_possibly_claimed_id() + 1).min(self.shapes.len());
            self.internal_resize(target, old_copy_len);
        }
    }

    fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        pool.return_buffer(&mut self.shapes_data);
        self.shapes = Buffer::default();
        self.id_pool.dispose(pool);
    }

    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize) {
        let ptr = self.shapes.get(shape_index as i32) as *const TShape as *const u8;
        (ptr, std::mem::size_of::<TShape>())
    }

    unsafe fn add_raw(&mut self, data: *const u8) -> usize {
        let shape = *(data as *const TShape);
        let pool = unsafe { &mut *self.pool };
        let index = self.id_pool.take() as usize;
        if (self.shapes.len() as usize) <= index {
            let old_len = self.shapes.len();
            self.internal_resize(index as i32 + 1, old_len);
        }
        *self.shapes.get_mut(index as i32) = shape;
        index
    }

    unsafe fn ray_test(&self, _shape_index: usize, _pose: &RigidPose,
        _ray: &crate::physics::collision_detection::ray_batchers::RayData,
        _maximum_t: &mut f32,
        _hit_handler: &mut dyn crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler) {
        // Compound ray tests require the shape's own ray test implementation.
        // This is a placeholder; compound ray tests go through the shape directly.
        todo!("HomogeneousCompoundShapeBatch::ray_test not yet specialized");
    }
}

// ─── CompoundShapeBatch ──────────────────────────────────────────────────────

use super::shape::ICompoundShape;

/// Shape batch for heterogeneous compound shapes (children are different types, e.g. Compound, BigCompound).
/// Stores a reference to the parent Shapes collection for recursive child operations.
/// Equivalent to C# `CompoundShapeBatch<TShape>`.
pub struct CompoundShapeBatch<TShape: ICompoundShape + Copy + Default> {
    shapes: Buffer<TShape>,
    shapes_data: Buffer<u8>,
    id_pool: IdPool,
    type_id_val: i32,
    pool: *mut BufferPool,
    /// Raw pointer to the parent Shapes collection, needed for recursive child disposal
    /// and compound bounds computation. Set during construction.
    shape_batches: *const Shapes,
}

impl<TShape: ICompoundShape + Copy + Default + 'static> CompoundShapeBatch<TShape> {
    pub fn new(pool: &mut BufferPool, initial_capacity: usize, shape_batches: *const Shapes) -> Self {
        let mut batch = Self {
            shapes: Buffer::default(),
            shapes_data: Buffer::default(),
            id_pool: IdPool::new(initial_capacity as i32, pool),
            type_id_val: TShape::type_id(),
            pool: pool as *mut BufferPool,
            shape_batches,
        };
        batch.internal_resize(initial_capacity as i32, 0);
        batch
    }

    fn internal_resize(&mut self, shape_count: i32, old_copy_length: i32) {
        let pool = unsafe { &mut *self.pool };
        let required_size_in_bytes = shape_count * std::mem::size_of::<TShape>() as i32;
        let new_shapes_data: Buffer<u8> = pool.take_at_least(required_size_in_bytes);
        let mut new_shapes: Buffer<TShape> = new_shapes_data.cast();
        if self.shapes_data.allocated() {
            self.shapes.copy_to(0, &mut new_shapes, 0, old_copy_length);
            pool.return_buffer(&mut self.shapes_data);
        }
        self.shapes = new_shapes;
        self.shapes_data = new_shapes_data;
    }

    /// Gets a reference to the shape at the given index.
    pub fn get(&self, index: usize) -> &TShape {
        self.shapes.get(index as i32)
    }

    /// Gets a mutable reference to the shape at the given index.
    pub fn get_mut(&mut self, index: usize) -> &mut TShape {
        self.shapes.get_mut(index as i32)
    }
}

impl<TShape: ICompoundShape + Copy + Default + 'static> ShapeBatch for CompoundShapeBatch<TShape> {
    fn capacity(&self) -> usize { self.shapes.len() as usize }
    fn type_id(&self) -> i32 { self.type_id_val }
    fn compound(&self) -> bool { true }
    fn shape_data_size(&self) -> usize { std::mem::size_of::<TShape>() }

    fn remove(&mut self, index: usize) {
        let pool = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool);
    }

    fn remove_and_dispose(&mut self, index: usize, pool: &mut BufferPool) {
        self.shapes.get_mut(index as i32).dispose(pool);
        let pool_ref = unsafe { &mut *self.pool };
        self.id_pool.return_id(index as i32, pool_ref);
    }

    fn recursively_remove_and_dispose(&mut self, index: usize, shapes: &mut Shapes, pool: &mut BufferPool) {
        let shape = self.shapes.get(index as i32);
        for i in 0..shape.child_count() {
            let child = shape.get_child(i);
            shapes.recursively_remove_and_dispose(&child.shape_index, pool);
        }
        self.remove_and_dispose(index, pool);
    }

    fn compute_bounds_by_orientation(&self, _shape_index: usize, _orientation: Quat, _min: &mut Vec3, _max: &mut Vec3) {
        // Compound bounds computation requires access to the shapes collection.
        // The batcher path is the primary computation pathway.
        todo!("CompoundShapeBatch::compute_bounds_by_orientation not yet specialized");
    }

    unsafe fn compute_bounds_for_batcher(&self, batcher: &mut crate::physics::bounding_box_batcher::BoundingBoxBatcher) {
        batcher.execute_compound_batch(self);
    }

    fn clear(&mut self) { self.id_pool.clear(); }

    fn ensure_capacity(&mut self, shape_capacity: usize) {
        if (self.shapes.len() as usize) < shape_capacity {
            let old_len = self.shapes.len();
            self.internal_resize(shape_capacity as i32, old_len);
        }
    }

    fn resize(&mut self, shape_capacity: usize) {
        let min_cap = (self.id_pool.highest_possibly_claimed_id() + 1) as usize;
        let target = shape_capacity.max(min_cap);
        let pool = unsafe { &mut *self.pool };
        let target = BufferPool::get_capacity_for_count::<TShape>(target as i32);
        if target != self.shapes.len() {
            let old_copy_len = (self.id_pool.highest_possibly_claimed_id() + 1).min(self.shapes.len());
            self.internal_resize(target, old_copy_len);
        }
    }

    fn dispose(&mut self) {
        let pool = unsafe { &mut *self.pool };
        pool.return_buffer(&mut self.shapes_data);
        self.shapes = Buffer::default();
        self.id_pool.dispose(pool);
    }

    unsafe fn get_shape_data(&self, shape_index: usize) -> (*const u8, usize) {
        let ptr = self.shapes.get(shape_index as i32) as *const TShape as *const u8;
        (ptr, std::mem::size_of::<TShape>())
    }

    unsafe fn add_raw(&mut self, data: *const u8) -> usize {
        let shape = *(data as *const TShape);
        let pool = unsafe { &mut *self.pool };
        let index = self.id_pool.take() as usize;
        if (self.shapes.len() as usize) <= index {
            let old_len = self.shapes.len();
            self.internal_resize(index as i32 + 1, old_len);
        }
        *self.shapes.get_mut(index as i32) = shape;
        index
    }

    unsafe fn ray_test(&self, _shape_index: usize, _pose: &RigidPose,
        _ray: &crate::physics::collision_detection::ray_batchers::RayData,
        _maximum_t: &mut f32,
        _hit_handler: &mut dyn crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler) {
        // Compound ray tests go through the shape's own implementation, passing the shapes collection.
        todo!("CompoundShapeBatch::ray_test not yet specialized");
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

    /// Ensures a batch slot exists and is initialized for the given type_id.
    fn ensure_batch_slot(&mut self, type_id: usize) {
        if self.registered_type_span <= type_id {
            self.registered_type_span = type_id + 1;
            if self.batches.len() <= type_id {
                self.batches.resize_with(type_id + 1, || None);
            }
        }
    }

    /// Adds a ConvexHull shape to the collection.
    /// ConvexHull shapes have internal buffers that require disposal.
    pub fn add_convex_hull(&mut self, shape: &ConvexHull) -> TypedIndex {
        let type_id = ConvexHull::type_id() as usize;
        self.ensure_batch_slot(type_id);
        if self.batches[type_id].is_none() {
            let pool = unsafe { &mut *self.pool };
            self.batches[type_id] = Some(Box::new(
                ConvexHullShapeBatch::new(pool, self.initial_capacity_per_type_batch),
            ));
        }
        let batch = self.batches[type_id].as_mut().unwrap();
        let index = unsafe { batch.add_raw(shape as *const ConvexHull as *const u8) };
        TypedIndex::new(type_id as i32, index as i32)
    }

    /// Adds a Mesh shape to the collection.
    /// Mesh shapes use a HomogeneousCompoundShapeBatch.
    pub fn add_mesh(&mut self, shape: &super::mesh::Mesh) -> TypedIndex {
        use super::mesh::Mesh;
        let type_id = Mesh::type_id() as usize;
        self.ensure_batch_slot(type_id);
        if self.batches[type_id].is_none() {
            let pool = unsafe { &mut *self.pool };
            self.batches[type_id] = Some(Box::new(
                HomogeneousCompoundShapeBatch::<Mesh>::new(pool, self.initial_capacity_per_type_batch),
            ));
        }
        let batch = self.batches[type_id].as_mut().unwrap();
        let index = unsafe { batch.add_raw(shape as *const Mesh as *const u8) };
        TypedIndex::new(type_id as i32, index as i32)
    }

    /// Adds a Compound shape to the collection.
    /// Compound shapes use a CompoundShapeBatch with a reference back to this Shapes collection.
    pub fn add_compound(&mut self, shape: &super::compound::Compound) -> TypedIndex {
        use super::compound::Compound;
        let type_id = Compound::type_id() as usize;
        self.ensure_batch_slot(type_id);
        if self.batches[type_id].is_none() {
            let pool = unsafe { &mut *self.pool };
            let shapes_ptr = self as *const Shapes;
            self.batches[type_id] = Some(Box::new(
                CompoundShapeBatch::<Compound>::new(pool, self.initial_capacity_per_type_batch, shapes_ptr),
            ));
        }
        let batch = self.batches[type_id].as_mut().unwrap();
        let index = unsafe { batch.add_raw(shape as *const Compound as *const u8) };
        TypedIndex::new(type_id as i32, index as i32)
    }

    /// Adds a BigCompound shape to the collection.
    /// BigCompound shapes use a CompoundShapeBatch with a reference back to this Shapes collection.
    pub fn add_big_compound(&mut self, shape: &super::big_compound::BigCompound) -> TypedIndex {
        use super::big_compound::BigCompound;
        let type_id = BigCompound::type_id() as usize;
        self.ensure_batch_slot(type_id);
        if self.batches[type_id].is_none() {
            let pool = unsafe { &mut *self.pool };
            let shapes_ptr = self as *const Shapes;
            self.batches[type_id] = Some(Box::new(
                CompoundShapeBatch::<BigCompound>::new(pool, self.initial_capacity_per_type_batch, shapes_ptr),
            ));
        }
        let batch = self.batches[type_id].as_mut().unwrap();
        let index = unsafe { batch.add_raw(shape as *const BigCompound as *const u8) };
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
            // Take the batch out temporarily to allow passing &mut self for child removal.
            if let Some(mut batch) = self.batches[type_id].take() {
                batch.recursively_remove_and_dispose(shape_index.index() as usize, self, pool);
                self.batches[type_id] = Some(batch);
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
