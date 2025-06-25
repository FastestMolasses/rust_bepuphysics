use glam::Vec3;
use crate::utilities::memory::{buffer::Buffer, buffer_pool::BufferPool};

pub struct RayData {
    pub origin: Vec3,
    pub id: i32,
    pub direction: Vec3,
}

/// Ray representation designed for quicker intersection against axis aligned bounding boxes.
pub struct TreeRay {
    pub origin_over_direction: Vec3,
    pub maximum_t: f32,
    pub inverse_direction: Vec3,
}

impl TreeRay {
    #[inline(always)]
    pub fn create_from(origin: Vec3, direction: Vec3, maximum_t: f32, tree_ray: &mut TreeRay) {
        // Note that this division has two odd properties:
        // 1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
        // The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
        // because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
        // 2) To compensate for the clamp and abs, we reintroduce the sign in the numerator.
        tree_ray.inverse_direction = Vec3::new(
            if direction.x < 0.0 { -1.0 } else { 1.0 },
            if direction.y < 0.0 { -1.0 } else { 1.0 },
            if direction.z < 0.0 { -1.0 } else { 1.0 },
        ) / Vec3::max(Vec3::splat(1e-15), Vec3::abs(direction));
        tree_ray.maximum_t = maximum_t;
        tree_ray.origin_over_direction = origin * tree_ray.inverse_direction;
    }

    #[inline(always)]
    pub fn create_from_ray(
        origin: Vec3,
        direction: Vec3,
        maximum_t: f32,
        id: i32,
        ray: &mut RayData,
        tree_ray: &mut TreeRay,
    ) {
        ray.origin = origin;
        ray.direction = direction;
        ray.id = id;
        Self::create_from(origin, direction, maximum_t, tree_ray);
    }
}

pub trait RaySourceTrait {
    fn ray_count(&self) -> i32;
    fn get_ray(&self, ray_index: i32) -> &RayData;
    unsafe fn get_ray_ptrs(&self, ray_index: i32) -> (*mut RayData, *mut f32);
}

#[repr(C)]
pub struct RaySource {
    tree_rays: *mut TreeRay,
    rays: *mut RayData,
    ray_pointers: *mut u16,
    ray_count: i32,
}

impl RaySourceTrait for RaySource {
    /// Gets the number of rays in the batch.
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    /// Gets a reference to the data for a ray.
    ///
    /// # Arguments
    /// * `ray_index` - Index of the ray to grab.
    ///
    /// # Returns
    /// Returns a reference to the ray in the ray source.
    #[inline(always)]
    fn get_ray(&self, ray_index: i32) -> &RayData {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The ray index must be within 0 and RayCount - 1."
        );
        unsafe {
            let remapped_index = *self.ray_pointers.offset(ray_index as isize);
            &*self.rays.offset(remapped_index as isize)
        }
    }

    /// Gets pointers to the data for a ray.
    ///
    /// # Arguments
    /// * `ray_index` - Index of the ray to grab.
    ///
    /// # Returns
    /// * Pointer to the ray's origin and direction. Note that changing the ray's origin and direction mid-traversal will not change the path of the traversal,
    ///   but it will be visible by any future leafs impacted by this ray.
    /// * Pointer to the maximum length of the ray in units of the ray's length.
    ///   Decreasing this value will prevent the traversal from visiting more distant nodes later in the traversal.
    #[inline(always)]
    unsafe fn get_ray_ptrs(&self, ray_index: i32) -> (*mut RayData, *mut f32) {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The ray index must be within 0 and RayCount - 1."
        );
        let remapped_index = *self.ray_pointers.offset(ray_index as isize);
        let ray = self.rays.offset(remapped_index as isize);
        let maximum_t =
            &mut (*self.tree_rays.offset(remapped_index as isize)).maximum_t as *mut f32;
        (ray, maximum_t)
    }
}

struct StackEntry {
    node_index: i32,
    ray_count: u16,
    ray_stack: u8,
    depth: u8,
}

// TODO: REST OF RAY BATCHER
// TODO: IMPLEMENT void TestNode
// TODO: IMPLEMENT public unsafe void TestRays
// TODO: IMPLEMENT public bool Add
// TODO: IMPLEMENT public void ResetRays()

pub struct RayBatcher {
    stack_pointer_a0: i32,
    stack_pointer_a1: i32,
    stack_pointer_b: i32,
    stack_pointer: i32,
    stack: Buffer<StackEntry>,
    pool: BufferPool,
    batch_ray_count: i32,
    batch_rays: Buffer<TreeRay>,
    batch_original_rays: Buffer<RayData>,
    ray_capacity: i32,
    preallocated_tree_depth: i32,
    fallback_stack: Buffer<i32>,
}

impl RayBatcher {
    
}

trait TreeRaySourceTrait {
    fn ray_count(&self) -> i32;
    fn get(&self, ray_index: i32) -> i32;
}

#[repr(C)]
struct RootRaySource {
    ray_count: i32,
}

impl RootRaySource {
    #[inline(always)]
    pub fn new(ray_count: i32) -> Self {
        Self { ray_count }
    }
}

impl TreeRaySourceTrait for RootRaySource {
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    #[inline(always)]
    fn get(&self, ray_index: i32) -> i32 {
        ray_index
    }
}

#[repr(C)]
struct TreeRaySource {
    ray_pointers: *mut u16,
    ray_count: i32,
}

impl TreeRaySource {
    #[inline(always)]
    pub fn new(ray_pointers: *mut u16, ray_count: i32) -> Self {
        Self {
            ray_pointers,
            ray_count,
        }
    }
}

impl TreeRaySourceTrait for TreeRaySource {
    #[inline(always)]
    fn ray_count(&self) -> i32 {
        self.ray_count
    }

    #[inline(always)]
    fn get(&self, ray_index: i32) -> i32 {
        debug_assert!(
            ray_index >= 0 && ray_index < self.ray_count,
            "The requested ray index must be within the source's region."
        );
        unsafe { *self.ray_pointers.offset(ray_index as isize) as i32 }
    }
}
