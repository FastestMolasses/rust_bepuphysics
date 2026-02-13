use glam::Vec3;

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector3_wide::Vector3Wide;

/// Data for a single ray.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RayData {
    pub origin: Vec3,
    pub id: i32,
    pub direction: Vec3,
}

impl Default for RayData {
    fn default() -> Self {
        Self {
            origin: Vec3::ZERO,
            id: 0,
            direction: Vec3::ZERO,
        }
    }
}

/// Bundled ray data for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct RayWide {
    pub origin: Vector3Wide,
    pub direction: Vector3Wide,
}

impl RayWide {
    /// Gathers a single ray into the first SIMD lane.
    #[inline(always)]
    pub fn gather(&mut self, ray: &RayData) {
        unsafe {
            *GatherScatter::get_first_mut(&mut self.origin.x) = ray.origin.x;
            *GatherScatter::get_first_mut(&mut self.origin.y) = ray.origin.y;
            *GatherScatter::get_first_mut(&mut self.origin.z) = ray.origin.z;
            *GatherScatter::get_first_mut(&mut self.direction.x) = ray.direction.x;
            *GatherScatter::get_first_mut(&mut self.direction.y) = ray.direction.y;
            *GatherScatter::get_first_mut(&mut self.direction.z) = ray.direction.z;
        }
    }
}
