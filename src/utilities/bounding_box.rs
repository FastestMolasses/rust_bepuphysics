use crate::utilities::bounding_sphere::BoundingSphere;
use glam::Vec4Swizzles;
use glam::{Vec3, Vec4};
use std::mem::MaybeUninit;
use std::simd::cmp::SimdPartialOrd;
use std::simd::Simd;

/// Represents a bounding box as two `Vec4` values to avoid complexity associated with a `Vec3`'s empty SIMD lane.
#[repr(C, align(32))]
#[derive(Clone, Copy, Debug)]
pub struct BoundingBox4 {
    /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
    pub min: Vec4,
    /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
    pub max: Vec4,
}

impl BoundingBox4 {
    /// Creates a string representation of the bounding box.
    #[inline(always)]
    pub fn to_string(&self) -> String {
        format!("({}, {})", self.min.xyz(), self.max.xyz())
    }
}

/// Provides simple axis-aligned bounding box functionality.
#[repr(C, align(32))]
#[derive(Clone, Copy, Debug)]
pub struct BoundingBox {
    /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub min: Vec3,
    /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub max: Vec3,
}

impl BoundingBox {
    /// Constructs a bounding box from the specified minimum and maximum.
    #[inline(always)]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Checks if two structures with memory layouts equivalent to the `BoundingBox` intersect.
    /// The referenced values must not be in unpinned managed memory.
    #[inline(always)]
    pub unsafe fn intersects_unsafe<TA, TB>(bbox_a: &TA, bbox_b: &TB) -> bool
    where
        TA: Sized + Copy,
        TB: Sized + Copy,
    {
        // This is a weird function. We're directly interpreting the memory of an incoming type as a vector where we assume the min/max layout matches the BoundingBox.
        // Happens to be convenient!
        debug_assert!(std::mem::size_of::<TA>() == 32 && std::mem::size_of::<TB>() == 32);

        let a_ptr = bbox_a as *const TA as *const f32;
        let b_ptr = bbox_b as *const TB as *const f32;

        let a_min = Simd::<f32, 4>::from_array(*(a_ptr as *const [f32; 4]));
        let a_max = Simd::<f32, 4>::from_array(*(a_ptr.add(4) as *const [f32; 4]));
        let b_min = Simd::<f32, 4>::from_array(*(b_ptr as *const [f32; 4]));
        let b_max = Simd::<f32, 4>::from_array(*(b_ptr.add(4) as *const [f32; 4]));

        let no_intersection =
            SimdPartialOrd::simd_lt(a_max, b_min) | SimdPartialOrd::simd_lt(b_max, a_min);
        // Check only xyz components (ignore w)
        (no_intersection.to_bitmask() & 0b111) == 0
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline(always)]
    pub fn intersects(a: Self, b: Self) -> bool {
        unsafe { Self::intersects_unsafe(&a, &b) }
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline(always)]
    pub fn intersects_bounds(min_a: Vec3, max_a: Vec3, min_b: Vec3, max_b: Vec3) -> bool {
        let max_a = Simd::<f32, 4>::from_array([max_a.x, max_a.y, max_a.z, 0.0]);
        let min_b = Simd::<f32, 4>::from_array([min_b.x, min_b.y, min_b.z, 0.0]);
        let max_b = Simd::<f32, 4>::from_array([max_b.x, max_b.y, max_b.z, 0.0]);
        let min_a = Simd::<f32, 4>::from_array([min_a.x, min_a.y, min_a.z, 0.0]);

        let no_intersection = max_a.simd_lt(min_b) | max_b.simd_lt(min_a);
        // Check only xyz components (ignore w)
        (no_intersection.to_bitmask() & 0b111) == 0
    }

    /// Computes the volume of the bounding box.
    #[inline(always)]
    pub fn compute_volume(bbox: &Self) -> f32 {
        let diagonal = bbox.max - bbox.min;
        diagonal.x * diagonal.y * diagonal.z
    }

    /// Computes a bounding box which contains two other bounding boxes.
    #[inline(always)]
    pub fn create_merged(
        min_a: Vec3,
        max_a: Vec3,
        min_b: Vec3,
        max_b: Vec3,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        *min = min_a.min(min_b);
        *max = max_a.max(max_b);
    }

    /// Computes a bounding box which contains two other bounding boxes.
    #[inline(always)]
    pub fn create_merged_boxes(a: Self, b: Self, merged: &mut Self) {
        Self::create_merged(a.min, a.max, b.min, b.max, &mut merged.min, &mut merged.max);
    }

    /// Merges two structures with memory layouts equivalent to the `BoundingBox`.
    /// The referenced values must not be in unpinned managed memory.
    /// Any data in the empty slots is preserved.
    #[inline(always)]
    pub unsafe fn create_merged_unsafe_with_preservation<TA, TB>(
        bounding_box_a: &TA,
        bounding_box_b: &TB,
        merged: &mut MaybeUninit<TA>,
    ) where
        TA: Sized + Copy,
        TB: Sized + Copy,
    {
        debug_assert!(std::mem::size_of::<TA>() == 32 && std::mem::size_of::<TB>() == 32);

        #[cfg(target_arch = "x86_64")]
        unsafe {
            use std::arch::x86_64::*;

            let a_ptr = bounding_box_a as *const TA as *const f32;
            let b_ptr = bounding_box_b as *const TB as *const f32;
            let result_ptr = merged.as_mut_ptr() as *mut f32;

            // Load vectors
            let a_min = _mm_load_ps(a_ptr);
            let a_max = _mm_load_ps(a_ptr.add(4));
            let b_min = _mm_load_ps(b_ptr);
            let b_max = _mm_load_ps(b_ptr.add(4));

            // Compute min and max
            let min = _mm_min_ps(a_min, b_min);
            let max = _mm_max_ps(a_max, b_max);

            // Load current result values to preserve w component
            let current_min = _mm_load_ps(result_ptr);
            let current_max = _mm_load_ps(result_ptr.add(4));

            // Blend the results - preserve W component (mask: 0b1000 = 8)
            let result_min = if is_x86_feature_detected!("sse4.1") {
                _mm_blend_ps(min, current_min, 8)
            } else {
                let mask = _mm_set_ps(0.0, -1.0, -1.0, -1.0);
                _mm_or_ps(_mm_and_ps(min, mask), _mm_andnot_ps(mask, current_min))
            };

            let result_max = if is_x86_feature_detected!("sse4.1") {
                _mm_blend_ps(max, current_max, 8)
            } else {
                let mask = _mm_set_ps(0.0, -1.0, -1.0, -1.0);
                _mm_or_ps(_mm_and_ps(max, mask), _mm_andnot_ps(mask, current_max))
            };

            // Store results
            _mm_store_ps(result_ptr, result_min);
            _mm_store_ps(result_ptr.add(4), result_max);
        }

        #[cfg(target_arch = "aarch64")]
        unsafe {
            use std::arch::aarch64::*;

            let a_ptr = bounding_box_a as *const TA as *const f32;
            let b_ptr = bounding_box_b as *const TB as *const f32;
            let result_ptr = merged.as_mut_ptr() as *mut f32;

            // Load vectors
            let a_min = vld1q_f32(a_ptr);
            let a_max = vld1q_f32(a_ptr.add(4));
            let b_min = vld1q_f32(b_ptr);
            let b_max = vld1q_f32(b_ptr.add(4));

            // Compute min and max
            let min = vminq_f32(a_min, b_min);
            let max = vmaxq_f32(a_max, b_max);

            // Load current result values to preserve w component
            let current_min = vld1q_f32(result_ptr);
            let current_max = vld1q_f32(result_ptr.add(4));

            // Create blend mask (true for xyz, false for w)
            let mask = vld1q_f32(
                [
                    f32::from_bits(u32::MAX),
                    f32::from_bits(u32::MAX),
                    f32::from_bits(u32::MAX),
                    0.0,
                ]
                .as_ptr(),
            );

            // Blend results preserving W component
            let result_min = vbslq_f32(vreinterpretq_u32_f32(mask), min, current_min);
            let result_max = vbslq_f32(vreinterpretq_u32_f32(mask), max, current_max);

            // Store results
            vst1q_f32(result_ptr, result_min);
            vst1q_f32(result_ptr.add(4), result_max);
        }

        #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
        unsafe {
            let a_ptr = bounding_box_a as *const TA as *const f32;
            let b_ptr = bounding_box_b as *const TB as *const f32;
            let result_ptr = merged.as_mut_ptr() as *mut f32;

            // Load into SIMD vectors
            let a_min = Simd::<f32, 4>::from_array(*(a_ptr as *const [f32; 4]));
            let a_max = Simd::<f32, 4>::from_array(*(a_ptr.add(4) as *const [f32; 4]));
            let b_min = Simd::<f32, 4>::from_array(*(b_ptr as *const [f32; 4]));
            let b_max = Simd::<f32, 4>::from_array(*(b_ptr.add(4) as *const [f32; 4]));

            // Compute min and max
            let current_min = Simd::<f32, 4>::from_array(*(result_ptr as *const [f32; 4]));
            let current_max = Simd::<f32, 4>::from_array(*(result_ptr.add(4) as *const [f32; 4]));
            let min = std::simd::num::SimdFloat::simd_min(a_min, b_min);
            let max = std::simd::num::SimdFloat::simd_max(a_max, b_max);
            let mask = std::simd::Mask::<i32, 4>::from_array([true, true, true, false]);

            // Use load_select to blend the results while preserving the W component
            let min_ptr = core::ptr::addr_of!(min) as *const f32;
            let max_ptr = core::ptr::addr_of!(max) as *const f32;
            let result_min = Simd::load_select_ptr(min_ptr, mask, current_min);
            let result_max = Simd::load_select_ptr(max_ptr, mask, current_max);

            // Store results
            *(result_ptr as *mut [f32; 4]) = result_min.to_array();
            *(result_ptr.add(4) as *mut [f32; 4]) = result_max.to_array();
        }
    }

    /// Merges two structures with memory layouts equivalent to the `BoundingBox`.
    /// The referenced values must not be in unpinned managed memory.
    /// Any data in the empty slots is not preserved.
    #[inline(always)]
    pub unsafe fn create_merged_unsafe<TA, TB>(
        bounding_box_a: &TA,
        bounding_box_b: &TB,
        merged: &mut MaybeUninit<TA>,
    ) where
        TA: Sized + Copy,
        TB: Sized + Copy,
    {
        debug_assert!(std::mem::size_of::<TA>() == 32 && std::mem::size_of::<TB>() == 32);

        let a_ptr = bounding_box_a as *const TA as *const f32;
        let b_ptr = bounding_box_b as *const TB as *const f32;
        let result_ptr = merged.as_mut_ptr() as *mut f32;

        let a_min = Simd::<f32, 4>::from_array(*(a_ptr as *const [f32; 4]));
        let a_max = Simd::<f32, 4>::from_array(*(a_ptr.add(4) as *const [f32; 4]));
        let b_min = Simd::<f32, 4>::from_array(*(b_ptr as *const [f32; 4]));
        let b_max = Simd::<f32, 4>::from_array(*(b_ptr.add(4) as *const [f32; 4]));

        let min = std::simd::num::SimdFloat::simd_min(a_min, b_min);
        let max = std::simd::num::SimdFloat::simd_max(a_max, b_max);

        // Store results
        *(result_ptr as *mut [f32; 4]) = min.to_array();
        *(result_ptr.add(4) as *mut [f32; 4]) = max.to_array();
    }

    /// Determines if a bounding box intersects a bounding sphere.
    #[inline(always)]
    pub fn intersects_sphere(&self, sphere: &BoundingSphere) -> bool {
        let offset = sphere.center - self.min.max(sphere.center).min(self.max);
        offset.dot(offset) <= sphere.radius * sphere.radius
    }

    #[inline(always)]
    pub fn contains(&self, bounding_box: &BoundingBox) -> ContainmentType {
        if self.max.x < bounding_box.min.x
            || self.min.x > bounding_box.max.x
            || self.max.y < bounding_box.min.y
            || self.min.y > bounding_box.max.y
            || self.max.z < bounding_box.min.z
            || self.min.z > bounding_box.max.z
        {
            ContainmentType::Disjoint
        } else if self.min.x <= bounding_box.min.x
            && self.max.x >= bounding_box.max.x
            && self.min.y <= bounding_box.min.y
            && self.max.y >= bounding_box.max.y
            && self.min.z <= bounding_box.min.z
            && self.max.z >= bounding_box.max.z
        {
            ContainmentType::Contains
        } else {
            ContainmentType::Intersects
        }
    }

    /// Creates the smallest possible bounding box that contains a list of points.
    pub fn create_from_points(points: &[Vec3]) -> Self {
        if points.is_empty() {
            panic!("Cannot construct a bounding box from an empty list.");
        }
        let aabb = BoundingBox {
            min: points[0],
            max: points[0],
        };
        for point in points.iter().skip(1) {
            aabb.min = min.min(*point);
            aabb.max = max.max(*point);
        }
        aabb
    }

    /// Creates a bounding box from a bounding sphere.
    #[inline(always)]
    pub fn create_from_sphere(bounding_sphere: &BoundingSphere, bounding_box: &mut Self) {
        let radius = Vec3::splat(sphere.radius);
        bounding_box.min = sphere.center - radius;
        bounding_box.max = sphere.center + radius;
    }

    /// Creates a string representation of the bounding box.
    pub fn to_string(&self) -> String {
        format!("({}, {})", self.min, self.max)
    }
}
