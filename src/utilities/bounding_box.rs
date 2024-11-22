use crate::utilities::bounding_sphere::BoundingSphere;
use glam::Vec4Swizzles;
use glam::{Vec3, Vec4};
use std::mem::MaybeUninit;

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::{
    __m128, _mm_and_ps, _mm_andnot_ps, _mm_blend_ps, _mm_cmplt_ps, _mm_load_ps, _mm_max_ps,
    _mm_min_ps, _mm_movemask_ps, _mm_or_ps,
};

#[cfg(target_arch = "aarch64")]
use std::arch::aarch64::{
    float32x4_t, vbslq_f32, vcltq_f32, vgetq_lane_u32, vld1q_f32, vmaxq_f32, vminq_f32, vorrq_u32,
    vst1q_f32,
};

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
#[derive(Default, Clone, Copy, Debug)]
pub struct BoundingBox {
    /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub min: Vec3,
    /// Padding to ensure 16-byte alignment for max
    _padding: [u8; 4],
    /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub max: Vec3,
    /// Additional padding to ensure total size of 32 bytes
    _padding2: [u8; 4],
}

impl BoundingBox {
    /// Constructs a bounding box from the specified minimum and maximum.
    #[inline(always)]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self {
            min,
            max,
            ..Default::default()
        }
    }

    /// Checks if two structures with memory layouts equivalent to the `BoundingBox` intersect.
    /// The referenced values must not be in unpinned managed memory.
    #[inline(always)]
    pub unsafe fn intersects_unsafe<TA, TB>(bounding_box_a: &TA, bounding_box_b: &TB) -> bool
    where
        TA: Sized + Copy,
        TB: Sized + Copy,
    {
        debug_assert_eq!(size_of::<TA>(), 32);
        debug_assert_eq!(size_of::<TB>(), 32);

        let a = (bounding_box_a as *const TA).cast::<f32>();
        let b = (bounding_box_b as *const TB).cast::<f32>();

        #[cfg(target_arch = "x86_64")]
        {
            if is_x86_feature_detected!("sse") {
                let a_min: __m128 = _mm_load_ps(a);
                let a_max: __m128 = _mm_load_ps(a.add(4));
                let b_min: __m128 = _mm_load_ps(b);
                let b_max: __m128 = _mm_load_ps(b.add(4));

                let no_intersection_on_axes =
                    _mm_or_ps(_mm_cmplt_ps(a_max, b_min), _mm_cmplt_ps(b_max, a_min));
                return (_mm_movemask_ps(no_intersection_on_axes) & 0b111) == 0;
            }
        }

        #[cfg(target_arch = "aarch64")]
        {
            if std::arch::is_aarch64_feature_detected!("neon") {
                let a_min: float32x4_t = vld1q_f32(a);
                let a_max: float32x4_t = vld1q_f32(a.add(4));
                let b_min: float32x4_t = vld1q_f32(b);
                let b_max: float32x4_t = vld1q_f32(b.add(4));

                let no_intersection_on_axes = vorrq_u32(
                    vcltq_f32(a_max, b_min).into_bits(),
                    vcltq_f32(b_max, a_min).into_bits(),
                );
                let mask = vgetq_lane_u32(no_intersection_on_axes, 0)
                    | vgetq_lane_u32(no_intersection_on_axes, 1)
                    | vgetq_lane_u32(no_intersection_on_axes, 2);
                return mask == 0;
            }
        }

        // Fallback scalar implementation for all architectures
        *a.add(4) >= *b
            && *a.add(5) >= *b.add(1)
            && *a.add(6) >= *b.add(2)
            && *b.add(4) >= *a
            && *b.add(5) >= *a.add(1)
            && *b.add(6) >= *a.add(2)
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline(always)]
    pub fn intersects(a: Self, b: Self) -> bool {
        unsafe { Self::intersects_unsafe(&a, &b) }
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline(always)]
    pub fn intersects_bounds(min_a: Vec3, max_a: Vec3, min_b: Vec3, max_b: Vec3) -> bool {
        #[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
        {
            unsafe {
                #[cfg(target_arch = "x86_64")]
                {
                    if is_x86_feature_detected!("sse") {
                        let max_a: __m128 = _mm_load_ps(&max_a as *const Vec3 as *const f32);
                        let min_b: __m128 = _mm_load_ps(&min_b as *const Vec3 as *const f32);
                        let max_b: __m128 = _mm_load_ps(&max_b as *const Vec3 as *const f32);
                        let min_a: __m128 = _mm_load_ps(&min_a as *const Vec3 as *const f32);

                        let no_intersection_on_axes =
                            _mm_or_ps(_mm_cmplt_ps(max_a, min_b), _mm_cmplt_ps(max_b, min_a));
                        return (_mm_movemask_ps(no_intersection_on_axes) & 0b111) == 0;
                    }
                }

                #[cfg(target_arch = "aarch64")]
                {
                    if std::arch::is_aarch64_feature_detected!("neon") {
                        let max_a: float32x4_t = vld1q_f32(&max_a as *const Vec3 as *const f32);
                        let min_b: float32x4_t = vld1q_f32(&min_b as *const Vec3 as *const f32);
                        let max_b: float32x4_t = vld1q_f32(&max_b as *const Vec3 as *const f32);
                        let min_a: float32x4_t = vld1q_f32(&min_a as *const Vec3 as *const f32);

                        let no_intersection_on_axes: uint32x4_t = vorrq_u32(
                            vcltq_f32(max_a, min_b).into_bits(),
                            vcltq_f32(max_b, min_a).into_bits(),
                        );
                        let mask = vgetq_lane_u32(no_intersection_on_axes, 0)
                            | vgetq_lane_u32(no_intersection_on_axes, 1)
                            | vgetq_lane_u32(no_intersection_on_axes, 2);
                        return mask == 0;
                    }
                }
            }
        }

        // Fallback scalar implementation for all architectures
        max_a.x >= min_b.x
            && max_a.y >= min_b.y
            && max_a.z >= min_b.z
            && max_b.x >= min_a.x
            && max_b.y >= min_a.y
            && max_b.z >= min_a.z
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
        debug_assert_eq!(std::mem::size_of::<TA>(), 32);
        debug_assert_eq!(std::mem::size_of::<TB>(), 32);

        #[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
        {
            #[cfg(target_arch = "x86_64")]
            if is_x86_feature_detected!("sse4.1") {
                let a_min: __m128 = _mm_load_ps(bounding_box_a as *const TA as *const f32);
                let a_max: __m128 = _mm_load_ps((bounding_box_a as *const TA as *const f32).add(4));
                let b_min: __m128 = _mm_load_ps(bounding_box_b as *const TB as *const f32);
                let b_max: __m128 = _mm_load_ps((bounding_box_b as *const TB as *const f32).add(4));

                let min = _mm_min_ps(a_min, b_min);
                let max = _mm_max_ps(a_max, b_max);

                let result_ptr = merged.as_mut_ptr() as *mut __m128;
                let mask = _mm_set_ps(0.0, -1.0, -1.0, -1.0);

                // TODO: IF NOT IN SSE4.1, NEED TO USE CONDITIONA SELECT. CHECK BoundingBox.cs:215
                _mm_store_ps(
                    result_ptr as *mut f32,
                    _mm_blend_ps(min, _mm_load_ps(result_ptr as *const f32), 0b1000),
                );
                _mm_store_ps(
                    result_ptr.add(1) as *mut f32,
                    _mm_blend_ps(max, _mm_load_ps(result_ptr.add(1) as *const f32), 0b1000),
                );

                return;
            }

            #[cfg(target_arch = "aarch64")]
            if std::arch::is_aarch64_feature_detected!("neon") {
                let a_min: float32x4_t = vld1q_f32(bounding_box_a as *const TA as *const f32);
                let a_max: float32x4_t =
                    vld1q_f32((bounding_box_a as *const TA as *const f32).add(4));
                let b_min: float32x4_t = vld1q_f32(bounding_box_b as *const TB as *const f32);
                let b_max: float32x4_t =
                    vld1q_f32((bounding_box_b as *const TB as *const f32).add(4));

                let min = vminq_f32(a_min, b_min);
                let max = vmaxq_f32(a_max, b_max);

                let result_ptr = merged.as_mut_ptr() as *mut float32x4_t;
                let mask =
                    vld1q_f32(&[f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY, 0.0]);

                vst1q_f32(
                    result_ptr as *mut f32,
                    vbslq_f32(
                        mask.to_int().into_bits(),
                        min,
                        vld1q_f32(result_ptr as *const f32),
                    ),
                );
                vst1q_f32(
                    (result_ptr as *mut f32).add(4),
                    vbslq_f32(
                        mask.to_int().into_bits(),
                        max,
                        vld1q_f32((result_ptr as *const f32).add(4)),
                    ),
                );

                return;
            }
        }

        // Fallback scalar implementation
        let a = &*(bounding_box_a as *const TA as *const BoundingBox);
        let b = &*(bounding_box_b as *const TB as *const BoundingBox);
        let result = &mut *(merged.as_mut_ptr() as *mut BoundingBox);

        result.min = Vec3::min(a.min, b.min);
        result.max = Vec3::max(a.max, b.max);
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
        debug_assert_eq!(std::mem::size_of::<TA>(), 32);
        debug_assert_eq!(std::mem::size_of::<TB>(), 32);

        #[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
        {
            #[cfg(target_arch = "x86_64")]
            if is_x86_feature_detected!("sse") {
                let a_min: __m128 = _mm_load_ps(bounding_box_a as *const TA as *const f32);
                let a_max: __m128 = _mm_load_ps((bounding_box_a as *const TA as *const f32).add(4));
                let b_min: __m128 = _mm_load_ps(bounding_box_b as *const TB as *const f32);
                let b_max: __m128 = _mm_load_ps((bounding_box_b as *const TB as *const f32).add(4));

                let result_min = _mm_min_ps(a_min, b_min);
                let result_max = _mm_max_ps(a_max, b_max);

                let result_ptr = merged.as_mut_ptr() as *mut __m128;
                _mm_store_ps(result_ptr as *mut f32, result_min);
                _mm_store_ps(result_ptr.add(1) as *mut f32, result_max);

                return;
            }

            #[cfg(target_arch = "aarch64")]
            if std::arch::is_aarch64_feature_detected!("neon") {
                let a_min: float32x4_t = vld1q_f32(bounding_box_a as *const TA as *const f32);
                let a_max: float32x4_t =
                    vld1q_f32((bounding_box_a as *const TA as *const f32).add(4));
                let b_min: float32x4_t = vld1q_f32(bounding_box_b as *const TB as *const f32);
                let b_max: float32x4_t =
                    vld1q_f32((bounding_box_b as *const TB as *const f32).add(4));

                let result_min = vminq_f32(a_min, b_min);
                let result_max = vmaxq_f32(a_max, b_max);

                let result_ptr = merged.as_mut_ptr() as *mut float32x4_t;
                vst1q_f32(result_ptr as *mut f32, result_min);
                vst1q_f32((result_ptr as *mut f32).add(4), result_max);

                return;
            }
        }

        // Fallback scalar implementation
        let a = &*(bounding_box_a as *const TA as *const BoundingBox);
        let b = &*(bounding_box_b as *const TB as *const BoundingBox);
        let result = &mut *(merged.as_mut_ptr() as *mut BoundingBox);
        result.min = Vec3::min(a.min, b.min);
        result.max = Vec3::max(a.max, b.max);
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
            ..Default::default()
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
