use crate::utilities::vector::Vector;
use std::simd::{cmp::SimdPartialOrd, Simd};

pub const VECTOR_MASK: usize = Vector::<f32>::LEN - 1;

/// Some helpers for indexing into vector bundles.
pub struct BundleIndexing;

impl BundleIndexing {
    /// Gets the mask value such that x & VECTOR_MASK computes x % Vector<f32>::LEN.
    #[inline(always)]
    pub const fn vector_mask() -> usize {
        VECTOR_MASK
    }

    /// Gets the shift value such that x >> VECTOR_SHIFT divides x by Vector<f32>::LEN.
    #[inline(always)]
    pub const fn vector_shift() -> usize {
        match Vector::<f32>::LEN {
            4 => 2,
            8 => 3,
            16 => 4,
            _ => 0,
        }
    }

    #[inline(always)]
    pub fn get_bundle_indices(
        linear_index: usize,
        bundle_index: &mut usize,
        index_in_bundle: &mut usize,
    ) {
        *bundle_index = linear_index >> Self::vector_shift();
        *index_in_bundle = linear_index & Self::vector_mask();
    }

    #[inline(always)]
    pub fn get_bundle_count(element_count: usize) -> usize {
        (element_count + Self::vector_mask()) >> Self::vector_shift()
    }

    #[inline(always)]
    pub fn create_trailing_mask_for_count_in_bundle(count_in_bundle: usize) -> Vector<i32> {
        let count = Vector::splat(count_in_bundle as f32);
        let indices = Vector::from_slice(
            &(0..Vector::<f32>::LEN)
                .map(|x| x as f32)
                .collect::<Vec<_>>(),
        );
        let mask = count.simd_le(indices);
        // Convert bool mask to -1/0 integers
        Simd::from_array(unsafe {
            std::array::from_fn(|i| if mask.test_unchecked(i) { -1 } else { 0 })
        })
    }

    #[inline(always)]
    pub fn create_mask_for_count_in_bundle(count_in_bundle: usize) -> Vector<i32> {
        let count = Vector::splat(count_in_bundle as f32);
        let indices = Vector::from_slice(
            &(0..Vector::<f32>::LEN)
                .map(|x| x as f32)
                .collect::<Vec<_>>(),
        );
        let mask = count.simd_gt(indices);
        // Convert bool mask to -1/0 integers
        Simd::from_array(unsafe {
            std::array::from_fn(|i| if mask.test_unchecked(i) { -1 } else { 0 })
        })
    }

    #[inline(always)]
    pub fn get_first_set_lane_index(v: Vector<i32>) -> i32 {
        // There's no guarantee that std::simd::Simd will use the move mask instruction,
        // so we use x86 intrinsics directly here.
        #[cfg(target_arch = "x86_64")]
        unsafe {
            if is_x86_feature_detected!("avx2") && Vector::<i32>::LEN == 8 {
                let float_vec = std::mem::transmute::<_, __m256>(v);
                let scalar_mask = _mm256_movemask_ps(float_vec);
                scalar_mask.trailing_zeros() as i32
            } else if is_x86_feature_detected!("sse4.1") && Vector::<i32>::LEN == 4 {
                let float_vec = std::mem::transmute::<_, __m128>(v);
                let scalar_mask = _mm_movemask_ps(float_vec);
                scalar_mask.trailing_zeros() as i32
            } else {
                Self::fallback_get_first_set_lane_index(v)
            }
        }
        // TODO: ARM support
        #[cfg(not(target_arch = "x86_64"))]
        {
            let mask = std::simd::cmp::SimdPartialEq::simd_eq(v, Vector::splat(-1));
            let bits = mask.to_bitmask();
            if bits == 0 {
                -1
            } else {
                bits.trailing_zeros() as i32
            }
        }
    }

    #[inline(always)]
    fn fallback_get_first_set_lane_index(v: Vector<i32>) -> i32 {
        for i in 0..Vector::<i32>::LEN {
            if v[i] == -1 {
                return i as i32;
            }
        }
        -1
    }

    /// Gets the number of lanes that occur at or before the last set lane.
    /// In other words, if the lanes in the vector are (-1, 0, -1, 0), then this will return 3.
    #[inline(always)]
    pub fn get_last_set_lane_count(v: Vector<i32>) -> usize {
        // There's no guarantee that std::simd::Simd will use the move mask instruction,
        // so we use x86 intrinsics directly here.
        #[cfg(target_arch = "x86_64")]
        unsafe {
            if is_x86_feature_detected!("avx2") && Vector::<i32>::LEN == 8 {
                let float_vec = std::mem::transmute::<_, __m256>(v);
                let scalar_mask = _mm256_movemask_ps(float_vec);
                32 - scalar_mask.leading_zeros() as usize
            } else if is_x86_feature_detected!("sse4.1") && Vector::<i32>::LEN == 4 {
                let float_vec = std::mem::transmute::<_, __m128>(v);
                let scalar_mask = _mm_movemask_ps(float_vec);
                32 - scalar_mask.leading_zeros() as usize
            } else {
                Self::fallback_get_last_set_lane_count(v)
            }
        }
        // TODO: ARM support
        #[cfg(not(target_arch = "x86_64"))]
        {
            let mask = std::simd::cmp::SimdPartialEq::simd_eq(v, Vector::splat(-1));
            let bits = mask.to_bitmask();
            if bits == 0 {
                0
            } else {
                (u32::BITS - bits.leading_zeros()) as usize
            }
        }
    }

    #[inline(always)]
    fn fallback_get_last_set_lane_count(v: Vector<i32>) -> usize {
        for i in (0..Vector::<i32>::LEN).rev() {
            if v[i] == -1 {
                return i + 1;
            }
        }
        0
    }
}
