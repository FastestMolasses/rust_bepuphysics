use std::arch::aarch64::{float32x4_t, uint32x4_t};
use std::arch::aarch64::{vcgtq_f32, vcleq_f32, vdupq_n_f32, vminvq_u32};
use std::arch::x86_64::{__m128, _mm_cmp_ps, _mm_cmp_ps_mask, _mm_movemask_ps};
use std::arch::x86_64::{__m256, _mm256_cmp_ps, _mm256_cmp_ps_mask, _mm256_movemask_ps};
use std::mem;
use std::simd::{f32x4, f32x8, u32x4, u32x8, Mask};

/// Some helpers for indexing into vector bundles.
pub struct BundleIndexing;

impl BundleIndexing {
    /// Gets the mask value such that x & VECTOR_MASK computes x % f32x8::LANES.
    ///
    /// The compiler recognizes that this value is constant!
    #[inline(always)]
    pub const fn vector_mask() -> usize {
        f32x8::LANES - 1
    }

    /// Gets the shift value such that x >> VECTOR_SHIFT divides x by f32x8::LANES.
    ///
    /// The compiler recognizes that this value is constant!
    #[inline(always)]
    pub const fn vector_shift() -> usize {
        match f32x8::LANES {
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
    pub fn create_trailing_mask_for_count_in_bundle(count_in_bundle: usize) -> u32x8 {
        if is_x86_feature_detected!("avx") && f32x8::LANES == 8 {
            unsafe {
                let cmp = _mm256_cmp_ps(
                    _mm256_set1_ps(count_in_bundle as f32),
                    _mm256_set_ps(7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0),
                    _CMP_LE_OQ,
                );
                let mask = _mm256_movemask_ps(cmp);
                mem::transmute::<i32, __m256>(mask as i32)
            }
        } else if is_x86_feature_detected!("sse") && f32x4::LANES == 4 {
            unsafe {
                let cmp = _mm_cmp_ps(
                    _mm_set1_ps(count_in_bundle as f32),
                    _mm_set_ps(3.0, 2.0, 1.0, 0.0),
                    _CMP_LE_OQ,
                );
                let mask = _mm_movemask_ps(cmp);
                mem::transmute::<i32, __m128>(mask as i32)
            }
        } else if is_aarch64_feature_detected!("neon") && f32x4::LANES == 4 {
            unsafe {
                let cmp = vcleq_f32(
                    vdupq_n_f32(count_in_bundle as f32),
                    float32x4_t(3.0, 2.0, 1.0, 0.0),
                );
                mem::transmute::<uint32x4_t, u32x4>(vminvq_u32(cmp))
            }
        } else {
            let mut mask = [0; f32x8::LANES];
            for i in 0..f32x8::LANES {
                mask[i] = if count_in_bundle <= i { !0 } else { 0 };
            }
            u32x8::from_array(mask)
        }
    }

    #[inline(always)]
    pub fn create_mask_for_count_in_bundle(count_in_bundle: usize) -> u32x8 {
        if is_x86_feature_detected!("avx") && f32x8::LANES == 8 {
            unsafe {
                let cmp = _mm256_cmp_ps(
                    _mm256_set1_ps(count_in_bundle as f32),
                    _mm256_set_ps(7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0),
                    _CMP_GT_OQ,
                );
                let mask = _mm256_movemask_ps(cmp);
                mem::transmute::<i32, __m256>(mask as i32)
            }
        } else if is_x86_feature_detected!("sse") && f32x4::LANES == 4 {
            unsafe {
                let cmp = _mm_cmp_ps(
                    _mm_set1_ps(count_in_bundle as f32),
                    _mm_set_ps(3.0, 2.0, 1.0, 0.0),
                    _CMP_GT_OQ,
                );
                let mask = _mm_movemask_ps(cmp);
                mem::transmute::<i32, __m128>(mask as i32)
            }
        } else if is_aarch64_feature_detected!("neon") && f32x4::LANES == 4 {
            unsafe {
                let cmp = vcgtq_f32(
                    vdupq_n_f32(count_in_bundle as f32),
                    float32x4_t(3.0, 2.0, 1.0, 0.0),
                );
                mem::transmute::<uint32x4_t, u32x4>(vminvq_u32(cmp))
            }
        } else {
            let mut mask = [0; f32x8::LANES];
            for i in 0..f32x8::LANES {
                mask[i] = if count_in_bundle > i { !0 } else { 0 };
            }
            u32x8::from_array(mask)
        }
    }

    #[inline(always)]
    pub fn get_first_set_lane_index(v: u32x8) -> i32 {
        if is_x86_feature_detected!("avx") && f32x8::LANES == 8 {
            unsafe {
                let scalar_mask = _mm256_movemask_ps(mem::transmute::<__m256, __m256>(v));
                scalar_mask.trailing_zeros() as i32
            }
        } else if is_x86_feature_detected!("sse") && f32x4::LANES == 4 {
            unsafe {
                let scalar_mask = _mm_movemask_ps(mem::transmute::<__m128, __m128>(v));
                scalar_mask.trailing_zeros() as i32
            }
        } else if is_aarch64_feature_detected!("neon") && f32x4::LANES == 4 {
            let mask: Mask<i32, 4> = v.lanes_le(f32x4::splat(0.0)).into();
            mask.trailing_zeros() as i32
        } else {
            for i in 0..f32x8::LANES {
                if v[i] == !0 {
                    return i as i32;
                }
            }
            -1
        }
    }

    /// Gets the number of lanes that occur at or before the last set lane.
    /// In other words, if the lanes in the vector are (-1, 0, -1, 0), then this will return 3.
    #[inline(always)]
    pub fn get_last_set_lane_count(v: u32x8) -> usize {
        if is_x86_feature_detected!("avx") && f32x8::LANES == 8 {
            unsafe {
                let scalar_mask = _mm256_movemask_ps(mem::transmute::<__m256, __m256>(v));
                32 - scalar_mask.leading_zeros() as usize
            }
        } else if is_x86_feature_detected!("sse") && f32x4::LANES == 4 {
            unsafe {
                let scalar_mask = _mm_movemask_ps(mem::transmute::<__m128, __m128>(v));
                32 - scalar_mask.leading_zeros() as usize
            }
        } else if is_aarch64_feature_detected!("neon") && f32x4::LANES == 4 {
            let mask: Mask<i32, 4> = v.lanes_le(f32x4::splat(0.0)).into();
            32 - mask.bitmask().leading_zeros() as usize
        } else {
            for i in (0..f32x8::LANES).rev() {
                if v[i] == !0 {
                    return i + 1;
                }
            }
            0
        }
    }
}
