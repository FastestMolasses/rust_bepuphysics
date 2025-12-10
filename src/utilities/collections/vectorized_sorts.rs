//! Vectorized sorting utilities using SIMD intrinsics.
//! 
//! This module provides architecture-specific optimized implementations:
//! - x86/x86_64: AVX2 (8-wide) and SSE2 (4-wide) implementations
//! - aarch64: NEON (4-wide) implementation
//! 
//! The counting sort computes the mapping from current index to ascending sorted index
//! for all elements. The keys buffer is not actually sorted - only the target indices are computed.

use std::cmp::min;

/// Scalar fallback implementation of counting sort.
/// This is used when no SIMD is available or for debugging.
#[inline(never)]
pub fn vector_counting_sort_scalar(
    padded_keys: &[f32],
    padded_target_indices: &mut [i32],
    element_count: usize,
) {
    for i in 0..element_count {
        let mut count = 0i32;
        let value = padded_keys[i];
        
        // Count elements before i that are <= value
        for j in 0..i {
            if padded_keys[j] <= value {
                count += 1;
            }
        }
        
        // Count elements after i that are < value (strict less than for stability)
        for j in (i + 1)..element_count {
            if padded_keys[j] < value {
                count += 1;
            }
        }
        
        padded_target_indices[i] = count;
    }
}

// ============================================================================
// x86/x86_64 Implementation (AVX2 and SSE2)
// ============================================================================

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
mod x86_impl {
    #[cfg(target_arch = "x86")]
    use std::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use std::arch::x86_64::*;
    use std::cmp::min;

    /// Performs a vectorized counting sort using AVX2 instructions (8-wide SIMD).
    /// 
    /// # Safety
    /// - Requires AVX2 support
    /// - `padded_keys` and `padded_target_indices` must have the same length
    /// - Length must be a multiple of 8
    /// - Length must be >= element_count
    /// - Buffers must be 32-byte aligned for optimal performance
    #[target_feature(enable = "avx2")]
    #[inline]
    pub unsafe fn vector_counting_sort_avx2(
        padded_keys: &[f32],
        padded_target_indices: &mut [i32],
        element_count: usize,
    ) {
        debug_assert!(
            padded_keys.len() == padded_target_indices.len()
                && padded_keys.len() % 8 == 0
                && padded_keys.len() >= element_count,
            "Preconditions not met: keys.len={}, indices.len={}, element_count={}",
            padded_keys.len(), padded_target_indices.len(), element_count
        );
        
        let index_offsets = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);
        let one_mask = _mm256_set1_epi32(1);
        let keys_ptr = padded_keys.as_ptr();
        let indices_ptr = padded_target_indices.as_mut_ptr();
        
        let mut i = 0;
        while i < element_count {
            // Load 8 values at once
            let values = _mm256_loadu_ps(keys_ptr.add(i));
            let mut counts = _mm256_setzero_si256();

            // Phase 1: Test all elements before current bundle (j < i)
            // These elements definitely have lower indices, so we only check value <= bundle_value
            for j in 0..i {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm256_set1_ps(test_value);
                // slot_precedes_value = test_value <= values (element j should come before if smaller or equal)
                let slot_precedes_value = _mm256_castps_si256(_mm256_cmp_ps::<_CMP_LE_OS>(test_vector, values));
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            // Phase 2: Test elements within the current bundle (i <= j < i+8)
            // Need to check both value AND index for tie-breaking
            let end_of_equality_testing = min(i + 8, element_count);
            let slot_index = _mm256_add_epi32(_mm256_set1_epi32(i as i32), index_offsets);
            for j in i..end_of_equality_testing {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm256_set1_ps(test_value);
                let j_vector = _mm256_set1_epi32(j as i32);
                
                // slot_is_lesser = test_value < values
                let slot_is_lesser = _mm256_castps_si256(_mm256_cmp_ps::<_CMP_LT_OS>(test_vector, values));
                // slot_index_is_lesser = j < slot_index (for tie-breaking equal values)
                let slot_index_is_lesser = _mm256_cmpgt_epi32(slot_index, j_vector);
                // slot_is_equal = test_value == values
                let slot_is_equal = _mm256_castps_si256(_mm256_cmp_ps::<_CMP_EQ_OS>(test_vector, values));
                // slot_precedes_value = lesser OR (equal AND index_lesser)
                let slot_precedes_value = _mm256_or_si256(
                    slot_is_lesser,
                    _mm256_and_si256(slot_is_equal, slot_index_is_lesser),
                );
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            // Phase 3: Test elements after current bundle (j >= i+8)
            // These elements have higher indices, so for tie-breaking we use strict less than
            for j in end_of_equality_testing..element_count {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm256_set1_ps(test_value);
                // slot_precedes_value = test_value < values (strict, since j > all indices in bundle)
                let slot_precedes_value = _mm256_castps_si256(_mm256_cmp_ps::<_CMP_LT_OS>(test_vector, values));
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            _mm256_storeu_si256(indices_ptr.add(i) as *mut __m256i, counts);
            i += 8;
        }
    }

    /// Performs a vectorized counting sort using SSE2/SSE instructions (4-wide SIMD).
    /// 
    /// # Safety
    /// - Requires SSE2 support
    /// - `padded_keys` and `padded_target_indices` must have the same length
    /// - Length must be a multiple of 4
    /// - Length must be >= element_count
    #[target_feature(enable = "sse2", enable = "sse")]
    #[inline]
    pub unsafe fn vector_counting_sort_sse2(
        padded_keys: &[f32],
        padded_target_indices: &mut [i32],
        element_count: usize,
    ) {
        debug_assert!(
            padded_keys.len() == padded_target_indices.len()
                && padded_keys.len() % 4 == 0
                && padded_keys.len() >= element_count,
            "Preconditions not met."
        );
        
        let index_offsets = _mm_setr_epi32(0, 1, 2, 3);
        let one_mask = _mm_set1_epi32(1);
        let keys_ptr = padded_keys.as_ptr();
        let indices_ptr = padded_target_indices.as_mut_ptr();
        
        let mut i = 0;
        while i < element_count {
            let values = _mm_loadu_ps(keys_ptr.add(i));
            let mut counts = _mm_setzero_si128();

            // Phase 1: Elements before bundle
            for j in 0..i {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm_set1_ps(test_value);
                let slot_precedes_value = _mm_castps_si128(_mm_cmple_ps(test_vector, values));
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            // Phase 2: Elements within bundle (need tie-breaking)
            let end_of_equality_testing = min(i + 4, element_count);
            let slot_index = _mm_add_epi32(_mm_set1_epi32(i as i32), index_offsets);
            for j in i..end_of_equality_testing {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm_set1_ps(test_value);
                let j_vector = _mm_set1_epi32(j as i32);
                
                let slot_is_lesser = _mm_castps_si128(_mm_cmplt_ps(test_vector, values));
                let slot_index_is_lesser = _mm_cmplt_epi32(j_vector, slot_index);
                let slot_is_equal = _mm_castps_si128(_mm_cmpeq_ps(test_vector, values));
                let slot_precedes_value = _mm_or_si128(
                    slot_is_lesser,
                    _mm_and_si128(slot_is_equal, slot_index_is_lesser),
                );
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            // Phase 3: Elements after bundle
            for j in end_of_equality_testing..element_count {
                let test_value = *keys_ptr.add(j);
                let test_vector = _mm_set1_ps(test_value);
                let slot_precedes_value = _mm_castps_si128(_mm_cmplt_ps(test_vector, values));
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            _mm_storeu_si128(indices_ptr.add(i) as *mut __m128i, counts);
            i += 4;
        }
    }

    /// Performs a vectorized counting sort, automatically selecting the best available instruction set.
    /// 
    /// # Safety
    /// - Buffer alignment and size requirements depend on the selected implementation
    /// - AVX2: length must be multiple of 8
    /// - SSE2: length must be multiple of 4
    pub unsafe fn vector_counting_sort(
        padded_keys: &[f32],
        padded_target_indices: &mut [i32],
        element_count: usize,
    ) {
        if is_x86_feature_detected!("avx2") {
            vector_counting_sort_avx2(padded_keys, padded_target_indices, element_count);
        } else if is_x86_feature_detected!("sse2") {
            vector_counting_sort_sse2(padded_keys, padded_target_indices, element_count);
        } else {
            super::vector_counting_sort_scalar(padded_keys, padded_target_indices, element_count);
        }
    }
}

// ============================================================================
// ARM aarch64 Implementation (NEON)
// ============================================================================

#[cfg(target_arch = "aarch64")]
mod arm_impl {
    use std::arch::aarch64::*;
    use std::cmp::min;

    /// Performs a vectorized counting sort using ARM NEON instructions (4-wide SIMD).
    /// 
    /// NEON provides 128-bit vector registers, allowing us to process 4 f32 values at once.
    /// This is equivalent in width to SSE on x86.
    /// 
    /// # Safety
    /// - `padded_keys` and `padded_target_indices` must have the same length
    /// - Length must be a multiple of 4
    /// - Length must be >= element_count
    #[inline]
    pub unsafe fn vector_counting_sort_neon(
        padded_keys: &[f32],
        padded_target_indices: &mut [i32],
        element_count: usize,
    ) {
        debug_assert!(
            padded_keys.len() == padded_target_indices.len()
                && padded_keys.len() % 4 == 0
                && padded_keys.len() >= element_count,
            "Preconditions not met: keys.len={}, indices.len={}, element_count={}",
            padded_keys.len(), padded_target_indices.len(), element_count
        );
        
        // Index offsets for the 4 lanes: [0, 1, 2, 3]
        let index_offsets: int32x4_t = vld1q_s32([0i32, 1, 2, 3].as_ptr());
        // Mask of all 1s for extracting comparison results
        let one_mask: int32x4_t = vdupq_n_s32(1);
        
        let keys_ptr = padded_keys.as_ptr();
        let indices_ptr = padded_target_indices.as_mut_ptr();
        
        let mut i = 0;
        while i < element_count {
            // Load 4 float values
            let values: float32x4_t = vld1q_f32(keys_ptr.add(i));
            let mut counts: int32x4_t = vdupq_n_s32(0);

            // Phase 1: Test all elements before current bundle (j < i)
            // These elements definitely have lower indices, so we only check value <= bundle_value
            for j in 0..i {
                let test_value = *keys_ptr.add(j);
                let test_vector: float32x4_t = vdupq_n_f32(test_value);
                
                // vcleq_f32: compare less than or equal (test_value <= values)
                // Returns all 1s (0xFFFFFFFF) for true lanes, 0 for false
                let slot_precedes_value: uint32x4_t = vcleq_f32(test_vector, values);
                
                // AND with one_mask to convert 0xFFFFFFFF -> 1, then add to counts
                let increment: int32x4_t = vandq_s32(vreinterpretq_s32_u32(slot_precedes_value), one_mask);
                counts = vaddq_s32(counts, increment);
            }

            // Phase 2: Test elements within the current bundle (i <= j < i+4)
            // Need to check both value AND index for tie-breaking
            let end_of_equality_testing = min(i + 4, element_count);
            let slot_index: int32x4_t = vaddq_s32(vdupq_n_s32(i as i32), index_offsets);
            
            for j in i..end_of_equality_testing {
                let test_value = *keys_ptr.add(j);
                let test_vector: float32x4_t = vdupq_n_f32(test_value);
                let j_vector: int32x4_t = vdupq_n_s32(j as i32);
                
                // vcltq_f32: compare less than (test_value < values)
                let slot_is_lesser: uint32x4_t = vcltq_f32(test_vector, values);
                
                // vcltq_s32: compare j < slot_index (for tie-breaking)
                let slot_index_is_lesser: uint32x4_t = vcltq_s32(j_vector, slot_index);
                
                // vceqq_f32: compare equal (test_value == values)
                let slot_is_equal: uint32x4_t = vceqq_f32(test_vector, values);
                
                // slot_precedes_value = lesser OR (equal AND index_lesser)
                let equal_and_index_lesser: uint32x4_t = vandq_u32(slot_is_equal, slot_index_is_lesser);
                let slot_precedes_value: uint32x4_t = vorrq_u32(slot_is_lesser, equal_and_index_lesser);
                
                let increment: int32x4_t = vandq_s32(vreinterpretq_s32_u32(slot_precedes_value), one_mask);
                counts = vaddq_s32(counts, increment);
            }

            // Phase 3: Test elements after current bundle (j >= i+4)
            // These elements have higher indices, so for tie-breaking we use strict less than
            for j in end_of_equality_testing..element_count {
                let test_value = *keys_ptr.add(j);
                let test_vector: float32x4_t = vdupq_n_f32(test_value);
                
                // vcltq_f32: compare less than (test_value < values)
                let slot_precedes_value: uint32x4_t = vcltq_f32(test_vector, values);
                
                let increment: int32x4_t = vandq_s32(vreinterpretq_s32_u32(slot_precedes_value), one_mask);
                counts = vaddq_s32(counts, increment);
            }

            // Store the 4 count values
            vst1q_s32(indices_ptr.add(i), counts);
            i += 4;
        }
    }

    /// Entry point for ARM counting sort - uses NEON.
    /// 
    /// # Safety
    /// - `padded_keys` and `padded_target_indices` must have the same length
    /// - Length must be a multiple of 4
    /// - Length must be >= element_count
    #[inline]
    pub unsafe fn vector_counting_sort(
        padded_keys: &[f32],
        padded_target_indices: &mut [i32],
        element_count: usize,
    ) {
        // NEON is always available on aarch64
        vector_counting_sort_neon(padded_keys, padded_target_indices, element_count);
    }
}

// ============================================================================
// Public API
// ============================================================================

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
pub use x86_impl::vector_counting_sort;

#[cfg(target_arch = "aarch64")]
pub use arm_impl::vector_counting_sort;

// Fallback for other architectures (e.g., 32-bit ARM, RISC-V, etc.)
#[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
pub unsafe fn vector_counting_sort(
    padded_keys: &[f32],
    padded_target_indices: &mut [i32],
    element_count: usize,
) {
    vector_counting_sort_scalar(padded_keys, padded_target_indices, element_count);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_counting_sort_basic() {
        // Pad to multiple of 8 for AVX2 compatibility
        let mut keys = vec![3.0f32, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0];
        let mut indices = vec![0i32; 8];
        let element_count = 8;
        
        unsafe {
            vector_counting_sort(&keys, &mut indices, element_count);
        }
        
        // Verify: indices should map to sorted order
        // Original: [3.0, 1.0, 4.0, 1.0, 5.0, 9.0, 2.0, 6.0]
        // Sorted:   [1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 9.0]
        // Indices should be: [3, 0, 4, 1, 5, 7, 2, 6]
        // (accounting for stable sort with equal values)
        
        // Basic sanity check: all indices should be unique and in range
        let mut seen = vec![false; 8];
        for &idx in &indices {
            assert!(idx >= 0 && idx < 8);
            assert!(!seen[idx as usize], "Duplicate index found");
            seen[idx as usize] = true;
        }
    }

    #[test]
    fn test_counting_sort_scalar_matches_simd() {
        let mut keys = vec![5.0f32, 2.0, 8.0, 1.0, 9.0, 3.0, 7.0, 4.0];
        let mut simd_indices = vec![0i32; 8];
        let mut scalar_indices = vec![0i32; 8];
        
        unsafe {
            vector_counting_sort(&keys, &mut simd_indices, 8);
        }
        vector_counting_sort_scalar(&keys, &mut scalar_indices, 8);
        
        assert_eq!(simd_indices, scalar_indices, "SIMD and scalar results should match");
    }
}
