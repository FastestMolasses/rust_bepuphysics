use std::arch::x86_64::*;
use std::cmp::min;
use std::sync::atomic::{AtomicBool, Ordering};

// TODO: IMPLEMENT aarch64 SUPPORT WITH CONDITIONAL COMPILATION

static HARDWARE_ACCELERATED: AtomicBool = AtomicBool::new(true);

/// Check hardware acceleration availability for Vector256 and Vector128
fn check_hardware_acceleration() {
    if is_x86_feature_detected!("avx2") {
        HARDWARE_ACCELERATED.store(true, Ordering::Relaxed);
    } else if is_x86_feature_detected!("sse2") {
        HARDWARE_ACCELERATED.store(false, Ordering::Relaxed);
    } else {
        panic!("This sort requires AVX2 or at least SSE2 to be supported by the hardware.");
    }
}

pub unsafe fn vector_counting_sort(
    padded_keys: &[f32],
    padded_target_indices: &mut [i32],
    element_count: usize,
) {
    check_hardware_acceleration();
    if HARDWARE_ACCELERATED.load(Ordering::Relaxed) {
        debug_assert!(
            padded_keys.len() == padded_target_indices.len()
                && padded_keys.len() % 8 == 0
                && padded_keys.len() >= element_count,
            "Preconditions not met."
        );
        let index_offsets = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);
        for i in (0..element_count).step_by(8) {
            let values = _mm256_loadu_ps(padded_keys.as_ptr().add(i) as *const f32);
            let mut counts = _mm256_setzero_si256();

            let one_mask = _mm256_set1_epi32(1);
            for j in 0..i {
                let test_vector = _mm256_set1_ps(*padded_keys.get_unchecked(j));
                let slot_precedes_value =
                    _mm256_castps_si256(_mm256_cmp_ps(test_vector, values, _CMP_LE_OS));
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            let end_of_equality_testing = min(i + 8, element_count);
            let slot_index = _mm256_add_epi32(_mm256_set1_epi32(i as i32), index_offsets);
            for j in i..end_of_equality_testing {
                let test_vector = _mm256_set1_ps(*padded_keys.get_unchecked(j));
                let slot_is_lesser =
                    _mm256_castps_si256(_mm256_cmp_ps(test_vector, values, _CMP_LT_OS));
                let slot_index_is_lesser =
                    _mm256_cmpgt_epi32(slot_index, _mm256_set1_epi32(j as i32));
                let slot_is_equal =
                    _mm256_castps_si256(_mm256_cmp_ps(test_vector, values, _CMP_EQ_OS));
                let slot_precedes_value = _mm256_or_si256(
                    slot_is_lesser,
                    _mm256_and_si256(slot_is_equal, slot_index_is_lesser),
                );
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            for j in end_of_equality_testing..element_count {
                let test_vector = _mm256_set1_ps(*padded_keys.get_unchecked(j));
                let slot_precedes_value =
                    _mm256_castps_si256(_mm256_cmp_ps(test_vector, values, _CMP_LT_OS));
                counts = _mm256_add_epi32(counts, _mm256_and_si256(slot_precedes_value, one_mask));
            }

            _mm256_storeu_si256(
                padded_target_indices.as_mut_ptr().add(i) as *mut __m256i,
                counts,
            );
        }
    } else {
        debug_assert!(
            padded_keys.len() == padded_target_indices.len()
                && padded_keys.len() % 4 == 0
                && padded_keys.len() >= element_count,
            "Preconditions not met."
        );
        let index_offsets = _mm_setr_epi32(0, 1, 2, 3);
        for i in (0..element_count).step_by(4) {
            let values = _mm_loadu_ps(padded_keys.as_ptr().add(i) as *const f32);
            let mut counts = _mm_setzero_si128();

            let one_mask = _mm_set1_epi32(1);
            for j in 0..i {
                let test_vector = _mm_set1_ps(*padded_keys.get_unchecked(j));
                let slot_precedes_value = _mm_castps_si128(_mm_cmple_ps(test_vector, values));
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            let end_of_equality_testing = min(i + 4, element_count);
            let slot_index = _mm_add_epi32(_mm_set1_epi32(i as i32), index_offsets);
            for j in i..end_of_equality_testing {
                let test_vector = _mm_set1_ps(*padded_keys.get_unchecked(j));
                let slot_is_lesser = _mm_castps_si128(_mm_cmplt_ps(test_vector, values));
                let slot_index_is_lesser = _mm_cmplt_epi32(_mm_set1_epi32(j as i32), slot_index);
                let slot_is_equal = _mm_castps_si128(_mm_cmpeq_ps(test_vector, values));
                let slot_precedes_value = _mm_or_si128(
                    slot_is_lesser,
                    _mm_and_si128(slot_is_equal, slot_index_is_lesser),
                );
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            for j in end_of_equality_testing..element_count {
                let test_vector = _mm_set1_ps(*padded_keys.get_unchecked(j));
                let slot_precedes_value = _mm_castps_si128(_mm_cmplt_ps(test_vector, values));
                counts = _mm_add_epi32(counts, _mm_and_si128(slot_precedes_value, one_mask));
            }

            _mm_storeu_si128(
                padded_target_indices.as_mut_ptr().add(i) as *mut __m128i,
                counts,
            );
        }
    }
}
