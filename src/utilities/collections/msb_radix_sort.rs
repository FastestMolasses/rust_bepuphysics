//! MSB (Most Significant Byte) radix sort implementation.

use std::ptr;

/// Swaps two values in memory.
#[inline(always)]
unsafe fn swap<T>(a: *mut T, b: *mut T) {
    ptr::swap(a, b);
}

/// MSB radix sort for 32-bit keys with associated values.
/// Uses in-place distribution with recursive calls for each byte level.
pub unsafe fn sort_u32<T: Copy>(
    keys: *mut i32,
    values: *mut T,
    bucket_counts: *mut i32,
    bucket_original_start_indices: *mut i32,
    key_count: i32,
    shift: i32,
) {
    if key_count < 32 {
        // Use insertion sort for small arrays
        for i in 1..key_count {
            let original_key = *keys.add(i as usize);
            let original_value = *values.add(i as usize);
            let mut compare_index = i - 1;

            while compare_index >= 0 {
                if original_key < *keys.add(compare_index as usize) {
                    // Move element up one slot
                    let upper_slot = (compare_index + 1) as usize;
                    *keys.add(upper_slot) = *keys.add(compare_index as usize);
                    *values.add(upper_slot) = *values.add(compare_index as usize);
                    compare_index -= 1;
                } else {
                    break;
                }
            }

            let target_index = (compare_index + 1) as usize;
            if target_index != i as usize {
                *keys.add(target_index) = original_key;
                *values.add(target_index) = original_value;
            }
        }
        return;
    }

    const BUCKET_COUNT: usize = 256;

    // Clear bucket counts
    for i in 0..BUCKET_COUNT {
        *bucket_counts.add(i) = 0;
    }

    // Count occurrences for each bucket
    for i in 0..key_count {
        let key = *keys.add(i as usize);
        let bucket_index = ((key >> shift) & 0xFF) as usize;
        *bucket_counts.add(bucket_index) += 1;
    }

    // Convert bucket counts to partial sums
    let mut sum = 0i32;
    for i in 0..BUCKET_COUNT {
        let previous_sum = sum;
        let count = *bucket_counts.add(i);
        sum += count;
        *bucket_original_start_indices.add(i) = previous_sum;
        *bucket_counts.add(i) = previous_sum;
    }

    // Distribute elements to their buckets
    for i in 0..BUCKET_COUNT {
        let bucket_start_index = bucket_counts.add(i);
        let next_start_index = if i == BUCKET_COUNT - 1 {
            key_count
        } else {
            *bucket_original_start_indices.add(i + 1)
        };

        while *bucket_start_index < next_start_index {
            let mut local_key = *keys.add(*bucket_start_index as usize);
            let mut local_value = *values.add(*bucket_start_index as usize);

            loop {
                let target_bucket_index = ((local_key >> shift) & 0xFF) as usize;
                if target_bucket_index == i {
                    // Local key belongs to local bucket
                    *keys.add(*bucket_start_index as usize) = local_key;
                    *values.add(*bucket_start_index as usize) = local_value;
                    *bucket_start_index += 1;
                    break;
                }

                let target_bucket_start_index = bucket_counts.add(target_bucket_index);
                swap(
                    keys.add(*target_bucket_start_index as usize),
                    &mut local_key,
                );
                swap(
                    values.add(*target_bucket_start_index as usize),
                    &mut local_value,
                );
                *target_bucket_start_index += 1;
            }
        }
    }

    // Recursively sort each bucket if there are more byte levels
    if shift > 0 {
        let new_shift = shift - 8;
        let next_level_bucket_counts = bucket_counts.add(BUCKET_COUNT);

        let mut previous_end = 0i32;
        for i in 0..BUCKET_COUNT {
            let bucket_end = *bucket_counts.add(i);
            let count = bucket_end - previous_end;
            if count > 0 {
                sort_u32(
                    keys.add(previous_end as usize),
                    values.add(previous_end as usize),
                    next_level_bucket_counts,
                    bucket_original_start_indices,
                    count,
                    new_shift,
                );
            }
            previous_end = bucket_end;
        }
    }
}

/// Simplified MSB radix sort that allocates its own scratch space.
/// For small arrays, falls back to insertion sort.
pub unsafe fn sort_u32_simple<T: Copy>(
    keys: &mut [i32],
    values: &mut [T],
    key_count: i32,
    shift: i32,
) {
    if key_count < 256 {
        // Use insertion sort for small arrays
        for i in 1..key_count as usize {
            let original_key = keys[i];
            let original_value = values[i];
            let mut j = i;
            while j > 0 && keys[j - 1] > original_key {
                keys[j] = keys[j - 1];
                values[j] = values[j - 1];
                j -= 1;
            }
            keys[j] = original_key;
            values[j] = original_value;
        }
        return;
    }

    const BUCKET_COUNT: usize = 128;
    const MASK: i32 = (BUCKET_COUNT - 1) as i32;

    let mut bucket_counts = [0i32; BUCKET_COUNT];
    let mut bucket_original_start_indices = [0i32; BUCKET_COUNT];

    // Count occurrences
    for i in 0..key_count as usize {
        let bucket_index = ((keys[i] >> shift) & MASK) as usize;
        bucket_counts[bucket_index] += 1;
    }

    // Convert to partial sums
    let mut sum = 0i32;
    for i in 0..BUCKET_COUNT {
        let previous_sum = sum;
        sum += bucket_counts[i];
        bucket_original_start_indices[i] = previous_sum;
        bucket_counts[i] = previous_sum;
    }

    // Distribute elements
    for i in 0..BUCKET_COUNT {
        let next_start_index = if i == BUCKET_COUNT - 1 {
            key_count
        } else {
            bucket_original_start_indices[i + 1]
        };

        while bucket_counts[i] < next_start_index {
            let idx = bucket_counts[i] as usize;
            let mut local_key = keys[idx];
            let mut local_value = values[idx];

            loop {
                let target_bucket_index = ((local_key >> shift) & MASK) as usize;
                if target_bucket_index == i {
                    keys[idx] = local_key;
                    values[idx] = local_value;
                    bucket_counts[i] += 1;
                    break;
                }

                let target_idx = bucket_counts[target_bucket_index] as usize;
                std::mem::swap(&mut keys[target_idx], &mut local_key);
                std::mem::swap(&mut values[target_idx], &mut local_value);
                bucket_counts[target_bucket_index] += 1;
            }
        }
    }

    // Recursive sort
    if shift > 0 {
        let new_shift = shift - 7; // 7 bits per level for 128 buckets

        let mut previous_end = 0usize;
        for i in 0..BUCKET_COUNT {
            let bucket_end = bucket_counts[i] as usize;
            let count = bucket_end - previous_end;
            if count > 0 {
                sort_u32_simple(
                    &mut keys[previous_end..bucket_end],
                    &mut values[previous_end..bucket_end],
                    count as i32,
                    new_shift,
                );
            }
            previous_end = bucket_end;
        }
    }
}
