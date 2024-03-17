use std::ptr;

// Swaps two values in memory.
#[inline(always)]
unsafe fn swap<T>(a: *mut T, b: *mut T) {
    ptr::swap(a, b);
}

pub unsafe fn sort_u32<T>(
    keys: &mut [i32],
    values: &mut [T],
    bucket_counts: &mut [i32],
    bucket_original_start_indices: &mut [i32],
    key_count: usize,
    shift: usize,
) {
    if key_count < 32 {
        // Use insertion sort for small arrays.
        for i in 1..key_count {
            let original_key = *keys.get_unchecked(i);
            let original_value = ptr::read(values.get_unchecked(i));

            let mut compare_index = i;
            while compare_index > 0 {
                compare_index -= 1;
                if original_key < *keys.get_unchecked(compare_index) {
                    *keys.get_unchecked_mut(compare_index + 1) = *keys.get_unchecked(compare_index);
                    ptr::write(
                        values.get_unchecked_mut(compare_index + 1),
                        ptr::read(values.get_unchecked(compare_index)),
                    );
                } else {
                    compare_index += 1;
                    break;
                }
            }

            if compare_index != i {
                *keys.get_unchecked_mut(compare_index) = original_key;
                ptr::write(values.get_unchecked_mut(compare_index), original_value);
            } else {
                // Prevent destructor from being called on the original value as it was moved.
                std::mem::forget(original_value);
            }
        }
        return;
    }

    const BUCKET_COUNT: usize = 256;
    bucket_counts.iter_mut().for_each(|x| *x = 0);

    for i in 0..key_count {
        let key = *keys.get_unchecked(i);
        *bucket_counts.get_unchecked_mut(((key >> shift) & 0xFF) as usize) += 1;
    }

    let mut sum = 0;
    for i in 0..BUCKET_COUNT {
        let bucket = bucket_counts.get_unchecked_mut(i);
        let previous_sum = sum;
        sum += *bucket;
        *bucket_original_start_indices.get_unchecked_mut(i) = previous_sum;
        *bucket = previous_sum;
    }

    for i in 0..BUCKET_COUNT {
        let mut bucket_start_index = *bucket_counts.get_unchecked(i);
        let next_start_index = if i == BUCKET_COUNT - 1 {
            key_count
        } else {
            *bucket_original_start_indices.get_unchecked(i + 1)
        };

        while bucket_start_index < next_start_index {
            let local_key = *keys.get_unchecked(bucket_start_index);
            let local_value = ptr::read(values.get_unchecked(bucket_start_index));

            let mut displaced = true;
            while displaced {
                let target_bucket_index = ((local_key >> shift) & 0xFF) as usize;
                if target_bucket_index == i {
                    *keys.get_unchecked_mut(bucket_start_index) = local_key;
                    ptr::write(values.get_unchecked_mut(bucket_start_index), local_value);
                    *bucket_counts.get_unchecked_mut(i) += 1;
                    displaced = false;
                } else {
                    let target_bucket_start_index =
                        bucket_counts.get_unchecked_mut(target_bucket_index);
                    swap(
                        keys.as_mut_ptr().add(*target_bucket_start_index),
                        &mut local_key,
                    );
                    swap(
                        values.as_mut_ptr().add(*target_bucket_start_index) as *mut T,
                        &mut local_value as *mut T,
                    );
                    *target_bucket_start_index += 1;
                }
            }
            bucket_start_index += 1;
        }
    }

    if shift > 0 {
        let new_shift = shift - 8;
        let mut previous_end = 0;
        for i in 0..BUCKET_COUNT {
            let bucket_end = *bucket_counts.get_unchecked(i);
            let count = bucket_end - previous_end;
            if count > 0 {
                sort_u32(
                    &mut keys[previous_end..],
                    &mut values[previous_end..],
                    &mut bucket_counts[BUCKET_COUNT..],
                    bucket_original_start_indices,
                    count,
                    new_shift,
                );
            }
            previous_end = bucket_end;
        }
    }
}

// TODO: REPLACE THIS WITH THE ACTUAL QUICKSORT IMPLEMENTATION
// Assumes the existence of a quicksort implementation.
// You may need to replace this with an actual implementation or use an existing crate.
fn quicksort<T: Ord>(keys: &mut [i32], values: &mut [T], low: usize, high: usize) {
    // This is a placeholder for a quicksort algorithm.
    // An actual implementation should be used here.
}

pub unsafe fn sort_u32_small<T>(
    keys: &mut [i32],
    values: &mut [T],
    key_count: usize,
    shift: usize,
) {
    if key_count < 256 {
        // Use an external quicksort for small arrays. This is a placeholder call.
        quicksort(keys, values, 0, key_count - 1);
        return;
    }

    const BUCKET_COUNT: usize = 128; // Half of 256 to demonstrate a different bucket count.
    const MASK: usize = BUCKET_COUNT - 1;

    let mut bucket_counts = [0i32; BUCKET_COUNT];
    let mut bucket_original_start_indices = [0i32; BUCKET_COUNT];

    for i in 0..key_count {
        let key = *keys.get_unchecked(i);
        bucket_counts[((key >> shift) & MASK as i32) as usize] += 1;
    }

    let mut sum = 0;
    for i in 0..BUCKET_COUNT {
        let previous_sum = sum;
        sum += bucket_counts[i];
        bucket_original_start_indices[i] = previous_sum;
        bucket_counts[i] = previous_sum;
    }

    for i in 0..BUCKET_COUNT {
        let mut bucket_start_index = bucket_counts[i];
        let next_start_index = if i == BUCKET_COUNT - 1 {
            key_count as i32
        } else {
            bucket_original_start_indices[i + 1]
        };

        while bucket_start_index < next_start_index {
            let local_key = *keys.get_unchecked(bucket_start_index as usize);
            let local_value = ptr::read(values.get_unchecked(bucket_start_index as usize));

            let mut displaced = true;
            while displaced {
                let target_bucket_index = ((local_key >> shift) & MASK as i32) as usize;
                if target_bucket_index == i {
                    *keys.get_unchecked_mut(bucket_start_index as usize) = local_key;
                    ptr::write(
                        values.get_unchecked_mut(bucket_start_index as usize),
                        local_value,
                    );
                    bucket_counts[i] += 1;
                    displaced = false;
                } else {
                    let target_bucket_start_index = &mut bucket_counts[target_bucket_index];
                    swap(
                        keys.as_mut_ptr().add(*target_bucket_start_index as usize),
                        &mut local_key,
                    );
                    swap(
                        values.as_mut_ptr().add(*target_bucket_start_index as usize) as *mut T,
                        &mut local_value as *mut T,
                    );
                    *target_bucket_start_index += 1;
                }
            }
            bucket_start_index += 1;
        }
    }

    if shift > 0 {
        let new_shift = shift.saturating_sub(8); // Ensure no underflow.
        let mut previous_end = 0;
        for i in 0..BUCKET_COUNT {
            let bucket_end = bucket_counts[i];
            let count = (bucket_end - previous_end) as usize;
            if count > 0 {
                sort_u32_small(
                    &mut keys[previous_end..previous_end + count],
                    &mut values[previous_end..previous_end + count],
                    count,
                    new_shift,
                );
            }
            previous_end = bucket_end as usize;
        }
    }
}
