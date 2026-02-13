//! LSB (Least Significant Byte) radix sort implementation.

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

/// Reorders elements based on a single byte of the key.
#[inline(always)]
unsafe fn reorder_for_byte<T: Copy>(
    source_keys: *const i32,
    target_keys: *mut i32,
    source_values: *const T,
    target_values: *mut T,
    key_count: i32,
    indices: *mut i32,
    shift: i32,
) {
    for i in 0..key_count {
        let key = *source_keys.add(i as usize);
        let bucket_index = ((key >> shift) & 0xFF) as usize;
        let bucket_start_index = indices.add(bucket_index);
        let target_index = *bucket_start_index as usize;
        *target_keys.add(target_index) = key;
        *target_values.add(target_index) = *source_values.add(i as usize);
        *bucket_start_index += 1;
    }
}

/// Sorts keys and values using 32-bit key radix sort (4 passes).
pub unsafe fn sort_u32<T: Copy>(
    keys: *mut i32,
    values: *mut T,
    keys_scratch: *mut i32,
    values_scratch: *mut T,
    bucket_counts: *mut i32,
    key_count: i32,
) {
    let byte1_counts = bucket_counts.add(256);
    let byte2_counts = bucket_counts.add(512);
    let byte3_counts = bucket_counts.add(768);

    // Count occurrences for each byte position
    for i in 0..key_count {
        let key = *keys.add(i as usize);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
        *byte2_counts.add(((key >> 16) & 0xFF) as usize) += 1;
        *byte3_counts.add((key >> 24) as usize) += 1;
    }

    // Convert bucket counts to partial sums
    let mut sum0 = 0i32;
    let mut sum1 = 0i32;
    let mut sum2 = 0i32;
    let mut sum3 = 0i32;
    for i in 0..256 {
        let prev_sum0 = sum0;
        let prev_sum1 = sum1;
        let prev_sum2 = sum2;
        let prev_sum3 = sum3;

        let byte0 = bucket_counts.add(i);
        let byte1 = byte1_counts.add(i);
        let byte2 = byte2_counts.add(i);
        let byte3 = byte3_counts.add(i);

        sum0 += *byte0;
        sum1 += *byte1;
        sum2 += *byte2;
        sum3 += *byte3;

        *byte0 = prev_sum0;
        *byte1 = prev_sum1;
        *byte2 = prev_sum2;
        *byte3 = prev_sum3;
    }

    reorder_for_byte(
        keys,
        keys_scratch,
        values,
        values_scratch,
        key_count,
        bucket_counts,
        0,
    );
    reorder_for_byte(
        keys_scratch,
        keys,
        values_scratch,
        values,
        key_count,
        byte1_counts,
        8,
    );
    reorder_for_byte(
        keys,
        keys_scratch,
        values,
        values_scratch,
        key_count,
        byte2_counts,
        16,
    );
    reorder_for_byte(
        keys_scratch,
        keys,
        values_scratch,
        values,
        key_count,
        byte3_counts,
        24,
    );
}

/// Sorts keys and values using 24-bit key radix sort (3 passes).
pub unsafe fn sort_u24<T: Copy>(
    input_keys: *mut i32,
    input_values: *mut T,
    output_keys: *mut i32,
    output_values: *mut T,
    bucket_counts: *mut i32,
    key_count: i32,
) {
    let byte1_counts = bucket_counts.add(256);
    let byte2_counts = bucket_counts.add(512);

    // Count occurrences
    for i in 0..key_count {
        let key = *input_keys.add(i as usize);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
        *byte2_counts.add(((key >> 16) & 0xFF) as usize) += 1;
    }

    // Convert to partial sums
    let mut sum0 = 0i32;
    let mut sum1 = 0i32;
    let mut sum2 = 0i32;
    for i in 0..256 {
        let prev_sum0 = sum0;
        let prev_sum1 = sum1;
        let prev_sum2 = sum2;

        let byte0 = bucket_counts.add(i);
        let byte1 = byte1_counts.add(i);
        let byte2 = byte2_counts.add(i);

        sum0 += *byte0;
        sum1 += *byte1;
        sum2 += *byte2;

        *byte0 = prev_sum0;
        *byte1 = prev_sum1;
        *byte2 = prev_sum2;
    }

    reorder_for_byte(
        input_keys,
        output_keys,
        input_values,
        output_values,
        key_count,
        bucket_counts,
        0,
    );
    reorder_for_byte(
        output_keys,
        input_keys,
        output_values,
        input_values,
        key_count,
        byte1_counts,
        8,
    );
    reorder_for_byte(
        input_keys,
        output_keys,
        input_values,
        output_values,
        key_count,
        byte2_counts,
        16,
    );
}

/// Sorts keys and values using 16-bit key radix sort (2 passes).
pub unsafe fn sort_u16<T: Copy>(
    keys: *mut i32,
    values: *mut T,
    keys_scratch: *mut i32,
    values_scratch: *mut T,
    bucket_counts: *mut i32,
    key_count: i32,
) {
    let byte1_counts = bucket_counts.add(256);

    // Count occurrences
    for i in 0..key_count {
        let key = *keys.add(i as usize);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
    }

    // Convert to partial sums
    let mut sum0 = 0i32;
    let mut sum1 = 0i32;
    for i in 0..256 {
        let prev_sum0 = sum0;
        let prev_sum1 = sum1;

        let byte0 = bucket_counts.add(i);
        let byte1 = byte1_counts.add(i);

        sum0 += *byte0;
        sum1 += *byte1;

        *byte0 = prev_sum0;
        *byte1 = prev_sum1;
    }

    reorder_for_byte(
        keys,
        keys_scratch,
        values,
        values_scratch,
        key_count,
        bucket_counts,
        0,
    );
    reorder_for_byte(
        keys_scratch,
        keys,
        values_scratch,
        values,
        key_count,
        byte1_counts,
        8,
    );
}

/// Sorts keys and values using 8-bit key radix sort (1 pass).
pub unsafe fn sort_u8<T: Copy>(
    input_keys: *const i32,
    input_values: *const T,
    output_keys: *mut i32,
    output_values: *mut T,
    bucket_counts: *mut i32,
    key_count: i32,
) {
    // Count occurrences
    for i in 0..key_count {
        let key = *input_keys.add(i as usize);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
    }

    // Convert to partial sums
    let mut sum0 = 0i32;
    for i in 0..256 {
        let prev_sum0 = sum0;
        let byte0 = bucket_counts.add(i);
        sum0 += *byte0;
        *byte0 = prev_sum0;
    }

    reorder_for_byte(
        input_keys,
        output_keys,
        input_values,
        output_values,
        key_count,
        bucket_counts,
        0,
    );
}

/// Result of a radix sort operation, indicating which buffers contain the sorted data.
pub struct SortResult<'a, T: Copy> {
    pub sorted_keys: &'a Buffer<i32>,
    pub sorted_values: &'a Buffer<T>,
}

/// Sorts a set of keys and their associated values using radix sort.
///
/// # Arguments
/// * `input_keys` - Buffer containing the keys to sort
/// * `input_values` - Buffer containing the values to sort
/// * `scratch_keys` - Scratch buffer for temporary results
/// * `scratch_values` - Scratch buffer for temporary results  
/// * `start_index` - Start location of the sort
/// * `count` - Number of elements to sort
/// * `keys_upper_bound` - Value >= any key in the sort region
/// * `buffer_pool` - Pool to pull temporary buffers from
///
/// # Returns
/// Tuple of (sorted_keys_in_scratch, sorted_values_in_scratch) - true if sorted data is in scratch buffers
pub fn sort<T: Copy>(
    input_keys: &mut Buffer<i32>,
    input_values: &mut Buffer<T>,
    scratch_keys: &mut Buffer<i32>,
    scratch_values: &mut Buffer<T>,
    start_index: i32,
    count: i32,
    keys_upper_bound: i32,
    buffer_pool: &mut BufferPool,
) -> (bool, bool) {
    debug_assert!(
        input_keys.len() >= start_index + count
            && input_values.len() >= start_index + count
            && scratch_keys.len() >= start_index + count
            && scratch_values.len() >= start_index + count,
        "The buffers must be able to hold the sort region."
    );

    let bucket_set_count = if keys_upper_bound < (1 << 16) {
        if keys_upper_bound < (1 << 8) {
            1
        } else {
            2
        }
    } else if keys_upper_bound < (1 << 24) {
        3
    } else {
        4
    };

    let mut bucket_counts: Buffer<i32> = buffer_pool.take_at_least(bucket_set_count * 256);
    bucket_counts.clear(0, bucket_set_count * 256);

    let (sorted_in_scratch_keys, sorted_in_scratch_values) = unsafe {
        let input_keys_ptr = input_keys.as_mut_ptr().add(start_index as usize);
        let input_values_ptr = input_values.as_mut_ptr().add(start_index as usize);
        let scratch_keys_ptr = scratch_keys.as_mut_ptr().add(start_index as usize);
        let scratch_values_ptr = scratch_values.as_mut_ptr().add(start_index as usize);
        let bucket_counts_ptr = bucket_counts.as_mut_ptr();

        match bucket_set_count {
            1 => {
                sort_u8(
                    input_keys_ptr,
                    input_values_ptr,
                    scratch_keys_ptr,
                    scratch_values_ptr,
                    bucket_counts_ptr,
                    count,
                );
                (true, true)
            }
            2 => {
                sort_u16(
                    input_keys_ptr,
                    input_values_ptr,
                    scratch_keys_ptr,
                    scratch_values_ptr,
                    bucket_counts_ptr,
                    count,
                );
                (false, false)
            }
            3 => {
                sort_u24(
                    input_keys_ptr,
                    input_values_ptr,
                    scratch_keys_ptr,
                    scratch_values_ptr,
                    bucket_counts_ptr,
                    count,
                );
                (true, true)
            }
            4 => {
                sort_u32(
                    input_keys_ptr,
                    input_values_ptr,
                    scratch_keys_ptr,
                    scratch_values_ptr,
                    bucket_counts_ptr,
                    count,
                );
                (false, false)
            }
            _ => panic!("Invalid bucket set count"),
        }
    };

    buffer_pool.return_buffer(&mut bucket_counts);
    (sorted_in_scratch_keys, sorted_in_scratch_values)
}
