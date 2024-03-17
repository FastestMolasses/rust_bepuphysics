use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

unsafe fn reorder_for_byte<T>(
    source_keys: *mut i32,
    target_keys: *mut i32,
    source_values: *mut T,
    target_values: *mut T,
    key_count: usize,
    indices: *mut i32,
    shift: i32,
) {
    for i in 0..key_count {
        let key = *source_keys.add(i);
        let bucket_start_index = indices.add(((key >> shift) & 0xFF) as usize);
        *target_keys.add(*bucket_start_index as usize) = key;
        *target_values.add(*bucket_start_index as usize) = *source_values.add(i);
        *bucket_start_index += 1;
    }
}

pub unsafe fn sort_u32<T>(
    keys: *mut i32,
    values: *mut T,
    keys_scratch: *mut i32,
    values_scratch: *mut T,
    bucket_counts: *mut i32,
    key_count: usize,
) {
    let byte1_counts = bucket_counts.add(256);
    let byte2_counts = bucket_counts.add(512);
    let byte3_counts = bucket_counts.add(768);

    for i in 0..key_count {
        let key = *keys.add(i);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
        *byte2_counts.add(((key >> 16) & 0xFF) as usize) += 1;
        *byte3_counts.add((key >> 24) as usize) += 1;
    }

    // Calculate partial sums for each byte slice
    let mut sum0 = 0;
    let mut sum1 = 0;
    let mut sum2 = 0;
    let mut sum3 = 0;
    for i in 0..256 {
        // For the least significant byte
        let temp = *bucket_counts.add(i);
        *bucket_counts.add(i) = sum0;
        sum0 += temp;

        // For the second byte
        let temp = *byte1_counts.add(i);
        *byte1_counts.add(i) = sum1;
        sum1 += temp;

        // For the third byte
        let temp = *byte2_counts.add(i);
        *byte2_counts.add(i) = sum2;
        sum2 += temp;

        // For the most significant byte
        let temp = *byte3_counts.add(i);
        *byte3_counts.add(i) = sum3;
        sum3 += temp;
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

// Helper function for calculating partial sums. This is a simplified version focusing on the logic structure.
unsafe fn calculate_partial_sums(bucket_counts: *mut i32, counts_length: usize) {
    let mut sum = 0;
    for i in 0..counts_length {
        let bucket = bucket_counts.add(i);
        sum += *bucket;
        *bucket = sum - *bucket; // Previous sum
    }
}

// For sort_u24
pub unsafe fn sort_u24<T>(
    input_keys: *mut i32,
    input_values: *mut T,
    output_keys: *mut i32,
    output_values: *mut T,
    bucket_counts: *mut i32,
    key_count: usize,
) {
    let byte1_counts = bucket_counts.add(256);
    let byte2_counts = bucket_counts.add(512);

    // Populate bucket counts
    for i in 0..key_count {
        let key = *input_keys.add(i);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
        *byte2_counts.add(((key >> 16) & 0xFF) as usize) += 1;
    }

    // Calculate partial sums for each byte segment
    calculate_partial_sums(bucket_counts, 256);
    calculate_partial_sums(byte1_counts, 256);
    calculate_partial_sums(byte2_counts, 256);

    // Reorder keys and values for each byte of the key
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

// For sort_u16
pub unsafe fn sort_u16<T>(
    input_keys: *mut i32,
    input_values: *mut T,
    output_keys: *mut i32,
    output_values: *mut T,
    bucket_counts: *mut i32,
    key_count: usize,
) {
    let byte1_counts = bucket_counts.add(256);

    // Populate bucket counts
    for i in 0..key_count {
        let key = *input_keys.add(i);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
        *byte1_counts.add(((key >> 8) & 0xFF) as usize) += 1;
    }

    // Calculate partial sums for each byte segment
    calculate_partial_sums(bucket_counts, 256);
    calculate_partial_sums(byte1_counts, 256);

    // Reorder keys and values for each byte of the key
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
}

// For sort_u8
pub unsafe fn sort_u8<T>(
    input_keys: *mut i32,
    input_values: *mut T,
    output_keys: *mut i32,
    output_values: *mut T,
    bucket_counts: *mut i32,
    key_count: usize,
) {
    // Populate bucket counts
    for i in 0..key_count {
        let key = *input_keys.add(i);
        *bucket_counts.add((key & 0xFF) as usize) += 1;
    }

    // Calculate partial sums for each byte segment
    calculate_partial_sums(bucket_counts, 256);

    // Reorder keys and values for each byte of the key
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

// Example implementation of the sort function.
// This is simplified and assumes you have a BufferPool and other necessary components already implemented.
pub unsafe fn sort<T: 'static>(
    input_keys: &mut Buffer<i32>,
    input_values: &mut Buffer<T>,
    scratch_keys: &mut Buffer<i32>,
    scratch_values: &mut Buffer<T>,
    start_index: usize,
    count: usize,
    keys_upper_bound: i32,
    buffer_pool: &mut BufferPool,
) {
    // Determine the number of buckets needed based on the keys_upper_bound
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

    // Assume buffer_pool.take and buffer_pool.return are available to manage bucket counts buffer
    let mut bucket_counts = buffer_pool.take(bucket_set_count * 256);

    // Zero the bucket counts
    for i in 0..(bucket_set_count * 256) {
        *bucket_counts.add(i) = 0;
    }

    match bucket_set_count {
        1 => sort_u8(
            input_keys.memory.as_ptr().add(start_index),
            input_values.memory.as_ptr().add(start_index),
            scratch_keys.memory.as_ptr().add(start_index),
            scratch_values.memory.as_ptr().add(start_index),
            bucket_counts,
            count,
        ),
        2 => sort_u16(
            input_keys.memory.as_ptr().add(start_index),
            input_values.memory.as_ptr().add(start_index),
            scratch_keys.memory.as_ptr().add(start_index),
            scratch_values.memory.as_ptr().add(start_index),
            bucket_counts,
            count,
        ),
        3 => sort_u24(
            input_keys.memory.as_ptr().add(start_index),
            input_values.memory.as_ptr().add(start_index),
            scratch_keys.memory.as_ptr().add(start_index),
            scratch_values.memory.as_ptr().add(start_index),
            bucket_counts,
            count,
        ),
        4 => sort_u32(
            input_keys.memory.as_ptr().add(start_index),
            input_values.memory.as_ptr().add(start_index),
            scratch_keys.memory.as_ptr().add(start_index),
            scratch_values.memory.as_ptr().add(start_index),
            bucket_counts,
            count,
        ),
        _ => panic!("Invalid bucket set count."),
    };

    buffer_pool.return_(bucket_counts);
}
