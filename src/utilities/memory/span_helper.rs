//! Helper functions for span/buffer operations.

use super::buffer::Buffer;
use std::ptr;

/// The highest size span exponent. The largest span is 2^MaximumSpanSizePower.
/// This avoids overflow.
pub const MAXIMUM_SPAN_SIZE_POWER: i32 = 30;

/// Validates that a power value is within the valid range.
#[inline(always)]
pub fn validate_power(power: i32) {
    debug_assert!(
        power >= 0 && power <= MAXIMUM_SPAN_SIZE_POWER,
        "Power must be from 0 to {}, inclusive.",
        MAXIMUM_SPAN_SIZE_POWER
    );
}

/// Computes the lowest integer N such that 2^N >= i.
#[inline(always)]
pub fn get_containing_power_of_2(i: i32) -> i32 {
    if i <= 0 {
        0
    } else {
        let unsigned = i as u32;
        32 - (unsigned - 1).leading_zeros() as i32
    }
}

/// Tests if a type is a primitive type.
/// The Rust compiler will optimize this to a constant for each monomorphized type.
#[inline(always)]
pub fn is_primitive<T: 'static>() -> bool {
    use std::any::TypeId;
    let t = TypeId::of::<T>();
    t == TypeId::of::<bool>()
        || t == TypeId::of::<u8>()
        || t == TypeId::of::<i8>()
        || t == TypeId::of::<u16>()
        || t == TypeId::of::<i16>()
        || t == TypeId::of::<u32>()
        || t == TypeId::of::<i32>()
        || t == TypeId::of::<u64>()
        || t == TypeId::of::<i64>()
        || t == TypeId::of::<usize>()
        || t == TypeId::of::<isize>()
        || t == TypeId::of::<char>()
        || t == TypeId::of::<f64>()
        || t == TypeId::of::<f32>()
}

/// Copies data from one buffer to another.
#[inline(always)]
pub fn copy<T>(
    source: &Buffer<T>,
    source_index: i32,
    target: &mut Buffer<T>,
    target_index: i32,
    count: i32,
) {
    debug_assert!(
        target_index >= 0 && target_index + count <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && source_index + count <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    unsafe {
        ptr::copy_nonoverlapping(
            source.get_ptr(source_index),
            target.get_mut_ptr(target_index),
            count as usize,
        );
    }
}

/// Copies data from a buffer to a slice.
#[inline(always)]
pub fn copy_to_slice<T>(
    source: &Buffer<T>,
    source_index: i32,
    target: &mut [T],
    target_index: i32,
    count: i32,
) {
    debug_assert!(
        target_index >= 0 && (target_index + count) as usize <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && source_index + count <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    unsafe {
        ptr::copy_nonoverlapping(
            source.get_ptr(source_index),
            target.as_mut_ptr().add(target_index as usize),
            count as usize,
        );
    }
}

/// Copies data from a slice to a buffer.
#[inline(always)]
pub fn copy_from_slice<T>(
    source: &[T],
    source_index: i32,
    target: &mut Buffer<T>,
    target_index: i32,
    count: i32,
) {
    debug_assert!(
        target_index >= 0 && target_index + count <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && (source_index + count) as usize <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    unsafe {
        ptr::copy_nonoverlapping(
            source.as_ptr().add(source_index as usize),
            target.get_mut_ptr(target_index),
            count as usize,
        );
    }
}
