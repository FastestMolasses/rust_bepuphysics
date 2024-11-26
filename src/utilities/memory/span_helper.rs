use crate::utilities::memory::buffer::Buffer;
use core::ptr;

pub const MAXIMUM_SPAN_SIZE_POWER: usize = 30;

/// Computes the lowest integer N such that 2^N >= i.
#[inline(always)]
pub fn get_containing_power_of_2(i: i32) -> i32 {
    if i == 0 {
        0
    } else {
        32 - ((i - 1) as u32).leading_zeros() as i32
    }
}

/// Tests if a generic parameter is primitive. Fast path; specialized compilation.
#[inline(always)]
pub fn is_primitive<T: 'static>() -> bool {
    let t = std::any::TypeId::of::<T>();
    t == std::any::TypeId::of::<bool>()
        || t == std::any::TypeId::of::<u8>()
        || t == std::any::TypeId::of::<i8>()
        || t == std::any::TypeId::of::<u16>()
        || t == std::any::TypeId::of::<i16>()
        || t == std::any::TypeId::of::<u32>()
        || t == std::any::TypeId::of::<i32>()
        || t == std::any::TypeId::of::<u64>()
        || t == std::any::TypeId::of::<i64>()
        || t == std::any::TypeId::of::<usize>()
        || t == std::any::TypeId::of::<isize>()
        || t == std::any::TypeId::of::<char>()
        || t == std::any::TypeId::of::<f64>()
        || t == std::any::TypeId::of::<f32>()
}

/// Copies data from one buffer to another.
#[inline(always)]
pub unsafe fn copy<T>(
    source: &Buffer<T>,
    source_index: i32,
    target: &mut Buffer<T>,
    target_index: i32,
    count: i32,
) where
    T: Copy + Sized,
{
    debug_assert!(
        target_index >= 0 && target_index + count <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && source_index + count <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    ptr::copy_nonoverlapping(
        source.get_ptr(source_index),
        target.get_mut_ptr(target_index),
        count as usize,
    );
}

/// Copies data from a buffer to a slice.
#[inline(always)]
pub unsafe fn copy_to_slice<T>(
    source: &Buffer<T>,
    source_index: i32,
    target: &mut [T],
    target_index: i32,
    count: i32,
) where
    T: Copy + Sized,
{
    debug_assert!(
        target_index >= 0 && (target_index + count) as usize <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && source_index + count <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    ptr::copy_nonoverlapping(
        source.get_ptr(source_index),
        target.as_mut_ptr().add(target_index as usize),
        count as usize,
    );
}

/// Copies data from a slice to a buffer.
#[inline(always)]
pub unsafe fn copy_from_slice<T>(
    source: &[T],
    source_index: i32,
    target: &mut Buffer<T>,
    target_index: i32,
    count: i32,
) where
    T: Copy + Sized,
{
    debug_assert!(
        target_index >= 0 && target_index + count <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && (source_index + count) as usize <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    ptr::copy_nonoverlapping(
        source.as_ptr().add(source_index as usize),
        target.get_mut_ptr(target_index),
        count as usize,
    );
}

/// Copies data from a slice to a buffer.
#[inline(always)]
pub unsafe fn copy_from_slice_readonly<T>(
    source: &[T],
    source_index: i32,
    target: &mut Buffer<T>,
    target_index: i32,
    count: i32,
) where
    T: Copy + Sized,
{
    debug_assert!(
        target_index >= 0 && target_index + count <= target.len(),
        "Can't perform a copy that extends beyond the target span."
    );
    debug_assert!(
        source_index >= 0 && (source_index + count) as usize <= source.len(),
        "Can't perform a copy that extends beyond the source span."
    );

    let source_slice =
        std::slice::from_raw_parts(source.as_ptr().add(source_index as usize), count as usize);
    let target_slice =
        std::slice::from_raw_parts_mut(target.get_mut_ptr(target_index), count as usize);
    target_slice.copy_from_slice(source_slice);
}
