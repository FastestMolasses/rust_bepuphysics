use crate::utilities::vector::Vector;
use std::simd::{cmp::SimdPartialOrd, Simd};

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;

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

    /// Creates a mask where lanes >= count_in_bundle are set to -1 (all bits set).
    #[inline(always)]
    pub fn create_trailing_mask_for_count_in_bundle(count_in_bundle: usize) -> Vector<i32> {
        let count = Vector::<i32>::splat(count_in_bundle as i32);
        let indices = Vector::<i32>::from_array(std::array::from_fn(|i| i as i32));
        let mask = count.simd_le(indices);
        // Convert bool mask to -1/0 integers
        Simd::from_array(std::array::from_fn(|i| if mask.test(i) { -1i32 } else { 0i32 }))
    }

    /// Creates a mask where lanes < count_in_bundle are set to -1 (all bits set).
    #[inline(always)]
    pub fn create_mask_for_count_in_bundle(count_in_bundle: usize) -> Vector<i32> {
        let count = Vector::<i32>::splat(count_in_bundle as i32);
        let indices = Vector::<i32>::from_array(std::array::from_fn(|i| i as i32));
        let mask = count.simd_gt(indices);
        // Convert bool mask to -1/0 integers
        Simd::from_array(std::array::from_fn(|i| if mask.test(i) { -1i32 } else { 0i32 }))
    }

    #[inline(always)]
    pub fn get_first_set_lane_index(v: Vector<i32>) -> i32 {
        // Use portable SIMD comparison and bitmask
        use std::simd::cmp::SimdPartialEq;
        let mask = v.simd_eq(Vector::splat(-1));
        let bits = mask.to_bitmask();
        if bits == 0 {
            -1
        } else {
            bits.trailing_zeros() as i32
        }
    }

    /// Gets the number of lanes that occur at or before the last set lane.
    /// In other words, if the lanes in the vector are (-1, 0, -1, 0), then this will return 3.
    #[inline(always)]
    pub fn get_last_set_lane_count(v: Vector<i32>) -> usize {
        use std::simd::cmp::SimdPartialEq;
        let mask = v.simd_eq(Vector::splat(-1));
        let bits = mask.to_bitmask();
        if bits == 0 {
            0
        } else {
            // Find the position of the highest set bit
            let bit_width = std::mem::size_of_val(&bits) * 8;
            (bit_width - bits.leading_zeros() as usize)
        }
    }
}
