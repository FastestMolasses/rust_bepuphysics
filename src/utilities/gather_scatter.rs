use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::vector::{optimal_lanes, Vector};
use std::mem;
use std::ptr;

pub struct GatherScatter;

impl GatherScatter {
    /// Gets a reference to an element from a vector without using pointers, bypassing direct vector access for codegen reasons.
    /// This performs no bounds testing!
    #[inline(always)]
    pub unsafe fn get<T>(vector: &Vector<T>, index: usize) -> &T
    where
        T: Copy,
        T: std::simd::SimdElement,
        std::simd::LaneCount<{ optimal_lanes::<T>() }>: std::simd::SupportedLaneCount,
    {
        let ptr = vector as *const Vector<T> as *const T;
        &*ptr.add(index)
    }

    /// Gets a mutable reference to an element from a vector without using pointers.
    /// This performs no bounds testing!
    #[inline(always)]
    pub unsafe fn get_mut<T>(vector: &mut Vector<T>, index: usize) -> &mut T
    where
        T: Copy,
        T: std::simd::SimdElement,
        std::simd::LaneCount<{ optimal_lanes::<T>() }>: std::simd::SupportedLaneCount,
    {
        let ptr = vector as *mut Vector<T> as *mut T;
        &mut *ptr.add(index)
    }

    /// Copies from one bundle lane to another. The bundle must be a contiguous block of Vector types.
    #[inline(always)]
    pub unsafe fn copy_lane<T>(
        source_bundle: &T,
        source_inner_index: usize,
        target_bundle: &mut T,
        target_inner_index: usize,
    ) where
        T: Copy,
    {
        // Note the truncation. Currently used for some types that don't have a size evenly divisible by the Vector<int>.Count * sizeof(int).
        let size_in_ints = (mem::size_of::<T>() >> 2) & !BundleIndexing::vector_mask();

        let source_base = (source_bundle as *const T as *const i32).add(source_inner_index);
        let target_base = (target_bundle as *mut T as *mut i32).add(target_inner_index);

        // Copy a single int (one lane per vector). Stride by Vector<int>::LEN to reach the next vector's same lane.
        *target_base = *source_base;

        let count = Vector::<i32>::LEN;
        let mut offset = count;
        // 8-wide unroll empirically chosen to match C#.
        while offset + count * 8 <= size_in_ints {
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
        }
        if offset + 4 * count <= size_in_ints {
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
        }
        if offset + 2 * count <= size_in_ints {
            *target_base.add(offset) = *source_base.add(offset); offset += count;
            *target_base.add(offset) = *source_base.add(offset); offset += count;
        }
        if offset + count <= size_in_ints {
            *target_base.add(offset) = *source_base.add(offset);
        }
    }

    /// Clears a bundle lane using the default value of the specified type.
    #[inline(always)]
    pub unsafe fn clear_lane<TOuter, TVector>(bundle: &mut TOuter, inner_index: usize)
    where
        TOuter: Copy,
        TVector: Copy + Default,
        TVector: std::simd::SimdElement,
        std::simd::LaneCount<{ optimal_lanes::<TVector>() }>: std::simd::SupportedLaneCount,
    {
        let vector_count = mem::size_of::<TOuter>() / mem::size_of::<Vector<TVector>>();
        let lane_base = (bundle as *mut TOuter as *mut TVector).add(inner_index);

        for i in 0..vector_count {
            *lane_base.add(i * Vector::<TVector>::LEN) = TVector::default();
        }
    }

    /// Gets a reference to a shifted bundle container.
    #[inline(always)]
    pub unsafe fn get_offset_instance<T>(bundle_container: &T, slot_index: usize) -> &T
    where
        T: Copy,
    {
        let ptr = bundle_container as *const T as *const f32;
        let shifted_ptr = ptr.add(slot_index);
        &*(shifted_ptr as *const T)
    }

    /// Gets a mutable reference to a shifted bundle container.
    #[inline(always)]
    pub unsafe fn get_offset_instance_mut<T>(bundle_container: &mut T, slot_index: usize) -> &mut T 
    where
        T: Copy,
    {
        let ptr = bundle_container as *mut T as *mut f32;
        let shifted_ptr = ptr.add(slot_index);
        &mut *(shifted_ptr as *mut T)
    }

    /// Gets a reference to the first element in the vector reference.
    #[inline(always)]
    pub unsafe fn get_first<T>(vector: &Vector<T>) -> &T
    where
        T: Copy + std::simd::SimdElement,
        std::simd::LaneCount<{ optimal_lanes::<T>() }>: std::simd::SupportedLaneCount,
    {
        let ptr = vector as *const Vector<T> as *const T;
        &*ptr
    }

    /// Gets a mutable reference to the first element in the vector reference.
    #[inline(always)]
    pub unsafe fn get_first_mut<T>(vector: &mut Vector<T>) -> &mut T 
    where
        T: Copy + std::simd::SimdElement,
        std::simd::LaneCount<{ optimal_lanes::<T>() }>: std::simd::SupportedLaneCount,
    {
        let ptr = vector as *mut Vector<T> as *mut T;
        &mut *ptr
    }
}
