use crate::utilities::vector::Vector;
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
    {
        &*vector.as_ptr().add(index)
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
        let size_in_ints =
            (mem::size_of::<T>() >> 2) & !crate::utilities::bundle_indexing::VECTOR_MASK;

        let source_base = (source_bundle as *const T as *const i32).add(source_inner_index);
        let target_base = (target_bundle as *mut T as *mut i32).add(target_inner_index);

        ptr::copy_nonoverlapping(source_base, target_base, Vector::<i32>::LANES);

        let mut offset = Vector::<i32>::LANES;
        while offset + Vector::<i32>::LANES * 8 <= size_in_ints {
            for _ in 0..8 {
                ptr::copy_nonoverlapping(
                    source_base.add(offset),
                    target_base.add(offset),
                    Vector::<i32>::LANES,
                );
                offset += Vector::<i32>::LANES;
            }
        }
        if offset + 4 * Vector::<i32>::LANES <= size_in_ints {
            for _ in 0..4 {
                ptr::copy_nonoverlapping(
                    source_base.add(offset),
                    target_base.add(offset),
                    Vector::<i32>::LANES,
                );
                offset += Vector::<i32>::LANES;
            }
        }
        if offset + 2 * Vector::<i32>::LANES <= size_in_ints {
            for _ in 0..2 {
                ptr::copy_nonoverlapping(
                    source_base.add(offset),
                    target_base.add(offset),
                    Vector::<i32>::LANES,
                );
                offset += Vector::<i32>::LANES;
            }
        }
        if offset + Vector::<i32>::LANES <= size_in_ints {
            ptr::copy_nonoverlapping(
                source_base.add(offset),
                target_base.add(offset),
                Vector::<i32>::LANES,
            );
        }
    }

    /// Clears a bundle lane using the default value of the specified type.
    #[inline(always)]
    pub unsafe fn clear_lane<TOuter, TVector>(bundle: &mut TOuter, inner_index: usize)
    where
        TOuter: Copy,
        TVector: Copy + Default,
    {
        let vector_count = mem::size_of::<TOuter>() / mem::size_of::<Vector<TVector>>();
        let lane_base = (bundle as *mut TOuter as *mut TVector).add(inner_index);
        for i in 0..vector_count {
            *lane_base.add(i * Vector::<TVector>::LANES) = TVector::default();
        }
    }

    /// Gets a reference to a shifted bundle container.
    #[inline(always)]
    pub unsafe fn get_offset_instance<T>(bundle_container: &T, inner_index: usize) -> &T
    where
        T: Copy,
    {
        // TODO:
    }

    /// Gets a reference to the first element in the vector reference.
    #[inline(always)]
    pub unsafe fn get_first<T>(vector: &Vector<T>) -> &T
    where
        T: Copy,
    {
        &*(vector as *const Vector<T> as *const T)
    }
}
