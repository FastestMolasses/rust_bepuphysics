use core::mem::size_of;
use core::ptr::{read_unaligned, write_unaligned};
use packed_simd::*;
use crate::utilities::bundle_indexing::BundleIndexing;

pub struct GatherScatter;

impl GatherScatter {
    /// Gets a reference to an element from a vector without using pointers, bypassing direct vector access for codegen reasons. This performs no bounds testing!
    #[inline(always)]
    pub unsafe fn get<T>(vector: &Vector<T>, index: usize) -> &T {
        &*((vector as *const Vector<T> as *const T).add(index))
    }

    /// Copies from one bundle lane to another. The bundle must be a contiguous block of Vector types.
    #[inline(always)]
    pub unsafe fn copy_lane<T>(
        source_bundle: &T,
        source_inner_index: usize,
        target_bundle: &mut T,
        target_inner_index: usize,
    ) {
        let size_in_ints = (size_of::<T>() >> 2) & !BundleIndexing::vector_mask();

        let source_base = (source_bundle as *const T as *const i32).add(source_inner_index);
        let target_base = (target_bundle as *mut T as *mut i32).add(target_inner_index);

        write_unaligned(target_base, read_unaligned(source_base));

        let mut offset = 4;

        while offset + 4 * 8 <= size_in_ints {
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
        }

        if offset + 4 * 4 <= size_in_ints {
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
        }

        if offset + 2 * 4 <= size_in_ints {
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
            offset += 4;
        }

        if offset + 4 <= size_in_ints {
            write_unaligned(
                target_base.add(offset),
                read_unaligned(source_base.add(offset)),
            );
        }
    }

    /// Clears a bundle lane using the default value of the specified type. The bundle must be a contiguous block of Vector types, all sharing the same type,
    /// and the first vector must start at the address pointed to by the bundle reference.
    #[inline(always)]
    pub unsafe fn clear_lane<TOuter, TVector>(bundle: &mut TOuter, inner_index: usize)
    where
        TVector: Copy + Default,
    {
        let vector_count = size_of::<TOuter>() / size_of::<Vector<TVector>>();
        let lane_base = (bundle as *mut TOuter as *mut TVector).add(inner_index);

        for i in 0..vector_count {
            write_unaligned(lane_base.add(i * 4), TVector::default());
        }
    }

    /// Gets a reference to a shifted bundle container such that the first slot of each bundle covers the given inner index of the original bundle reference.
    #[inline(always)]
    pub unsafe fn get_offset_instance<T>(bundle_container: &T, inner_index: usize) -> &T {
        &*((bundle_container as *const T as *const f32).add(inner_index) as *const T)
    }

    /// Gets a reference to the first element in the vector reference.
    #[inline(always)]
    pub unsafe fn get_first<T>(vector: &Vector<T>) -> &T {
        &*(vector as *const Vector<T> as *const T)
    }
}
