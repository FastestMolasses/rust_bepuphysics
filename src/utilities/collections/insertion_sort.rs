use crate::utilities::collections::comparer_ref::RefComparer;
use std::cmp::Ordering;
use std::ptr;

/// Sort function for keys and values with a custom comparer.
/// This function uses unsafe operations to achieve performance characteristics similar to the C# version.
#[inline(always)]
pub unsafe fn sort<TKey: Copy, TValue: Copy, TComparer>(
    keys: &mut [TKey],
    values: &mut [TValue],
    start: usize,
    inclusive_end: usize,
    comparer: &TComparer,
) where
    TComparer: RefComparer<TKey>,
{
    for i in start + 1..=inclusive_end {
        let original_key = *keys.get_unchecked(i);
        let original_value = *values.get_unchecked(i);
        let mut compare_index = i;

        while compare_index > start {
            let compare_key = keys.get_unchecked(compare_index - 1);
            if comparer.compare(compare_key, &original_key) == Ordering::Greater {
                // Shift elements up
                ptr::copy_nonoverlapping(
                    keys.as_ptr().add(compare_index - 1),
                    keys.as_mut_ptr().add(compare_index),
                    1,
                );
                ptr::copy_nonoverlapping(
                    values.as_ptr().add(compare_index - 1),
                    values.as_mut_ptr().add(compare_index),
                    1,
                );
                compare_index -= 1;
            } else {
                break;
            }
        }

        *keys.get_unchecked_mut(compare_index) = original_key;
        *values.get_unchecked_mut(compare_index) = original_value;
    }
}

/// Sort function for keys only with a custom comparer.
#[inline(always)]
pub unsafe fn sort_keys_only<TKey: Copy, TComparer>(
    keys: &mut [TKey],
    start: usize,
    inclusive_end: usize,
    comparer: &TComparer,
) where
    TComparer: RefComparer<TKey>,
{
    for i in start + 1..=inclusive_end {
        let original_key = *keys.get_unchecked(i);
        let mut compare_index = i;

        while compare_index > start {
            let compare_key = keys.get_unchecked(compare_index - 1);
            if comparer.compare(compare_key, &original_key) == Ordering::Greater {
                // Shift element up
                ptr::copy_nonoverlapping(
                    keys.as_ptr().add(compare_index - 1),
                    keys.as_mut_ptr().add(compare_index),
                    1,
                );
                compare_index -= 1;
            } else {
                break;
            }
        }

        *keys.get_unchecked_mut(compare_index) = original_key;
    }
}
