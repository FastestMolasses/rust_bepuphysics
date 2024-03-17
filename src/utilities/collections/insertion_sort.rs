use crate::utilities::collections::comparer_ref::RefComparer;
use std::cmp::Ordering;

// Sort function for keys and values with a custom comparer.
// This function uses unsafe operations to achieve performance characteristics similar to the C# version.
#[inline(always)]
pub unsafe fn sort<TKey, TValue, TComparer>(
    keys: &mut [TKey],
    values: &mut [TValue],
    start: usize,
    inclusive_end: usize,
    comparer: &TComparer,
) where
    TComparer: RefComparer<TKey>,
{
    for i in start + 1..=inclusive_end {
        let original_key = &keys[i];
        let original_value = &values[i];
        let mut compare_index = i;

        while compare_index > start {
            if comparer.compare(&keys[compare_index - 1], original_key) == Ordering::Greater {
                keys.swap(compare_index, compare_index - 1);
                values.swap(compare_index, compare_index - 1);
                compare_index -= 1;
            } else {
                break;
            }
        }
    }
}

// Sort function for keys only with a custom comparer.
#[inline(always)]
pub unsafe fn sort_keys_only<TKey, TComparer>(
    keys: &mut [TKey],
    start: usize,
    inclusive_end: usize,
    comparer: &TComparer,
) where
    TComparer: RefComparer<TKey>,
{
    for i in start + 1..=inclusive_end {
        let original_key = &keys[i];
        let mut compare_index = i;

        while compare_index > start {
            if comparer.compare(&keys[compare_index - 1], original_key) == Ordering::Greater {
                keys.swap(compare_index, compare_index - 1);
                compare_index -= 1;
            } else {
                break;
            }
        }
    }
}
