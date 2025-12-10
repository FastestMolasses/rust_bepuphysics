use crate::utilities::collections::comparer_ref::RefComparer;
use crate::utilities::collections::insertion_sort;
use std::cmp::Ordering;

/// Quicksort implementation using median-of-3 pivot selection with insertion sort fallback.
pub struct Quicksort;

impl Quicksort {
    /// Threshold below which insertion sort is used instead of quicksort.
    const INSERTION_SORT_THRESHOLD: i32 = 30;

    /// Swaps two elements in the keys and values arrays.
    #[inline(always)]
    fn swap<TKey: Copy, TValue: Copy>(keys: &mut [TKey], values: &mut [TValue], i: i32, j: i32) {
        let i = i as usize;
        let j = j as usize;
        keys.swap(i, j);
        values.swap(i, j);
    }

    /// Swaps two elements in the keys array.
    #[inline(always)]
    fn swap_keys<TKey: Copy>(keys: &mut [TKey], i: i32, j: i32) {
        keys.swap(i as usize, j as usize);
    }

    /// Finds the index of the median-of-3 pivot for better partitioning.
    #[inline(always)]
    fn find_mo3_index<TKey: Copy, TComparer: RefComparer<TKey>>(
        keys: &[TKey],
        l: i32,
        r: i32,
        comparer: &TComparer,
    ) -> i32 {
        let m = l + (r - l) / 2;
        let l_key = &keys[l as usize];
        let m_key = &keys[m as usize];
        let r_key = &keys[r as usize];

        // Find median of l, m, r
        if comparer.compare(l_key, m_key) == Ordering::Less {
            if comparer.compare(m_key, r_key) == Ordering::Less {
                m
            } else if comparer.compare(l_key, r_key) == Ordering::Less {
                r
            } else {
                l
            }
        } else if comparer.compare(l_key, r_key) == Ordering::Less {
            l
        } else if comparer.compare(m_key, r_key) == Ordering::Less {
            r
        } else {
            m
        }
    }

    /// Sorts keys and values together.
    /// Uses quicksort with median-of-3 pivot selection and insertion sort for small partitions.
    pub fn sort<TKey: Copy, TValue: Copy, TComparer: RefComparer<TKey>>(
        keys: &mut [TKey],
        values: &mut [TValue],
        l: i32,
        r: i32,
        comparer: &TComparer,
    ) {
        if r - l <= Self::INSERTION_SORT_THRESHOLD {
            // Use insertion sort for small ranges
            unsafe {
                insertion_sort::sort(keys, values, l as usize, r as usize, comparer);
            }
            return;
        }

        // Use MO3 to find a pivot
        let pivot_index = Self::find_mo3_index(keys, l, r, comparer);
        let pivot = keys[pivot_index as usize];
        let pivot_value = values[pivot_index as usize];

        // Move pivot value to position r, leaving r unused for partitioning
        keys[pivot_index as usize] = keys[r as usize];
        values[pivot_index as usize] = values[r as usize];

        let mut i = l - 1;
        let mut j = r;

        loop {
            // Claim elements from the left that are less than pivot
            loop {
                i += 1;
                if comparer.compare(&keys[i as usize], &pivot) != Ordering::Less {
                    break;
                }
            }

            // Claim elements from the right that are greater than pivot
            loop {
                j -= 1;
                if comparer.compare(&pivot, &keys[j as usize]) != Ordering::Less || j <= i {
                    break;
                }
            }

            // If the claims have met, partitioning is complete
            if i >= j {
                break;
            }

            // Swap elements that are on the wrong side
            Self::swap(keys, values, i, j);
        }

        // Reintroduce the pivot
        keys[r as usize] = keys[i as usize];
        values[r as usize] = values[i as usize];
        keys[i as usize] = pivot;
        values[i as usize] = pivot_value;

        // Recursively sort the partitions
        let j = i - 1;
        let i = i + 1;

        if j > l {
            Self::sort(keys, values, l, j, comparer);
        }
        if r > i {
            Self::sort(keys, values, i, r, comparer);
        }
    }

    /// Sorts keys only (no associated values).
    /// Uses quicksort with median-of-3 pivot selection and insertion sort for small partitions.
    pub fn sort_keys<TKey: Copy, TComparer: RefComparer<TKey>>(
        keys: &mut [TKey],
        l: i32,
        r: i32,
        comparer: &TComparer,
    ) {
        if r - l <= Self::INSERTION_SORT_THRESHOLD {
            // Use insertion sort for small ranges
            unsafe {
                insertion_sort::sort_keys_only(keys, l as usize, r as usize, comparer);
            }
            return;
        }

        // Use MO3 to find a pivot
        let pivot_index = Self::find_mo3_index(keys, l, r, comparer);
        let pivot = keys[pivot_index as usize];

        // Move pivot value to position r, leaving r unused for partitioning
        keys[pivot_index as usize] = keys[r as usize];

        let mut i = l - 1;
        let mut j = r;

        loop {
            // Claim elements from the left that are less than pivot
            loop {
                i += 1;
                if comparer.compare(&keys[i as usize], &pivot) != Ordering::Less {
                    break;
                }
            }

            // Claim elements from the right that are greater than pivot
            loop {
                j -= 1;
                if comparer.compare(&pivot, &keys[j as usize]) != Ordering::Less || j <= i {
                    break;
                }
            }

            // If the claims have met, partitioning is complete
            if i >= j {
                break;
            }

            // Swap elements that are on the wrong side
            Self::swap_keys(keys, i, j);
        }

        // Reintroduce the pivot
        keys[r as usize] = keys[i as usize];
        keys[i as usize] = pivot;

        // Recursively sort the partitions
        let j = i - 1;
        let i = i + 1;

        if j > l {
            Self::sort_keys(keys, l, j, comparer);
        }
        if r > i {
            Self::sort_keys(keys, i, r, comparer);
        }
    }

    /// Sorts a slice using the natural ordering.
    /// Convenience wrapper that creates a default comparer.
    pub fn sort_slice<TKey: Copy + Ord>(keys: &mut [TKey]) {
        if keys.len() <= 1 {
            return;
        }
        let comparer = OrdComparer::<TKey>::new();
        Self::sort_keys(keys, 0, (keys.len() - 1) as i32, &comparer);
    }

    /// Sorts keys and values together using the natural key ordering.
    pub fn sort_slice_with_values<TKey: Copy + Ord, TValue: Copy>(
        keys: &mut [TKey],
        values: &mut [TValue],
    ) {
        if keys.len() <= 1 {
            return;
        }
        let comparer = OrdComparer::<TKey>::new();
        Self::sort(keys, values, 0, (keys.len() - 1) as i32, &comparer);
    }
}

/// A comparer that uses the Ord trait for comparison.
pub struct OrdComparer<T> {
    _marker: std::marker::PhantomData<T>,
}

impl<T> OrdComparer<T> {
    #[inline(always)]
    pub fn new() -> Self {
        Self {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<T> Default for OrdComparer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Ord> RefComparer<T> for OrdComparer<T> {
    #[inline(always)]
    fn compare(&self, a: &T, b: &T) -> Ordering {
        Ord::cmp(a, b)
    }
}
