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
    /// Uses `<=` comparisons matching C# to ensure stable pivot selection for equal elements.
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

        // Find median of l, m, r using <= (matching C#'s `Compare(...) <= 0`)
        if comparer.compare(l_key, m_key) != Ordering::Greater
            && comparer.compare(l_key, r_key) != Ordering::Greater
        {
            // l is lowest
            if comparer.compare(m_key, r_key) != Ordering::Greater {
                m
            } else {
                r
            }
        } else if comparer.compare(m_key, r_key) != Ordering::Greater {
            // m is lowest
            if comparer.compare(l_key, r_key) != Ordering::Greater {
                l
            } else {
                r
            }
        } else {
            // r is lowest
            if comparer.compare(l_key, m_key) != Ordering::Greater {
                l
            } else {
                m
            }
        }
    }

    /// Sorts keys and values using Bentley-McIlroy 3-way partitioning.
    /// This avoids performance drops in corner cases with many duplicate keys.
    pub fn sort_with_three_way_partitioning<TKey: Copy, TValue: Copy, TComparer: RefComparer<TKey>>(
        keys: &mut [TKey],
        values: &mut [TValue],
        l: i32,
        r: i32,
        comparer: &TComparer,
    ) {
        if r - l <= Self::INSERTION_SORT_THRESHOLD {
            unsafe {
                insertion_sort::sort(keys, values, l as usize, r as usize, comparer);
            }
            return;
        }

        let pivot_index = Self::find_mo3_index(keys, l, r, comparer);
        let pivot = keys[pivot_index as usize];
        let pivot_value = values[pivot_index as usize];

        // Move pivot to position r
        keys[pivot_index as usize] = keys[r as usize];
        values[pivot_index as usize] = values[r as usize];

        let mut i = l - 1;
        let mut j = r;
        let mut p = l - 1;
        let mut q = r;

        if r <= l {
            return;
        }

        loop {
            // Claim from left: find first >= pivot
            loop {
                i += 1;
                if comparer.compare(&keys[i as usize], &pivot) != Ordering::Less {
                    break;
                }
            }
            // Claim from right: find first <= pivot
            loop {
                j -= 1;
                if comparer.compare(&pivot, &keys[j as usize]) != Ordering::Less || j == l {
                    break;
                }
            }

            if i >= j {
                break;
            }

            // Swap elements on wrong sides
            Self::swap(keys, values, i, j);

            // Track equal elements at edges for later consolidation
            if comparer.compare(&keys[i as usize], &pivot) == Ordering::Equal {
                p += 1;
                Self::swap(keys, values, p, i);
            }
            if comparer.compare(&pivot, &keys[j as usize]) == Ordering::Equal {
                q -= 1;
                Self::swap(keys, values, j, q);
            }
        }

        // Reintroduce pivot
        keys[r as usize] = keys[i as usize];
        values[r as usize] = values[i as usize];
        keys[i as usize] = pivot;
        values[i as usize] = pivot_value;

        j = i - 1;
        i = i + 1;

        // Move equal elements from edges to center
        let mut k = l;
        while k < p {
            Self::swap(keys, values, k, j);
            k += 1;
            j -= 1;
        }
        k = r - 1;
        while k > q {
            Self::swap(keys, values, i, k);
            i += 1;
            k -= 1;
        }

        // Recurse on partitions excluding equal-to-pivot elements
        Self::sort_with_three_way_partitioning(keys, values, l, j, comparer);
        Self::sort_with_three_way_partitioning(keys, values, i, r, comparer);
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
