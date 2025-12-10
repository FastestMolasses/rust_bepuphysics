//! Array enumeration utilities for iterating over slices with custom bounds.

/// An enumerable view over a slice with custom start/end bounds.
pub struct ArrayEnumerable<'a, T> {
    array: &'a [T],
    start: usize,
    end: usize,
}

impl<'a, T> ArrayEnumerable<'a, T> {
    /// Returns the number of elements in the enumerable range.
    pub fn count(&self) -> usize {
        self.end - self.start
    }

    /// Creates a new ArrayEnumerable with custom start and end bounds.
    pub fn new(array: &'a [T], start: usize, end: usize) -> Self {
        ArrayEnumerable { array, start, end }
    }

    /// Creates a new ArrayEnumerable from the start up to the given count.
    pub fn from_count(array: &'a [T], count: usize) -> Self {
        ArrayEnumerable {
            array,
            start: 0,
            end: count,
        }
    }

    /// Creates a new ArrayEnumerable over the full array.
    pub fn from_full_array(array: &'a [T]) -> Self {
        ArrayEnumerable {
            array,
            start: 0,
            end: array.len(),
        }
    }
}

impl<'a, T> IntoIterator for ArrayEnumerable<'a, T> {
    type Item = &'a T;
    type IntoIter = ArrayEnumerator<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        ArrayEnumerator {
            array: self.array,
            index: self.start,
            end: self.end,
        }
    }
}

/// An iterator over a slice with custom bounds.
pub struct ArrayEnumerator<'a, T> {
    array: &'a [T],
    index: usize,
    end: usize,
}

impl<'a, T> Iterator for ArrayEnumerator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.end {
            let item = unsafe { self.array.get_unchecked(self.index) };
            self.index += 1;
            Some(item)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.end.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<'a, T> ExactSizeIterator for ArrayEnumerator<'a, T> {}
