struct ArrayEnumerable<T> {
    array: &[T], // Using a slice for flexibility
    start: usize,
    end: usize,
}

impl<T> ArrayEnumerable<T> {
    pub fn count(&self) -> usize {
        self.end - self.start
    }

    pub fn new(array: &[T], start: usize, end: usize) -> Self {
        ArrayEnumerable { array, start, end }
    }

    pub fn from_count(array: &[T], count: usize) -> Self {
        ArrayEnumerable {
            array,
            start: 0,
            end: count,
        }
    }

    pub fn from_full_array(array: &[T]) -> Self {
        ArrayEnumerable {
            array,
            start: 0,
            end: array.len(),
        }
    }
}

// Implementing the Iterator trait for ArrayEnumerable
impl<T> IntoIterator for ArrayEnumerable<T> {
    type Item = T;
    type IntoIter = ArrayEnumerator<T>;

    fn into_iter(self) -> Self::IntoIter {
        ArrayEnumerator {
            array: self.array,
            index: self.start - 1,
            end: self.end,
        }
    }
}

struct ArrayEnumerator<T> {
    array: &[T],
    index: usize,
    end: usize,
}

// Implementing the Iterator trait for ArrayEnumerator
impl<T> Iterator for ArrayEnumerator<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        self.index += 1;
        if self.index < self.end {
            // TODO: The value is cloned when returning from next. Depending on the usage, we could avoid this if we return references to elements within the slice
            Some(unsafe { self.array.get_unchecked(self.index) }).cloned()
        } else {
            None
        }
    }
}
