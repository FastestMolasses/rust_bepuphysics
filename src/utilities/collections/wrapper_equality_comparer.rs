use crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

/// IEqualityComparerRef wrapper around an EqualityComparer.
pub struct WrapperEqualityComparer<T> {
    _phantom: PhantomData<T>,
}

impl<T> WrapperEqualityComparer<T> {
    /// Creates a default comparer for the given type.
    pub fn new() -> Self {
        WrapperEqualityComparer {
            _phantom: PhantomData,
        }
    }
}

impl<T> Default for WrapperEqualityComparer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Eq + Hash> RefEqualityComparer<T> for WrapperEqualityComparer<T> {
    fn hash(&self, item: &T) -> i32 {
        let mut hasher = DefaultHasher::new();
        item.hash(&mut hasher);
        hasher.finish() as i32
    }

    fn equals(&self, a: &T, b: &T) -> bool {
        a == b
    }
}
