use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;

pub trait EqualityComparerRef<T> {
    fn equals(&self, a: &T, b: &T) -> bool;
    fn hash(&self, item: &T) -> u64;
}

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

impl<T: Eq + Hash> EqualityComparerRef<T> for WrapperEqualityComparer<T> {
    fn equals(&self, a: &T, b: &T) -> bool {
        a == b
    }

    fn hash(&self, item: &T) -> u64 {
        let mut hasher = DefaultHasher::new();
        item.hash(&mut hasher);
        hasher.finish()
    }
}
