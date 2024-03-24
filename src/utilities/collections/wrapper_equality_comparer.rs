use std::hash::{Hash, Hasher};
use std::collections::hash_map::DefaultHasher;

pub trait EqualityComparerRef<T> {
    fn equals(&self, a: &T, b: &T) -> bool;
    fn hash(&self, item: &T) -> u64;
}

/// IEqualityComparerRef wrapper around an EqualityComparer.
pub struct WrapperEqualityComparer<T> {
    // This is a simplification, as Rust's standard library does not have an `EqualityComparer` trait like .NET.
    // In Rust, `Eq` and `Hash` traits are used for these purposes.
}

impl<T> WrapperEqualityComparer<T> {
    /// Creates a default comparer for the given type.
    pub fn new() -> Self {
        WrapperEqualityComparer {}
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
