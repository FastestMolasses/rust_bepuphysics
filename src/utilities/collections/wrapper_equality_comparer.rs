use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::sync::Arc;

/// A thread-safe wrapper around a Rust equality comparer.
pub struct WrapperEqualityComparer<T> {
    comparer: Arc<dyn Fn(&T, &T) -> bool + Send + Sync>,
    hasher: Arc<dyn Fn(&T) -> u64 + Send + Sync>,
}

impl<T: 'static + Eq + Hash + Send + Sync> WrapperEqualityComparer<T> {
    /// Creates a new WrapperEqualityComparer using the default equality and hash implementations for the type T.
    pub fn new() -> Self {
        Self {
            comparer: Arc::new(|a: &T, b: &T| a == b),
            hasher: Arc::new(|item: &T| {
                let mut hasher = DefaultHasher::new();
                item.hash(&mut hasher);
                hasher.finish()
            }),
        }
    }

    /// Compares two references of type T for equality.
    pub fn equals(&self, a: &T, b: &T) -> bool {
        (self.comparer)(a, b)
    }

    /// Generates a hash code for a reference of type T.
    pub fn hash(&self, item: &T) -> u64 {
        (self.hasher)(item)
    }
}

// Optional: Implementing Clone for WrapperEqualityComparer to ensure that it can be easily cloned
// while still maintaining thread safety through Arc.
impl<T> Clone for WrapperEqualityComparer<T> {
    fn clone(&self) -> Self {
        Self {
            comparer: Arc::clone(&self.comparer),
            hasher: Arc::clone(&self.hasher),
        }
    }
}
