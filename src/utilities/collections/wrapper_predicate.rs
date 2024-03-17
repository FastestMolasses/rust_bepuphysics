use crate::utilities::collections::predicate::Predicate;
use std::hash::Hash;
use std::sync::Arc;

/// A structure that holds a value of type `T` and compares other values of type `T` against it for equality.
pub struct WrapperPredicate<T> {
    item: T,
    comparer: Arc<dyn Fn(&T, &T) -> bool + Send + Sync>,
}

impl<T: 'static + Eq + Hash + Send + Sync> WrapperPredicate<T> {
    /// Creates a new `WrapperPredicate` with the default equality comparer.
    pub fn new(item: T) -> Self {
        Self {
            item,
            comparer: Arc::new(|a: &T, b: &T| a == b),
        }
    }
}

impl<T: 'static + Eq + Hash + Send + Sync> Predicate<T> for WrapperPredicate<T> {
    fn matches(&self, other_item: &T) -> bool {
        (self.comparer)(&self.item, other_item)
    }
}

// Optional: Implementing Clone for `WrapperPredicate` to ensure it can be easily duplicated.
// This is particularly useful if the predicate needs to be shared across threads.
impl<T: Clone> Clone for WrapperPredicate<T> {
    fn clone(&self) -> Self {
        Self {
            item: self.item.clone(),
            comparer: Arc::clone(&self.comparer),
        }
    }
}
