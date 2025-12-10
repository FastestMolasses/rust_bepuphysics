use crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer;
use crate::utilities::collections::predicate::Predicate;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

/// Provides a default implementation for a reference equality comparer that can be used with any `T` that implements `Hash` and `Eq`.
pub struct DefaultRefEqualityComparer;

impl<T: Hash + Eq> RefEqualityComparer<T> for DefaultRefEqualityComparer {
    fn hash(&self, item: &T) -> i32 {
        let mut hasher = DefaultHasher::new();
        item.hash(&mut hasher);
        hasher.finish() as i32
    }

    fn equals(&self, a: &T, b: &T) -> bool {
        a == b
    }
}

/// IPredicate wrapper around an EqualityComparer and an object to compare against.
pub struct WrapperPredicate<T, C>
where
    C: RefEqualityComparer<T>,
{
    item: T,
    comparer: C,
}

impl<T> WrapperPredicate<T, DefaultRefEqualityComparer>
where
    T: Hash + Eq,
{
    /// Creates a default comparer for the given type.
    pub fn new(item: T) -> Self {
        WrapperPredicate {
            item,
            comparer: DefaultRefEqualityComparer,
        }
    }
}

impl<T, C> WrapperPredicate<T, C>
where
    T: Hash + Eq,
    C: RefEqualityComparer<T>,
{
    /// Creates with a custom comparer.
    pub fn with_comparer(item: T, comparer: C) -> Self {
        WrapperPredicate { item, comparer }
    }
}

impl<T, C> Predicate<T> for WrapperPredicate<T, C>
where
    T: Hash + Eq,
    C: RefEqualityComparer<T>,
{
    fn matches(&self, other_item: &T) -> bool {
        self.comparer.equals(&self.item, other_item)
    }
}
