/// Trait for types that can compare and hash items by reference.
pub trait RefEqualityComparer<T> {
    /// Computes a hash code for the given item.
    fn hash(&self, item: &T) -> i32;
    /// Checks if two items are equal.
    fn equals(&self, a: &T, b: &T) -> bool;
}
