pub trait RefEqualityComparer<T> {
    fn hash(&self, item: &T) -> u64;
    fn equals(&self, a: &T, b: &T) -> bool;
}
