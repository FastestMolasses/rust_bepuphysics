use std::cmp::Ordering;

pub trait RefComparer<T> {
    fn compare(&self, a: &T, b: &T) -> Ordering;
}
