/// Defines a type able to match an element.
pub trait Predicate<T> {
    // We're assuming here that the inlining will be good enough that we won't pay extra for passing by ref under any circumstance. This isn't always the case.
    fn matches(&self, item: &T) -> bool;
}
