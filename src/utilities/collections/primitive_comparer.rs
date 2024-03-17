use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;
use std::mem::transmute;

// the original code leverages the Unsafe.As method to reinterpret the references of
// generic types (T) as specific primitive types without actual copying or boxing,
// allowing for efficient comparisons and hashing directly on the stack. This approach
// is critical for achieving high performance, especially in scenarios where cache
// locality and avoiding heap allocations are paramount.

/// Provides optimized equality testing, comparison, and hashing for primitive types.
pub struct PrimitiveComparer<T> {
    _marker: PhantomData<T>,
}

impl<T> PrimitiveComparer<T> {
    pub fn new() -> Self {
        PrimitiveComparer {
            _marker: PhantomData,
        }
    }

    /// SAFETY: This function assumes that T is a primitive type that can be safely reinterpreted to U.
    pub unsafe fn compare<U>(&self, a: &T, b: &T) -> Ordering
    where
        U: Ord,
    {
        let a_transmuted: &U = transmute(a);
        let b_transmuted: &U = transmute(b);
        a_transmuted.cmp(b_transmuted)
    }

    /// SAFETY: This function assumes that T is a primitive type that can be safely reinterpreted to U.
    pub unsafe fn equals<U>(&self, a: &T, b: &T) -> bool
    where
        U: PartialEq,
    {
        let a_transmuted: &U = transmute(a);
        let b_transmuted: &U = transmute(b);
        a_transmuted == b_transmuted
    }

    /// SAFETY: This function assumes that T is a primitive type that can be safely reinterpreted to U.
    pub unsafe fn hash<U, H: Hasher>(&self, item: &T, state: &mut H)
    where
        U: Hash,
    {
        let item_transmuted: &U = transmute(item);
        item_transmuted.hash(state);
    }
}

pub trait Primitive {
    unsafe fn cmp(&self, other: &Self) -> Ordering;
    unsafe fn eq(&self, other: &Self) -> bool;
    unsafe fn hash<H: Hasher>(&self, state: &mut H);
}

macro_rules! impl_primitive {
    ($t:ty, $as_type:ty) => {
        impl Primitive for $t {
            unsafe fn cmp(&self, other: &Self) -> Ordering {
                let a = *(self as *const $t as *const $as_type);
                let b = *(other as *const $t as *const $as_type);
                a.cmp(&b)
            }

            unsafe fn eq(&self, other: &Self) -> bool {
                let a = *(self as *const $t as *const $as_type);
                let b = *(other as *const $t as *const $as_type);
                a == b
            }

            unsafe fn hash<H: Hasher>(&self, state: &mut H) {
                let val = *(self as *const $t as *const $as_type);
                val.hash(state);
            }
        }
    };
}

impl_primitive!(bool, u8);
impl_primitive!(u8, u8);
impl_primitive!(i8, i8);
impl_primitive!(u16, u16);
impl_primitive!(i16, i16);
impl_primitive!(u32, u32);
impl_primitive!(i32, i32);
impl_primitive!(u64, u64);
impl_primitive!(i64, i64);
impl_primitive!(usize, usize);
impl_primitive!(isize, isize);
impl_primitive!(f32, u32);
impl_primitive!(f64, u64);
impl_primitive!(char, char);

// TODO: TESTS
#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::hash_map::DefaultHasher;
    use std::hash::Hasher;

    #[test]
    fn test_compare_integers() {
        let comparer: PrimitiveComparer<i32> = PrimitiveComparer::new();
        unsafe {
            assert_eq!(comparer.compare::<i32>(&5, &10), Ordering::Less);
            assert_eq!(comparer.compare::<i32>(&10, &5), Ordering::Greater);
            assert_eq!(comparer.compare::<i32>(&10, &10), Ordering::Equal);
        }
    }

    #[test]
    fn test_equals_floats() {
        let comparer: PrimitiveComparer<f32> = PrimitiveComparer::new();
        unsafe {
            assert!(comparer.equals::<f32>(&1.0, &1.0));
            assert!(!comparer.equals::<f32>(&1.0, &1.1));
        }
    }

    #[test]
    fn test_hash_bool() {
        let comparer: PrimitiveComparer<bool> = PrimitiveComparer::new();
        let mut hasher_true = DefaultHasher::new();
        let mut hasher_false = DefaultHasher::new();

        unsafe {
            comparer.hash::<bool, _>(&true, &mut hasher_true);
            comparer.hash::<bool, _>(&false, &mut hasher_false);
        }

        let hash_true = hasher_true.finish();
        let hash_false = hasher_false.finish();

        assert_ne!(hash_true, hash_false);
    }

    #[test]
    fn test_compare_chars() {
        let comparer: PrimitiveComparer<char> = PrimitiveComparer::new();
        unsafe {
            assert_eq!(comparer.compare::<char>(&'a', &'b'), Ordering::Less);
            assert_eq!(comparer.compare::<char>(&'c', &'b'), Ordering::Greater);
            assert_eq!(comparer.compare::<char>(&'a', &'a'), Ordering::Equal);
        }
    }

    #[test]
    fn test_equals_bytes() {
        let comparer: PrimitiveComparer<u8> = PrimitiveComparer::new();
        unsafe {
            assert!(comparer.equals::<u8>(&0x0F, &0x0F));
            assert!(!comparer.equals::<u8>(&0x0F, &0xF0));
        }
    }
}
