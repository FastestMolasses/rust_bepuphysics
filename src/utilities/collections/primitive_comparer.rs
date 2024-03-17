use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

// TODO: LOOK INTO `std::mem::transmute;` FOR THIS FILE

// the original code leverages the Unsafe.As method to reinterpret the references of
// generic types (T) as specific primitive types without actual copying or boxing,
// allowing for efficient comparisons and hashing directly on the stack. This approach
// is critical for achieving high performance, especially in scenarios where cache
// locality and avoiding heap allocations are paramount.

/// Provides optimized equality testing, comparison, and hashing for primitive types.
pub struct PrimitiveComparer;

impl PrimitiveComparer {
    pub unsafe fn compare<T: Primitive>(a: &T, b: &T) -> Ordering {
        T::cmp(a, b)
    }

    pub unsafe fn equals<T: Primitive>(a: &T, b: &T) -> bool {
        T::eq(a, b)
    }

    pub unsafe fn hash<T: Primitive, H: Hasher>(item: &T, state: &mut H) {
        T::hash(item, state);
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
