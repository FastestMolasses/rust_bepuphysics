use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::ptr;

pub struct ReferenceComparer<T> {
    _marker: std::marker::PhantomData<*const T>,
}

impl<T> ReferenceComparer<T> {
    pub fn new() -> Self {
        ReferenceComparer {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<T> ReferenceComparer<T> {
    pub fn equals(&self, a: *const T, b: *const T) -> bool {
        ptr::eq(a, b)
    }

    pub fn hash(&self, item: *const T) -> u64 {
        let mut hasher = DefaultHasher::new();
        (item as *const () as usize).hash(&mut hasher);
        hasher.finish()
    }
}

unsafe impl<T> Send for ReferenceComparer<T> {}
unsafe impl<T> Sync for ReferenceComparer<T> {}
