use std::hash::Hash;

// Newtype Pattern for enhanced type safety.
// repr(transparent): layout identical to i32, so &[XHandle] may be reinterpreted as &[i32].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(transparent)]
pub struct BodyHandle(pub i32);

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(transparent)]
pub struct StaticHandle(pub i32);

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(transparent)]
pub struct ConstraintHandle(pub i32);

const _: () = {
    assert!(
        std::mem::size_of::<BodyHandle>() == std::mem::size_of::<i32>()
            && std::mem::align_of::<BodyHandle>() == std::mem::align_of::<i32>()
    );
    assert!(
        std::mem::size_of::<StaticHandle>() == std::mem::size_of::<i32>()
            && std::mem::align_of::<StaticHandle>() == std::mem::align_of::<i32>()
    );
    assert!(
        std::mem::size_of::<ConstraintHandle>() == std::mem::size_of::<i32>()
            && std::mem::align_of::<ConstraintHandle>() == std::mem::align_of::<i32>()
    );
};

// Simple implementations for Display for user-friendliness
impl std::fmt::Display for BodyHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "BodyHandle<{}>", self.0)
    }
}

impl std::fmt::Display for StaticHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "StaticHandle<{}>", self.0)
    }
}

impl std::fmt::Display for ConstraintHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "ConstraintHandle<{}>", self.0)
    }
}
