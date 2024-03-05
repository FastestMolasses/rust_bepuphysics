use std::hash::Hash;

// Newtype Pattern for enhanced type safety
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct BodyHandle(pub i32);

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct StaticHandle(pub i32);

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub struct ConstraintHandle(pub i32);

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
