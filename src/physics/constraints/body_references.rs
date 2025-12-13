// Translated from BepuPhysics/Constraints/TwoBodyTypeProcessor.cs (body references struct only)
// and ThreeBodyTypeProcessor.cs, FourBodyTypeProcessor.cs

use crate::utilities::vector::Vector;

/// A two-body constraint's body references in AOSOA layout.
/// Stored separately from the iteration data since it is accessed by both the prestep and solve.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct TwoBodyReferences {
    pub index_a: Vector<i32>,
    pub index_b: Vector<i32>,
}

/// A three-body constraint's body references in AOSOA layout.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ThreeBodyReferences {
    pub index_a: Vector<i32>,
    pub index_b: Vector<i32>,
    pub index_c: Vector<i32>,
}

/// A four-body constraint's body references in AOSOA layout.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct FourBodyReferences {
    pub index_a: Vector<i32>,
    pub index_b: Vector<i32>,
    pub index_c: Vector<i32>,
    pub index_d: Vector<i32>,
}
