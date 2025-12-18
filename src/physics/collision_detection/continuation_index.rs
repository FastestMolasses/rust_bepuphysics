// Translated from BepuPhysics/CollisionDetection/ContinuationIndex.cs

use std::fmt;

/// A packed index referring to a CCD continuation.
/// From least to most significant: 30 bits index, 1 bit type, 1 bit 'exists' flag.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct CCDContinuationIndex {
    pub packed: u32,
}

impl CCDContinuationIndex {
    /// Gets the index of the continuation.
    #[inline(always)]
    pub fn index(&self) -> i32 {
        (self.packed & 0x3FFFFFFF) as i32
    }

    /// Gets the type index of the continuation.
    #[inline(always)]
    pub fn type_index(&self) -> i32 {
        ((self.packed >> 30) & 1) as i32
    }

    /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
    #[inline(always)]
    pub fn exists(&self) -> bool {
        (self.packed & (1 << 31)) > 0
    }

    /// Creates a new CCD continuation index.
    #[inline(always)]
    pub fn new(type_index: i32, index: i32) -> Self {
        debug_assert!(
            type_index >= 0 && type_index < 2,
            "Do you really have that many type indices, or is the index corrupt?"
        );
        debug_assert!(
            index >= 0 && index < (1 << 30),
            "Do you really have that many instances, or is the index corrupt?"
        );
        // Note the inclusion of a set bit in the most significant slot.
        // This encodes that the index was explicitly constructed, so it is a 'real' reference.
        // A default constructed TypeIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
        Self {
            packed: ((type_index as u32) << 30) | (index as u32) | (1u32 << 31),
        }
    }

    /// Creates a CCD continuation index from a raw packed value.
    #[inline(always)]
    pub fn from_packed(packed: i32) -> Self {
        Self {
            packed: packed as u32,
        }
    }
}

impl fmt::Display for CCDContinuationIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<{}, {}>", self.type_index(), self.index())
    }
}
