use std::fmt;
use std::hash::{Hash, Hasher};

/// Represents an index with an associated type packed into a single integer.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TypedIndex {
    /// Bit packed representation of the typed index.
    pub packed: u32,
}

impl TypedIndex {
    /// Gets the type index of the object.
    #[inline(always)]
    pub fn type_id(&self) -> i32 {
        ((self.packed & 0x7F000000) >> 24) as i32
    }

    /// Gets the index of the object.
    #[inline(always)]
    pub fn index(&self) -> i32 {
        (self.packed & 0x00FFFFFF) as i32
    }

    /// Gets whether this index actually refers to anything.
    /// The Type and Index should only be used if this is true.
    #[inline(always)]
    pub fn exists(&self) -> bool {
        (self.packed & (1 << 31)) > 0
    }

    /// Creates a new TypedIndex.
    #[inline(always)]
    pub fn new(type_id: i32, index: i32) -> Self {
        debug_assert!(
            type_id >= 0 && type_id < 128,
            "Do you really have that many type indices, or is the index corrupt?"
        );
        debug_assert!(
            index >= 0 && index < (1 << 24),
            "Do you really have that many instances, or is the index corrupt?"
        );
        // Note the inclusion of a set bit in the most significant slot.
        // This encodes that the index was explicitly constructed, so it is a 'real' reference.
        // A default constructed TypedIndex will have a 0 in the MSB, so we can use default for empty references.
        Self {
            packed: ((type_id as u32) << 24) | (index as u32) | (1u32 << 31),
        }
    }
}

impl Default for TypedIndex {
    fn default() -> Self {
        Self { packed: 0 }
    }
}

impl PartialEq for TypedIndex {
    fn eq(&self, other: &Self) -> bool {
        self.packed == other.packed
    }
}

impl Eq for TypedIndex {}

impl Hash for TypedIndex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.packed.hash(state);
    }
}

impl fmt::Display for TypedIndex {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "<{}, {}>", self.type_id(), self.index())
    }
}
