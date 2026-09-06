use std::debug_assert;

/// Pointer to a leaf's tree location.
///
/// The identity of a leaf is implicit in its position within the leaf array.
#[repr(transparent)]
#[derive(Clone, Copy, Debug)]
pub struct Leaf {
    packed: u32,
}

impl Leaf {
    /// Gets the index of the node that the leaf is directly held by.
    #[inline(always)]
    pub fn node_index(&self) -> i32 {
        (self.packed & 0x7FFF_FFFF) as i32
    }

    /// Gets which child within the owning node the leaf is in.
    #[inline(always)]
    pub fn child_index(&self) -> i32 {
        ((self.packed & 0x8000_0000) >> 31) as i32
    }

    /// Creates a new leaf with the given node index and child index.
    #[inline(always)]
    pub fn new(node_index: i32, child_index: i32) -> Self {
        debug_assert!(
            (child_index & !1) == 0,
            "Binary trees can't have children in slots other than 0 and 1!"
        );
        Self {
            packed: ((node_index as u32) & 0x7FFF_FFFF) | ((child_index as u32) << 31),
        }
    }
}

const _: () = assert!(std::mem::size_of::<Leaf>() == 4);
