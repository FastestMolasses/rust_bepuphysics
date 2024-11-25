use std::debug_assert;

/// Pointer to a leaf's tree location.
///
/// The identity of a leaf is implicit in its position within the leaf array.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_leaf() {
        let leaf = Leaf::new(123, 1);
        assert_eq!(leaf.node_index(), 123);
        assert_eq!(leaf.child_index(), 1);

        let leaf = Leaf::new(123, 0);
        assert_eq!(leaf.node_index(), 123);
        assert_eq!(leaf.child_index(), 0);
    }

    #[test]
    #[should_panic(expected = "Binary trees can't have children in slots other than 0 and 1!")]
    fn test_leaf_invalid_child_index() {
        let _leaf = Leaf::new(123, 2);
    }
}
