use glam::Vec3;

/// A child node in a tree structure, containing bounding box and metadata.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct NodeChild {
    /// Minimum bounds of the bounding box
    pub min: Vec3,
    /// Index for internal tracking
    pub index: i32,
    /// Maximum bounds of the bounding box
    pub max: Vec3,
    /// Number of leaves under this node
    pub leaf_count: i32,
}

/// 2-wide tree node.
///
/// Note that the format of this node implies that we don't explicitly test against the root bounding box during normal execution.
/// For almost all broad phase use cases, queries will be inside the root bounding box anyway. For non-broad phase uses, the outer bounding box will likely be stored
/// elsewhere- for example, in the broad phase.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub a: NodeChild,
    pub b: NodeChild,
}

/// Metadata associated with a 2-child tree node.
///
/// Node metadata isn't required or used during collision testing, so it is stored separately.
/// This helps avoid splitting Nodes across cache lines and decreases memory bandwidth requirements during testing.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Metanode {
    pub parent: i32,
    pub index_in_parent: i32,
    /// Either refine_flag or local_cost_change depending on context
    pub cost_or_flag: CostOrFlag,
}

/// Union of refine flag and local cost change.
/// The local cost change is unioned with the refine flags. They're never used simultaneously.
/// This will be overwritten right after use, so don't expect anything meaningful here outside of refinement scheduling's scope.
#[repr(C)]
#[derive(Copy, Clone)]
pub union CostOrFlag {
    pub refine_flag: i32,
    pub local_cost_change: f32,
}

impl std::fmt::Debug for CostOrFlag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "CostOrFlag {{ ... }}")
    }
}

// Verify sizes and alignments match C#
#[cfg(test)]
mod tests {
    use super::*;
    use std::mem;

    #[test]
    fn test_sizes() {
        assert_eq!(mem::size_of::<NodeChild>(), 32);
        assert_eq!(mem::size_of::<Node>(), 64);
        assert_eq!(mem::size_of::<Metanode>(), 12);
        assert_eq!(mem::size_of::<CostOrFlag>(), 4);
    }

    #[test]
    fn test_alignments() {
        assert_eq!(mem::align_of::<NodeChild>(), 4);
        assert_eq!(mem::align_of::<Node>(), 4);
        assert_eq!(mem::align_of::<Metanode>(), 4);
        assert_eq!(mem::align_of::<CostOrFlag>(), 4);
    }

    #[test]
    fn test_offsets() {
        let node_child_offsets = unsafe {
            (
                &(*(0 as *const NodeChild)).min as *const _ as usize,
                &(*(0 as *const NodeChild)).index as *const _ as usize,
                &(*(0 as *const NodeChild)).max as *const _ as usize,
                &(*(0 as *const NodeChild)).leaf_count as *const _ as usize,
            )
        };
        assert_eq!(node_child_offsets.0, 0);
        assert_eq!(node_child_offsets.1, 12);
        assert_eq!(node_child_offsets.2, 16);
        assert_eq!(node_child_offsets.3, 28);

        let node_offsets = unsafe {
            (
                &(*(0 as *const Node)).a as *const _ as usize,
                &(*(0 as *const Node)).b as *const _ as usize,
            )
        };
        assert_eq!(node_offsets.0, 0);
        assert_eq!(node_offsets.1, 32);

        let metanode_offsets = unsafe {
            (
                &(*(0 as *const Metanode)).parent as *const _ as usize,
                &(*(0 as *const Metanode)).index_in_parent as *const _ as usize,
                &(*(0 as *const Metanode)).cost_or_flag as *const _ as usize,
            )
        };
        assert_eq!(metanode_offsets.0, 0);
        assert_eq!(metanode_offsets.1, 4);
        assert_eq!(metanode_offsets.2, 8);
    }
}
