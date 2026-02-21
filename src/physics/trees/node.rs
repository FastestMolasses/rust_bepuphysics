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
