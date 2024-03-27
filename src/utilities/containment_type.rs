#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContainmentType {
    /// The objects are separate.
    Disjoint,
    /// One object fully contains the other.
    Contains,
    /// The objects are intersecting, but neither object fully contains the other.
    Intersects,
}
