/// Location in memory where a constraint is stored.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ConstraintLocation {
    /// Index of the constraint set that owns the constraint. If zero, the constraint is attached to bodies that are awake.
    pub set_index: i32,
    /// Index of the constraint batch the constraint belongs to.
    pub batch_index: i32,
    /// Type id of the constraint. Used to look up the type batch index in a constraint batch's type id to type batch index table.
    pub type_id: i32,
    /// Index of the constraint in a type batch.
    pub index_in_type_batch: i32,
}
