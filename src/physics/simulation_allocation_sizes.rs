/// The common set of allocation sizes for a simulation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SimulationAllocationSizes {
    /// The number of bodies to allocate space for.
    pub bodies: i32,
    /// The number of statics to allocate space for.
    pub statics: i32,
    /// The number of inactive islands to allocate space for.
    pub islands: i32,
    /// Minimum number of shapes to allocate space for in each shape type batch.
    ///
    /// Unused if a `Shapes` instance was directly provided to the `Simulation` constructor.
    pub shapes_per_type: i32,
    /// The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes,
    /// only the solver-level constraint handle storage.
    pub constraints: i32,
    /// The minimum number of constraints to allocate space for in each individual type batch.
    /// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
    /// The number of constraints can vary greatly across types — there are usually far more contacts than ragdoll constraints.
    /// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.
    pub constraints_per_type_batch: i32,
    /// The minimum number of constraints to allocate space for in each body's constraint list.
    /// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
    pub constraint_count_per_body_estimate: i32,
}

impl SimulationAllocationSizes {
    /// Constructs a description of simulation allocations.
    pub fn new(
        bodies: i32,
        statics: i32,
        islands: i32,
        shapes_per_type: i32,
        constraints: i32,
        constraints_per_type_batch: i32,
        constraint_count_per_body_estimate: i32,
    ) -> Self {
        Self {
            bodies,
            statics,
            islands,
            shapes_per_type,
            constraints,
            constraints_per_type_batch,
            constraint_count_per_body_estimate,
        }
    }
}

impl Default for SimulationAllocationSizes {
    fn default() -> Self {
        Self {
            bodies: 0,
            statics: 0,
            islands: 0,
            shapes_per_type: 0,
            constraints: 0,
            constraints_per_type_batch: 0,
            constraint_count_per_body_estimate: 0,
        }
    }
}
