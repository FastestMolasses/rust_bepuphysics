// Translated from BepuPhysics/InvasiveHashDiagnostics.cs

/// Hardcoded hash types used by invasive hash diagnostics.
#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HashDiagnosticType {
    AwakeBodyStates0 = 0,
    AwakeBodyStates1,
    AwakeBodyStates2,
    AwakeBodyStates3,
    AwakeBodyStates4,
    AwakeBodyStates5,
    AwakeBodyStates6,
    AwakeBodyStates7,
    AwakeBodyStates8,
    AwakeBodyStates9,
    AwakeBodyStates10,
    AwakeBodyCollidableStates0,
    AwakeBodyCollidableStates1,
    AwakeBodyCollidableStates2,
    AwakeBodyCollidableStates3,
    AwakeBodyCollidableStates4,
    AwakeBodyCollidableStates5,
    AwakeBodyCollidableStates6,
    AwakeBodyCollidableStates7,
    AwakeBodyCollidableStates8,
    AwakeBodyCollidableStates9,
    AwakeBodyCollidableStates10,
    AddSleepingToActiveForFallback,
    SolverBodyReferenceBeforeCollisionDetection,
    SolverBodyReferenceBeforePreflush,
    SolverBodyReferenceAfterPreflushPhase1,
    SolverBodyReferenceAfterPreflushPhase2,
    SolverBodyReferenceAfterPreflushPhase3,
    SolverBodyReferenceAfterPreflush,
    SolverBodyReferenceBeforeSolver,
    SolverBodyReferenceAfterSolver,
    SolverBodyReferenceAtEnd,
    DeterministicConstraintAdd,
    AddToSimulationSpeculative,
    AddToSimulationSpeculativeFallbackSolverReferences,
    EnqueueStaleRemoval,
    RemoveConstraintsFromFallbackBatchReferencedHandles,
    RemoveConstraintsFromBatchReferencedHandles,
    RemoveConstraintsFromBodyLists,
    RemoveConstraintsFromTypeBatch,
    ReturnConstraintHandles,
    PreflushJobs,
    AllocateInTypeBatchForFallback,
    AllocateInTypeBatchForFallbackProbes,
    AllocateInBatch,
    TypeProcessorRemove,
}

const HASH_TYPE_COUNT: usize = 46;

/// Helper diagnostics class for monitoring internal state determinism across runs.
/// Typically used by inserting tests into engine internals.
pub struct InvasiveHashDiagnostics {
    pub current_run_index: i32,
    pub current_hash_index: i32,
    /// Hashes[run][hash_type][hash_index]
    pub hashes: Vec<Vec<Vec<i32>>>,
}

/// Global singleton instance (matches C# `static Instance`).
static mut INSTANCE: Option<InvasiveHashDiagnostics> = None;

impl InvasiveHashDiagnostics {
    pub fn initialize(run_count: usize, hash_capacity_per_type: usize) {
        let mut instance = InvasiveHashDiagnostics {
            current_run_index: 0,
            current_hash_index: 0,
            hashes: Vec::with_capacity(run_count),
        };
        for _ in 0..run_count {
            let mut run = Vec::with_capacity(HASH_TYPE_COUNT);
            for _ in 0..HASH_TYPE_COUNT {
                run.push(vec![0i32; hash_capacity_per_type]);
            }
            instance.hashes.push(run);
        }
        unsafe {
            INSTANCE = Some(instance);
        }
    }

    pub fn instance() -> Option<&'static mut InvasiveHashDiagnostics> {
        unsafe { INSTANCE.as_mut() }
    }

    pub fn type_is_active(&self, hash_type: HashDiagnosticType) -> bool {
        let run = self.current_run_index as usize;
        let ht = hash_type as usize;
        let hi = self.current_hash_index as usize;
        run < self.hashes.len()
            && ht < self.hashes[run].len()
            && hi < self.hashes[run][ht].len()
    }

    pub fn move_to_next_run(&mut self) {
        self.current_run_index += 1;
        self.current_hash_index = 0;
    }

    pub fn move_to_next_hash_frame(&mut self) {
        let run = self.current_run_index as usize;
        if run >= self.hashes.len() {
            return;
        }
        debug_assert!(
            self.current_hash_index >= 0,
            "Invalid hash index: {}",
            self.current_hash_index
        );
        let hi = self.current_hash_index as usize;
        let mut any_failed = false;
        for hash_type_index in 0..self.hashes[run].len() {
            if hi >= self.hashes[run][hash_type_index].len() {
                continue;
            }
            for previous_run_index in 0..run {
                if self.hashes[run][hash_type_index][hi]
                    != self.hashes[previous_run_index][hash_type_index][hi]
                {
                    eprintln!(
                        "Hash failure on {:?} frame {}, current run {} vs previous {}: {} vs {}.",
                        hash_type_index,
                        hi,
                        run,
                        previous_run_index,
                        self.hashes[run][hash_type_index][hi],
                        self.hashes[previous_run_index][hash_type_index][hi]
                    );
                    any_failed = true;
                }
            }
        }
        if any_failed {
            eprintln!("Hash failure detected in InvasiveHashDiagnostics.");
        }
        self.current_hash_index += 1;
    }

    pub fn get_hash_for_type(&mut self, hash_type: HashDiagnosticType) -> &mut i32 {
        let run = self.current_run_index as usize;
        let ht = hash_type as usize;
        let hi = self.current_hash_index as usize;
        &mut self.hashes[run][ht][hi]
    }

    #[inline(always)]
    pub fn contribute_to_hash_value(hash: &mut i32, value: i32) {
        *hash = Self::rehash(*hash ^ value);
    }

    #[inline(always)]
    pub fn contribute_to_hash_bytes<T: Copy>(hash: &mut i32, value: T) {
        let size = std::mem::size_of::<T>();
        let int_count = size / 4;
        let ptr = &value as *const T as *const i32;
        for i in 0..int_count {
            Self::contribute_to_hash_value(hash, unsafe { *ptr.add(i) });
        }
        let byte_ptr = unsafe { ptr.add(int_count) as *const u8 };
        let byte_remainder = size - int_count * 4;
        for i in 0..byte_remainder {
            Self::contribute_to_hash_value(hash, unsafe { *byte_ptr.add(i) } as i32);
        }
    }

    pub fn contribute_to_hash_type(&mut self, hash_type: HashDiagnosticType, value: i32) {
        let hash = self.get_hash_for_type(hash_type);
        Self::contribute_to_hash_value(hash, value);
    }

    pub fn contribute_to_hash_type_bytes<T: Copy>(&mut self, hash_type: HashDiagnosticType, value: T) {
        let hash = self.get_hash_for_type(hash_type);
        Self::contribute_to_hash_bytes(hash, value);
    }

    #[inline(always)]
    fn rehash(hash: i32) -> i32 {
        // Uses the same hash helper as BepuUtilities.HashHelper.Rehash
        let h = hash as u32;
        let h = h.wrapping_add(h << 15) ^ 0xFFFFFFFF;
        let h = h ^ (h >> 10);
        let h = h.wrapping_add(h << 3);
        let h = h ^ (h >> 6);
        let h = h.wrapping_add(h << 2).wrapping_add(h << 14);
        let h = h ^ (h >> 16);
        h as i32
    }
}
