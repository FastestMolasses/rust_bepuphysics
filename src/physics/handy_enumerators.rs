// Translated from BepuPhysics/HandyEnumerators.cs

use crate::physics::bodies::Bodies;
use crate::utilities::for_each_ref::IForEach;

/// Collects body handles associated with an active constraint as integers.
pub struct ActiveConstraintBodyHandleCollector {
    pub bodies: *const Bodies,
    pub handles: *mut i32,
    pub count: i32,
}

impl ActiveConstraintBodyHandleCollector {
    pub unsafe fn new(bodies: *const Bodies, handles: *mut i32) -> Self {
        Self {
            bodies,
            handles,
            count: 0,
        }
    }
}

impl IForEach<i32> for ActiveConstraintBodyHandleCollector {
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            // Note that this enumerator is used with prefiltered body indices and with raw body
            // indices. A redundant & isn't much of a concern; lets us share more frequently.
            let bodies = &*self.bodies;
            let body_index = encoded_body_index & Bodies::BODY_REFERENCE_MASK;
            *self.handles.offset(self.count as isize) =
                bodies.active_set().index_to_handle.get(body_index).0;
            self.count += 1;
        }
    }
}

/// Collects body handles associated with an active constraint as integers,
/// only for dynamic bodies.
pub struct ActiveConstraintDynamicBodyHandleCollector {
    pub bodies: *const Bodies,
    pub handles: *mut i32,
    pub count: i32,
}

impl ActiveConstraintDynamicBodyHandleCollector {
    pub unsafe fn new(bodies: *const Bodies, handles: *mut i32) -> Self {
        Self {
            bodies,
            handles,
            count: 0,
        }
    }
}

impl IForEach<i32> for ActiveConstraintDynamicBodyHandleCollector {
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            if Bodies::is_encoded_dynamic_reference(encoded_body_index) {
                let bodies = &*self.bodies;
                let body_index = encoded_body_index & Bodies::BODY_REFERENCE_MASK;
                *self.handles.offset(self.count as isize) =
                    bodies.active_set().index_to_handle.get(body_index).0;
                self.count += 1;
            }
        }
    }
}

/// Collects body indices associated with an active constraint.
/// Encoded metadata is stripped.
pub struct ActiveConstraintBodyIndexCollector {
    pub indices: *mut i32,
    pub count: i32,
}

impl ActiveConstraintBodyIndexCollector {
    pub unsafe fn new(indices: *mut i32) -> Self {
        Self { indices, count: 0 }
    }
}

impl IForEach<i32> for ActiveConstraintBodyIndexCollector {
    fn loop_body(&mut self, encoded_body_index: i32) {
        unsafe {
            *self.indices.offset(self.count as isize) =
                encoded_body_index & Bodies::BODY_REFERENCE_MASK;
            self.count += 1;
        }
    }
}

/// Directly reports references as provided by whatever is being enumerated.
/// For example, when used directly with TypeProcessor's EnumerateConnectedRawBodyReferences,
/// if the constraint is active, this will report encoded body indices. If the constraint is
/// sleeping, this will report body handles.
pub struct PassthroughReferenceCollector {
    pub references: *mut i32,
    pub index: i32,
}

impl PassthroughReferenceCollector {
    pub unsafe fn new(references: *mut i32) -> Self {
        Self {
            references,
            index: 0,
        }
    }
}

impl IForEach<i32> for PassthroughReferenceCollector {
    fn loop_body(&mut self, reference: i32) {
        unsafe {
            *self.references.offset(self.index as isize) = reference;
            self.index += 1;
        }
    }
}

/// Collects float values.
pub struct FloatCollector {
    pub values: *mut f32,
    pub index: i32,
}

impl FloatCollector {
    pub unsafe fn new(values: *mut f32) -> Self {
        Self { values, index: 0 }
    }
}

impl IForEach<f32> for FloatCollector {
    fn loop_body(&mut self, value: f32) {
        unsafe {
            *self.values.offset(self.index as isize) = value;
            self.index += 1;
        }
    }
}
