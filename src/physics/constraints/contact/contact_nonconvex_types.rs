// Translated from BepuPhysics/Constraints/Contact/ContactNonconvexTypes.cs

use crate::physics::constraints::contact::contact_convex_common::*;
use crate::physics::constraints::contact::contact_nonconvex_common::*;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

// ============================================================================
// Contact2Nonconvex (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2NonconvexPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub offset_b: Vector3Wide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact2NonconvexPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        2
    }
    fn body_count() -> i32 {
        2
    }
}

impl INonconvexContactPrestep for Contact2NonconvexPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 2);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

impl ITwoBodyNonconvexContactPrestep for Contact2NonconvexPrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2NonconvexAccumulatedImpulses {
    pub contact0: NonconvexAccumulatedImpulses,
    pub contact1: NonconvexAccumulatedImpulses,
}

impl INonconvexContactAccumulatedImpulses for Contact2NonconvexAccumulatedImpulses {
    fn contact_count() -> i32 {
        2
    }
    fn get_impulses_for_contact(&self, index: usize) -> &NonconvexAccumulatedImpulses {
        debug_assert!(index < 2);
        unsafe { &*(&self.contact0 as *const NonconvexAccumulatedImpulses).add(index) }
    }
    fn get_impulses_for_contact_mut(&mut self, index: usize) -> &mut NonconvexAccumulatedImpulses {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexAccumulatedImpulses).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 2-contact nonconvex two body manifold constraints.
pub struct Contact2NonconvexTypeProcessor;
impl Contact2NonconvexTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 15;
}

// ============================================================================
// Contact2NonconvexOneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact2NonconvexOneBodyPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact2NonconvexOneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        2
    }
    fn body_count() -> i32 {
        1
    }
}

impl INonconvexContactPrestep for Contact2NonconvexOneBodyPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 2);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 2);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 2-contact nonconvex one body manifold constraints.
pub struct Contact2NonconvexOneBodyTypeProcessor;
impl Contact2NonconvexOneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 8;
}

// ============================================================================
// Contact3Nonconvex (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3NonconvexPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub offset_b: Vector3Wide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
    pub contact2: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact3NonconvexPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        3
    }
    fn body_count() -> i32 {
        2
    }
}

impl INonconvexContactPrestep for Contact3NonconvexPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 3);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

impl ITwoBodyNonconvexContactPrestep for Contact3NonconvexPrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3NonconvexAccumulatedImpulses {
    pub contact0: NonconvexAccumulatedImpulses,
    pub contact1: NonconvexAccumulatedImpulses,
    pub contact2: NonconvexAccumulatedImpulses,
}

impl INonconvexContactAccumulatedImpulses for Contact3NonconvexAccumulatedImpulses {
    fn contact_count() -> i32 {
        3
    }
    fn get_impulses_for_contact(&self, index: usize) -> &NonconvexAccumulatedImpulses {
        debug_assert!(index < 3);
        unsafe { &*(&self.contact0 as *const NonconvexAccumulatedImpulses).add(index) }
    }
    fn get_impulses_for_contact_mut(&mut self, index: usize) -> &mut NonconvexAccumulatedImpulses {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexAccumulatedImpulses).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 3-contact nonconvex two body manifold constraints.
pub struct Contact3NonconvexTypeProcessor;
impl Contact3NonconvexTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 16;
}

// ============================================================================
// Contact3NonconvexOneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact3NonconvexOneBodyPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
    pub contact2: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact3NonconvexOneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        3
    }
    fn body_count() -> i32 {
        1
    }
}

impl INonconvexContactPrestep for Contact3NonconvexOneBodyPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 3);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 3);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 3-contact nonconvex one body manifold constraints.
pub struct Contact3NonconvexOneBodyTypeProcessor;
impl Contact3NonconvexOneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 9;
}

// ============================================================================
// Contact4Nonconvex (TwoBody)
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4NonconvexPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub offset_b: Vector3Wide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
    pub contact2: NonconvexContactPrestepData,
    pub contact3: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact4NonconvexPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        4
    }
    fn body_count() -> i32 {
        2
    }
}

impl INonconvexContactPrestep for Contact4NonconvexPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 4);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

impl ITwoBodyNonconvexContactPrestep for Contact4NonconvexPrestepData {
    fn get_offset_b(&self) -> &Vector3Wide {
        &self.offset_b
    }
    fn get_offset_b_mut(&mut self) -> &mut Vector3Wide {
        &mut self.offset_b
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4NonconvexAccumulatedImpulses {
    pub contact0: NonconvexAccumulatedImpulses,
    pub contact1: NonconvexAccumulatedImpulses,
    pub contact2: NonconvexAccumulatedImpulses,
    pub contact3: NonconvexAccumulatedImpulses,
}

impl INonconvexContactAccumulatedImpulses for Contact4NonconvexAccumulatedImpulses {
    fn contact_count() -> i32 {
        4
    }
    fn get_impulses_for_contact(&self, index: usize) -> &NonconvexAccumulatedImpulses {
        debug_assert!(index < 4);
        unsafe { &*(&self.contact0 as *const NonconvexAccumulatedImpulses).add(index) }
    }
    fn get_impulses_for_contact_mut(&mut self, index: usize) -> &mut NonconvexAccumulatedImpulses {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexAccumulatedImpulses).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 4-contact nonconvex two body manifold constraints.
pub struct Contact4NonconvexTypeProcessor;
impl Contact4NonconvexTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 17;
}

// ============================================================================
// Contact4NonconvexOneBody
// ============================================================================

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact4NonconvexOneBodyPrestepData {
    pub material_properties: MaterialPropertiesWide,
    pub contact0: NonconvexContactPrestepData,
    pub contact1: NonconvexContactPrestepData,
    pub contact2: NonconvexContactPrestepData,
    pub contact3: NonconvexContactPrestepData,
}

impl IContactPrestep for Contact4NonconvexOneBodyPrestepData {
    fn get_material_properties(&self) -> &MaterialPropertiesWide {
        &self.material_properties
    }
    fn get_material_properties_mut(&mut self) -> &mut MaterialPropertiesWide {
        &mut self.material_properties
    }
    fn contact_count() -> i32 {
        4
    }
    fn body_count() -> i32 {
        1
    }
}

impl INonconvexContactPrestep for Contact4NonconvexOneBodyPrestepData {
    fn get_contact(&self, index: usize) -> &NonconvexContactPrestepData {
        debug_assert!(index < 4);
        unsafe { &*(&self.contact0 as *const NonconvexContactPrestepData).add(index) }
    }
    fn get_contact_mut(&mut self, index: usize) -> &mut NonconvexContactPrestepData {
        debug_assert!(index < 4);
        unsafe { &mut *(&mut self.contact0 as *mut NonconvexContactPrestepData).add(index) }
    }
}

/// Handles the solve iterations of a bunch of 4-contact nonconvex one body manifold constraints.
pub struct Contact4NonconvexOneBodyTypeProcessor;
impl Contact4NonconvexOneBodyTypeProcessor {
    pub const BATCH_TYPE_ID: i32 = 10;
}
