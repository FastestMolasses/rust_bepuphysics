// Translated from BepuPhysics/CollisionDetection/ContactManifold.cs

use glam::Vec3;
use std::fmt;

/// Information about a single contact.
/// This type contains a field for the normal; it can be used to represent contacts within nonconvex contact manifolds or convex manifolds.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Contact {
    /// Offset from the position of collidable A to the contact position.
    pub offset: Vec3,
    /// Penetration depth between the two collidables at this contact. Negative values represent separation.
    pub depth: f32,
    /// Surface basis of the contact. Points from collidable B to collidable A.
    pub normal: Vec3,
    /// Id of the features involved in the collision that generated this contact.
    pub feature_id: i32,
}

const _: () = {
    assert!(std::mem::size_of::<Contact>() == 32);
    assert!(std::mem::offset_of!(Contact, offset) == 0);
    assert!(std::mem::offset_of!(Contact, depth) == 12);
    assert!(std::mem::offset_of!(Contact, normal) == 16);
    assert!(std::mem::offset_of!(Contact, feature_id) == 28);
};

impl Default for Contact {
    fn default() -> Self {
        Self {
            offset: Vec3::ZERO,
            depth: 0.0,
            normal: Vec3::ZERO,
            feature_id: 0,
        }
    }
}

/// Information about a single contact in a convex collidable pair.
/// Convex collidable pairs share one surface basis across the manifold.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ConvexContact {
    /// Offset from the position of collidable A to the contact position.
    pub offset: Vec3,
    /// Penetration depth between the two collidables at this contact. Negative values represent separation.
    pub depth: f32,
    /// Id of the features involved in the collision that generated this contact.
    pub feature_id: i32,
}

const _: () = {
    assert!(std::mem::size_of::<ConvexContact>() == 20);
    assert!(std::mem::offset_of!(ConvexContact, offset) == 0);
    assert!(std::mem::offset_of!(ConvexContact, depth) == 12);
    assert!(std::mem::offset_of!(ConvexContact, feature_id) == 16);
};

impl Default for ConvexContact {
    fn default() -> Self {
        Self {
            offset: Vec3::ZERO,
            depth: 0.0,
            feature_id: 0,
        }
    }
}

/// Trait for contact manifolds.
pub trait IContactManifold {
    /// Gets the number of contacts in the manifold.
    fn count(&self) -> i32;

    /// Gets whether the contact manifold was created by a pair of convex objects.
    fn convex(&self) -> bool;

    /// Gets the feature id associated with a requested contact.
    fn get_feature_id(&self, contact_index: i32) -> i32;

    /// Gets the depth associated with a requested contact.
    fn get_depth(&self, contact_index: i32) -> f32;

    /// Gets a contact's normal. Points from collidable B to collidable A.
    fn get_normal(&self, contact_index: i32) -> Vec3;

    /// Gets the offset from collidable A to the requested contact.
    fn get_offset(&self, contact_index: i32) -> Vec3;

    /// Gets a copy of a contact's data.
    fn get_contact(&self, contact_index: i32) -> Contact;

    /// Gets a copy of a contact's data decomposed into individual fields.
    fn get_contact_data(
        &self,
        contact_index: i32,
        offset: &mut Vec3,
        normal: &mut Vec3,
        depth: &mut f32,
        feature_id: &mut i32,
    );
}

/// Contains the data associated with a nonconvex contact manifold.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct NonconvexContactManifold {
    /// Offset from collidable A to collidable B.
    pub offset_b: Vec3,
    pub count: i32,
    pub contact0: Contact,
    pub contact1: Contact,
    pub contact2: Contact,
    pub contact3: Contact,
}

const _: () = {
    assert!(std::mem::size_of::<NonconvexContactManifold>() == 144);
    assert!(std::mem::offset_of!(NonconvexContactManifold, offset_b) == 0);
    assert!(std::mem::offset_of!(NonconvexContactManifold, count) == 12);
    assert!(std::mem::offset_of!(NonconvexContactManifold, contact0) == 16);
    assert!(std::mem::offset_of!(NonconvexContactManifold, contact1) == 48);
    assert!(std::mem::offset_of!(NonconvexContactManifold, contact2) == 80);
    assert!(std::mem::offset_of!(NonconvexContactManifold, contact3) == 112);
};

impl NonconvexContactManifold {
    /// The maximum number of contacts that can exist within a nonconvex manifold.
    pub const MAXIMUM_CONTACT_COUNT: i32 = 4;

    #[cfg(debug_assertions)]
    fn validate_index(&self, contact_index: i32) {
        debug_assert!(
            contact_index >= 0 && contact_index < self.count,
            "Contact index must be within the contact count."
        );
    }

    #[cfg(not(debug_assertions))]
    fn validate_index(&self, _contact_index: i32) {}

    /// Gets a reference to the contact at the given index.
    #[inline(always)]
    pub unsafe fn get_contact_ref(&self, contact_index: i32) -> &Contact {
        self.validate_index(contact_index);
        &*(&self.contact0 as *const Contact).add(contact_index as usize)
    }

    /// Gets a mutable reference to the contact at the given index.
    #[inline(always)]
    pub unsafe fn get_contact_mut(&mut self, contact_index: i32) -> &mut Contact {
        self.validate_index(contact_index);
        &mut *(&mut self.contact0 as *mut Contact).add(contact_index as usize)
    }

    /// Gets a reference to the depth of a contact.
    #[inline(always)]
    pub unsafe fn get_depth_ref(manifold: &mut Self, contact_index: i32) -> &mut f32 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut Contact).add(contact_index as usize)).depth
    }

    /// Gets a reference to the normal of a contact.
    #[inline(always)]
    pub unsafe fn get_normal_ref(manifold: &mut Self, contact_index: i32) -> &mut Vec3 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut Contact).add(contact_index as usize)).normal
    }

    /// Gets a reference to the offset of a contact.
    #[inline(always)]
    pub unsafe fn get_offset_ref(manifold: &mut Self, contact_index: i32) -> &mut Vec3 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut Contact).add(contact_index as usize)).offset
    }

    /// Gets a reference to the feature id of a contact.
    #[inline(always)]
    pub unsafe fn get_feature_id_ref(manifold: &mut Self, contact_index: i32) -> &mut i32 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut Contact).add(contact_index as usize)).feature_id
    }

    /// Quickly removes a contact at the given index by swapping with the last.
    pub unsafe fn fast_remove_at(manifold: *mut Self, index: i32) {
        (*manifold).count -= 1;
        if index < (*manifold).count {
            let contacts = &mut (*manifold).contact0 as *mut Contact;
            *contacts.add(index as usize) = *contacts.add((*manifold).count as usize);
        }
    }

    /// Adds a contact to the manifold from a convex contact and a normal.
    pub unsafe fn add(manifold: *mut Self, normal: &Vec3, convex_contact: &ConvexContact) {
        debug_assert!((*manifold).count < Self::MAXIMUM_CONTACT_COUNT);
        let contacts = &mut (*manifold).contact0 as *mut Contact;
        let target = &mut *contacts.add((*manifold).count as usize);
        (*manifold).count += 1;
        target.depth = convex_contact.depth;
        target.offset = convex_contact.offset;
        target.normal = *normal;
        target.feature_id = convex_contact.feature_id;
    }

    /// Allocates a new contact slot in the manifold, returning a mutable reference.
    pub unsafe fn allocate(manifold: *mut Self) -> &'static mut Contact {
        debug_assert!((*manifold).count < Self::MAXIMUM_CONTACT_COUNT);
        let contacts = &mut (*manifold).contact0 as *mut Contact;
        let result = &mut *contacts.add((*manifold).count as usize);
        (*manifold).count += 1;
        result
    }
}

impl Default for NonconvexContactManifold {
    fn default() -> Self {
        Self {
            offset_b: Vec3::ZERO,
            count: 0,
            contact0: Contact::default(),
            contact1: Contact::default(),
            contact2: Contact::default(),
            contact3: Contact::default(),
        }
    }
}

impl IContactManifold for NonconvexContactManifold {
    #[inline(always)]
    fn count(&self) -> i32 {
        self.count
    }

    #[inline(always)]
    fn convex(&self) -> bool {
        false
    }

    #[inline(always)]
    fn get_feature_id(&self, contact_index: i32) -> i32 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const Contact).add(contact_index as usize)).feature_id }
    }

    #[inline(always)]
    fn get_depth(&self, contact_index: i32) -> f32 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const Contact).add(contact_index as usize)).depth }
    }

    #[inline(always)]
    fn get_normal(&self, contact_index: i32) -> Vec3 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const Contact).add(contact_index as usize)).normal }
    }

    #[inline(always)]
    fn get_offset(&self, contact_index: i32) -> Vec3 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const Contact).add(contact_index as usize)).offset }
    }

    fn get_contact(&self, contact_index: i32) -> Contact {
        self.validate_index(contact_index);
        unsafe { *(&self.contact0 as *const Contact).add(contact_index as usize) }
    }

    fn get_contact_data(
        &self,
        contact_index: i32,
        offset: &mut Vec3,
        normal: &mut Vec3,
        depth: &mut f32,
        feature_id: &mut i32,
    ) {
        self.validate_index(contact_index);
        unsafe {
            let contact = &*(&self.contact0 as *const Contact).add(contact_index as usize);
            *offset = contact.offset;
            *normal = contact.normal;
            *depth = contact.depth;
            *feature_id = contact.feature_id;
        }
    }
}

/// Contains the data associated with a convex contact manifold.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ConvexContactManifold {
    /// Offset from collidable A to collidable B.
    pub offset_b: Vec3,
    pub count: i32,
    /// Surface normal shared by all contacts. Points from collidable B to collidable A.
    pub normal: Vec3,
    pub contact0: ConvexContact,
    pub contact1: ConvexContact,
    pub contact2: ConvexContact,
    pub contact3: ConvexContact,
}

const _: () = {
    assert!(std::mem::size_of::<ConvexContactManifold>() == 108);
    assert!(std::mem::offset_of!(ConvexContactManifold, offset_b) == 0);
    assert!(std::mem::offset_of!(ConvexContactManifold, count) == 12);
    assert!(std::mem::offset_of!(ConvexContactManifold, normal) == 16);
    assert!(std::mem::offset_of!(ConvexContactManifold, contact0) == 28);
    assert!(std::mem::offset_of!(ConvexContactManifold, contact1) == 48);
    assert!(std::mem::offset_of!(ConvexContactManifold, contact2) == 68);
    assert!(std::mem::offset_of!(ConvexContactManifold, contact3) == 88);
};

impl ConvexContactManifold {
    /// The maximum number of contacts that can exist within a convex manifold.
    pub const MAXIMUM_CONTACT_COUNT: i32 = 4;

    #[cfg(debug_assertions)]
    fn validate_index(&self, contact_index: i32) {
        debug_assert!(
            contact_index >= 0 && contact_index < self.count,
            "Contact index must be within the contact count."
        );
    }

    #[cfg(not(debug_assertions))]
    fn validate_index(&self, _contact_index: i32) {}

    /// Gets a reference to the contact at the given index.
    #[inline(always)]
    pub unsafe fn get_contact_ref(&self, contact_index: i32) -> &ConvexContact {
        self.validate_index(contact_index);
        &*(&self.contact0 as *const ConvexContact).add(contact_index as usize)
    }

    /// Gets a mutable reference to the contact at the given index.
    #[inline(always)]
    pub unsafe fn get_contact_mut(&mut self, contact_index: i32) -> &mut ConvexContact {
        self.validate_index(contact_index);
        &mut *(&mut self.contact0 as *mut ConvexContact).add(contact_index as usize)
    }

    /// Gets a reference to the depth of a contact.
    #[inline(always)]
    pub unsafe fn get_depth_ref(manifold: &mut Self, contact_index: i32) -> &mut f32 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut ConvexContact).add(contact_index as usize)).depth
    }

    /// Gets a reference to the normal (shared across all contacts in a convex manifold).
    #[inline(always)]
    pub fn get_normal_ref(manifold: &mut Self, _contact_index: i32) -> &mut Vec3 {
        &mut manifold.normal
    }

    /// Gets a reference to the offset of a contact.
    #[inline(always)]
    pub unsafe fn get_offset_ref(manifold: &mut Self, contact_index: i32) -> &mut Vec3 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut ConvexContact).add(contact_index as usize)).offset
    }

    /// Gets a reference to the feature id of a contact.
    #[inline(always)]
    pub unsafe fn get_feature_id_ref(manifold: &mut Self, contact_index: i32) -> &mut i32 {
        manifold.validate_index(contact_index);
        &mut (*(&mut manifold.contact0 as *mut ConvexContact).add(contact_index as usize)).feature_id
    }

    /// Quickly removes a contact at the given index by swapping with the last.
    pub unsafe fn fast_remove_at(manifold: &mut Self, index: i32) {
        manifold.count -= 1;
        if index < manifold.count {
            let contacts = &mut manifold.contact0 as *mut ConvexContact;
            *contacts.add(index as usize) = *contacts.add(manifold.count as usize);
        }
    }
}

impl Default for ConvexContactManifold {
    fn default() -> Self {
        Self {
            offset_b: Vec3::ZERO,
            count: 0,
            normal: Vec3::ZERO,
            contact0: ConvexContact::default(),
            contact1: ConvexContact::default(),
            contact2: ConvexContact::default(),
            contact3: ConvexContact::default(),
        }
    }
}

impl IContactManifold for ConvexContactManifold {
    #[inline(always)]
    fn count(&self) -> i32 {
        self.count
    }

    #[inline(always)]
    fn convex(&self) -> bool {
        true
    }

    #[inline(always)]
    fn get_feature_id(&self, contact_index: i32) -> i32 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const ConvexContact).add(contact_index as usize)).feature_id }
    }

    #[inline(always)]
    fn get_depth(&self, contact_index: i32) -> f32 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const ConvexContact).add(contact_index as usize)).depth }
    }

    #[inline(always)]
    fn get_normal(&self, _contact_index: i32) -> Vec3 {
        self.normal
    }

    #[inline(always)]
    fn get_offset(&self, contact_index: i32) -> Vec3 {
        self.validate_index(contact_index);
        unsafe { (*(&self.contact0 as *const ConvexContact).add(contact_index as usize)).offset }
    }

    fn get_contact(&self, contact_index: i32) -> Contact {
        self.validate_index(contact_index);
        unsafe {
            let c = &*(&self.contact0 as *const ConvexContact).add(contact_index as usize);
            Contact {
                offset: c.offset,
                depth: c.depth,
                normal: self.normal,
                feature_id: c.feature_id,
            }
        }
    }

    fn get_contact_data(
        &self,
        contact_index: i32,
        offset: &mut Vec3,
        normal: &mut Vec3,
        depth: &mut f32,
        feature_id: &mut i32,
    ) {
        self.validate_index(contact_index);
        unsafe {
            let contact = &*(&self.contact0 as *const ConvexContact).add(contact_index as usize);
            *offset = contact.offset;
            *normal = self.normal;
            *depth = contact.depth;
            *feature_id = contact.feature_id;
        }
    }
}
