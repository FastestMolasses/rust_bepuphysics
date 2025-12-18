// Translated from BepuPhysics/CollisionDetection/ConvexContactManifoldWide.cs

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use super::contact_manifold::ConvexContactManifold;

/// Trait for SIMD-wide contact manifold types used in collision detection.
pub trait IContactManifoldWide {
    /// Applies a flip mask to swap the roles of A and B in the manifold.
    fn apply_flip_mask(&mut self, offset_b: &mut Vector3Wide, flip_mask: &Vector<i32>);

    /// Reads the first lane of the wide manifold into a scalar manifold.
    fn read_first(&self, offset_b: &Vector3Wide, target: &mut ConvexContactManifold);
}

/// Wide manifold for single-contact convex pairs.
pub struct Convex1ContactManifoldWide {
    pub offset_a: Vector3Wide,
    pub normal: Vector3Wide,
    pub depth: Vector<f32>,
    pub feature_id: Vector<i32>,
    pub contact_exists: Vector<i32>,
}

impl IContactManifoldWide for Convex1ContactManifoldWide {
    #[inline(always)]
    fn apply_flip_mask(&mut self, offset_b: &mut Vector3Wide, flip_mask: &Vector<i32>) {
        let mut flipped_normal = Vector3Wide::default();
        Vector3Wide::negate(&self.normal, &mut flipped_normal);
        let mut flipped_contact_position = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a, offset_b, &mut flipped_contact_position);
        let mut flipped_offset_b = Vector3Wide::default();
        Vector3Wide::negate(offset_b, &mut flipped_offset_b);
        self.normal = Vector3Wide::conditional_select(flip_mask, &flipped_normal, &self.normal);
        self.offset_a = Vector3Wide::conditional_select(flip_mask, &flipped_contact_position, &self.offset_a);
        *offset_b = Vector3Wide::conditional_select(flip_mask, &flipped_offset_b, offset_b);
    }

    #[inline(always)]
    fn read_first(&self, offset_b: &Vector3Wide, target: &mut ConvexContactManifold) {
        if self.contact_exists[0] < 0 {
            target.count = 1;
            target.offset_b.x = offset_b.x[0];
            target.offset_b.y = offset_b.y[0];
            target.offset_b.z = offset_b.z[0];
            target.contact0.offset.x = self.offset_a.x[0];
            target.contact0.offset.y = self.offset_a.y[0];
            target.contact0.offset.z = self.offset_a.z[0];
            target.contact0.depth = self.depth[0];
            target.contact0.feature_id = self.feature_id[0];
            target.normal.x = self.normal.x[0];
            target.normal.y = self.normal.y[0];
            target.normal.z = self.normal.z[0];
        } else {
            target.count = 0;
        }
    }
}

/// Wide manifold for two-contact convex pairs.
pub struct Convex2ContactManifoldWide {
    pub offset_a0: Vector3Wide,
    pub offset_a1: Vector3Wide,
    pub normal: Vector3Wide,
    pub depth0: Vector<f32>,
    pub depth1: Vector<f32>,
    pub feature_id0: Vector<i32>,
    pub feature_id1: Vector<i32>,
    pub contact0_exists: Vector<i32>,
    pub contact1_exists: Vector<i32>,
}

impl IContactManifoldWide for Convex2ContactManifoldWide {
    #[inline(always)]
    fn apply_flip_mask(&mut self, offset_b: &mut Vector3Wide, flip_mask: &Vector<i32>) {
        let mut flipped_normal = Vector3Wide::default();
        Vector3Wide::negate(&self.normal, &mut flipped_normal);
        let mut flipped_a0 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a0, offset_b, &mut flipped_a0);
        let mut flipped_a1 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a1, offset_b, &mut flipped_a1);
        let mut flipped_offset_b = Vector3Wide::default();
        Vector3Wide::negate(offset_b, &mut flipped_offset_b);
        self.normal = Vector3Wide::conditional_select(flip_mask, &flipped_normal, &self.normal);
        self.offset_a0 = Vector3Wide::conditional_select(flip_mask, &flipped_a0, &self.offset_a0);
        self.offset_a1 = Vector3Wide::conditional_select(flip_mask, &flipped_a1, &self.offset_a1);
        *offset_b = Vector3Wide::conditional_select(flip_mask, &flipped_offset_b, offset_b);
    }

    #[inline(always)]
    fn read_first(&self, offset_b: &Vector3Wide, target: &mut ConvexContactManifold) {
        target.count = 0;
        if self.contact0_exists[0] < 0 {
            target.count += 1;
            target.contact0.offset.x = self.offset_a0.x[0];
            target.contact0.offset.y = self.offset_a0.y[0];
            target.contact0.offset.z = self.offset_a0.z[0];
            target.contact0.depth = self.depth0[0];
            target.contact0.feature_id = self.feature_id0[0];
        }
        if self.contact1_exists[0] < 0 {
            unsafe {
                let contact = &mut *(&mut target.contact0 as *mut _ as *mut crate::physics::collision_detection::contact_manifold::ConvexContact)
                    .add(target.count as usize);
                target.count += 1;
                contact.offset.x = self.offset_a1.x[0];
                contact.offset.y = self.offset_a1.y[0];
                contact.offset.z = self.offset_a1.z[0];
                contact.depth = self.depth1[0];
                contact.feature_id = self.feature_id1[0];
            }
        }
        if target.count > 0 {
            target.offset_b.x = offset_b.x[0];
            target.offset_b.y = offset_b.y[0];
            target.offset_b.z = offset_b.z[0];
            target.normal.x = self.normal.x[0];
            target.normal.y = self.normal.y[0];
            target.normal.z = self.normal.z[0];
        }
    }
}

/// Wide manifold for four-contact convex pairs.
pub struct Convex4ContactManifoldWide {
    pub offset_a0: Vector3Wide,
    pub offset_a1: Vector3Wide,
    pub offset_a2: Vector3Wide,
    pub offset_a3: Vector3Wide,
    pub normal: Vector3Wide,
    pub depth0: Vector<f32>,
    pub depth1: Vector<f32>,
    pub depth2: Vector<f32>,
    pub depth3: Vector<f32>,
    pub feature_id0: Vector<i32>,
    pub feature_id1: Vector<i32>,
    pub feature_id2: Vector<i32>,
    pub feature_id3: Vector<i32>,
    pub contact0_exists: Vector<i32>,
    pub contact1_exists: Vector<i32>,
    pub contact2_exists: Vector<i32>,
    pub contact3_exists: Vector<i32>,
}

impl IContactManifoldWide for Convex4ContactManifoldWide {
    #[inline(always)]
    fn apply_flip_mask(&mut self, offset_b: &mut Vector3Wide, flip_mask: &Vector<i32>) {
        let mut flipped_normal = Vector3Wide::default();
        Vector3Wide::negate(&self.normal, &mut flipped_normal);
        let mut flipped_a0 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a0, offset_b, &mut flipped_a0);
        let mut flipped_a1 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a1, offset_b, &mut flipped_a1);
        let mut flipped_a2 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a2, offset_b, &mut flipped_a2);
        let mut flipped_a3 = Vector3Wide::default();
        Vector3Wide::subtract(&self.offset_a3, offset_b, &mut flipped_a3);
        let mut flipped_offset_b = Vector3Wide::default();
        Vector3Wide::negate(offset_b, &mut flipped_offset_b);
        self.normal = Vector3Wide::conditional_select(flip_mask, &flipped_normal, &self.normal);
        self.offset_a0 = Vector3Wide::conditional_select(flip_mask, &flipped_a0, &self.offset_a0);
        self.offset_a1 = Vector3Wide::conditional_select(flip_mask, &flipped_a1, &self.offset_a1);
        self.offset_a2 = Vector3Wide::conditional_select(flip_mask, &flipped_a2, &self.offset_a2);
        self.offset_a3 = Vector3Wide::conditional_select(flip_mask, &flipped_a3, &self.offset_a3);
        *offset_b = Vector3Wide::conditional_select(flip_mask, &flipped_offset_b, offset_b);
    }

    #[inline(always)]
    fn read_first(&self, offset_b: &Vector3Wide, target: &mut ConvexContactManifold) {
        target.count = 0;
        if self.contact0_exists[0] < 0 {
            target.count += 1;
            target.contact0.offset.x = self.offset_a0.x[0];
            target.contact0.offset.y = self.offset_a0.y[0];
            target.contact0.offset.z = self.offset_a0.z[0];
            target.contact0.depth = self.depth0[0];
            target.contact0.feature_id = self.feature_id0[0];
        }
        unsafe {
            if self.contact1_exists[0] < 0 {
                let contact = &mut *(&mut target.contact0 as *mut _ as *mut crate::physics::collision_detection::contact_manifold::ConvexContact)
                    .add(target.count as usize);
                target.count += 1;
                contact.offset.x = self.offset_a1.x[0];
                contact.offset.y = self.offset_a1.y[0];
                contact.offset.z = self.offset_a1.z[0];
                contact.depth = self.depth1[0];
                contact.feature_id = self.feature_id1[0];
            }
            if self.contact2_exists[0] < 0 {
                let contact = &mut *(&mut target.contact0 as *mut _ as *mut crate::physics::collision_detection::contact_manifold::ConvexContact)
                    .add(target.count as usize);
                target.count += 1;
                contact.offset.x = self.offset_a2.x[0];
                contact.offset.y = self.offset_a2.y[0];
                contact.offset.z = self.offset_a2.z[0];
                contact.depth = self.depth2[0];
                contact.feature_id = self.feature_id2[0];
            }
            if self.contact3_exists[0] < 0 {
                let contact = &mut *(&mut target.contact0 as *mut _ as *mut crate::physics::collision_detection::contact_manifold::ConvexContact)
                    .add(target.count as usize);
                target.count += 1;
                contact.offset.x = self.offset_a3.x[0];
                contact.offset.y = self.offset_a3.y[0];
                contact.offset.z = self.offset_a3.z[0];
                contact.depth = self.depth3[0];
                contact.feature_id = self.feature_id3[0];
            }
        }
        if target.count > 0 {
            target.offset_b.x = offset_b.x[0];
            target.offset_b.y = offset_b.y[0];
            target.offset_b.z = offset_b.z[0];
            target.normal.x = self.normal.x[0];
            target.normal.y = self.normal.y[0];
            target.normal.z = self.normal.z[0];
        }
    }
}
