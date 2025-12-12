use crate::physics::handles::{BodyHandle, StaticHandle};
use std::fmt;
use std::hash::{Hash, Hasher};

/// Represents how a collidable can interact and move.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollidableMobility {
    /// Marks a collidable as owned by a dynamic body.
    Dynamic = 0,
    /// Marks a collidable as owned by a kinematic body.
    Kinematic = 1,
    /// Marks the collidable as an independent immobile collidable.
    Static = 2,
}

/// Uses a bitpacked representation to refer to a body or static collidable.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CollidableReference {
    /// Bitpacked representation of the collidable reference.
    pub packed: u32,
}

// Ensure size matches C# [StructLayout(LayoutKind.Sequential, Size = 4)]
const _: () = {
    assert!(std::mem::size_of::<CollidableReference>() == 4);
};

impl CollidableReference {
    /// Gets the mobility state of the owner of this collidable.
    #[inline(always)]
    pub fn mobility(&self) -> CollidableMobility {
        match self.packed >> 30 {
            0 => CollidableMobility::Dynamic,
            1 => CollidableMobility::Kinematic,
            2 => CollidableMobility::Static,
            _ => unreachable!(),
        }
    }

    /// Gets the body handle of the owner of the collidable referred to by this instance.
    #[inline(always)]
    pub fn body_handle(&self) -> BodyHandle {
        debug_assert!(
            self.mobility() == CollidableMobility::Dynamic
                || self.mobility() == CollidableMobility::Kinematic,
            "Extracting a body handle from a collidable reference requires that the collidable is owned by a body."
        );
        BodyHandle(self.raw_handle_value())
    }

    /// Gets the static handle of the owner of the collidable referred to by this instance.
    #[inline(always)]
    pub fn static_handle(&self) -> StaticHandle {
        debug_assert!(
            self.mobility() == CollidableMobility::Static,
            "Extracting a static handle from a collidable reference requires that the collidable is owned by a static."
        );
        StaticHandle(self.raw_handle_value())
    }

    /// Gets the integer value of the handle of the owner of the collidable.
    #[inline(always)]
    pub fn raw_handle_value(&self) -> i32 {
        (self.packed & 0x3FFFFFFF) as i32
    }

    /// Creates a collidable reference from mobility and raw handle value.
    #[inline(always)]
    pub(crate) fn from_raw(mobility: CollidableMobility, handle: i32) -> Self {
        debug_assert!(
            (mobility as i32) >= 0 && (mobility as i32) <= 2,
            "Invalid mobility type."
        );
        debug_assert!(
            handle >= 0 && handle < (1 << 30),
            "Do you actually have more than 2^30 collidables?"
        );
        Self {
            packed: ((mobility as u32) << 30) | (handle as u32),
        }
    }

    /// Creates a collidable reference for a body.
    #[inline(always)]
    pub fn from_body(mobility: CollidableMobility, handle: BodyHandle) -> Self {
        debug_assert!(
            mobility == CollidableMobility::Dynamic || mobility == CollidableMobility::Kinematic,
            "Creating a collidable reference associated with a body requires a body-related mobility."
        );
        Self::from_raw(mobility, handle.0)
    }

    /// Creates a collidable reference for a static.
    #[inline(always)]
    pub fn from_static(handle: StaticHandle) -> Self {
        Self::from_raw(CollidableMobility::Static, handle.0)
    }
}

impl Default for CollidableReference {
    fn default() -> Self {
        Self { packed: 0 }
    }
}

impl PartialEq for CollidableReference {
    fn eq(&self, other: &Self) -> bool {
        self.packed == other.packed
    }
}

impl Eq for CollidableReference {}

impl Hash for CollidableReference {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.packed.hash(state);
    }
}

impl fmt::Display for CollidableReference {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let handle = if self.mobility() == CollidableMobility::Static {
            self.static_handle().0
        } else {
            self.body_handle().0
        };
        write!(f, "{:?}[{}]", self.mobility(), handle)
    }
}

/// Comparer for CollidableReference that operates by reference.
pub struct CollidableReferenceComparer;

impl CollidableReferenceComparer {
    #[inline(always)]
    pub fn equals(a: &CollidableReference, b: &CollidableReference) -> bool {
        a.packed == b.packed
    }

    #[inline(always)]
    pub fn hash(item: &CollidableReference) -> i32 {
        item.packed as i32
    }
}
