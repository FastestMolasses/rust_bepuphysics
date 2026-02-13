// Translated from BepuPhysics/StaticReference.cs

use crate::physics::body_properties::RigidPose;
use crate::physics::collidables::collidable::ContinuousDetection;
use crate::physics::collidables::collidable_reference::CollidableReference;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::physics::handles::StaticHandle;
use crate::physics::static_description::StaticDescription;
use crate::physics::statics::Statics;

/// Convenience structure for directly referring to a static's properties.
///
/// Note that this type makes no attempt to protect against unsafe modification of static properties.
pub struct StaticReference {
    /// Handle of the static that this reference refers to.
    pub handle: StaticHandle,
    /// The collection containing the static.
    pub statics: *mut Statics,
}

impl StaticReference {
    /// Constructs a new static reference.
    #[inline(always)]
    pub fn new(handle: StaticHandle, statics: *mut Statics) -> Self {
        Self { handle, statics }
    }

    /// Gets the statics collection as a shared reference.
    #[inline(always)]
    fn statics(&self) -> &Statics {
        unsafe { &*self.statics }
    }

    /// Gets the statics collection as a mutable reference.
    #[inline(always)]
    fn statics_mut(&self) -> &mut Statics {
        unsafe { &mut *self.statics }
    }

    /// Gets whether the static reference exists within the static set.
    pub fn exists(&self) -> bool {
        if self.statics.is_null() {
            return false;
        }
        self.statics().static_exists(self.handle)
    }

    /// Gets the static's index in the statics collection.
    #[inline(always)]
    pub fn index(&self) -> i32 {
        *self.statics().handle_to_index.get(self.handle.0)
    }

    /// Gets a reference to the static's data.
    #[inline(always)]
    pub fn get_static(&self) -> &crate::physics::statics::Static {
        self.statics().get_direct_reference(self.handle)
    }

    /// Gets a mutable reference to the static's data.
    #[inline(always)]
    pub fn get_static_mut(&self) -> &mut crate::physics::statics::Static {
        self.statics_mut().get_direct_reference_mut(self.handle)
    }

    /// Gets a reference to the static's pose.
    #[inline(always)]
    pub fn pose(&self) -> &RigidPose {
        &self.get_static().pose
    }

    /// Gets a mutable reference to the static's pose.
    #[inline(always)]
    pub fn pose_mut(&self) -> &mut RigidPose {
        &mut self.get_static_mut().pose
    }

    /// Gets a reference to the static's continuity settings.
    #[inline(always)]
    pub fn continuity(&self) -> &ContinuousDetection {
        &self.get_static().continuity
    }

    /// Gets a mutable reference to the static's continuity settings.
    #[inline(always)]
    pub fn continuity_mut(&self) -> &mut ContinuousDetection {
        &mut self.get_static_mut().continuity
    }

    /// Gets the shape used by the static.
    #[inline(always)]
    pub fn shape(&self) -> TypedIndex {
        self.get_static().shape
    }

    /// Gets a `CollidableReference` for this static.
    #[inline(always)]
    pub fn collidable_reference(&self) -> CollidableReference {
        CollidableReference::from_static(self.handle)
    }

    /// Gets a description of the static.
    pub fn get_description(&self, description: &mut StaticDescription) {
        self.statics().get_description(self.handle, description);
    }

    /// Gets a description of the static (returned by value).
    pub fn get_description_value(&self) -> StaticDescription {
        self.statics().get_description_value(self.handle)
    }

    /// Sets a static's properties according to a description.
    pub fn apply_description(&self, description: &StaticDescription) {
        self.statics_mut()
            .apply_description(self.handle, description);
    }

    /// Changes the shape of a static.
    pub fn set_shape(&self, new_shape: TypedIndex) {
        self.statics_mut().set_shape(self.handle, new_shape);
    }

    // NOTE: BoundingBox and GetBoundsReferencesFromBroadPhase omitted — require BroadPhase implementation.

    /// Updates the static's bounds in the broad phase for its current state.
    pub fn update_bounds(&self) {
        self.statics_mut().update_bounds(self.handle);
    }
}

/// Implicit conversion: `StaticReference` → `StaticHandle`.
impl From<StaticReference> for StaticHandle {
    fn from(reference: StaticReference) -> Self {
        reference.handle
    }
}
