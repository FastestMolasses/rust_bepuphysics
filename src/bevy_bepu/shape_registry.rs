//! Reference-counted shape sharing for the Bevy integration.
//!
//! Bepu's [`Shapes`] collection is an arena: every `add` allocates a slot that lives until it is
//! explicitly removed. Nothing about a body's removal frees the shape it referenced, so a naive
//! "add a shape per spawned collider" integration leaks a `Shapes` entry for every entity that has
//! ever existed. A destruction-heavy game spawning thousands of fragments a second runs out of
//! memory in minutes.
//!
//! This registry fixes that in two steps:
//!
//! 1. Every collider acquired through [`ShapeRegistry::acquire`] is keyed by its *rounded* shape
//!    parameters, so a thousand identical 1×1×1 crates share a single `Shapes` entry.
//! 2. Each entry carries a reference count. [`ShapeRegistry::release`] decrements it and calls
//!    [`Shapes::remove_and_dispose`] on the last release, returning the slot (and any buffers the
//!    shape batch allocated for it) to the pool.
//!
//! # Rounding
//!
//! Keys quantize each dimension to [`ShapeRegistry::QUANTUM`] (0.1 mm) before hashing, so colliders
//! whose parameters differ only by float noise — the usual result of computing extents from mesh
//! bounds — still share. Colliders that differ by more than the quantum get their own entry, and a
//! shared entry always uses the parameters of the *first* acquirer, so two colliders that round to
//! the same key are simulated with identical geometry. Set the quantum smaller than any size
//! difference your game cares about; at 0.1 mm this is far below the scale at which Bepu's solver
//! behaves differently.
//!
//! # Compound shapes
//!
//! [`Shapes::remove_and_dispose`] does **not** touch children. Every variant of
//! [`BepuCollider`](super::components::BepuCollider) is currently a convex leaf shape with no
//! children, so `remove_and_dispose` is complete for all of them. When compound support lands
//! (`Compound`, `BigCompound`, and `Mesh` — a mesh owns a child triangle buffer) those variants must
//! release through [`Shapes::recursively_remove_and_dispose`] instead, or they will leak their
//! children exactly the way the whole collection used to leak. [`ShapeRegistry::release`] has a
//! single call site for this reason: add the branch there.

use std::collections::HashMap;

use crate::physics::collidables::box_shape::Box as PhysicsBox;
use crate::physics::collidables::capsule::Capsule;
use crate::physics::collidables::cylinder::Cylinder;
use crate::physics::collidables::shapes::Shapes;
use crate::physics::collidables::sphere::Sphere;
use crate::physics::collidables::typed_index::TypedIndex;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::components::BepuCollider;

/// Identity of a collider for deduplication purposes: the variant plus its quantized parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct ShapeKey {
    variant: u8,
    a: i64,
    b: i64,
    c: i64,
}

/// A shared `Shapes` entry and the number of live users of it.
#[derive(Debug, Clone, Copy)]
struct ShapeEntry {
    index: TypedIndex,
    ref_count: u32,
}

/// Maps collider descriptions to shared, reference-counted [`Shapes`] entries.
///
/// Owned by [`BepuSimulation`](super::resources::BepuSimulation); systems reach it through
/// [`BepuSimulation::acquire_shape`](super::resources::BepuSimulation::acquire_shape) and
/// [`BepuSimulation::release_shape`](super::resources::BepuSimulation::release_shape).
#[derive(Debug, Default)]
pub struct ShapeRegistry {
    by_key: HashMap<ShapeKey, ShapeEntry>,
    /// Reverse lookup so a release only needs the [`TypedIndex`] that was handed out.
    by_index: HashMap<TypedIndex, ShapeKey>,
}

impl ShapeRegistry {
    /// Size of the grid collider dimensions are snapped to before being used as a dedup key,
    /// in metres. 0.1 mm.
    pub const QUANTUM: f32 = 1.0e-4;

    #[inline]
    fn quantize(value: f32) -> i64 {
        // `round` rather than `trunc` so that values straddling a grid line from either side land on
        // the same bucket, and so that negative (invalid, but not our problem here) values behave
        // symmetrically. Non-finite inputs collapse to a single bucket, which is fine: a NaN-sized
        // collider is a user error either way and sharing one broken shape beats leaking many.
        if value.is_finite() {
            (value as f64 / Self::QUANTUM as f64).round() as i64
        } else {
            i64::MIN
        }
    }

    fn key_for(collider: &BepuCollider) -> ShapeKey {
        match collider {
            BepuCollider::Sphere { radius } => ShapeKey {
                variant: 0,
                a: Self::quantize(*radius),
                b: 0,
                c: 0,
            },
            BepuCollider::Box {
                width,
                height,
                depth,
            } => ShapeKey {
                variant: 1,
                a: Self::quantize(*width),
                b: Self::quantize(*height),
                c: Self::quantize(*depth),
            },
            BepuCollider::Capsule { radius, length } => ShapeKey {
                variant: 2,
                a: Self::quantize(*radius),
                b: Self::quantize(*length),
                c: 0,
            },
            BepuCollider::Cylinder { radius, length } => ShapeKey {
                variant: 3,
                a: Self::quantize(*radius),
                b: Self::quantize(*length),
                c: 0,
            },
        }
    }

    /// Returns a shape index for `collider`, adding it to `shapes` only if no equivalent shape is
    /// already registered. Increments the entry's reference count either way; every call must be
    /// balanced by exactly one [`ShapeRegistry::release`].
    pub fn acquire(&mut self, shapes: &mut Shapes, collider: &BepuCollider) -> TypedIndex {
        let key = Self::key_for(collider);
        if let Some(entry) = self.by_key.get_mut(&key) {
            entry.ref_count += 1;
            return entry.index;
        }

        let index = match collider {
            BepuCollider::Sphere { radius } => shapes.add(&Sphere::new(*radius)),
            BepuCollider::Box {
                width,
                height,
                depth,
            } => shapes.add(&PhysicsBox::new(*width, *height, *depth)),
            BepuCollider::Capsule { radius, length } => shapes.add(&Capsule::new(*radius, *length)),
            BepuCollider::Cylinder { radius, length } => {
                shapes.add(&Cylinder::new(*radius, *length))
            }
        };

        self.by_key.insert(
            key,
            ShapeEntry {
                index,
                ref_count: 1,
            },
        );
        self.by_index.insert(index, key);
        index
    }

    /// Drops one reference to `index`. When the last reference goes away the shape is removed from
    /// `shapes` and its resources are returned to `pool`.
    ///
    /// Indices this registry did not hand out are ignored, so releasing twice (or releasing a shape
    /// a user added directly) is a no-op rather than a double free.
    pub fn release(&mut self, shapes: &mut Shapes, pool: &mut BufferPool, index: TypedIndex) {
        let Some(key) = self.by_index.get(&index).copied() else {
            return;
        };
        let Some(entry) = self.by_key.get_mut(&key) else {
            debug_assert!(false, "by_index and by_key must stay in sync");
            self.by_index.remove(&index);
            return;
        };
        entry.ref_count -= 1;
        if entry.ref_count > 0 {
            return;
        }
        self.by_key.remove(&key);
        self.by_index.remove(&index);
        // Every `BepuCollider` variant is a childless convex shape; see the module docs for what has
        // to change here when compound and mesh colliders land.
        shapes.remove_and_dispose(&index, pool);
    }

    /// Number of distinct shapes currently registered. Intended for tests and diagnostics.
    #[inline]
    pub fn len(&self) -> usize {
        self.by_key.len()
    }

    /// Whether no shapes are registered.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.by_key.is_empty()
    }

    /// Number of live users of the shape backing `collider`, or 0 if it is not registered.
    /// Intended for tests and diagnostics.
    #[inline]
    pub fn reference_count(&self, collider: &BepuCollider) -> u32 {
        self.by_key
            .get(&Self::key_for(collider))
            .map(|entry| entry.ref_count)
            .unwrap_or(0)
    }
}
