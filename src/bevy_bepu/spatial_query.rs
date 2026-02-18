//! Spatial query system parameter for on-demand raycasting and sweeps.

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;

use super::resources::BepuSimulation;

use crate::physics::collidables::collidable_reference::{CollidableMobility, CollidableReference};
use crate::physics::collision_detection::ray_batchers::RayData;
use crate::physics::handles::{BodyHandle, StaticHandle};
use crate::physics::simulation::IRayHitHandler;

/// Result of a ray cast against the physics world.
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    /// The entity that was hit, if the handle could be resolved.
    pub entity: Option<Entity>,
    /// The parametric distance along the ray (0 = origin, 1 = origin + direction).
    pub t: f32,
    /// The surface normal at the hit point.
    pub normal: Vec3,
}

/// A system parameter for performing on-demand spatial queries against the
/// BepuPhysics simulation.
///
/// # Example
/// ```ignore
/// fn my_system(spatial: BepuSpatialQuery) {
///     if let Some(hit) = spatial.ray_cast(Vec3::new(0., 10., 0.), Vec3::NEG_Y, 100.0) {
///         info!("Hit entity {:?} at t={}", hit.entity, hit.t);
///     }
/// }
/// ```
#[derive(SystemParam)]
pub struct BepuSpatialQuery<'w> {
    sim: Res<'w, BepuSimulation>,
}

impl BepuSpatialQuery<'_> {
    /// Cast a ray and return the closest hit, if any.
    pub fn ray_cast(&self, origin: Vec3, direction: Vec3, max_t: f32) -> Option<RayHit> {
        struct ClosestHitHandler {
            best_t: f32,
            best_normal: glam::Vec3,
            best_collidable: Option<(CollidableMobility, i32)>,
        }

        impl IRayHitHandler for ClosestHitHandler {
            fn allow_test_collidable(&self, _collidable: &CollidableReference) -> bool {
                true
            }
            fn allow_test_child(
                &self,
                _collidable: &CollidableReference,
                _child_index: i32,
            ) -> bool {
                true
            }
            fn on_ray_hit(
                &mut self,
                _ray: &RayData,
                maximum_t: &mut f32,
                t: f32,
                normal: glam::Vec3,
                collidable: &CollidableReference,
                _child_index: i32,
            ) {
                if t < self.best_t {
                    self.best_t = t;
                    self.best_normal = normal;
                    self.best_collidable =
                        Some((collidable.mobility(), collidable.raw_handle_value()));
                    *maximum_t = t; // Narrow the search to closer hits only.
                }
            }
        }

        let mut handler = ClosestHitHandler {
            best_t: max_t,
            best_normal: glam::Vec3::ZERO,
            best_collidable: None,
        };

        unsafe {
            self.sim.simulation.ray_cast(
                glam::Vec3::new(origin.x, origin.y, origin.z),
                glam::Vec3::new(direction.x, direction.y, direction.z),
                max_t,
                &mut handler,
                0,
            );
        }

        handler.best_collidable.map(|(mobility, raw_handle)| {
            let entity = match mobility {
                CollidableMobility::Dynamic | CollidableMobility::Kinematic => {
                    let bh = BodyHandle(raw_handle);
                    self.sim.body_to_entity.get(&bh).copied()
                }
                CollidableMobility::Static => {
                    let sh = StaticHandle(raw_handle);
                    self.sim.static_to_entity.get(&sh).copied()
                }
            };

            RayHit {
                entity,
                t: handler.best_t,
                normal: Vec3::new(
                    handler.best_normal.x,
                    handler.best_normal.y,
                    handler.best_normal.z,
                ),
            }
        })
    }
}
