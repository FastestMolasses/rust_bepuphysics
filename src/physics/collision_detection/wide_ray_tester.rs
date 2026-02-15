// Translated from BepuPhysics/CollisionDetection/WideRayTester.cs

use crate::physics::body_properties::{RigidPose, RigidPoseWide};
use crate::physics::collidables::ray::RayWide;
use crate::physics::collidables::shape::{IConvexShape, IShapeWide};
use crate::physics::collision_detection::ray_batchers::{
    IShapeRayHitHandler, RayData as CollisionRayData,
};
use crate::physics::trees::ray_batcher::RaySourceTrait;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;

/// Helper for creating runtime-specialized vectorized ray intersection tests
/// with shapes that support broadcasting.
pub struct WideRayTester;

impl WideRayTester {
    /// Tests multiple rays against a single shape using SIMD batching via IShapeWide.
    ///
    /// Rays are processed in Vector<float>::Count-wide chunks. If the remaining ray
    /// count is below the shape's `minimum_wide_ray_count`, we fall back to scalar
    /// per-ray testing to avoid wasting SIMD lanes.
    ///
    /// # Safety
    /// Caller must ensure `ray_source` provides valid ray data and pointers.
    pub unsafe fn test<TShape, TShapeWide, TRaySource, TRayHitHandler>(
        shape: &TShape,
        pose: &RigidPose,
        ray_source: &mut TRaySource,
        ray_hit_handler: &mut TRayHitHandler,
    ) where
        TShape: IConvexShape<Wide = TShapeWide>,
        TShapeWide: IShapeWide<TShape> + Default,
        TRaySource: RaySourceTrait,
        TRayHitHandler: IShapeRayHitHandler,
    {
        let mut ray_wide = RayWide::default();
        let mut wide = TShapeWide::default();

        // Handle internal allocation for shape types that need it (e.g. ConvexHullWide).
        if wide.internal_allocation_size() > 0 {
            let alloc_size = wide.internal_allocation_size();
            let mut memory = vec![0u8; alloc_size];
            let buffer = crate::utilities::memory::buffer::Buffer::new(
                memory.as_mut_ptr(),
                alloc_size as i32,
                0,
            );
            wide.initialize(&buffer);
            std::mem::forget(memory); // Buffer lifetime is managed by the wide shape.
        }
        wide.broadcast(shape);

        let mut poses = RigidPoseWide::default();
        RigidPoseWide::broadcast(pose, &mut poses);

        let vector_count = Vector::<f32>::LEN as i32;
        let ray_count = ray_source.ray_count();
        let min_wide = TShapeWide::minimum_wide_ray_count();
        let mut i: i32 = 0;

        while i < ray_count {
            let remaining = ray_count - i;

            if remaining < min_wide {
                // Scalar fallback for the tail — not enough rays to justify SIMD.
                for j in 0..remaining {
                    let (ray_ptr, max_t_ptr) = ray_source.get_ray_ptrs(i + j);
                    // Cast tree::RayData → collision_detection::RayData (identical repr(C) layout).
                    let ray = &*(ray_ptr as *const CollisionRayData);
                    let mut t = 0.0f32;
                    let mut normal = Vec3::ZERO;
                    if shape.ray_test(pose, ray.origin, ray.direction, &mut t, &mut normal)
                        && t <= *max_t_ptr
                    {
                        ray_hit_handler.on_ray_hit(ray, &mut *max_t_ptr, t, normal, 0);
                    }
                }
                break;
            }

            // Process up to vector_count rays in a SIMD-wide batch.
            let count = remaining.min(vector_count);

            // Gather rays into the wide bundle.
            for j in 0..count {
                let ray_ref = ray_source.get_ray(i + j);
                // Cast tree::RayData → collidables::ray::RayData (identical repr(C) layout).
                let ray_data =
                    &*(ray_ref as *const crate::physics::trees::ray_batcher::RayData
                        as *const crate::physics::collidables::ray::RayData);
                let target = GatherScatter::get_offset_instance_mut(&mut ray_wide, j as usize);
                target.gather(ray_data);
            }

            // Perform SIMD-wide ray test.
            let mut intersected = Vector::<i32>::splat(0);
            let mut t_wide = Vector::<f32>::splat(0.0);
            let mut normal_wide = Vector3Wide::default();
            wide.ray_test(
                &mut poses,
                &mut ray_wide,
                &mut intersected,
                &mut t_wide,
                &mut normal_wide,
            );

            // Scatter results back to individual rays.
            for j in 0..count {
                if intersected[j as usize] < 0 {
                    let (ray_ptr, max_t_ptr) = ray_source.get_ray_ptrs(i + j);
                    let tj = t_wide[j as usize];
                    if tj <= *max_t_ptr {
                        let scalar_normal = Vec3::new(
                            normal_wide.x[j as usize],
                            normal_wide.y[j as usize],
                            normal_wide.z[j as usize],
                        );
                        // Cast tree::RayData → collision_detection::RayData.
                        let ray = &*(ray_ptr as *const CollisionRayData);
                        ray_hit_handler.on_ray_hit(ray, &mut *max_t_ptr, tj, scalar_normal, 0);
                    }
                }
            }

            i += vector_count;
        }
    }
}
