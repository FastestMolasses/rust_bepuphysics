use glam::{Quat, Vec3};
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use crate::physics::collision_detection::support_finder::ISupportFinder as DepthRefinerSupportFinder;
use super::shape::{IShape, IConvexShape, IShapeWide, ISupportFinder};
use super::ray::RayWide;

/// Collision shape representing a sphere.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Sphere {
    /// Radius of the sphere.
    pub radius: f32,
}

impl Sphere {
    /// Creates a sphere shape.
    #[inline(always)]
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }

    /// Type id of sphere shapes.
    pub const ID: i32 = 0;
}

impl IShape for Sphere {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for Sphere {
    #[inline(always)]
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        *maximum_radius = self.radius;
        *maximum_angular_expansion = 0.0;
    }

    #[inline(always)]
    fn compute_bounds(&self, _orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        *min = Vec3::new(-self.radius, -self.radius, -self.radius);
        *max = Vec3::new(self.radius, self.radius, self.radius);
    }

    fn ray_test(
        &self,
        pose: &RigidPose,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool {
        // Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things
        // and helps avoid numerical problems.
        let inverse_d_length = 1.0 / direction.length();
        let d = direction * inverse_d_length;

        // Move the origin up to the earliest possible impact time.
        let mut o = origin - pose.position;
        let mut t_offset = -o.dot(d) - self.radius;
        if t_offset < 0.0 {
            t_offset = 0.0;
        }
        o += d * t_offset;
        let b = o.dot(d);
        let c = o.dot(o) - self.radius * self.radius;

        if b > 0.0 && c > 0.0 {
            // Ray is outside and pointing away, no hit.
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }

        let discriminant = b * b - c;
        if discriminant < 0.0 {
            // Ray misses, no hit.
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        *t = -b - discriminant.sqrt();
        if *t < -t_offset {
            *t = -t_offset;
        }
        *normal = (o + d * *t) / self.radius;
        *t = (*t + t_offset) * inverse_d_length;
        true
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = 1.0 / mass;
        inertia.inverse_inertia_tensor.xx =
            inertia.inverse_mass / ((2.0 / 5.0) * self.radius * self.radius);
        inertia.inverse_inertia_tensor.yx = 0.0;
        inertia.inverse_inertia_tensor.yy = inertia.inverse_inertia_tensor.xx;
        inertia.inverse_inertia_tensor.zx = 0.0;
        inertia.inverse_inertia_tensor.zy = 0.0;
        inertia.inverse_inertia_tensor.zz = inertia.inverse_inertia_tensor.xx;
        inertia
    }
}

/// Wide representation of a sphere for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct SphereWide {
    pub radius: Vector<f32>,
}

impl IShapeWide<Sphere> for SphereWide {
    fn allow_offset_memory_access(&self) -> bool {
        true
    }

    fn internal_allocation_size(&self) -> usize {
        0
    }

    fn initialize(&mut self, _memory: &Buffer<u8>) {}

    #[inline(always)]
    fn broadcast(&mut self, shape: &Sphere) {
        self.radius = Vector::<f32>::splat(shape.radius);
    }

    #[inline(always)]
    fn write_first(&mut self, source: &Sphere) {
        unsafe {
            *GatherScatter::get_first_mut(&mut self.radius) = source.radius;
        }
    }

    #[inline(always)]
    fn write_slot(&mut self, index: usize, source: &Sphere) {
        unsafe {
            let slot = GatherScatter::get_offset_instance_mut(self, index);
            slot.write_first(source);
        }
    }

    fn get_bounds(
        &self,
        _orientations: &mut QuaternionWide,
        _count_in_bundle: i32,
        maximum_radius: &mut Vector<f32>,
        maximum_angular_expansion: &mut Vector<f32>,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    ) {
        // Spheres have perfect symmetry, so there is no need for angular expansion.
        *maximum_radius = Vector::<f32>::splat(0.0);
        *maximum_angular_expansion = Vector::<f32>::splat(0.0);

        let negated_radius = -self.radius;
        max.x = self.radius;
        max.y = self.radius;
        max.z = self.radius;
        min.x = negated_radius;
        min.y = negated_radius;
        min.z = negated_radius;
    }

    fn minimum_wide_ray_count() -> i32 {
        2
    }

    #[inline(always)]
    fn ray_test(
        &self,
        pose: &mut RigidPoseWide,
        ray_wide: &mut RayWide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        let one = Vector::<f32>::splat(1.0);
        let zero = Vector::<f32>::splat(0.0);
        let _zero_i = Vector::<i32>::splat(0);

        // Normalize the direction.
        let inverse_d_length = {
            let len = ray_wide.direction.length();
            one / len
        };
        let mut d = Vector3Wide::default();
        Vector3Wide::scale_to(&ray_wide.direction, &inverse_d_length, &mut d);

        // Move the origin up to the earliest possible impact time.
        let mut o = Vector3Wide::default();
        Vector3Wide::subtract(&ray_wide.origin, &pose.position, &mut o);
        let dot = Vector3Wide::dot_val(&o, &d);
        let t_offset = ((-dot) - self.radius).simd_max(zero);
        let mut o_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&d, &t_offset, &mut o_offset);
        let tmp = o;
        Vector3Wide::add(&o_offset, &tmp, &mut o);
        let b = Vector3Wide::dot_val(&o, &d);
        let mut c = Vector3Wide::dot_val(&o, &o);
        c -= self.radius * self.radius;

        // If b > 0 && c > 0, ray is outside and pointing away, no hit.
        // If discriminant < 0, the ray misses.
        let discriminant = b * b - c;
        *intersected = (b.simd_le(zero) | c.simd_le(zero)).to_int()
            & discriminant.simd_ge(zero).to_int();

        *t = (-t_offset).simd_max(-b - discriminant.simd_max(zero).sqrt());
        Vector3Wide::scale_to(&d, t, &mut o_offset);
        Vector3Wide::add(&o, &o_offset, normal);
        let inverse_radius = one / self.radius;
        *normal = Vector3Wide::scale(normal, &inverse_radius);
        *t = (*t + t_offset) * inverse_d_length;
    }
}

/// Support finder for spheres.
pub struct SphereSupportFinder;

impl ISupportFinder<Sphere, SphereWide> for SphereSupportFinder {
    fn has_margin(&self) -> bool {
        true
    }

    #[inline(always)]
    fn get_margin(&self, shape: &SphereWide, margin: &mut Vector<f32>) {
        *margin = shape.radius;
    }

    #[inline(always)]
    fn compute_support(
        &self,
        _shape: &SphereWide,
        _orientation: &Matrix3x3Wide,
        _direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        *support = Vector3Wide::default();
    }

    #[inline(always)]
    fn compute_local_support(
        &self,
        _shape: &SphereWide,
        _direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        *support = Vector3Wide::default();
    }
}

impl DepthRefinerSupportFinder<SphereWide> for SphereSupportFinder {
    fn has_margin() -> bool {
        true
    }

    #[inline(always)]
    fn get_margin(shape: &SphereWide, margin: &mut Vector<f32>) {
        *margin = shape.radius;
    }

    #[inline(always)]
    fn compute_local_support(
        _shape: &SphereWide,
        _direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        *support = Vector3Wide::default();
    }

    #[inline(always)]
    fn compute_support(
        _shape: &SphereWide,
        _orientation: &Matrix3x3Wide,
        _direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        *support = Vector3Wide::default();
    }
}
