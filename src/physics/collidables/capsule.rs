use glam::{Quat, Vec3};
use std::simd::prelude::*;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use super::shape::{IShape, IConvexShape, IShapeWide, ISupportFinder};
use super::ray::RayWide;

/// Collision shape representing a sphere-expanded line segment.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Capsule {
    /// Spherical expansion applied to the internal line segment.
    pub radius: f32,
    /// Half of the length of the internal line segment. Oriented along the local Y axis.
    pub half_length: f32,
}

impl Capsule {
    /// Creates a capsule shape.
    #[inline(always)]
    pub fn new(radius: f32, length: f32) -> Self {
        Self {
            radius,
            half_length: length * 0.5,
        }
    }

    /// Gets the length of the capsule's internal line segment along the local Y axis.
    pub fn length(&self) -> f32 {
        self.half_length * 2.0
    }

    pub fn set_length(&mut self, value: f32) {
        self.half_length = value * 0.5;
    }

    /// Type id of capsule shapes.
    pub const ID: i32 = 1;
}

impl IShape for Capsule {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for Capsule {
    #[inline(always)]
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        *maximum_radius = self.half_length + self.radius;
        *maximum_angular_expansion = self.half_length;
    }

    #[inline(always)]
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut segment_offset = Vec3::ZERO;
        quaternion_ex::transform_unit_y(orientation, &mut segment_offset);
        *max = (self.half_length * segment_offset).abs() + Vec3::splat(self.radius);
        *min = -*max;
    }

    fn ray_test(
        &self,
        pose: &RigidPose,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool {
        // Work in local space.
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);
        let ow = origin - pose.position;
        let mut o = Vec3::ZERO;
        Matrix3x3::transform_transpose(&ow, &orientation, &mut o);
        let mut d = Vec3::ZERO;
        Matrix3x3::transform_transpose(&direction, &orientation, &mut d);

        // Normalize the direction.
        let inverse_d_length = 1.0 / d.length();
        d *= inverse_d_length;

        // Move the origin up to the earliest possible impact time.
        let t_offset = 0.0f32.max(-o.dot(d) - (self.half_length + self.radius));
        o += d * t_offset;

        let oh = Vec3::new(o.x, 0.0, o.z);
        let dh = Vec3::new(d.x, 0.0, d.z);
        let a = dh.dot(dh);
        let b = oh.dot(dh);
        let radius_squared = self.radius * self.radius;
        let c = oh.dot(oh) - radius_squared;

        if b > 0.0 && c > 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }

        let sphere_y;
        if a > 1e-8 {
            let discriminant = b * b - a * c;
            if discriminant < 0.0 {
                *t = 0.0;
                *normal = Vec3::ZERO;
                return false;
            }
            *t = (-b - discriminant.sqrt()) / a;
            if *t < -t_offset {
                *t = -t_offset;
            }
            let cylinder_hit = o + d * *t;
            if cylinder_hit.y < -self.half_length {
                sphere_y = -self.half_length;
            } else if cylinder_hit.y > self.half_length {
                sphere_y = self.half_length;
            } else {
                // Hit is on the cylindrical portion.
                *normal = Vec3::new(cylinder_hit.x, 0.0, cylinder_hit.z) / self.radius;
                let n = *normal;
                Matrix3x3::transform(&n, &orientation, normal);
                *t = (*t + t_offset) * inverse_d_length;
                return true;
            }
        } else {
            // Ray is parallel to the axis.
            sphere_y = if d.y > 0.0 {
                self.half_length.max(o.y.min(self.half_length)).min(self.half_length).max(-self.half_length)
            } else {
                (-self.half_length).min(o.y.max(-self.half_length)).max(-self.half_length).min(self.half_length)
            };
        }

        let os = o - Vec3::new(0.0, sphere_y, 0.0);
        let cap_b = os.dot(d);
        let cap_c = os.dot(os) - radius_squared;

        if cap_b > 0.0 && cap_c > 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }

        let cap_discriminant = cap_b * cap_b - cap_c;
        if cap_discriminant < 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        *t = (-cap_b - cap_discriminant.sqrt()).max(-t_offset);
        *normal = (os + d * *t) / self.radius;
        *t = (*t + t_offset) * inverse_d_length;
        let n = *normal;
        Matrix3x3::transform(&n, &orientation, normal);
        true
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = 1.0 / mass;
        let r2 = self.radius * self.radius;
        let h2 = self.half_length * self.half_length;
        let cylinder_volume = 2.0 * self.half_length * r2 * std::f32::consts::PI;
        let sphere_volume = (4.0 / 3.0) * r2 * self.radius * std::f32::consts::PI;
        let inverse_total = 1.0 / (cylinder_volume + sphere_volume);
        let cv = cylinder_volume * inverse_total;
        let sv = sphere_volume * inverse_total;
        inertia.inverse_inertia_tensor.xx = inertia.inverse_mass
            / (cv * ((3.0 / 12.0) * r2 + (4.0 / 12.0) * h2)
                + sv * ((2.0 / 5.0) * r2 + (6.0 / 8.0) * self.radius * self.half_length + h2));
        inertia.inverse_inertia_tensor.yx = 0.0;
        inertia.inverse_inertia_tensor.yy =
            inertia.inverse_mass / (cv * (1.0 / 2.0) * r2 + sv * (2.0 / 5.0) * r2);
        inertia.inverse_inertia_tensor.zx = 0.0;
        inertia.inverse_inertia_tensor.zy = 0.0;
        inertia.inverse_inertia_tensor.zz = inertia.inverse_inertia_tensor.xx;
        inertia
    }
}

/// Wide representation of a capsule for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct CapsuleWide {
    pub radius: Vector<f32>,
    pub half_length: Vector<f32>,
}

impl IShapeWide<Capsule> for CapsuleWide {
    fn allow_offset_memory_access(&self) -> bool {
        true
    }
    fn internal_allocation_size(&self) -> usize {
        0
    }
    fn initialize(&mut self, _memory: &Buffer<u8>) {}

    #[inline(always)]
    fn broadcast(&mut self, shape: &Capsule) {
        self.radius = Vector::<f32>::splat(shape.radius);
        self.half_length = Vector::<f32>::splat(shape.half_length);
    }

    #[inline(always)]
    fn write_first(&mut self, source: &Capsule) {
        unsafe {
            *GatherScatter::get_first_mut(&mut self.radius) = source.radius;
            *GatherScatter::get_first_mut(&mut self.half_length) = source.half_length;
        }
    }

    #[inline(always)]
    fn write_slot(&mut self, index: usize, source: &Capsule) {
        unsafe {
            GatherScatter::get_offset_instance_mut(self, index).write_first(source);
        }
    }

    fn get_bounds(
        &self,
        orientations: &mut QuaternionWide,
        _count_in_bundle: i32,
        maximum_radius: &mut Vector<f32>,
        maximum_angular_expansion: &mut Vector<f32>,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    ) {
        let segment_offset = QuaternionWide::transform_unit_y(*orientations);
        let mut abs_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&segment_offset, &self.half_length, &mut abs_offset);
        abs_offset = abs_offset.abs();

        Vector3Wide::add_scalar(&abs_offset, &self.radius, max);
        Vector3Wide::negate(max, min);

        *maximum_radius = self.half_length + self.radius;
        *maximum_angular_expansion = self.half_length;
    }

    fn minimum_wide_ray_count() -> i32 {
        2
    }

    fn ray_test(
        &self,
        pose: &mut RigidPoseWide,
        ray: &mut RayWide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        // TODO: Wide ray test implementation for capsule
        // This is complex and will be filled in when collision detection is fully wired up.
        *intersected = Vector::<i32>::splat(0);
        *t = Vector::<f32>::splat(0.0);
        *normal = Vector3Wide::default();
    }
}

/// Support finder for capsules.
pub struct CapsuleSupportFinder;

impl ISupportFinder<Capsule, CapsuleWide> for CapsuleSupportFinder {
    fn has_margin(&self) -> bool {
        true
    }

    #[inline(always)]
    fn get_margin(&self, shape: &CapsuleWide, margin: &mut Vector<f32>) {
        *margin = shape.radius;
    }

    #[inline(always)]
    fn compute_support(
        &self,
        shape: &CapsuleWide,
        orientation: &Matrix3x3Wide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        Vector3Wide::scale_to(&orientation.y, &shape.half_length, support);
        let mut negated = Vector3Wide::default();
        Vector3Wide::negate(support, &mut negated);
        let mut dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&orientation.y, direction, &mut dot);
        let zero = Vector::<f32>::splat(0.0);
        let should_negate = dot.simd_lt(zero).to_int();
        *support = Vector3Wide::conditional_select(&should_negate, &negated, support);
    }

    #[inline(always)]
    fn compute_local_support(
        &self,
        shape: &CapsuleWide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        support.x = zero;
        support.y = direction.y.simd_lt(zero).select(-shape.half_length, shape.half_length);
        support.z = zero;
    }
}
