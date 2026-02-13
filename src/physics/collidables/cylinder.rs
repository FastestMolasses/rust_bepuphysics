use glam::{Quat, Vec3};
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

use super::ray::RayWide;
use super::shape::{IConvexShape, IShape, IShapeWide, IShapeWideAllocation, ISupportFinder};
use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use crate::physics::collision_detection::support_finder::ISupportFinder as DepthRefinerSupportFinder;

/// Collision shape representing a cylinder.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Cylinder {
    /// Radius of the cylinder.
    pub radius: f32,
    /// Half length of the cylinder along its local Y axis.
    pub half_length: f32,
}

impl Cylinder {
    /// Creates a cylinder shape.
    #[inline(always)]
    pub fn new(radius: f32, length: f32) -> Self {
        Self {
            radius,
            half_length: length * 0.5,
        }
    }

    /// Gets the length of the cylinder along its local Y axis.
    pub fn length(&self) -> f32 {
        self.half_length * 2.0
    }

    pub fn set_length(&mut self, value: f32) {
        self.half_length = value * 0.5;
    }

    /// Type id of cylinder shapes.
    pub const ID: i32 = 4;
}

impl IShape for Cylinder {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for Cylinder {
    #[inline(always)]
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        *maximum_radius = (self.half_length * self.half_length + self.radius * self.radius).sqrt();
        *maximum_angular_expansion = *maximum_radius - self.half_length.min(self.radius);
    }

    #[inline(always)]
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut y = Vec3::ZERO;
        quaternion_ex::transform_unit_y(orientation, &mut y);
        let positive_disc_bound_offsets = Vec3::new(
            (1.0 - y.x * y.x).max(0.0).sqrt(),
            (1.0 - y.y * y.y).max(0.0).sqrt(),
            (1.0 - y.z * y.z).max(0.0).sqrt(),
        ) * self.radius;
        *max = (self.half_length * y).abs() + positive_disc_bound_offsets;
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
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);
        let ow = origin - pose.position;
        let mut o = Vec3::ZERO;
        Matrix3x3::transform_transpose(&ow, &orientation, &mut o);
        let mut d = Vec3::ZERO;
        Matrix3x3::transform_transpose(&direction, &orientation, &mut d);

        let inverse_d_length = 1.0 / d.length();
        d *= inverse_d_length;

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

        let disc_y;
        if a > 1e-8 {
            let discriminant = b * b - a * c;
            if discriminant < 0.0 {
                *t = 0.0;
                *normal = Vec3::ZERO;
                return false;
            }
            *t = (-b - discriminant.sqrt()) / a;
            *t = (*t).max(-t_offset);
            let cylinder_hit = o + d * *t;
            if cylinder_hit.y < -self.half_length {
                disc_y = -self.half_length;
            } else if cylinder_hit.y > self.half_length {
                disc_y = self.half_length;
            } else {
                // Hit is on the side of the cylinder.
                *normal = Vec3::new(cylinder_hit.x, 0.0, cylinder_hit.z) / self.radius;
                let n = *normal;
                Matrix3x3::transform(&n, &orientation, normal);
                *t = (*t + t_offset) * inverse_d_length;
                return true;
            }
        } else {
            // Ray is parallel to the axis.
            disc_y = if d.y > 0.0 {
                (-self.half_length).min(o.y)
            } else {
                self.half_length.max(o.y).min(self.half_length)
            };
        }

        // Intersect with disc cap at disc_y.
        if o.y.abs() > self.half_length && o.y * d.y >= 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        *t = (disc_y - o.y) / d.y;
        let hit_location = o + d * *t;
        if hit_location.x * hit_location.x + hit_location.z * hit_location.z > radius_squared {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        *t = (*t + t_offset) * inverse_d_length;
        *normal = if d.y < 0.0 {
            Vec3::new(orientation.y.x, orientation.y.y, orientation.y.z)
        } else {
            Vec3::new(-orientation.y.x, -orientation.y.y, -orientation.y.z)
        };
        true
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = 1.0 / mass;
        let diag_value = inertia.inverse_mass
            / (4.0 * 0.0833333333 * self.half_length * self.half_length
                + 0.25 * self.radius * self.radius);
        inertia.inverse_inertia_tensor.xx = diag_value;
        inertia.inverse_inertia_tensor.yx = 0.0;
        inertia.inverse_inertia_tensor.yy =
            2.0 * inertia.inverse_mass / (self.radius * self.radius);
        inertia.inverse_inertia_tensor.zx = 0.0;
        inertia.inverse_inertia_tensor.zy = 0.0;
        inertia.inverse_inertia_tensor.zz = diag_value;
        inertia
    }
}

/// Wide representation of a cylinder for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct CylinderWide {
    pub radius: Vector<f32>,
    pub half_length: Vector<f32>,
}

impl IShapeWideAllocation for CylinderWide {}

impl IShapeWide<Cylinder> for CylinderWide {
    fn allow_offset_memory_access(&self) -> bool {
        true
    }
    fn internal_allocation_size(&self) -> usize {
        0
    }
    fn initialize(&mut self, _memory: &Buffer<u8>) {}

    #[inline(always)]
    fn broadcast(&mut self, shape: &Cylinder) {
        self.radius = Vector::<f32>::splat(shape.radius);
        self.half_length = Vector::<f32>::splat(shape.half_length);
    }

    #[inline(always)]
    fn write_first(&mut self, source: &Cylinder) {
        unsafe {
            *GatherScatter::get_first_mut(&mut self.radius) = source.radius;
            *GatherScatter::get_first_mut(&mut self.half_length) = source.half_length;
        }
    }

    #[inline(always)]
    fn write_slot(&mut self, index: usize, source: &Cylinder) {
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
        let y = QuaternionWide::transform_unit_y(*orientations);
        let mut yy = Vector3Wide::default();
        Vector3Wide::multiply(&y, &y, &mut yy);
        let one = Vector::<f32>::splat(1.0);
        let zero = Vector::<f32>::splat(0.0);
        let mut squared = Vector3Wide::default();
        Vector3Wide::subtract_from_scalar(&one, &yy, &mut squared);
        max.x = (self.half_length * y.x).abs() + (squared.x.simd_max(zero)).sqrt() * self.radius;
        max.y = (self.half_length * y.y).abs() + (squared.y.simd_max(zero)).sqrt() * self.radius;
        max.z = (self.half_length * y.z).abs() + (squared.z.simd_max(zero)).sqrt() * self.radius;
        Vector3Wide::negate(max, min);

        *maximum_radius = (self.half_length * self.half_length + self.radius * self.radius).sqrt();
        *maximum_angular_expansion = *maximum_radius - self.half_length.simd_min(self.radius);
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
        // Work in local space.
        let mut orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(&pose.orientation, &mut orientation);
        let mut o_world = Vector3Wide::default();
        Vector3Wide::subtract(&ray.origin, &pose.position, &mut o_world);
        let mut o = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(&o_world, &orientation, &mut o);
        let mut d = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &ray.direction,
            &orientation,
            &mut d,
        );

        // Normalize direction.
        let mut d_length = Vector::<f32>::splat(0.0);
        Vector3Wide::length_into(&d, &mut d_length);
        let inverse_d_length = Vector::<f32>::splat(1.0) / d_length;
        let d_copy = d;
        Vector3Wide::scale_to(&d_copy, &inverse_d_length, &mut d);

        // Move origin to earliest possible impact time.
        let mut od = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&o, &d, &mut od);
        let t_offset = (-od - (self.half_length + self.radius)).simd_max(Vector::<f32>::splat(0.0));
        let mut o_offset = Vector3Wide::default();
        Vector3Wide::scale_to(&d, &t_offset, &mut o_offset);
        let o_copy = o;
        Vector3Wide::add(&o_copy, &o_offset, &mut o);

        let a = d.x * d.x + d.z * d.z;
        let b = o.x * d.x + o.z * d.z;
        let radius_squared = self.radius * self.radius;
        let c = (o.x * o.x + o.z * o.z) - radius_squared;

        let ray_isnt_parallel = a.simd_gt(Vector::<f32>::splat(1e-8));
        let discriminant = b * b - a * c;
        let cylinder_intersected = (b.simd_le(Vector::<f32>::splat(0.0))
            | c.simd_le(Vector::<f32>::splat(0.0)))
            & discriminant.simd_ge(Vector::<f32>::splat(0.0));
        let cylinder_t = ((-b - discriminant.sqrt()) / a).simd_max(-t_offset);
        let mut cylinder_hit_location = Vector3Wide::default();
        Vector3Wide::scale_to(&d, &cylinder_t, &mut o_offset);
        Vector3Wide::add(&o, &o_offset, &mut cylinder_hit_location);
        let inverse_radius = Vector::<f32>::splat(1.0) / self.radius;
        let cylinder_normal_x = cylinder_hit_location.x * inverse_radius;
        let cylinder_normal_z = cylinder_hit_location.z * inverse_radius;
        let use_cylinder = cylinder_hit_location.y.simd_ge(-self.half_length)
            & cylinder_hit_location.y.simd_le(self.half_length);

        // Disc cap intersection.
        let disc_y = ((cylinder_hit_location.y.simd_gt(self.half_length) & ray_isnt_parallel)
            | (d.y.simd_le(Vector::<f32>::splat(0.0)) & !ray_isnt_parallel))
            .select(self.half_length, -self.half_length);

        let within_discs_or_toward =
            o.y.abs().simd_le(self.half_length) | (o.y * d.y).simd_lt(Vector::<f32>::splat(0.0));

        let cap_t = (disc_y - o.y) / d.y;
        let hit_location_x = o.x + d.x * cap_t;
        let hit_location_z = o.z + d.z * cap_t;
        let cap_hit_within_radius = (hit_location_x * hit_location_x
            + hit_location_z * hit_location_z)
            .simd_le(radius_squared);
        let hit_cap = within_discs_or_toward & cap_hit_within_radius;

        let zero = Vector::<f32>::splat(0.0);
        let one = Vector::<f32>::splat(1.0);
        let neg_one = Vector::<f32>::splat(-1.0);

        *t = (t_offset + use_cylinder.select(cylinder_t, hit_cap.select(cap_t, zero)))
            * inverse_d_length;

        let cap_uses_upward = d.y.simd_lt(zero);
        let mut local_normal = Vector3Wide::default();
        local_normal.x = use_cylinder.select(cylinder_normal_x, zero);
        local_normal.y = use_cylinder.select(zero, cap_uses_upward.select(one, neg_one));
        local_normal.z = use_cylinder.select(cylinder_normal_z, zero);
        Matrix3x3Wide::transform_without_overlap(&local_normal, &orientation, normal);
        *intersected = use_cylinder.select(cylinder_intersected.to_int(), hit_cap.to_int());
    }
}

/// Support finder for cylinders.
#[derive(Default)]
pub struct CylinderSupportFinder;

impl ISupportFinder<Cylinder, CylinderWide> for CylinderSupportFinder {
    fn has_margin(&self) -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(&self, _shape: &CylinderWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::default();
    }

    #[inline(always)]
    fn compute_support(
        &self,
        shape: &CylinderWide,
        orientation: &Matrix3x3Wide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let mut local_direction = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            direction,
            orientation,
            &mut local_direction,
        );
        let mut local_support = Vector3Wide::default();
        self.compute_local_support(
            shape,
            &local_direction,
            terminated_lanes,
            &mut local_support,
        );
        Matrix3x3Wide::transform_without_overlap(&local_support, orientation, support);
    }

    #[inline(always)]
    fn compute_local_support(
        &self,
        shape: &CylinderWide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        support.y = direction
            .y
            .simd_gt(zero)
            .select(shape.half_length, -shape.half_length);
        let horizontal_length = (direction.x * direction.x + direction.z * direction.z).sqrt();
        let normalize_scale = shape.radius / horizontal_length;
        let use_horizontal = horizontal_length.simd_gt(Vector::<f32>::splat(1e-8));
        support.x = use_horizontal.select(direction.x * normalize_scale, zero);
        support.z = use_horizontal.select(direction.z * normalize_scale, zero);
    }
}

impl DepthRefinerSupportFinder<CylinderWide> for CylinderSupportFinder {
    fn has_margin() -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(_shape: &CylinderWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::default();
    }

    #[inline(always)]
    fn compute_local_support(
        shape: &CylinderWide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        support.y = direction
            .y
            .simd_gt(zero)
            .select(shape.half_length, -shape.half_length);
        let horizontal_length = (direction.x * direction.x + direction.z * direction.z).sqrt();
        let normalize_scale = shape.radius / horizontal_length;
        let use_horizontal = horizontal_length.simd_gt(Vector::<f32>::splat(1e-8));
        support.x = use_horizontal.select(direction.x * normalize_scale, zero);
        support.z = use_horizontal.select(direction.z * normalize_scale, zero);
    }

    #[inline(always)]
    fn compute_support(
        shape: &CylinderWide,
        orientation: &Matrix3x3Wide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let mut local_direction = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            direction,
            orientation,
            &mut local_direction,
        );
        let mut local_support = Vector3Wide::default();
        <Self as DepthRefinerSupportFinder<CylinderWide>>::compute_local_support(
            shape,
            &local_direction,
            terminated_lanes,
            &mut local_support,
        );
        Matrix3x3Wide::transform_without_overlap(&local_support, orientation, support);
    }
}
