use glam::{Quat, Vec3};
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use super::shape::{IShape, IConvexShape, IShapeWide, ISupportFinder};
use super::ray::RayWide;

/// Collision shape representing a solid cuboid.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Box {
    /// Half of the box's width along its local X axis.
    pub half_width: f32,
    /// Half of the box's height along its local Y axis.
    pub half_height: f32,
    /// Half of the box's length along its local Z axis.
    pub half_length: f32,
}

impl Box {
    /// Creates a Box shape.
    #[inline(always)]
    pub fn new(width: f32, height: f32, length: f32) -> Self {
        Self {
            half_width: width * 0.5,
            half_height: height * 0.5,
            half_length: length * 0.5,
        }
    }

    pub fn width(&self) -> f32 {
        self.half_width * 2.0
    }
    pub fn height(&self) -> f32 {
        self.half_height * 2.0
    }
    pub fn length(&self) -> f32 {
        self.half_length * 2.0
    }
    pub fn set_width(&mut self, value: f32) {
        self.half_width = value * 0.5;
    }
    pub fn set_height(&mut self, value: f32) {
        self.half_height = value * 0.5;
    }
    pub fn set_length(&mut self, value: f32) {
        self.half_length = value * 0.5;
    }

    /// Type id of box shapes.
    pub const ID: i32 = 2;
}

impl IShape for Box {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for Box {
    #[inline(always)]
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut basis = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&orientation, &mut basis);
        let x = self.half_width * basis.x;
        let y = self.half_height * basis.y;
        let z = self.half_length * basis.z;
        *max = x.abs() + y.abs() + z.abs();
        *min = -*max;
    }

    #[inline(always)]
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        *maximum_radius = (self.half_width * self.half_width
            + self.half_height * self.half_height
            + self.half_length * self.half_length)
            .sqrt();
        let min_half = self.half_length.min(self.half_height.min(self.half_length));
        *maximum_angular_expansion = *maximum_radius - min_half;
    }

    fn ray_test(
        &self,
        pose: &RigidPose,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool {
        let offset = origin - pose.position;
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);
        let mut local_offset = Vec3::ZERO;
        Matrix3x3::transform_transpose(&offset, &orientation, &mut local_offset);
        let mut local_direction = Vec3::ZERO;
        Matrix3x3::transform_transpose(&direction, &orientation, &mut local_direction);

        let offset_to_t_scale = Vec3::new(
            if local_direction.x < 0.0 { 1.0 } else { -1.0 },
            if local_direction.y < 0.0 { 1.0 } else { -1.0 },
            if local_direction.z < 0.0 { 1.0 } else { -1.0 },
        ) / local_direction.abs().max(Vec3::splat(1e-15));

        let half_extent = Vec3::new(self.half_width, self.half_height, self.half_length);
        let negative_t = (local_offset - half_extent) * offset_to_t_scale;
        let positive_t = (local_offset + half_extent) * offset_to_t_scale;
        let entry_t = negative_t.min(positive_t);
        let exit_t = negative_t.max(positive_t);

        let mut earliest_exit = exit_t.x.min(exit_t.y);
        if exit_t.z < earliest_exit {
            earliest_exit = exit_t.z;
        }
        if earliest_exit < 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }

        let latest_entry;
        if entry_t.x > entry_t.y {
            if entry_t.x > entry_t.z {
                latest_entry = entry_t.x;
                *normal = orientation.x;
            } else {
                latest_entry = entry_t.z;
                *normal = orientation.z;
            }
        } else {
            if entry_t.y > entry_t.z {
                latest_entry = entry_t.y;
                *normal = orientation.y;
            } else {
                latest_entry = entry_t.z;
                *normal = orientation.z;
            }
        }

        if earliest_exit < latest_entry {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        *t = if latest_entry < 0.0 {
            0.0
        } else {
            latest_entry
        };
        // The normal should point away from the center of the box.
        if normal.dot(offset) < 0.0 {
            *normal = -*normal;
        }
        true
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = 1.0 / mass;
        let x2 = self.half_width * self.half_width;
        let y2 = self.half_height * self.half_height;
        let z2 = self.half_length * self.half_length;
        inertia.inverse_inertia_tensor.xx = inertia.inverse_mass * 3.0 / (y2 + z2);
        inertia.inverse_inertia_tensor.yx = 0.0;
        inertia.inverse_inertia_tensor.yy = inertia.inverse_mass * 3.0 / (x2 + z2);
        inertia.inverse_inertia_tensor.zx = 0.0;
        inertia.inverse_inertia_tensor.zy = 0.0;
        inertia.inverse_inertia_tensor.zz = inertia.inverse_mass * 3.0 / (x2 + y2);
        inertia
    }
}

/// Wide representation of a box for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct BoxWide {
    pub half_width: Vector<f32>,
    pub half_height: Vector<f32>,
    pub half_length: Vector<f32>,
}

impl IShapeWide<Box> for BoxWide {
    fn allow_offset_memory_access(&self) -> bool {
        true
    }

    fn internal_allocation_size(&self) -> usize {
        0
    }

    fn initialize(&mut self, _memory: &Buffer<u8>) {}

    #[inline(always)]
    fn broadcast(&mut self, shape: &Box) {
        self.half_width = Vector::<f32>::splat(shape.half_width);
        self.half_height = Vector::<f32>::splat(shape.half_height);
        self.half_length = Vector::<f32>::splat(shape.half_length);
    }

    #[inline(always)]
    fn write_first(&mut self, source: &Box) {
        unsafe {
            *GatherScatter::get_first_mut(&mut self.half_width) = source.half_width;
            *GatherScatter::get_first_mut(&mut self.half_height) = source.half_height;
            *GatherScatter::get_first_mut(&mut self.half_length) = source.half_length;
        }
    }

    #[inline(always)]
    fn write_slot(&mut self, index: usize, source: &Box) {
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
        let mut basis = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientations, &mut basis);
        max.x = (self.half_width * basis.x.x).abs()
            + (self.half_height * basis.y.x).abs()
            + (self.half_length * basis.z.x).abs();
        max.y = (self.half_width * basis.x.y).abs()
            + (self.half_height * basis.y.y).abs()
            + (self.half_length * basis.z.y).abs();
        max.z = (self.half_width * basis.x.z).abs()
            + (self.half_height * basis.y.z).abs()
            + (self.half_length * basis.z.z).abs();

        Vector3Wide::negate(max, min);

        *maximum_radius = (self.half_width * self.half_width
            + self.half_height * self.half_height
            + self.half_length * self.half_length)
            .sqrt();
        *maximum_angular_expansion =
            *maximum_radius - self.half_length.simd_min(self.half_height.simd_min(self.half_length));
    }

    fn minimum_wide_ray_count() -> i32 {
        3
    }

    fn ray_test(
        &self,
        pose: &mut RigidPoseWide,
        ray: &mut RayWide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        let mut offset = Vector3Wide::default();
        Vector3Wide::subtract(&ray.origin, &pose.position, &mut offset);
        let mut orientation = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(&pose.orientation, &mut orientation);
        let mut local_offset = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &offset,
            &orientation,
            &mut local_offset,
        );
        let mut local_direction = Vector3Wide::default();
        Matrix3x3Wide::transform_by_transposed_without_overlap(
            &ray.direction,
            &orientation,
            &mut local_direction,
        );

        let negative_one = -Vector::<f32>::splat(1.0);
        let epsilon = Vector::<f32>::splat(1e-15);
        let zero = Vector::<f32>::splat(0.0);

        let offset_to_t_scale_x = local_direction
            .x
            .simd_gt(zero)
            .select(negative_one, Vector::<f32>::splat(1.0))
            / local_direction.x.abs().simd_max(epsilon);
        let offset_to_t_scale_y = local_direction
            .y
            .simd_gt(zero)
            .select(negative_one, Vector::<f32>::splat(1.0))
            / local_direction.y.abs().simd_max(epsilon);
        let offset_to_t_scale_z = local_direction
            .z
            .simd_gt(zero)
            .select(negative_one, Vector::<f32>::splat(1.0))
            / local_direction.z.abs().simd_max(epsilon);

        // Compute impact times for each pair of planes in local space.
        let neg_t_x = (local_offset.x - self.half_width) * offset_to_t_scale_x;
        let neg_t_y = (local_offset.y - self.half_height) * offset_to_t_scale_y;
        let neg_t_z = (local_offset.z - self.half_length) * offset_to_t_scale_z;
        let pos_t_x = (local_offset.x + self.half_width) * offset_to_t_scale_x;
        let pos_t_y = (local_offset.y + self.half_height) * offset_to_t_scale_y;
        let pos_t_z = (local_offset.z + self.half_length) * offset_to_t_scale_z;

        let entry_t_x = neg_t_x.simd_min(pos_t_x);
        let entry_t_y = neg_t_y.simd_min(pos_t_y);
        let entry_t_z = neg_t_z.simd_min(pos_t_z);
        let exit_t_x = neg_t_x.simd_max(pos_t_x);
        let exit_t_y = neg_t_y.simd_max(pos_t_y);
        let exit_t_z = neg_t_z.simd_max(pos_t_z);

        let earliest_exit = exit_t_x.simd_min(exit_t_y).simd_min(exit_t_z);
        let earliest_entry = entry_t_x.simd_max(entry_t_y).simd_max(entry_t_z);
        *t = zero.simd_max(earliest_entry);
        *intersected = t.simd_le(earliest_exit).to_int();

        let use_x = earliest_entry.simd_eq(entry_t_x);
        let use_y = earliest_entry.simd_eq(entry_t_y) & !use_x;

        normal.x = use_x.select(
            orientation.x.x,
            use_y.select(orientation.y.x, orientation.z.x),
        );
        normal.y = use_x.select(
            orientation.x.y,
            use_y.select(orientation.y.y, orientation.z.y),
        );
        normal.z = use_x.select(
            orientation.x.z,
            use_y.select(orientation.y.z, orientation.z.z),
        );

        let dot = Vector3Wide::dot_val(normal, &offset);
        let should_negate = dot.simd_lt(zero);
        normal.x = should_negate.select(-normal.x, normal.x);
        normal.y = should_negate.select(-normal.y, normal.y);
        normal.z = should_negate.select(-normal.z, normal.z);
    }
}

/// Support finder for boxes.
pub struct BoxSupportFinder;

impl ISupportFinder<Box, BoxWide> for BoxSupportFinder {
    fn has_margin(&self) -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(&self, _shape: &BoxWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::splat(0.0);
    }

    #[inline(always)]
    fn compute_support(
        &self,
        shape: &BoxWide,
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
        self.compute_local_support(shape, &local_direction, terminated_lanes, support);
        let local_support = *support;
        Matrix3x3Wide::transform_without_overlap(&local_support, orientation, support);
    }

    #[inline(always)]
    fn compute_local_support(
        &self,
        shape: &BoxWide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let zero = Vector::<f32>::splat(0.0);
        support.x = direction.x.simd_lt(zero).select(-shape.half_width, shape.half_width);
        support.y = direction.y.simd_lt(zero).select(-shape.half_height, shape.half_height);
        support.z = direction.z.simd_lt(zero).select(-shape.half_length, shape.half_length);
    }
}
