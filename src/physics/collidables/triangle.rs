use glam::{Quat, Vec3};
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::math_helper;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;

use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use super::shape::{IShape, IConvexShape, IShapeWide, ISupportFinder};
use super::ray::RayWide;

/// Collision shape representing an individual triangle.
/// Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound
/// clockwise in right handed coordinates or counterclockwise in left handed coordinates will
/// generate contacts.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    /// First vertex of the triangle in local space.
    pub a: Vec3,
    /// Second vertex of the triangle in local space.
    pub b: Vec3,
    /// Third vertex of the triangle in local space.
    pub c: Vec3,
}

impl Triangle {
    /// Creates a triangle shape.
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        Self { a, b, c }
    }

    /// Type id of triangle shapes.
    pub const ID: i32 = 3;

    /// Tests a ray against a triangle defined by vertices a, b, c.
    /// Assumes clockwise winding in right-handed coordinates. Triangles are one-sided.
    #[inline(always)]
    pub fn ray_test_vertices(
        a: Vec3,
        b: Vec3,
        c: Vec3,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool {
        let ab = b - a;
        let ac = c - a;
        *normal = ac.cross(ab);
        let dn = -direction.dot(*normal);
        if dn <= 0.0 {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        }
        let ao = origin - a;
        *t = ao.dot(*normal);
        if *t < 0.0 {
            return false;
        }
        let aoxd = ao.cross(direction);
        let v = -ac.dot(aoxd);
        if v < 0.0 || v > dn {
            return false;
        }
        let w = ab.dot(aoxd);
        if w < 0.0 || v + w > dn {
            return false;
        }
        *t /= dn;
        *normal /= normal.dot(*normal).sqrt();
        true
    }
}

impl IShape for Triangle {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for Triangle {
    #[inline(always)]
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        *maximum_radius = math_helper::max(
            self.a.length_squared(),
            math_helper::max(self.b.length_squared(), self.c.length_squared()),
        )
        .sqrt();
        *maximum_angular_expansion = *maximum_radius;
    }

    #[inline(always)]
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut basis = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&orientation, &mut basis);
        let mut world_a = Vec3::ZERO;
        let mut world_b = Vec3::ZERO;
        let mut world_c = Vec3::ZERO;
        Matrix3x3::transform(&self.a, &basis, &mut world_a);
        Matrix3x3::transform(&self.b, &basis, &mut world_b);
        Matrix3x3::transform(&self.c, &basis, &mut world_c);
        *min = world_a.min(world_b.min(world_c));
        *max = world_a.max(world_b.max(world_c));
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
        if Triangle::ray_test_vertices(self.a, self.b, self.c, local_offset, local_direction, t, normal)
        {
            let n = *normal;
            Matrix3x3::transform(&n, &orientation, normal);
            return true;
        }
        false
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        // TODO: Requires MeshInertiaHelper.ComputeTriangleContribution + Symmetric3x3::invert
        // For now, return a placeholder inertia.
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = 1.0 / mass;
        // Approximate triangle as a thin disc with radius = max vertex distance
        let max_r2 = self
            .a
            .length_squared()
            .max(self.b.length_squared().max(self.c.length_squared()));
        let inv = inertia.inverse_mass / (0.25 * max_r2);
        inertia.inverse_inertia_tensor.xx = inv;
        inertia.inverse_inertia_tensor.yx = 0.0;
        inertia.inverse_inertia_tensor.yy = inv * 2.0;
        inertia.inverse_inertia_tensor.zx = 0.0;
        inertia.inverse_inertia_tensor.zy = 0.0;
        inertia.inverse_inertia_tensor.zz = inv;
        inertia
    }
}

/// Wide representation of a triangle for SIMD processing.
#[derive(Clone, Copy, Default)]
pub struct TriangleWide {
    pub a: Vector3Wide,
    pub b: Vector3Wide,
    pub c: Vector3Wide,
}

impl TriangleWide {
    /// Minimum dot product between the detected local normal and the face normal
    /// of a triangle necessary to create contacts.
    pub const BACKFACE_NORMAL_DOT_REJECTION_THRESHOLD: f32 = -1e-2;

    /// Epsilon to apply to testing triangles for degeneracy (scaled by a pair-determined epsilon scale).
    pub const DEGENERATE_TRIANGLE_EPSILON: f32 = 1e-6;

    /// Computes the epsilon scale for triangle degeneracy testing.
    #[inline(always)]
    pub fn compute_triangle_epsilon_scale(
        ab_length_squared: &Vector<f32>,
        ca_length_squared: &Vector<f32>,
        epsilon_scale: &mut Vector<f32>,
    ) {
        *epsilon_scale = ab_length_squared.simd_max(*ca_length_squared).sqrt();
    }

    /// Computes the degenerate triangle epsilon.
    #[inline(always)]
    pub fn compute_degenerate_triangle_epsilon(
        ab_length_squared: &Vector<f32>,
        ca_length_squared: &Vector<f32>,
        epsilon_scale: &mut Vector<f32>,
        epsilon: &mut Vector<f32>,
    ) {
        Self::compute_triangle_epsilon_scale(ab_length_squared, ca_length_squared, epsilon_scale);
        *epsilon = Vector::<f32>::splat(Self::DEGENERATE_TRIANGLE_EPSILON) * *epsilon_scale;
    }

    /// Computes a mask indicating which triangle lanes are non-degenerate.
    #[inline(always)]
    pub fn compute_nondegenerate_triangle_mask_from_edges(
        ab: &Vector3Wide,
        ca: &Vector3Wide,
        triangle_normal_length: &Vector<f32>,
        epsilon_scale: &mut Vector<f32>,
        nondegenerate_mask: &mut Vector<i32>,
    ) {
        let mut ab_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(ab, &mut ab_length_squared);
        let mut ca_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(ca, &mut ca_length_squared);
        let mut epsilon = Vector::<f32>::splat(0.0);
        Self::compute_degenerate_triangle_epsilon(
            &ab_length_squared,
            &ca_length_squared,
            epsilon_scale,
            &mut epsilon,
        );
        *nondegenerate_mask = triangle_normal_length.simd_gt(epsilon).to_int();
    }

    /// Computes a mask indicating which triangle lanes are non-degenerate from precomputed lengths.
    #[inline(always)]
    pub fn compute_nondegenerate_triangle_mask(
        ab_length_squared: &Vector<f32>,
        ca_length_squared: &Vector<f32>,
        triangle_normal_length: &Vector<f32>,
        epsilon_scale: &mut Vector<f32>,
        nondegenerate_mask: &mut Vector<i32>,
    ) {
        let mut epsilon = Vector::<f32>::splat(0.0);
        Self::compute_degenerate_triangle_epsilon(
            ab_length_squared,
            ca_length_squared,
            epsilon_scale,
            &mut epsilon,
        );
        *nondegenerate_mask = triangle_normal_length.simd_gt(epsilon).to_int();
    }

    /// Wide ray test against triangle vertices.
    #[inline(always)]
    pub fn ray_test_wide(
        a: &Vector3Wide,
        b: &Vector3Wide,
        c: &Vector3Wide,
        origin: &Vector3Wide,
        direction: &Vector3Wide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        let mut ab = Vector3Wide::default();
        Vector3Wide::subtract(b, a, &mut ab);
        let mut ac = Vector3Wide::default();
        Vector3Wide::subtract(c, a, &mut ac);
        Vector3Wide::cross(&ac, &ab, normal);
        let mut dn = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(direction, normal, &mut dn);
        dn = -dn;
        let mut ao = Vector3Wide::default();
        Vector3Wide::subtract(origin, a, &mut ao);
        Vector3Wide::dot(&ao, normal, t);
        *t = *t / dn;
        let mut aoxd = Vector3Wide::default();
        Vector3Wide::cross(&ao, direction, &mut aoxd);
        let mut v = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&ac, &aoxd, &mut v);
        v = -v;
        let mut w = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&ab, &aoxd, &mut w);
        *normal = Vector3Wide::normalize(normal);
        let zero = Vector::<f32>::splat(0.0);
        *intersected = (dn.simd_gt(zero)
            & t.simd_ge(zero)
            & v.simd_ge(zero)
            & w.simd_ge(zero)
            & (v + w).simd_le(dn))
        .to_int();
    }
}

impl IShapeWide<Triangle> for TriangleWide {
    fn allow_offset_memory_access(&self) -> bool {
        true
    }
    fn internal_allocation_size(&self) -> usize {
        0
    }
    fn initialize(&mut self, _memory: &Buffer<u8>) {}

    #[inline(always)]
    fn broadcast(&mut self, shape: &Triangle) {
        self.a = Vector3Wide::broadcast(shape.a);
        self.b = Vector3Wide::broadcast(shape.b);
        self.c = Vector3Wide::broadcast(shape.c);
    }

    #[inline(always)]
    fn write_first(&mut self, source: &Triangle) {
        Vector3Wide::write_first(source.a, &mut self.a);
        Vector3Wide::write_first(source.b, &mut self.b);
        Vector3Wide::write_first(source.c, &mut self.c);
    }

    #[inline(always)]
    fn write_slot(&mut self, index: usize, source: &Triangle) {
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
        let mut world_a = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&self.a, &basis, &mut world_a);
        let mut world_b = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&self.b, &basis, &mut world_b);
        let mut world_c = Vector3Wide::default();
        Matrix3x3Wide::transform_without_overlap(&self.c, &basis, &mut world_c);
        min.x = world_a.x.simd_min(world_b.x.simd_min(world_c.x));
        min.y = world_a.y.simd_min(world_b.y.simd_min(world_c.y));
        min.z = world_a.z.simd_min(world_b.z.simd_min(world_c.z));
        max.x = world_a.x.simd_max(world_b.x.simd_max(world_c.x));
        max.y = world_a.y.simd_max(world_b.y.simd_max(world_c.y));
        max.z = world_a.z.simd_max(world_b.z.simd_max(world_c.z));

        let mut a_length_squared = Vector::<f32>::splat(0.0);
        let mut b_length_squared = Vector::<f32>::splat(0.0);
        let mut c_length_squared = Vector::<f32>::splat(0.0);
        Vector3Wide::length_squared_to(&self.a, &mut a_length_squared);
        Vector3Wide::length_squared_to(&self.b, &mut b_length_squared);
        Vector3Wide::length_squared_to(&self.c, &mut c_length_squared);
        *maximum_radius = a_length_squared
            .simd_max(b_length_squared.simd_max(c_length_squared))
            .sqrt();
        *maximum_angular_expansion = *maximum_radius;
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
        let mut local_normal = Vector3Wide::default();
        TriangleWide::ray_test_wide(
            &self.a,
            &self.b,
            &self.c,
            &local_offset,
            &local_direction,
            intersected,
            t,
            &mut local_normal,
        );
        Matrix3x3Wide::transform_without_overlap(&local_normal, &orientation, normal);
    }
}

/// Support finder for triangles.
pub struct TriangleSupportFinder;

impl ISupportFinder<Triangle, TriangleWide> for TriangleSupportFinder {
    fn has_margin(&self) -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(&self, _shape: &TriangleWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::default();
    }

    #[inline(always)]
    fn compute_support(
        &self,
        shape: &TriangleWide,
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
        self.compute_local_support(shape, &local_direction, terminated_lanes, &mut local_support);
        Matrix3x3Wide::transform_without_overlap(&local_support, orientation, support);
    }

    #[inline(always)]
    fn compute_local_support(
        &self,
        shape: &TriangleWide,
        direction: &Vector3Wide,
        _terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let mut a_dot = Vector::<f32>::splat(0.0);
        let mut b_dot = Vector::<f32>::splat(0.0);
        let mut c_dot = Vector::<f32>::splat(0.0);
        Vector3Wide::dot(&shape.a, direction, &mut a_dot);
        Vector3Wide::dot(&shape.b, direction, &mut b_dot);
        Vector3Wide::dot(&shape.c, direction, &mut c_dot);
        let max_val = a_dot.simd_max(b_dot.simd_max(c_dot));
        let pick_a = max_val.simd_eq(a_dot).to_int();
        let pick_c = max_val.simd_eq(c_dot).to_int();
        *support = Vector3Wide::conditional_select(&pick_a, &shape.a, &shape.b);
        *support = Vector3Wide::conditional_select(&pick_c, &shape.c, support);
    }
}
