use glam::{Quat, Vec3};
use std::simd::prelude::*;
use std::simd::StdFloat;

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::symmetric3x3::Symmetric3x3;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use crate::physics::collision_detection::support_finder::ISupportFinder as DepthRefinerSupportFinder;
use crate::physics::body_properties::{BodyInertia, RigidPose, RigidPoseWide};
use super::mesh_inertia_helper::{MeshInertiaHelper, ITriangleSource};
use super::shape::{IShape, IConvexShape, IDisposableShape, IShapeWide, ISupportFinder};
use super::ray::RayWide;

/// Bounding plane of a convex hull face.
pub struct HullBoundingPlanes {
    /// Normal of the bounding plane.
    pub normal: Vector3Wide,
    /// Offset from the origin to a point on the plane along the normal.
    pub offset: Vector<f32>,
}

/// Index of a vertex in a convex hull.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct HullVertexIndex {
    /// Index of the SIMD bundle containing this vertex.
    pub bundle_index: u16,
    /// Lane index within the bundle.
    pub inner_index: u16,
}

impl std::fmt::Display for HullVertexIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {})", self.bundle_index, self.inner_index)
    }
}

/// Collision shape representing a convex hull.
pub struct ConvexHull {
    /// Bundled points of the convex hull.
    pub points: Buffer<Vector3Wide>,
    /// Bundled bounding planes associated with the convex hull's faces.
    pub bounding_planes: Buffer<HullBoundingPlanes>,
    /// Combined set of vertices used by each face. Use face_to_vertex_indices_start to index
    /// into this for a particular face. Indices stored in counterclockwise winding in right
    /// handed space, clockwise in left handed space.
    pub face_vertex_indices: Buffer<HullVertexIndex>,
    /// Start indices of faces in the face_vertex_indices.
    pub face_to_vertex_indices_start: Buffer<i32>,
}

impl ConvexHull {
    /// Type id of convex hull shapes.
    pub const ID: i32 = 5;

    /// Gets the vertex indices for a specific face.
    #[inline(always)]
    pub fn get_vertex_indices_for_face(
        &self,
        face_index: usize,
        start: &mut usize,
        count: &mut usize,
    ) {
        *start = self.face_to_vertex_indices_start[face_index] as usize;
        let next_face_index = face_index + 1;
        let end = if next_face_index as i32 == self.face_to_vertex_indices_start.len() {
            self.face_vertex_indices.len() as usize
        } else {
            self.face_to_vertex_indices_start[next_face_index] as usize
        };
        *count = end - *start;
    }

    /// Gets a point from the hull by its vertex index.
    #[inline(always)]
    pub fn get_point_by_vertex_index(&self, point_index: &HullVertexIndex, point: &mut Vec3) {
        Vector3Wide::read_slot(
            &self.points[point_index.bundle_index as usize],
            point_index.inner_index as usize,
            point,
        );
    }

    /// Gets a point from the hull by linear index.
    #[inline(always)]
    pub fn get_point(&self, point_index: usize, point: &mut Vec3) {
        let mut bundle_index = 0usize;
        let mut inner_index = 0usize;
        BundleIndexing::get_bundle_indices(point_index, &mut bundle_index, &mut inner_index);
        Vector3Wide::read_slot(&self.points[bundle_index], inner_index, point);
    }

    /// Computes bounds internally using a wide orientation.
    fn compute_bounds_internal(
        &self,
        orientation_wide: &QuaternionWide,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        let mut orientation_matrix = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_wide, &mut orientation_matrix);
        let mut min_wide = Vector3Wide::broadcast(Vec3::splat(f32::MAX));
        let mut max_wide = Vector3Wide::broadcast(Vec3::splat(f32::MIN));
        for i in 0..self.points.len() as usize {
            let mut p = Vector3Wide::default();
            Matrix3x3Wide::transform_without_overlap(&self.points[i], &orientation_matrix, &mut p);
            min_wide = Vector3Wide::min_new(&min_wide, &p);
            max_wide = Vector3Wide::max_new(&max_wide, &p);
        }
        Vector3Wide::read_first(&min_wide, min);
        Vector3Wide::read_first(&max_wide, max);
        let lanes = Vector::<f32>::LEN;
        for i in 1..lanes {
            let min_candidate = Vec3::new(min_wide.x[i], min_wide.y[i], min_wide.z[i]);
            let max_candidate = Vec3::new(max_wide.x[i], max_wide.y[i], max_wide.z[i]);
            *min = min.min(min_candidate);
            *max = max.max(max_candidate);
        }
    }
}

/// Triangle source for iterating over all triangles of a convex hull's surface.
/// Performs fan triangulation of each face.
pub struct ConvexHullTriangleSource<'a> {
    hull: &'a ConvexHull,
    face_index: usize,
    subtriangle_index: usize,
}

impl<'a> ConvexHullTriangleSource<'a> {
    pub fn new(hull: &'a ConvexHull) -> Self {
        Self {
            hull,
            face_index: 0,
            subtriangle_index: 2,
        }
    }
}

impl<'a> ITriangleSource for ConvexHullTriangleSource<'a> {
    fn get_next_triangle(&mut self) -> Option<(Vec3, Vec3, Vec3)> {
        if (self.face_index as i32) < self.hull.face_to_vertex_indices_start.len() {
            let mut start = 0usize;
            let mut count = 0usize;
            self.hull
                .get_vertex_indices_for_face(self.face_index, &mut start, &mut count);

            let mut a = Vec3::ZERO;
            let mut b = Vec3::ZERO;
            let mut c = Vec3::ZERO;

            self.hull
                .get_point_by_vertex_index(&self.hull.face_vertex_indices[start], &mut a);
            // Note: b and c are swapped compared to face winding.
            // ConvexHull uses clockwise winding but MeshInertiaHelper expects counterclockwise
            // externally visible triangles in right handed coordinates.
            self.hull.get_point_by_vertex_index(
                &self.hull.face_vertex_indices[start + self.subtriangle_index - 1],
                &mut c,
            );
            self.hull.get_point_by_vertex_index(
                &self.hull.face_vertex_indices[start + self.subtriangle_index],
                &mut b,
            );

            self.subtriangle_index += 1;
            if self.subtriangle_index == count {
                self.subtriangle_index = 2;
                self.face_index += 1;
            }

            Some((a, b, c))
        } else {
            None
        }
    }
}

impl IShape for ConvexHull {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IConvexShape for ConvexHull {
    fn compute_angular_expansion_data(
        &self,
        maximum_radius: &mut f32,
        maximum_angular_expansion: &mut f32,
    ) {
        let mut maximum_radius_squared_wide = Vector::<f32>::splat(0.0);
        let mut minimum_radius_squared_wide = Vector::<f32>::splat(f32::MAX);
        for i in 0..self.points.len() as usize {
            let mut candidate = Vector::<f32>::splat(0.0);
            Vector3Wide::length_squared_to(&self.points[i], &mut candidate);
            maximum_radius_squared_wide = maximum_radius_squared_wide.simd_max(candidate);
            minimum_radius_squared_wide = minimum_radius_squared_wide.simd_min(candidate);
        }
        let minimum_radius_wide = minimum_radius_squared_wide.sqrt();
        let maximum_radius_wide = maximum_radius_squared_wide.sqrt();
        *maximum_radius = maximum_radius_wide[0];
        let mut minimum_radius = minimum_radius_wide[0];
        let lanes = Vector::<f32>::LEN;
        for i in 1..lanes {
            let max_candidate = maximum_radius_wide[i];
            let min_candidate = minimum_radius_wide[i];
            if max_candidate > *maximum_radius {
                *maximum_radius = max_candidate;
            }
            if min_candidate < minimum_radius {
                minimum_radius = min_candidate;
            }
        }
        *maximum_angular_expansion = *maximum_radius - minimum_radius;
    }

    #[inline(always)]
    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut orientation_wide = QuaternionWide::default();
        QuaternionWide::broadcast(orientation, &mut orientation_wide);
        self.compute_bounds_internal(&orientation_wide, min, max);
    }

    fn ray_test(
        &self,
        pose: &RigidPose,
        origin: Vec3,
        direction: Vec3,
        t: &mut f32,
        normal: &mut Vec3,
    ) -> bool {
        debug_assert!(
            self.face_to_vertex_indices_start.len() > 2,
            "Convex hull appears to be degenerate; convex hull must have volume or ray tests will fail."
        );
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);
        let shape_to_ray = origin - pose.position;
        let mut local_origin = Vec3::ZERO;
        Matrix3x3::transform_transpose(&shape_to_ray, &orientation, &mut local_origin);
        let mut local_direction = Vec3::ZERO;
        Matrix3x3::transform_transpose(&direction, &orientation, &mut local_direction);
        let local_origin_bundle = Vector3Wide::broadcast(local_origin);
        let local_direction_bundle = Vector3Wide::broadcast(local_direction);

        let index_offsets =
            Vector::<i32>::from_array(std::array::from_fn(|i| i as i32));
        let mut latest_entry_wide = Vector::<f32>::splat(-f32::MAX);
        let mut earliest_exit_wide = Vector::<f32>::splat(f32::MAX);
        let mut latest_entry_index_bundle = Vector::<i32>::splat(0);
        let epsilon = Vector::<f32>::splat(1e-14);
        let min_value = Vector::<f32>::splat(f32::MIN);
        let zero = Vector::<f32>::splat(0.0);

        for i in 0..self.bounding_planes.len() as usize {
            let bounding_plane = &self.bounding_planes[i];
            let candidate_indices =
                Vector::<i32>::splat((i << BundleIndexing::vector_shift()) as i32) + index_offsets;
            let mut normal_dot_origin = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&local_origin_bundle, &bounding_plane.normal, &mut normal_dot_origin);
            let numerator = bounding_plane.offset - normal_dot_origin;
            let mut denominator = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(
                &local_direction_bundle,
                &bounding_plane.normal,
                &mut denominator,
            );
            // Clamp near-zero denominators
            let neg_eps = -epsilon;
            let abs_denom = denominator.abs();
            let denom_too_small = abs_denom.simd_lt(epsilon);
            let denom_negative = denominator.simd_lt(zero);
            denominator = denom_too_small.select(denom_negative.select(neg_eps, epsilon), denominator);

            let plane_t = numerator / denominator;
            let exit_candidate = denominator.simd_gt(zero);
            let lane_exists = bounding_plane.offset.simd_gt(min_value);
            earliest_exit_wide = (lane_exists & exit_candidate)
                .select(plane_t.simd_min(earliest_exit_wide), earliest_exit_wide);
            let entry_candidate = plane_t.simd_gt(latest_entry_wide)
                & (lane_exists & !exit_candidate);
            latest_entry_wide = entry_candidate.select(plane_t, latest_entry_wide);
            latest_entry_index_bundle =
                entry_candidate.to_int().simd_eq(Vector::<i32>::splat(-1))
                    .select(candidate_indices, latest_entry_index_bundle);
        }

        let mut latest_entry_t = latest_entry_wide[0];
        let mut earliest_exit_t = earliest_exit_wide[0];
        let mut latest_entry_index = latest_entry_index_bundle[0];
        let lanes = Vector::<f32>::LEN;
        for i in 1..lanes {
            let entry_candidate = latest_entry_wide[i];
            let exit_candidate = earliest_exit_wide[i];
            if entry_candidate > latest_entry_t {
                latest_entry_t = entry_candidate;
                latest_entry_index = latest_entry_index_bundle[i];
            }
            if exit_candidate < earliest_exit_t {
                earliest_exit_t = exit_candidate;
            }
        }
        if earliest_exit_t < 0.0 || latest_entry_t > earliest_exit_t {
            *t = 0.0;
            *normal = Vec3::ZERO;
            return false;
        } else {
            *t = if latest_entry_t < 0.0 {
                0.0
            } else {
                latest_entry_t
            };
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(
                latest_entry_index as usize,
                &mut bundle_index,
                &mut inner_index,
            );
            Vector3Wide::read_slot(
                &self.bounding_planes[bundle_index].normal,
                inner_index,
                normal,
            );
            let n = *normal;
            Matrix3x3::transform(&n, &orientation, normal);
            return true;
        }
    }

    fn compute_inertia(&self, mass: f32) -> BodyInertia {
        let mut triangle_source = ConvexHullTriangleSource::new(self);
        let (_volume, inertia_tensor) =
            MeshInertiaHelper::compute_closed_inertia(&mut triangle_source, mass);
        debug_assert!(
            self.face_to_vertex_indices_start.len() > 2
                && _volume > 0.0
                && !inertia_tensor.xx.is_nan()
                && !inertia_tensor.yx.is_nan()
                && !inertia_tensor.yy.is_nan()
                && !inertia_tensor.zx.is_nan()
                && !inertia_tensor.zy.is_nan()
                && !inertia_tensor.zz.is_nan(),
            "Convex hull must have volume."
        );
        let mut inverse_inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&inertia_tensor, &mut inverse_inertia);
        let mut result = BodyInertia::default();
        result.inverse_mass = 1.0 / mass;
        result.inverse_inertia_tensor = inverse_inertia;
        result
    }
}

impl IDisposableShape for ConvexHull {
    fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.points);
        pool.return_buffer(&mut self.bounding_planes);
        pool.return_buffer(&mut self.face_vertex_indices);
        pool.return_buffer(&mut self.face_to_vertex_indices_start);
    }
}

/// Wide representation of a convex hull.
/// Unlike other shapes, single convex hulls are internally vectorized.
/// The "wide" variant is simply a collection of convex hull instances.
pub struct ConvexHullWide {
    pub hulls: Buffer<ConvexHull>,
}

impl ConvexHullWide {
    /// Estimates an epsilon scale for the convex hull by sampling the first point of each hull.
    #[inline(always)]
    pub fn estimate_epsilon_scale(
        &self,
        terminated_lanes: &Vector<i32>,
        epsilon_scale: &mut Vector<f32>,
    ) {
        let mut bundle = Vector3Wide::default();
        let lanes = Vector::<f32>::LEN;
        for i in 0..lanes {
            if terminated_lanes.as_array()[i] < 0 {
                continue;
            }
            debug_assert!((self.hulls.len() as usize) > i);
            Vector3Wide::copy_slot(&self.hulls[i].points[0], 0, &mut bundle, i);
        }
        *epsilon_scale = (bundle.x.abs() + bundle.y.abs() + bundle.z.abs())
            * Vector::<f32>::splat(1.0 / 3.0);
    }
}

impl IShapeWide<ConvexHull> for ConvexHullWide {
    fn allow_offset_memory_access(&self) -> bool {
        false
    }

    fn internal_allocation_size(&self) -> usize {
        Vector::<f32>::LEN * std::mem::size_of::<ConvexHull>()
    }

    fn initialize(&mut self, memory: &Buffer<u8>) {
        debug_assert!(memory.len() as usize == self.internal_allocation_size());
        self.hulls = memory.cast::<ConvexHull>();
    }

    fn broadcast(&mut self, shape: &ConvexHull) {
        for i in 0..self.hulls.len() as usize {
            // Note: This copies the buffer handles, not the actual data.
            // The ConvexHull struct is lightweight (just buffer handles).
            unsafe {
                std::ptr::copy_nonoverlapping(
                    shape as *const ConvexHull,
                    &mut self.hulls[i] as *mut ConvexHull,
                    1,
                );
            }
        }
    }

    fn write_first(&mut self, source: &ConvexHull) {
        unsafe {
            std::ptr::copy_nonoverlapping(
                source as *const ConvexHull,
                &mut self.hulls[0] as *mut ConvexHull,
                1,
            );
        }
    }

    fn write_slot(&mut self, index: usize, source: &ConvexHull) {
        unsafe {
            std::ptr::copy_nonoverlapping(
                source as *const ConvexHull,
                &mut self.hulls[index] as *mut ConvexHull,
                1,
            );
        }
    }

    fn get_bounds(
        &self,
        orientations: &mut QuaternionWide,
        count_in_bundle: i32,
        maximum_radius: &mut Vector<f32>,
        maximum_angular_expansion: &mut Vector<f32>,
        min: &mut Vector3Wide,
        max: &mut Vector3Wide,
    ) {
        for i in 0..(count_in_bundle as usize) {
            let min_wide_init = Vector3Wide::broadcast(Vec3::splat(f32::MAX));
            let max_wide_init = Vector3Wide::broadcast(Vec3::splat(f32::MIN));
            let mut min_wide = min_wide_init;
            let mut max_wide = max_wide_init;
            let mut orientation_wide = QuaternionWide::default();
            QuaternionWide::rebroadcast(orientations, i, &mut orientation_wide);
            let mut orientation_matrix = Matrix3x3Wide::default();
            Matrix3x3Wide::create_from_quaternion(&orientation_wide, &mut orientation_matrix);
            let mut maximum_radius_squared_wide = Vector::<f32>::splat(0.0);
            let hull = &self.hulls[i];
            for j in 0..hull.points.len() as usize {
                let local_point = &hull.points[j];
                let mut p = Vector3Wide::default();
                Matrix3x3Wide::transform_without_overlap(local_point, &orientation_matrix, &mut p);
                let mut length_squared = Vector::<f32>::splat(0.0);
                Vector3Wide::length_squared_to(local_point, &mut length_squared);
                maximum_radius_squared_wide =
                    maximum_radius_squared_wide.simd_max(length_squared);
                min_wide = Vector3Wide::min_new(&min_wide, &p);
                max_wide = Vector3Wide::max_new(&max_wide, &p);
            }
            let mut min_narrow = Vec3::ZERO;
            let mut max_narrow = Vec3::ZERO;
            Vector3Wide::read_first(&min_wide, &mut min_narrow);
            Vector3Wide::read_first(&max_wide, &mut max_narrow);
            let mut maximum_radius_squared = maximum_radius_squared_wide[0];
            let lanes = Vector::<f32>::LEN;
            for j in 1..lanes {
                let min_candidate = Vec3::new(min_wide.x[j], min_wide.y[j], min_wide.z[j]);
                let max_candidate = Vec3::new(max_wide.x[j], max_wide.y[j], max_wide.z[j]);
                min_narrow = min_narrow.min(min_candidate);
                max_narrow = max_narrow.max(max_candidate);
                let max_radius_candidate = maximum_radius_squared_wide[j];
                if max_radius_candidate > maximum_radius_squared {
                    maximum_radius_squared = max_radius_candidate;
                }
            }
            maximum_radius.as_mut_array()[i] = maximum_radius_squared;
            Vector3Wide::write_slot(min_narrow, i, min);
            Vector3Wide::write_slot(max_narrow, i, max);
        }
        *maximum_radius = StdFloat::sqrt(*maximum_radius);
        *maximum_angular_expansion = *maximum_radius;
    }

    fn minimum_wide_ray_count() -> i32 {
        i32::MAX // 'Wide' ray tests just fall through to scalar tests anyway.
    }

    fn ray_test(
        &self,
        poses: &mut RigidPoseWide,
        ray_wide: &mut RayWide,
        intersected: &mut Vector<i32>,
        t: &mut Vector<f32>,
        normal: &mut Vector3Wide,
    ) {
        debug_assert!(self.hulls.len() > 0 && (self.hulls.len() as usize) <= Vector::<f32>::LEN);
        for i in 0..self.hulls.len() as usize {
            let offset_pose =
                unsafe { GatherScatter::get_offset_instance(poses, i) };
            let mut pose = RigidPose::default();
            RigidPoseWide::read_first(offset_pose, &mut pose);
            let offset_ray = unsafe { GatherScatter::get_offset_instance(ray_wide, i) };
            let mut origin = Vec3::ZERO;
            let mut direction = Vec3::ZERO;
            Vector3Wide::read_first(&offset_ray.origin, &mut origin);
            Vector3Wide::read_first(&offset_ray.direction, &mut direction);
            let mut t_narrow = 0.0f32;
            let mut normal_narrow = Vec3::ZERO;
            let hit = self.hulls[i].ray_test(
                &pose,
                origin,
                direction,
                &mut t_narrow,
                &mut normal_narrow,
            );
            intersected.as_mut_array()[i] = if hit { -1 } else { 0 };
            t.as_mut_array()[i] = t_narrow;
            Vector3Wide::write_slot(normal_narrow, i, normal);
        }
    }
}

/// Support finder for convex hulls.
pub struct ConvexHullSupportFinder;

impl ISupportFinder<ConvexHull, ConvexHullWide> for ConvexHullSupportFinder {
    fn has_margin(&self) -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(&self, _shape: &ConvexHullWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::default();
    }

    fn compute_local_support(
        &self,
        shape: &ConvexHullWide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let index_offsets =
            Vector::<i32>::from_array(std::array::from_fn(|i| i as i32));
        let lanes = Vector::<f32>::LEN;
        for slot_index in 0..lanes {
            if terminated_lanes[slot_index] < 0 {
                continue;
            }
            let hull = &shape.hulls[slot_index];
            debug_assert!(hull.points.allocated(), "If the lane isn't terminated, then the hull should actually exist.");
            let mut slot_direction = QuaternionWide::default();
            // Rebroadcast the direction for this slot
            // (access lane slot_index and broadcast across all lanes)
            let dir_as_v3w = direction;
            let mut rebroadcast_dir = Vector3Wide {
                x: Vector::<f32>::splat(dir_as_v3w.x[slot_index]),
                y: Vector::<f32>::splat(dir_as_v3w.y[slot_index]),
                z: Vector::<f32>::splat(dir_as_v3w.z[slot_index]),
            };

            let mut best_indices = index_offsets;
            let mut dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&rebroadcast_dir, &hull.points[0], &mut dot);
            for j in 1..hull.points.len() as usize {
                let candidate = &hull.points[j];
                let mut dot_candidate = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&rebroadcast_dir, candidate, &mut dot_candidate);
                let use_candidate = dot_candidate.simd_gt(dot);
                best_indices = use_candidate.to_int().simd_eq(Vector::<i32>::splat(-1)).select(
                    index_offsets + Vector::<i32>::splat((j << BundleIndexing::vector_shift()) as i32),
                    best_indices,
                );
                dot = use_candidate.select(dot_candidate, dot);
            }
            // Horizontal reduction
            let mut best_slot_index = 0usize;
            let mut best_slot_dot = dot[0];
            for j in 1..lanes {
                let candidate_dot = dot[j];
                if candidate_dot > best_slot_dot {
                    best_slot_dot = candidate_dot;
                    best_slot_index = j;
                }
            }
            let support_index = best_indices[best_slot_index] as usize;
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(support_index, &mut bundle_index, &mut inner_index);
            Vector3Wide::copy_slot(
                &hull.points[bundle_index],
                inner_index,
                support,
                slot_index,
            );
        }
    }

    #[inline(always)]
    fn compute_support(
        &self,
        shape: &ConvexHullWide,
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
}

impl DepthRefinerSupportFinder<ConvexHullWide> for ConvexHullSupportFinder {
    fn has_margin() -> bool {
        false
    }

    #[inline(always)]
    fn get_margin(_shape: &ConvexHullWide, margin: &mut Vector<f32>) {
        *margin = Vector::<f32>::default();
    }

    fn compute_local_support(
        shape: &ConvexHullWide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    ) {
        let index_offsets =
            Vector::<i32>::from_array(std::array::from_fn(|i| i as i32));
        let lanes = Vector::<f32>::LEN;
        for slot_index in 0..lanes {
            if terminated_lanes[slot_index] < 0 {
                continue;
            }
            let hull = &shape.hulls[slot_index];
            debug_assert!(hull.points.allocated(), "If the lane isn't terminated, then the hull should actually exist.");
            let rebroadcast_dir = Vector3Wide {
                x: Vector::<f32>::splat(direction.x[slot_index]),
                y: Vector::<f32>::splat(direction.y[slot_index]),
                z: Vector::<f32>::splat(direction.z[slot_index]),
            };

            let mut best_indices = index_offsets;
            let mut dot = Vector::<f32>::splat(0.0);
            Vector3Wide::dot(&rebroadcast_dir, &hull.points[0], &mut dot);
            for j in 1..hull.points.len() as usize {
                let candidate = &hull.points[j];
                let mut dot_candidate = Vector::<f32>::splat(0.0);
                Vector3Wide::dot(&rebroadcast_dir, candidate, &mut dot_candidate);
                let use_candidate = dot_candidate.simd_gt(dot);
                best_indices = use_candidate.to_int().simd_eq(Vector::<i32>::splat(-1)).select(
                    index_offsets + Vector::<i32>::splat((j << BundleIndexing::vector_shift()) as i32),
                    best_indices,
                );
                dot = use_candidate.select(dot_candidate, dot);
            }
            let mut best_slot_index = 0usize;
            let mut best_slot_dot = dot[0];
            for j in 1..lanes {
                let candidate_dot = dot[j];
                if candidate_dot > best_slot_dot {
                    best_slot_dot = candidate_dot;
                    best_slot_index = j;
                }
            }
            let support_index = best_indices[best_slot_index] as usize;
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(support_index, &mut bundle_index, &mut inner_index);
            Vector3Wide::copy_slot(
                &hull.points[bundle_index],
                inner_index,
                support,
                slot_index,
            );
        }
    }

    #[inline(always)]
    fn compute_support(
        shape: &ConvexHullWide,
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
        <Self as DepthRefinerSupportFinder<ConvexHullWide>>::compute_local_support(
            shape, &local_direction, terminated_lanes, &mut local_support,
        );
        Matrix3x3Wide::transform_without_overlap(&local_support, orientation, support);
    }
}
