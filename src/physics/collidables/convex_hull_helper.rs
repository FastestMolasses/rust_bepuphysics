use glam::Vec3;
use std::simd::prelude::*;

use crate::utilities::bundle_indexing::BundleIndexing;
use crate::utilities::collections::quick_dictionary::QuickDictionary;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::vector::{Vector, VECTOR_WIDTH};
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

use crate::physics::helpers::Helpers;

use super::convex_hull::{ConvexHull, HullBoundingPlanes};
use super::mesh_inertia_helper::MeshInertiaHelper;

/// Stores references to the points composing one of a convex hull's faces.
pub struct HullFace {
    pub original_vertex_mapping: Buffer<i32>,
    pub vertex_indices: Buffer<i32>,
}

impl HullFace {
    /// Gets the number of vertices in the face.
    #[inline(always)]
    pub fn vertex_count(&self) -> i32 {
        self.vertex_indices.len()
    }

    /// Gets the index of the vertex associated with the given face vertex index in the source point set.
    #[inline(always)]
    pub fn get(&self, index: i32) -> i32 {
        self.original_vertex_mapping[self.vertex_indices[index] as usize]
    }
}

/// Raw data representing a convex hull.
/// This is not yet transformed into a runtime format. It requires additional processing
/// to be used in a ConvexHull shape; see ConvexHullHelper::create_shape.
pub struct HullData {
    /// Mapping of points on the convex hull back to the original point set.
    pub original_vertex_mapping: Buffer<i32>,
    /// List of indices composing the faces of the hull. Individual faces indexed by the FaceStartIndices.
    pub face_vertex_indices: Buffer<i32>,
    /// Starting index in the FaceVertexIndices for each face.
    pub face_start_indices: Buffer<i32>,
}

impl HullData {
    #[inline(always)]
    pub fn get_face(&self, face_index: i32) -> HullFace {
        let next_face_index = face_index + 1;
        let start = self.face_start_indices[face_index as usize];
        let end = if next_face_index == self.face_start_indices.len() {
            self.face_vertex_indices.len() as i32
        } else {
            self.face_start_indices[next_face_index as usize]
        };
        let vertex_indices = self.face_vertex_indices.slice_offset(start, end - start);
        HullFace {
            original_vertex_mapping: self.original_vertex_mapping,
            vertex_indices,
        }
    }

    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.original_vertex_mapping);
        if self.face_vertex_indices.allocated() {
            pool.return_buffer(&mut self.face_vertex_indices);
        }
        if self.face_start_indices.allocated() {
            pool.return_buffer(&mut self.face_start_indices);
        }
    }
}

/// Edge endpoints used as keys in the edge-face dictionary.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct EdgeEndpoints {
    pub a: i32,
    pub b: i32,
}

impl EdgeEndpoints {
    #[inline(always)]
    pub fn equals(a: &EdgeEndpoints, b: &EdgeEndpoints) -> bool {
        // Same edge in either direction.
        (a.a == b.a && a.b == b.b) || (a.a == b.b && a.b == b.a)
    }

    #[inline(always)]
    pub fn hash(item: &EdgeEndpoints) -> i32 {
        item.a ^ item.b
    }
}

/// Equality comparer for EdgeEndpoints in QuickDictionary.
#[derive(Clone, Copy, Default)]
pub struct EdgeEndpointsComparer;

impl crate::utilities::collections::equaility_comparer_ref::RefEqualityComparer<EdgeEndpoints>
    for EdgeEndpointsComparer
{
    #[inline(always)]
    fn hash(&self, item: &EdgeEndpoints) -> i32 {
        EdgeEndpoints::hash(item)
    }

    #[inline(always)]
    fn equals(&self, a: &EdgeEndpoints, b: &EdgeEndpoints) -> bool {
        EdgeEndpoints::equals(a, b)
    }
}

#[derive(Clone, Copy)]
struct EarlyFace {
    vertex_indices: QuickList<i32>,
    normal: Vec3,
}

#[derive(Clone, Copy)]
struct EdgeToTest {
    endpoints: EdgeEndpoints,
    face_normal: Vec3,
}

/// Helper methods to create and process convex hulls from point clouds.
pub struct ConvexHullHelper;

impl ConvexHullHelper {
    fn find_extreme_face(
        basis_x: &Vector3Wide,
        basis_y: &Vector3Wide,
        basis_origin: &Vector3Wide,
        source_edge_endpoints: &EdgeEndpoints,
        point_bundles: &mut Buffer<Vector3Wide>,
        index_offsets: &Vector<i32>,
        allow_vertices: &Buffer<i32>,
        point_count: usize,
        projected_on_x: &mut Buffer<Vector<f32>>,
        projected_on_y: &mut Buffer<Vector<f32>>,
        plane_epsilon: &Vector<f32>,
        vertex_indices: &mut QuickList<i32>,
        face_normal: &mut Vec3,
    ) {
        debug_assert!(
            projected_on_x.len() >= point_bundles.len()
                && projected_on_y.len() >= point_bundles.len()
                && vertex_indices.count == 0
                && vertex_indices.span.len() >= point_bundles.len() as i32 * VECTOR_WIDTH as i32
        );

        // Find the candidate-basisOrigin which has the smallest angle with basisY when projected onto the plane.
        let mut to_candidate = Vector3Wide::default();
        Vector3Wide::subtract(&point_bundles[0], basis_origin, &mut to_candidate);
        let x = unsafe { projected_on_x.get_mut(0) };
        let y = unsafe { projected_on_y.get_mut(0) };
        Vector3Wide::dot(basis_x, &to_candidate, x);
        *x = (*x).simd_max(Vector::<f32>::splat(0.0));
        Vector3Wide::dot(basis_y, &to_candidate, y);
        let mut best_y = *y;
        let mut best_x = *x;

        let edge_index_a = Vector::<i32>::splat(source_edge_endpoints.a);
        let edge_index_b = Vector::<i32>::splat(source_edge_endpoints.b);

        let allow_vertex_bundles: &[Vector<i32>] = unsafe {
            std::slice::from_raw_parts(
                allow_vertices.as_ptr() as *const Vector<i32>,
                point_bundles.len() as usize,
            )
        };

        let ignore_slot = (!allow_vertex_bundles[0])
            | (best_x.simd_le(*plane_epsilon).to_int() & best_y.simd_le(*plane_epsilon).to_int())
            | (index_offsets.simd_eq(edge_index_a).to_int()
                | index_offsets.simd_eq(edge_index_b).to_int());
        best_x = Mask::from_int(ignore_slot).select(Vector::<f32>::splat(1.0), best_x);
        best_y = Mask::from_int(ignore_slot).select(Vector::<f32>::splat(f32::MIN), best_y);
        let mut best_indices = *index_offsets;

        for i in 1..point_bundles.len() as usize {
            Vector3Wide::subtract(&point_bundles[i], basis_origin, &mut to_candidate);
            let x = unsafe { projected_on_x.get_mut(i as i32) };
            let y = unsafe { projected_on_y.get_mut(i as i32) };
            Vector3Wide::dot(basis_x, &to_candidate, x);
            *x = (*x).simd_max(Vector::<f32>::splat(0.0));
            Vector3Wide::dot(basis_y, &to_candidate, y);

            let candidate_indices =
                *index_offsets + Vector::<i32>::splat((i << BundleIndexing::vector_shift()) as i32);
            let ignore_slot = (!allow_vertex_bundles[i])
                | ((*x).simd_le(*plane_epsilon).to_int() & (*y).simd_le(*plane_epsilon).to_int())
                | (candidate_indices.simd_eq(edge_index_a).to_int()
                    | candidate_indices.simd_eq(edge_index_b).to_int());
            let use_candidate = (*y * best_x).simd_gt(best_y * *x).to_int() & !ignore_slot;

            let mask = Mask::from_int(use_candidate);
            best_y = mask.select(*y, best_y);
            best_x = mask.select(*x, best_x);
            best_indices = mask.cast().select(candidate_indices, best_indices);
        }

        let mut best_y_narrow = best_y[0];
        let mut best_x_narrow = best_x[0];
        let mut best_index_narrow = best_indices[0];
        for i in 1..VECTOR_WIDTH {
            let candidate_numerator = best_y[i];
            let candidate_denominator = best_x[i];
            if candidate_numerator * best_x_narrow > best_y_narrow * candidate_denominator {
                best_y_narrow = candidate_numerator;
                best_x_narrow = candidate_denominator;
                best_index_narrow = best_indices[i];
            }
        }

        // Compute projected plane normal.
        let candidate_normal_direction = glam::Vec2::new(-best_y_narrow, best_x_narrow);
        let length = candidate_normal_direction.length();
        let projected_plane_normal_narrow = if length.is_finite() {
            candidate_normal_direction / length
        } else {
            glam::Vec2::new(1.0, 0.0)
        };
        let mut projected_plane_normal = Vector2Wide::default();
        Vector2Wide::broadcast(&projected_plane_normal_narrow, &mut projected_plane_normal);

        let mut basis_x_narrow = Vec3::ZERO;
        Vector3Wide::read_first(basis_x, &mut basis_x_narrow);
        let mut basis_y_narrow = Vec3::ZERO;
        Vector3Wide::read_first(basis_y, &mut basis_y_narrow);
        *face_normal = basis_x_narrow * projected_plane_normal_narrow.x
            + basis_y_narrow * projected_plane_normal_narrow.y;

        let _face_normal_wide = Vector3Wide::broadcast(*face_normal);
        let negated_plane_epsilon = -*plane_epsilon;
        for i in 0..point_bundles.len() as usize {
            let dot = projected_on_x[i] * projected_plane_normal.x
                + projected_on_y[i] * projected_plane_normal.y;
            let coplanar = dot.simd_gt(negated_plane_epsilon).to_int();
            if coplanar.simd_lt(Vector::<i32>::splat(0)).any() {
                let bundle_base_index = i << BundleIndexing::vector_shift();
                let mut local_index_maximum = point_count - bundle_base_index;
                if local_index_maximum > VECTOR_WIDTH {
                    local_index_maximum = VECTOR_WIDTH;
                }
                for j in 0..local_index_maximum {
                    let vertex_index = bundle_base_index + j;
                    if coplanar[j] < 0 && allow_vertices[vertex_index as i32] != 0 {
                        *vertex_indices.allocate_unsafely() = vertex_index as i32;
                    }
                }
            }
        }
    }

    /// Finds the next index in the 2D hull of a face using gift wrapping.
    fn find_next_index_for_face_hull(
        start: glam::Vec2,
        previous_edge_direction: glam::Vec2,
        plane_epsilon: f32,
        face_points: &QuickList<glam::Vec2>,
    ) -> i32 {
        let basis_x = glam::Vec2::new(previous_edge_direction.y, -previous_edge_direction.x);
        let basis_y = -previous_edge_direction;
        let mut best_x = 1.0f32;
        let mut best_y = f32::MAX;
        let mut best_index: i32 = -1;

        for i in 0..face_points.count {
            let candidate = face_points.span[i as usize];
            let to_candidate = candidate - start;
            let x = (to_candidate.dot(basis_x)).max(0.0);
            let y = to_candidate.dot(basis_y);

            let ignore_slot = x <= plane_epsilon && y >= -plane_epsilon;
            let use_candidate = (y * best_x < best_y * x) && !ignore_slot;
            if use_candidate {
                best_y = y;
                best_x = x;
                best_index = i;
            }
        }

        if best_index == -1 {
            return -1;
        }

        // Find the most distant point on the best edge direction.
        let projected_best_edge_direction = glam::Vec2::new(best_x, best_y);
        let length = projected_best_edge_direction.length();
        let projected_best_edge_direction = if length.is_finite() {
            projected_best_edge_direction / length
        } else {
            glam::Vec2::new(1.0, 0.0)
        };
        let edge_direction =
            basis_x * projected_best_edge_direction.x + basis_y * projected_best_edge_direction.y;
        let face_normal = glam::Vec2::new(-edge_direction.y, edge_direction.x);

        let mut distance = 0.0f32;
        let mut most_distant_index: i32 = -1;
        for i in 0..face_points.count {
            let candidate = face_points.span[i as usize];
            let to_candidate = candidate - start;
            let along_normal = to_candidate.dot(face_normal);
            if along_normal > -plane_epsilon {
                let along_edge = to_candidate.dot(edge_direction);
                if along_edge > distance {
                    distance = along_edge;
                    most_distant_index = i;
                }
            }
        }

        if most_distant_index == -1 {
            best_index
        } else {
            most_distant_index
        }
    }

    fn reduce_face(
        face_vertex_indices: &mut QuickList<i32>,
        face_normal: Vec3,
        points: &[Vec3],
        plane_epsilon: f32,
        face_points: &mut QuickList<glam::Vec2>,
        allow_vertex: &mut Buffer<i32>,
        reduced_indices: &mut QuickList<i32>,
    ) {
        debug_assert!(
            face_points.count == 0
                && reduced_indices.count == 0
                && face_points.span.len() >= face_vertex_indices.count
                && reduced_indices.span.len() >= face_vertex_indices.count
        );

        // Remove disallowed vertices.
        let mut i = face_vertex_indices.count - 1;
        while i >= 0 {
            if allow_vertex[face_vertex_indices.span[i as usize] as usize] == 0 {
                face_vertex_indices.remove_at(i);
            }
            i -= 1;
        }

        if face_vertex_indices.count <= 3 {
            for i in 0..face_vertex_indices.count {
                *reduced_indices.allocate_unsafely() = face_vertex_indices.span[i as usize];
            }
            if face_vertex_indices.count == 3 {
                let a = points[reduced_indices.span[0] as usize];
                let b = points[reduced_indices.span[1] as usize];
                let c = points[reduced_indices.span[2] as usize];
                let ab = b - a;
                let ac = c - a;
                let uncalibrated_normal = ab.cross(ac);
                if uncalibrated_normal.length_squared() < 1e-14 {
                    if ab.length_squared() > 1e-14 {
                        unsafe {
                            *allow_vertex.get_mut(reduced_indices.span[2] as i32) = 0;
                        }
                        reduced_indices.fast_remove_at(2);
                    } else if ac.length_squared() > 1e-14 {
                        unsafe {
                            *allow_vertex.get_mut(reduced_indices.span[1] as i32) = 0;
                        }
                        reduced_indices.fast_remove_at(1);
                    } else {
                        unsafe {
                            *allow_vertex.get_mut(reduced_indices.span[1] as i32) = 0;
                            *allow_vertex.get_mut(reduced_indices.span[2] as i32) = 0;
                        }
                        reduced_indices.count = 1;
                    }
                } else {
                    if face_normal.dot(uncalibrated_normal) < 0.0 {
                        let tmp = reduced_indices.span[0];
                        unsafe {
                            *reduced_indices.span.get_mut(0) = reduced_indices.span[1];
                            *reduced_indices.span.get_mut(1) = tmp;
                        }
                    }
                }
            }
            return;
        }

        let mut basis_x_out = Vec3::ZERO;
        let mut basis_y_out = Vec3::ZERO;
        Helpers::build_orthonormal_basis_scalar(face_normal, &mut basis_x_out, &mut basis_y_out);
        let mut centroid = glam::Vec2::ZERO;
        for i in 0..face_vertex_indices.count {
            let source = points[face_vertex_indices.span[i as usize] as usize];
            let face_point = face_points.allocate_unsafely();
            *face_point = glam::Vec2::new(basis_x_out.dot(source), basis_y_out.dot(source));
            centroid += *face_point;
        }
        centroid /= face_vertex_indices.count as f32;

        let mut greatest_distance_squared = -1.0f32;
        let mut initial_index = 0i32;
        for i in 0..face_vertex_indices.count {
            let face_point = face_points.span[i as usize];
            let distance_squared = (face_point - centroid).length_squared();
            if greatest_distance_squared < distance_squared {
                greatest_distance_squared = distance_squared;
                initial_index = i;
            }
        }

        if greatest_distance_squared < 1e-14 {
            for i in 0..face_vertex_indices.count {
                unsafe {
                    *allow_vertex.get_mut(face_vertex_indices.span[i as usize] as i32) = 0;
                }
            }
            return;
        }

        let greatest_distance = greatest_distance_squared.sqrt();
        let initial_offset_direction =
            (face_points.span[initial_index as usize] - centroid) / greatest_distance;
        let mut previous_edge_direction =
            glam::Vec2::new(initial_offset_direction.y, -initial_offset_direction.x);
        *reduced_indices.allocate_unsafely() = face_vertex_indices.span[initial_index as usize];

        let mut previous_end_index = initial_index;
        for _i in 0..face_points.count {
            let next_index = Self::find_next_index_for_face_hull(
                face_points.span[previous_end_index as usize],
                previous_edge_direction,
                plane_epsilon,
                face_points,
            );
            if next_index == -1
                || reduced_indices.contains(&face_vertex_indices.span[next_index as usize])
            {
                if next_index >= 0 {
                    let target_val = face_vertex_indices.span[next_index as usize];
                    if let Some(cycle_start_index) = reduced_indices.index_of(&target_val) {
                        if cycle_start_index > 0 {
                            // Self-copy: use memmove via raw pointers.
                            let count_to_copy =
                                (reduced_indices.count - cycle_start_index) as usize;
                            unsafe {
                                let src = reduced_indices
                                    .span
                                    .as_ptr()
                                    .add(cycle_start_index as usize);
                                let dst = reduced_indices.span.as_ptr() as *mut i32;
                                std::ptr::copy(src, dst, count_to_copy);
                            }
                            reduced_indices.count -= cycle_start_index;
                        }
                    }
                }
                break;
            }
            *reduced_indices.allocate_unsafely() = face_vertex_indices.span[next_index as usize];
            let prev_point = face_points.span[previous_end_index as usize];
            let next_point = face_points.span[next_index as usize];
            previous_edge_direction = (next_point - prev_point).normalize();
            previous_end_index = next_index;
        }

        // Ignore vertices not on the outer boundary.
        for i in 0..face_vertex_indices.count {
            let index = face_vertex_indices.span[i as usize];
            if !reduced_indices.contains(&index) {
                unsafe {
                    *allow_vertex.get_mut(index as i32) = 0;
                }
            }
        }
    }

    fn add_face(
        faces: &mut QuickList<EarlyFace>,
        pool: &mut BufferPool,
        normal: Vec3,
        vertex_indices: &QuickList<i32>,
    ) {
        let face = faces.allocate(pool);
        let mut new_list = QuickList::with_capacity(vertex_indices.count, pool);
        new_list.add_range_unsafely(&vertex_indices.span, 0, vertex_indices.count);
        *face = EarlyFace {
            normal,
            vertex_indices: new_list,
        };
    }

    fn add_face_edges_to_test_list(
        pool: &mut BufferPool,
        reduced_face_indices: &mut QuickList<i32>,
        edges_to_test: &mut QuickList<EdgeToTest>,
        edge_face_counts: &mut QuickDictionary<EdgeEndpoints, i32, EdgeEndpointsComparer>,
        face_normal: Vec3,
        _new_face_index: i32,
    ) {
        let mut previous_index =
            reduced_face_indices.span[(reduced_face_indices.count - 1) as usize];
        for i in 0..reduced_face_indices.count {
            let mut endpoints = EdgeEndpoints::default();
            endpoints.a = previous_index;
            endpoints.b = reduced_face_indices.span[i as usize];
            previous_index = endpoints.b;

            let mut slot_index = 0i32;
            if !edge_face_counts.find_or_allocate_slot_unsafely(&endpoints, &mut slot_index) {
                let edge = edges_to_test.allocate(pool);
                edge.endpoints = endpoints;
                edge.face_normal = face_normal;
                unsafe {
                    *edge_face_counts.values.get_mut(slot_index) = 1;
                }
            } else {
                unsafe {
                    *edge_face_counts.values.get_mut(slot_index) += 1;
                }
            }
        }
    }

    /// Computes the convex hull of a set of points.
    pub fn compute_hull(points: &[Vec3], pool: &mut BufferPool, hull_data: &mut HullData) {
        if points.is_empty() {
            *hull_data = HullData {
                original_vertex_mapping: Buffer::default(),
                face_vertex_indices: Buffer::default(),
                face_start_indices: Buffer::default(),
            };
            return;
        }
        if points.len() <= 3 {
            hull_data.original_vertex_mapping = pool.take_at_least(points.len() as i32);
            for i in 0..points.len() {
                unsafe {
                    *hull_data.original_vertex_mapping.get_mut(i as i32) = i as i32;
                }
            }
            if points.len() == 3 {
                hull_data.face_start_indices = pool.take_at_least(1);
                hull_data.face_vertex_indices = pool.take_at_least(3);
                unsafe {
                    *hull_data.face_start_indices.get_mut(0) = 0;
                    *hull_data.face_vertex_indices.get_mut(0) = 0;
                    *hull_data.face_vertex_indices.get_mut(1) = 1;
                    *hull_data.face_vertex_indices.get_mut(2) = 2;
                }
            } else {
                hull_data.face_start_indices = Buffer::default();
                hull_data.face_vertex_indices = Buffer::default();
            }
            return;
        }

        let point_bundle_count = BundleIndexing::get_bundle_count(points.len());
        let mut point_bundles: Buffer<Vector3Wide> = pool.take_at_least(point_bundle_count as i32);

        // Create AOSOA version of input data.
        let mut centroid = Vec3::ZERO;
        for i in 0..points.len() {
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(i, &mut bundle_index, &mut inner_index);
            Vector3Wide::write_slot(points[i], inner_index, unsafe {
                point_bundles.get_mut(bundle_index as i32)
            });
            centroid += points[i];
        }
        centroid /= points.len() as f32;

        // Fill trailing slots with centroid.
        let bundle_slots = point_bundles.len() as usize * VECTOR_WIDTH;
        for i in points.len()..bundle_slots {
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(i, &mut bundle_index, &mut inner_index);
            Vector3Wide::write_slot(centroid, inner_index, unsafe {
                point_bundles.get_mut(bundle_index as i32)
            });
        }

        // Find starting point — furthest from centroid.
        let centroid_bundle = Vector3Wide::broadcast(centroid);
        let mut index_offset_bundle = Vector::<i32>::default();
        Helpers::fill_vector_with_lane_indices(&mut index_offset_bundle);
        let mut most_distant_indices_bundle = index_offset_bundle;
        let mut distance_squared_bundle =
            Vector3Wide::distance_squared(&point_bundles[0], &centroid_bundle);

        for i in 1..point_bundles.len() as usize {
            let bundle_indices = Vector::<i32>::splat((i << BundleIndexing::vector_shift()) as i32)
                + index_offset_bundle;
            let distance_squared_candidate =
                Vector3Wide::distance_squared(&point_bundles[i], &centroid_bundle);
            let use_new = distance_squared_candidate.simd_gt(distance_squared_bundle);
            most_distant_indices_bundle = use_new
                .cast()
                .select(bundle_indices, most_distant_indices_bundle);
            distance_squared_bundle = distance_squared_bundle.simd_max(distance_squared_candidate);
        }

        let mut best_distance_squared = distance_squared_bundle[0];
        let mut initial_index = most_distant_indices_bundle[0];
        for i in 1..VECTOR_WIDTH {
            let distance_candidate = distance_squared_bundle[i];
            if distance_candidate > best_distance_squared {
                best_distance_squared = distance_candidate;
                initial_index = most_distant_indices_bundle[i];
            }
        }

        let mut most_distant_bundle_index = 0usize;
        let mut most_distant_inner_index = 0usize;
        BundleIndexing::get_bundle_indices(
            initial_index as usize,
            &mut most_distant_bundle_index,
            &mut most_distant_inner_index,
        );
        let mut initial_vertex = Vec3::ZERO;
        Vector3Wide::read_slot(
            &point_bundles[most_distant_bundle_index],
            most_distant_inner_index,
            &mut initial_vertex,
        );

        let initial_to_centroid = centroid - initial_vertex;
        let initial_distance = initial_to_centroid.length();
        if initial_distance < 1e-7 {
            hull_data.original_vertex_mapping = pool.take_at_least(1);
            unsafe {
                *hull_data.original_vertex_mapping.get_mut(0) = 0;
            }
            hull_data.face_start_indices = Buffer::default();
            hull_data.face_vertex_indices = Buffer::default();
            pool.return_buffer(&mut point_bundles);
            return;
        }

        let initial_dir = initial_to_centroid / initial_distance;
        let initial_basis_x = Vector3Wide::broadcast(initial_dir);
        let mut initial_basis_y = Vector3Wide::default();
        Helpers::find_perpendicular(&initial_basis_x, &mut initial_basis_y);
        let initial_vertex_bundle = Vector3Wide::broadcast(initial_vertex);

        let mut projected_on_x: Buffer<Vector<f32>> = pool.take_at_least(point_bundles.len());
        let mut projected_on_y: Buffer<Vector<f32>> = pool.take_at_least(point_bundles.len());

        let plane_slab_epsilon_narrow = best_distance_squared.sqrt() * 1e-4;
        let normal_coplanarity_epsilon = 1.0 - 1e-6f32;
        let plane_slab_epsilon = Vector::<f32>::splat(plane_slab_epsilon_narrow);
        let mut raw_face_vertex_indices =
            QuickList::<i32>::with_capacity(point_bundles.len() as i32 * VECTOR_WIDTH as i32, pool);

        let mut allow_vertices: Buffer<i32> =
            pool.take_at_least(point_bundle_count as i32 * VECTOR_WIDTH as i32);
        for i in 0..points.len() {
            unsafe {
                *allow_vertices.get_mut(i as i32) = -1i32;
            }
        }
        for i in points.len()..allow_vertices.len() as usize {
            unsafe {
                *allow_vertices.get_mut(i as i32) = 0;
            }
        }

        let initial_source_edge = EdgeEndpoints {
            a: initial_index,
            b: initial_index,
        };
        let mut initial_face_normal = Vec3::ZERO;

        Self::find_extreme_face(
            &initial_basis_x,
            &initial_basis_y,
            &initial_vertex_bundle,
            &initial_source_edge,
            &mut point_bundles,
            &index_offset_bundle,
            &allow_vertices,
            points.len(),
            &mut projected_on_x,
            &mut projected_on_y,
            &plane_slab_epsilon,
            &mut raw_face_vertex_indices,
            &mut initial_face_normal,
        );
        debug_assert!(raw_face_vertex_indices.count >= 2);

        let mut face_points = QuickList::<glam::Vec2>::with_capacity(points.len() as i32, pool);
        let mut reduced_face_indices = QuickList::<i32>::with_capacity(points.len() as i32, pool);

        Self::reduce_face(
            &mut raw_face_vertex_indices,
            initial_face_normal,
            points,
            plane_slab_epsilon_narrow,
            &mut face_points,
            &mut allow_vertices,
            &mut reduced_face_indices,
        );

        let mut faces = QuickList::<EarlyFace>::with_capacity(points.len() as i32, pool);
        let mut edges_to_test = QuickList::<EdgeToTest>::with_capacity(points.len() as i32, pool);
        let mut edge_face_counts =
            QuickDictionary::<EdgeEndpoints, i32, EdgeEndpointsComparer>::with_capacity(
                points.len() as i32,
                3,
                pool,
                EdgeEndpointsComparer,
            );

        if reduced_face_indices.count >= 3 {
            for i in 0..reduced_face_indices.count {
                let prev_i = if i == 0 {
                    reduced_face_indices.count - 1
                } else {
                    i - 1
                };
                let edge = edges_to_test.allocate(pool);
                edge.endpoints.a = reduced_face_indices.span[prev_i as usize];
                edge.endpoints.b = reduced_face_indices.span[i as usize];
                edge.face_normal = initial_face_normal;
            }
            Self::add_face(&mut faces, pool, initial_face_normal, &reduced_face_indices);
        } else {
            debug_assert!(
                reduced_face_indices.count == 2,
                "The point set size was verified to be at least 4 earlier."
            );
            let edge = edges_to_test.allocate(pool);
            edge.endpoints.a = reduced_face_indices.span[0];
            edge.endpoints.b = reduced_face_indices.span[1];
            edge.face_normal = initial_face_normal;
            let edge_offset = points[edge.endpoints.b as usize] - points[edge.endpoints.a as usize];
            let basis_y = edge_offset.cross(edge.face_normal);
            let basis_x = edge_offset.cross(basis_y);
            if basis_x.dot(edge.face_normal) > 0.0 {
                let tmp = edge.endpoints.a;
                edge.endpoints.a = edge.endpoints.b;
                edge.endpoints.b = tmp;
            }
        }

        while edges_to_test.count > 0 {
            let edge_to_test = edges_to_test.pop();
            if edge_face_counts
                .get(&edge_to_test.endpoints)
                .map_or(false, |c| *c >= 2)
            {
                continue;
            }

            let edge_a = points[edge_to_test.endpoints.a as usize];
            let edge_b = points[edge_to_test.endpoints.b as usize];
            let edge_offset = edge_b - edge_a;
            let basis_y = edge_offset.cross(edge_to_test.face_normal);
            let basis_x = edge_offset.cross(basis_y);
            let basis_x_norm = basis_x.normalize();
            let basis_y_norm = basis_y.normalize();
            let basis_x_bundle = Vector3Wide::broadcast(basis_x_norm);
            let basis_y_bundle = Vector3Wide::broadcast(basis_y_norm);
            let basis_origin = Vector3Wide::broadcast(edge_a);

            raw_face_vertex_indices.count = 0;
            let mut face_normal = Vec3::ZERO;
            Self::find_extreme_face(
                &basis_x_bundle,
                &basis_y_bundle,
                &basis_origin,
                &edge_to_test.endpoints,
                &mut point_bundles,
                &index_offset_bundle,
                &allow_vertices,
                points.len(),
                &mut projected_on_x,
                &mut projected_on_y,
                &plane_slab_epsilon,
                &mut raw_face_vertex_indices,
                &mut face_normal,
            );

            reduced_face_indices.count = 0;
            face_points.count = 0;
            Self::reduce_face(
                &mut raw_face_vertex_indices,
                face_normal,
                points,
                plane_slab_epsilon_narrow,
                &mut face_points,
                &mut allow_vertices,
                &mut reduced_face_indices,
            );

            if reduced_face_indices.count < 3 {
                continue;
            }

            // Check for coplanar face merging.
            let mut merged_face = false;
            for i in 0..faces.count {
                let face = unsafe { &mut *faces.span.get_mut(i as i32) };
                if face.normal.dot(face_normal) > normal_coplanarity_epsilon {
                    // Merge the new face into the existing face.
                    raw_face_vertex_indices.ensure_capacity(
                        reduced_face_indices.count + face.vertex_indices.count,
                        pool,
                    );
                    raw_face_vertex_indices.count = reduced_face_indices.count;
                    reduced_face_indices.span.copy_to(
                        0,
                        &mut raw_face_vertex_indices.span,
                        0,
                        reduced_face_indices.count,
                    );
                    for j in 0..face.vertex_indices.count {
                        let vertex_index = face.vertex_indices.span[j as usize];
                        if allow_vertices[vertex_index as usize] != 0
                            && !reduced_face_indices.contains(&vertex_index)
                        {
                            raw_face_vertex_indices.add_unsafely(vertex_index);
                        }
                    }

                    face.vertex_indices.count = 0;
                    face_points.count = 0;
                    face.vertex_indices
                        .ensure_capacity(raw_face_vertex_indices.count, pool);
                    Self::reduce_face(
                        &mut raw_face_vertex_indices,
                        face_normal,
                        points,
                        plane_slab_epsilon_narrow,
                        &mut face_points,
                        &mut allow_vertices,
                        &mut face.vertex_indices,
                    );
                    merged_face = true;
                    break;
                }
            }

            if !merged_face {
                let face_count_prior_to_add = faces.count;
                Self::add_face(&mut faces, pool, face_normal, &reduced_face_indices);
                Self::add_face_edges_to_test_list(
                    pool,
                    &mut reduced_face_indices,
                    &mut edges_to_test,
                    &mut edge_face_counts,
                    face_normal,
                    face_count_prior_to_add,
                );
            }

            // Check all faces for disallowed vertices; delete faces that contain them.
            let mut deleted_face_count = 0i32;
            for i in 0..faces.count {
                let face = unsafe { &mut *faces.span.get_mut(i as i32) };
                let mut deleted_face = false;
                for j in 0..face.vertex_indices.count {
                    if allow_vertices[face.vertex_indices.span[j as usize] as usize] == 0 {
                        deleted_face_count += 1;
                        deleted_face = true;
                        break;
                    }
                }
                if deleted_face {
                    // Edges may have been exposed; adjust edge-face counts.
                    for j in 0..face.vertex_indices.count {
                        let prev_j = if j == 0 {
                            face.vertex_indices.count - 1
                        } else {
                            j - 1
                        };
                        let previous_index = face.vertex_indices.span[prev_j as usize];
                        let next_index = face.vertex_indices.span[j as usize];
                        // Note: endpoints are flipped to fill the void.
                        let endpoints = EdgeEndpoints {
                            a: next_index,
                            b: previous_index,
                        };
                        let mut table_index = 0i32;
                        let mut element_index = 0i32;
                        if edge_face_counts.get_table_indices(
                            &endpoints,
                            &mut table_index,
                            &mut element_index,
                        ) {
                            if allow_vertices[endpoints.a as usize] != 0
                                && allow_vertices[endpoints.b as usize] != 0
                            {
                                edges_to_test.add(
                                    EdgeToTest {
                                        endpoints,
                                        face_normal: face.normal,
                                    },
                                    pool,
                                );
                            }
                        }
                    }
                    face.vertex_indices.dispose(pool);
                }
                if !deleted_face && deleted_face_count > 0 {
                    let src = unsafe { std::ptr::read(faces.span.get(i as i32)) };
                    unsafe {
                        std::ptr::write(faces.span.get_mut((i - deleted_face_count) as i32), src);
                    }
                }
            }
            faces.count -= deleted_face_count;
        }

        edges_to_test.dispose(pool);
        face_points.dispose(pool);
        reduced_face_indices.dispose(pool);
        raw_face_vertex_indices.dispose(pool);
        pool.return_buffer(&mut allow_vertices);
        pool.return_buffer(&mut projected_on_x);
        pool.return_buffer(&mut projected_on_y);
        pool.return_buffer(&mut point_bundles);

        // Create reduced hull point set from face vertex references.
        let mut total_index_count = 0i32;
        for i in 0..faces.count {
            total_index_count += faces.span[i as usize].vertex_indices.count;
        }
        hull_data.face_start_indices = pool.take_at_least(faces.count);
        hull_data.face_vertex_indices = pool.take_at_least(total_index_count);
        let mut next_start_index = 0i32;
        let mut original_to_hull_index_mapping: Buffer<i32> =
            pool.take_at_least(points.len() as i32);
        let mut hull_to_original_index_mapping =
            QuickList::<i32>::with_capacity(points.len() as i32, pool);
        for i in 0..points.len() {
            unsafe {
                *original_to_hull_index_mapping.get_mut(i as i32) = -1;
            }
        }

        for i in 0..faces.count {
            let source = &faces.span[i as usize].vertex_indices;
            unsafe {
                *hull_data.face_start_indices.get_mut(i) = next_start_index;
            }
            for j in 0..source.count {
                let original_vertex_index = source.span[j as usize];
                let original_to_hull =
                    unsafe { original_to_hull_index_mapping.get_mut(original_vertex_index) };
                if *original_to_hull < 0 {
                    *original_to_hull = hull_to_original_index_mapping.count;
                    hull_to_original_index_mapping.add_unsafely(original_vertex_index);
                }
                unsafe {
                    *hull_data.face_vertex_indices.get_mut(next_start_index + j) =
                        *original_to_hull;
                }
            }
            next_start_index += source.count;
        }

        hull_data.original_vertex_mapping =
            pool.take_at_least(hull_to_original_index_mapping.count);
        hull_to_original_index_mapping.span.copy_to(
            0,
            &mut hull_data.original_vertex_mapping,
            0,
            hull_to_original_index_mapping.count,
        );

        pool.return_buffer(&mut original_to_hull_index_mapping);
        hull_to_original_index_mapping.dispose(pool);
        for i in 0..faces.count {
            let face = unsafe { &mut *faces.span.get_mut(i as i32) };
            face.vertex_indices.dispose(pool);
        }
        faces.dispose(pool);
    }

    /// Processes hull data into a runtime usable convex hull shape.
    /// Recenters the convex hull's points around its center of mass.
    /// Returns true if the shape was created successfully; false if the hull has no volume.
    pub fn create_shape(
        points: &[Vec3],
        hull_data: &HullData,
        pool: &mut BufferPool,
        center: &mut Vec3,
        hull_shape: &mut ConvexHull,
    ) -> bool {
        debug_assert!(!points.is_empty(), "Convex hulls need nonzero points!");
        *hull_shape = ConvexHull {
            points: Buffer::default(),
            bounding_planes: Buffer::default(),
            face_vertex_indices: Buffer::default(),
            face_to_vertex_indices_start: Buffer::default(),
        };
        let point_bundle_count =
            BundleIndexing::get_bundle_count(hull_data.original_vertex_mapping.len() as usize);
        hull_shape.points = pool.take_at_least(point_bundle_count as i32);

        let mut volume = 0.0f32;
        *center = Vec3::ZERO;
        for face_index in 0..hull_data.face_start_indices.len() as i32 {
            let face = hull_data.get_face(face_index);
            for subtriangle_index in 2..face.vertex_count() {
                let a = points[face.get(0) as usize];
                let b = points[face.get(subtriangle_index - 1) as usize];
                let c = points[face.get(subtriangle_index) as usize];
                let volume_contribution = MeshInertiaHelper::compute_tetrahedron_volume(a, b, c);
                volume += volume_contribution;
                let centroid = a + b + c;
                *center += centroid * volume_contribution;
            }
        }
        // Division by 4 since we accumulated (a + b + c) rather than (a + b + c + 0) / 4.
        *center /= volume * 4.0;

        if center.x.is_nan()
            || center.y.is_nan()
            || center.z.is_nan()
            || hull_data.face_start_indices.len() == 2
        {
            pool.return_buffer(&mut hull_shape.points);
            *center = Vec3::ZERO;
            return false;
        }

        let last_index = hull_data.original_vertex_mapping.len() as i32 - 1;
        for bundle_index in 0..hull_shape.points.len() as usize {
            let bundle = unsafe { hull_shape.points.get_mut(bundle_index as i32) };
            for inner_index in 0..VECTOR_WIDTH {
                let mut index =
                    ((bundle_index << BundleIndexing::vector_shift()) + inner_index) as i32;
                if index > last_index {
                    index = last_index;
                }
                let point = points[hull_data.original_vertex_mapping[index as usize] as usize];
                Vector3Wide::write_slot(point - *center, inner_index, bundle);
            }
        }

        // Create face->vertex mapping.
        hull_shape.face_to_vertex_indices_start =
            pool.take_at_least(hull_data.face_start_indices.len());
        {
            let count = hull_shape.face_to_vertex_indices_start.len();
            hull_data.face_start_indices.copy_to(
                0,
                &mut hull_shape.face_to_vertex_indices_start,
                0,
                count,
            );
        }
        hull_shape.face_vertex_indices = pool.take_at_least(hull_data.face_vertex_indices.len());
        for i in 0..hull_shape.face_vertex_indices.len() as usize {
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(
                hull_data.face_vertex_indices[i] as usize,
                &mut bundle_index,
                &mut inner_index,
            );
            let face_vertex = unsafe { hull_shape.face_vertex_indices.get_mut(i as i32) };
            face_vertex.bundle_index = bundle_index as u16;
            face_vertex.inner_index = inner_index as u16;
        }

        // Create bounding planes.
        let face_bundle_count = BundleIndexing::get_bundle_count(
            hull_shape.face_to_vertex_indices_start.len() as usize,
        );
        hull_shape.bounding_planes = pool.take_at_least(face_bundle_count as i32);

        for i in 0..hull_shape.face_to_vertex_indices_start.len() as usize {
            let mut start = 0usize;
            let mut count = 0usize;
            hull_shape.get_vertex_indices_for_face(i, &mut start, &mut count);
            debug_assert!(count >= 3);

            let mut face_normal = Vec3::ZERO;
            let mut face_pivot = Vec3::ZERO;
            hull_shape
                .get_point_by_vertex_index(&hull_shape.face_vertex_indices[start], &mut face_pivot);
            let mut face_vertex = Vec3::ZERO;
            hull_shape.get_point_by_vertex_index(
                &hull_shape.face_vertex_indices[start + 1],
                &mut face_vertex,
            );
            let mut previous_offset = face_vertex - face_pivot;
            for j in 2..count {
                hull_shape.get_point_by_vertex_index(
                    &hull_shape.face_vertex_indices[start + j],
                    &mut face_vertex,
                );
                let offset = face_vertex - face_pivot;
                face_normal += previous_offset.cross(offset);
                previous_offset = offset;
            }
            let length = face_normal.length();
            debug_assert!(length > 1e-10);
            face_normal /= length;

            let mut bounding_plane_bundle_index = 0usize;
            let mut bounding_plane_inner_index = 0usize;
            BundleIndexing::get_bundle_indices(
                i,
                &mut bounding_plane_bundle_index,
                &mut bounding_plane_inner_index,
            );
            let bounding_bundle = unsafe {
                hull_shape
                    .bounding_planes
                    .get_mut(bounding_plane_bundle_index as i32)
            };
            let bounding_offset_bundle = unsafe {
                GatherScatter::get_offset_instance_mut(bounding_bundle, bounding_plane_inner_index)
            };
            Vector3Wide::write_first(face_normal, &mut bounding_offset_bundle.normal);
            unsafe {
                *GatherScatter::get_first_mut(&mut bounding_offset_bundle.offset) =
                    face_pivot.dot(face_normal);
            }
        }

        // Clear trailing bounding plane data.
        let bounding_plane_capacity = hull_shape.bounding_planes.len() as usize * VECTOR_WIDTH;
        for i in hull_shape.face_to_vertex_indices_start.len() as usize..bounding_plane_capacity {
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(i, &mut bundle_index, &mut inner_index);
            let offset_instance = unsafe {
                GatherScatter::get_offset_instance_mut(
                    hull_shape.bounding_planes.get_mut(bundle_index as i32),
                    inner_index,
                )
            };
            Vector3Wide::write_first(Vec3::ZERO, &mut offset_instance.normal);
            unsafe {
                *GatherScatter::get_first_mut(&mut offset_instance.offset) = f32::MIN;
            }
        }

        true
    }

    /// Creates a convex hull shape from an input point set, returning intermediate hull data.
    /// Recenters the convex hull's points around its center of mass.
    pub fn create_shape_with_hull_data(
        points: &[Vec3],
        pool: &mut BufferPool,
        hull_data: &mut HullData,
        center: &mut Vec3,
        convex_hull: &mut ConvexHull,
    ) -> bool {
        Self::compute_hull(points, pool, hull_data);
        Self::create_shape(points, hull_data, pool, center, convex_hull)
    }

    /// Creates a convex hull shape from an input point set.
    /// Recenters the convex hull's points around its center of mass.
    pub fn create_shape_simple(
        points: &[Vec3],
        pool: &mut BufferPool,
        center: &mut Vec3,
        convex_hull: &mut ConvexHull,
    ) -> bool {
        let mut hull_data = HullData {
            original_vertex_mapping: Buffer::default(),
            face_vertex_indices: Buffer::default(),
            face_start_indices: Buffer::default(),
        };
        Self::compute_hull(points, pool, &mut hull_data);
        let result = Self::create_shape(points, &hull_data, pool, center, convex_hull);
        if hull_data.original_vertex_mapping.allocated() {
            hull_data.dispose(pool);
        }
        result
    }

    /// Creates a transformed copy of a convex hull.
    pub fn create_transformed_copy(
        source: &ConvexHull,
        transform: &Matrix3x3,
        target_points: &mut Buffer<Vector3Wide>,
        target_bounding_planes: &mut Buffer<HullBoundingPlanes>,
    ) {
        debug_assert!(target_points.len() >= source.points.len());
        debug_assert!(target_bounding_planes.len() >= source.bounding_planes.len());

        let mut transform_wide = Matrix3x3Wide::default();
        Matrix3x3Wide::broadcast(transform, &mut transform_wide);
        for i in 0..source.points.len() as usize {
            Matrix3x3Wide::transform_without_overlap(&source.points[i], &transform_wide, unsafe {
                target_points.get_mut(i as i32)
            });
        }
        let mut inverse = Matrix3x3::default();
        Matrix3x3::invert(transform, &mut inverse);
        let mut inverse_wide = Matrix3x3Wide::default();
        Matrix3x3Wide::broadcast(&inverse, &mut inverse_wide);
        for i in 0..source.bounding_planes.len() as usize {
            let mut normal = Vector3Wide::default();
            Matrix3x3Wide::transform_by_transposed_without_overlap(
                &source.bounding_planes[i].normal,
                &inverse_wide,
                &mut normal,
            );
            unsafe {
                target_bounding_planes.get_mut(i as i32).normal = Vector3Wide::normalize(&normal);
            }
        }

        for face_index in 0..source.face_to_vertex_indices_start.len() as usize {
            let vertex_index = source.face_vertex_indices
                [source.face_to_vertex_indices_start[face_index] as usize];
            let mut bundle_index = 0usize;
            let mut index_in_bundle = 0usize;
            BundleIndexing::get_bundle_indices(face_index, &mut bundle_index, &mut index_in_bundle);
            let mut point = Vec3::ZERO;
            Vector3Wide::read_slot(
                &target_points[vertex_index.bundle_index as usize],
                vertex_index.inner_index as usize,
                &mut point,
            );
            let mut normal = Vec3::ZERO;
            Vector3Wide::read_slot(
                unsafe { &target_bounding_planes.get(bundle_index as i32).normal },
                index_in_bundle,
                &mut normal,
            );
            unsafe {
                let offset_vec = &mut target_bounding_planes.get_mut(bundle_index as i32).offset;
                let slot_ptr = (offset_vec as *mut Vector<f32> as *mut f32).add(index_in_bundle);
                *slot_ptr = point.dot(normal);
            }
        }

        // Clear trailing bounding plane data.
        let bounding_plane_capacity = target_bounding_planes.len() as usize * VECTOR_WIDTH;
        for i in source.face_to_vertex_indices_start.len() as usize..bounding_plane_capacity {
            let mut bundle_index = 0usize;
            let mut inner_index = 0usize;
            BundleIndexing::get_bundle_indices(i, &mut bundle_index, &mut inner_index);
            let offset_instance = unsafe {
                GatherScatter::get_offset_instance_mut(
                    target_bounding_planes.get_mut(bundle_index as i32),
                    inner_index,
                )
            };
            Vector3Wide::write_first(Vec3::ZERO, &mut offset_instance.normal);
            unsafe {
                *GatherScatter::get_first_mut(&mut offset_instance.offset) = f32::MIN;
            }
        }
    }

    /// Creates a transformed shallow copy of a convex hull.
    /// FaceVertexIndices and FaceToVertexIndicesStart are shared with the source.
    pub fn create_transformed_shallow_copy(
        source: &ConvexHull,
        transform: &Matrix3x3,
        pool: &mut BufferPool,
        target: &mut ConvexHull,
    ) {
        target.points = pool.take_at_least(source.points.len());
        target.bounding_planes = pool.take_at_least(source.bounding_planes.len());
        Self::create_transformed_copy(
            source,
            transform,
            &mut target.points,
            &mut target.bounding_planes,
        );
        target.face_vertex_indices = source.face_vertex_indices;
        target.face_to_vertex_indices_start = source.face_to_vertex_indices_start;
    }

    /// Creates a transformed deep copy of a convex hull.
    pub fn create_transformed_deep_copy(
        source: &ConvexHull,
        transform: &Matrix3x3,
        pool: &mut BufferPool,
        target: &mut ConvexHull,
    ) {
        target.points = pool.take_at_least(source.points.len());
        target.bounding_planes = pool.take_at_least(source.bounding_planes.len());
        target.face_vertex_indices = pool.take_at_least(source.face_vertex_indices.len());
        target.face_to_vertex_indices_start =
            pool.take_at_least(source.face_to_vertex_indices_start.len());
        Self::create_transformed_copy(
            source,
            transform,
            &mut target.points,
            &mut target.bounding_planes,
        );
        {
            let count = target.face_vertex_indices.len();
            source
                .face_vertex_indices
                .copy_to(0, &mut target.face_vertex_indices, 0, count);
        }
        {
            let count = target.face_to_vertex_indices_start.len();
            source.face_to_vertex_indices_start.copy_to(
                0,
                &mut target.face_to_vertex_indices_start,
                0,
                count,
            );
        }
    }
}
