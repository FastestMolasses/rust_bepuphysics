use glam::{Quat, Vec3};
use std::mem::size_of;

use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::symmetric3x3::Symmetric3x3;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::for_each_ref::IBreakableForEach;

use super::mesh_inertia_helper::{ITriangleSource, MeshInertiaHelper};
use crate::physics::collision_detection::ray_batchers::RayData;
use super::shape::{IShape, IDisposableShape, IHomogeneousCompoundShape};
use crate::physics::collision_detection::ray_batchers::IShapeRayHitHandler;
use super::shapes::Shapes;
use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use super::triangle::Triangle;
use super::triangle::TriangleWide;

// The IRayLeafTester trait uses the RayData type from ray_batcher, not collidables::ray.
use crate::physics::trees::ray_batcher::RayData as TreeRayData;

use crate::physics::body_properties::{BodyInertia, RigidPose};
use crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::{
    ICollisionTaskOverlaps, ICollisionTaskSubpairOverlaps, OverlapQueryForPair,
};
use crate::physics::trees::node::NodeChild;
use crate::physics::trees::tree::Tree;
use crate::physics::trees::tree_ray_cast::IRayLeafTester;
use crate::physics::trees::tree_sweep::ISweepLeafTester;

/// Enumerator for tree overlap queries, used by Mesh and BigCompound.
pub struct ShapeTreeOverlapEnumerator<'a, TSubpairOverlaps: ICollisionTaskSubpairOverlaps> {
    pub pool: &'a mut BufferPool,
    pub overlaps: *mut TSubpairOverlaps,
}

impl<'a, TSubpairOverlaps: ICollisionTaskSubpairOverlaps> IBreakableForEach<i32>
    for ShapeTreeOverlapEnumerator<'a, TSubpairOverlaps>
{
    #[inline(always)]
    fn loop_body(&mut self, i: i32) -> bool {
        unsafe {
            *(*self.overlaps).allocate(self.pool) = i;
        }
        true
    }
}

/// Sweep leaf tester for tree sweep queries, used by Mesh and BigCompound.
pub struct ShapeTreeSweepLeafTester<'a, TOverlaps: ICollisionTaskSubpairOverlaps> {
    pub pool: &'a mut BufferPool,
    pub overlaps: *mut TOverlaps,
}

impl<'a, TOverlaps: ICollisionTaskSubpairOverlaps> ISweepLeafTester
    for ShapeTreeSweepLeafTester<'a, TOverlaps>
{
    #[inline(always)]
    fn test_leaf(&mut self, leaf_index: i32, _maximum_t: &mut f32) {
        unsafe {
            *(*self.overlaps).allocate(self.pool) = leaf_index;
        }
    }
}

/// Shape designed to contain a whole bunch of triangles.
/// Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound
/// clockwise in right handed coordinates or counterclockwise in left handed coordinates
/// will generate contacts.
pub struct Mesh {
    /// Acceleration structure of the mesh.
    pub tree: Tree,

    /// Buffer of triangles composing the mesh.
    pub triangles: Buffer<Triangle>,

    /// Scale applied to all vertices at runtime.
    scale: Vec3,

    /// Precomputed inverse scale.
    inverse_scale: Vec3,
}

/// Ray leaf tester for mesh ray casts.
struct HitLeafTester<THandler: IShapeRayHitHandler> {
    triangles: *const Triangle,
    hit_handler: THandler,
    orientation: Matrix3x3,
    inverse_scale: Vec3,
    original_ray: RayData,
}

impl<THandler: IShapeRayHitHandler> IRayLeafTester for HitLeafTester<THandler> {
    #[inline(always)]
    unsafe fn test_leaf(
        &mut self,
        leaf_index: i32,
        ray_data: *mut TreeRayData,
        maximum_t: *mut f32,
    ) {
        let triangle = &*self.triangles.add(leaf_index as usize);
        let mut t = 0.0f32;
        let mut normal = Vec3::ZERO;
        if Triangle::ray_test_vertices(
            triangle.a,
            triangle.b,
            triangle.c,
            (*ray_data).origin,
            (*ray_data).direction,
            &mut t,
            &mut normal,
        ) && t <= *maximum_t
        {
            // Pull the hit back into world space before handing it off to the user.
            let scaled_normal = normal * self.inverse_scale;
            let mut world_normal = Vec3::ZERO;
            Matrix3x3::transform(&scaled_normal, &self.orientation, &mut world_normal);
            world_normal = world_normal.normalize();
            self.hit_handler
                .on_ray_hit(&self.original_ray, &mut *maximum_t, t, world_normal, leaf_index);
        }
    }
}

/// Triangle source for mesh inertia computation.
pub struct MeshTriangleSource<'a> {
    mesh: &'a Mesh,
    triangle_index: i32,
}

impl<'a> MeshTriangleSource<'a> {
    pub fn new(mesh: &'a Mesh) -> Self {
        Self {
            mesh,
            triangle_index: 0,
        }
    }
}

impl<'a> ITriangleSource for MeshTriangleSource<'a> {
    #[inline(always)]
    fn get_next_triangle(&mut self) -> Option<(Vec3, Vec3, Vec3)> {
        if self.triangle_index < self.mesh.triangles.len() {
            let triangle = &self.mesh.triangles[self.triangle_index as usize];
            self.triangle_index += 1;
            Some((
                triangle.a * self.mesh.scale,
                triangle.b * self.mesh.scale,
                triangle.c * self.mesh.scale,
            ))
        } else {
            None
        }
    }
}

impl Mesh {
    /// Type id of mesh shapes.
    pub const ID: i32 = 8;

    /// Gets the scale of the mesh.
    pub fn scale(&self) -> Vec3 {
        self.scale
    }

    /// Sets the scale of the mesh.
    pub fn set_scale(&mut self, value: Vec3) {
        self.scale = value;
        self.inverse_scale = Vec3::new(
            if value.x != 0.0 {
                1.0 / value.x
            } else {
                f32::MAX
            },
            if value.y != 0.0 {
                1.0 / value.y
            } else {
                f32::MAX
            },
            if value.z != 0.0 {
                1.0 / value.z
            } else {
                f32::MAX
            },
        );
    }

    /// Gets the inverse scale.
    pub fn inverse_scale(&self) -> Vec3 {
        self.inverse_scale
    }

    /// Fills a buffer of subtrees according to a buffer of triangles.
    pub fn fill_subtrees_for_triangles(triangles: &[Triangle], subtrees: &mut [NodeChild]) {
        assert_eq!(
            triangles.len(),
            subtrees.len(),
            "Triangles and subtrees span lengths should match."
        );
        for i in 0..triangles.len() {
            let t = &triangles[i];
            let subtree = &mut subtrees[i];
            subtree.min = t.a.min(t.b.min(t.c));
            subtree.max = t.a.max(t.b.max(t.c));
            subtree.leaf_count = 1;
            subtree.index = Tree::encode(i as i32);
        }
    }

    /// Creates a mesh shape instance, but leaves the Tree in an unbuilt state.
    /// The tree must be built before the mesh can be used.
    pub fn create_without_tree_build(
        triangles: Buffer<Triangle>,
        scale: Vec3,
        pool: &mut BufferPool,
    ) -> Self {
        let tri_len = triangles.len();
        let mut tree = Tree::new(pool, tri_len);
        // Set node/leaf counts as if the tree is fully populated.
        // Note that the tree still has a root node even if there's one leaf.
        tree.node_count = 1i32.max(tri_len - 1);
        tree.leaf_count = tri_len;

        let mut mesh = Self {
            tree,
            triangles,
            scale: Vec3::ZERO,
            inverse_scale: Vec3::ZERO,
        };
        mesh.set_scale(scale);
        mesh
    }

    /// Creates a mesh shape instance and builds an acceleration structure using a sweep builder.
    pub unsafe fn create_with_sweep_build(
        triangles: Buffer<Triangle>,
        scale: Vec3,
        pool: &mut BufferPool,
    ) -> Self {
        let tri_len = triangles.len();
        let mut mesh = Self::create_without_tree_build(triangles, scale, pool);
        let mut subtrees: Buffer<NodeChild> = pool.take_at_least(tri_len);
        Self::fill_subtrees_for_triangles(
            std::slice::from_raw_parts(mesh.triangles.as_ptr(), tri_len as usize),
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), tri_len as usize),
        );
        // NodeChild intentionally shares the same byte size as BoundingBox.
        // SweepBuild takes Buffer<BoundingBox>.
        let leaf_bounds = subtrees.cast::<crate::utilities::bounding_box::BoundingBox>();
        mesh.tree.sweep_build(pool, &leaf_bounds);
        pool.return_buffer(&mut subtrees);
        mesh
    }

    /// Creates a mesh shape with an acceleration structure built using the binned builder.
    pub unsafe fn new(
        triangles: Buffer<Triangle>,
        scale: Vec3,
        pool: &mut BufferPool,
    ) -> Self {
        let tri_len = triangles.len();
        let mut mesh = Self::create_without_tree_build(triangles, scale, pool);
        let mut subtrees: Buffer<NodeChild> = pool.take_at_least(tri_len);
        Self::fill_subtrees_for_triangles(
            std::slice::from_raw_parts(mesh.triangles.as_ptr(), tri_len as usize),
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), tri_len as usize),
        );
        mesh.tree.binned_build(subtrees, Some(pool as *mut BufferPool), None, None, 0, -1, -1, 16, 64, 1.0 / 16.0, 64, false);
        pool.return_buffer(&mut subtrees);
        mesh
    }

    /// Loads a mesh from data stored in a byte buffer previously stored by the `serialize` function.
    pub unsafe fn from_bytes(data: &[u8], pool: &mut BufferPool) -> Self {
        assert!(data.len() >= 16, "Data is not large enough to contain a header.");

        let scale = *(data.as_ptr() as *const Vec3);
        let triangle_count = *(data.as_ptr().add(12) as *const i32);
        let triangle_byte_count = triangle_count as usize * size_of::<Triangle>();
        assert!(
            data.len() >= 16 + triangle_byte_count,
            "Data is not large enough to contain the number of triangles specified in the header."
        );

        let tree_data = &data[16 + triangle_byte_count..];
        let tree = Tree::from_bytes(tree_data, pool);

        let mut triangles: Buffer<Triangle> = pool.take_at_least(triangle_count);
        std::ptr::copy_nonoverlapping(
            data.as_ptr().add(16),
            triangles.as_mut_ptr() as *mut u8,
            triangle_byte_count,
        );

        let mut mesh = Self {
            tree,
            triangles,
            scale: Vec3::ZERO,
            inverse_scale: Vec3::ZERO,
        };
        mesh.set_scale(scale);
        mesh
    }

    /// Gets the number of bytes it would take to store the mesh in a byte buffer.
    pub fn get_serialized_byte_count(&self) -> usize {
        16 + self.triangles.len() as usize * size_of::<Triangle>()
            + self.tree.get_serialized_byte_count()
    }

    /// Writes the mesh's data to a byte buffer.
    pub unsafe fn serialize(&self, data: &mut [u8]) {
        let required = self.get_serialized_byte_count();
        assert!(
            data.len() >= required,
            "Target span size is less than the required size."
        );
        *(data.as_mut_ptr() as *mut Vec3) = self.scale;
        *(data.as_mut_ptr().add(12) as *mut i32) = self.triangles.len();
        let triangle_byte_count = self.triangles.len() as usize * size_of::<Triangle>();
        std::ptr::copy_nonoverlapping(
            self.triangles.as_ptr() as *const u8,
            data.as_mut_ptr().add(16),
            triangle_byte_count,
        );
        self.tree
            .serialize(&mut data[16 + triangle_byte_count..]);
    }

    /// Gets the number of children (triangles) in the mesh.
    #[inline(always)]
    pub fn child_count(&self) -> i32 {
        self.triangles.len()
    }

    /// Gets a local child triangle with scale applied.
    #[inline(always)]
    pub fn get_local_child(&self, triangle_index: i32, target: &mut Triangle) {
        let source = &self.triangles[triangle_index as usize];
        target.a = self.scale * source.a;
        target.b = self.scale * source.b;
        target.c = self.scale * source.c;
    }

    /// Gets a posed local child triangle. The pose is the centroid of the triangle.
    #[inline(always)]
    pub fn get_posed_local_child(
        &self,
        triangle_index: i32,
        target: &mut Triangle,
        child_pose: &mut RigidPose,
    ) {
        self.get_local_child(triangle_index, target);
        child_pose.position = (target.a + target.b + target.c) * (1.0 / 3.0);
        child_pose.orientation = Quat::IDENTITY;
        target.a -= child_pose.position;
        target.b -= child_pose.position;
        target.c -= child_pose.position;
    }

    /// Gets a local child triangle, inserting it into the first slot of a wide instance.
    #[inline(always)]
    pub fn get_local_child_wide(&self, triangle_index: i32, target: &mut TriangleWide) {
        let source = &self.triangles[triangle_index as usize];
        Vector3Wide::write_first(source.a * self.scale, &mut target.a);
        Vector3Wide::write_first(source.b * self.scale, &mut target.b);
        Vector3Wide::write_first(source.c * self.scale, &mut target.c);
    }

    /// Computes the bounding box of the mesh given an orientation.
    pub fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        let mut r = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&orientation, &mut r);
        *min = Vec3::splat(f32::MAX);
        *max = Vec3::splat(f32::MIN);
        for i in 0..self.triangles.len() as usize {
            let triangle = &self.triangles[i];
            let mut a = Vec3::ZERO;
            let mut b = Vec3::ZERO;
            let mut c = Vec3::ZERO;
            Matrix3x3::transform(&(self.scale * triangle.a), &r, &mut a);
            Matrix3x3::transform(&(self.scale * triangle.b), &r, &mut b);
            Matrix3x3::transform(&(self.scale * triangle.c), &r, &mut c);
            let min0 = a.min(b);
            let min1 = c.min(*min);
            let max0 = a.max(b);
            let max1 = c.max(*max);
            *min = min0.min(min1);
            *max = max0.max(max1);
        }
    }

    /// Casts a ray against the mesh. Executes a callback for every test candidate and every hit.
    pub unsafe fn ray_test<TRayHitHandler: IShapeRayHitHandler>(
        &self,
        pose: &RigidPose,
        ray: &RayData,
        maximum_t: &mut f32,
        hit_handler: &mut TRayHitHandler,
    ) {
        let mut leaf_tester = HitLeafTester {
            triangles: self.triangles.as_ptr(),
            hit_handler: std::ptr::read(hit_handler),
            orientation: Matrix3x3::default(),
            inverse_scale: self.inverse_scale,
            original_ray: *ray,
        };
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut leaf_tester.orientation);
        let mut inverse_orientation = Matrix3x3::default();
        Matrix3x3::transpose(&leaf_tester.orientation, &mut inverse_orientation);
        let offset = ray.origin - pose.position;
        let mut local_origin = Vec3::ZERO;
        let mut local_direction = Vec3::ZERO;
        Matrix3x3::transform(&offset, &inverse_orientation, &mut local_origin);
        Matrix3x3::transform(&ray.direction, &inverse_orientation, &mut local_direction);
        local_origin *= self.inverse_scale;
        local_direction *= self.inverse_scale;
        self.tree
            .ray_cast(local_origin, local_direction, maximum_t, &mut leaf_tester, 0);
        // The leaf tester could have mutated the hit handler; copy it back over.
        std::ptr::write(hit_handler, leaf_tester.hit_handler);
    }

    /// Finds overlapping children for pairs of bounding boxes against the mesh tree.
    pub unsafe fn find_local_overlaps<
        TOverlaps: ICollisionTaskOverlaps<TSubpairOverlaps>,
        TSubpairOverlaps: ICollisionTaskSubpairOverlaps,
    >(
        &self,
        pairs: &Buffer<OverlapQueryForPair>,
        pool: &mut BufferPool,
        overlaps: &mut TOverlaps,
    ) {
        let mut enumerator = ShapeTreeOverlapEnumerator::<TSubpairOverlaps> {
            pool,
            overlaps: std::ptr::null_mut(),
        };
        for i in 0..pairs.len() {
            let pair = &pairs[i as usize];
            let mesh = &*(pair.container as *const Mesh);
            let scaled_min = mesh.inverse_scale * pair.min;
            let scaled_max = mesh.inverse_scale * pair.max;
            enumerator.overlaps = overlaps.get_overlaps_for_pair(i) as *mut TSubpairOverlaps;
            // Take a min/max to compensate for negative scales.
            mesh.tree.get_overlaps_minmax(
                scaled_min.min(scaled_max),
                scaled_min.max(scaled_max),
                &mut enumerator,
            );
        }
    }

    /// Finds overlapping children for a swept bounding box against the mesh tree.
    pub unsafe fn find_local_overlaps_sweep<TOverlaps: ICollisionTaskSubpairOverlaps>(
        &self,
        min: Vec3,
        max: Vec3,
        sweep: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        overlaps: *mut TOverlaps,
    ) {
        let scaled_min = min * self.inverse_scale;
        let scaled_max = max * self.inverse_scale;
        let scaled_sweep = sweep * self.inverse_scale;
        let mut enumerator = ShapeTreeSweepLeafTester::<TOverlaps> { pool, overlaps };
        // Take a min/max to compensate for negative scales.
        self.tree.sweep(
            scaled_min.min(scaled_max),
            scaled_min.max(scaled_max),
            scaled_sweep,
            maximum_t,
            &mut enumerator,
        );
    }

    /// Subtracts the `new_center` from all points in the mesh hull.
    pub fn recenter(&mut self, new_center: Vec3) {
        let scaled_offset = new_center * self.inverse_scale;
        for i in 0..self.triangles.len() as usize {
            let triangle = &mut self.triangles[i];
            triangle.a -= scaled_offset;
            triangle.b -= scaled_offset;
            triangle.c -= scaled_offset;
        }
        for i in 0..self.tree.node_count as usize {
            let node = unsafe { self.tree.nodes.get_mut(i as i32) };
            node.a.min -= scaled_offset;
            node.a.max -= scaled_offset;
            node.b.min -= scaled_offset;
            node.b.max -= scaled_offset;
        }
    }

    /// Computes the inertia of the mesh around its volumetric center and recenters the points
    /// of the mesh around it. Assumes the mesh is closed and should be treated as solid.
    pub fn compute_closed_inertia_recentered(&mut self, mass: f32, center: &mut Vec3) -> BodyInertia {
        let mut triangle_source = MeshTriangleSource::new(self);
        let (_volume, inertia_tensor, com) =
            MeshInertiaHelper::compute_closed_inertia_with_center(&mut triangle_source, mass);
        *center = com;
        let inertia_offset = MeshInertiaHelper::get_inertia_offset(mass, *center);
        let recentered_inertia = inertia_tensor + inertia_offset;
        self.recenter(*center);
        let mut inverse_inertia_tensor = Symmetric3x3::default();
        Symmetric3x3::invert(&recentered_inertia, &mut inverse_inertia_tensor);
        let mut inertia = BodyInertia::default();
        inertia.inverse_inertia_tensor = inverse_inertia_tensor;
        inertia.inverse_mass = 1.0 / mass;
        inertia
    }

    /// Computes the inertia of the mesh.
    /// Assumes the mesh is closed and should be treated as solid.
    pub fn compute_closed_inertia(&self, mass: f32) -> BodyInertia {
        let mut triangle_source = MeshTriangleSource::new(self);
        let (_volume, inertia_tensor) =
            MeshInertiaHelper::compute_closed_inertia(&mut triangle_source, mass);
        let mut inverse_inertia_tensor = Symmetric3x3::default();
        Symmetric3x3::invert(&inertia_tensor, &mut inverse_inertia_tensor);
        let mut inertia = BodyInertia::default();
        inertia.inverse_inertia_tensor = inverse_inertia_tensor;
        inertia.inverse_mass = 1.0 / mass;
        inertia
    }

    /// Computes the volume and center of mass of the mesh.
    /// Assumes the mesh is closed and should be treated as solid.
    pub fn compute_closed_center_of_mass_with_volume(&self) -> (f32, Vec3) {
        let mut triangle_source = MeshTriangleSource::new(self);
        MeshInertiaHelper::compute_closed_center_of_mass(&mut triangle_source)
    }

    /// Computes the center of mass of the mesh.
    /// Assumes the mesh is closed and should be treated as solid.
    pub fn compute_closed_center_of_mass(&self) -> Vec3 {
        let mut triangle_source = MeshTriangleSource::new(self);
        let (_volume, center) =
            MeshInertiaHelper::compute_closed_center_of_mass(&mut triangle_source);
        center
    }

    /// Computes the inertia of the mesh around its volumetric center and recenters the points
    /// of the mesh around it. Assumes the mesh is open and should be treated as a triangle soup.
    pub fn compute_open_inertia_recentered(&mut self, mass: f32, center: &mut Vec3) -> BodyInertia {
        let mut triangle_source = MeshTriangleSource::new(self);
        let (inertia_tensor, com) =
            MeshInertiaHelper::compute_open_inertia_with_center(&mut triangle_source, mass);
        *center = com;
        let inertia_offset = MeshInertiaHelper::get_inertia_offset(mass, *center);
        let recentered_inertia = inertia_tensor + inertia_offset;
        self.recenter(*center);
        let mut inverse_inertia_tensor = Symmetric3x3::default();
        Symmetric3x3::invert(&recentered_inertia, &mut inverse_inertia_tensor);
        let mut inertia = BodyInertia::default();
        inertia.inverse_inertia_tensor = inverse_inertia_tensor;
        inertia.inverse_mass = 1.0 / mass;
        inertia
    }

    /// Computes the inertia of the mesh.
    /// Assumes the mesh is open and should be treated as a triangle soup.
    pub fn compute_open_inertia(&self, mass: f32) -> BodyInertia {
        let mut triangle_source = MeshTriangleSource::new(self);
        let inertia_tensor =
            MeshInertiaHelper::compute_open_inertia(&mut triangle_source, mass);
        let mut inverse_inertia_tensor = Symmetric3x3::default();
        Symmetric3x3::invert(&inertia_tensor, &mut inverse_inertia_tensor);
        let mut inertia = BodyInertia::default();
        inertia.inverse_inertia_tensor = inverse_inertia_tensor;
        inertia.inverse_mass = 1.0 / mass;
        inertia
    }

    /// Computes the center of mass of the mesh.
    /// Assumes the mesh is open and should be treated as a triangle soup.
    pub fn compute_open_center_of_mass(&self) -> Vec3 {
        let mut triangle_source = MeshTriangleSource::new(self);
        MeshInertiaHelper::compute_open_center_of_mass(&mut triangle_source)
    }

    /// Returns the mesh's resources to a buffer pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.triangles);
        self.tree.dispose(pool);
    }
}

impl IShape for Mesh {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IDisposableShape for Mesh {
    fn dispose(&mut self, pool: &mut BufferPool) {
        self.dispose(pool);
    }
}

impl IHomogeneousCompoundShape<Triangle, TriangleWide> for Mesh {
    fn child_count(&self) -> i32 {
        self.child_count()
    }

    fn get_local_child(&self, child_index: i32, child_data: &mut Triangle) {
        self.get_local_child(child_index, child_data);
    }

    fn get_local_child_wide(&self, child_index: i32, target: &mut TriangleWide) {
        self.get_local_child_wide(child_index, target);
    }

    fn get_posed_local_child(&self, child_index: i32, child_data: &mut Triangle, child_pose: &mut RigidPose) {
        self.get_posed_local_child(child_index, child_data, child_pose);
    }

    fn compute_bounds(&self, orientation: Quat, min: &mut Vec3, max: &mut Vec3) {
        self.compute_bounds(orientation, min, max);
    }

    fn find_local_overlaps<TOverlaps: super::compound::IOverlapCollector>(
        &self,
        local_min: &Vec3,
        local_max: &Vec3,
        overlaps: &mut TOverlaps,
    ) {
        // Adapter: bridge IOverlapCollector to IBreakableForEach<i32> for tree query.
        struct OverlapAdapter<'a, T: super::compound::IOverlapCollector>(&'a mut T);
        impl<T: super::compound::IOverlapCollector> IBreakableForEach<i32> for OverlapAdapter<'_, T> {
            fn loop_body(&mut self, i: i32) -> bool {
                self.0.add(i);
                true
            }
        }
        let mut adapter = OverlapAdapter(overlaps);
        self.tree.get_overlaps_minmax(*local_min, *local_max, &mut adapter);
    }
}

impl IBoundsQueryableCompound for Mesh {
    fn child_count(&self) -> i32 {
        self.child_count()
    }

    fn find_local_overlaps<TOverlaps, TSubpairOverlaps>(
        &self,
        query_bounds: &Buffer<OverlapQueryForPair>,
        pool: &mut BufferPool,
        _shapes: &Shapes,
        overlaps: &mut TOverlaps,
    ) where
        TSubpairOverlaps: ICollisionTaskSubpairOverlaps,
        TOverlaps: ICollisionTaskOverlaps<TSubpairOverlaps>,
    {
        struct MeshOverlapAdapter<'a, T: ICollisionTaskSubpairOverlaps> {
            overlaps: &'a mut T,
            pool: &'a mut BufferPool,
        }
        impl<T: ICollisionTaskSubpairOverlaps> IBreakableForEach<i32> for MeshOverlapAdapter<'_, T> {
            fn loop_body(&mut self, leaf_index: i32) -> bool {
                *self.overlaps.allocate(self.pool) = leaf_index;
                true
            }
        }

        for i in 0..query_bounds.len() {
            let pair = &query_bounds[i as usize];
            let mesh = unsafe { &*(pair.container as *const Mesh) };
            let scaled_min = mesh.inverse_scale * pair.min;
            let scaled_max = mesh.inverse_scale * pair.max;
            let overlaps_for_pair = overlaps.get_overlaps_for_pair(i);
            let mut adapter = MeshOverlapAdapter { overlaps: overlaps_for_pair, pool };
            // Take a min/max to compensate for negative scales.
            mesh.tree.get_overlaps_minmax(
                scaled_min.min(scaled_max),
                scaled_min.max(scaled_max),
                &mut adapter,
            );
        }
    }

    unsafe fn find_local_overlaps_sweep(
        &self,
        min: Vec3,
        max: Vec3,
        sweep: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        _shapes: &Shapes,
        overlaps: *mut u8,
    ) {
        let scaled_min = min * self.inverse_scale;
        let scaled_max = max * self.inverse_scale;
        let scaled_sweep = sweep * self.inverse_scale;
        let mut enumerator = ShapeTreeSweepLeafTester::<crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::ChildOverlapsCollection> {
            pool,
            overlaps: overlaps as *mut _,
        };
        // Take a min/max to compensate for negative scales.
        self.tree.sweep(
            scaled_min.min(scaled_max),
            scaled_min.max(scaled_max),
            scaled_sweep,
            maximum_t,
            &mut enumerator,
        );
    }
}
