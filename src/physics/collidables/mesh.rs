use glam::{Quat, Vec3};

use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;

use super::shape::IShape;
use super::triangle::Triangle;

// Forward reference: Tree type from physics::trees module
// use crate::physics::trees::Tree;

/// Shape designed to contain a whole bunch of triangles.
/// Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound
/// clockwise in right handed coordinates or counterclockwise in left handed coordinates
/// will generate contacts.
pub struct Mesh {
    /// Acceleration structure of the mesh.
    /// TODO: Wire up Tree type from physics::trees
    // pub tree: Tree,

    /// Buffer of triangles composing the mesh.
    pub triangles: Buffer<Triangle>,

    /// Scale applied to all vertices at runtime.
    scale: Vec3,

    /// Precomputed inverse scale.
    inverse_scale: Vec3,
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

    /// Disposes resources.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.triangles);
        // TODO: self.tree.dispose(pool);
    }

    /// Creates a mesh without building the tree (for use before tree build is available).
    pub fn create_without_tree_build(triangles: Buffer<Triangle>, scale: Vec3) -> Self {
        let inverse_scale = Vec3::new(
            if scale.x != 0.0 { 1.0 / scale.x } else { f32::MAX },
            if scale.y != 0.0 { 1.0 / scale.y } else { f32::MAX },
            if scale.z != 0.0 { 1.0 / scale.z } else { f32::MAX },
        );
        Self {
            triangles,
            scale,
            inverse_scale,
        }
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

    // TODO: The following methods depend on Tree and collision infrastructure:
    // - fill_subtrees_for_triangles
    // - create_with_sweep_build
    // - new (constructor with binned build — requires Tree)
    // - serialize / deserialize
    // - ray_test
    // - find_local_overlaps
}

impl IShape for Mesh {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
