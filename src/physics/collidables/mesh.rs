use glam::Vec3;

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

    // TODO: The following methods depend on Tree and collision infrastructure:
    // - fill_subtrees_for_triangles
    // - create_without_tree_build
    // - create_with_sweep_build
    // - new (constructor with binned build)
    // - serialize / deserialize
    // - compute_bounds
    // - ray_test
    // - find_local_overlaps
}

impl IShape for Mesh {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}
