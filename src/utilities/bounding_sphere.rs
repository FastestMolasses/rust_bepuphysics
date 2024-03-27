use glam::Vec3;

/// Provides XNA-like bounding sphere functionality.
#[derive(Clone, Copy, Debug)]
pub struct BoundingSphere {
    /// Location of the center of the sphere.
    pub center: Vec3,
    /// Radius of the sphere.
    pub radius: f32,
}

impl BoundingSphere {
    /// Constructs a new bounding sphere.
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }
}
