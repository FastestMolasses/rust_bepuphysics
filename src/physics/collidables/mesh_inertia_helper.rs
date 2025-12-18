use glam::Vec3;

use crate::utilities::symmetric3x3::Symmetric3x3;

/// Defines a type capable of providing a sequence of triangles.
pub trait ITriangleSource {
    /// Gets the next triangle in the sequence, if any.
    /// Returns `Some((a, b, c))` if there was another triangle, `None` otherwise.
    fn get_next_triangle(&mut self) -> Option<(Vec3, Vec3, Vec3)>;
}

/// Provides helpers for computing the inertia of objects with triangular surfaces.
pub struct MeshInertiaHelper;

impl MeshInertiaHelper {
    /// Computes the inertia tensor contribution of a tetrahedron with vertices at a, b, c, and (0,0,0).
    ///
    /// The mass parameter incorporates both the density and the volume scaling:
    /// density * abs(determinant) = density * volume * 6 = mass * 6.
    pub fn compute_tetrahedron_contribution(a: Vec3, b: Vec3, c: Vec3, mass: f32) -> Symmetric3x3 {
        let diagonal_scaling = mass * (6.0 / 60.0);
        let off_scaling = mass * (6.0 / 120.0);

        let xx = diagonal_scaling
            * (a.y * a.y + a.z * a.z + b.y * b.y + b.z * b.z + c.y * c.y + c.z * c.z
                + b.y * c.y
                + b.z * c.z
                + a.y * (b.y + c.y)
                + a.z * (b.z + c.z));

        let yy = diagonal_scaling
            * (a.x * a.x + a.z * a.z + b.x * b.x + b.z * b.z + c.x * c.x + c.z * c.z
                + b.x * c.x
                + b.z * c.z
                + a.x * (b.x + c.x)
                + a.z * (b.z + c.z));

        let zz = diagonal_scaling
            * (a.x * a.x + a.y * a.y + b.x * b.x + b.y * b.y + c.x * c.x + c.y * c.y
                + b.x * c.x
                + b.y * c.y
                + a.x * (b.x + c.x)
                + a.y * (b.y + c.y));

        let yx = off_scaling
            * (-2.0 * b.x * b.y
                - 2.0 * c.x * c.y
                - b.y * c.x
                - b.x * c.y
                - a.y * (b.x + c.x)
                - a.x * (2.0 * a.y + b.y + c.y));

        let zx = off_scaling
            * (-2.0 * b.x * b.z
                - 2.0 * c.x * c.z
                - b.z * c.x
                - b.x * c.z
                - a.z * (b.x + c.x)
                - a.x * (2.0 * a.z + b.z + c.z));

        let zy = off_scaling
            * (-2.0 * b.y * b.z
                - 2.0 * c.y * c.z
                - b.z * c.y
                - b.y * c.z
                - a.z * (b.y + c.y)
                - a.y * (2.0 * a.z + b.z + c.z));

        Symmetric3x3 {
            xx,
            yx,
            yy,
            zx,
            zy,
            zz,
        }
    }

    /// Computes the signed volume of a tetrahedron where the fourth vertex is at the origin.
    /// Triangles visible from outside the shape are assumed to have clockwise winding in right handed coordinates.
    #[inline(always)]
    pub fn compute_tetrahedron_volume(a: Vec3, b: Vec3, c: Vec3) -> f32 {
        (1.0 / 6.0) * b.cross(a).dot(c)
    }

    /// Integrates the inertia contribution of a tetrahedron with vertices at a, b, c, and (0,0,0) assuming a density of 1.
    /// Returns (volume, inertia_tensor).
    pub fn compute_tetrahedron_contribution_with_volume(
        a: Vec3,
        b: Vec3,
        c: Vec3,
    ) -> (f32, Symmetric3x3) {
        let volume = Self::compute_tetrahedron_volume(a, b, c);
        let inertia = Self::compute_tetrahedron_contribution(a, b, c, volume);
        (volume, inertia)
    }

    /// Computes the inertia of a closed mesh.
    /// Triangles visible from outside the shape are assumed to have clockwise winding in right handed coordinates.
    pub fn compute_closed_inertia(
        triangle_source: &mut dyn ITriangleSource,
        mass: f32,
    ) -> (f32, Symmetric3x3) {
        let mut volume = 0.0f32;
        let mut summed = Symmetric3x3::default();
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let (t_volume, t_contribution) =
                Self::compute_tetrahedron_contribution_with_volume(a, b, c);
            summed = summed + t_contribution;
            volume += t_volume;
        }
        let mut inertia = Symmetric3x3::default();
        Symmetric3x3::scale(&summed, mass / volume, &mut inertia);
        (volume, inertia)
    }

    /// Computes the inertia of a closed mesh with center of mass.
    pub fn compute_closed_inertia_with_center(
        triangle_source: &mut dyn ITriangleSource,
        mass: f32,
    ) -> (f32, Symmetric3x3, Vec3) {
        let mut volume = 0.0f32;
        let mut summed = Symmetric3x3::default();
        let mut center = Vec3::ZERO;
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let (t_volume, t_contribution) =
                Self::compute_tetrahedron_contribution_with_volume(a, b, c);
            summed = summed + t_contribution;
            volume += t_volume;
            center += (a + b + c) * t_volume;
        }
        let inverse_volume = 1.0 / volume;
        center *= inverse_volume * 0.25;
        let mut inertia = Symmetric3x3::default();
        Symmetric3x3::scale(&summed, mass * inverse_volume, &mut inertia);
        (volume, inertia, center)
    }

    /// Computes the center of mass of a closed mesh.
    pub fn compute_closed_center_of_mass(
        triangle_source: &mut dyn ITriangleSource,
    ) -> (f32, Vec3) {
        let mut center = Vec3::ZERO;
        let mut volume = 0.0f32;
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let t_volume = Self::compute_tetrahedron_volume(a, b, c);
            volume += t_volume;
            center += t_volume * (a + b + c);
        }
        center /= volume * 4.0;
        (volume, center)
    }

    /// Integrates the inertia contribution from a single triangle.
    pub fn compute_triangle_contribution(a: Vec3, b: Vec3, c: Vec3, mass: f32) -> Symmetric3x3 {
        let diagonal_scaling = mass * (2.0 / 12.0);
        let off_scaling = mass * (2.0 / 24.0);

        let xx = diagonal_scaling
            * (a.y * a.y + a.z * a.z + b.y * b.y + b.z * b.z + c.y * c.y + c.z * c.z
                + a.y * b.y
                + a.z * b.z
                + a.y * c.y
                + b.y * c.y
                + a.z * c.z
                + b.z * c.z);

        let yy = diagonal_scaling
            * (a.x * a.x + a.z * a.z + b.x * b.x + b.z * b.z + c.x * c.x + c.z * c.z
                + a.x * b.x
                + a.z * b.z
                + a.x * c.x
                + b.x * c.x
                + a.z * c.z
                + b.z * c.z);

        let zz = diagonal_scaling
            * (a.x * a.x + a.y * a.y + b.x * b.x + b.y * b.y + c.x * c.x + c.y * c.y
                + a.x * b.x
                + a.y * b.y
                + a.x * c.x
                + b.x * c.x
                + a.y * c.y
                + b.y * c.y);

        let yx = off_scaling
            * (-a.y * (b.x + c.x)
                - b.y * (2.0 * b.x + c.x)
                - (b.x + 2.0 * c.x) * c.y
                - a.x * (2.0 * a.y + b.y + c.y));

        let zx = off_scaling
            * (-a.z * (b.x + c.x)
                - b.z * (2.0 * b.x + c.x)
                - (b.x + 2.0 * c.x) * c.z
                - a.x * (2.0 * a.z + b.z + c.z));

        let zy = off_scaling
            * (-a.z * (b.y + c.y)
                - b.z * (2.0 * b.y + c.y)
                - (b.y + 2.0 * c.y) * c.z
                - a.y * (2.0 * a.z + b.z + c.z));

        Symmetric3x3 {
            xx,
            yx,
            yy,
            zx,
            zy,
            zz,
        }
    }

    /// Computes the area of a triangle.
    #[inline(always)]
    pub fn compute_triangle_area(a: Vec3, b: Vec3, c: Vec3) -> f32 {
        0.5 * (b - a).cross(c - a).length()
    }

    /// Integrates the inertia contribution from a single triangle assuming a density of 1.
    /// Returns (area, inertia_tensor).
    pub fn compute_triangle_contribution_with_area(
        a: Vec3,
        b: Vec3,
        c: Vec3,
    ) -> (f32, Symmetric3x3) {
        let area = Self::compute_triangle_area(a, b, c);
        let inertia = Self::compute_triangle_contribution(a, b, c, area);
        (area, inertia)
    }

    /// Computes the inertia of an open mesh, treating it as a triangle soup.
    pub fn compute_open_inertia(
        triangle_source: &mut dyn ITriangleSource,
        mass: f32,
    ) -> Symmetric3x3 {
        let mut area = 0.0f32;
        let mut inertia = Symmetric3x3::default();
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let (t_area, t_contribution) =
                Self::compute_triangle_contribution_with_area(a, b, c);
            area += t_area;
            inertia = inertia + t_contribution;
        }
        let mut result = Symmetric3x3::default();
        Symmetric3x3::scale(&inertia, mass / area, &mut result);
        result
    }

    /// Computes the inertia of an open mesh with center of mass.
    pub fn compute_open_inertia_with_center(
        triangle_source: &mut dyn ITriangleSource,
        mass: f32,
    ) -> (Symmetric3x3, Vec3) {
        let mut center = Vec3::ZERO;
        let mut area = 0.0f32;
        let mut inertia = Symmetric3x3::default();
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let (t_area, t_contribution) =
                Self::compute_triangle_contribution_with_area(a, b, c);
            area += t_area;
            center += t_area * (a + b + c);
            inertia = inertia + t_contribution;
        }
        let inverse_area = 1.0 / area;
        center *= inverse_area * (1.0 / 3.0);
        let mut result = Symmetric3x3::default();
        Symmetric3x3::scale(&inertia, mass * inverse_area, &mut result);
        (result, center)
    }

    /// Computes the center of mass of an open mesh, treating it as a triangle soup.
    pub fn compute_open_center_of_mass(
        triangle_source: &mut dyn ITriangleSource,
    ) -> Vec3 {
        let mut center = Vec3::ZERO;
        let mut area = 0.0f32;
        while let Some((a, b, c)) = triangle_source.get_next_triangle() {
            let t_area = Self::compute_triangle_area(a, b, c);
            area += t_area;
            center += t_area * (a + b + c);
        }
        center /= area * 3.0;
        center
    }

    /// Computes an offset for an inertia tensor based on an offset frame of reference (parallel axis theorem).
    #[inline(always)]
    pub fn get_inertia_offset(mass: f32, offset: Vec3) -> Symmetric3x3 {
        let squared = offset * offset;
        let diagonal = squared.x + squared.y + squared.z;
        Symmetric3x3 {
            xx: mass * (squared.x - diagonal),
            yx: mass * (offset.x * offset.y),
            yy: mass * (squared.y - diagonal),
            zx: mass * (offset.x * offset.z),
            zy: mass * (offset.y * offset.z),
            zz: mass * (squared.z - diagonal),
        }
    }
}
