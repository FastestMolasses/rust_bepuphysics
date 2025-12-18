// Translated from BepuPhysics/CollisionDetection/ISupportFinder.cs

use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// Interface for support finders used in GJK and related algorithms.
/// Computes support points for shapes in local and world space.
pub trait ISupportFinder<TShapeWide> {
    /// Gets whether the support finder is sampling a shape with a spherical margin.
    fn has_margin() -> bool;

    /// Returns the margin of the shape.
    fn get_margin(shape: &TShapeWide, margin: &mut Vector<f32>);

    /// Computes the support point in local space.
    fn compute_local_support(
        shape: &TShapeWide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    );

    /// Computes the support point in world space given an orientation.
    fn compute_support(
        shape: &TShapeWide,
        orientation: &crate::utilities::matrix3x3_wide::Matrix3x3Wide,
        direction: &Vector3Wide,
        terminated_lanes: &Vector<i32>,
        support: &mut Vector3Wide,
    );
}
