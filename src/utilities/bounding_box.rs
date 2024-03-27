use glam::{Vec3, Vec4};
use std::mem::MaybeUninit;

/// Represents a bounding box as two `Vec4` values to avoid complexity associated with a `Vec3`'s empty SIMD lane.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BoundingBox4 {
    /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
    pub min: Vec4,
    /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
    pub max: Vec4,
}

impl BoundingBox4 {
    /// Creates a string representation of the bounding box.
    pub fn to_string(&self) -> String {
        format!("({}, {})", self.min.xyz(), self.max.xyz())
    }
}

/// Provides simple axis-aligned bounding box functionality.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BoundingBox {
    /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub min: Vec3,
    /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box.
    pub max: Vec3,
}

impl BoundingBox {
    /// Constructs a bounding box from the specified minimum and maximum.
    #[inline]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Checks if two structures with memory layouts equivalent to the `BoundingBox` intersect.
    /// The referenced values must not be in unpinned managed memory.
    #[inline]
    pub unsafe fn intersects_unsafe<T, U>(a: &T, b: &U) -> bool
    where
        T: Unpin,
        U: Unpin,
    {
        debug_assert!(std::mem::size_of::<T>() == 32 && std::mem::size_of::<U>() == 32);
        let a_min = Vec4::from_slice(&(a as *const T as *const [f32; 4]));
        let a_max = Vec4::from_slice(&(a as *const T as *const [f32; 4]).offset(1));
        let b_min = Vec4::from_slice(&(b as *const U as *const [f32; 4]));
        let b_max = Vec4::from_slice(&(b as *const U as *const [f32; 4]).offset(1));
        let no_intersection_on_axes = a_max.cmplt(b_min) | b_max.cmplt(a_min);
        no_intersection_on_axes.extract(0)
            & no_intersection_on_axes.extract(1)
            & no_intersection_on_axes.extract(2)
            == 0
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline]
    pub fn intersects(a: Self, b: Self) -> bool {
        unsafe { Self::intersects_unsafe(&a, &b) }
    }

    /// Determines if a bounding box intersects another bounding box.
    #[inline]
    pub fn intersects_bounds(min_a: Vec3, max_a: Vec3, min_b: Vec3, max_b: Vec3) -> bool {
        let no_intersection_on_axes = max_a.cmplt(min_b) | max_b.cmplt(min_a);
        no_intersection_on_axes.x & no_intersection_on_axes.y & no_intersection_on_axes.z == 0
    }

    /// Computes the volume of the bounding box.
    #[inline]
    pub fn compute_volume(&self) -> f32 {
        let diagonal = self.max - self.min;
        diagonal.x * diagonal.y * diagonal.z
    }

    /// Computes a bounding box which contains two other bounding boxes.
    #[inline]
    pub fn create_merged(min_a: Vec3, max_a: Vec3, min_b: Vec3, max_b: Vec3) -> (Vec3, Vec3) {
        (min_a.min(min_b), max_a.max(max_b))
    }

    /// Computes a bounding box which contains two other bounding boxes.
    #[inline]
    pub fn create_merged_boxes(a: Self, b: Self) -> Self {
        let (min, max) = Self::create_merged(a.min, a.max, b.min, b.max);
        Self { min, max }
    }

    /// Merges two structures with memory layouts equivalent to the `BoundingBox`.
    /// The referenced values must not be in unpinned managed memory.
    /// Any data in the empty slots is preserved.
    #[inline]
    pub unsafe fn create_merged_unsafe_with_preservation<T, U>(
        a: &T,
        b: &U,
        merged: &mut MaybeUninit<T>,
    ) where
        T: Unpin,
        U: Unpin,
    {
        let min = Vec4::from_slice(&(a as *const T as *const [f32; 4]))
            .min(Vec4::from_slice(&(b as *const U as *const [f32; 4])));
        let max = Vec4::from_slice(&(a as *const T as *const [f32; 4]).offset(1)).max(
            Vec4::from_slice(&(b as *const U as *const [f32; 4]).offset(1)),
        );
        min.write_to_slice(&mut (merged.as_mut_ptr() as *mut [f32; 4]));
        max.write_to_slice(&mut (merged.as_mut_ptr() as *mut [f32; 4]).offset(1));
    }

    /// Merges two structures with memory layouts equivalent to the `BoundingBox`.
    /// The referenced values must not be in unpinned managed memory.
    /// Any data in the empty slots is not preserved.
    #[inline]
    pub unsafe fn create_merged_unsafe<T, U>(a: &T, b: &U, merged: &mut MaybeUninit<T>)
    where
        T: Unpin,
        U: Unpin,
    {
        let min = Vec4::from_slice(&(a as *const T as *const [f32; 4]))
            .min(Vec4::from_slice(&(b as *const U as *const [f32; 4])));
        let max = Vec4::from_slice(&(a as *const T as *const [f32; 4]).offset(1)).max(
            Vec4::from_slice(&(b as *const U as *const [f32; 4]).offset(1)),
        );
        min.write_to_slice(&mut (merged.as_mut_ptr() as *mut [f32; 4]));
        max.write_to_slice(&mut (merged.as_mut_ptr() as *mut [f32; 4]).offset(1));
    }

    /// Determines if a bounding box intersects a bounding sphere.
    #[inline]
    pub fn intersects_sphere(&self, sphere: &BoundingSphere) -> bool {
        let offset = sphere.center - self.min.max(sphere.center).min(self.max);
        offset.dot(offset) <= sphere.radius * sphere.radius
    }

    #[inline]
    pub fn contains(&self, other: &BoundingBox) -> ContainmentType {
        if self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y
            || self.max.z < other.min.z
            || self.min.z > other.max.z
        {
            ContainmentType::Disjoint
        } else if self.min.x <= other.min.x
            && self.max.x >= other.max.x
            && self.min.y <= other.min.y
            && self.max.y >= other.max.y
            && self.min.z <= other.min.z
            && self.max.z >= other.max.z
        {
            ContainmentType::Contains
        } else {
            ContainmentType::Intersects
        }
    }

    /// Creates the smallest possible bounding box that contains a list of points.
    pub fn from_points(points: &[Vec3]) -> Self {
        if points.is_empty() {
            panic!("Cannot construct a bounding box from an empty list.");
        }
        let mut min = points[0];
        let mut max = min;
        for point in points.iter().skip(1) {
            min = min.min(*point);
            max = max.max(*point);
        }
        Self { min, max }
    }

    /// Creates a bounding box from a bounding sphere.
    #[inline]
    pub fn from_sphere(sphere: &BoundingSphere) -> Self {
        let radius = Vec3::splat(sphere.radius);
        Self {
            min: sphere.center - radius,
            max: sphere.center + radius,
        }
    }

    /// Creates a string representation of the bounding box.
    pub fn to_string(&self) -> String {
        format!("({}, {})", self.min, self.max)
    }
}
