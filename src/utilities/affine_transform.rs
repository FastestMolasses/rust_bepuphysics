use crate::out;
use crate::utilities::matrix3x3::Matrix3x3;
use glam::{Quat, Vec3};

/// A transformation composed of a linear transformation and a translation.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct AffineTransform {
    /// Translation in the affine transform.
    pub translation: Vec3,
    /// Linear transform in the affine transform.
    pub linear_transform: Matrix3x3,
}

impl AffineTransform {
    /// Gets the identity affine transform.
    #[inline(always)]
    pub const fn identity() -> Self {
        Self {
            linear_transform: Matrix3x3::identity(),
            translation: Vec3::ZERO,
        }
    }

    /// Constructs a new affine transform.
    #[inline(always)]
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            linear_transform: Matrix3x3::identity(),
            translation,
        }
    }

    /// Constructs a new affine transform.
    #[inline(always)]
    pub fn from_rotation_translation(orientation: Quat, translation: Vec3) -> Self {
        Self {
            linear_transform: Matrix3x3::create_new_from_quaternion(&orientation),
            translation,
        }
    }

    /// Constructs a new affine transform.
    #[inline(always)]
    pub fn from_scale_rotation_translation(
        scaling: Vec3,
        orientation: Quat,
        translation: Vec3,
    ) -> Self {
        let linear_transform = Matrix3x3::create_new_scale(&scaling);
        let rotation = Matrix3x3::create_new_from_quaternion(&orientation);
        Self {
            linear_transform: linear_transform * rotation,
            translation,
        }
    }

    /// Constructs a new affine transform.
    #[inline(always)]
    pub fn from_linear_transform_translation(
        linear_transform: Matrix3x3,
        translation: Vec3,
    ) -> Self {
        Self {
            linear_transform,
            translation,
        }
    }

    /// Transforms a vector by an affine transform.
    #[inline(always)]
    pub fn transform(position: Vec3, transform: &Self, transformed: &mut Vec3) {
        Matrix3x3::transform(&position, &transform.linear_transform, transformed);
        *transformed += transform.translation;
    }

    /// Inverts an affine transform.
    #[inline(always)]
    pub fn invert(transform: &Self, inverse: &mut Self) {
        Matrix3x3::invert(&transform.linear_transform, &mut inverse.linear_transform);
        Matrix3x3::transform(
            &transform.translation,
            &inverse.linear_transform,
            &mut inverse.translation,
        );
        inverse.translation = -inverse.translation;
    }

    /// Inverts a rigid transform.
    #[inline(always)]
    pub fn invert_rigid(transform: &Self, inverse: &mut Self) {
        unsafe {
            Matrix3x3::transpose(
                &transform.linear_transform as *const _,
                &mut inverse.linear_transform,
            );
        }
        Matrix3x3::transform(
            &transform.translation,
            &inverse.linear_transform,
            &mut inverse.translation,
        );
        inverse.translation = -inverse.translation;
    }

    /// Multiplies a transform by another transform.
    #[inline(always)]
    pub fn multiply(a: &Self, b: &Self, transform: &mut Self) {
        let translation = out!(Matrix3x3::transform(&a.translation, &b.linear_transform));
        transform.translation = translation + b.translation;
        Matrix3x3::multiply(
            &a.linear_transform,
            &b.linear_transform,
            &mut transform.linear_transform,
        );
    }
}
