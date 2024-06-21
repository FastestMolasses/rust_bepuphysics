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
   pub fn identity() -> Self {
       Self {
           linear_transform: Matrix3x3::IDENTITY,
           translation: Vec3::ZERO,
       }
   }

   /// Constructs a new affine transform.
   #[inline(always)]
   pub fn from_translation(translation: Vec3) -> Self {
       Self {
           linear_transform: Matrix3x3::IDENTITY,
           translation,
       }
   }

   /// Constructs a new affine transform.
   pub fn from_rotation_translation(orientation: Quat, translation: Vec3) -> Self {
       Self {
           linear_transform: Matrix3x3::from_quat(orientation),
           translation,
       }
   }

   /// Constructs a new affine transform.
   pub fn from_scale_rotation_translation(
       scaling: Vec3,
       orientation: Quat,
       translation: Vec3,
   ) -> Self {
       Self {
           linear_transform: Matrix3x3::from_scale(scaling) * Matrix3x3::from_quat(orientation),
           translation,
       }
   }

   /// Constructs a new affine transform.
   pub fn from_linear_transform_translation(linear_transform: Matrix3x3, translation: Vec3) -> Self {
       Self {
           linear_transform,
           translation,
       }
   }

   /// Transforms a vector by an affine transform.
   pub fn transform_vector(&self, position: Vec3) -> Vec3 {
       self.linear_transform.mul_vec3(position) + self.translation
   }

   /// Inverts an affine transform.
   pub fn invert(&self) -> Self {
       let inverse_linear_transform = self.linear_transform.inverse();
       let inverse_translation = -inverse_linear_transform.mul_vec3(self.translation);
       Self {
           linear_transform: inverse_linear_transform,
           translation: inverse_translation,
       }
   }

   /// Inverts a rigid transform.
   pub fn invert_rigid(&self) -> Self {
       let inverse_linear_transform = self.linear_transform.transpose();
       let inverse_translation = -inverse_linear_transform.mul_vec3(self.translation);
       Self {
           linear_transform: inverse_linear_transform,
           translation: inverse_translation,
       }
   }

   /// Multiplies a transform by another transform.
   pub fn multiply(&self, other: &Self) -> Self {
       let translation = other.linear_transform.mul_vec3(self.translation) + other.translation;
       let linear_transform = self.linear_transform * other.linear_transform;
       Self {
           linear_transform,
           translation,
       }
   }
}
