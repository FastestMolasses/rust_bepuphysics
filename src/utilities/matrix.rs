use crate::{out, out_unsafe, utilities::matrix3x3::Matrix3x3};
use glam::{Quat, Vec3, Vec4};
use std::ops::Mul;

/// Provides SIMD-aware 4x4 matrix math.
/// All functions assume row vectors.
#[derive(Copy, Clone, Debug)]
#[repr(C, align(16))]
pub struct Matrix {
    /// Row 1 of the matrix.
    pub x: Vec4,
    /// Row 2 of the matrix.
    pub y: Vec4,
    /// Row 3 of the matrix.
    pub z: Vec4,
    /// Row 4 of the matrix.
    pub w: Vec4,
}

/// A representation of a 4x4 matrix that will be mapped linearly in memory.
#[repr(C)]
struct M {
    m11: f32,
    m12: f32,
    m13: f32,
    m14: f32,
    m21: f32,
    m22: f32,
    m23: f32,
    m24: f32,
    m31: f32,
    m32: f32,
    m33: f32,
    m34: f32,
    m41: f32,
    m42: f32,
    m43: f32,
    m44: f32,
}

impl Matrix {
    #[inline(always)]
    pub const fn identity() -> Self {
        Self {
            x: Vec4::new(1.0, 0.0, 0.0, 0.0),
            y: Vec4::new(0.0, 1.0, 0.0, 0.0),
            z: Vec4::new(0.0, 0.0, 1.0, 0.0),
            w: Vec4::new(0.0, 0.0, 0.0, 1.0),
        }
    }

    #[inline(always)]
    pub fn get_translation(&self) -> Vec3 {
        Vec3::new(self.w.x, self.w.y, self.w.z)
    }

    #[inline(always)]
    pub fn set_translation(&mut self, value: &Vec3) {
        self.w = Vec4::new(value.x, value.y, value.z, self.w.w);
    }

    #[inline(always)]
    pub unsafe fn transpose_m_to(m: *const M, transposed: *mut M) {
        // From original code:
        // A weird function! Why?
        // 1) Missing some helpful instructions for actual SIMD accelerated transposition.
        // 2) Difficult to get SIMD types to generate competitive codegen due to lots of componentwise access.

        let m = &*m;
        let transposed = &mut *transposed;

        let m12 = m.m12;
        let m13 = m.m13;
        let m14 = m.m14;
        let m23 = m.m23;
        let m24 = m.m24;
        let m34 = m.m34;
        transposed.m11 = m.m11;
        transposed.m12 = m.m21;
        transposed.m13 = m.m31;
        transposed.m14 = m.m41;

        transposed.m21 = m12;
        transposed.m22 = m.m22;
        transposed.m23 = m.m32;
        transposed.m24 = m.m42;

        transposed.m31 = m13;
        transposed.m32 = m23;
        transposed.m33 = m.m33;
        transposed.m34 = m.m43;

        transposed.m41 = m14;
        transposed.m42 = m24;
        transposed.m43 = m34;
        transposed.m44 = m.m44;
    }

    #[inline(always)]
    pub unsafe fn transpose_m(m: *const Self, transposed: *mut Self) {
        Self::transpose_m_to(m as *const M, transposed as *mut M);
    }

    pub unsafe fn transpose_to(m: &Self, transposed: &mut Self) {
        // Not an ideal implementation. Shuffles would be handy.
        let xy = m.x.y;
        let xz = m.x.z;
        let xw = m.x.w;
        let yz = m.y.z;
        let yw = m.y.w;
        let zw = m.z.w;
        transposed.x = Vec4::new(m.x.x, m.y.x, m.z.x, m.w.x);
        transposed.y = Vec4::new(xy, m.y.y, m.z.y, m.w.y);
        transposed.z = Vec4::new(xz, yz, m.z.z, m.w.z);
        transposed.w = Vec4::new(xw, yw, zw, m.w.w);
    }

    #[inline(always)]
    pub fn transpose(m: Self) -> Self {
        out_unsafe!(Self::transpose_to(&m))
    }

    /// Transforms a vector with a transposed matrix.
    #[inline(always)]
    pub fn transform_transpose(v: &Vec4, m: &Self, result: &mut Vec4) {
        *result = Vec4::new(v.dot(m.x), v.dot(m.y), v.dot(m.z), v.dot(m.w));
    }

    /// Transforms a vector with a matrix.
    #[inline(always)]
    pub fn transform(v: &Vec4, m: &Self, result: &mut Vec4) {
        let x = Vec4::splat(v.x);
        let y = Vec4::splat(v.y);
        let z = Vec4::splat(v.z);
        let w = Vec4::splat(v.w);
        *result = m.x * x + m.y * y + m.z * z + m.w * w;
    }

    /// Transforms a vector with a matrix. Implicitly uses 1 as the fourth component of the input vector.
    #[inline(always)]
    pub fn transform_vec3(v: Vec3, m: &Self, result: &mut Vec4) {
        let x = Vec4::splat(v.x);
        let y = Vec4::splat(v.y);
        let z = Vec4::splat(v.z);
        *result = m.x * x + m.y * y + m.z * z + m.w;
    }

    /// Multiplies a matrix by another matrix.
    #[inline(always)]
    pub fn multiply(a: &Self, b: &Self, result: &mut Self) {
        let b_x = b.x;
        let b_y = b.y;
        let b_z = b.z;

        {
            let x = Vec4::splat(a.x.x);
            let y = Vec4::splat(a.x.y);
            let z = Vec4::splat(a.x.z);
            let w = Vec4::splat(a.x.w);
            result.x = (x * b_x + y * b_y) + (z * b_z + w * b.w);
        }

        {
            let x = Vec4::splat(a.y.x);
            let y = Vec4::splat(a.y.y);
            let z = Vec4::splat(a.y.z);
            let w = Vec4::splat(a.y.w);
            result.y = (x * b_x + y * b_y) + (z * b_z + w * b.w);
        }

        {
            let x = Vec4::splat(a.z.x);
            let y = Vec4::splat(a.z.y);
            let z = Vec4::splat(a.z.z);
            let w = Vec4::splat(a.z.w);
            result.z = (x * b_x + y * b_y) + (z * b_z + w * b.w);
        }

        {
            let x = Vec4::splat(a.w.x);
            let y = Vec4::splat(a.w.y);
            let z = Vec4::splat(a.w.z);
            let w = Vec4::splat(a.w.w);
            result.w = (x * b_x + y * b_y) + (z * b_z + w * b.w);
        }
    }

    #[inline(always)]
    pub fn create_from_axis_angle(axis: &Vec3, angle: f32, result: &mut Self) {
        // TODO: Could be better simdified.
        let xx = axis.x * axis.x;
        let yy = axis.y * axis.y;
        let zz = axis.z * axis.z;
        let xy = axis.x * axis.y;
        let xz = axis.x * axis.z;
        let yz = axis.y * axis.z;

        let sin_angle = angle.sin();
        let one_minus_cos_angle = 1.0 - angle.cos();

        result.x = Vec4::new(
            1.0 + one_minus_cos_angle * (xx - 1.0),
            axis.z * sin_angle + one_minus_cos_angle * xy,
            -axis.y * sin_angle + one_minus_cos_angle * xz,
            0.0,
        );
        result.y = Vec4::new(
            -axis.z * sin_angle + one_minus_cos_angle * xy,
            1.0 + one_minus_cos_angle * (yy - 1.0),
            axis.x * sin_angle + one_minus_cos_angle * yz,
            0.0,
        );
        result.z = Vec4::new(
            axis.y * sin_angle + one_minus_cos_angle * xz,
            -axis.x * sin_angle + one_minus_cos_angle * yz,
            1.0 + one_minus_cos_angle * (zz - 1.0),
            0.0,
        );
        result.w = Vec4::new(0.0, 0.0, 0.0, 1.0);
    }

    #[inline(always)]
    pub fn create_value_from_axis_angle(axis: &Vec3, angle: f32, result: &mut Self) -> Self {
        let result;
        Self::create_from_axis_angle(axis, angle, result);
        return result;
    }

    #[inline(always)]
    pub fn create_from_quaternion(quaternion: &Quat, result: &mut Self) {
        let qx2 = quaternion.x + quaternion.x;
        let qy2 = quaternion.y + quaternion.y;
        let qz2 = quaternion.z + quaternion.z;
        let xx = qx2 * quaternion.x;
        let yy = qy2 * quaternion.y;
        let zz = qz2 * quaternion.z;
        let xy = qx2 * quaternion.y;
        let xz = qx2 * quaternion.z;
        let xw = qx2 * quaternion.w;
        let yz = qy2 * quaternion.z;
        let yw = qy2 * quaternion.w;
        let zw = qz2 * quaternion.w;

        result.x = Vec4::new(1.0 - yy - zz, xy + zw, xz - yw, 0.0);
        result.y = Vec4::new(xy - zw, 1.0 - xx - zz, yz + xw, 0.0);
        result.z = Vec4::new(xz + yw, yz - xw, 1.0 - xx - yy, 0.0);
        result.w = Vec4::new(0.0, 0.0, 0.0, 1.0);
    }

    pub fn create_value_from_quaternion(quaternion: &Quat) -> Self {
        let result;
        Self::create_from_quaternion(quaternion, result);
        return result;
    }

    /// Creates a right-handed perspective matrix.
    #[inline(always)]
    pub fn create_perspective_fov(
        field_of_view: f32,
        aspect_ratio: f32,
        near_clip: f32,
        far_clip: f32,
        perspective: &mut Self,
    ) {
        let h = 1.0 / (field_of_view * 0.5).tan();
        let w = h / aspect_ratio;
        let m33 = far_clip / (near_clip - far_clip);

        perspective.x = Vec4::new(w, 0.0, 0.0, 0.0);
        perspective.y = Vec4::new(0.0, h, 0.0, 0.0);
        perspective.z = Vec4::new(0.0, 0.0, m33, -1.0);
        perspective.w = Vec4::new(0.0, 0.0, near_clip * m33, 0.0);
    }

    /// Creates a right-handed perspective matrix.
    #[inline(always)]
    pub fn create_value_perspective_fov(
        field_of_view: f32,
        aspect_ratio: f32,
        near_clip: f32,
        far_clip: f32,
    ) {
        let perspective;
        Self::create_perspective_fov(
            field_of_view,
            aspect_ratio,
            near_clip,
            far_clip,
            perspective,
        );
        return perspective;
    }

    /// Creates a left-handed perspective matrix.
    #[inline(always)]
    pub fn create_perspective_fov_lh(
        field_of_view: f32,
        aspect_ratio: f32,
        near_clip: f32,
        far_clip: f32,
        perspective: &mut Self,
    ) {
        let h = 1.0 / (field_of_view * 0.5).tan();
        let w = h / aspect_ratio;
        let m33 = far_clip / (far_clip - near_clip);
        perspective.x = Vec4::new(w, 0.0, 0.0, 0.0);
        perspective.y = Vec4::new(0.0, h, 0.0, 0.0);
        perspective.z = Vec4::new(0.0, 0.0, m33, 1.0);
        perspective.w = Vec4::new(0.0, 0.0, -near_clip * m33, 0.0);
    }

    /// Creates a right-handed perspective matrix.
    #[inline(always)]
    pub fn create_perspective_from_fovs(
        vertical_fov: f32,
        horizontal_fov: f32,
        near_clip: f32,
        far_clip: f32,
        perspective: &mut Self,
    ) {
        let h = 1.0 / (vertical_fov * 0.5).tan();
        let w = 1.0 / (horizontal_fov * 0.5).tan();
        let m33: f32 = far_clip / (near_clip - far_clip);
        perspective.x = Vec4::new(w, 0.0, 0.0, 0.0);
        perspective.y = Vec4::new(0.0, h, 0.0, 0.0);
        perspective.z = Vec4::new(0.0, 0.0, m33, -1.0);
        perspective.w = Vec4::new(0.0, 0.0, near_clip * m33, 0.0);
    }

    /// Creates a right-handed perspective matrix.
    #[inline(always)]
    pub fn create_value_perspective_from_fovs(
        vertical_fov: f32,
        horizontal_fov: f32,
        near_clip: f32,
        far_clip: f32,
    ) -> Self {
        out!(Self::create_perspective_from_fovs(
            vertical_fov,
            horizontal_fov,
            near_clip,
            far_clip
        ))
    }

    /// Creates a right handed orthographic projection.
    #[inline(always)]
    pub fn create_orthographic(
        left: f32,
        right: f32,
        bottom: f32,
        top: f32,
        z_near: f32,
        z_far: f32,
        projection: &mut Self,
    ) {
        let width = right - left;
        let height = top - bottom;
        let depth = z_far - z_near;
        projection.x = Vec4::new(2.0 / width, 0.0, 0.0, 0.0);
        projection.y = Vec4::new(0.0, 2.0 / height, 0.0, 0.0);
        projection.z = Vec4::new(0.0, 0.0, -1.0 / depth, 0.0);
        projection.w = Vec4::new(
            (left + right) / -width,
            (top + bottom) / -height,
            z_near / -depth,
            1.0,
        );
    }

    /// Inverts the matrix.
    #[inline(always)]
    pub fn invert(m: &Self, inverted: &mut Self) {
        // From original code:
        // TODO: This could be quite a bit faster, especially once shuffles exist... But inverting a 4x4 matrix should approximately never occur.
        let s0 = m.x.x * m.y.y - m.y.x * m.x.y;
        let s1 = m.x.x * m.y.z - m.y.x * m.x.z;
        let s2 = m.x.x * m.y.w - m.y.x * m.x.w;
        let s3 = m.x.y * m.y.z - m.y.y * m.x.z;
        let s4 = m.x.y * m.y.w - m.y.y * m.x.w;
        let s5 = m.x.z * m.y.w - m.y.z * m.x.w;

        let c5 = m.z.z * m.w.w - m.w.z * m.z.w;
        let c4 = m.z.y * m.w.w - m.w.y * m.z.w;
        let c3 = m.z.y * m.w.z - m.w.y * m.z.z;
        let c2 = m.z.x * m.w.w - m.w.x * m.z.w;
        let c1 = m.z.x * m.w.z - m.w.x * m.z.z;
        let c0 = m.z.x * m.w.y - m.w.x * m.z.y;

        let inverse_determinant = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

        let m11 = m.x.x;
        let m12 = m.x.y;
        let m13 = m.x.z;
        let m14 = m.x.w;
        let m21 = m.y.x;
        let m22 = m.y.y;
        let m23 = m.y.z;
        let m31 = m.z.x;
        let m32 = m.z.y;
        let m33 = m.z.z;

        let m41 = m.W.X;
        let m42 = m.W.Y;

        inverted.x = inverse_determinant
            * Vec4::new(
                m.y.y * c5 - m.y.z * c4 + m.y.w * c3,
                -m.x.y * c5 + m.x.z * c4 - m.x.w * c3,
                m.w.y * s5 - m.w.z * s4 + m.w.w * s3,
                -m.z.y * s5 + m.z.z * s4 - m.z.w * s3,
            );
        inverted.y = inverse_determinant
            * Vec4::new(
                -m.y.x * c5 + m.y.z * c2 - m.y.w * c1,
                m11 * c5 - m13 * c2 + m14 * c1,
                -m.w.x * s5 + m.w.z * s2 - m.w.w * s1,
                m.z.x * s5 - m.z.z * s2 + m.z.w * s1,
            );
        inverted.z = inverse_determinant
            * Vec4::new(
                m21 * c4 - m22 * c2 + m.y.w * c0,
                -m11 * c4 + m12 * c2 - m14 * c0,
                m.w.x * s4 - m.w.y * s2 + m.w.w * s0,
                -m31 * s4 + m32 * s2 - m.z.w * s0,
            );
        inverted.w = inverse_determinant
            * Vec4::new(
                -m21 * c3 + m22 * c1 - m23 * c0,
                m11 * c3 - m12 * c1 + m13 * c0,
                -m41 * s3 + m42 * s1 - m.w.z * s0,
                m31 * s3 - m32 * s1 + m33 * s0,
            );
    }

    /// Inverts the matrix.
    #[inline(always)]
    pub fn invert_to_value(m: &Self) -> Self {
        out!(Self::invert(m))
    }

    /// Creates a view matrix pointing in a direction with a given up vector.
    #[inline(always)]
    pub fn create_view(
        position: &Vec3,
        forward: &Vec3,
        up_vector: &Vec3,
        view_matrix: &mut Self,
    ) -> Matrix {
        let length = forward.length();
        let z = forward / -length;
        let x = up_vector.cross(z).normalize();
        let y = z.cross(x);

        view_matrix.x = Vec4::new(x.x, y.x, z.x, 0.0);
        view_matrix.y = Vec4::new(x.y, y.y, z.y, 0.0);
        view_matrix.z = Vec4::new(x.z, y.z, z.z, 0.0);
        view_matrix.w = Vec4::new(-x.dot(position), -y.dot(position), -z.dot(position), 1.0);
    }

    /// Creates a view matrix pointing in a direction with a given up vector.
    #[inline(always)]
    pub fn create_view_value(position: &Vec3, forward: &Vec3, up_vector: &Vec3) -> Self {
        out!(Self::create_view(position, forward, up_vector))
    }

    /// Creates a view matrix pointing from a position to a target with the given up vector.
    #[inline(always)]
    pub fn create_look_at(
        position: &Vec3,
        target: &Vec3,
        up_vector: &Vec3,
        view_matrix: &mut Self,
    ) {
        Self::create_view(position, target - position, up_vector, view_matrix);
    }

    /// Creates a view matrix pointing from a position to a target with the given up vector.
    #[inline(always)]
    pub fn create_look_at_value(position: &Vec3, target: &Vec3, up_vector: &Vec3) -> Self {
        out!(Self::create_look_at(position, target, up_vector))
    }

    /// Creates a rigid world matrix from a rotation matrix and position.
    #[inline(always)]
    pub fn create_rigid(rotation: &Matrix3x3, position: &Vec3, world: &mut Self) {
        world.x = Vec4::new(rotation.x.x, rotation.x.y, rotation.x.z, 0.0);
        world.y = Vec4::new(rotation.y.x, rotation.y.y, rotation.y.z, 0.0);
        world.z = Vec4::new(rotation.z.x, rotation.z.y, rotation.z.z, 0.0);
        world.w = Vec4::new(position.x, position.y, position.z, 1.0);
    }

    /// Creates a rigid world matrix from a rotation quaternion and position.
    #[inline(always)]
    pub fn create_rigid_from_quat(rotation: &Quat, position: &Vec3, world: &mut Self) -> Matrix {
        let rotation_matrix = Matrix3x3::create_new_from_quaternion(rotation);
        world.x = Vec4::new(
            rotation_matrix.x.x,
            rotation_matrix.x.y,
            rotation_matrix.x.z,
            0.0,
        );
        world.y = Vec4::new(
            rotation_matrix.y.x,
            rotation_matrix.y.y,
            rotation_matrix.y.z,
            0.0,
        );
        world.z = Vec4::new(
            rotation_matrix.z.x,
            rotation_matrix.z.y,
            rotation_matrix.z.z,
            0.0,
        );
        world.w = Vec4::new(position.x, position.y, position.z, 1.0);
    }

    /// Creates a 4x4 matrix from a 3x3 matrix. All extra columns and rows filled with 0 except the W.W, which is set to 1.
    #[inline(always)]
    pub fn create_from_3x3(matrix3x3: &Matrix3x3, matrix4x4: &mut Self) -> Matrix {
        matrix4x4.x = Vec4::new(matrix3x3.x.x, matrix3x3.x.y, matrix3x3.x.z, 0.0);
        matrix4x4.y = Vec4::new(matrix3x3.y.x, matrix3x3.y.y, matrix3x3.y.z, 0.0);
        matrix4x4.z = Vec4::new(matrix3x3.z.x, matrix3x3.z.y, matrix3x3.z.z, 0.0);
        matrix4x4.w = Vec4::new(0.0, 0.0, 0.0, 1.0);
    }

    /// Creates a 4x4 matrix from a 3x3 matrix. All extra columns and rows filled with 0 except the W.W, which is set to 1.
    #[inline(always)]
    pub fn create_value_from_3x3(matrix3x3: &Matrix3x3) -> Self {
        out!(Self::create_from_3x3(matrix3x3))
    }
}

impl Mul for Matrix {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        out!(Self::multiply(&self, &other))
    }
}
