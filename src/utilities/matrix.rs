use std::arch::x86_64::*;
use std::mem::MaybeUninit;
use std::ops::{Add, Mul, Sub};
use crate::utilities::vector3::Vector3;
use crate::utilities::vector4::Vector4;
use crate::utilities::matrix3x3::Matrix3x3;
// use crate::utilities::::Quaternion;

#[derive(Copy, Clone, Debug)]
#[repr(C, align(16))]
pub struct Matrix {
    pub x: Vector4,
    pub y: Vector4,
    pub z: Vector4,
    pub w: Vector4,
}

impl Matrix {
    #[inline(always)]
    pub fn identity() -> Self {
        Self {
            x: Vector4::new(1.0, 0.0, 0.0, 0.0),
            y: Vector4::new(0.0, 1.0, 0.0, 0.0),
            z: Vector4::new(0.0, 0.0, 1.0, 0.0),
            w: Vector4::new(0.0, 0.0, 0.0, 1.0),
        }
    }

    #[inline(always)]
    pub fn get_translation(&self) -> Vector3 {
        Vector3::new(self.w.x(), self.w.y(), self.w.z())
    }

    #[inline(always)]
    pub fn set_translation(&mut self, value: Vector3) {
        self.w = Vector4::new(value.x(), value.y(), value.z(), self.w.w());
    }

    #[inline(always)]
    pub unsafe fn transpose(m: *const Self, transposed: *mut Self) {
        let m = &*(m as *const [f32; 16]);
        let transposed = &mut *(transposed as *mut [f32; 16]);
        
        let m12 = m[1];
        let m13 = m[2];
        let m14 = m[3];
        let m23 = m[6];
        let m24 = m[7];
        let m34 = m[11];
        transposed[0] = m[0];
        transposed[1] = m[4];
        transposed[2] = m[8];
        transposed[3] = m[12];

        transposed[4] = m12;
        transposed[5] = m[5];
        transposed[6] = m[9];
        transposed[7] = m[13];

        transposed[8] = m13;
        transposed[9] = m23;
        transposed[10] = m[10];
        transposed[11] = m[14];

        transposed[12] = m14;
        transposed[13] = m24;
        transposed[14] = m34;
        transposed[15] = m[15];
    }

    #[inline(always)]
    pub fn transpose_new(&self) -> Self {
        let mut result = MaybeUninit::uninit();
        unsafe {
            Self::transpose(self, result.as_mut_ptr());
            result.assume_init()
        }
    }

    #[inline(always)]
    pub fn transform_transpose(v: &Vector4, m: &Matrix) -> Vector4 {
        Vector4::new(
            v.dot(m.x),
            v.dot(m.y),
            v.dot(m.z),
            v.dot(m.w)
        )
    }

    #[inline(always)]
    pub fn transform(v: &Vector4, m: &Matrix) -> Vector4 {
        let x = Vector4::splat(v.x());
        let y = Vector4::splat(v.y());
        let z = Vector4::splat(v.z());
        let w = Vector4::splat(v.w());
        m.x * x + m.y * y + m.z * z + m.w * w
    }

    #[inline(always)]
    pub fn transform_vector3(v: Vector3, m: &Matrix) -> Vector4 {
        let x = Vector4::splat(v.x());
        let y = Vector4::splat(v.y());
        let z = Vector4::splat(v.z());
        m.x * x + m.y * y + m.z * z + m.w
    }

    #[inline(always)]
    pub fn multiply(a: &Matrix, b: &Matrix) -> Matrix {
        let bx = b.x.into();
        let by = b.y.into();
        let bz = b.z.into();
        let bw = b.w.into();
        
        unsafe {
            let mut result: Matrix = MaybeUninit::uninit().assume_init();
            
            let x = _mm_set1_ps(a.x.x());
            let y = _mm_set1_ps(a.x.y());
            let z = _mm_set1_ps(a.x.z());
            let w = _mm_set1_ps(a.x.w());
            result.x = Vector4::from(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)), _mm_add_ps(_mm_mul_ps(z, bz), _mm_mul_ps(w, bw))));
            
            let x = _mm_set1_ps(a.y.x());
            let y = _mm_set1_ps(a.y.y());
            let z = _mm_set1_ps(a.y.z());
            let w = _mm_set1_ps(a.y.w());
            result.y = Vector4::from(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)), _mm_add_ps(_mm_mul_ps(z, bz), _mm_mul_ps(w, bw))));
            
            let x = _mm_set1_ps(a.z.x());
            let y = _mm_set1_ps(a.z.y());
            let z = _mm_set1_ps(a.z.z());
            let w = _mm_set1_ps(a.z.w());
            result.z = Vector4::from(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)), _mm_add_ps(_mm_mul_ps(z, bz), _mm_mul_ps(w, bw))));
            
            let x = _mm_set1_ps(a.w.x());
            let y = _mm_set1_ps(a.w.y());
            let z = _mm_set1_ps(a.w.z());
            let w = _mm_set1_ps(a.w.w());
            result.w = Vector4::from(_mm_add_ps(_mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)), _mm_add_ps(_mm_mul_ps(z, bz), _mm_mul_ps(w, bw))));
            
            result
        }
    }

    #[inline(always)]
    pub fn create_from_axis_angle(axis: Vector3, angle: f32) -> Matrix {
        let xx = axis.x() * axis.x();
        let yy = axis.y() * axis.y();
        let zz = axis.z() * axis.z();
        let xy = axis.x() * axis.y();
        let xz = axis.x() * axis.z();
        let yz = axis.y() * axis.z();

        let sin_angle = angle.sin();
        let one_minus_cos_angle = 1.0 - angle.cos();

        Matrix {
            x: Vector4::new(
                1.0 + one_minus_cos_angle * (xx - 1.0),
                axis.z() * sin_angle + one_minus_cos_angle * xy,
                -axis.y() * sin_angle + one_minus_cos_angle * xz,
                0.0
            ),
            y: Vector4::new(
                -axis.z() * sin_angle + one_minus_cos_angle * xy,
                1.0 + one_minus_cos_angle * (yy - 1.0),
                axis.x() * sin_angle + one_minus_cos_angle * yz,
                0.0
            ),
            z: Vector4::new(
                axis.y() * sin_angle + one_minus_cos_angle * xz,
                -axis.x() * sin_angle + one_minus_cos_angle * yz,
                1.0 + one_minus_cos_angle * (zz - 1.0),
                0.0
            ),
            w: Vector4::new(0.0, 0.0, 0.0, 1.0),
        }
    }

    #[inline(always)]
    pub fn create_from_quaternion(quaternion: &Quaternion) -> Matrix {
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

        Matrix {
            x: Vector4::new(
                1.0 - yy - zz,
                xy + zw,
                xz - yw,
                0.0
            ),
            y: Vector4::new(
                xy - zw,
                1.0 - xx - zz,
                yz + xw,
                0.0
            ),
            z: Vector4::new(
                xz + yw,
                yz - xw,
                1.0 - xx - yy,
                0.0
            ),
            w: Vector4::new(0.0, 0.0, 0.0, 1.0),
        }
    }

    #[inline(always)]
    pub fn create_perspective_fov(field_of_view: f32, aspect_ratio: f32, near_clip: f32, far_clip: f32) -> Matrix {
        let h = 1.0 / (field_of_view * 0.5).tan();
        let w = h / aspect_ratio;
        let m33 = far_clip / (near_clip - far_clip);
        Matrix {
            x: Vector4::new(w, 0.0, 0.0, 0.0),
            y: Vector4::new(0.0, h, 0.0, 0.0),
            z: Vector4::new(0.0, 0.0, m33, -1.0),
            w: Vector4::new(0.0, 0.0, near_clip * m33, 0.0),
        }
    }

    #[inline(always)]
    pub fn create_perspective_fov_lh(field_of_view: f32, aspect_ratio: f32, near_clip: f32, far_clip: f32) -> Matrix {
        let h = 1.0 / (field_of_view * 0.5).tan();
        let w = h / aspect_ratio;
        let m33 = far_clip / (far_clip - near_clip);
        Matrix {
            x: Vector4::new(w, 0.0, 0.0, 0.0),
            y: Vector4::new(0.0, h, 0.0, 0.0),
            z: Vector4::new(0.0, 0.0, m33, 1.0),
            w: Vector4::new(0.0, 0.0, -near_clip * m33, 0.0),
        }
    }

    #[inline(always)]
    pub fn create_orthographic(left: f32, right: f32, bottom: f32, top: f32, z_near: f32, z_far: f32) -> Matrix {
        let width = right - left;
        let height = top - bottom;
        let depth = z_far - z_near;
        Matrix {
            x: Vector4::new(2.0 / width, 0.0, 0.0, 0.0),
            y: Vector4::new(0.0, 2.0 / height, 0.0, 0.0),
            z: Vector4::new(0.0, 0.0, -1.0 / depth, 0.0),
            w: Vector4::new((left + right) / -width, (top + bottom) / -height, z_near / -depth, 1.0),
        }
    }

    #[inline(always)]
    pub fn invert(&self) -> Matrix {
        let s0 = self.x.x() * self.y.y() - self.y.x() * self.x.y();
        let s1 = self.x.x() * self.y.z() - self.y.x() * self.x.z();
        let s2 = self.x.x() * self.y.w() - self.y.x() * self.x.w();
        let s3 = self.x.y() * self.y.z() - self.y.y() * self.x.z();
        let s4 = self.x.y() * self.y.w() - self.y.y() * self.x.w();
        let s5 = self.x.z() * self.y.w() - self.y.z() * self.x.w();

        let c5 = self.z.z() * self.w.w() - self.w.z() * self.z.w();
        let c4 = self.z.y() * self.w.w() - self.w.y() * self.z.w();
        let c3 = self.z.y() * self.w.z() - self.w.y() * self.z.z();
        let c2 = self.z.x() * self.w.w() - self.w.x() * self.z.w();
        let c1 = self.z.x() * self.w.z() - self.w.x() * self.z.z();
        let c0 = self.z.x() * self.w.y() - self.w.x() * self.z.y();

        let inverse_determinant = 1.0 / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

        Matrix {
            x: inverse_determinant * Vector4::new(
                self.y.y() * c5 - self.y.z() * c4 + self.y.w() * c3,
                -self.x.y() * c5 + self.x.z() * c4 - self.x.w() * c3,
                self.w.y() * s5 - self.w.z() * s4 + self.w.w() * s3,
                -self.z.y() * s5 + self.z.z() * s4 - self.z.w() * s3
            ),
            y: inverse_determinant * Vector4::new(
                -self.y.x() * c5 + self.y.z() * c2 - self.y.w() * c1,
                self.x.x() * c5 - self.x.z() * c2 + self.x.w() * c1,
                -self.w.x() * s5 + self.w.z() * s2 - self.w.w() * s1,
                self.z.x() * s5 - self.z.z() * s2 + self.z.w() * s1
            ),
            z: inverse_determinant * Vector4::new(
                self.y.x() * c4 - self.y.y() * c2 + self.y.w() * c0,
                -self.x.x() * c4 + self.x.y() * c2 - self.x.w() * c0,
                self.w.x() * s4 - self.w.y() * s2 + self.w.w() * s0,
                -self.z.x() * s4 + self.z.y() * s2 - self.z.w() * s0
            ),
            w: inverse_determinant * Vector4::new(
                -self.y.x() * c3 + self.y.y() * c1 - self.y.z() * c0,
                self.x.x() * c3 - self.x.y() * c1 + self.x.z() * c0,
                -self.w.x() * s3 + self.w.y() * s1 - self.w.z() * s0,
                self.z.x() * s3 - self.z.y() * s1 + self.z.z() * s0
            ),
        }
    }

    #[inline(always)]
    pub fn create_look_at(position: Vector3, target: Vector3, up_vector: Vector3) -> Matrix {
        Self::create_view(position, target - position, up_vector)
    }

    #[inline(always)]
    pub fn create_view(position: Vector3, forward: Vector3, up_vector: Vector3) -> Matrix {
        let length = forward.length();
        let z = forward / -length;
        let x = up_vector.cross(z).normalize();
        let y = z.cross(x);

        Matrix {
            x: Vector4::new(x.x(), y.x(), z.x(), 0.0),
            y: Vector4::new(x.y(), y.y(), z.y(), 0.0),
            z: Vector4::new(x.z(), y.z(), z.z(), 0.0),
            w: Vector4::new(
                -x.dot(position),
                -y.dot(position),
                -z.dot(position),
                1.0
            ),
        }
    }

    #[inline(always)]
    pub fn create_rigid(rotation: &Matrix3x3, position: Vector3) -> Matrix {
        Matrix {
            x: Vector4::new(rotation.x.x(), rotation.x.y(), rotation.x.z(), 0.0),
            y: Vector4::new(rotation.y.x(), rotation.y.y(), rotation.y.z(), 0.0),
            z: Vector4::new(rotation.z.x(), rotation.z.y(), rotation.z.z(), 0.0),
            w: Vector4::new(position.x(), position.y(), position.z(), 1.0),
        }
    }

    #[inline(always)]
    pub fn create_rigid_from_quaternion(rotation: &Quaternion, position: Vector3) -> Matrix {
        let rotation_matrix = Matrix3x3::create_from_quaternion(rotation);
        Self::create_rigid(&rotation_matrix, position)
    }

    #[inline(always)]
    pub fn create_from_3x3(matrix3x3: &Matrix3x3) -> Matrix {
        Matrix {
            x: Vector4::new(matrix3x3.x.x(), matrix3x3.x.y(), matrix3x3.x.z(), 0.0),
            y: Vector4::new(matrix3x3.y.x(), matrix3x3.y.y(), matrix3x3.y.z(), 0.0),
            z: Vector4::new(matrix3x3.z.x(), matrix3x3.z.y(), matrix3x3.z.z(), 0.0),
            w: Vector4::new(0.0, 0.0, 0.0, 1.0),
        }
    }
}

impl Add for Matrix {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        Matrix {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            w: self.w + other.w,
        }
    }
}

impl Sub for Matrix {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        Matrix {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            w: self.w - other.w,
        }
    }
}

impl Mul for Matrix {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        Matrix::multiply(&self, &other)
    }
}

impl Mul<Vector4> for Matrix {
    type Output = Vector4;

    #[inline(always)]
    fn mul(self, v: Vector4) -> Self::Output {
        Matrix::transform(&v, &self)
    }
}

impl Mul<Vector3> for Matrix {
    type Output = Vector4;

    #[inline(always)]
    fn mul(self, v: Vector3) -> Self::Output {
        Matrix::transform_vector3(v, &self)
    }
}
