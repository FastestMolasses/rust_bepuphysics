use glam::Vec3;
use std::mem::MaybeUninit;
use std::ops::{Add, Mul, Sub};

#[derive(Copy, Clone, Debug)]
#[repr(C, align(16))]
pub struct Matrix3x3 {
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Matrix3x3 {
    #[inline(always)]
    pub fn identity() -> Self {
        Self {
            x: Vec3::new(1.0, 0.0, 0.0),
            y: Vec3::new(0.0, 1.0, 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        }
    }

    #[inline(always)]
    pub fn add(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
        }
    }

    #[inline(always)]
    pub fn scale(&self, scale: f32) -> Self {
        Self {
            x: self.x * scale,
            y: self.y * scale,
            z: self.z * scale,
        }
    }

    #[inline(always)]
    pub fn subtract(a: &Self, b: &Self) -> Self {
        Self {
            x: a.x - b.x,
            y: a.y - b.y,
            z: a.z - b.z,
        }
    }

    #[inline(always)]
    pub unsafe fn transpose(m: *const Self, transposed: *mut Self) {
        let m = &*(m as *const [f32; 9]);
        let transposed = &mut *(transposed as *mut [f32; 9]);

        let m12 = m[1];
        let m13 = m[2];
        let m23 = m[5];
        transposed[0] = m[0];
        transposed[1] = m[3];
        transposed[2] = m[6];

        transposed[3] = m12;
        transposed[4] = m[4];
        transposed[5] = m[7];

        transposed[6] = m13;
        transposed[7] = m23;
        transposed[8] = m[8];
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
    pub fn determinant(&self) -> f32 {
        self.x.dot(self.y.cross(self.z))
    }

    #[inline(always)]
    pub fn invert(&self) -> Self {
        let yz = self.y.cross(self.z);
        let zx = self.z.cross(self.x);
        let xy = self.x.cross(self.y);
        let inverse_determinant = 1.0 / self.x.dot(yz);

        let mut result = Self {
            x: yz * inverse_determinant,
            y: zx * inverse_determinant,
            z: xy * inverse_determinant,
        };
        result.transpose_new()
    }

    #[inline(always)]
    pub unsafe fn invert_ptr(m: *const Self, inverse: *mut Self) {
        let m = &*(m as *const [f32; 9]);
        let inverse = &mut *(inverse as *mut [f32; 9]);

        let m11 = m[4] * m[8] - m[7] * m[5];
        let m21 = m[5] * m[6] - m[8] * m[3];
        let m31 = m[3] * m[7] - m[6] * m[4];
        let determinant_inverse = 1.0 / (m11 * m[0] + m21 * m[1] + m31 * m[2]);

        let m12 = m[7] * m[2] - m[1] * m[8];
        let m22 = m[8] * m[0] - m[2] * m[6];
        let m32 = m[6] * m[1] - m[0] * m[7];

        let m13 = m[1] * m[5] - m[4] * m[2];
        let m23 = m[2] * m[3] - m[5] * m[0];
        let m33 = m[0] * m[4] - m[3] * m[1];

        inverse[0] = m11 * determinant_inverse;
        inverse[3] = m21 * determinant_inverse;
        inverse[6] = m31 * determinant_inverse;

        inverse[1] = m12 * determinant_inverse;
        inverse[4] = m22 * determinant_inverse;
        inverse[7] = m32 * determinant_inverse;

        inverse[2] = m13 * determinant_inverse;
        inverse[5] = m23 * determinant_inverse;
        inverse[8] = m33 * determinant_inverse;
    }

    #[inline(always)]
    pub fn transform(&self, v: Vec3) -> Vec3 {
        self.x * v.0.extract(0) + self.y * v.0.extract(1) + self.z * v.0.extract(2)
    }

    #[inline(always)]
    pub fn transform_transpose(&self, v: Vec3) -> Vec3 {
        Vec3::new(v.dot(self.x), v.dot(self.y), v.dot(self.z))
    }

    #[inline(always)]
    pub fn multiply(&self, other: &Self) -> Self {
        let bx = other.x.0;
        let by = other.y.0;
        let bz = other.z.0;

        unsafe {
            let mut result: Self = MaybeUninit::uninit().assume_init();

            let x = _mm_set1_ps(self.x.0.extract(0));
            let y = _mm_set1_ps(self.x.0.extract(1));
            let z = _mm_set1_ps(self.x.0.extract(2));
            result.x.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            let x = _mm_set1_ps(self.y.0.extract(0));
            let y = _mm_set1_ps(self.y.0.extract(1));
            let z = _mm_set1_ps(self.y.0.extract(2));
            result.y.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            let x = _mm_set1_ps(self.z.0.extract(0));
            let y = _mm_set1_ps(self.z.0.extract(1));
            let z = _mm_set1_ps(self.z.0.extract(2));
            result.z.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            result
        }
    }

    #[inline(always)]
    pub fn multiply_transposed(&self, other: &Self) -> Self {
        let bx = other.x.0;
        let by = other.y.0;
        let bz = other.z.0;

        unsafe {
            let mut result: Self = MaybeUninit::uninit().assume_init();

            let x = _mm_set1_ps(self.x.0.extract(0));
            let y = _mm_set1_ps(self.y.0.extract(0));
            let z = _mm_set1_ps(self.z.0.extract(0));
            result.x.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            let x = _mm_set1_ps(self.x.0.extract(1));
            let y = _mm_set1_ps(self.y.0.extract(1));
            let z = _mm_set1_ps(self.z.0.extract(1));
            result.y.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            let x = _mm_set1_ps(self.x.0.extract(2));
            let y = _mm_set1_ps(self.y.0.extract(2));
            let z = _mm_set1_ps(self.z.0.extract(2));
            result.z.0 = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(x, bx), _mm_mul_ps(y, by)),
                _mm_mul_ps(z, bz),
            );

            result
        }
    }

    #[inline(always)]
    pub fn create_from_quaternion(q: &Quaternion) -> Self {
        let qx2 = q.x + q.x;
        let qy2 = q.y + q.y;
        let qz2 = q.z + q.z;
        let xx = qx2 * q.x;
        let yy = qy2 * q.y;
        let zz = qz2 * q.z;
        let xy = qx2 * q.y;
        let xz = qx2 * q.z;
        let xw = qx2 * q.w;
        let yz = qy2 * q.z;
        let yw = qy2 * q.w;
        let zw = qz2 * q.w;

        Self {
            x: Vec3::new(1.0 - yy - zz, xy + zw, xz - yw),
            y: Vec3::new(xy - zw, 1.0 - xx - zz, yz + xw),
            z: Vec3::new(xz + yw, yz - xw, 1.0 - xx - yy),
        }
    }

    #[inline(always)]
    pub fn create_scale(scale: Vec3) -> Self {
        Self {
            x: Vec3::new(scale.0.extract(0), 0.0, 0.0),
            y: Vec3::new(0.0, scale.0.extract(1), 0.0),
            z: Vec3::new(0.0, 0.0, scale.0.extract(2)),
        }
    }

    #[inline(always)]
    pub fn create_from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let xx = axis.0.extract(0) * axis.0.extract(0);
        let yy = axis.0.extract(1) * axis.0.extract(1);
        let zz = axis.0.extract(2) * axis.0.extract(2);
        let xy = axis.0.extract(0) * axis.0.extract(1);
        let xz = axis.0.extract(0) * axis.0.extract(2);
        let yz = axis.0.extract(1) * axis.0.extract(2);

        let sin_angle = angle.sin();
        let one_minus_cos_angle = 1.0 - angle.cos();

        Self {
            x: Vec3::new(
                1.0 + one_minus_cos_angle * (xx - 1.0),
                axis.0.extract(2) * sin_angle + one_minus_cos_angle * xy,
                -axis.0.extract(1) * sin_angle + one_minus_cos_angle * xz,
            ),
            y: Vec3::new(
                -axis.0.extract(2) * sin_angle + one_minus_cos_angle * xy,
                1.0 + one_minus_cos_angle * (yy - 1.0),
                axis.0.extract(0) * sin_angle + one_minus_cos_angle * yz,
            ),
            z: Vec3::new(
                axis.0.extract(1) * sin_angle + one_minus_cos_angle * xz,
                -axis.0.extract(0) * sin_angle + one_minus_cos_angle * yz,
                1.0 + one_minus_cos_angle * (zz - 1.0),
            ),
        }
    }

    #[inline(always)]
    pub fn create_cross_product(v: Vec3) -> Self {
        Self {
            x: Vec3::new(0.0, -v.0.extract(2), v.0.extract(1)),
            y: Vec3::new(v.0.extract(2), 0.0, -v.0.extract(0)),
            z: Vec3::new(-v.0.extract(1), v.0.extract(0), 0.0),
        }
    }
}

impl Add for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        Self::add(&self, &other)
    }
}

impl Sub for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        Self::subtract(&self, &other)
    }
}

impl Mul for Matrix3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self::Output {
        self.multiply(&other)
    }
}
