use crate::{out, utilities::matrix3x3::Matrix3x3};
use core::ops::{Add, Mul, Sub};
use glam::Vec3;

/// Lower left triangle (including diagonal) of a symmetric 3x3 matrix.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric3x3 {
    /// First row, first column of the matrix.
    pub xx: f32,
    /// Second row, first column of the matrix.
    pub yx: f32,
    /// Second row, second column of the matrix.
    pub yy: f32,
    /// Third row, first column of the matrix.
    pub zx: f32,
    /// Third row, second column of the matrix.
    pub zy: f32,
    /// Third row, third column of the matrix.
    pub zz: f32,
}

impl Symmetric3x3 {
    /// Computes rT * m * r for a symmetric matrix m and a rotation matrix r.
    #[inline(always)]
    pub fn rotation_sandwich(r: &Matrix3x3, m: &Self, sandwich: &mut Self) {
        // TODO: We just copied this from the wide implementation. There are a lot of ways to improve this, should it be necessary.
        // (There's virtually no chance that optimizing this to a serious degree would be worth it- at the time of writing, it's only called by the pose integrator, which is
        // horribly memory bound anyway.)
        let i11 = r.x.x * m.xx + r.y.x * m.yx + r.z.x * m.zx;
        let i12 = r.x.x * m.yx + r.y.x * m.yy + r.z.x * m.zy;
        let i13 = r.x.x * m.zx + r.y.x * m.zy + r.z.x * m.zz;

        let i21 = r.x.y * m.xx + r.y.y * m.yx + r.z.y * m.zx;
        let i22 = r.x.y * m.yx + r.y.y * m.yy + r.z.y * m.zy;
        let i23 = r.x.y * m.zx + r.y.y * m.zy + r.z.y * m.zz;

        let i31 = r.x.z * m.xx + r.y.z * m.yx + r.z.z * m.zx;
        let i32 = r.x.z * m.yx + r.y.z * m.yy + r.z.z * m.zy;
        let i33 = r.x.z * m.zx + r.y.z * m.zy + r.z.z * m.zz;

        sandwich.xx = i11 * r.x.x + i12 * r.y.x + i13 * r.z.x;
        sandwich.yx = i21 * r.x.x + i22 * r.y.x + i23 * r.z.x;
        sandwich.yy = i21 * r.x.y + i22 * r.y.y + i23 * r.z.y;
        sandwich.zx = i31 * r.x.x + i32 * r.y.x + i33 * r.z.x;
        sandwich.zy = i31 * r.x.y + i32 * r.y.y + i33 * r.z.y;
        sandwich.zz = i31 * r.x.z + i32 * r.y.z + i33 * r.z.z;
    }

    /// Computes the determinant of a symmetric matrix.
    #[inline(always)]
    pub fn determinant(m: &Self) -> f32 {
        let m11 = m.yy * m.zz - m.zy * m.zy;
        let m21 = m.zy * m.zx - m.zz * m.yx;
        let m31 = m.yx * m.zy - m.zx * m.yy;
        m11 * m.xx + m21 * m.yx + m31 * m.zx
    }

    /// Inverts the given matix.
    #[inline(always)]
    pub fn invert(m: &Self, inverse: &mut Self) {
        let m11 = m.yy * m.zz - m.zy * m.zy;
        let m21 = m.zy * m.zx - m.zz * m.yx;
        let m31 = m.yx * m.zy - m.zx * m.yy;
        let determinant_inverse = 1.0 / (m11 * m.xx + m21 * m.yx + m31 * m.zx);

        let m22 = m.zz * m.xx - m.zx * m.zx;
        let m32 = m.zx * m.yx - m.xx * m.zy;

        let m33 = m.xx * m.yy - m.yx * m.yx;

        inverse.xx = m11 * determinant_inverse;
        inverse.yx = m21 * determinant_inverse;
        inverse.zx = m31 * determinant_inverse;
        inverse.yy = m22 * determinant_inverse;
        inverse.zy = m32 * determinant_inverse;
        inverse.zz = m33 * determinant_inverse;
    }

    #[inline(always)]
    pub fn scale(m: &Self, scale: f32, scaled: &mut Self) {
        scaled.xx = m.xx * scale;
        scaled.yx = m.yx * scale;
        scaled.yy = m.yy * scale;
        scaled.zx = m.zx * scale;
        scaled.zy = m.zy * scale;
        scaled.zz = m.zz * scale;
    }

    #[inline(always)]
    pub fn multiply_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        let ayxbyx = a.yx * b.yx;
        let azxbzx = a.zx * b.zx;
        let azybzy = a.zy * b.zy;

        result.xx = a.xx * b.xx + ayxbyx + azxbzx;
        result.yx = a.yx * b.xx + a.yy * b.yx + a.zy * b.zx;
        result.yy = ayxbyx + a.yy * b.yy + azybzy;
        result.zx = a.zx * b.xx + a.zy * b.yx + a.zz * b.zx;
        result.zy = a.zx * b.yx + a.zy * b.yy + a.zz * b.zy;
        result.zz = azxbzx + azybzy + a.zz * b.zz;
    }

    /// Multiplies the two matrices.
    #[inline(always)]
    pub fn multiply(a: &Matrix3x3, b: &Symmetric3x3, result: &mut Matrix3x3) {
        let bx = Vec3::new(b.xx, b.yx, b.zx);
        let by = Vec3::new(b.yx, b.yy, b.zy);
        let bz = Vec3::new(b.zx, b.zy, b.zz);
        result.x = {
            let x = Vec3::splat(a.x.x);
            let y = Vec3::splat(a.x.y);
            let z = Vec3::splat(a.x.z);
            x * bx + y * by + z * bz
        };
        result.y = {
            let x = Vec3::splat(a.y.x);
            let y = Vec3::splat(a.y.y);
            let z = Vec3::splat(a.y.z);
            x * bx + y * by + z * bz
        };
        result.z = {
            let x = Vec3::splat(a.z.x);
            let y = Vec3::splat(a.z.y);
            let z = Vec3::splat(a.z.z);
            x * bx + y * by + z * bz
        };
    }

    /// Transforms a vector by a symmetric matrix.
    #[inline(always)]
    pub fn transform_without_overlap(v: &Vec3, m: &Symmetric3x3, result: &mut Vec3) {
        result.x = v.x * m.xx + v.y * m.yx + v.z * m.zx;
        result.y = v.x * m.yx + v.y * m.yy + v.z * m.zy;
        result.z = v.x * m.zx + v.y * m.zy + v.z * m.zz;
    }

    /// Transforms a vector by a symmetric matrix.
    pub fn transform(v: Vec3, m: &Self) -> Vec3 {
        Vec3::new(
            v.x * m.xx + v.y * m.yx + v.z * m.zx,
            v.x * m.yx + v.y * m.yy + v.z * m.zy,
            v.x * m.zx + v.y * m.zy + v.z * m.zz,
        )
    }
}

impl Add for Symmetric3x3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        Self {
            xx: self.xx + other.xx,
            yx: self.yx + other.yx,
            yy: self.yy + other.yy,
            zx: self.zx + other.zx,
            zy: self.zy + other.zy,
            zz: self.zz + other.zz,
        }
    }
}

impl Sub for Symmetric3x3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self {
        Self {
            xx: self.xx - other.xx,
            yx: self.yx - other.yx,
            yy: self.yy - other.yy,
            zx: self.zx - other.zx,
            zy: self.zy - other.zy,
            zz: self.zz - other.zz,
        }
    }
}

impl Mul<f32> for Symmetric3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scale: f32) -> Self {
        out!(Self::scale(&self, scale))
    }
}

impl Mul for Symmetric3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self {
        out!(Self::multiply_without_overlap(&self, &other))
    }
}

impl Add<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn add(self, other: Symmetric3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: self.x + Vec3::new(other.xx, other.yx, other.zx),
            y: self.y + Vec3::new(other.yx, other.yy, other.zy),
            z: self.z + Vec3::new(other.zx, other.zy, other.zz),
        }
    }
}

impl Add<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn add(self, other: Matrix3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: Vec3::new(self.xx, self.yx, self.zx) + other.x,
            y: Vec3::new(self.yx, self.yy, self.zy) + other.y,
            z: Vec3::new(self.zx, self.zy, self.zz) + other.z,
        }
    }
}

impl Sub<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn sub(self, other: Symmetric3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: self.x - Vec3::new(other.xx, other.yx, other.zx),
            y: self.y - Vec3::new(other.yx, other.yy, other.zy),
            z: self.z - Vec3::new(other.zx, other.zy, other.zz),
        }
    }
}

impl Sub<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn sub(self, other: Matrix3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: Vec3::new(self.xx, self.yx, self.zx) - other.x,
            y: Vec3::new(self.yx, self.yy, self.zy) - other.y,
            z: Vec3::new(self.zx, self.zy, self.zz) - other.z,
        }
    }
}

impl Mul<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn mul(self, other: Symmetric3x3) -> Matrix3x3 {
        out!(Symmetric3x3::multiply(&self, &other))
    }
}

impl Mul<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn mul(self, other: Matrix3x3) -> Matrix3x3 {
        let axx = Vec3::splat(self.xx);
        let ayx = Vec3::splat(self.yx);
        let ayy = Vec3::splat(self.yy);
        let azx = Vec3::splat(self.zx);
        let azy = Vec3::splat(self.zy);
        let azz = Vec3::splat(self.zz);
        Matrix3x3 {
            x: axx * other.x + ayx * other.y + azx * other.z,
            y: ayx * other.x + ayy * other.y + azy * other.z,
            z: azx * other.x + azy * other.y + azz * other.z,
        }
    }
}

impl std::fmt::Display for Symmetric3x3 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "x: {}, y: {}, {}, z: {}, {}, {}",
            self.xx, self.yx, self.yy, self.zx, self.zy, self.zz
        )
    }
}
