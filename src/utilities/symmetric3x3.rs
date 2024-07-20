use crate::utilities::matrix3x3::Matrix3x3;
use crate::utilities::vector3::Vector3;
use core::ops::{Add, Mul, Sub};

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric3x3 {
    pub xx: f32,
    pub yx: f32,
    pub yy: f32,
    pub zx: f32,
    pub zy: f32,
    pub zz: f32,
}

impl Symmetric3x3 {
    #[inline(always)]
    pub fn rotation_sandwich(r: &Matrix3x3, m: &Symmetric3x3) -> Self {
        let i11 = r.x.x * m.xx + r.y.x * m.yx + r.z.x * m.zx;
        let i12 = r.x.x * m.yx + r.y.x * m.yy + r.z.x * m.zy;
        let i13 = r.x.x * m.zx + r.y.x * m.zy + r.z.x * m.zz;

        let i21 = r.x.y * m.xx + r.y.y * m.yx + r.z.y * m.zx;
        let i22 = r.x.y * m.yx + r.y.y * m.yy + r.z.y * m.zy;
        let i23 = r.x.y * m.zx + r.y.y * m.zy + r.z.y * m.zz;

        let i31 = r.x.z * m.xx + r.y.z * m.yx + r.z.z * m.zx;
        let i32 = r.x.z * m.yx + r.y.z * m.yy + r.z.z * m.zy;
        let i33 = r.x.z * m.zx + r.y.z * m.zy + r.z.z * m.zz;

        Self {
            xx: i11 * r.x.x + i12 * r.y.x + i13 * r.z.x,
            yx: i21 * r.x.x + i22 * r.y.x + i23 * r.z.x,
            yy: i21 * r.x.y + i22 * r.y.y + i23 * r.z.y,
            zx: i31 * r.x.x + i32 * r.y.x + i33 * r.z.x,
            zy: i31 * r.x.y + i32 * r.y.y + i33 * r.z.y,
            zz: i31 * r.x.z + i32 * r.y.z + i33 * r.z.z,
        }
    }

    #[inline(always)]
    pub fn determinant(&self) -> f32 {
        let m11 = self.yy * self.zz - self.zy * self.zy;
        let m21 = self.zy * self.zx - self.zz * self.yx;
        let m31 = self.yx * self.zy - self.zx * self.yy;
        m11 * self.xx + m21 * self.yx + m31 * self.zx
    }

    #[inline(always)]
    pub fn invert(&self) -> Self {
        let m11 = self.yy * self.zz - self.zy * self.zy;
        let m21 = self.zy * self.zx - self.zz * self.yx;
        let m31 = self.yx * self.zy - self.zx * self.yy;
        let determinant_inverse = 1.0 / (m11 * self.xx + m21 * self.yx + m31 * self.zx);

        let m22 = self.zz * self.xx - self.zx * self.zx;
        let m32 = self.zx * self.yx - self.xx * self.zy;

        let m33 = self.xx * self.yy - self.yx * self.yx;

        Self {
            xx: m11 * determinant_inverse,
            yx: m21 * determinant_inverse,
            zx: m31 * determinant_inverse,
            yy: m22 * determinant_inverse,
            zy: m32 * determinant_inverse,
            zz: m33 * determinant_inverse,
        }
    }

    #[inline(always)]
    pub fn scale(&self, scale: f32) -> Self {
        Self {
            xx: self.xx * scale,
            yx: self.yx * scale,
            yy: self.yy * scale,
            zx: self.zx * scale,
            zy: self.zy * scale,
            zz: self.zz * scale,
        }
    }

    #[inline(always)]
    pub fn multiply(a: &Matrix3x3, b: &Symmetric3x3) -> Matrix3x3 {
        let bx = Vector3::new(b.xx, b.yx, b.zx);
        let by = Vector3::new(b.yx, b.yy, b.zy);
        let bz = Vector3::new(b.zx, b.zy, b.zz);

        let result_x = {
            let x = Vector3::splat(a.x.0.extract(0));
            let y = Vector3::splat(a.x.0.extract(1));
            let z = Vector3::splat(a.x.0.extract(2));
            x * bx + y * by + z * bz
        };

        let result_y = {
            let x = Vector3::splat(a.y.0.extract(0));
            let y = Vector3::splat(a.y.0.extract(1));
            let z = Vector3::splat(a.y.0.extract(2));
            x * bx + y * by + z * bz
        };

        let result_z = {
            let x = Vector3::splat(a.z.0.extract(0));
            let y = Vector3::splat(a.z.0.extract(1));
            let z = Vector3::splat(a.z.0.extract(2));
            x * bx + y * by + z * bz
        };

        Matrix3x3 {
            x: result_x,
            y: result_y,
            z: result_z,
        }
    }

    #[inline(always)]
    pub fn multiply_without_overlap(a: &Symmetric3x3, b: &Symmetric3x3) -> Symmetric3x3 {
        let ayxbyx = a.yx * b.yx;
        let azxbzx = a.zx * b.zx;
        let azybzy = a.zy * b.zy;

        Symmetric3x3 {
            xx: a.xx * b.xx + ayxbyx + azxbzx,
            yx: a.yx * b.xx + a.yy * b.yx + a.zy * b.zx,
            yy: ayxbyx + a.yy * b.yy + azybzy,
            zx: a.zx * b.xx + a.zy * b.yx + a.zz * b.zx,
            zy: a.zx * b.yx + a.zy * b.yy + a.zz * b.zy,
            zz: azxbzx + azybzy + a.zz * b.zz,
        }
    }

    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector3, m: &Symmetric3x3) -> Vector3 {
        let v_x = v.0.extract(0);
        let v_y = v.0.extract(1);
        let v_z = v.0.extract(2);

        let x = v_x * m.xx + v_y * m.yx + v_z * m.zx;
        let y = v_x * m.yx + v_y * m.yy + v_z * m.zy;
        let z = v_x * m.zx + v_y * m.zy + v_z * m.zz;

        Vector3(f32x4::new(x, y, z, 0.0))
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
        self.scale(scale)
    }
}

impl Mul for Symmetric3x3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self {
        self.multiply_without_overlap(&other)
    }
}

impl Add<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn add(self, other: Symmetric3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: self.x + Vector3::new(other.xx, other.yx, other.zx),
            y: self.y + Vector3::new(other.yx, other.yy, other.zy),
            z: self.z + Vector3::new(other.zx, other.zy, other.zz),
        }
    }
}

impl Add<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn add(self, other: Matrix3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: Vector3::new(self.xx, self.yx, self.zx) + other.x,
            y: Vector3::new(self.yx, self.yy, self.zy) + other.y,
            z: Vector3::new(self.zx, self.zy, self.zz) + other.z,
        }
    }
}

impl Sub<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn sub(self, other: Symmetric3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: self.x - Vector3::new(other.xx, other.yx, other.zx),
            y: self.y - Vector3::new(other.yx, other.yy, other.zy),
            z: self.z - Vector3::new(other.zx, other.zy, other.zz),
        }
    }
}

impl Sub<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn sub(self, other: Matrix3x3) -> Matrix3x3 {
        Matrix3x3 {
            x: Vector3::new(self.xx, self.yx, self.zx) - other.x,
            y: Vector3::new(self.yx, self.yy, self.zy) - other.y,
            z: Vector3::new(self.zx, self.zy, self.zz) - other.z,
        }
    }
}

impl Mul<Symmetric3x3> for Matrix3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn mul(self, other: Symmetric3x3) -> Matrix3x3 {
        Symmetric3x3::multiply(&self, &other)
    }
}

impl Mul<Matrix3x3> for Symmetric3x3 {
    type Output = Matrix3x3;

    #[inline(always)]
    fn mul(self, other: Matrix3x3) -> Matrix3x3 {
        let axx = Vector3::splat(self.xx);
        let ayx = Vector3::splat(self.yx);
        let ayy = Vector3::splat(self.yy);
        let azx = Vector3::splat(self.zx);
        let azy = Vector3::splat(self.zy);
        let azz = Vector3::splat(self.zz);
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
