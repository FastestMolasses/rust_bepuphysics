use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::vector2_wide::Vector2Wide;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric2x2Wide {
    pub xx: Vector<f32>,
    pub yx: Vector<f32>,
    pub yy: Vector<f32>,
}

impl Symmetric2x2Wide {
    /// Computes m * scale * mT.
    /// This is a peculiar operation, but it's useful for computing linear effective mass contributions in 2DOF constraints.
    #[inline(always)]
    pub fn sandwich_scale(m: &Matrix2x3Wide, scale: &Vector<f32>, result: &mut Self) {
        result.xx = scale * (m.x.x * m.x.x + m.x.y * m.x.y + m.x.z * m.x.z);
        result.yx = scale * (m.y.x * m.x.x + m.y.y * m.x.y + m.y.z * m.x.z);
        result.yy = scale * (m.y.x * m.y.x + m.y.y * m.y.y + m.y.z * m.y.z);
    }

    #[inline(always)]
    pub fn scale(t: &Self, scale: &Vector<f32>, result: &mut Self) {
        result.xx = t.xx * scale;
        result.yx = t.yx * scale;
        result.yy = t.yy * scale;
    }

    #[inline(always)]
    pub fn add(a: &Self, other: &Self, result: &mut Self) {
        result.xx = a.xx + other.xx;
        result.yx = a.yx + other.yx;
        result.yy = a.yy + other.yy;
    }

    #[inline(always)]
    pub fn sub(a: &Self, other: &Self, result: &mut Self) {
        result.xx = a.xx - other.xx;
        result.yx = a.yx - other.yx;
        result.yy = a.yy - other.yy;
    }

    #[inline(always)]
    pub fn invert_without_overlap(m: &Self, inverse: &mut Self) {
        let denom = Vector::<f32>::splat(1.0) / (m.yx * m.yx - m.xx * m.yy);
        inverse.xx = -m.yy * denom;
        inverse.yx = m.yx * denom;
        inverse.yy = -m.xx * denom;
    }

    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector2Wide, m: &Self, result: &mut Vector2Wide) {
        result.x = v.x * m.xx + v.y * m.yx;
        result.y = v.x * m.yx + v.y * m.yy;
    }

    #[inline(always)]
    pub fn multiply_transposed(&self, a: &Matrix2x3Wide) -> Matrix2x3Wide {
        Matrix2x3Wide {
            x: Vector3Wide {
                x: a.x.x * self.xx + a.y.x * self.yx,
                y: a.x.y * self.xx + a.y.y * self.yx,
                z: a.x.z * self.xx + a.y.z * self.yx,
            },
            y: Vector3Wide {
                x: a.x.x * self.yx + a.y.x * self.yy,
                y: a.x.y * self.yx + a.y.y * self.yy,
                z: a.x.z * self.yx + a.y.z * self.yy,
            },
        }
    }

    /// Computes a * transpose(b), assuming a = b * M for some symmetric matrix M. This is conceptually the second half of Triangular3x3Wide.MatrixSandwich.
    #[inline(always)]
    pub fn complete_matrix_sandwich(a: &Matrix2x3Wide, b: &Matrix2x3Wide) -> Self {
        Self {
            xx: a.x.x * b.x.x + a.x.y * b.x.y + a.x.z * b.x.z,
            yx: a.y.x * b.x.x + a.y.y * b.x.y + a.y.z * b.x.z,
            yy: a.y.x * b.y.x + a.y.y * b.y.y + a.y.z * b.y.z,
        }
    }
}
