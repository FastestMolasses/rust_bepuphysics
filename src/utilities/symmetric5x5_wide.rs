use crate::out;
use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// A wide SIMD-accelerated symmetric 5x5 matrix.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric5x5Wide {
    /// Upper left 3x3 block of the matrix.
    pub a: Symmetric3x3Wide,
    /// Lower left 2x3 block of the matrix.
    pub b: Matrix2x3Wide,
    /// Lower right 2x2 block of the matrix.
    pub d: Symmetric2x2Wide,
}

impl Symmetric5x5Wide {
    /// Scales each component of m by the given scale.
    #[inline(always)]
    pub fn scale(m: &Self, scale: &Vector<f32>, result: &mut Self) {
        Symmetric3x3Wide::scale(&m.a, scale, &mut result.a);
        Matrix2x3Wide::scale(&m.b, scale, &mut result.b);
        Symmetric2x2Wide::scale(&m.d, scale, &mut result.d);
    }

    #[inline(always)]
    pub fn invert(
        a: &Symmetric3x3Wide,
        b: &Matrix2x3Wide,
        d: &Symmetric2x2Wide,
        result: &mut Self,
    ) {
        // [ A  BT ]^-1 = [ (A - BT * D^-1 * B)^-1, -(A - BT * D^-1 * B)^-1 * BT * D^-1                   ]
        // [ B  D  ]      [ symmetric               D^-1 + D^-1 * B * (A - BT * D^-1 * B)^-1 * BT * D^-1 ]
        let inv_d = out!(Symmetric2x2Wide::invert_without_overlap(d));
        let bt_inv_d = out!(Symmetric2x2Wide::multiply_transposed(b, &inv_d));
        let bt_inv_db = out!(Symmetric3x3Wide::complete_matrix_sandwich_2x3(&bt_inv_d, b));
        let result_a_inverse = out!(Symmetric3x3Wide::subtract(a, &bt_inv_db));
        Symmetric3x3Wide::invert(&result_a_inverse, &mut result.a);

        let negated_result_bt = out!(Symmetric3x3Wide::multiply_by_transposed_2x3(
            &result.a, &bt_inv_d
        ));
        Matrix2x3Wide::negate(&negated_result_bt, &mut result.b);
        Symmetric2x2Wide::complete_matrix_sandwich(&bt_inv_d, &negated_result_bt, &mut result.d);
        result.d = out!(Symmetric2x2Wide::add(&result.d, &inv_d));
    }

    #[inline(always)]
    pub fn invert_without_overlap(m: &Self, result: &mut Self) {
        Self::invert(&m.a, &m.b, &m.d, result);
    }

    /// Computes result = v * m, where v and result are 1x5 vectors which are split into two subvectors.
    #[inline(always)]
    pub fn transform_without_overlap(
        v0: &Vector3Wide,
        v1: &Vector2Wide,
        m: &Self,
        result0: &mut Vector3Wide,
        result1: &mut Vector2Wide,
    ) {
        // [ v0x v0y v0z v1x v1y ] * [ m.A.XX m.A.YX m.A.ZX b.X.X b.Y.X ]
        //                           [ m.A.YX m.A.YY m.A.ZY b.X.Y b.Y.Y ]
        //                           [ m.A.ZX m.A.ZY m.A.ZZ b.X.Z b.Y.Z ]
        //                           [ b.X.X  b.X.Y  b.X.Z  d.XX  d.YX  ]
        //                           [ b.Y.X  b.Y.Y  b.Y.Z  d.YX  d.YY  ]
        result0.x = v0.x * m.a.xx + v0.y * m.a.yx + v0.z * m.a.zx + v1.x * m.b.x.x + v1.y * m.b.y.x;
        result0.y = v0.x * m.a.yx + v0.y * m.a.yy + v0.z * m.a.zy + v1.x * m.b.x.y + v1.y * m.b.y.y;
        result0.z = v0.x * m.a.zx + v0.y * m.a.zy + v0.z * m.a.zz + v1.x * m.b.x.z + v1.y * m.b.y.z;

        result1.x =
            v0.x * m.b.x.x + v0.y * m.b.x.y + v0.z * m.b.x.z + v1.x * m.d.xx + v1.y * m.d.yx;
        result1.y =
            v0.x * m.b.y.x + v0.y * m.b.y.y + v0.z * m.b.y.z + v1.x * m.d.yx + v1.y * m.d.yy;
    }
}
