use crate::out;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;

/// A wide SIMD-accelerated symmetric 6x6 matrix.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric6x6Wide {
    pub a: Symmetric3x3Wide,
    pub b: Matrix3x3Wide,
    pub d: Symmetric3x3Wide,
}

impl Symmetric6x6Wide {
    /// Scales each component of m by the given scale.
    #[inline(always)]
    pub fn scale(m: &Self, scale: &Vector<f32>, result: &mut Self) {
        Symmetric3x3Wide::scale(&m.a, scale, &mut result.a);
        Matrix3x3Wide::scale(&m.b, scale, &mut result.b);
        Symmetric3x3Wide::scale(&m.d, scale, &mut result.d);
    }

    #[inline(always)]
    pub fn invert(
        a: &Symmetric3x3Wide,
        b: &Matrix3x3Wide,
        d: &Symmetric3x3Wide,
        result: &mut Self,
    ) {
        // [ A  B ]^-1 = [ (A - B * D^-1 * BT)^-1, -(A - B * D^-1 * BT)^-1 * B * D^-1                   ]
        // [ BT D ]      [ symmetric               D^-1 + D^-1 * BT * (A - B * D^-1 * BT)^-1 * B * D^-1 ]
        // Note that B * D^-1 * BT produces a symmetric result. Likewise for the more complex result:
        // N = D^-1 * BT =>
        // D^-1 * BT * M * B * D^-1 = N * M * NT, because D is symmetric.
        let inv_d = out!(Symmetric3x3Wide::invert(d));
        let b_inv_d = out!(Symmetric3x3Wide::multiply_without_overlap_3x3(b, &inv_d));
        let b_inv_dbt = out!(Symmetric3x3Wide::complete_matrix_sandwich_by_transpose(
            &b_inv_d, b
        ));
        let result_a_inverse = out!(Symmetric3x3Wide::subtract(a, &b_inv_dbt));
        Symmetric3x3Wide::invert(&result_a_inverse, &mut result.a);

        let negated_result_b = out!(Symmetric3x3Wide::multiply(&result.a, &b_inv_d));
        Matrix3x3Wide::negate(&negated_result_b, &mut result.b);
        Symmetric3x3Wide::complete_matrix_sandwich_transpose(
            &b_inv_d,
            &negated_result_b,
            &mut result.d,
        );
        result.d = out!(Symmetric3x3Wide::add(&result.d, &inv_d));
    }

    // NOTE: C# intentionally does NOT force-inline this large method.
    pub fn invert_without_overlap(m: &Self, result: &mut Self) {
        Self::invert(&m.a, &m.b, &m.d, result);
    }

    /// Computes result = v * m, where v and result are 1x6 vectors which are split into two 1x3 values.
    #[inline(always)]
    pub fn transform_without_overlap(
        v0: &Vector3Wide,
        v1: &Vector3Wide,
        m: &Self,
        result0: &mut Vector3Wide,
        result1: &mut Vector3Wide,
    ) {
        result0.x = v0.x * m.a.xx
            + v0.y * m.a.yx
            + v0.z * m.a.zx
            + v1.x * m.b.x.x
            + v1.y * m.b.x.y
            + v1.z * m.b.x.z;
        result0.y = v0.x * m.a.yx
            + v0.y * m.a.yy
            + v0.z * m.a.zy
            + v1.x * m.b.y.x
            + v1.y * m.b.y.y
            + v1.z * m.b.y.z;
        result0.z = v0.x * m.a.zx
            + v0.y * m.a.zy
            + v0.z * m.a.zz
            + v1.x * m.b.z.x
            + v1.y * m.b.z.y
            + v1.z * m.b.z.z;

        result1.x = v0.x * m.b.x.x
            + v0.y * m.b.y.x
            + v0.z * m.b.z.x
            + v1.x * m.d.xx
            + v1.y * m.d.yx
            + v1.z * m.d.zx;
        result1.y = v0.x * m.b.x.y
            + v0.y * m.b.y.y
            + v0.z * m.b.z.y
            + v1.x * m.d.yx
            + v1.y * m.d.yy
            + v1.z * m.d.zy;
        result1.z = v0.x * m.b.x.z
            + v0.y * m.b.y.z
            + v0.z * m.b.z.z
            + v1.x * m.d.zx
            + v1.y * m.d.zy
            + v1.z * m.d.zz;
    }

    /// Solves [vLower, vUpper] = [resultLower, resultUpper] * [[a, b], [bT, d]] for [resultLower, resultUpper] using LDLT decomposition.
    /// [[a, b], [bT, d]] should be positive semidefinite.
    #[inline(always)]
    pub fn ldlt_solve(
        v0: &Vector3Wide,
        v1: &Vector3Wide,
        a: &Symmetric3x3Wide,
        b: &Matrix3x3Wide,
        d: &Symmetric3x3Wide,
        result0: &mut Vector3Wide,
        result1: &mut Vector3Wide,
    ) {
        let d1 = a.xx;
        let inverse_d1 = Vector::splat(1.0) / d1;
        let l21 = inverse_d1 * a.yx;
        let l31 = inverse_d1 * a.zx;
        let l41 = inverse_d1 * b.x.x;
        let l51 = inverse_d1 * b.x.y;
        let l61 = inverse_d1 * b.x.z;
        let d2 = a.yy - l21 * l21 * d1;
        let inverse_d2 = Vector::splat(1.0) / d2;
        let l32 = inverse_d2 * (a.zy - l31 * l21 * d1);
        let l42 = inverse_d2 * (b.y.x - l41 * l21 * d1);
        let l52 = inverse_d2 * (b.y.y - l51 * l21 * d1);
        let l62 = inverse_d2 * (b.y.z - l61 * l21 * d1);
        let d3 = a.zz - l31 * l31 * d1 - l32 * l32 * d2;
        let inverse_d3 = Vector::splat(1.0) / d3;
        let l43 = inverse_d3 * (b.z.x - l41 * l31 * d1 - l42 * l32 * d2);
        let l53 = inverse_d3 * (b.z.y - l51 * l31 * d1 - l52 * l32 * d2);
        let l63 = inverse_d3 * (b.z.z - l61 * l31 * d1 - l62 * l32 * d2);
        let d4 = d.xx - l41 * l41 * d1 - l42 * l42 * d2 - l43 * l43 * d3;
        let inverse_d4 = Vector::splat(1.0) / d4;
        let l54 = inverse_d4 * (d.yx - l51 * l41 * d1 - l52 * l42 * d2 - l53 * l43 * d3);
        let l64 = inverse_d4 * (d.zx - l61 * l41 * d1 - l62 * l42 * d2 - l63 * l43 * d3);
        let d5 = d.yy - l51 * l51 * d1 - l52 * l52 * d2 - l53 * l53 * d3 - l54 * l54 * d4;
        let inverse_d5 = Vector::splat(1.0) / d5;
        let l65 =
            inverse_d5 * (d.zy - l61 * l51 * d1 - l62 * l52 * d2 - l63 * l53 * d3 - l64 * l54 * d4);
        let d6 = d.zz
            - l61 * l61 * d1
            - l62 * l62 * d2
            - l63 * l63 * d3
            - l64 * l64 * d4
            - l65 * l65 * d5;
        let inverse_d6 = Vector::splat(1.0) / d6;

        // We now have the components of L and D, so substitute.
        result0.x = v0.x;
        result0.y = v0.y - l21 * result0.x;
        result0.z = v0.z - l31 * result0.x - l32 * result0.y;
        result1.x = v1.x - l41 * result0.x - l42 * result0.y - l43 * result0.z;
        result1.y = v1.y - l51 * result0.x - l52 * result0.y - l53 * result0.z - l54 * result1.x;
        result1.z = v1.z
            - l61 * result0.x
            - l62 * result0.y
            - l63 * result0.z
            - l64 * result1.x
            - l65 * result1.y;

        result1.z *= inverse_d6;
        result1.y = result1.y * inverse_d5 - l65 * result1.z;
        result1.x = result1.x * inverse_d4 - l64 * result1.z - l54 * result1.y;
        result0.z = result0.z * inverse_d3 - l63 * result1.z - l53 * result1.y - l43 * result1.x;
        result0.y = result0.y * inverse_d2
            - l62 * result1.z
            - l52 * result1.y
            - l42 * result1.x
            - l32 * result0.z;
        result0.x = result0.x * inverse_d1
            - l61 * result1.z
            - l51 * result1.y
            - l41 * result1.x
            - l31 * result0.z
            - l21 * result0.y;
    }
}
