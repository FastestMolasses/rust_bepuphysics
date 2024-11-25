use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::utilities::vector4_wide::Vector4Wide;

/// A wide SIMD-accelerated symmetric 4x4 matrix.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Symmetric4x4Wide {
    /// First row, first column of the matrix.
    pub xx: Vector<f32>,
    /// Second row, first column of the matrix.
    pub yx: Vector<f32>,
    /// Second row, second column of the matrix.
    pub yy: Vector<f32>,
    /// Third row, first column of the matrix.
    pub zx: Vector<f32>,
    /// Third row, second column of the matrix.
    pub zy: Vector<f32>,
    /// Third row, third column of the matrix.
    pub zz: Vector<f32>,
    /// Fourth row, first column of the matrix.
    pub wx: Vector<f32>,
    /// Fourth row, second column of the matrix.
    pub wy: Vector<f32>,
    /// Fourth row, third column of the matrix.
    pub wz: Vector<f32>,
    /// Fourth row, fourth column of the matrix.
    pub ww: Vector<f32>,
}

impl Symmetric4x4Wide {
    /// Returns a reference to the upper left 3x3 block of the matrix.
    #[inline(always)]
    pub fn get_upper_left_3x3_block(m: &Self) -> &Symmetric3x3Wide {
        unsafe { &*(m as *const _ as *const Symmetric3x3Wide) }
    }

    /// Returns a mutable reference to the upper left 3x3 block of the matrix.
    #[inline(always)]
    pub fn get_upper_left_3x3_block_mut(m: &mut Self) -> &mut Symmetric3x3Wide {
        unsafe { &mut *(m as *mut _ as *mut Symmetric3x3Wide) }
    }

    /// Returns a reference to the upper right 3x1 (or lower left 1x3) block of the matrix.
    #[inline(always)]
    pub fn get_upper_right_3x1_block(m: &Self) -> &Vector3Wide {
        unsafe { &*((&m.wx as *const Vector<f32>) as *const Vector3Wide) }
    }

    /// Returns a mutable reference to the upper right 3x1 (or lower left 1x3) block of the matrix.
    #[inline(always)]
    pub fn get_upper_right_3x1_block_mut(m: &mut Self) -> &mut Vector3Wide {
        unsafe { &mut *((&mut m.wx as *mut Vector<f32>) as *mut Vector3Wide) }
    }

    /// Scales each component of m by the given scale.
    #[inline(always)]
    pub fn scale(m: &Self, scale: Vector<f32>, result: &mut Self) {
        result.xx = m.xx * scale;
        result.yx = m.yx * scale;
        result.yy = m.yy * scale;
        result.zx = m.zx * scale;
        result.zy = m.zy * scale;
        result.zz = m.zz * scale;
        result.wx = m.wx * scale;
        result.wy = m.wy * scale;
        result.wz = m.wz * scale;
        result.ww = m.ww * scale;
    }

    /// Inverts the matrix without allowing input and output to overlap.
    #[inline(always)]
    pub fn invert_without_overlap(m: &Self, result: &mut Self) {
        let s0 = m.xx * m.yy - m.yx * m.yx;
        let s1 = m.xx * m.zy - m.yx * m.zx;
        let s2 = m.xx * m.wy - m.yx * m.wx;
        let s3 = m.yx * m.zy - m.yy * m.zx;
        let s4 = m.yx * m.wy - m.yy * m.wx;
        let s5 = m.zx * m.wy - m.zy * m.wx;
        let c5 = m.zz * m.ww - m.wz * m.wz;
        let c4 = m.zy * m.ww - m.wy * m.wz;
        let c3 = m.zy * m.wz - m.wy * m.zz;
        let c2 = m.zx * m.ww - m.wx * m.wz;
        let c1 = m.zx * m.wz - m.wx * m.zz;

        let inverse_determinant =
            Vector::splat(1.0) / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * s5);

        result.xx = (m.yy * c5 - m.zy * c4 + m.wy * c3) * inverse_determinant;

        result.yx = (-m.yx * c5 + m.zy * c2 - m.wy * c1) * inverse_determinant;
        result.yy = (m.xx * c5 - m.zx * c2 + m.wx * c1) * inverse_determinant;

        result.zx = (m.yx * c4 - m.yy * c2 + m.wy * s5) * inverse_determinant;
        result.zy = (-m.xx * c4 + m.yx * c2 - m.wx * s5) * inverse_determinant;
        result.zz = (m.wx * s4 - m.wy * s2 + m.ww * s0) * inverse_determinant;

        result.wx = (-m.yx * c3 + m.yy * c1 - m.zy * s5) * inverse_determinant;
        result.wy = (m.xx * c3 - m.yx * c1 + m.zx * s5) * inverse_determinant;
        result.wz = (-m.wx * s3 + m.wy * s1 - m.wz * s0) * inverse_determinant;
        result.ww = (m.zx * s3 - m.zy * s1 + m.zz * s0) * inverse_determinant;
    }

    /// Computes result = v * m.
    #[inline(always)]
    pub fn transform_without_overlap(v: &Vector4Wide, m: &Self, result: &mut Vector4Wide) {
        result.x = v.x * m.xx + v.y * m.yx + v.z * m.zx + v.w * m.wx;
        result.y = v.x * m.yx + v.y * m.yy + v.z * m.zy + v.w * m.wy;
        result.z = v.x * m.zx + v.y * m.zy + v.z * m.zz + v.w * m.wz;
        result.w = v.x * m.wx + v.y * m.wy + v.z * m.wz + v.w * m.ww;
    }
}
