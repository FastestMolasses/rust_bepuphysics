use packed_simd::*;
use std::arch::aarch64::*;
use std::arch::x86_64::*;
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use crate::Quaternion;
use crate::Vector3;

#[inline(always)]
pub fn clamp<T: PartialOrd>(value: T, min: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[inline(always)]
pub fn max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
pub fn to_radians(degrees: f32) -> f32 {
    degrees * (PI / 180.0)
}

#[inline(always)]
pub fn to_degrees(radians: f32) -> f32 {
    radians * (180.0 / PI)
}

#[inline(always)]
pub fn binary_sign(x: f32) -> f32 {
    if x < 0.0 {
        -1.0
    } else {
        1.0
    }
}

#[inline(always)]
pub fn cos(x: f32) -> f32 {
    let period_count = x * (0.5 / PI as f64) as f32;
    let period_fraction = period_count - period_count.floor();
    let period_x = period_fraction * 2.0 * PI;

    let y = if period_x > 3.0 * FRAC_PI_2 {
        2.0 * PI - period_x
    } else if period_x > PI {
        period_x - PI
    } else if period_x > FRAC_PI_2 {
        PI - period_x
    } else {
        period_x
    };

    let numerator =
        (((((-0.003436308368583229 * y + 0.021317031205957775) * y + 0.06955843390178032) * y
            - 0.4578088075324152)
            * y
            - 0.15082367674208508)
            * y
            + 1.0);
    let denominator =
        (((((-0.00007650398834677185 * y + 0.0007451378206294365) * y - 0.00585321045829395) * y
            + 0.04219116713777847)
            * y
            - 0.15082367538305258)
            * y
            + 1.0);
    let result = numerator / denominator;

    if period_x > FRAC_PI_2 && period_x < 3.0 * FRAC_PI_2 {
        -result
    } else {
        result
    }
}

#[inline(always)]
pub fn sin(x: f32) -> f32 {
    let period_count = x * (0.5 / PI as f64) as f32;
    let period_fraction = period_count - period_count.floor();
    let period_x = period_fraction * 2.0 * PI;

    let y = if period_x > 3.0 * FRAC_PI_2 {
        2.0 * PI - period_x
    } else if period_x > PI {
        period_x - PI
    } else if period_x > FRAC_PI_2 {
        PI - period_x
    } else {
        period_x
    };

    let numerator =
        ((((0.0040507708755727605 * y - 0.006685815219853882) * y - 0.13993701695343166) * y
            + 0.06174562337697123)
            * y
            + 1.00000000151466040)
            * y;
    let denominator =
        ((((0.00009018370615921334 * y + 0.0001700784176413186) * y + 0.003606014457152456) * y
            + 0.02672943625500751)
            * y
            + 0.061745651499203795)
            * y
            + 1.0;
    let result = numerator / denominator;

    if period_x > PI {
        -result
    } else {
        result
    }
}

#[inline(always)]
pub fn acos(x: f32) -> f32 {
    let negative_input = x < 0.0;
    let x = 1.0f32.min(x.abs());
    let numerator = (1.0 - x).sqrt()
        * (62.95741097600742
            + x * (69.6550664543659 + x * (17.54512349463405 + x * 0.6022076120669532)));
    let denominator = 40.07993264439811 + x * (49.81949855726789 + x * (15.703851745284796 + x));
    let result = numerator / denominator;
    if negative_input {
        PI - result
    } else {
        result
    }
}

#[inline(always)]
pub fn cos_simd(x: f32x4) -> f32x4 {
    let period_count = x * (0.5 / PI as f64) as f32;
    let period_fraction = period_count - period_count.floor();
    let two_pi = f32x4::splat(2.0 * PI);
    let period_x = period_fraction * two_pi;

    let pi_over_2 = f32x4::splat(FRAC_PI_2);
    let pi = f32x4::splat(PI);
    let pi_3_over_2 = f32x4::splat(3.0 * FRAC_PI_2);

    let y = (period_x.gt(pi_over_2) & (pi - period_x).lt(period_x))
        | (period_x.gt(pi) & (period_x - pi).lt(period_x))
        | (period_x.gt(pi_3_over_2) & (two_pi - period_x).lt(period_x));

    let numerator =
        ((((f32x4::splat(-0.003436308368583229) * y + f32x4::splat(0.021317031205957775)) * y
            + f32x4::splat(0.06955843390178032))
            * y
            - f32x4::splat(0.4578088075324152))
            * y
            - f32x4::splat(0.15082367674208508))
            * y
            + f32x4::splat(1.0);
    let denominator =
        ((((f32x4::splat(-0.00007650398834677185) * y + f32x4::splat(0.0007451378206294365)) * y
            - f32x4::splat(0.00585321045829395))
            * y
            + f32x4::splat(0.04219116713777847))
            * y
            - f32x4::splat(0.15082367538305258))
            * y
            + f32x4::splat(1.0);
    let result = numerator / denominator;

    (period_x.gt(pi_over_2) & period_x.lt(pi_3_over_2)).select(-result, result)
}

#[inline(always)]
pub fn sin_simd(x: f32x4) -> f32x4 {
    let period_count = x * (0.5 / PI as f64) as f32;
    let period_fraction = period_count - period_count.floor();
    let two_pi = f32x4::splat(2.0 * PI);
    let period_x = period_fraction * two_pi;

    let pi = f32x4::splat(PI);
    let pi_over_2 = f32x4::splat(FRAC_PI_2);
    let pi_3_over_2 = f32x4::splat(3.0 * FRAC_PI_2);

    let y = (period_x.gt(pi_over_2) & (pi - period_x).lt(period_x))
        | (period_x.gt(pi) & (period_x - pi).lt(period_x))
        | (period_x.gt(pi_3_over_2) & (two_pi - period_x).lt(period_x));

    let in_second_half = period_x.gt(pi);

    let numerator =
        ((((f32x4::splat(0.0040507708755727605) * y - f32x4::splat(0.006685815219853882)) * y
            - f32x4::splat(0.13993701695343166))
            * y
            + f32x4::splat(0.06174562337697123))
            * y
            + f32x4::splat(1.00000000151466040))
            * y;
    let denominator =
        ((((f32x4::splat(0.00009018370615921334) * y + f32x4::splat(0.0001700784176413186)) * y
            + f32x4::splat(0.003606014457152456))
            * y
            + f32x4::splat(0.02672943625500751))
            * y
            + f32x4::splat(0.061745651499203795))
            * y
            + f32x4::splat(1.0);
    let result = numerator / denominator;

    in_second_half.select(-result, result)
}

#[inline(always)]
pub fn acos_simd(x: f32x4) -> f32x4 {
    let negative_input = x.lt(f32x4::splat(0.0));
    let x = f32x4::splat(1.0).min(x.abs());
    let numerator = (f32x4::splat(1.0) - x).sqrt()
        * (f32x4::splat(62.95741097600742)
            + x * (f32x4::splat(69.6550664543659)
                + x * (f32x4::splat(17.54512349463405) + x * f32x4::splat(0.6022076120669532))));
    let denominator = f32x4::splat(40.07993264439811)
        + x * (f32x4::splat(49.81949855726789) + x * (f32x4::splat(15.703851745284796) + x));
    let result = numerator / denominator;
    negative_input.select(f32x4::splat(PI) - result, result)
}

#[inline(always)]
pub fn get_signed_angle_difference(a: f32x4, b: f32x4) -> f32x4 {
    let half = f32x4::splat(0.5);
    let x = (b - a) * f32x4::splat(1.0 / (2.0 * PI)) + half;
    (x - x.floor() - half) * f32x4::splat(2.0 * PI)
}

#[inline(always)]
pub fn fast_reciprocal(v: f32x4) -> f32x4 {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    unsafe {
        if is_x86_feature_detected!("avx") {
            let v256 = _mm256_castps128_ps256(_mm_loadu_ps(v.as_ptr()));
            let result256 = _mm256_rcp_ps(v256);
            let result128 = _mm256_castps256_ps128(result256);
            std::mem::transmute(result128)
        } else if is_x86_feature_detected!("sse") {
            let v128 = _mm_loadu_ps(v.as_ptr());
            let result128 = _mm_rcp_ps(v128);
            std::mem::transmute(result128)
        } else {
            f32x4::splat(1.0) / v
        }
    }
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let v_neon = vld1q_f32(v.as_ptr());
        let result_neon = vrecpeq_f32(v_neon);
        // Optional: Perform a Newton-Raphson iteration for higher precision
        // let step = vmulq_f32(result_neon, vrecpsq_f32(v_neon, result_neon));
        // result_neon = vmulq_f32(step, vrecpsq_f32(v_neon, step));
        std::mem::transmute(result_neon)
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
    {
        f32x4::splat(1.0) / v
    }
}

#[inline(always)]
pub fn fast_reciprocal_square_root(v: f32x4) -> f32x4 {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    unsafe {
        if is_x86_feature_detected!("avx") {
            let v256 = _mm256_castps128_ps256(_mm_loadu_ps(v.as_ptr()));
            let result256 = _mm256_rsqrt_ps(v256);
            let result128 = _mm256_castps256_ps128(result256);
            std::mem::transmute(result128)
        } else if is_x86_feature_detected!("sse") {
            let v128 = _mm_loadu_ps(v.as_ptr());
            let result128 = _mm_rsqrt_ps(v128);
            std::mem::transmute(result128)
        } else {
            f32x4::splat(1.0) / v.sqrt()
        }
    }
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let v_neon = vld1q_f32(v.as_ptr());
        let result_neon = vrsqrteq_f32(v_neon);
        // Optional: Perform a Newton-Raphson iteration for higher precision
        // let step = vmulq_f32(vrsqrtsq_f32(vmulq_f32(v_neon, result_neon), result_neon), result_neon);
        // result_neon = vmulq_f32(vrsqrtsq_f32(vmulq_f32(v_neon, step), step), step);
        std::mem::transmute(result_neon)
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
    {
        f32x4::splat(1.0) / v.sqrt()
    }
}
