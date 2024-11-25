use crate::utilities::vector::Vector;
use std::f32::consts::{FRAC_PI_2, PI};
use std::ops::BitAnd;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;
use std::simd::StdFloat;

const TWO_PI: f32 = 2.0 * PI;

/// Clamps a value between a minimum and maximum value.
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

/// Returns the higher value of the two parameters.
#[inline(always)]
pub fn max<T: PartialOrd>(a: T, b: T) -> T {
    if a > b {
        a
    } else {
        b
    }
}

/// Returns the lower value of the two parameters.
#[inline(always)]
pub fn min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// Returns -1 if the value is negative and 1 otherwise.
#[inline(always)]
pub fn binary_sign(x: f32) -> f32 {
    if x < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Computes an approximation of cosine. Maximum error a little below 8e-7 for the interval -2 * Pi to 2 * Pi. Values further from the interval near zero have gracefully degrading error.
/// Note that these cos/sin implementations are not here for performance, but rather to:
/// 1) Provide a SIMD accelerated version for wide processing, and
/// 2) Provide a scalar implementation that is consistent with the SIMD version for systems which need to match its behavior.
#[inline(always)]
pub fn cos(x: f32) -> f32 {
    // Rational approximation over [0, pi/2], use symmetry for the rest.
    let period_count = x * (0.5 / PI);
    let period_fraction = period_count - period_count.floor(); // This is a source of error as you get away from 0.
    let period_x = period_fraction * TWO_PI;

    // [0, pi/2] = f(x)
    // (pi/2, pi] = -f(Pi - x)
    // (pi, 3 * pi / 2] = -f(x - Pi)
    // (3*pi/2, 2*pi] = f(2 * Pi - x)
    let y = if period_x > 3.0 * FRAC_PI_2 {
        TWO_PI - period_x
    } else if period_x > PI {
        period_x - PI
    } else if period_x > FRAC_PI_2 {
        PI - period_x
    } else {
        period_x
    };

    // Using a fifth degree numerator and denominator.
    // This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
    // TODO: FMA could help here, primarily in terms of precision.
    let numerator =
        ((((-0.003436308368583229 * y + 0.021317031205957775) * y + 0.06955843390178032) * y
            - 0.4578088075324152)
            * y
            - 0.15082367674208508)
            * y
            + 1.0;
    let denominator =
        ((((-0.00007650398834677185 * y + 0.0007451378206294365) * y - 0.00585321045829395) * y
            + 0.04219116713777847)
            * y
            - 0.15082367538305258)
            * y
            + 1.0;
    let result = numerator / denominator;

    if period_x > FRAC_PI_2 && period_x < 3.0 * FRAC_PI_2 {
        -result
    } else {
        result
    }
}

/// Computes an approximation of sine. Maximum error a little below 5e-7 for the interval -2 * Pi to 2 * Pi.
/// Values further from the interval near zero have gracefully degrading error.
#[inline(always)]
pub fn sin(x: f32) -> f32 {
    // Similar to cos, use a rational approximation for the region of sin from [0, pi/2]. Use symmetry to cover the rest.
    // This has its own implementation rather than just calling into Cos because we want maximum fidelity near 0.
    let period_count = x * (0.5 / PI);
    let period_fraction = period_count - period_count.floor(); // This is a source of error as you get away from 0.
    let period_x = period_fraction * TWO_PI;

    // [0, pi/2] = f(x)
    // (pi/2, pi] = f(pi - x)
    // (pi, 3/2 * pi] = -f(x - pi)
    // (3/2 * pi, 2*pi] = -f(2 * pi - x)
    let y = if period_x > 3.0 * FRAC_PI_2 {
        TWO_PI - period_x
    } else if period_x > PI {
        period_x - PI
    } else if period_x > FRAC_PI_2 {
        PI - period_x
    } else {
        period_x
    };

    // Using a fifth degree numerator and denominator.
    // This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
    // TODO: FMA could help here, primarily in terms of precision.
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

/// Computes an approximation of arccos. Inputs outside of [-1, 1] are clamped. Maximum error less than 5.17e-07.
#[inline(always)]
pub fn acos(x: f32) -> f32 {
    let negative_input = x < 0.0;
    let x = 1.0f32.min(x.abs());
    // Rational approximation (scaling sqrt(1-x)) over [0, 1], use symmetry for the rest. TODO: FMA would help with precision.
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

/// Computes an approximation of cosine. Maximum error a little below 8e-7 for the interval -2 * Pi to 2 * Pi.
/// Values further from the interval near zero have gracefully degrading error.
#[inline(always)]
pub fn cos_simd(x: Vector<f32>) -> Vector<f32> {
    // Rational approximation over [0, pi/2], use symmetry for the rest.
    let period_count = x * Vector::splat(0.5 / PI);
    let period_fraction = period_count - period_count.floor(); // This is a source of error as you get away from 0.
    let two_pi = Vector::splat(TWO_PI);
    let period_x = period_fraction * two_pi;

    // [0, pi/2] = f(x)
    // (pi/2, pi] = -f(Pi - x)
    // (pi, 3 * pi / 2] = -f(x - Pi)
    // (3*pi/2, 2*pi] = f(2 * Pi - x)
    let pi_over_2 = Vector::splat(FRAC_PI_2);
    let pi = Vector::splat(PI);
    let pi_3_over_2 = Vector::splat(3.0 * FRAC_PI_2);

    let mut y: Vector<f32> = Vector::simd_gt(period_x, pi_over_2).select(pi - period_x, period_x);
    y = Vector::simd_gt(period_x, pi).select(period_x - pi, y);
    y = Vector::simd_gt(period_x, pi_3_over_2).select(Vector::splat(TWO_PI) - period_x, y);

    // Using a fifth degree numerator and denominator.
    // This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
    // TODO: FMA could help here, primarily in terms of precision.
    // let y2 = y * y;
    // let y3 = y2 * y;
    // let y4 = y2 * y2;
    // let y5 = y3 * y2;
    // let numerator = Vector::<f32>::splat(1.0) - 0.15082367674208508 * y - 0.4578088075324152 * y2 + 0.06955843390178032 * y3 + 0.021317031205957775 * y4 - 0.003436308368583229 * y5;
    // let denominator = Vector::<f32>::splat(1.0) - 0.15082367538305258 * y + 0.04219116713777847 * y2 - 0.00585321045829395 * y3 + 0.0007451378206294365 * y4 - 0.00007650398834677185 * y5;
    let numerator = ((((Vector::splat(-0.003436308368583229_f32) * y
        + Vector::splat(0.021317031205957775_f32))
        * y
        + Vector::splat(0.06955843390178032_f32))
        * y
        - Vector::splat(0.4578088075324152_f32))
        * y
        - Vector::splat(0.15082367674208508_f32))
        * y
        + Vector::<f32>::splat(1.0);
    let denominator = ((((Vector::splat(-0.00007650398834677185_f32) * y
        + Vector::splat(0.0007451378206294365_f32))
        * y
        - Vector::splat(0.00585321045829395_f32))
        * y
        + Vector::splat(0.04219116713777847_f32))
        * y
        - Vector::splat(0.15082367538305258_f32))
        * y
        + Vector::<f32>::splat(1.0);
    let result = numerator / denominator;
    Vector::simd_gt(period_x, pi_over_2)
        .bitand(Vector::simd_lt(period_x, pi_3_over_2))
        .select(-result, result)
}

/// Computes an approximation of sine. Maximum error a little below 5e-7 for the interval -2 * Pi to 2 * Pi.
/// Values further from the interval near zero have gracefully degrading error.
#[inline(always)]
pub fn sin_simd(x: Vector<f32>) -> Vector<f32> {
    // Similar to cos, use a rational approximation for the region of sin from [0, pi/2]. Use symmetry to cover the rest.
    // This has its own implementation rather than just calling into Cos because we want maximum fidelity near 0.
    let period_count = x * Vector::splat(0.5 / PI);
    let period_fraction = period_count - period_count.floor(); // This is a source of error as you get away from 0.
    let two_pi = Vector::splat(TWO_PI);
    let period_x = period_fraction * two_pi;

    // [0, pi/2] = f(x)
    // (pi/2, pi] = f(pi - x)
    // (pi, 3/2 * pi] = -f(x - pi)
    // (3/2 * pi, 2*pi] = -f(2 * pi - x)
    let pi = Vector::splat(PI);
    let pi_over_2 = Vector::splat(FRAC_PI_2);

    let mut y: Vector<f32> = Vector::simd_gt(period_x, pi_over_2).select(pi - period_x, period_x);
    let in_second_half = Vector::simd_gt(period_x, pi);
    y = in_second_half.select(period_x - pi, y);
    y = Vector::simd_gt(period_x, Vector::splat(3.0 * FRAC_PI_2)).select(two_pi - period_x, y);

    // Using a fifth degree numerator and denominator.
    // This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
    // TODO: FMA could help here, primarily in terms of precision.
    // let y2 = y * y;
    // let y3 = y2 * y;
    // let y4 = y2 * y2;
    // let y5 = y3 * y2;
    // let numerator = 1.0000000015146604_f32 * y + 0.06174562337697123_f32 * y2 - 0.13993701695343166_f32 * y3 - 0.006685815219853882_f32 * y4 + 0.0040507708755727605_f32 * y5;
    // let denominator = Vector::<f32>::splat(1.0) + 0.061745651499203795_f32 * y + 0.02672943625500751_f32 * y2 + 0.003606014457152456_f32 * y3 + 0.0001700784176413186_f32 * y4 + 0.00009018370615921334_f32 * y5;
    let numerator = ((((Vector::splat(0.0040507708755727605_f32) * y
        - Vector::splat(0.006685815219853882_f32))
        * y
        - Vector::splat(0.13993701695343166_f32))
        * y
        + Vector::splat(0.06174562337697123_f32))
        * y
        + Vector::splat(1.00000000151466040_f32))
        * y;
    let denominator = ((((Vector::splat(0.00009018370615921334_f32) * y
        + Vector::splat(0.0001700784176413186_f32))
        * y
        + Vector::splat(0.003606014457152456_f32))
        * y
        + Vector::splat(0.02672943625500751_f32))
        * y
        + Vector::splat(0.061745651499203795_f32))
        * y
        + Vector::<f32>::splat(1.0);
    let result = numerator / denominator;
    in_second_half.select(-result, result)
}

/// Computes an approximation of arccos. Inputs outside of [-1, 1] are clamped.
/// Maximum error less than 5.17e-07.
#[inline(always)]
pub fn acos_simd(x: Vector<f32>) -> Vector<f32> {
    let negative_input = Vector::simd_lt(x, Vector::splat(0.0));
    let x = SimdFloat::simd_min(Vector::splat(1.0), SimdFloat::abs(x));

    // Rational approximation (scaling sqrt(1-x)) over [0, 1], use symmetry for the rest. TODO: FMA would help with precision.
    let numerator = Vector::sqrt(Vector::<f32>::splat(1.0) - x)
        * (Vector::splat(62.95741097600742_f32)
            + x * (Vector::splat(69.6550664543659_f32)
                + x * (Vector::splat(17.54512349463405_f32)
                    + x * Vector::splat(0.6022076120669532_f32))));
    let denominator = Vector::splat(40.07993264439811_f32)
        + x * (Vector::splat(49.81949855726789_f32)
            + x * (Vector::splat(15.703851745284796_f32) + x));
    let result = numerator / denominator;
    negative_input.select(Vector::splat(PI) - result, result)
}

/// Gets the change in angle from a to b as a signed value from -pi to pi.
#[inline(always)]
pub fn get_signed_angle_difference(a: &Vector<f32>, b: &Vector<f32>, difference: &mut Vector<f32>) {
    let half = Vector::splat(0.5_f32);
    let x = (b - a) * Vector::splat(1.0 / TWO_PI) + half;
    *difference = (x - x.floor() - half) * Vector::splat(TWO_PI);
}

#[inline(always)]
pub fn fast_reciprocal(v: Vector<f32>) -> Vector<f32> {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        if is_x86_feature_detected!("avx512f") {
            let v512 = _mm512_load_ps(v.as_ptr());
            let result512 = _mm512_rcp14_ps(v512);
            std::mem::transmute(result512)
        } else if is_x86_feature_detected!("avx") {
            let v256 = _mm256_castps128_ps256(_mm_load_ps(v.as_ptr()));
            let result256 = _mm256_rcp_ps(v256);
            let result128 = _mm256_castps256_ps128(result256);
            std::mem::transmute(result128)
        } else if is_x86_feature_detected!("sse") {
            let v128 = _mm_load_ps(v.as_ptr());
            let result128 = _mm_rcp_ps(v128);
            std::mem::transmute(result128)
        } else {
            Vector::<f32>::splat(1.0) / v
        }
    }
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let v_neon = std::arch::aarch64::vld1q_f32(v.as_array().as_ptr());
        let result_neon = std::arch::aarch64::vrecpeq_f32(v_neon);
        std::mem::transmute(result_neon)
    }
    #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
    {
        Vector::<f32>::splat(1.0) / v
    }
}

#[inline(always)]
pub fn fast_reciprocal_square_root(v: Vector<f32>) -> Vector<f32> {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        if is_x86_feature_detected!("avx512f") {
            let v512 = _mm512_load_ps(v.as_ptr());
            let result512 = _mm512_rsqrt14_ps(v512);
            std::mem::transmute(result512)
        } else if is_x86_feature_detected!("avx") {
            let v256 = _mm256_castps128_ps256(_mm_load_ps(v.as_ptr()));
            let result256 = _mm256_rsqrt_ps(v256);
            let result128 = _mm256_castps256_ps128(result256);
            std::mem::transmute(result128)
        } else if is_x86_feature_detected!("sse") {
            let v128 = _mm_load_ps(v.as_ptr());
            let result128 = _mm_rsqrt_ps(v128);
            std::mem::transmute(result128)
        } else {
            Vector::<f32>::splat(1.0) / v.sqrt()
        }
    }
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let v_neon = std::arch::aarch64::vld1q_f32(v.as_array().as_ptr());
        let result_neon = std::arch::aarch64::vrsqrteq_f32(v_neon);
        std::mem::transmute(result_neon)
    }
    #[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
    {
        Vector::<f32>::splat(1.0) / v.sqrt()
    }
}
