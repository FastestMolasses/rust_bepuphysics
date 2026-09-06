use glam::{Vec3, Vec4};
use std::simd::cmp::SimdPartialOrd;
use std::simd::{Select, Simd};

#[cfg(all(target_arch = "aarch64", target_feature = "neon"))]
use std::arch::aarch64 as arch_simd;
#[cfg(all(target_arch = "x86", target_feature = "sse"))]
use std::arch::x86 as arch_simd;
#[cfg(all(target_arch = "x86_64", target_feature = "sse"))]
use std::arch::x86_64 as arch_simd;

#[cfg(target_arch = "x86_64")]
const fn preferred_byte_size() -> usize {
    #[cfg(target_feature = "avx512f")]
    {
        64
    }
    #[cfg(all(target_feature = "avx2", not(target_feature = "avx512f")))]
    {
        32
    }
    #[cfg(all(
        target_feature = "avx",
        not(any(target_feature = "avx2", target_feature = "avx512f"))
    ))]
    {
        16
    }
    #[cfg(all(
        target_feature = "sse4.2",
        not(any(
            target_feature = "avx",
            target_feature = "avx2",
            target_feature = "avx512f"
        ))
    ))]
    {
        16
    }
    #[cfg(all(
        target_feature = "sse4.1",
        not(any(
            target_feature = "sse4.2",
            target_feature = "avx",
            target_feature = "avx2",
            target_feature = "avx512f"
        ))
    ))]
    {
        16
    }
    #[cfg(all(
        target_feature = "ssse3",
        not(any(
            target_feature = "sse4.1",
            target_feature = "sse4.2",
            target_feature = "avx",
            target_feature = "avx2",
            target_feature = "avx512f"
        ))
    ))]
    {
        16
    }
    #[cfg(all(
        target_feature = "sse3",
        not(any(
            target_feature = "ssse3",
            target_feature = "sse4.1",
            target_feature = "sse4.2",
            target_feature = "avx",
            target_feature = "avx2",
            target_feature = "avx512f"
        ))
    ))]
    {
        16
    }
    #[cfg(all(
        target_feature = "sse2",
        not(any(
            target_feature = "sse3",
            target_feature = "ssse3",
            target_feature = "sse4.1",
            target_feature = "sse4.2",
            target_feature = "avx",
            target_feature = "avx2",
            target_feature = "avx512f"
        ))
    ))]
    {
        16
    }
    #[cfg(not(any(
        target_feature = "sse2",
        target_feature = "sse3",
        target_feature = "ssse3",
        target_feature = "sse4.1",
        target_feature = "sse4.2",
        target_feature = "avx",
        target_feature = "avx2",
        target_feature = "avx512f"
    )))]
    {
        8
    }
}

#[cfg(target_arch = "aarch64")]
const fn preferred_byte_size() -> usize {
    #[cfg(all(target_feature = "neon", target_feature = "sve2"))]
    {
        64
    }
    #[cfg(all(
        target_feature = "neon",
        target_feature = "sve",
        not(target_feature = "sve2")
    ))]
    {
        32
    }
    #[cfg(all(
        target_feature = "neon",
        not(any(target_feature = "sve", target_feature = "sve2"))
    ))]
    {
        16
    }
    #[cfg(not(target_feature = "neon"))]
    {
        8
    }
}

#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
const fn preferred_byte_size() -> usize {
    16
}

pub const fn optimal_lanes<T>() -> usize {
    const fn max(a: usize, b: usize) -> usize {
        if a > b {
            a
        } else {
            b
        }
    }
    max(preferred_byte_size() / std::mem::size_of::<T>(), 2)
}

pub type Vector<T> = std::simd::Simd<T, { optimal_lanes::<T>() }>;

/// The number of f32 lanes in a SIMD vector (equivalent to C# `Vector<float>.Count`).
pub const VECTOR_WIDTH: usize = optimal_lanes::<f32>();

/// Min/max with the semantics of the `minps`/`maxps` instructions the .NET 8 JIT emits for
/// `Vector.Min`/`Vector.Max`/`Vector3.Min`/`Vector4.Max`: an ordered compare and a select, so a
/// NaN in either operand, and a `+0.0`/`-0.0` tie, both yield the *second* operand. `f32::min`
/// and `Simd::simd_min` instead implement IEEE `minNum`, which costs extra NaN-fixup
/// instructions and disagrees with the C# in those lanes.
///
/// On aarch64 these lower to `FMIN`/`FMAX`, which propagate NaN and order `-0.0` below `+0.0`;
/// that is the same per-platform divergence the .NET JIT has, so the C# is still matched there.
pub trait HwMinMax {
    /// `if self < other { self } else { other }`; NaN in either operand yields `other`.
    fn hw_min(self, other: Self) -> Self;
    /// `if self > other { self } else { other }`; NaN in either operand yields `other`.
    fn hw_max(self, other: Self) -> Self;
}

/// Fallback for widths with no single-instruction min; identical semantics to `minps`.
#[inline(always)]
fn portable_min<const N: usize>(a: Simd<f32, N>, b: Simd<f32, N>) -> Simd<f32, N> {
    a.simd_lt(b).select(a, b)
}

/// Fallback for widths with no single-instruction max; identical semantics to `maxps`.
#[inline(always)]
fn portable_max<const N: usize>(a: Simd<f32, N>, b: Simd<f32, N>) -> Simd<f32, N> {
    a.simd_gt(b).select(a, b)
}

impl HwMinMax for f32 {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        if self < other {
            self
        } else {
            other
        }
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        if self > other {
            self
        } else {
            other
        }
    }
}

impl HwMinMax for Simd<f32, 2> {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        portable_min(self, other)
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        portable_max(self, other)
    }
}

impl HwMinMax for Simd<f32, 4> {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "sse"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm_min_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(all(target_arch = "aarch64", target_feature = "neon"))]
        {
            unsafe {
                std::mem::transmute(arch_simd::vminq_f32(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(any(
            all(
                any(target_arch = "x86", target_arch = "x86_64"),
                target_feature = "sse"
            ),
            all(target_arch = "aarch64", target_feature = "neon")
        )))]
        {
            portable_min(self, other)
        }
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "sse"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm_max_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(all(target_arch = "aarch64", target_feature = "neon"))]
        {
            unsafe {
                std::mem::transmute(arch_simd::vmaxq_f32(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(any(
            all(
                any(target_arch = "x86", target_arch = "x86_64"),
                target_feature = "sse"
            ),
            all(target_arch = "aarch64", target_feature = "neon")
        )))]
        {
            portable_max(self, other)
        }
    }
}

impl HwMinMax for Simd<f32, 8> {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm256_min_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx"
        )))]
        {
            portable_min(self, other)
        }
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm256_max_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx"
        )))]
        {
            portable_max(self, other)
        }
    }
}

impl HwMinMax for Simd<f32, 16> {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx512f"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm512_min_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx512f"
        )))]
        {
            portable_min(self, other)
        }
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        #[cfg(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx512f"
        ))]
        {
            unsafe {
                std::mem::transmute(arch_simd::_mm512_max_ps(
                    std::mem::transmute(self),
                    std::mem::transmute(other),
                ))
            }
        }
        #[cfg(not(all(
            any(target_arch = "x86", target_arch = "x86_64"),
            target_feature = "avx512f"
        )))]
        {
            portable_max(self, other)
        }
    }
}

impl HwMinMax for Vec3 {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        Vec3::new(
            self.x.hw_min(other.x),
            self.y.hw_min(other.y),
            self.z.hw_min(other.z),
        )
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        Vec3::new(
            self.x.hw_max(other.x),
            self.y.hw_max(other.y),
            self.z.hw_max(other.z),
        )
    }
}

impl HwMinMax for Vec4 {
    #[inline(always)]
    fn hw_min(self, other: Self) -> Self {
        Vec4::from_array(
            Simd::<f32, 4>::from_array(self.to_array())
                .hw_min(Simd::<f32, 4>::from_array(other.to_array()))
                .to_array(),
        )
    }

    #[inline(always)]
    fn hw_max(self, other: Self) -> Self {
        Vec4::from_array(
            Simd::<f32, 4>::from_array(self.to_array())
                .hw_max(Simd::<f32, 4>::from_array(other.to_array()))
                .to_array(),
        )
    }
}

#[cfg(test)]
mod hw_min_max_tests {
    use super::*;

    #[test]
    fn scalar_nan_yields_second_operand() {
        assert_eq!(f32::NAN.hw_min(1.0), 1.0);
        assert!(1.0f32.hw_min(f32::NAN).is_nan());
        assert_eq!(f32::NAN.hw_max(1.0), 1.0);
        assert!(1.0f32.hw_max(f32::NAN).is_nan());
    }

    #[test]
    fn scalar_signed_zero_tie_yields_second_operand() {
        assert!(0.0f32.hw_min(-0.0).is_sign_negative());
        assert!((-0.0f32).hw_min(0.0).is_sign_positive());
        assert!(0.0f32.hw_max(-0.0).is_sign_negative());
        assert!((-0.0f32).hw_max(0.0).is_sign_positive());
    }

    #[test]
    fn portable_fallback_matches_minps_semantics() {
        let nan = Simd::<f32, 2>::splat(f32::NAN);
        let ones = Simd::<f32, 2>::splat(1.0);
        assert_eq!(nan.hw_min(ones), ones);
        assert!(ones.hw_min(nan)[0].is_nan());
        assert_eq!(nan.hw_max(ones), ones);
        assert!(ones.hw_max(nan)[0].is_nan());
        let positive = Simd::<f32, 2>::splat(0.0);
        let negative = Simd::<f32, 2>::splat(-0.0);
        assert!(positive.hw_min(negative)[0].is_sign_negative());
        assert!(negative.hw_min(positive)[0].is_sign_positive());
        assert!(positive.hw_max(negative)[0].is_sign_negative());
        assert!(negative.hw_max(positive)[0].is_sign_positive());
    }

    #[cfg(not(all(target_arch = "aarch64", target_feature = "neon")))]
    #[test]
    fn vector_nan_yields_second_operand() {
        let nan = Simd::<f32, 4>::splat(f32::NAN);
        let ones = Simd::<f32, 4>::splat(1.0);
        assert_eq!(nan.hw_min(ones), ones);
        assert!(ones.hw_min(nan)[0].is_nan());
        assert_eq!(nan.hw_max(ones), ones);
        assert!(ones.hw_max(nan)[0].is_nan());
    }

    #[cfg(not(all(target_arch = "aarch64", target_feature = "neon")))]
    #[test]
    fn vector_signed_zero_tie_yields_second_operand() {
        let positive = Simd::<f32, 4>::splat(0.0);
        let negative = Simd::<f32, 4>::splat(-0.0);
        assert!(positive.hw_min(negative)[0].is_sign_negative());
        assert!(negative.hw_min(positive)[0].is_sign_positive());
        assert!(positive.hw_max(negative)[0].is_sign_negative());
        assert!(negative.hw_max(positive)[0].is_sign_positive());
    }

    #[cfg(all(target_arch = "aarch64", target_feature = "neon"))]
    #[test]
    fn vector_fmin_propagates_nan_and_orders_negative_zero_first() {
        let nan = Simd::<f32, 4>::splat(f32::NAN);
        let ones = Simd::<f32, 4>::splat(1.0);
        assert!(nan.hw_min(ones)[0].is_nan());
        assert!(ones.hw_min(nan)[0].is_nan());
        let positive = Simd::<f32, 4>::splat(0.0);
        let negative = Simd::<f32, 4>::splat(-0.0);
        assert!(positive.hw_min(negative)[0].is_sign_negative());
        assert!(negative.hw_min(positive)[0].is_sign_negative());
        assert!(positive.hw_max(negative)[0].is_sign_positive());
        assert!(negative.hw_max(positive)[0].is_sign_positive());
    }

    #[test]
    fn ordinary_values_match_min_max() {
        assert_eq!(2.0f32.hw_min(3.0), 2.0);
        assert_eq!(3.0f32.hw_min(2.0), 2.0);
        assert_eq!(2.0f32.hw_max(3.0), 3.0);
        assert_eq!(3.0f32.hw_max(2.0), 3.0);
        assert_eq!(
            Vec3::new(1.0, 5.0, -3.0).hw_min(Vec3::new(4.0, 2.0, 0.0)),
            Vec3::new(1.0, 2.0, -3.0)
        );
        assert_eq!(
            Vec4::new(1.0, 5.0, -3.0, 7.0).hw_max(Vec4::new(4.0, 2.0, 0.0, 6.0)),
            Vec4::new(4.0, 5.0, 0.0, 7.0)
        );
    }
}
