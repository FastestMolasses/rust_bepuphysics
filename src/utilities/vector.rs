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
        32
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
