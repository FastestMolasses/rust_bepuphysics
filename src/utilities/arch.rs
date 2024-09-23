// Define supported vector widths based on architecture
#[cfg(any(target_arch = "x86_64", target_arch = "x86"))]
mod arch {
    #[cfg(target_feature = "avx512f")]
    pub type VectorWidth = std::simd::f32x16;
    #[cfg(all(target_feature = "avx", not(target_feature = "avx512f")))]
    pub type VectorWidth = std::simd::f32x8;
    #[cfg(all(target_feature = "sse2", not(target_feature = "avx")))]
    pub type VectorWidth = std::simd::f32x4;
}

#[cfg(target_arch = "aarch64")]
mod arch {
    // ARM NEON supports 128-bit vectors (4 x f32)
    pub type VectorWidth = std::simd::f32x4;
}

#[cfg(target_arch = "arm")]
mod arch {
    // Older ARM architectures might only support smaller vectors
    pub type VectorWidth = std::simd::f32x4;
}

// For other architectures, default to f32x4
#[cfg(not(any(
    target_arch = "x86_64",
    target_arch = "x86",
    target_arch = "aarch64",
    target_arch = "arm"
)))]
mod arch {
    pub type VectorWidth = std::simd::f32x4;
}

pub use arch::VectorWidth;
