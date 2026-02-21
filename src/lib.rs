#![feature(portable_simd)]
#![feature(generic_const_exprs)]
#![allow(unused_unsafe)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::new_without_default)]

pub mod physics;
pub mod utilities;

#[cfg(feature = "bevy")]
pub mod bevy_bepu;

// NOTE: Could replace is_x86_feature_detected with cfg, maybe OnceLock and do feature detection in build script.
// NOTE: SVE intrinsics not yet implemented for aarch64.
// NOTE: Tests for determinism across platforms not yet written.

/// Provides a zero-cost abstraction for out parameters similar to C#'s `out` keyword.
///
/// # Examples
/// ```
/// // Instead of:
/// let mut result = MaybeUninit::<Symmetric3x3Wide>::uninit();
/// Symmetric3x3Wide::scale(&self, rhs, unsafe { result.as_mut_ptr().as_mut().unwrap() });
/// let result = unsafe { result.assume_init() };
///
/// // You can write:
/// let result = out!(Symmetric3x3Wide::scale(&self, rhs));
/// ```
/// In C#, the equivalent would be:
/// ```
/// Symmetric3x3Wide.Scale(ref this, rhs, out var result);
/// ```
#[macro_export]
macro_rules! out {
    ($e:ident :: $method:ident ( $($arg:expr),* )) => {{
        let mut __result = std::mem::MaybeUninit::uninit();
        $e::$method($($arg,)* unsafe { &mut *(__result.as_mut_ptr()) });
        unsafe { __result.assume_init() }
    }};
}

/// Provides a zero-cost abstraction for out parameters similar to C#'s `out` keyword,
/// but wraps the function call in an unsafe block.
///
/// # Examples
/// ```
/// // For functions marked as unsafe:
/// let result = out_unsafe!(Symmetric3x3Wide::scale(&self, rhs));
/// ```
/// This supports up to two out parameters. When 2 is specified, the macro returns a tuple.
#[macro_export]
macro_rules! out_unsafe {
    // Single out parameter
    ($e:ident :: $method:ident ( $($arg:expr),* )) => {{
        let mut __result = std::mem::MaybeUninit::uninit();
        unsafe {
            $e::$method($($arg,)* unsafe { &mut *(__result.as_mut_ptr()) });
            __result.assume_init()
        }
    }};

    // Two out parameters
    ($e:ident :: $method:ident ( $($arg:expr),* ), 2) => {{
        let mut __result1 = std::mem::MaybeUninit::uninit();
        let mut __result2 = std::mem::MaybeUninit::uninit();
        // The call to the (unsafe) associated function is left to the caller's unsafe block.
        $e::$method(
            $($arg,)*
            unsafe { &mut *(__result1.as_mut_ptr()) },
            unsafe { &mut *(__result2.as_mut_ptr()) }
        );
        // assume_init is unsafe but only touches macro-local variables.
        unsafe {
            (
                __result1.assume_init(),
                __result2.assume_init()
            )
        }
    }};
}
