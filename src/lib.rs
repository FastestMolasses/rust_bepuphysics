#![feature(portable_simd)]
#![feature(effects)]
#![feature(generic_const_exprs)]

mod physics;
mod utilities;

pub use utilities::*;

// TODO: DOES is_x86_feature_detected GET COMPILED OUT?

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
#[macro_export]
macro_rules! out_unsafe {
    ($e:ident :: $method:ident ( $($arg:expr),* )) => {{
        let mut __result = std::mem::MaybeUninit::uninit();
        unsafe {
            $e::$method($($arg,)* &mut *(__result.as_mut_ptr()));
            __result.assume_init()
        }
    }};
}
