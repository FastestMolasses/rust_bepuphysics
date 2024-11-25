#![feature(portable_simd)]
// #![feature(effects)]
#![feature(generic_const_exprs)]

mod physics;
pub mod utilities;

// TODO: REPLACE is_x86_feature_detected WITH CFG, MAYBE OnceLock AND DO FEATURE DETECTION IN BUILD SCRIPT
// TODO: IMPLEMENT SVE INTRINSICS

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
/// This supports up to two out parameters. When 2 is specified, the macro returns a tuple.
#[macro_export]
macro_rules! out {
    // Single out parameter
    ($e:ident :: $method:ident ( $($arg:expr),* )) => {{
        let mut __result = std::mem::MaybeUninit::uninit();
        $e::$method($($arg,)* unsafe { &mut *(__result.as_mut_ptr()) });
        unsafe { __result.assume_init() }
    }};

    // Two out parameters
    ($e:ident :: $method:ident ( $($arg:expr),* ), 2) => {{
        let mut __result1 = std::mem::MaybeUninit::uninit();
        let mut __result2 = std::mem::MaybeUninit::uninit();
        $e::$method(
            $($arg,)*
            unsafe { &mut *(__result1.as_mut_ptr()) },
            unsafe { &mut *(__result2.as_mut_ptr()) }
        );
        (
            unsafe { __result1.assume_init() },
            unsafe { __result2.assume_init() }
        )
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
            $e::$method($($arg,)* &mut *(__result.as_mut_ptr()));
            __result.assume_init()
        }
    }};

    // Two out parameters
    ($e:ident :: $method:ident ( $($arg:expr),* ), 2) => {{
        let mut __result1 = std::mem::MaybeUninit::uninit();
        let mut __result2 = std::mem::MaybeUninit::uninit();
        unsafe {
            $e::$method(
                $($arg,)*
                &mut *(__result1.as_mut_ptr()),
                &mut *(__result2.as_mut_ptr())
            );
            (
                __result1.assume_init(),
                __result2.assume_init()
            )
        }
    }};
}
