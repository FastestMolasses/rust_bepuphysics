#![feature(portable_simd)]
#![feature(effects)]
#![feature(generic_const_exprs)]

// mod private_macros;
mod physics;
mod utilities;

pub use utilities::*;

#[macro_export]
macro_rules! out {
    ($($e:tt)*) => {{
        let mut result = std::mem::MaybeUninit::uninit();
        unsafe {
            $($e)*(unsafe { &mut *result.as_mut_ptr() });
            result.assume_init()
        }
    }};
}
