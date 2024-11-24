#[macro_use]
macro_rules! out {
    // Match function calls with arguments
    ($f:path ( $($arg:expr),* )) => {{
        let mut result = std::mem::MaybeUninit::uninit();
        unsafe {
            $f($($arg,)* &mut *result.as_mut_ptr());
            result.assume_init()
        }
    }};
    // Match method calls with arguments
    ($obj:expr . $method:ident ( $($arg:expr),* )) => {{
        let mut result = std::mem::MaybeUninit::uninit();
        unsafe {
            $obj.$method($($arg,)* &mut *result.as_mut_ptr());
            result.assume_init()
        }
    }};
}
