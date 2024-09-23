use std::hash::{Hash, Hasher};
use std::mem;

/// A set of 4 integers, useful for spatial hashing.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct Int4 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
}

impl Int4 {
    #[inline]
    pub fn new(x: i32, y: i32, z: i32, w: i32) -> Self {
        Int4 { x, y, z, w }
    }
}

impl PartialEq for Int4 {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z && self.w == other.w
    }
}

impl Eq for Int4 {}

impl Hash for Int4 {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        unsafe {
            let p1 = mem::transmute::<u64, m64>(961748927);
            let p2 = mem::transmute::<u64, m64>(899809343);
            let p3 = mem::transmute::<u64, m64>(715225741);
            let p4 = mem::transmute::<u64, m64>(472882027);

            let x = mem::transmute::<i32, m32>(self.x);
            let y = mem::transmute::<i32, m32>(self.y);
            let z = mem::transmute::<i32, m32>(self.z);
            let w = mem::transmute::<i32, m32>(self.w);

            let hash64 = (x.to_u64() * (p1 * p2 * p3))
                + (y.to_u64() * (p2 * p3))
                + (z.to_u64() * p3)
                + (w.to_u64() * p4);
            state.write_u64(hash64 ^ (hash64 >> 32));
        }
    }
}
