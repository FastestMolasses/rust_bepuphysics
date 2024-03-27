use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use std::mem;

#[derive(Clone, Copy, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Int3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Int3 {
    #[inline]
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

impl Hash for Int3 {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        const P1: u64 = 961748927;
        const P2: u64 = 899809343;
        const P3: u64 = 715225741;
        let hash64 = self.x as u64 * P1.wrapping_mul(P2).wrapping_mul(P3)
            + self.y as u64 * P2.wrapping_mul(P3)
            + self.z as u64 * P3;
        state.write_u64(hash64 ^ (hash64 >> 32));
    }
}

impl PartialOrd for Int3 {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match self.x.cmp(&other.x) {
            Ordering::Equal => {}
            ord => return Some(ord),
        }
        match self.y.cmp(&other.y) {
            Ordering::Equal => {}
            ord => return Some(ord),
        }
        Some(self.z.cmp(&other.z))
    }
}

impl Ord for Int3 {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

impl AsRef<Int3> for Int3 {
    #[inline]
    fn as_ref(&self) -> &Int3 {
        self
    }
}

impl AsMut<Int3> for Int3 {
    #[inline]
    fn as_mut(&mut self) -> &mut Int3 {
        self
    }
}

#[cfg(target_arch = "aarch64")]
#[cfg(target_feature = "neon")]
mod neon {
    use super::Int3;
    use core::arch::aarch64::*;

    #[inline]
    pub fn eq(a: Int3, b: Int3) -> bool {
        unsafe {
            let va = vld1q_s32(mem::transmute(&a));
            let vb = vld1q_s32(mem::transmute(&b));
            let vcmp = vceqq_s32(va, vb);
            let vand = vandq_u32(vreinterpretq_u32_s32(vcmp), vdupq_n_u32(0x80000000));
            vminvq_u32(vand) != 0
        }
    }

    #[inline]
    pub fn ne(a: Int3, b: Int3) -> bool {
        !eq(a, b)
    }

    #[inline]
    pub fn hash(item: &Int3) -> u64 {
        let vitem = unsafe { vld1q_s32(mem::transmute(item)) };
        let vp = vdupq_n_u64(961748927899809343715225741);
        let vhash = unsafe {
            vmlal_high_u32(
                vmlal_u32(
                    vmovq_n_u64(0),
                    vget_low_u32(vreinterpretq_u32_s32(vitem)),
                    vget_low_u32(vreinterpretq_u32_u64(vp)),
                ),
                vget_high_u32(vreinterpretq_u32_s32(vitem)),
                vget_high_u32(vreinterpretq_u32_u64(vp)),
            )
        };
        let result = vgetq_lane_u64(vhash, 0) ^ (vgetq_lane_u64(vhash, 1) >> 32);
        result as u64
    }
}

#[cfg(not(all(target_arch = "aarch64", target_feature = "neon")))]
mod fallback {
    use super::Int3;

    #[inline]
    pub fn eq(a: Int3, b: Int3) -> bool {
        a.x == b.x && a.y == b.y && a.z == b.z
    }

    #[inline]
    pub fn ne(a: Int3, b: Int3) -> bool {
        !eq(a, b)
    }

    #[inline]
    pub fn hash(item: &Int3) -> u64 {
        const P1: u64 = 961748927;
        const P2: u64 = 899809343;
        const P3: u64 = 715225741;
        let hash64 = item.x as u64 * P1.wrapping_mul(P2).wrapping_mul(P3)
            + item.y as u64 * P2.wrapping_mul(P3)
            + item.z as u64 * P3;
        hash64 ^ (hash64 >> 32)
    }
}

#[cfg(not(all(target_arch = "aarch64", target_feature = "neon")))]
use fallback::{eq, hash, ne};
#[cfg(target_arch = "aarch64")]
#[cfg(target_feature = "neon")]
use neon::{eq, hash, ne};

impl Int3 {
    #[inline]
    pub fn eq(&self, other: Self) -> bool {
        eq(*self, other)
    }

    #[inline]
    pub fn ne(&self, other: Self) -> bool {
        ne(*self, other)
    }

    #[inline]
    pub fn hash(&self) -> u64 {
        hash(self)
    }
}

impl core::fmt::Debug for Int3 {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Int3 {{ x: {}, y: {}, z: {} }}", self.x, self.y, self.z)
    }
}

impl core::fmt::Display for Int3 {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{{{}, {}, {}}}", self.x, self.y, self.z)
    }
}
