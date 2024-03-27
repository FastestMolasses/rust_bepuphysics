use glam::{IVec2, Vec2};
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};

/// Provides simple 2d cell hashing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Int2 {
    pub x: i32,
    pub y: i32,
}

impl Int2 {
    #[inline]
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

impl PartialOrd for Int2 {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Int2 {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        self.y.cmp(&other.y).then(self.x.cmp(&other.x))
    }
}

impl Hash for Int2 {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        const P1: u64 = 961748927;
        const P2: u64 = 899809343;
        let hash64 = self.x as u64 * (P1 * P2) + self.y as u64 * P2;
        (hash64 ^ (hash64 >> 32)).hash(state);
    }
}

impl From<Int2> for Vec2 {
    #[inline]
    fn from(value: Int2) -> Self {
        Vec2::new(value.x as f32, value.y as f32)
    }
}

impl From<Vec2> for Int2 {
    #[inline]
    fn from(value: Vec2) -> Self {
        Self {
            x: value.x as i32,
            y: value.y as i32,
        }
    }
}

impl From<Int2> for IVec2 {
    #[inline]
    fn from(value: Int2) -> Self {
        IVec2::new(value.x, value.y)
    }
}

impl From<IVec2> for Int2 {
    #[inline]
    fn from(value: IVec2) -> Self {
        Self {
            x: value.x,
            y: value.y,
        }
    }
}
