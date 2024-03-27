use packed_simd::f32x4;

#[derive(Copy, Clone, Debug)]
pub struct Vector3(f32x4);

impl Vector3 {
    /// Constructs a new `Vector3`.
    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self(f32x4::new(x, y, z, 0.0))
    }

    /// Computes the dot product of two vectors.
    #[inline(always)]
    pub fn dot(self, other: Self) -> f32 {
        let mul = self.0 * other.0;
        mul.extract(0) + mul.extract(1) + mul.extract(2)
    }

    /// Computes the cross product of two vectors.
    #[inline(always)]
    pub fn cross(self, other: Self) -> Self {
        let shuf_a = f32x4::new(self.0.extract(1), self.0.extract(2), self.0.extract(0), 0.0);
        let shuf_b = f32x4::new(
            other.0.extract(1),
            other.0.extract(2),
            other.0.extract(0),
            0.0,
        );
        let cross = shuf_a * other.0 - self.0 * shuf_b;
        Vector3(f32x4::new(
            cross.extract(1),
            cross.extract(2),
            cross.extract(0),
            0.0,
        ))
    }

    /// Scales the vector by a scalar.
    #[inline(always)]
    pub fn scale(self, scale: f32) -> Self {
        Self(self.0 * scale)
    }
}

impl std::ops::Mul<f32> for Vector3 {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f32) -> Self::Output {
        self.scale(scalar)
    }
}

impl std::ops::Add for Vector3 {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        Vector3(self.0 + other.0)
    }
}

impl std::ops::Sub for Vector3 {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        Vector3(self.0 - other.0)
    }
}
