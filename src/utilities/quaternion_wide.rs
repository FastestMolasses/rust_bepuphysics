use portable_simd::*;
use std::ops::{Add, Mul, Neg, Sub};

use crate::utilities::math_helper::MathHelper;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
// use crate::utilities::quaternion::Quaternion;
use crate::utilities::vector3_wide::Vector3Wide;

#[derive(Clone, Copy)]
pub struct QuaternionWide {
    pub x: f32x8,
    pub y: f32x8,
    pub z: f32x8,
    pub w: f32x8,
}

impl QuaternionWide {
    #[inline(always)]
    pub fn broadcast(source: Quaternion) -> Self {
        Self {
            x: f32x8::splat(source.x),
            y: f32x8::splat(source.y),
            z: f32x8::splat(source.z),
            w: f32x8::splat(source.w),
        }
    }

    #[inline(always)]
    pub fn rebroadcast(source: &Self, slot_index: usize) -> Self {
        Self {
            x: f32x8::splat(source.x.extract(slot_index)),
            y: f32x8::splat(source.y.extract(slot_index)),
            z: f32x8::splat(source.z.extract(slot_index)),
            w: f32x8::splat(source.w.extract(slot_index)),
        }
    }

    #[inline(always)]
    pub fn create_from_rotation_matrix(r: &Matrix3x3Wide) -> Self {
        let one_add_x = f32x8::splat(1.0) + r.x.x;
        let one_sub_x = f32x8::splat(1.0) - r.x.x;
        let y_add_z = r.y.y + r.z.z;
        let y_sub_z = r.y.y - r.z.z;
        let t_x = one_add_x - y_add_z;
        let t_y = one_sub_x + y_sub_z;
        let t_z = one_sub_x - y_sub_z;
        let t_w = one_add_x + y_add_z;

        let use_upper = r.z.z.lt(f32x8::splat(0.0));
        let use_upper_upper = r.x.x.gt(r.y.y);
        let use_lower_upper = r.x.x.lt(-r.y.y);
        let t = use_upper.select(
            use_upper_upper.select(t_x, t_y),
            use_lower_upper.select(t_z, t_w),
        );
        let xy_add_yx = r.x.y + r.y.x;
        let yz_sub_zy = r.y.z - r.z.y;
        let zx_add_xz = r.z.x + r.x.z;
        let x = use_upper.select(
            use_upper_upper.select(t_x, xy_add_yx),
            use_lower_upper.select(zx_add_xz, yz_sub_zy),
        );
        let yz_add_zy = r.y.z + r.z.y;
        let zx_sub_xz = r.z.x - r.x.z;
        let y = use_upper.select(
            use_upper_upper.select(xy_add_yx, t_y),
            use_lower_upper.select(yz_add_zy, zx_sub_xz),
        );
        let xy_sub_yx = r.x.y - r.y.x;
        let z = use_upper.select(
            use_upper_upper.select(zx_add_xz, yz_add_zy),
            use_lower_upper.select(t_z, xy_sub_yx),
        );
        let w = use_upper.select(
            use_upper_upper.select(yz_sub_zy, zx_sub_xz),
            use_lower_upper.select(xy_sub_yx, t_w),
        );

        let scale = f32x8::splat(0.5) / t.sqrt();
        Self { x, y, z, w }.scale(scale)
    }

    #[inline(always)]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            w: self.w + other.w,
        }
    }

    #[inline(always)]
    pub fn scale(&self, scale: f32x8) -> Self {
        Self {
            x: self.x * scale,
            y: self.y * scale,
            z: self.z * scale,
            w: self.w * scale,
        }
    }

    #[inline(always)]
    pub fn length_squared(&self) -> f32x8 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    #[inline(always)]
    pub fn length(&self) -> f32x8 {
        self.length_squared().sqrt()
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let inverse_norm = f32x8::splat(1.0) / self.length();
        self.scale(inverse_norm)
    }

    #[inline(always)]
    pub fn negate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: -self.w,
        }
    }

    #[inline(always)]
    pub fn get_quaternion_between_normalized_vectors(v1: &Vector3Wide, v2: &Vector3Wide) -> Self {
        let dot = Vector3Wide::dot(v1, v2);
        let cross = Vector3Wide::cross(v1, v2);
        let use_normal_case = dot.gt(f32x8::splat(-0.999999));
        let abs_x = v1.x.abs();
        let abs_y = v1.y.abs();
        let abs_z = v1.z.abs();
        let x_is_smallest = abs_x.lt(abs_y) & abs_x.lt(abs_z);
        let y_is_smaller = abs_y.lt(abs_z);
        let x = use_normal_case.select(
            cross.x,
            x_is_smallest.select(f32x8::splat(0.0), y_is_smaller.select(-v1.z, -v1.y)),
        );
        let y = use_normal_case.select(
            cross.y,
            x_is_smallest.select(-v1.z, y_is_smaller.select(f32x8::splat(0.0), v1.x)),
        );
        let z = use_normal_case.select(
            cross.z,
            x_is_smallest.select(v1.y, y_is_smaller.select(v1.x, f32x8::splat(0.0))),
        );
        let w = use_normal_case.select(dot + f32x8::splat(1.0), f32x8::splat(0.0));

        Self { x, y, z, w }.normalize()
    }

    #[inline(always)]
    pub fn get_axis_angle(&self) -> (Vector3Wide, f32x8) {
        let should_negate = self.w.lt(f32x8::splat(0.0));
        let axis = Vector3Wide {
            x: should_negate.select(-self.x, self.x),
            y: should_negate.select(-self.y, self.y),
            z: should_negate.select(-self.z, self.z),
        };
        let qw = should_negate.select(-self.w, self.w);

        let axis_length = axis.length();
        let normalized_axis = axis.scale(f32x8::splat(1.0) / axis_length);
        let use_fallback = axis_length.lt(f32x8::splat(1e-14));
        let final_axis = Vector3Wide {
            x: use_fallback.select(f32x8::splat(1.0), normalized_axis.x),
            y: use_fallback.select(f32x8::splat(0.0), normalized_axis.y),
            z: use_fallback.select(f32x8::splat(0.0), normalized_axis.z),
        };
        let half_angle = MathHelper::acos(qw);
        let angle = f32x8::splat(2.0) * half_angle;

        (final_axis, angle)
    }

    #[inline(always)]
    pub fn transform(&self, v: &Vector3Wide) -> Vector3Wide {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx2 = self.x * x2;
        let xy2 = self.x * y2;
        let xz2 = self.x * z2;
        let yy2 = self.y * y2;
        let yz2 = self.y * z2;
        let zz2 = self.z * z2;
        let wx2 = self.w * x2;
        let wy2 = self.w * y2;
        let wz2 = self.w * z2;

        Vector3Wide {
            x: v.x * (f32x8::splat(1.0) - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
            y: v.x * (xy2 + wz2) + v.y * (f32x8::splat(1.0) - xx2 - zz2) + v.z * (yz2 - wx2),
            z: v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (f32x8::splat(1.0) - xx2 - yy2),
        }
    }

    #[inline(always)]
    pub fn transform_by_conjugate(&self, v: &Vector3Wide) -> Vector3Wide {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx2 = self.x * x2;
        let xy2 = self.x * y2;
        let xz2 = self.x * z2;
        let yy2 = self.y * y2;
        let yz2 = self.y * z2;
        let zz2 = self.z * z2;
        let nw = -self.w;
        let wx2 = nw * x2;
        let wy2 = nw * y2;
        let wz2 = nw * z2;

        Vector3Wide {
            x: v.x * (f32x8::splat(1.0) - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
            y: v.x * (xy2 + wz2) + v.y * (f32x8::splat(1.0) - xx2 - zz2) + v.z * (yz2 - wx2),
            z: v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (f32x8::splat(1.0) - xx2 - yy2),
        }
    }

    #[inline(always)]
    pub fn transform_unit_x(&self) -> Vector3Wide {
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xy2 = self.x * y2;
        let xz2 = self.x * z2;
        let yy2 = self.y * y2;
        let zz2 = self.z * z2;
        let wy2 = self.w * y2;
        let wz2 = self.w * z2;

        Vector3Wide {
            x: f32x8::splat(1.0) - yy2 - zz2,
            y: xy2 + wz2,
            z: xz2 - wy2,
        }
    }

    #[inline(always)]
    pub fn transform_unit_y(&self) -> Vector3Wide {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx2 = self.x * x2;
        let xy2 = self.x * y2;
        let yz2 = self.y * z2;
        let zz2 = self.z * z2;
        let wx2 = self.w * x2;
        let wz2 = self.w * z2;

        Vector3Wide {
            x: xy2 - wz2,
            y: f32x8::splat(1.0) - xx2 - zz2,
            z: yz2 + wx2,
        }
    }

    #[inline(always)]
    pub fn transform_unit_z(&self) -> Vector3Wide {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx2 = self.x * x2;
        let xz2 = self.x * z2;
        let yy2 = self.y * y2;
        let yz2 = self.y * z2;
        let wx2 = self.w * x2;
        let wy2 = self.w * y2;

        Vector3Wide {
            x: xz2 + wy2,
            y: yz2 - wx2,
            z: f32x8::splat(1.0) - xx2 - yy2,
        }
    }

    #[inline(always)]
    pub fn transform_unit_xy(&self) -> (Vector3Wide, Vector3Wide) {
        let x2 = self.x + self.x;
        let y2 = self.y + self.y;
        let z2 = self.z + self.z;
        let xx2 = self.x * x2;
        let xy2 = self.x * y2;
        let xz2 = self.x * z2;
        let yy2 = self.y * y2;
        let yz2 = self.y * z2;
        let zz2 = self.z * z2;
        let wx2 = self.w * x2;
        let wy2 = self.w * y2;
        let wz2 = self.w * z2;

        let x = Vector3Wide {
            x: f32x8::splat(1.0) - yy2 - zz2,
            y: xy2 + wz2,
            z: xz2 - wy2,
        };

        let y = Vector3Wide {
            x: xy2 - wz2,
            y: f32x8::splat(1.0) - xx2 - zz2,
            z: yz2 + wx2,
        };

        (x, y)
    }

    #[inline(always)]
    pub fn transform_unit_xz(&self) -> (Vector3Wide, Vector3Wide) {
        let q_x2 = self.x + self.x;
        let q_y2 = self.y + self.y;
        let q_z2 = self.z + self.z;

        let yy = q_y2 * self.y;
        let zz = q_z2 * self.z;
        let xy = q_x2 * self.y;
        let zw = q_z2 * self.w;
        let xz = q_x2 * self.z;
        let yw = q_y2 * self.w;

        let x = Vector3Wide {
            x: f32x8::splat(1.0) - yy - zz,
            y: xy + zw,
            z: xz - yw,
        };

        let xx = q_x2 * self.x;
        let xw = q_x2 * self.w;
        let yz = q_y2 * self.z;

        let z = Vector3Wide {
            x: xz + yw,
            y: yz - xw,
            z: f32x8::splat(1.0) - xx - yy,
        };

        (x, z)
    }

    #[inline(always)]
    pub fn concatenate(&self, other: &Self) -> Self {
        Self {
            x: self.w * other.x + self.x * other.w + self.z * other.y - self.y * other.z,
            y: self.w * other.y + self.y * other.w + self.x * other.z - self.z * other.x,
            z: self.w * other.z + self.z * other.w + self.y * other.x - self.x * other.y,
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        }
    }

    #[inline(always)]
    pub fn conjugate(&self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    #[inline(always)]
    pub fn conditional_select(condition: m32x8, left: &Self, right: &Self) -> Self {
        Self {
            x: condition.select(left.x, right.x),
            y: condition.select(left.y, right.y),
            z: condition.select(left.z, right.z),
            w: condition.select(left.w, right.w),
        }
    }

    #[inline(always)]
    pub fn read_first(&self) -> Quaternion {
        Quaternion {
            x: self.x.extract(0),
            y: self.y.extract(0),
            z: self.z.extract(0),
            w: self.w.extract(0),
        }
    }

    #[inline(always)]
    pub fn write_first(&mut self, source: &Quaternion) {
        self.x.replace(0, source.x);
        self.y.replace(0, source.y);
        self.z.replace(0, source.z);
        self.w.replace(0, source.w);
    }

    #[inline(always)]
    pub fn write_slot(&mut self, source: &Quaternion, slot_index: usize) {
        self.x.replace(slot_index, source.x);
        self.y.replace(slot_index, source.y);
        self.z.replace(slot_index, source.z);
        self.w.replace(slot_index, source.w);
    }

    #[inline(always)]
    pub fn read_slot(&self, slot_index: usize) -> Quaternion {
        Quaternion {
            x: self.x.extract(slot_index),
            y: self.y.extract(slot_index),
            z: self.z.extract(slot_index),
            w: self.w.extract(slot_index),
        }
    }
}

impl Add for QuaternionWide {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self {
        self.add(&other)
    }
}

impl Mul for QuaternionWide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, other: Self) -> Self {
        self.concatenate(&other)
    }
}

impl Mul<Vector3Wide> for QuaternionWide {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, other: Vector3Wide) -> Vector3Wide {
        self.transform(&other)
    }
}

impl Neg for QuaternionWide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self {
        self.negate()
    }
}
