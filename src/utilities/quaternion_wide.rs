use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::math_helper;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use crate::{out, out_unsafe};
use glam::Quat;
use std::ops::BitAnd;
use std::simd::cmp::SimdPartialOrd;
use std::simd::num::SimdFloat;
use std::simd::{Mask, StdFloat};

#[derive(Clone, Copy)]
pub struct QuaternionWide {
    pub x: Vector<f32>,
    pub y: Vector<f32>,
    pub z: Vector<f32>,
    pub w: Vector<f32>,
}

impl QuaternionWide {
    #[inline(always)]
    pub fn broadcast(source: Quaternion, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x);
        broadcasted.y = Vector::splat(source.y);
        broadcasted.z = Vector::splat(source.z);
        broadcasted.w = Vector::splat(source.w);
    }

    /// Takes a slot from the source quaternion and broadcasts it into all slots of the target quaternion.
    #[inline(always)]
    pub fn rebroadcast(source: &Self, slot_index: usize, broadcasted: &mut Self) {
        broadcasted.x = Vector::splat(source.x[slot_index]);
        broadcasted.y = Vector::splat(source.y[slot_index]);
        broadcasted.z = Vector::splat(source.z[slot_index]);
        broadcasted.w = Vector::splat(source.w[slot_index]);
    }

    /// Constructs a quaternion from a rotation matrix.
    #[inline(always)]
    pub fn create_from_rotation_matrix(r: &Matrix3x3Wide, q: &mut Self) {
        // Since we can't branch, we're going to end up calculating the possible states of all branches.
        // This requires doing more ALU work than the branching implementation, but there are a lot of common terms across the branches, and (random-ish) branches aren't free.
        // Overall, this turns out to be about 2x-2.5x more expensive per call than the scalar version, but it handles multiple lanes, so it's a net win.
        let one_add_x = Vector::splat(1.0) + r.x.x;
        let one_sub_x = Vector::splat(1.0) - r.x.x;
        let y_add_z = r.y.y + r.z.z;
        let y_sub_z = r.y.y - r.z.z;
        let t_x = one_add_x - y_add_z;
        let t_y = one_sub_x + y_sub_z;
        let t_z = one_sub_x - y_sub_z;
        let t_w = one_add_x + y_add_z;

        // There are two layers of conditions- inner, and outer. We have to first select each of the two inner halves- upper, and lower-
        // and then we will select which of the two inners to use for the outer.
        let use_upper = Vector::simd_lt(r.z.z, Vector::<f32>::splat(0.0));
        let use_upper_upper = Vector::simd_gt(r.x.x, r.y.y);
        let use_lower_upper = Vector::simd_lt(r.x.x, -r.y.y);
        let t = use_upper.select(
            use_upper_upper.select(t_x, t_y),
            use_lower_upper.select(t_z, t_w),
        );
        let xy_add_yx = r.x.y + r.y.x;
        let yz_sub_zy = r.y.z - r.z.y;
        let zx_add_xz = r.z.x + r.x.z;
        q.x = use_upper.select(
            use_upper_upper.select(t_x, xy_add_yx),
            use_lower_upper.select(zx_add_xz, yz_sub_zy),
        );
        let yz_add_zy = r.y.z + r.z.y;
        let zx_sub_xz = r.z.x - r.x.z;
        q.y = use_upper.select(
            use_upper_upper.select(xy_add_yx, t_y),
            use_lower_upper.select(yz_add_zy, zx_sub_xz),
        );
        let xy_sub_yx = r.x.y - r.y.x;
        q.z = use_upper.select(
            use_upper_upper.select(zx_add_xz, yz_add_zy),
            use_lower_upper.select(t_z, xy_sub_yx),
        );
        q.w = use_upper.select(
            use_upper_upper.select(yz_sub_zy, zx_sub_xz),
            use_lower_upper.select(xy_sub_yx, t_w),
        );

        let scale = Vector::<f32>::splat(0.5) / Vector::sqrt(t);
        *q = out!(Self::scale(q, scale));
    }

    /// Adds the components of two quaternions together.
    #[inline(always)]
    pub fn add(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.x + b.x;
        result.y = a.y + b.y;
        result.z = a.z + b.z;
        result.w = a.w + b.w;
    }

    #[inline(always)]
    pub fn scale(q: &Self, scale: Vector<f32>, result: &mut Self) {
        result.x = q.x * scale;
        result.y = q.y * scale;
        result.z = q.z * scale;
        result.w = q.w * scale;
    }

    #[inline(always)]
    pub fn length_squared(&self) -> Vector<f32> {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    #[inline(always)]
    pub fn length(&self) -> Vector<f32> {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    #[inline(always)]
    pub fn normalize(q: Self) -> Self {
        // TODO: fast path is possible with intrinsics.
        let inverse_norm =
            Vector::<f32>::splat(1.0) / (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w).sqrt();
        Self {
            x: q.x * inverse_norm,
            y: q.y * inverse_norm,
            z: q.z * inverse_norm,
            w: q.w * inverse_norm,
        }
    }

    #[inline(always)]
    pub fn negate(q: &Self, negated: &mut Self) {
        negated.x = -q.x;
        negated.y = -q.y;
        negated.z = -q.z;
        negated.w = -q.w;
    }

    /// Computes the quaternion rotation between two normalized vectors.
    #[inline(always)]
    pub fn get_quaternion_between_normalized_vectors(
        v1: &Vector3Wide,
        v2: &Vector3Wide,
        q: &mut Self,
    ) {
        let dot = out!(Vector3Wide::dot(v1, v2));
        // For non-normal vectors, the multiplying the axes length squared would be necessary:
        // float w = dot + Sqrt(v1.LengthSquared() * v2.LengthSquared());

        // There exists an ambiguity at dot == -1. If the directions point away from each other, there are an infinite number of shortest paths.
        // One must be chosen arbitrarily. Here, we choose one by projecting onto the plane whose normal is associated with the smallest magnitude.
        // Since this is a SIMD operation, the special case is always executed and its result is conditionally selected.
        let cross = out_unsafe!(Vector3Wide::cross_without_overlap(v1, v2));
        let use_normal_case = Vector::simd_gt(dot, Vector::<f32>::splat(-0.999999));
        let abs_x = Vector::abs(v1.x);
        let abs_y = Vector::abs(v1.y);
        let abs_z = Vector::abs(v1.z);
        let x_is_smallest = Vector::simd_lt(abs_x, abs_y).bitand(Vector::simd_lt(abs_x, abs_z));
        let y_is_smaller = Vector::simd_lt(abs_y, abs_z);
        q.x = use_normal_case.select(
            cross.x,
            x_is_smallest.select(Vector::<f32>::splat(0.0), y_is_smaller.select(-v1.z, -v1.y)),
        );
        q.y = use_normal_case.select(
            cross.y,
            x_is_smallest.select(-v1.z, y_is_smaller.select(Vector::<f32>::splat(0.0), v1.x)),
        );
        q.z = use_normal_case.select(
            cross.z,
            x_is_smallest.select(v1.y, y_is_smaller.select(v1.x, Vector::<f32>::splat(0.0))),
        );
        q.w = use_normal_case.select(dot + Vector::<f32>::splat(1.0), Vector::<f32>::splat(0.0));
        *q = Self::normalize(*q);
    }

    /// Computes the quaternion rotation between two normalized vectors.
    #[inline(always)]
    pub fn get_new_quaternion_between_normalized_vectors(v1: Vector3Wide, v2: Vector3Wide) -> Self {
        let dot = out!(Vector3Wide::dot(&v1, &v2));
        // For non-normal vectors, the multiplying the axes length squared would be necessary:
        // float w = dot + Sqrt(v1.LengthSquared() * v2.LengthSquared());

        // There exists an ambiguity at dot == -1. If the directions point away from each other, there are an infinite number of shortest paths.
        // One must be chosen arbitrarily. Here, we choose one by projecting onto the plane whose normal is associated with the smallest magnitude.
        // Since this is a SIMD operation, the special case is always executed and its result is conditionally selected.
        let cross = Vector3Wide::cross_new(&v1, &v2);
        let use_normal_case = Vector::simd_gt(dot, Vector::<f32>::splat(-0.999999));
        let abs_x = Vector::abs(v1.x);
        let abs_y = Vector::abs(v1.y);
        let abs_z = Vector::abs(v1.z);
        let x_is_smallest = Vector::simd_lt(abs_x, abs_y).bitand(Vector::simd_lt(abs_x, abs_z));
        let y_is_smaller = Vector::simd_lt(abs_y, abs_z);
        let q = Self {
            x: use_normal_case.select(
                cross.x,
                x_is_smallest.select(Vector::<f32>::splat(0.0), y_is_smaller.select(-v1.z, -v1.y)),
            ),
            y: use_normal_case.select(
                cross.y,
                x_is_smallest.select(-v1.z, y_is_smaller.select(Vector::<f32>::splat(0.0), v1.x)),
            ),
            z: use_normal_case.select(
                cross.z,
                x_is_smallest.select(v1.y, y_is_smaller.select(v1.x, Vector::<f32>::splat(0.0))),
            ),
            w: use_normal_case.select(dot + Vector::<f32>::splat(1.0), Vector::<f32>::splat(0.0)),
        };
        return Self::normalize(q);
    }

    /// Gets an axis and angle representation of the rotation stored in a quaternion.
    #[inline(always)]
    pub fn get_axis_angle_from_quaternion(
        q: &Self,
        axis: &mut Vector3Wide,
        angle: &mut Vector<f32>,
    ) {
        let should_negate = Vector::simd_lt(q.w, Vector::<f32>::splat(0.0));
        axis.x = should_negate.select(-q.x, q.x);
        axis.y = should_negate.select(-q.y, q.y);
        axis.z = should_negate.select(-q.z, q.z);
        let qw = should_negate.select(-q.w, q.w);

        let axis_length = Vector3Wide::length(&axis);
        *axis = out!(Vector3Wide::scale_to(
            &axis,
            &(Vector::<f32>::splat(1.0) / axis_length)
        ));
        let use_fallback = Vector::simd_lt(axis_length, Vector::<f32>::splat(1e-14));
        axis.x = use_fallback.select(Vector::<f32>::splat(1.0), axis.x);
        axis.y = use_fallback.select(Vector::<f32>::splat(0.0), axis.y);
        axis.z = use_fallback.select(Vector::<f32>::splat(0.0), axis.z);
        let half_angle = math_helper::acos_simd(qw);
        *angle = Vector::<f32>::splat(2.0) * half_angle;
    }

    /// Transforms the vector using a quaternion. Assumes that the memory backing the input and output do not overlap.
    pub fn transform_without_overlap(v: &Vector3Wide, rotation: &Self, result: &mut Vector3Wide) {
        // This operation is an optimized-down version of v' = q * v * q^-1.
        // The expanded form would be to treat v as an 'axis only' quaternion
        // and perform standard quaternion multiplication.  Assuming q is normalized,
        // q^-1 can be replaced by a conjugation.
        let x2 = rotation.x + rotation.x;
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xx2 = rotation.x * x2;
        let xy2 = rotation.x * y2;
        let xz2 = rotation.x * z2;
        let yy2 = rotation.y * y2;
        let yz2 = rotation.y * z2;
        let zz2 = rotation.z * z2;
        let wx2 = rotation.w * x2;
        let wy2 = rotation.w * y2;
        let wz2 = rotation.w * z2;
        result.x =
            v.x * (Vector::<f32>::splat(1.0) - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2);
        result.y =
            v.x * (xy2 + wz2) + v.y * (Vector::<f32>::splat(1.0) - xx2 - zz2) + v.z * (yz2 - wx2);
        result.z =
            v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (Vector::<f32>::splat(1.0) - xx2 - yy2);
    }

    /// Transforms the vector using a quaternion.
    #[inline(always)]
    pub fn transform(v: &Vector3Wide, rotation: &Self) -> Vector3Wide {
        out!(Self::transform_without_overlap(v, rotation))
    }

    /// Transforms the vector using a quaternion.
    #[inline(always)]
    pub fn transform_by_conjugate(v: Vector3Wide, rotation: Self) -> Vector3Wide {
        // This operation is an optimized-down version of v' = q * v * q^-1.
        // The expanded form would be to treat v as an 'axis only' quaternion
        // and perform standard quaternion multiplication.  Assuming q is normalized,
        // q^-1 can be replaced by a conjugation.
        let x2 = rotation.x + rotation.x;
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xx2 = rotation.x * x2;
        let xy2 = rotation.x * y2;
        let xz2 = rotation.x * z2;
        let yy2 = rotation.y * y2;
        let yz2 = rotation.y * z2;
        let zz2 = rotation.z * z2;
        let n_w = -rotation.w;
        let wx2 = n_w * x2;
        let wy2 = n_w * y2;
        let wz2 = n_w * z2;
        Vector3Wide {
            x: v.x * (Vector::<f32>::splat(1.0) - yy2 - zz2)
                + v.y * (xy2 - wz2)
                + v.z * (xz2 + wy2),
            y: v.x * (xy2 + wz2)
                + v.y * (Vector::<f32>::splat(1.0) - xx2 - zz2)
                + v.z * (yz2 - wx2),
            z: v.x * (xz2 - wy2)
                + v.y * (yz2 + wx2)
                + v.z * (Vector::<f32>::splat(1.0) - xx2 - yy2),
        }
    }

    /// Transforms the unit X direction using a quaternion.
    #[inline(always)]
    pub fn transform_unit_x(rotation: Self) -> Vector3Wide {
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xy2 = rotation.x * y2;
        let xz2 = rotation.x * z2;
        let yy2 = rotation.y * y2;
        let zz2 = rotation.z * z2;
        let wy2 = rotation.w * y2;
        let wz2 = rotation.w * z2;
        Vector3Wide {
            x: Vector::<f32>::splat(1.0) - yy2 - zz2,
            y: xy2 + wz2,
            z: xz2 - wy2,
        }
    }

    /// Transforms the unit Y vector using a quaternion.
    #[inline(always)]
    pub fn transform_unit_y(rotation: Self) -> Vector3Wide {
        let x2 = rotation.x + rotation.x;
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xx2 = rotation.x * x2;
        let xy2 = rotation.x * y2;
        let yz2 = rotation.y * z2;
        let zz2 = rotation.z * z2;
        let wx2 = rotation.w * x2;
        let wz2 = rotation.w * z2;
        Vector3Wide {
            x: xy2 - wz2,
            y: Vector::<f32>::splat(1.0) - xx2 - zz2,
            z: yz2 + wx2,
        }
    }

    /// Transforms the unit Z vector using a quaternion.
    #[inline(always)]
    pub fn transform_unit_z(rotation: Self) -> Vector3Wide {
        let x2 = rotation.x + rotation.x;
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xx2 = rotation.x * x2;
        let xz2 = rotation.x * z2;
        let yy2 = rotation.y * y2;
        let yz2 = rotation.y * z2;
        let wx2 = rotation.w * x2;
        let wy2 = rotation.w * y2;
        Vector3Wide {
            x: xz2 + wy2,
            y: yz2 - wx2,
            z: Vector::<f32>::splat(1.0) - xx2 - yy2,
        }
    }

    /// Transforms the unit X and unit Y direction using a quaternion.
    #[inline(always)]
    pub fn transform_unit_xy(rotation: &Self, x: &mut Vector3Wide, y: &mut Vector3Wide) {
        let x2 = rotation.x + rotation.x;
        let y2 = rotation.y + rotation.y;
        let z2 = rotation.z + rotation.z;
        let xx2 = rotation.x * x2;
        let xy2 = rotation.x * y2;
        let xz2 = rotation.x * z2;
        let yy2 = rotation.y * y2;
        let yz2 = rotation.y * z2;
        let zz2 = rotation.z * z2;
        let wx2 = rotation.w * x2;
        let wy2 = rotation.w * y2;
        let wz2 = rotation.w * z2;
        x.x = Vector::<f32>::splat(1.0) - yy2 - zz2;
        x.y = xy2 + wz2;
        x.z = xz2 - wy2;
        y.x = xy2 - wz2;
        y.y = Vector::<f32>::splat(1.0) - xx2 - zz2;
        y.z = yz2 + wx2;
    }

    /// Transforms the unit X and unit Z direction using a quaternion.
    #[inline(always)]
    pub fn transform_unit_xz(rotation: &Self, x: &mut Vector3Wide, z: &mut Vector3Wide) {
        let q_x2 = rotation.x + rotation.x;
        let q_y2 = rotation.y + rotation.y;
        let q_z2 = rotation.z + rotation.z;

        let yy = q_y2 * rotation.y;
        let zz = q_z2 * rotation.z;
        x.x = Vector::<f32>::splat(1.0) - yy - zz;
        let xy = q_x2 * rotation.y;
        let zw = q_z2 * rotation.w;
        x.y = xy + zw;
        let xz = q_x2 * rotation.z;
        let yw = q_y2 * rotation.w;
        x.z = xz - yw;

        let xx = q_x2 * rotation.x;
        let xw = q_x2 * rotation.w;
        let yz = q_y2 * rotation.z;
        z.x = xz + yw;
        z.y = yz - xw;
        z.z = Vector::<f32>::splat(1.0) - xx - yy;
    }

    /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
    /// transformed = (v * a) * b. Assumes that the memory backing the input and output do not overlap.
    #[inline(always)]
    pub fn concatenate_without_overlap(a: &Self, b: &Self, result: &mut Self) {
        result.x = a.w * b.x + a.x * b.w + a.z * b.y - a.y * b.z;
        result.y = a.w * b.y + a.y * b.w + a.x * b.z - a.z * b.x;
        result.z = a.w * b.z + a.z * b.w + a.y * b.x - a.x * b.y;
        result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    }

    /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
    /// transformed = (v * a) * b.
    #[inline(always)]
    pub fn concatenate(a: &Self, b: &Self, result: &mut Self) {
        *result = out!(Self::concatenate_without_overlap(a, b));
    }

    /// Computes the conjugate of the quaternion into another QuaternionWide.
    #[inline(always)]
    pub fn conjugate_into(quaternion: &Self, result: &mut Self) {
        result.x = quaternion.x;
        result.y = quaternion.y;
        result.z = quaternion.z;
        result.w = -quaternion.w;
    }

    /// Computes the conjugate of the quaternion.
    #[inline(always)]
    pub fn conjugate(quaternion: &Self) -> Self {
        Self {
            x: quaternion.x,
            y: quaternion.y,
            z: quaternion.z,
            w: -quaternion.w,
        }
    }

    #[inline(always)]
    pub fn conditional_select_into(
        condition: Vector<i32>,
        left: &Self,
        right: &Self,
        result: &mut Self,
    ) {
        let condition = Mask::from_int(condition);
        result.x = condition.select(left.x, right.x);
        result.y = condition.select(left.y, right.y);
        result.z = condition.select(left.z, right.z);
        result.w = condition.select(left.w, right.w);
    }

    #[inline(always)]
    pub fn conditional_select(condition: Vector<i32>, left: &Self, right: &Self) -> Self {
        let condition = Mask::from_int(condition);
        Self {
            x: condition.select(left.x, right.x),
            y: condition.select(left.y, right.y),
            z: condition.select(left.z, right.z),
            w: condition.select(left.w, right.w),
        }
    }

    /// Gathers values from the first slot of a wide quaternion and puts them into a narrow representation.
    #[inline(always)]
    pub fn read_first(source: &Self, target: &mut Quat) {
        target.x = source.x[0];
        target.y = source.y[0];
        target.z = source.z[0];
        target.w = source.w[0];
    }

    /// Gathers values from a quaternion and places them into the first indices of the target wide quaternion.
    #[inline(always)]
    pub fn write_first(source: Quat, target_slot: &mut Self) {
        unsafe {
            *GatherScatter::get_first_mut(&mut target_slot.x) = source.x;
            *GatherScatter::get_first_mut(&mut target_slot.y) = source.y;
            *GatherScatter::get_first_mut(&mut target_slot.z) = source.z;
            *GatherScatter::get_first_mut(&mut target_slot.w) = source.w;
        }
    }

    /// Writes a value into a slot of the target bundle.
    #[inline(always)]
    pub fn write_slot(source: Quat, slot_index: usize, target: &mut Self) {
        Self::write_first(source, unsafe {
            GatherScatter::get_offset_instance_mut(target, slot_index)
        });
    }

    #[inline(always)]
    pub fn read_slot(wide: &Self, slot_index: usize, narrow: &mut Quat) -> Quaternion {
        let offset = unsafe { GatherScatter::get_offset_instance(wide, slot_index) };
        Self::read_first(offset, narrow);
    }
}

impl Mul for QuaternionWide {
    type Output = Self;

    #[inline(always)]
    fn mul(self, rhs: Self) -> Self {
        QuaternionWide {
            x: self.w * rhs.x + self.x * rhs.w + self.z * rhs.y - self.y * rhs.z,
            y: self.w * rhs.y + self.y * rhs.w + self.x * rhs.z - self.z * rhs.x,
            z: self.w * rhs.z + self.z * rhs.w + self.y * rhs.x - self.x * rhs.y,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }
}

impl Mul<QuaternionWide> for Vector3Wide {
    type Output = Vector3Wide;

    #[inline(always)]
    fn mul(self, rotation: QuaternionWide) -> Vector3Wide {
        QuaternionWide::transform(&self, &rotation)
    }
}

impl Neg for QuaternionWide {
    type Output = Self;

    #[inline(always)]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: -self.w,
        }
    }
}
