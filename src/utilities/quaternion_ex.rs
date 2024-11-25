use crate::utilities::{matrix::Matrix, matrix3x3::Matrix3x3};
use glam::{Vec3, Quat};
use std::mem::transmute;

#[inline(always)]
pub fn add(a: Quat, b: Quat) -> Quat {
    Quaternion(a.0 + b.0)
}

#[inline(always)]
pub fn scale(q: Quaternion, scale: f32) -> Quaternion {
    Quaternion(q.0 * f32x4::splat(scale))
}

#[inline(always)]
pub fn concatenate_without_overlap(a: Quaternion, b: Quaternion) -> Quaternion {
    let (ax, ay, az, aw) = unsafe { transmute::<f32x4, (f32, f32, f32, f32)>(a.0) };
    let (bx, by, bz, bw) = unsafe { transmute::<f32x4, (f32, f32, f32, f32)>(b.0) };
    Quaternion(f32x4::new(
        aw * bx + ax * bw + az * by - ay * bz,
        aw * by + ay * bw + ax * bz - az * bx,
        aw * bz + az * bw + ay * bx - ax * by,
        aw * bw - ax * bx - ay * by - az * bz,
    ))
}

#[inline(always)]
pub fn concatenate(a: Quaternion, b: Quaternion) -> Quaternion {
    concatenate_without_overlap(a, b)
}

pub const IDENTITY: Quaternion = Quaternion(f32x4::new(0.0, 0.0, 0.0, 1.0));

#[inline(always)]
pub fn create_from_rotation_matrix(r: &Matrix3x3) -> Quaternion {
    let mut q = Quaternion::default();
    let mut t: f32;
    if r.z.z < 0.0 {
        if r.x.x > r.y.y {
            t = 1.0 + r.x.x - r.y.y - r.z.z;
            q.0 = f32x4::new(t, r.x.y + r.y.x, r.z.x + r.x.z, r.y.z - r.z.y);
        } else {
            t = 1.0 - r.x.x + r.y.y - r.z.z;
            q.0 = f32x4::new(r.x.y + r.y.x, t, r.y.z + r.z.y, r.z.x - r.x.z);
        }
    } else {
        if r.x.x < -r.y.y {
            t = 1.0 - r.x.x - r.y.y + r.z.z;
            q.0 = f32x4::new(r.z.x + r.x.z, r.y.z + r.z.y, t, r.x.y - r.y.x);
        } else {
            t = 1.0 + r.x.x + r.y.y + r.z.z;
            q.0 = f32x4::new(r.y.z - r.z.y, r.z.x - r.x.z, r.x.y - r.y.x, t);
        }
    }
    scale(q, 0.5 / t.sqrt())
}

#[inline(always)]
pub fn create_from_rotation_matrix_3x3(r: &Matrix3x3) -> Quaternion {
    create_from_rotation_matrix(r)
}

#[inline(always)]
pub fn create_from_rotation_matrix_4x4(r: &Matrix) -> Quaternion {
    let rotation3x3 = Matrix3x3::create_from_matrix(r);
    create_from_rotation_matrix(&rotation3x3)
}

#[inline(always)]
pub fn normalize(q: &mut Quaternion) {
    let length_sq = q.0.dot(q.0);
    q.0 *= f32x4::splat(1.0 / length_sq.sqrt());
}

#[inline(always)]
pub fn length_squared(q: &Quaternion) -> f32 {
    q.0.dot(q.0)
}

#[inline(always)]
pub fn length(q: &Quaternion) -> f32 {
    q.0.dot(q.0).sqrt()
}

#[inline(always)]
pub fn slerp(start: Quaternion, end: Quaternion, t: f32) -> Quaternion {
    let mut cos_half_theta = start.0.dot(end.0);
    let mut end = end;

    if cos_half_theta < 0.0 {
        end.0 = -end.0;
        cos_half_theta = -cos_half_theta;
    }

    if cos_half_theta > 1.0 - 1e-12 {
        return start;
    }

    let half_theta = cos_half_theta.acos();
    let sin_half_theta = (1.0 - cos_half_theta * cos_half_theta).sqrt();

    if sin_half_theta.abs() < 1e-12 {
        return Quaternion((start.0 + end.0) * f32x4::splat(0.5));
    }

    let a = ((1.0 - t) * half_theta).sin() / sin_half_theta;
    let b = (t * half_theta).sin() / sin_half_theta;

    Quaternion(start.0 * f32x4::splat(a) + end.0 * f32x4::splat(b))
}

#[inline(always)]
pub fn conjugate(q: Quaternion) -> Quaternion {
    Quaternion(q.0 * f32x4::new(-1.0, -1.0, -1.0, 1.0))
}

#[inline(always)]
pub fn inverse(q: Quaternion) -> Quaternion {
    let inv_norm_sq = 1.0 / q.0.dot(q.0);
    Quaternion(q.0 * f32x4::new(-inv_norm_sq, -inv_norm_sq, -inv_norm_sq, inv_norm_sq))
}

#[inline(always)]
pub fn negate(q: Quaternion) -> Quaternion {
    Quaternion(-q.0)
}

#[inline(always)]
pub fn transform_without_overlap(v: Vec3, rotation: Quaternion) -> Vec3 {
    let x2 = rotation.0[0] + rotation.0[0];
    let y2 = rotation.0[1] + rotation.0[1];
    let z2 = rotation.0[2] + rotation.0[2];
    let xx2 = rotation.0[0] * x2;
    let xy2 = rotation.0[0] * y2;
    let xz2 = rotation.0[0] * z2;
    let yy2 = rotation.0[1] * y2;
    let yz2 = rotation.0[1] * z2;
    let zz2 = rotation.0[2] * z2;
    let wx2 = rotation.0[3] * x2;
    let wy2 = rotation.0[3] * y2;
    let wz2 = rotation.0[3] * z2;

    Vec3::new(
        v.x * (1.0 - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
        v.x * (xy2 + wz2) + v.y * (1.0 - xx2 - zz2) + v.z * (yz2 - wx2),
        v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1.0 - xx2 - yy2),
    )
}

#[inline(always)]
pub fn transform(v: Vec3, rotation: Quaternion) -> Vec3 {
    transform_without_overlap(v, rotation)
}

#[inline(always)]
pub fn transform_unit_x(rotation: Quaternion) -> Vec3 {
    let y2 = rotation.0[1] + rotation.0[1];
    let z2 = rotation.0[2] + rotation.0[2];
    let xy2 = rotation.0[0] * y2;
    let xz2 = rotation.0[0] * z2;
    let yy2 = rotation.0[1] * y2;
    let zz2 = rotation.0[2] * z2;
    let wy2 = rotation.0[3] * y2;
    let wz2 = rotation.0[3] * z2;
    Vec3::new(1.0 - yy2 - zz2, xy2 + wz2, xz2 - wy2)
}

#[inline(always)]
pub fn transform_unit_y(rotation: Quaternion) -> Vec3 {
    let x2 = rotation.0[0] + rotation.0[0];
    let y2 = rotation.0[1] + rotation.0[1];
    let z2 = rotation.0[2] + rotation.0[2];
    let xx2 = rotation.0[0] * x2;
    let xy2 = rotation.0[0] * y2;
    let yz2 = rotation.0[1] * z2;
    let zz2 = rotation.0[2] * z2;
    let wx2 = rotation.0[3] * x2;
    let wz2 = rotation.0[3] * z2;
    Vec3::new(xy2 - wz2, 1.0 - xx2 - zz2, yz2 + wx2)
}

#[inline(always)]
pub fn transform_unit_z(rotation: Quaternion) -> Vec3 {
    let x2 = rotation.0[0] + rotation.0[0];
    let y2 = rotation.0[1] + rotation.0[1];
    let z2 = rotation.0[2] + rotation.0[2];
    let xx2 = rotation.0[0] * x2;
    let xz2 = rotation.0[0] * z2;
    let yy2 = rotation.0[1] * y2;
    let yz2 = rotation.0[1] * z2;
    let wx2 = rotation.0[3] * x2;
    let wy2 = rotation.0[3] * y2;
    Vec3::new(xz2 + wy2, yz2 - wx2, 1.0 - xx2 - yy2)
}

#[inline(always)]
pub fn create_from_axis_angle(axis: Vec3, angle: f32) -> Quaternion {
    let half_angle = angle * 0.5;
    let s = half_angle.sin();
    Quaternion(f32x4::new(
        axis.x * s,
        axis.y * s,
        axis.z * s,
        half_angle.cos(),
    ))
}

#[inline(always)]
pub fn create_from_yaw_pitch_roll(yaw: f32, pitch: f32, roll: f32) -> Quaternion {
    let half_roll = roll * 0.5;
    let half_pitch = pitch * 0.5;
    let half_yaw = yaw * 0.5;

    let sin_roll = half_roll.sin();
    let sin_pitch = half_pitch.sin();
    let sin_yaw = half_yaw.sin();

    let cos_roll = half_roll.cos();
    let cos_pitch = half_pitch.cos();
    let cos_yaw = half_yaw.cos();

    let cos_yaw_cos_pitch = cos_yaw * cos_pitch;
    let cos_yaw_sin_pitch = cos_yaw * sin_pitch;
    let sin_yaw_cos_pitch = sin_yaw * cos_pitch;
    let sin_yaw_sin_pitch = sin_yaw * sin_pitch;

    Quaternion(f32x4::new(
        cos_yaw_sin_pitch * cos_roll + sin_yaw_cos_pitch * sin_roll,
        sin_yaw_cos_pitch * cos_roll - cos_yaw_sin_pitch * sin_roll,
        cos_yaw_cos_pitch * sin_roll - sin_yaw_sin_pitch * cos_roll,
        cos_yaw_cos_pitch * cos_roll + sin_yaw_sin_pitch * sin_roll,
    ))
}

#[inline(always)]
pub fn get_angle_from_quaternion(q: Quaternion) -> f32 {
    let qw = q.0[3].abs();
    if qw > 1.0 {
        0.0
    } else {
        2.0 * qw.acos()
    }
}

#[inline(always)]
pub fn get_axis_angle_from_quaternion(q: Quaternion) -> (Vec3, f32) {
    let mut axis = Vec3::new(q.0[0], q.0[1], q.0[2]);
    let mut qw = q.0[3];

    if qw < 0.0 {
        axis = -axis;
        qw = -qw;
    }

    let length_squared = axis.length_squared();
    if length_squared > 1e-14 {
        axis /= length_squared.sqrt();
        // TODO: USE MATH_HELPER HERE
        let angle = 2.0 * qw.clamp(-1.0, 1.0).acos();
        (axis, angle)
    } else {
        (Vec3::unit_y(), 0.0)
    }
}

#[inline(always)]
pub fn get_quaternion_between_normalized_vectors(v1: Vec3, v2: Vec3) -> Quaternion {
    let dot = v1.dot(v2);
    if dot < -0.9999 {
        let abs_x = v1.x.abs();
        let abs_y = v1.y.abs();
        let abs_z = v1.z.abs();
        if abs_x < abs_y && abs_x < abs_z {
            Quaternion(f32x4::new(0.0, -v1.z, v1.y, 0.0))
        } else if abs_y < abs_z {
            Quaternion(f32x4::new(-v1.z, 0.0, v1.x, 0.0))
        } else {
            Quaternion(f32x4::new(-v1.y, v1.x, 0.0, 0.0))
        }
    } else {
        let axis = v1.cross(v2);
        Quaternion(f32x4::new(axis.x, axis.y, axis.z, dot + 1.0))
    }
    .normalized()
}

// TODO: IMPLEMENT REMAINING 2 GetRelativeRotationWithoutOverlap FUNCTIONS
