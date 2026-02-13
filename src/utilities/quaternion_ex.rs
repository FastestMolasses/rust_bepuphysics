use crate::utilities::math_helper;
use crate::{
    out,
    utilities::{matrix::Matrix, matrix3x3::Matrix3x3},
};
use glam::{Quat, Vec3, Vec4};

/// Adds two quaternions together.
#[inline(always)]
pub fn add(a: Quat, b: Quat, result: &mut Quat) {
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    result.w = a.w + b.w;
}

/// Scales a quaternion.
#[inline(always)]
pub fn scale(q: Quat, scale: f32, result: &mut Quat) {
    result.x = q.x * scale;
    result.y = q.y * scale;
    result.z = q.z * scale;
    result.w = q.w * scale;
}

/// Concatenates the transforms of two quaternions together such that the resulting quaternion,
/// applied as an orientation to a vector v, is equivalent to transformed = (v * a) * b.
/// Assumes that neither input parameter overlaps the output parameter.
#[inline(always)]
pub fn concatenate_without_overlap(a: Quat, b: Quat, result: &mut Quat) {
    result.x = a.w * b.x + a.x * b.w + a.z * b.y - a.y * b.z;
    result.y = a.w * b.y + a.y * b.w + a.x * b.z - a.z * b.x;
    result.z = a.w * b.z + a.z * b.w + a.y * b.x - a.x * b.y;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
}

/// Concatenates the transforms of two quaternions together such that the resulting quaternion,
/// applied as an orientation to a vector v, is equivalent to transformed = (v * a) * b.
#[inline(always)]
pub fn concatenate_to(a: Quat, b: Quat, result: &mut Quat) {
    *result = out!(self::concatenate_without_overlap(a, b));
}

/// Concatenates the transforms of two quaternions together such that the resulting quaternion,
/// applied as an orientation to a vector v, is equivalent to transformed = (v * a) * b.
#[inline(always)]
pub fn concatenate(a: Quat, b: Quat) -> Quat {
    out!(self::concatenate_without_overlap(a, b))
}

/// Constructs a quaternion from a rotation matrix.
#[inline(always)]
pub fn create_from_rotation_matrix3x3_into(r: &Matrix3x3, q: &mut Quat) {
    let t;
    if r.z.z < 0.0 {
        if r.x.x > r.y.y {
            t = 1.0 + r.x.x - r.y.y - r.z.z;
            q.x = t;
            q.y = r.x.y + r.y.x;
            q.z = r.z.x + r.x.z;
            q.w = r.y.z - r.z.y;
        } else {
            t = 1.0 - r.x.x + r.y.y - r.z.z;
            q.x = r.x.y + r.y.x;
            q.y = t;
            q.z = r.y.z + r.z.y;
            q.w = r.z.x - r.x.z;
        }
    } else {
        #[allow(clippy::collapsible_else_if)]
        if r.x.x < -r.y.y {
            t = 1.0 - r.x.x - r.y.y + r.z.z;
            q.x = r.z.x + r.x.z;
            q.y = r.y.z + r.z.y;
            q.z = t;
            q.w = r.x.y - r.y.x;
        } else {
            t = 1.0 + r.x.x + r.y.y + r.z.z;
            q.x = r.y.z - r.z.y;
            q.y = r.z.x - r.x.z;
            q.z = r.x.y - r.y.x;
            q.w = t;
        }
    }
    *q = out!(self::scale(*q, 0.5 / t.sqrt()));
}

/// Creates a quaternion from a rotation matrix.
#[inline(always)]
pub fn create_from_rotation_matrix3x3(r: &Matrix3x3) -> Quat {
    out!(self::create_from_rotation_matrix3x3_into(r))
}

/// Constructs a quaternion from a rotation matrix.
#[inline(always)]
pub fn create_from_rotation_matrix_into(r: &Matrix, q: &mut Quat) {
    let rotation3x3 = out!(Matrix3x3::create_from_matrix(r));
    create_from_rotation_matrix3x3_into(&rotation3x3, q);
}

/// Constructs a quaternion from a rotation matrix.
pub fn create_from_rotation_matrix(r: &Matrix) -> Quat {
    let rotation3x3 = out!(Matrix3x3::create_from_matrix(r));
    out!(self::create_from_rotation_matrix3x3_into(&rotation3x3))
}

/// Ensures the quaternion has unit length.
#[inline(always)]
pub fn normalize_into(quaternion: &mut Quat) {
    let v = Vec4::from(*quaternion);
    let scale = v.dot(v).sqrt().recip();
    *quaternion = Quat::from_vec4(v * scale);
}

/// Ensures the quaternion has unit length.
#[inline(always)]
pub fn normalize(mut quaternion: Quat) -> Quat {
    self::normalize_into(&mut quaternion);
    quaternion
}

/// Blends two quaternions together to get an intermediate state.
#[inline(always)]
pub fn slerp_into(start: Quat, mut end: Quat, interpolation_amount: f32, result: &mut Quat) {
    let mut cos_half_theta: f64 =
        (start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z) as f64;
    if cos_half_theta < 0.0 {
        // Negating a quaternion results in the same orientation,
        // but we need cosHalfTheta to be positive to get the shortest path.
        end.x = -end.x;
        end.y = -end.y;
        end.z = -end.z;
        end.w = -end.w;
        cos_half_theta = -cos_half_theta;
    }
    // If the orientations are similar enough, then just pick one of the inputs.
    if cos_half_theta > (1.0 - 1e-12) {
        result.w = start.w;
        result.x = start.x;
        result.y = start.y;
        result.z = start.z;
        return;
    }
    // Calculate temporary values.
    let half_theta = cos_half_theta.acos();
    let sin_half_theta = (1.0 - cos_half_theta * cos_half_theta).sqrt();

    let a_fraction = ((1.0 - interpolation_amount) as f64 * half_theta).sin() / sin_half_theta;
    let b_fraction = (interpolation_amount as f64 * half_theta).sin() / sin_half_theta;

    // Blend the two quaternions to get the result!
    result.x = (start.x as f64 * a_fraction + end.x as f64 * b_fraction) as f32;
    result.y = (start.y as f64 * a_fraction + end.y as f64 * b_fraction) as f32;
    result.z = (start.z as f64 * a_fraction + end.z as f64 * b_fraction) as f32;
    result.w = (start.w as f64 * a_fraction + end.w as f64 * b_fraction) as f32;
}

/// Blends two quaternions together to get an intermediate state.
#[inline(always)]
pub fn slerp(start: Quat, end: Quat, interpolation_amount: f32) -> Quat {
    out!(self::slerp_into(start, end, interpolation_amount))
}

// /// Computes the conjugate of the quaternion.
#[inline(always)]
pub fn conjugate_into(quaternion: Quat, result: &mut Quat) {
    result.x = -quaternion.x;
    result.y = -quaternion.y;
    result.z = -quaternion.z;
    result.w = quaternion.w;
}

/// Computes the conjugate of the quaternion.
#[inline(always)]
pub fn conjugate(quaternion: Quat) -> Quat {
    out!(self::conjugate_into(quaternion))
}

/// Computes the inverse of the quaternion.
#[inline(always)]
pub fn inverse_into(quaternion: Quat, result: &mut Quat) {
    let inverse_squared_norm = quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w;
    result.x = -quaternion.x * inverse_squared_norm;
    result.y = -quaternion.y * inverse_squared_norm;
    result.z = -quaternion.z * inverse_squared_norm;
    result.w = quaternion.w * inverse_squared_norm;
}

/// Computes the inverse of the quaternion.
#[inline(always)]
pub fn inverse(quaternion: Quat) -> Quat {
    out!(self::inverse_into(quaternion))
}

/// Negates the components of a quaternion.
#[inline(always)]
pub fn negate_into(a: Quat, b: &mut Quat) {
    b.x = -a.x;
    b.y = -a.y;
    b.z = -a.z;
    b.w = -a.w;
}

/// Negates the components of a quaternion.
#[inline(always)]
pub fn negate(q: Quat) -> Quat {
    out!(self::negate_into(q))
}

/// Transforms the vector using a quaternion, assuming that the output does not alias with the input.
#[inline(always)]
pub fn transform_without_overlap(v: Vec3, rotation: Quat, result: &mut Vec3) {
    // This operation is an optimized-down version of v' = q * v * q^-1.
    // The expanded form would be to treat v as an 'axis only' quaternion
    // and perform standard quaternion multiplication. Assuming q is normalized,
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
    // Defer the component setting since they're used in computation.
    result.x = v.x * (1.0 - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2);
    result.y = v.x * (xy2 + wz2) + v.y * (1.0 - xx2 - zz2) + v.z * (yz2 - wx2);
    result.z = v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1.0 - xx2 - yy2);
}

/// Transforms the vector using a quaternion.
#[inline(always)]
pub fn transform_into(v: Vec3, rotation: Quat, result: &mut Vec3) {
    *result = out!(self::transform_without_overlap(v, rotation));
}

/// Transforms the vector using a quaternion.
#[inline(always)]
pub fn transform(v: Vec3, rotation: Quat) -> Vec3 {
    out!(self::transform_without_overlap(v, rotation))
}

/// Transforms the unit X direction using a quaternion.
#[inline(always)]
pub fn transform_unit_x(rotation: Quat, result: &mut Vec3) {
    //This operation is an optimized-down version of v' = q * v * q^-1.
    //The expanded form would be to treat v as an 'axis only' quaternion
    //and perform standard quaternion multiplication.  Assuming q is normalized,
    //q^-1 can be replaced by a conjugation.
    let y2 = rotation.y + rotation.y;
    let z2 = rotation.z + rotation.z;
    let xy2 = rotation.x * y2;
    let xz2 = rotation.x * z2;
    let yy2 = rotation.y * y2;
    let zz2 = rotation.z * z2;
    let wy2 = rotation.w * y2;
    let wz2 = rotation.w * z2;
    result.x = 1.0 - yy2 - zz2;
    result.y = xy2 + wz2;
    result.z = xz2 - wy2;
}

/// Transforms the unit Y vector using a quaternion.
#[inline(always)]
pub fn transform_unit_y(rotation: Quat, result: &mut Vec3) {
    // This operation is an optimized-down version of v' = q * v * q^-1.
    // The expanded form would be to treat v as an 'axis only' quaternion
    // and perform standard quaternion multiplication.  Assuming q is normalized,
    // q^-1 can be replaced by a conjugation.
    let x2 = rotation.x + rotation.x;
    let y2 = rotation.y + rotation.y;
    let z2 = rotation.z + rotation.z;
    let xx2 = rotation.x * x2;
    let xy2 = rotation.x * y2;
    let yz2 = rotation.y * z2;
    let zz2 = rotation.z * z2;
    let wx2 = rotation.w * x2;
    let wz2 = rotation.w * z2;
    result.x = xy2 - wz2;
    result.y = 1.0 - xx2 - zz2;
    result.z = yz2 + wx2;
}

/// Transforms the unit Z vector using a quaternion.
#[inline(always)]
pub fn transform_unit_z(rotation: Quat, result: &mut Vec3) {
    // This operation is an optimized-down version of v' = q * v * q^-1.
    // The expanded form would be to treat v as an 'axis only' quaternion
    // and perform standard quaternion multiplication.  Assuming q is normalized,
    // q^-1 can be replaced by a conjugation.
    let x2 = rotation.x + rotation.x;
    let y2 = rotation.y + rotation.y;
    let z2 = rotation.z + rotation.z;
    let xx2 = rotation.x * x2;
    let xz2 = rotation.x * z2;
    let yy2 = rotation.y * y2;
    let yz2 = rotation.y * z2;
    let wx2 = rotation.w * x2;
    let wy2 = rotation.w * y2;
    result.x = xz2 + wy2;
    result.y = yz2 - wx2;
    result.z = 1.0 - xx2 - yy2;
}

/// Creates a quaternion from an axis and angle.
#[inline(always)]
pub fn create_from_axis_angle(axis: Vec3, angle: f32) -> Quat {
    let half_angle: f64 = angle as f64 * 0.5;
    let s: f64 = half_angle.sin();
    Quat::from_xyzw(
        (axis.x as f64 * s) as f32,
        (axis.y as f64 * s) as f32,
        (axis.z as f64 * s) as f32,
        half_angle.cos() as f32,
    )
}

/// Creates a quaternion from an axis and angle.
#[inline(always)]
pub fn create_from_axis_angle_into(axis: Vec3, angle: f32, q: &mut Quat) {
    let half_angle: f64 = angle as f64 * 0.5;
    let s: f64 = half_angle.sin();
    q.x = (axis.x as f64 * s) as f32;
    q.y = (axis.y as f64 * s) as f32;
    q.z = (axis.z as f64 * s) as f32;
    q.w = half_angle.cos() as f32;
}

/// Constructs a quaternion from yaw, pitch, and roll.
#[inline(always)]
pub fn create_from_yaw_pitch_roll(yaw: f32, pitch: f32, roll: f32) -> Quat {
    out!(self::create_from_yaw_pitch_roll_into(yaw, pitch, roll))
}

/// Constructs a quaternion from yaw, pitch, and roll.
#[inline(always)]
pub fn create_from_yaw_pitch_roll_into(yaw: f32, pitch: f32, roll: f32, q: &mut Quat) {
    let half_roll: f64 = roll as f64 * 0.5;
    let half_pitch: f64 = pitch as f64 * 0.5;
    let half_yaw: f64 = yaw as f64 * 0.5;

    let sin_roll: f64 = half_roll.sin();
    let sin_pitch: f64 = half_pitch.sin();
    let sin_yaw: f64 = half_yaw.sin();

    let cos_roll: f64 = half_roll.cos();
    let cos_pitch: f64 = half_pitch.cos();
    let cos_yaw: f64 = half_yaw.cos();

    let cos_yaw_cos_pitch: f64 = cos_yaw * cos_pitch;
    let cos_yaw_sin_pitch: f64 = cos_yaw * sin_pitch;
    let sin_yaw_cos_pitch: f64 = sin_yaw * cos_pitch;
    let sin_yaw_sin_pitch: f64 = sin_yaw * sin_pitch;

    q.x = (cos_yaw_sin_pitch * cos_roll + sin_yaw_cos_pitch * sin_roll) as f32;
    q.y = (sin_yaw_cos_pitch * cos_roll - cos_yaw_sin_pitch * sin_roll) as f32;
    q.z = (cos_yaw_cos_pitch * sin_roll - sin_yaw_sin_pitch * cos_roll) as f32;
    q.w = (cos_yaw_cos_pitch * cos_roll + sin_yaw_sin_pitch * sin_roll) as f32;
}

/// Computes the angle change represented by a normalized quaternion.
#[inline(always)]
pub fn angle_from_quaternion(q: Quat) -> f32 {
    let qw = q.w.abs();
    if qw > 1.0 {
        return 0.0;
    }
    2.0 * qw.acos()
}

/// Computes the axis angle representation of a normalized quaternion.
#[inline(always)]
pub fn axis_angle_from_quaternion(q: Quat, axis: &mut Vec3, angle: &mut f32) {
    let mut qw = q.w;
    if qw > 0.0 {
        axis.x = q.x;
        axis.y = q.y;
        axis.z = q.z;
    } else {
        axis.x = -q.x;
        axis.y = -q.y;
        axis.z = -q.z;
        qw = -qw;
    }

    let length_squared = axis.length_squared();
    if length_squared > 1e-14 {
        *axis /= length_squared.sqrt();
        *angle = 2.0 * math_helper::clamp(qw, -1.0, 1.0).acos();
    } else {
        *axis = Vec3::Y;
        *angle = 0.0;
    }
}

/// Computes the quaternion rotation between two normalized vectors.
#[inline(always)]
pub fn quaternion_between_normalized_vectors(v1: Vec3, v2: Vec3, q: &mut Quat) {
    let dot = v1.dot(v2);
    // For non-normal vectors, the multiplying the axes length squared would be necessary:
    // float w = dot + (float)Math.Sqrt(v1.LengthSquared() * v2.LengthSquared());
    // parallel, opposing direction
    if dot < -0.9999 {
        // If this occurs, the rotation required is ~180 degrees.
        // The problem is that we could choose any perpendicular axis for the rotation. It's not uniquely defined.
        // The solution is to pick an arbitrary perpendicular axis.
        // Project onto the plane which has the lowest component magnitude.
        // On that 2d plane, perform a 90 degree rotation.
        let abs_x = v1.x.abs();
        let abs_y = v1.y.abs();
        let abs_z = v1.z.abs();
        if abs_x < abs_y && abs_x < abs_z {
            *q = Quat::from_xyzw(0.0, -v1.z, v1.y, 0.0);
        } else if abs_y < abs_z {
            *q = Quat::from_xyzw(v1.z, 0.0, -v1.x, 0.0);
        } else {
            *q = Quat::from_xyzw(-v1.y, v1.x, 0.0, 0.0);
        }
    } else {
        let axis = v1.cross(v2);
        *q = Quat::from_xyzw(axis.x, axis.y, axis.z, dot + 1.0);
    }
    self::normalize_into(q);
}

// The following two functions are highly similar, but it's a bit of a brain teaser to phrase one in terms of the other.
// Providing both simplifies things.

/// Computes the rotation from the start orientation to the end orientation such that end = Quaternion.Concatenate(start, relative).
/// Assumes that neither input parameter overlaps with the output parameter.
#[inline(always)]
pub fn relative_rotation_without_overlap(start: Quat, end: Quat, relative: &mut Quat) {
    let start_inverse = self::conjugate(start);
    self::concatenate_without_overlap(start_inverse, end, relative);
}

/// Transforms the rotation into the local space of the target basis such that rotation = Quaternion.Concatenate(localRotation, targetBasis)
/// Assumes that neither input parameter overlaps with the output parameter.
#[inline(always)]
pub fn local_rotation_without_overlap(
    rotation: Quat,
    target_basis: Quat,
    local_rotation: &mut Quat,
) {
    let basis_inverse = self::conjugate(target_basis);
    self::concatenate_without_overlap(rotation, basis_inverse, local_rotation);
}

/// Returns the identity quaternion (0, 0, 0, 1).
#[inline(always)]
pub fn identity() -> Quat {
    Quat::from_xyzw(0.0, 0.0, 0.0, 1.0)
}

/// Computes the squared length of a quaternion (treating it as a Vec4).
#[inline(always)]
pub fn length_squared(q: &Quat) -> f32 {
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
}

/// Computes the length of a quaternion (treating it as a Vec4).
#[inline(always)]
pub fn length(q: &Quat) -> f32 {
    length_squared(q).sqrt()
}
