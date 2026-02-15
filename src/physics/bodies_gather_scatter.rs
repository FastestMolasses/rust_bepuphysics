// Translated from BepuPhysics/Bodies_GatherScatter.cs
//
// This file extends Bodies with gather/scatter operations for the solver.
// The C# version uses AVX intrinsics for 8-wide SIMD on x86-64. We use portable_simd's
// simd_swizzle! behind #[cfg(target_feature = "avx")] to generate the same AVX
// vshufps/vperm2f128 instructions. On non-AVX platforms (ARM NEON, x86 SSE),
// a 4×4 transpose path using portable_simd compiles to native shuffle instructions.

use crate::physics::bodies::Bodies;
use crate::physics::body_properties::{
    BodyDynamics, BodyInertiaWide, BodyVelocityWide, MotionState,
};
use crate::physics::constraints::body_access_filter::IBodyAccessFilter;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
#[allow(unused_imports)]
use glam::{Quat, Vec3};

// Float offsets within a MotionState (64 bytes = 16 floats):
//   [0..3]  = orientation (x, y, z, w)
//   [4..6]  = position (x, y, z)
//   [7]     = padding
//   [8..10] = linear velocity (x, y, z)
//   [11]    = padding
//   [12..14] = angular velocity (x, y, z)
//   [15]    = padding
impl MotionState {
    pub const OFFSET_TO_ORIENTATION_X: usize = 0;
    pub const OFFSET_TO_ORIENTATION_Y: usize = 1;
    pub const OFFSET_TO_ORIENTATION_Z: usize = 2;
    pub const OFFSET_TO_ORIENTATION_W: usize = 3;
    pub const OFFSET_TO_POSITION_X: usize = 4;
    pub const OFFSET_TO_POSITION_Y: usize = 5;
    pub const OFFSET_TO_POSITION_Z: usize = 6;
    pub const OFFSET_TO_LINEAR_X: usize = 8;
    pub const OFFSET_TO_LINEAR_Y: usize = 9;
    pub const OFFSET_TO_LINEAR_Z: usize = 10;
    pub const OFFSET_TO_ANGULAR_X: usize = 12;
    pub const OFFSET_TO_ANGULAR_Y: usize = 13;
    pub const OFFSET_TO_ANGULAR_Z: usize = 14;
}

/// 8x8 matrix transpose using portable_simd swizzle operations.
/// On x86-64 with AVX, this compiles to the same vunpcklps/vshufps/vperm2f128
/// instructions as the C# Avx.UnpackLow/Shuffle/Permute2x128 calls.
#[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
#[inline(always)]
fn transpose_8x8(
    m0: Vector<f32>,
    m1: Vector<f32>,
    m2: Vector<f32>,
    m3: Vector<f32>,
    m4: Vector<f32>,
    m5: Vector<f32>,
    m6: Vector<f32>,
    m7: Vector<f32>,
) -> [Vector<f32>; 8] {
    use std::simd::simd_swizzle;
    // Stage 1: UnpackLow/UnpackHigh (interleave pairs within 128-bit lanes)
    let n0 = simd_swizzle!(m0, m1, [0, 8, 1, 9, 4, 12, 5, 13]);
    let n1 = simd_swizzle!(m2, m3, [0, 8, 1, 9, 4, 12, 5, 13]);
    let n2 = simd_swizzle!(m4, m5, [0, 8, 1, 9, 4, 12, 5, 13]);
    let n3 = simd_swizzle!(m6, m7, [0, 8, 1, 9, 4, 12, 5, 13]);
    let n4 = simd_swizzle!(m0, m1, [2, 10, 3, 11, 6, 14, 7, 15]);
    let n5 = simd_swizzle!(m2, m3, [2, 10, 3, 11, 6, 14, 7, 15]);
    let n6 = simd_swizzle!(m4, m5, [2, 10, 3, 11, 6, 14, 7, 15]);
    let n7 = simd_swizzle!(m6, m7, [2, 10, 3, 11, 6, 14, 7, 15]);
    // Stage 2: Shuffle (group 4-element blocks from pairs)
    let o0 = simd_swizzle!(n0, n1, [0, 1, 8, 9, 4, 5, 12, 13]);
    let o1 = simd_swizzle!(n2, n3, [0, 1, 8, 9, 4, 5, 12, 13]);
    let o2 = simd_swizzle!(n4, n5, [0, 1, 8, 9, 4, 5, 12, 13]);
    let o3 = simd_swizzle!(n6, n7, [0, 1, 8, 9, 4, 5, 12, 13]);
    let o4 = simd_swizzle!(n0, n1, [2, 3, 10, 11, 6, 7, 14, 15]);
    let o5 = simd_swizzle!(n2, n3, [2, 3, 10, 11, 6, 7, 14, 15]);
    let o6 = simd_swizzle!(n4, n5, [2, 3, 10, 11, 6, 7, 14, 15]);
    let o7 = simd_swizzle!(n6, n7, [2, 3, 10, 11, 6, 7, 14, 15]);
    // Stage 3: Permute2x128 (recombine 128-bit halves)
    [
        simd_swizzle!(o0, o1, [0, 1, 2, 3, 8, 9, 10, 11]),
        simd_swizzle!(o4, o5, [0, 1, 2, 3, 8, 9, 10, 11]),
        simd_swizzle!(o2, o3, [0, 1, 2, 3, 8, 9, 10, 11]),
        simd_swizzle!(o6, o7, [0, 1, 2, 3, 8, 9, 10, 11]),
        simd_swizzle!(o0, o1, [4, 5, 6, 7, 12, 13, 14, 15]),
        simd_swizzle!(o4, o5, [4, 5, 6, 7, 12, 13, 14, 15]),
        simd_swizzle!(o2, o3, [4, 5, 6, 7, 12, 13, 14, 15]),
        simd_swizzle!(o6, o7, [4, 5, 6, 7, 12, 13, 14, 15]),
    ]
}

/// Load 8 contiguous f32 values into a SIMD vector.
#[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
#[inline(always)]
unsafe fn load_8f32(ptr: *const f32) -> Vector<f32> {
    Vector::<f32>::from_array(*(ptr as *const [f32; 8]))
}

/// Store 8 f32 values from a SIMD vector to contiguous memory.
#[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
#[inline(always)]
unsafe fn store_8f32(ptr: *mut f32, v: Vector<f32>) {
    *(ptr as *mut [f32; 8]) = v.to_array();
}

/// Store 4 f32 values (128-bit lane) to contiguous memory.
#[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
#[inline(always)]
unsafe fn store_4f32(ptr: *mut f32, v: std::simd::Simd<f32, 4>) {
    *(ptr as *mut [f32; 4]) = v.to_array();
}

/// 4x4 matrix transpose using portable_simd swizzle operations.
/// On ARM (aarch64), compiles to NEON vzip/vtrn instructions.
/// On x86-64 (SSE), compiles to unpcklps/unpckhps/movlhps/movehlps.
#[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
#[inline(always)]
fn transpose_4x4(
    m0: Vector<f32>,
    m1: Vector<f32>,
    m2: Vector<f32>,
    m3: Vector<f32>,
) -> [Vector<f32>; 4] {
    use std::simd::simd_swizzle;
    // Stage 1: interleave pairs
    let n0 = simd_swizzle!(m0, m1, [0, 4, 1, 5]);
    let n1 = simd_swizzle!(m2, m3, [0, 4, 1, 5]);
    let n2 = simd_swizzle!(m0, m1, [2, 6, 3, 7]);
    let n3 = simd_swizzle!(m2, m3, [2, 6, 3, 7]);
    // Stage 2: merge
    [
        simd_swizzle!(n0, n1, [0, 1, 4, 5]),
        simd_swizzle!(n0, n1, [2, 3, 6, 7]),
        simd_swizzle!(n2, n3, [0, 1, 4, 5]),
        simd_swizzle!(n2, n3, [2, 3, 6, 7]),
    ]
}

/// Load 4 contiguous f32 values into a SIMD vector (non-AVX path).
#[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
#[inline(always)]
unsafe fn load_4f32(ptr: *const f32) -> Vector<f32> {
    Vector::<f32>::from_array(*(ptr as *const [f32; 4]))
}

/// Store 4 f32 values from a SIMD vector to contiguous memory (non-AVX path).
#[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
#[inline(always)]
unsafe fn store_4f32_v(ptr: *mut f32, v: Vector<f32>) {
    *(ptr as *mut [f32; 4]) = v.to_array();
}

impl Bodies {
    #[inline(always)]
    fn write_gather_inertia(
        index: i32,
        body_index_in_bundle: usize,
        states: &Buffer<BodyDynamics>,
        gathered_inertias: &mut BodyInertiaWide,
    ) {
        let source = &states.get(index).inertia.world;
        unsafe {
            let target_slot =
                GatherScatter::get_offset_instance_mut(gathered_inertias, body_index_in_bundle);
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.xx) =
                source.inverse_inertia_tensor.xx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.yx) =
                source.inverse_inertia_tensor.yx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.yy) =
                source.inverse_inertia_tensor.yy;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zx) =
                source.inverse_inertia_tensor.zx;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zy) =
                source.inverse_inertia_tensor.zy;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_inertia_tensor.zz) =
                source.inverse_inertia_tensor.zz;
            *GatherScatter::get_first_mut(&mut target_slot.inverse_mass) = source.inverse_mass;
        }
    }

    #[inline(always)]
    fn write_gather_motion_state(
        index: i32,
        body_index_in_bundle: usize,
        states: &Buffer<BodyDynamics>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        let state = &states.get(index).motion;
        unsafe {
            Vector3Wide::write_first(
                state.pose.position,
                GatherScatter::get_offset_instance_mut(position, body_index_in_bundle),
            );
            QuaternionWide::write_first(
                state.pose.orientation,
                GatherScatter::get_offset_instance_mut(orientation, body_index_in_bundle),
            );
            Vector3Wide::write_first(
                state.velocity.linear,
                GatherScatter::get_offset_instance_mut(&mut velocity.linear, body_index_in_bundle),
            );
            Vector3Wide::write_first(
                state.velocity.angular,
                GatherScatter::get_offset_instance_mut(&mut velocity.angular, body_index_in_bundle),
            );
        }
    }

    #[allow(dead_code)]
    unsafe fn fallback_gather_motion_state(
        states: *const BodyDynamics,
        encoded_body_indices: &Vector<i32>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        let p_position_x = &mut position.x as *mut Vector<f32> as *mut f32;
        let p_position_y = &mut position.y as *mut Vector<f32> as *mut f32;
        let p_position_z = &mut position.z as *mut Vector<f32> as *mut f32;
        let p_orientation_x = &mut orientation.x as *mut Vector<f32> as *mut f32;
        let p_orientation_y = &mut orientation.y as *mut Vector<f32> as *mut f32;
        let p_orientation_z = &mut orientation.z as *mut Vector<f32> as *mut f32;
        let p_orientation_w = &mut orientation.w as *mut Vector<f32> as *mut f32;
        let p_linear_x = &mut velocity.linear.x as *mut Vector<f32> as *mut f32;
        let p_linear_y = &mut velocity.linear.y as *mut Vector<f32> as *mut f32;
        let p_linear_z = &mut velocity.linear.z as *mut Vector<f32> as *mut f32;
        let p_angular_x = &mut velocity.angular.x as *mut Vector<f32> as *mut f32;
        let p_angular_y = &mut velocity.angular.y as *mut Vector<f32> as *mut f32;
        let p_angular_z = &mut velocity.angular.z as *mut Vector<f32> as *mut f32;

        for i in 0..Vector::<i32>::LEN {
            let encoded_body_index = encoded_body_indices[i];
            if encoded_body_index < 0 {
                continue;
            }
            let state_values = states
                .offset((encoded_body_index & Bodies::BODY_REFERENCE_MASK) as isize)
                as *const f32;
            *p_position_x.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_X);
            *p_position_y.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_Y);
            *p_position_z.add(i) = *state_values.add(MotionState::OFFSET_TO_POSITION_Z);
            *p_orientation_x.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_X);
            *p_orientation_y.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_Y);
            *p_orientation_z.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_Z);
            *p_orientation_w.add(i) = *state_values.add(MotionState::OFFSET_TO_ORIENTATION_W);
            *p_linear_x.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_X);
            *p_linear_y.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_Y);
            *p_linear_z.add(i) = *state_values.add(MotionState::OFFSET_TO_LINEAR_Z);
            *p_angular_x.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_X);
            *p_angular_y.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_Y);
            *p_angular_z.add(i) = *state_values.add(MotionState::OFFSET_TO_ANGULAR_Z);
        }
    }

    #[allow(dead_code)]
    unsafe fn fallback_gather_inertia(
        states: *const BodyDynamics,
        encoded_body_indices: &Vector<i32>,
        inertia: &mut BodyInertiaWide,
        offset_in_floats: usize,
    ) {
        let p_mass = &mut inertia.inverse_mass as *mut Vector<f32> as *mut f32;
        let p_inertia_xx = &mut inertia.inverse_inertia_tensor.xx as *mut Vector<f32> as *mut f32;
        let p_inertia_yx = &mut inertia.inverse_inertia_tensor.yx as *mut Vector<f32> as *mut f32;
        let p_inertia_yy = &mut inertia.inverse_inertia_tensor.yy as *mut Vector<f32> as *mut f32;
        let p_inertia_zx = &mut inertia.inverse_inertia_tensor.zx as *mut Vector<f32> as *mut f32;
        let p_inertia_zy = &mut inertia.inverse_inertia_tensor.zy as *mut Vector<f32> as *mut f32;
        let p_inertia_zz = &mut inertia.inverse_inertia_tensor.zz as *mut Vector<f32> as *mut f32;

        for i in 0..Vector::<i32>::LEN {
            let encoded_body_index = encoded_body_indices[i];
            if encoded_body_index < 0 {
                continue;
            }
            let inertia_values = (states
                .offset((encoded_body_index & Bodies::BODY_REFERENCE_MASK) as isize)
                as *const f32)
                .add(offset_in_floats);
            *p_inertia_xx.add(i) = *inertia_values.add(0);
            *p_inertia_yx.add(i) = *inertia_values.add(1);
            *p_inertia_yy.add(i) = *inertia_values.add(2);
            *p_inertia_zx.add(i) = *inertia_values.add(3);
            *p_inertia_zy.add(i) = *inertia_values.add(4);
            *p_inertia_zz.add(i) = *inertia_values.add(5);
            *p_mass.add(i) = *inertia_values.add(6);
        }
    }

    /// Transposes a bundle of array-of-structures layout motion states into a bundle of
    /// array-of-structures-of-arrays layout.
    /// Buffer size must be no larger than `Vector::<f32>::LEN`.
    pub unsafe fn transpose_motion_states(
        states: &Buffer<MotionState>,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
    ) {
        debug_assert!(states.len() > 0 && (states.len() as usize) <= Vector::<f32>::LEN);
        #[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
        {
            let len = states.len() as usize;
            let base = states.as_ptr();
            let z = Vector::<f32>::splat(0.0);
            let s = |i: usize| -> *const f32 { base.add(i) as *const f32 };

            // First half: orientation (floats 0-3) + position (floats 4-6) + padding
            let m0 = load_8f32(s(0));
            let m1 = if len <= 1 { z } else { load_8f32(s(1)) };
            let m2 = if len <= 2 { z } else { load_8f32(s(2)) };
            let m3 = if len <= 3 { z } else { load_8f32(s(3)) };
            let m4 = if len <= 4 { z } else { load_8f32(s(4)) };
            let m5 = if len <= 5 { z } else { load_8f32(s(5)) };
            let m6 = if len <= 6 { z } else { load_8f32(s(6)) };
            let m7 = if len <= 7 { z } else { load_8f32(s(7)) };

            let t = transpose_8x8(m0, m1, m2, m3, m4, m5, m6, m7);
            orientation.x = t[0];
            orientation.y = t[1];
            orientation.z = t[2];
            orientation.w = t[3];
            position.x = t[4];
            position.y = t[5];
            position.z = t[6];

            // Second half: velocity (floats 8-10=linear, 12-14=angular, 11&15=padding)
            let m0 = load_8f32(s(0).add(8));
            let m1 = if len <= 1 { z } else { load_8f32(s(1).add(8)) };
            let m2 = if len <= 2 { z } else { load_8f32(s(2).add(8)) };
            let m3 = if len <= 3 { z } else { load_8f32(s(3).add(8)) };
            let m4 = if len <= 4 { z } else { load_8f32(s(4).add(8)) };
            let m5 = if len <= 5 { z } else { load_8f32(s(5).add(8)) };
            let m6 = if len <= 6 { z } else { load_8f32(s(6).add(8)) };
            let m7 = if len <= 7 { z } else { load_8f32(s(7).add(8)) };

            let t = transpose_8x8(m0, m1, m2, m3, m4, m5, m6, m7);
            velocity.linear.x = t[0];
            velocity.linear.y = t[1];
            velocity.linear.z = t[2];
            // t[3] is padding (_pad0), skip
            velocity.angular.x = t[4];
            velocity.angular.y = t[5];
            velocity.angular.z = t[6];
            // t[7] is padding (_pad1), skip
        }
        #[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
        {
            // 4×4 SIMD transpose — compiles to NEON vzip/vtrn on ARM, SSE unpack on x86.
            let len = states.len() as usize;
            let base = states.as_ptr();
            let z = Vector::<f32>::splat(0.0);
            let s = |i: usize| -> *const f32 { base.add(i) as *const f32 };

            // Block 1: orientation (floats 0-3: ori.x, ori.y, ori.z, ori.w)
            {
                let m0 = load_4f32(s(0));
                let m1 = if len <= 1 { z } else { load_4f32(s(1)) };
                let m2 = if len <= 2 { z } else { load_4f32(s(2)) };
                let m3 = if len <= 3 { z } else { load_4f32(s(3)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                orientation.x = t[0];
                orientation.y = t[1];
                orientation.z = t[2];
                orientation.w = t[3];
            }

            // Block 2: position + padding (floats 4-7: pos.x, pos.y, pos.z, pad)
            {
                let m0 = load_4f32(s(0).add(4));
                let m1 = if len <= 1 { z } else { load_4f32(s(1).add(4)) };
                let m2 = if len <= 2 { z } else { load_4f32(s(2).add(4)) };
                let m3 = if len <= 3 { z } else { load_4f32(s(3).add(4)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                position.x = t[0];
                position.y = t[1];
                position.z = t[2];
                // t[3] is padding, skip
            }

            // Block 3: linear velocity + padding (floats 8-11: lin.x, lin.y, lin.z, pad)
            {
                let m0 = load_4f32(s(0).add(8));
                let m1 = if len <= 1 { z } else { load_4f32(s(1).add(8)) };
                let m2 = if len <= 2 { z } else { load_4f32(s(2).add(8)) };
                let m3 = if len <= 3 { z } else { load_4f32(s(3).add(8)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                velocity.linear.x = t[0];
                velocity.linear.y = t[1];
                velocity.linear.z = t[2];
                // t[3] is padding, skip
            }

            // Block 4: angular velocity + padding (floats 12-15: ang.x, ang.y, ang.z, pad)
            {
                let m0 = load_4f32(s(0).add(12));
                let m1 = if len <= 1 { z } else { load_4f32(s(1).add(12)) };
                let m2 = if len <= 2 { z } else { load_4f32(s(2).add(12)) };
                let m3 = if len <= 3 { z } else { load_4f32(s(3).add(12)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                velocity.angular.x = t[0];
                velocity.angular.y = t[1];
                velocity.angular.z = t[2];
                // t[3] is padding, skip
            }
        }
    }

    /// Gathers state (position, orientation, velocity, inertia) from per-body AOS storage into
    /// AOSOA wide bundles. Uses AVX 8x8 transpose on x86-64, scalar fallback on ARM.
    pub unsafe fn gather_state<TAccessFilter: IBodyAccessFilter>(
        &self,
        encoded_body_indices: &Vector<i32>,
        world_inertia: bool,
        position: &mut Vector3Wide,
        orientation: &mut QuaternionWide,
        velocity: &mut BodyVelocityWide,
        inertia: &mut BodyInertiaWide,
    ) {
        let solver_states = self.active_set().dynamics_state.as_ptr();

        #[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
        {
            let z = Vector::<f32>::splat(0.0);

            // Extract body indices and compute base float pointers.
            let bi0 = encoded_body_indices[0];
            let empty0 = bi0 < 0;
            let s0 = solver_states.offset((bi0 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi1 = encoded_body_indices[1];
            let empty1 = bi1 < 0;
            let s1 = solver_states.offset((bi1 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi2 = encoded_body_indices[2];
            let empty2 = bi2 < 0;
            let s2 = solver_states.offset((bi2 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi3 = encoded_body_indices[3];
            let empty3 = bi3 < 0;
            let s3 = solver_states.offset((bi3 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi4 = encoded_body_indices[4];
            let empty4 = bi4 < 0;
            let s4 = solver_states.offset((bi4 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi5 = encoded_body_indices[5];
            let empty5 = bi5 < 0;
            let s5 = solver_states.offset((bi5 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi6 = encoded_body_indices[6];
            let empty6 = bi6 < 0;
            let s6 = solver_states.offset((bi6 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi7 = encoded_body_indices[7];
            let empty7 = bi7 < 0;
            let s7 = solver_states.offset((bi7 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;

            // First half of MotionState (orientation + position).
            {
                let m0 = if empty0 { z } else { load_8f32(s0) };
                let m1 = if empty1 { z } else { load_8f32(s1) };
                let m2 = if empty2 { z } else { load_8f32(s2) };
                let m3 = if empty3 { z } else { load_8f32(s3) };
                let m4 = if empty4 { z } else { load_8f32(s4) };
                let m5 = if empty5 { z } else { load_8f32(s5) };
                let m6 = if empty6 { z } else { load_8f32(s6) };
                let m7 = if empty7 { z } else { load_8f32(s7) };

                let t = transpose_8x8(m0, m1, m2, m3, m4, m5, m6, m7);
                if TAccessFilter::gather_orientation() {
                    orientation.x = t[0];
                    orientation.y = t[1];
                    orientation.z = t[2];
                    orientation.w = t[3];
                }
                if TAccessFilter::gather_position() {
                    position.x = t[4];
                    position.y = t[5];
                    position.z = t[6];
                }
            }

            // Second half of MotionState (velocity).
            {
                let m0 = if empty0 { z } else { load_8f32(s0.add(8)) };
                let m1 = if empty1 { z } else { load_8f32(s1.add(8)) };
                let m2 = if empty2 { z } else { load_8f32(s2.add(8)) };
                let m3 = if empty3 { z } else { load_8f32(s3.add(8)) };
                let m4 = if empty4 { z } else { load_8f32(s4.add(8)) };
                let m5 = if empty5 { z } else { load_8f32(s5.add(8)) };
                let m6 = if empty6 { z } else { load_8f32(s6.add(8)) };
                let m7 = if empty7 { z } else { load_8f32(s7.add(8)) };

                let t = transpose_8x8(m0, m1, m2, m3, m4, m5, m6, m7);
                if TAccessFilter::access_linear_velocity() {
                    velocity.linear.x = t[0];
                    velocity.linear.y = t[1];
                    velocity.linear.z = t[2];
                }
                if TAccessFilter::access_angular_velocity() {
                    velocity.angular.x = t[4];
                    velocity.angular.y = t[5];
                    velocity.angular.z = t[6];
                }
            }

            // Inertia.
            {
                let offset_in_floats: usize = if world_inertia { 24 } else { 16 };
                let m0 = if empty0 { z } else { load_8f32(s0.add(offset_in_floats)) };
                let m1 = if empty1 { z } else { load_8f32(s1.add(offset_in_floats)) };
                let m2 = if empty2 { z } else { load_8f32(s2.add(offset_in_floats)) };
                let m3 = if empty3 { z } else { load_8f32(s3.add(offset_in_floats)) };
                let m4 = if empty4 { z } else { load_8f32(s4.add(offset_in_floats)) };
                let m5 = if empty5 { z } else { load_8f32(s5.add(offset_in_floats)) };
                let m6 = if empty6 { z } else { load_8f32(s6.add(offset_in_floats)) };
                let m7 = if empty7 { z } else { load_8f32(s7.add(offset_in_floats)) };

                let t = transpose_8x8(m0, m1, m2, m3, m4, m5, m6, m7);
                if TAccessFilter::gather_inertia_tensor() {
                    inertia.inverse_inertia_tensor.xx = t[0];
                    inertia.inverse_inertia_tensor.yx = t[1];
                    inertia.inverse_inertia_tensor.yy = t[2];
                    inertia.inverse_inertia_tensor.zx = t[3];
                    inertia.inverse_inertia_tensor.zy = t[4];
                    inertia.inverse_inertia_tensor.zz = t[5];
                }
                if TAccessFilter::gather_mass() {
                    inertia.inverse_mass = t[6];
                }
            }
        }

        #[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
        {
            // 4×4 SIMD transpose path for non-AVX (ARM NEON, x86 SSE).
            let z = Vector::<f32>::splat(0.0);

            // Extract body indices and compute base float pointers.
            let bi0 = encoded_body_indices[0];
            let empty0 = bi0 < 0;
            let s0 = solver_states.offset((bi0 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi1 = encoded_body_indices[1];
            let empty1 = bi1 < 0;
            let s1 = solver_states.offset((bi1 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi2 = encoded_body_indices[2];
            let empty2 = bi2 < 0;
            let s2 = solver_states.offset((bi2 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;
            let bi3 = encoded_body_indices[3];
            let empty3 = bi3 < 0;
            let s3 = solver_states.offset((bi3 & Bodies::BODY_REFERENCE_MASK) as isize) as *const f32;

            // Block 1: orientation (floats 0-3)
            if TAccessFilter::gather_orientation() {
                let m0 = if empty0 { z } else { load_4f32(s0) };
                let m1 = if empty1 { z } else { load_4f32(s1) };
                let m2 = if empty2 { z } else { load_4f32(s2) };
                let m3 = if empty3 { z } else { load_4f32(s3) };
                let t = transpose_4x4(m0, m1, m2, m3);
                orientation.x = t[0];
                orientation.y = t[1];
                orientation.z = t[2];
                orientation.w = t[3];
            }

            // Block 2: position (floats 4-7)
            if TAccessFilter::gather_position() {
                let m0 = if empty0 { z } else { load_4f32(s0.add(4)) };
                let m1 = if empty1 { z } else { load_4f32(s1.add(4)) };
                let m2 = if empty2 { z } else { load_4f32(s2.add(4)) };
                let m3 = if empty3 { z } else { load_4f32(s3.add(4)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                position.x = t[0];
                position.y = t[1];
                position.z = t[2];
            }

            // Block 3: linear velocity (floats 8-11)
            if TAccessFilter::access_linear_velocity() {
                let m0 = if empty0 { z } else { load_4f32(s0.add(8)) };
                let m1 = if empty1 { z } else { load_4f32(s1.add(8)) };
                let m2 = if empty2 { z } else { load_4f32(s2.add(8)) };
                let m3 = if empty3 { z } else { load_4f32(s3.add(8)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                velocity.linear.x = t[0];
                velocity.linear.y = t[1];
                velocity.linear.z = t[2];
            }

            // Block 4: angular velocity (floats 12-15)
            if TAccessFilter::access_angular_velocity() {
                let m0 = if empty0 { z } else { load_4f32(s0.add(12)) };
                let m1 = if empty1 { z } else { load_4f32(s1.add(12)) };
                let m2 = if empty2 { z } else { load_4f32(s2.add(12)) };
                let m3 = if empty3 { z } else { load_4f32(s3.add(12)) };
                let t = transpose_4x4(m0, m1, m2, m3);
                velocity.angular.x = t[0];
                velocity.angular.y = t[1];
                velocity.angular.z = t[2];
            }

            // Inertia (7 useful values across 2 blocks of 4 floats)
            {
                let offset: usize = if world_inertia { 24 } else { 16 };
                // Block 5: [xx, yx, yy, zx]
                let m0 = if empty0 { z } else { load_4f32(s0.add(offset)) };
                let m1 = if empty1 { z } else { load_4f32(s1.add(offset)) };
                let m2 = if empty2 { z } else { load_4f32(s2.add(offset)) };
                let m3 = if empty3 { z } else { load_4f32(s3.add(offset)) };
                let t_a = transpose_4x4(m0, m1, m2, m3);

                // Block 6: [zy, zz, inv_mass, pad]
                let m0 = if empty0 { z } else { load_4f32(s0.add(offset + 4)) };
                let m1 = if empty1 { z } else { load_4f32(s1.add(offset + 4)) };
                let m2 = if empty2 { z } else { load_4f32(s2.add(offset + 4)) };
                let m3 = if empty3 { z } else { load_4f32(s3.add(offset + 4)) };
                let t_b = transpose_4x4(m0, m1, m2, m3);

                if TAccessFilter::gather_inertia_tensor() {
                    inertia.inverse_inertia_tensor.xx = t_a[0];
                    inertia.inverse_inertia_tensor.yx = t_a[1];
                    inertia.inverse_inertia_tensor.yy = t_a[2];
                    inertia.inverse_inertia_tensor.zx = t_a[3];
                    inertia.inverse_inertia_tensor.zy = t_b[0];
                    inertia.inverse_inertia_tensor.zz = t_b[1];
                }
                if TAccessFilter::gather_mass() {
                    inertia.inverse_mass = t_b[2];
                }
            }
        }
    }

    /// Scatters pose (position + orientation) from wide bundles back to per-body AOS storage.
    pub unsafe fn scatter_pose(
        &self,
        position: &Vector3Wide,
        orientation: &QuaternionWide,
        encoded_body_indices: &Vector<i32>,
        mask: &Vector<i32>,
    ) {
        #[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
        {
            let states = self.active_set_dynamics_ptr();
            // Reverse transpose: 7 SOA → 8 AOS (pos.z duplicated as 8th column = padding).
            let t = transpose_8x8(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
                position.x,
                position.y,
                position.z,
                position.z, // Laze: duplicated, lands in padding slot.
            );
            // Store each row to the corresponding body's MotionState (first 8 floats = pose).
            if mask[0] != 0 {
                store_8f32(states.add(encoded_body_indices[0] as usize) as *mut f32, t[0]);
            }
            if mask[1] != 0 {
                store_8f32(states.add(encoded_body_indices[1] as usize) as *mut f32, t[1]);
            }
            if mask[2] != 0 {
                store_8f32(states.add(encoded_body_indices[2] as usize) as *mut f32, t[2]);
            }
            if mask[3] != 0 {
                store_8f32(states.add(encoded_body_indices[3] as usize) as *mut f32, t[3]);
            }
            if mask[4] != 0 {
                store_8f32(states.add(encoded_body_indices[4] as usize) as *mut f32, t[4]);
            }
            if mask[5] != 0 {
                store_8f32(states.add(encoded_body_indices[5] as usize) as *mut f32, t[5]);
            }
            if mask[6] != 0 {
                store_8f32(states.add(encoded_body_indices[6] as usize) as *mut f32, t[6]);
            }
            if mask[7] != 0 {
                store_8f32(states.add(encoded_body_indices[7] as usize) as *mut f32, t[7]);
            }
        }
        #[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
        {
            let states = self.active_set_dynamics_ptr();
            // Reverse transpose: 4 SOA orientation → 4 AOS rows.
            let t_ori = transpose_4x4(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            );
            // Reverse transpose: 3 SOA position → 4 AOS rows (pos.z duplicated as padding).
            let t_pos = transpose_4x4(
                position.x,
                position.y,
                position.z,
                position.z, // Laze: duplicated, lands in padding slot.
            );
            // Store each row: 4 floats at offset 0 (orientation) + 4 floats at offset 4 (position).
            for i in 0..4 {
                if mask[i] != 0 {
                    let base = states.add(encoded_body_indices[i] as usize) as *mut f32;
                    store_4f32_v(base, t_ori[i]);
                    store_4f32_v(base.add(4), t_pos[i]);
                }
            }
        }
    }

    /// Scatters inertia from wide bundles back to per-body AOS storage (world inertia).
    pub unsafe fn scatter_inertia(
        &self,
        inertia: &BodyInertiaWide,
        encoded_body_indices: &Vector<i32>,
        mask: &Vector<i32>,
    ) {
        #[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
        {
            let states = self.active_set_dynamics_ptr();
            // 7 SOA inertia fields → 8 AOS rows (inverse_mass duplicated as padding).
            let t = transpose_8x8(
                inertia.inverse_inertia_tensor.xx,
                inertia.inverse_inertia_tensor.yx,
                inertia.inverse_inertia_tensor.yy,
                inertia.inverse_inertia_tensor.zx,
                inertia.inverse_inertia_tensor.zy,
                inertia.inverse_inertia_tensor.zz,
                inertia.inverse_mass,
                inertia.inverse_mass, // Laze: duplicated, lands in padding slot.
            );
            // Store at offset 24 floats (= world inertia) from each body's BodyDynamics.
            if mask[0] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[0] as usize) as *mut f32).add(24),
                    t[0],
                );
            }
            if mask[1] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[1] as usize) as *mut f32).add(24),
                    t[1],
                );
            }
            if mask[2] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[2] as usize) as *mut f32).add(24),
                    t[2],
                );
            }
            if mask[3] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[3] as usize) as *mut f32).add(24),
                    t[3],
                );
            }
            if mask[4] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[4] as usize) as *mut f32).add(24),
                    t[4],
                );
            }
            if mask[5] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[5] as usize) as *mut f32).add(24),
                    t[5],
                );
            }
            if mask[6] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[6] as usize) as *mut f32).add(24),
                    t[6],
                );
            }
            if mask[7] != 0 {
                store_8f32(
                    (states.add(encoded_body_indices[7] as usize) as *mut f32).add(24),
                    t[7],
                );
            }
        }
        #[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
        {
            let states = self.active_set_dynamics_ptr();
            // Reverse transpose: 7 SOA inertia fields → 4 AOS rows across 2 blocks.
            // Block A: [xx, yx, yy, zx]
            let t_a = transpose_4x4(
                inertia.inverse_inertia_tensor.xx,
                inertia.inverse_inertia_tensor.yx,
                inertia.inverse_inertia_tensor.yy,
                inertia.inverse_inertia_tensor.zx,
            );
            // Block B: [zy, zz, inv_mass, inv_mass] (padding slot)
            let t_b = transpose_4x4(
                inertia.inverse_inertia_tensor.zy,
                inertia.inverse_inertia_tensor.zz,
                inertia.inverse_mass,
                inertia.inverse_mass, // Laze: duplicated, lands in padding slot.
            );
            // Store at offset 24 floats (= world inertia) from each body's BodyDynamics.
            for i in 0..4 {
                if mask[i] != 0 {
                    let base = (states.add(encoded_body_indices[i] as usize) as *mut f32).add(24);
                    store_4f32_v(base, t_a[i]);
                    store_4f32_v(base.add(4), t_b[i]);
                }
            }
        }
    }

    /// Scatters velocities from wide bundles back to per-body AOS storage.
    pub unsafe fn scatter_velocities<TAccessFilter: IBodyAccessFilter>(
        &self,
        source_velocities: &BodyVelocityWide,
        encoded_body_indices: &Vector<i32>,
    ) {
        #[cfg(all(target_arch = "x86_64", target_feature = "avx"))]
        {
            use std::simd::{simd_swizzle, Simd};
            let states = self.active_set_dynamics_ptr();

            if TAccessFilter::access_linear_velocity() ^ TAccessFilter::access_angular_velocity() {
                // Partial: only linear OR angular velocity (128-bit / 4-float stores).
                let (m0, m1, m2, target_offset): (Vector<f32>, Vector<f32>, Vector<f32>, usize) =
                    if TAccessFilter::access_linear_velocity() {
                        (
                            source_velocities.linear.x,
                            source_velocities.linear.y,
                            source_velocities.linear.z,
                            8,
                        )
                    } else {
                        (
                            source_velocities.angular.x,
                            source_velocities.angular.y,
                            source_velocities.angular.z,
                            12,
                        )
                    };

                // 3-input partial transpose (m2 self-paired, "laze alert").
                let n0 = simd_swizzle!(m0, m1, [0, 8, 1, 9, 4, 12, 5, 13]);
                let n1 = simd_swizzle!(m2, m2, [0, 8, 1, 9, 4, 12, 5, 13]);
                let n4 = simd_swizzle!(m0, m1, [2, 10, 3, 11, 6, 14, 7, 15]);
                let n5 = simd_swizzle!(m2, m2, [2, 10, 3, 11, 6, 14, 7, 15]);

                let o0 = simd_swizzle!(n0, n1, [0, 1, 8, 9, 4, 5, 12, 13]);
                let o2 = simd_swizzle!(n4, n5, [0, 1, 8, 9, 4, 5, 12, 13]);
                let o4 = simd_swizzle!(n0, n1, [2, 3, 10, 11, 6, 7, 14, 15]);
                let o6 = simd_swizzle!(n4, n5, [2, 3, 10, 11, 6, 7, 14, 15]);

                // Extract lower 128-bit (bodies 0-3) and upper 128-bit (bodies 4-7).
                let r0: Simd<f32, 4> = simd_swizzle!(o0, [0, 1, 2, 3]);
                let r1: Simd<f32, 4> = simd_swizzle!(o4, [0, 1, 2, 3]);
                let r2: Simd<f32, 4> = simd_swizzle!(o2, [0, 1, 2, 3]);
                let r3: Simd<f32, 4> = simd_swizzle!(o6, [0, 1, 2, 3]);
                let r4: Simd<f32, 4> = simd_swizzle!(o0, [4, 5, 6, 7]);
                let r5: Simd<f32, 4> = simd_swizzle!(o4, [4, 5, 6, 7]);
                let r6: Simd<f32, 4> = simd_swizzle!(o2, [4, 5, 6, 7]);
                let r7: Simd<f32, 4> = simd_swizzle!(o6, [4, 5, 6, 7]);

                let indices = encoded_body_indices as *const Vector<i32> as *const u32;
                if *indices.add(0) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(0) as usize) as *mut f32).add(target_offset),
                        r0,
                    );
                }
                if *indices.add(1) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(1) as usize) as *mut f32).add(target_offset),
                        r1,
                    );
                }
                if *indices.add(2) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(2) as usize) as *mut f32).add(target_offset),
                        r2,
                    );
                }
                if *indices.add(3) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(3) as usize) as *mut f32).add(target_offset),
                        r3,
                    );
                }
                if *indices.add(4) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(4) as usize) as *mut f32).add(target_offset),
                        r4,
                    );
                }
                if *indices.add(5) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(5) as usize) as *mut f32).add(target_offset),
                        r5,
                    );
                }
                if *indices.add(6) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(6) as usize) as *mut f32).add(target_offset),
                        r6,
                    );
                }
                if *indices.add(7) < Bodies::DYNAMIC_LIMIT {
                    store_4f32(
                        (states.add(*indices.add(7) as usize) as *mut f32).add(target_offset),
                        r7,
                    );
                }
            } else {
                // Full: both linear and angular velocity (256-bit / 8-float stores).
                let t = transpose_8x8(
                    source_velocities.linear.x,
                    source_velocities.linear.y,
                    source_velocities.linear.z,
                    source_velocities.linear.z,  // Laze: duplicated, lands in _pad0.
                    source_velocities.angular.x,
                    source_velocities.angular.y,
                    source_velocities.angular.z,
                    source_velocities.angular.z, // Laze: duplicated, lands in _pad1.
                );
                // Store at offset 8 floats (= velocity section of MotionState).
                let indices = encoded_body_indices as *const Vector<i32> as *const u32;
                if *indices.add(0) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(0) as usize) as *mut f32).add(8),
                        t[0],
                    );
                }
                if *indices.add(1) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(1) as usize) as *mut f32).add(8),
                        t[1],
                    );
                }
                if *indices.add(2) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(2) as usize) as *mut f32).add(8),
                        t[2],
                    );
                }
                if *indices.add(3) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(3) as usize) as *mut f32).add(8),
                        t[3],
                    );
                }
                if *indices.add(4) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(4) as usize) as *mut f32).add(8),
                        t[4],
                    );
                }
                if *indices.add(5) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(5) as usize) as *mut f32).add(8),
                        t[5],
                    );
                }
                if *indices.add(6) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(6) as usize) as *mut f32).add(8),
                        t[6],
                    );
                }
                if *indices.add(7) < Bodies::DYNAMIC_LIMIT {
                    store_8f32(
                        (states.add(*indices.add(7) as usize) as *mut f32).add(8),
                        t[7],
                    );
                }
            }
        }
        #[cfg(not(all(target_arch = "x86_64", target_feature = "avx")))]
        {
            let states = self.active_set_dynamics_ptr();

            if TAccessFilter::access_linear_velocity() {
                // Transpose [lin.x, lin.y, lin.z, lin.z] → 4 AOS rows.
                let t = transpose_4x4(
                    source_velocities.linear.x,
                    source_velocities.linear.y,
                    source_velocities.linear.z,
                    source_velocities.linear.z, // Laze: duplicated, lands in padding slot.
                );
                for i in 0..4 {
                    if (encoded_body_indices[i] as u32) < Bodies::DYNAMIC_LIMIT {
                        store_4f32_v(
                            (states.add(encoded_body_indices[i] as usize) as *mut f32).add(8),
                            t[i],
                        );
                    }
                }
            }

            if TAccessFilter::access_angular_velocity() {
                // Transpose [ang.x, ang.y, ang.z, ang.z] → 4 AOS rows.
                let t = transpose_4x4(
                    source_velocities.angular.x,
                    source_velocities.angular.y,
                    source_velocities.angular.z,
                    source_velocities.angular.z, // Laze: duplicated, lands in padding slot.
                );
                for i in 0..4 {
                    if (encoded_body_indices[i] as u32) < Bodies::DYNAMIC_LIMIT {
                        store_4f32_v(
                            (states.add(encoded_body_indices[i] as usize) as *mut f32).add(12),
                            t[i],
                        );
                    }
                }
            }
        }
    }

    /// Helper to get a raw pointer to the DynamicsState buffer of the active set.
    #[inline(always)]
    unsafe fn active_set_dynamics_ptr(&self) -> *mut BodyDynamics {
        self.active_set().dynamics_state.as_ptr() as *mut BodyDynamics
    }
}
