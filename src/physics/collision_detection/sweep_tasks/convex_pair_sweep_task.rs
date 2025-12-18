// Translated from BepuPhysics/CollisionDetection/SweepTasks/ConvexSweepTaskCommon.cs

use crate::physics::body_properties::{BodyVelocity, RigidPose, RigidPoseWide};
use crate::physics::collidables::compound::Compound;
use crate::physics::collidables::shape::{IConvexShape, IShapeWide};
use crate::physics::collision_detection::sweep_task_registry::{SweepTask, SweepTaskRegistry};
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::physics::pose_integration::PoseIntegration;
use crate::physics::collidables::shapes::Shapes;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::quaternion_ex;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::{Quat, Vec3};
use std::marker::PhantomData;
use std::simd::prelude::*;
use std::simd::StdFloat;

const VECTOR_WIDTH: usize = crate::utilities::vector::VECTOR_WIDTH;

#[inline(always)]
fn get_sphere_cast_interval(
    origin: Vec3,
    direction: Vec3,
    radius: f32,
    t0: &mut f32,
    t1: &mut f32,
) -> bool {
    // Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
    let d_length = direction.length();
    if d_length == 0.0 {
        *t0 = 0.0;
        *t1 = f32::MAX;
        return origin.length_squared() <= radius * radius;
    }
    let inverse_d_length = 1.0 / d_length;
    let d = direction * inverse_d_length;

    // Move the origin up to the earliest possible impact time.
    let mut t_offset = -origin.dot(d) - radius;
    if t_offset < 0.0 {
        t_offset = 0.0;
    }
    let o = origin + d * t_offset;
    let b = o.dot(d);
    let c = o.dot(o) - radius * radius;

    if b > 0.0 && c > 0.0 {
        *t0 = 0.0;
        *t1 = 0.0;
        return false;
    }

    let discriminant = b * b - c;
    if discriminant < 0.0 {
        *t0 = 0.0;
        *t1 = 0.0;
        return false;
    }
    let interval_radius = discriminant.sqrt();
    *t0 = (t_offset - interval_radius - b) * inverse_d_length;
    *t1 = (t_offset + interval_radius - b) * inverse_d_length;
    true
}

#[inline(always)]
fn get_sample_times(t0: f32, t1: f32, samples: &mut Vector<f32>) {
    let sample_spacing = (t1 - t0) * (1.0 / (VECTOR_WIDTH as f32 - 1.0));
    for i in 0..VECTOR_WIDTH {
        samples[i] = t0 + i as f32 * sample_spacing;
    }
}

trait ISweepModifier<TShapeA: IConvexShape, TShapeWideA: IShapeWide<TShapeA>, TShapeB: IConvexShape, TShapeWideB: IShapeWide<TShapeB>, TPairDistanceTester> {
    fn get_sphere_cast_interval(
        &mut self,
        offset_b: Vec3,
        linear_velocity_b: Vec3,
        maximum_t: f32,
        maximum_radius_a: f32,
        maximum_radius_b: f32,
        orientation_a: Quat,
        angular_velocity_a: Vec3,
        angular_speed_a: f32,
        orientation_b: Quat,
        angular_velocity_b: Vec3,
        angular_speed_b: f32,
        t0: &mut f32,
        t1: &mut f32,
        hit_normal: &mut Vec3,
        hit_location: &mut Vec3,
    ) -> bool;

    fn construct_samples(
        &self,
        t0: f32,
        t1: f32,
        linear_b: &Vector3Wide,
        angular_a: &Vector3Wide,
        angular_b: &Vector3Wide,
        initial_offset_b: &Vector3Wide,
        initial_orientation_a: &QuaternionWide,
        initial_orientation_b: &QuaternionWide,
        samples: &mut Vector<f32>,
        sample_offset_b: &mut Vector3Wide,
        sample_orientation_a: &mut QuaternionWide,
        sample_orientation_b: &mut QuaternionWide,
    );

    fn get_nonlinear_velocity_contribution(
        &self,
        normal: &Vector3Wide,
        velocity_contribution_a: &mut Vector<f32>,
        maximum_displacement_a: &mut Vector<f32>,
        velocity_contribution_b: &mut Vector<f32>,
        maximum_displacement_b: &mut Vector<f32>,
    );

    fn adjust_hit_location(
        &self,
        initial_orientation_a: Quat,
        velocity_a: &BodyVelocity,
        t0: f32,
        hit_location: &mut Vec3,
    );
}

struct UnoffsetSweep;

impl<TShapeA: IConvexShape, TShapeWideA: IShapeWide<TShapeA>, TShapeB: IConvexShape, TShapeWideB: IShapeWide<TShapeB>, TPairDistanceTester>
    ISweepModifier<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>
    for UnoffsetSweep
{
    #[inline(always)]
    fn adjust_hit_location(
        &self,
        _initial_orientation_a: Quat,
        velocity_a: &BodyVelocity,
        t0: f32,
        hit_location: &mut Vec3,
    ) {
        *hit_location += t0 * velocity_a.linear;
    }

    #[inline(always)]
    fn construct_samples(
        &self,
        t0: f32,
        t1: f32,
        linear_b: &Vector3Wide,
        angular_a: &Vector3Wide,
        angular_b: &Vector3Wide,
        initial_offset_b: &Vector3Wide,
        initial_orientation_a: &QuaternionWide,
        initial_orientation_b: &QuaternionWide,
        samples: &mut Vector<f32>,
        sample_offset_b: &mut Vector3Wide,
        sample_orientation_a: &mut QuaternionWide,
        sample_orientation_b: &mut QuaternionWide,
    ) {
        get_sample_times(t0, t1, samples);
        // Integrate offsetB to sample locations.
        let mut displacement = Vector3Wide::default();
        Vector3Wide::scale_to(linear_b, samples, &mut displacement);
        Vector3Wide::add(initial_offset_b, &displacement, sample_offset_b);

        // Integrate orientations to sample locations.
        let half_samples = *samples * Vector::<f32>::splat(0.5);
        PoseIntegration::integrate_orientation_wide(initial_orientation_a, angular_a, &half_samples, sample_orientation_a);
        PoseIntegration::integrate_orientation_wide(initial_orientation_b, angular_b, &half_samples, sample_orientation_b);
    }

    #[inline(always)]
    fn get_sphere_cast_interval(
        &mut self,
        offset_b: Vec3,
        linear_velocity_b: Vec3,
        _maximum_t: f32,
        maximum_radius_a: f32,
        maximum_radius_b: f32,
        _orientation_a: Quat,
        _angular_velocity_a: Vec3,
        _angular_speed_a: f32,
        _orientation_b: Quat,
        _angular_velocity_b: Vec3,
        _angular_speed_b: f32,
        t0: &mut f32,
        t1: &mut f32,
        hit_normal: &mut Vec3,
        hit_location: &mut Vec3,
    ) -> bool {
        let hit = get_sphere_cast_interval(
            offset_b,
            linear_velocity_b,
            maximum_radius_a + maximum_radius_b,
            t0,
            t1,
        );
        *hit_location = offset_b + linear_velocity_b * *t0;
        *hit_normal = (-*hit_location).normalize_or_zero(); // Normals are calibrated to point from B to A.
        *hit_location += *hit_normal * maximum_radius_b;
        hit
    }

    #[inline(always)]
    fn get_nonlinear_velocity_contribution(
        &self,
        _normal: &Vector3Wide,
        velocity_contribution_a: &mut Vector<f32>,
        maximum_displacement_a: &mut Vector<f32>,
        velocity_contribution_b: &mut Vector<f32>,
        maximum_displacement_b: &mut Vector<f32>,
    ) {
        *velocity_contribution_a = Vector::<f32>::splat(0.0);
        *maximum_displacement_a = Vector::<f32>::splat(0.0);
        *velocity_contribution_b = Vector::<f32>::splat(0.0);
        *maximum_displacement_b = Vector::<f32>::splat(0.0);
    }
}

struct OffsetSweep {
    local_pose_a: RigidPose,
    local_pose_b: RigidPose,
    tangent_speed_a: f32,
    tangent_speed_b: f32,
    twice_radius_a: f32,
    twice_radius_b: f32,
    angular_velocity_direction_a: Vec3,
    angular_velocity_direction_b: Vec3,
}

impl<TShapeA: IConvexShape, TShapeWideA: IShapeWide<TShapeA>, TShapeB: IConvexShape, TShapeWideB: IShapeWide<TShapeB>, TPairDistanceTester>
    ISweepModifier<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>
    for OffsetSweep
{
    #[inline(always)]
    fn adjust_hit_location(
        &self,
        initial_orientation_a: Quat,
        velocity_a: &BodyVelocity,
        t0: f32,
        hit_location: &mut Vec3,
    ) {
        let mut integrated_pose = RigidPose::default();
        PoseIntegration::integrate_pose(
            &RigidPose {
                position: Vec3::ZERO,
                orientation: initial_orientation_a,
            },
            velocity_a,
            t0,
            &mut integrated_pose,
        );
        let mut child_offset = Vec3::ZERO;
        quaternion_ex::transform_into(self.local_pose_a.position, integrated_pose.orientation, &mut child_offset);
        *hit_location = *hit_location + integrated_pose.position + child_offset;
    }

    #[inline(always)]
    fn construct_samples(
        &self,
        t0: f32,
        t1: f32,
        linear_b: &Vector3Wide,
        angular_a: &Vector3Wide,
        angular_b: &Vector3Wide,
        initial_offset_b: &Vector3Wide,
        initial_orientation_a: &QuaternionWide,
        initial_orientation_b: &QuaternionWide,
        samples: &mut Vector<f32>,
        sample_offset_b: &mut Vector3Wide,
        sample_orientation_a: &mut QuaternionWide,
        sample_orientation_b: &mut QuaternionWide,
    ) {
        get_sample_times(t0, t1, samples);
        // Integrate offsetB to sample locations.
        let mut displacement = Vector3Wide::default();
        Vector3Wide::scale_to(linear_b, samples, &mut displacement);
        Vector3Wide::add(initial_offset_b, &displacement, sample_offset_b);

        // Note that the initial orientations are properties of the owning body, not of the child.
        let half_samples = *samples * Vector::<f32>::splat(0.5);
        let mut local_poses_a = RigidPoseWide::default();
        RigidPoseWide::broadcast(&self.local_pose_a, &mut local_poses_a);
        let mut integrated_orientation_a = QuaternionWide::default();
        PoseIntegration::integrate_orientation_wide(initial_orientation_a, angular_a, &half_samples, &mut integrated_orientation_a);
        let mut child_position_a = Vector3Wide::default();
        Compound::get_rotated_child_pose_wide(&local_poses_a, &integrated_orientation_a, &mut child_position_a, sample_orientation_a);

        let mut local_poses_b = RigidPoseWide::default();
        RigidPoseWide::broadcast(&self.local_pose_b, &mut local_poses_b);
        let mut integrated_orientation_b = QuaternionWide::default();
        PoseIntegration::integrate_orientation_wide(initial_orientation_b, angular_b, &half_samples, &mut integrated_orientation_b);
        let mut child_position_b = Vector3Wide::default();
        Compound::get_rotated_child_pose_wide(&local_poses_b, &integrated_orientation_b, &mut child_position_b, sample_orientation_b);

        let mut net_offset_b = Vector3Wide::default();
        Vector3Wide::subtract(&child_position_b, &child_position_a, &mut net_offset_b);
        let current_offset_b = *sample_offset_b;
        Vector3Wide::add(&current_offset_b, &net_offset_b, sample_offset_b);
    }

    #[inline(always)]
    fn get_sphere_cast_interval(
        &mut self,
        offset_b: Vec3,
        linear_velocity_b: Vec3,
        maximum_t: f32,
        maximum_radius_a: f32,
        maximum_radius_b: f32,
        orientation_a: Quat,
        angular_velocity_a: Vec3,
        angular_speed_a: f32,
        orientation_b: Quat,
        angular_velocity_b: Vec3,
        angular_speed_b: f32,
        t0: &mut f32,
        t1: &mut f32,
        hit_normal: &mut Vec3,
        hit_location: &mut Vec3,
    ) -> bool {
        let mut r_a = Vec3::ZERO;
        quaternion_ex::transform_without_overlap(self.local_pose_a.position, orientation_a, &mut r_a);
        let tangent_a = r_a.cross(angular_velocity_a);
        self.tangent_speed_a = tangent_a.length();
        let mut r_b = Vec3::ZERO;
        quaternion_ex::transform_without_overlap(self.local_pose_b.position, orientation_b, &mut r_b);
        let tangent_b = r_b.cross(angular_velocity_b);
        self.tangent_speed_b = tangent_b.length();
        self.twice_radius_a = 2.0 * self.local_pose_a.position.length();
        self.twice_radius_b = 2.0 * self.local_pose_b.position.length();
        self.angular_velocity_direction_a = if angular_speed_a > 1e-8 {
            angular_velocity_a / angular_speed_a
        } else {
            Vec3::ZERO
        };
        self.angular_velocity_direction_b = if angular_speed_b > 1e-8 {
            angular_velocity_b / angular_speed_b
        } else {
            Vec3::ZERO
        };
        let nonlinear_expansion = (maximum_t * (self.tangent_speed_a + self.tangent_speed_b))
            .min(self.twice_radius_a + self.twice_radius_b);
        let offset_b_including_child_poses = offset_b + r_b - r_a;
        let hit = get_sphere_cast_interval(
            offset_b_including_child_poses,
            linear_velocity_b,
            maximum_radius_a + maximum_radius_b + nonlinear_expansion,
            t0,
            t1,
        );
        *hit_location = offset_b_including_child_poses + linear_velocity_b * *t0;
        *hit_normal = (-*hit_location).normalize_or_zero();
        *hit_location += *hit_normal * (maximum_radius_b + nonlinear_expansion);
        hit
    }

    #[inline(always)]
    fn get_nonlinear_velocity_contribution(
        &self,
        normal: &Vector3Wide,
        velocity_contribution_a: &mut Vector<f32>,
        maximum_displacement_a: &mut Vector<f32>,
        velocity_contribution_b: &mut Vector<f32>,
        maximum_displacement_b: &mut Vector<f32>,
    ) {
        let mut direction_a = Vector3Wide::default();
        Vector3Wide::broadcast_to(self.angular_velocity_direction_a, &mut direction_a);
        let mut dot_a = Vector::<f32>::default();
        Vector3Wide::dot(normal, &direction_a, &mut dot_a);
        let scale_a = (Vector::<f32>::splat(0.0).simd_max(Vector::<f32>::splat(1.0) - dot_a * dot_a)).sqrt();
        *velocity_contribution_a = Vector::<f32>::splat(self.tangent_speed_a) * scale_a;
        *maximum_displacement_a = Vector::<f32>::splat(self.twice_radius_a) * scale_a;
        let mut direction_b = Vector3Wide::default();
        Vector3Wide::broadcast_to(self.angular_velocity_direction_b, &mut direction_b);
        let mut dot_b = Vector::<f32>::default();
        Vector3Wide::dot(normal, &direction_b, &mut dot_b);
        let scale_b = (Vector::<f32>::splat(0.0).simd_max(Vector::<f32>::splat(1.0) - dot_b * dot_b)).sqrt();
        *velocity_contribution_b = Vector::<f32>::splat(self.tangent_speed_b) * scale_b;
        *maximum_displacement_b = Vector::<f32>::splat(self.twice_radius_b) * scale_b;
    }
}

#[allow(clippy::too_many_arguments)]
unsafe fn sweep<
    TShapeA: IConvexShape,
    TShapeWideA: IShapeWide<TShapeA> + Default,
    TShapeB: IConvexShape,
    TShapeWideB: IShapeWide<TShapeB> + Default,
    TPairDistanceTester: IPairDistanceTester<TShapeWideA, TShapeWideB> + Default,
    TSweepModifier: ISweepModifier<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>,
>(
    shape_data_a: *const u8,
    orientation_a: Quat,
    velocity_a: &BodyVelocity,
    shape_data_b: *const u8,
    offset_b: Vec3,
    orientation_b: Quat,
    velocity_b: &BodyVelocity,
    maximum_t: f32,
    minimum_progression: f32,
    convergence_threshold: f32,
    maximum_iteration_count: i32,
    sweep_modifier: &mut TSweepModifier,
    t0: &mut f32,
    t1: &mut f32,
    hit_location: &mut Vec3,
    hit_normal: &mut Vec3,
) -> bool {
    let shape_a = &*(shape_data_a as *const TShapeA);
    let shape_b = &*(shape_data_b as *const TShapeB);
    let mut wide_a = TShapeWideA::default();
    let mut wide_b = TShapeWideB::default();
    if wide_a.internal_allocation_size() > 0 {
        let mut memory = vec![0u8; wide_a.internal_allocation_size()];
        wide_a.initialize(&crate::utilities::memory::buffer::Buffer::new(memory.as_mut_ptr(), memory.len() as i32, -1));
        std::mem::forget(memory);
    }
    if wide_b.internal_allocation_size() > 0 {
        let mut memory = vec![0u8; wide_b.internal_allocation_size()];
        wide_b.initialize(&crate::utilities::memory::buffer::Buffer::new(memory.as_mut_ptr(), memory.len() as i32, -1));
        std::mem::forget(memory);
    }
    wide_a.broadcast(shape_a);
    wide_b.broadcast(shape_b);
    let linear_velocity_b = velocity_b.linear - velocity_a.linear;
    let mut maximum_radius_a = 0.0f32;
    let mut maximum_angular_expansion_a = 0.0f32;
    shape_a.compute_angular_expansion_data(&mut maximum_radius_a, &mut maximum_angular_expansion_a);
    let mut maximum_radius_b = 0.0f32;
    let mut maximum_angular_expansion_b = 0.0f32;
    shape_b.compute_angular_expansion_data(&mut maximum_radius_b, &mut maximum_angular_expansion_b);
    let angular_speed_a = velocity_a.angular.length();
    let angular_speed_b = velocity_b.angular.length();
    if !sweep_modifier.get_sphere_cast_interval(
        offset_b,
        linear_velocity_b,
        maximum_t,
        maximum_radius_a,
        maximum_radius_b,
        orientation_a,
        velocity_a.angular,
        angular_speed_a,
        orientation_b,
        velocity_b.angular,
        angular_speed_b,
        t0,
        t1,
        hit_normal,
        hit_location,
    ) || *t0 > maximum_t
        || *t1 < 0.0
    {
        *hit_location = Vec3::ZERO;
        *hit_normal = Vec3::ZERO;
        return false;
    }
    if *t0 < 0.0 {
        *t0 = 0.0;
    }
    if *t1 > maximum_t {
        *t1 = maximum_t;
    }

    let tangent_speed_a = Vector::<f32>::splat(maximum_radius_a * angular_speed_a);
    let tangent_speed_b = Vector::<f32>::splat(maximum_radius_b * angular_speed_b);
    let max_angular_expansion_a = Vector::<f32>::splat(maximum_angular_expansion_a);
    let max_angular_expansion_b = Vector::<f32>::splat(maximum_angular_expansion_b);

    let mut initial_offset_b = Vector3Wide::default();
    Vector3Wide::broadcast_to(offset_b, &mut initial_offset_b);
    let mut initial_orientation_a = QuaternionWide::default();
    QuaternionWide::broadcast(orientation_a, &mut initial_orientation_a);
    let mut initial_orientation_b = QuaternionWide::default();
    QuaternionWide::broadcast(orientation_b, &mut initial_orientation_b);
    let mut wide_linear_velocity_b = Vector3Wide::default();
    Vector3Wide::broadcast_to(linear_velocity_b, &mut wide_linear_velocity_b);
    let mut wide_angular_velocity_a = Vector3Wide::default();
    Vector3Wide::broadcast_to(velocity_a.angular, &mut wide_angular_velocity_a);
    let mut wide_angular_velocity_b = Vector3Wide::default();
    Vector3Wide::broadcast_to(velocity_b.angular, &mut wide_angular_velocity_b);

    let mut samples = Vector::<f32>::default();
    let mut sample_offset_b = Vector3Wide::default();
    let mut sample_orientation_a = QuaternionWide::default();
    let mut sample_orientation_b = QuaternionWide::default();
    let minimum_progression_wide = Vector::<f32>::splat(minimum_progression);

    let mut next0 = *t0;
    let mut next1 = *t1;
    sweep_modifier.construct_samples(
        *t0, *t1,
        &wide_linear_velocity_b, &wide_angular_velocity_a, &wide_angular_velocity_b,
        &initial_offset_b, &initial_orientation_a, &initial_orientation_b,
        &mut samples, &mut sample_offset_b, &mut sample_orientation_a, &mut sample_orientation_b,
    );

    let mut intersection_encountered = false;
    let mut iteration_index = 0i32;
    loop {
        let mut intersections = Vector::<i32>::default();
        let mut distances = Vector::<f32>::default();
        let mut closest_a = Vector3Wide::default();
        let mut normals = Vector3Wide::default();
        let pair_tester = TPairDistanceTester::default();
        pair_tester.test(
            &wide_a,
            &wide_b,
            &sample_offset_b,
            &sample_orientation_a,
            &sample_orientation_b,
            &Simd::splat(0),
            &mut intersections,
            &mut distances,
            &mut closest_a,
            &mut normals,
        );

        let mut linear_velocity_along_normal = Vector::<f32>::default();
        Vector3Wide::dot(&normals, &wide_linear_velocity_b, &mut linear_velocity_along_normal);
        let mut nonlinear_velocity_contribution_a = Vector::<f32>::default();
        let mut nonlinear_maximum_displacement_a = Vector::<f32>::default();
        let mut nonlinear_velocity_contribution_b = Vector::<f32>::default();
        let mut nonlinear_maximum_displacement_b = Vector::<f32>::default();
        sweep_modifier.get_nonlinear_velocity_contribution(
            &normals,
            &mut nonlinear_velocity_contribution_a,
            &mut nonlinear_maximum_displacement_a,
            &mut nonlinear_velocity_contribution_b,
            &mut nonlinear_maximum_displacement_b,
        );

        let zero = Vector::<f32>::splat(0.0);
        let a_worst_case_distances = zero.simd_max(distances - max_angular_expansion_a - nonlinear_maximum_displacement_a);
        let angular_displacement_b = max_angular_expansion_b + nonlinear_maximum_displacement_b;
        let b_worst_case_distances = zero.simd_max(distances - angular_displacement_b);
        let both_worst_case_distances = zero.simd_max(a_worst_case_distances - angular_displacement_b);
        let division_guard = Vector::<f32>::splat(1e-15);
        let both_worst_case_next_time = both_worst_case_distances / division_guard.simd_max(linear_velocity_along_normal);
        let angular_contribution_a = nonlinear_velocity_contribution_a + tangent_speed_a;
        let angular_contribution_b = nonlinear_velocity_contribution_b + tangent_speed_b;
        let a_worst_case_next_time = a_worst_case_distances / division_guard.simd_max(linear_velocity_along_normal + angular_contribution_b);
        let b_worst_case_next_time = b_worst_case_distances / division_guard.simd_max(linear_velocity_along_normal + angular_contribution_a);
        let best_case_next_time = distances / division_guard.simd_max(linear_velocity_along_normal + angular_contribution_a + angular_contribution_b);
        let time_to_next = both_worst_case_next_time
            .simd_max(a_worst_case_next_time)
            .simd_max(b_worst_case_next_time.simd_max(best_case_next_time));
        let a_worst_case_previous_time = a_worst_case_distances / division_guard.simd_max(angular_contribution_b - linear_velocity_along_normal);
        let b_worst_case_previous_time = b_worst_case_distances / division_guard.simd_max(angular_contribution_a - linear_velocity_along_normal);
        let best_case_previous_time = distances / division_guard.simd_max(angular_contribution_a + angular_contribution_b - linear_velocity_along_normal);
        let time_to_previous = (-both_worst_case_next_time)
            .simd_max(a_worst_case_previous_time)
            .simd_max(b_worst_case_previous_time.simd_max(best_case_previous_time));
        let safe_interval_start = samples - time_to_previous;
        let safe_interval_end = samples + time_to_next;
        let forced_interval_end = samples + time_to_next.simd_max(minimum_progression_wide);

        if intersections[0] < 0 {
            // First sample was intersected.
            next1 = samples[0];
            intersection_encountered = true;
        } else {
            let mut first_intersecting_index = VECTOR_WIDTH;
            for i in 0..VECTOR_WIDTH {
                if intersections[i] < 0 {
                    first_intersecting_index = i;
                    debug_assert!(samples[i] >= *t0);
                    next1 = samples[i];
                    intersection_encountered = true;
                    break;
                }
            }
            let mut last_safe_index = 0usize;
            for i in 0..first_intersecting_index {
                last_safe_index = i;
                let next_index = i + 1;
                if next_index < first_intersecting_index {
                    if safe_interval_start[i + 1] > forced_interval_end[i] {
                        break;
                    }
                }
            }
            debug_assert!(safe_interval_end[last_safe_index] >= *t0 || !intersection_encountered);
            next0 = safe_interval_end[last_safe_index];
            *hit_normal = Vec3::new(
                normals.x[last_safe_index],
                normals.y[last_safe_index],
                normals.z[last_safe_index],
            );
            *hit_location = Vec3::new(
                closest_a.x[last_safe_index],
                closest_a.y[last_safe_index],
                closest_a.z[last_safe_index],
            );

            if !intersection_encountered {
                for i in (0..VECTOR_WIDTH).rev() {
                    next1 = safe_interval_start[i];
                    if i > 0 && forced_interval_end[i - 1] < next1 {
                        break;
                    }
                }
            }
        }

        let mut sample0 = *t0 + minimum_progression;
        let mut sample1 = *t1 - minimum_progression;
        let previous_interval_span = *t1 - *t0;
        *t0 += (next0 - *t0) * 0.9999;
        *t1 = next1;

        let interval_span = *t1 - *t0;
        if interval_span < 0.0
            || (intersection_encountered && interval_span < convergence_threshold)
            || interval_span >= previous_interval_span
            || { iteration_index += 1; iteration_index } >= maximum_iteration_count
        {
            break;
        }

        if sample0 < *t0 {
            sample0 = *t0;
        } else if sample0 > *t1 {
            sample0 = *t1;
        }
        if sample1 < *t0 {
            sample1 = *t0;
        } else if sample1 > *t1 {
            sample1 = *t1;
        }

        let minimum_span = minimum_progression * (VECTOR_WIDTH as f32 - 1.0);
        let mut sample_span = sample1 - sample0;
        if sample_span < minimum_span {
            sample0 -= minimum_span - sample_span;
            if sample0 < *t0 {
                sample0 = *t0;
            }
            sample_span = sample1 - sample0;
            if sample_span < minimum_span {
                sample1 += (minimum_span - sample_span).min((*t1 - sample1) * 0.5);
            }
        }

        sweep_modifier.construct_samples(
            sample0, sample1,
            &wide_linear_velocity_b, &wide_angular_velocity_a, &wide_angular_velocity_b,
            &initial_offset_b, &initial_orientation_a, &initial_orientation_b,
            &mut samples, &mut sample_offset_b, &mut sample_orientation_a, &mut sample_orientation_b,
        );
    }
    sweep_modifier.adjust_hit_location(orientation_a, velocity_a, *t0, hit_location);
    intersection_encountered
}

/// A sweep task for convex shape pairs using a pair distance tester.
pub struct ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester> {
    shape_type_index_a: i32,
    shape_type_index_b: i32,
    _phantom: PhantomData<(TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester)>,
}

impl<
        TShapeA: IConvexShape + 'static,
        TShapeWideA: IShapeWide<TShapeA> + Default + 'static,
        TShapeB: IConvexShape + 'static,
        TShapeWideB: IShapeWide<TShapeB> + Default + 'static,
        TPairDistanceTester: IPairDistanceTester<TShapeWideA, TShapeWideB> + Default + 'static,
    > ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>
{
    pub fn new() -> Self {
        Self {
            shape_type_index_a: TShapeA::type_id(),
            shape_type_index_b: TShapeB::type_id(),
            _phantom: PhantomData,
        }
    }
}

impl<
        TShapeA: IConvexShape + 'static,
        TShapeWideA: IShapeWide<TShapeA> + Default + 'static,
        TShapeB: IConvexShape + 'static,
        TShapeWideB: IShapeWide<TShapeB> + Default + 'static,
        TPairDistanceTester: IPairDistanceTester<TShapeWideA, TShapeWideB> + Default + 'static,
    > SweepTask
    for ConvexPairSweepTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester>
{
    fn shape_type_index_a(&self) -> i32 {
        self.shape_type_index_a
    }
    fn shape_type_index_b(&self) -> i32 {
        self.shape_type_index_b
    }

    unsafe fn preordered_type_sweep(
        &self,
        shape_data_a: *const u8,
        local_pose_a: &RigidPose,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        local_pose_b: &RigidPose,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool {
        let mut sweep_modifier = OffsetSweep {
            local_pose_a: *local_pose_a,
            local_pose_b: *local_pose_b,
            tangent_speed_a: 0.0,
            tangent_speed_b: 0.0,
            twice_radius_a: 0.0,
            twice_radius_b: 0.0,
            angular_velocity_direction_a: Vec3::ZERO,
            angular_velocity_direction_b: Vec3::ZERO,
        };
        sweep::<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester, OffsetSweep>(
            shape_data_a,
            orientation_a,
            velocity_a,
            shape_data_b,
            offset_b,
            orientation_b,
            velocity_b,
            maximum_t,
            minimum_progression,
            convergence_threshold,
            maximum_iteration_count,
            &mut sweep_modifier,
            t0,
            t1,
            hit_location,
            hit_normal,
        )
    }

    unsafe fn preordered_type_sweep_filtered(
        &self,
        shape_data_a: *const u8,
        orientation_a: Quat,
        velocity_a: &BodyVelocity,
        shape_data_b: *const u8,
        offset_b: Vec3,
        orientation_b: Quat,
        velocity_b: &BodyVelocity,
        maximum_t: f32,
        minimum_progression: f32,
        convergence_threshold: f32,
        maximum_iteration_count: i32,
        _flip_required: bool,
        _filter: *mut u8,
        _shapes: *mut Shapes,
        _sweep_tasks: *mut SweepTaskRegistry,
        _pool: *mut BufferPool,
        t0: &mut f32,
        t1: &mut f32,
        hit_location: &mut Vec3,
        hit_normal: &mut Vec3,
    ) -> bool {
        let mut sweep_modifier = UnoffsetSweep;
        sweep::<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairDistanceTester, UnoffsetSweep>(
            shape_data_a,
            orientation_a,
            velocity_a,
            shape_data_b,
            offset_b,
            orientation_b,
            velocity_b,
            maximum_t,
            minimum_progression,
            convergence_threshold,
            maximum_iteration_count,
            &mut sweep_modifier,
            t0,
            t1,
            hit_location,
            hit_normal,
        )
    }
}
