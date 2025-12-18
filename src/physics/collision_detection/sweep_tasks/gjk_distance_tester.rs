// Translated from BepuPhysics/CollisionDetection/SweepTasks/GJKDistanceTester.cs

use crate::physics::collidables::shape::{IConvexShape, IShapeWide};
use crate::physics::collision_detection::support_finder::ISupportFinder;
use crate::physics::collision_detection::sweep_tasks::IPairDistanceTester;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::matrix3x3_wide::Matrix3x3Wide;
use crate::utilities::quaternion_wide::QuaternionWide;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use std::simd::prelude::*;
use std::simd::StdFloat;

pub struct GJKDistanceTester<
    TShapeA,
    TShapeWideA,
    TSupportFinderA,
    TShapeB,
    TShapeWideB,
    TSupportFinderB,
> {
    pub termination_epsilon: f32,
    pub containment_epsilon: f32,
    _phantom: std::marker::PhantomData<(
        TShapeA,
        TShapeWideA,
        TSupportFinderA,
        TShapeB,
        TShapeWideB,
        TSupportFinderB,
    )>,
}

impl<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
    GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
{
    pub const TERMINATION_EPSILON_DEFAULT: f32 = 1e-7;
    pub const CONTAINMENT_EPSILON_DEFAULT: f32 = 0.000316227766;

    pub fn new() -> Self {
        Self {
            termination_epsilon: 0.0,
            containment_epsilon: 0.0,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB> Default
    for GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
{
    fn default() -> Self {
        Self::new()
    }
}

struct Simplex {
    a: Vector3Wide,
    b: Vector3Wide,
    c: Vector3Wide,
    d: Vector3Wide,
    a_on_a: Vector3Wide,
    b_on_a: Vector3Wide,
    c_on_a: Vector3Wide,
    d_on_a: Vector3Wide,
    count: Vector<i32>,
}

const VECTOR_WIDTH: usize = crate::utilities::vector::VECTOR_WIDTH;

#[inline(always)]
fn sample_minkowski_difference<TShapeWideA, TShapeWideB, TSupportFinderA: ISupportFinder<TShapeWideA>, TSupportFinderB: ISupportFinder<TShapeWideB>>(
    a: &TShapeWideA,
    r_a: &Matrix3x3Wide,
    _support_finder_a: &TSupportFinderA,
    b: &TShapeWideB,
    r_b: &Matrix3x3Wide,
    _support_finder_b: &TSupportFinderB,
    offset_b: &Vector3Wide,
    direction: &Vector3Wide,
    terminated_lanes: &Vector<i32>,
    support_a: &mut Vector3Wide,
    support: &mut Vector3Wide,
) {
    TSupportFinderA::compute_support(a, r_a, direction, terminated_lanes, support_a);
    let mut negated_direction = Vector3Wide::default();
    Vector3Wide::negate(direction, &mut negated_direction);
    let mut support_b = Vector3Wide::default();
    TSupportFinderB::compute_support(b, r_b, &negated_direction, terminated_lanes, &mut support_b);
    support_b += *offset_b;
    Vector3Wide::subtract(support_a, &support_b, support);
}

#[inline(always)]
fn append(mask: &Vector<i32>, simplex: &mut Simplex, v_a: &Vector3Wide, v: &Vector3Wide) {
    for i in 0..VECTOR_WIDTH {
        if mask[i] == 0 {
            let count = unsafe {
                &mut *(GatherScatter::get_first_mut(&mut simplex.count) as *mut i32).add(i)
            };
            unsafe {
                // Get the target slot based on count
                let v_target = match *count {
                    0 => &mut simplex.a,
                    1 => &mut simplex.b,
                    2 => &mut simplex.c,
                    3 => &mut simplex.d,
                    _ => unreachable!(),
                };
                *((&mut v_target.x) as *mut _ as *mut f32).add(i) = v.x[i];
                *((&mut v_target.y) as *mut _ as *mut f32).add(i) = v.y[i];
                *((&mut v_target.z) as *mut _ as *mut f32).add(i) = v.z[i];

                let va_target = match *count {
                    0 => &mut simplex.a_on_a,
                    1 => &mut simplex.b_on_a,
                    2 => &mut simplex.c_on_a,
                    3 => &mut simplex.d_on_a,
                    _ => unreachable!(),
                };
                *((&mut va_target.x) as *mut _ as *mut f32).add(i) = v_a.x[i];
                *((&mut va_target.y) as *mut _ as *mut f32).add(i) = v_a.y[i];
                *((&mut va_target.z) as *mut _ as *mut f32).add(i) = v_a.z[i];
            }
            *count += 1;
        }
    }
}

#[inline(always)]
fn select(
    mask: &Vector<i32>,
    distance_squared: &mut Vector<f32>,
    closest: &mut Vector3Wide,
    closest_a: &mut Vector3Wide,
    feature_id: &mut Vector<i32>,
    distance_squared_candidate: &Vector<f32>,
    closest_candidate: &Vector3Wide,
    closest_a_candidate: &Vector3Wide,
    feature_id_candidate: &Vector<i32>,
) {
    let use_candidate = *mask & distance_squared_candidate.simd_lt(*distance_squared).to_int();
    *distance_squared = Mask::from_int(use_candidate).select(*distance_squared_candidate, *distance_squared);
    *closest = Vector3Wide::conditional_select(&use_candidate, closest_candidate, closest);
    *closest_a = Vector3Wide::conditional_select(&use_candidate, closest_a_candidate, closest_a);
    *feature_id = Mask::from_int(use_candidate).select(*feature_id_candidate, *feature_id);
}

#[inline(always)]
fn edge(
    a: &Vector3Wide,
    b: &Vector3Wide,
    a_on_a: &Vector3Wide,
    b_on_a: &Vector3Wide,
    ab: &mut Vector3Wide,
    abab: &mut Vector<f32>,
    ab_a: &mut Vector<f32>,
    a_feature_id: &Vector<i32>,
    b_feature_id: &Vector<i32>,
    mask: &Vector<i32>,
    distance_squared: &mut Vector<f32>,
    closest: &mut Vector3Wide,
    closest_a: &mut Vector3Wide,
    feature_id: &mut Vector<i32>,
) {
    Vector3Wide::subtract(b, a, ab);
    Vector3Wide::dot(ab, ab, abab);
    Vector3Wide::dot(ab, a, ab_a);
    // Note that vertex B (and technically A, too) is handled by clamping the edge. No need to handle the vertices by themselves (apart from the base case).
    let mut ab_t = -*ab_a / *abab;
    // Protect against division by zero. It's a degenerate edge, so just pick vertex a as a representative.
    ab_t = abab
        .abs()
        .simd_gt(Vector::<f32>::splat(1e-15))
        .select(ab_t, Vector::<f32>::splat(0.0));
    let a_feature_contribution = ab_t
        .simd_lt(Vector::<f32>::splat(1.0))
        .select(*a_feature_id, Simd::splat(0));
    let b_feature_contribution = ab_t
        .simd_gt(Vector::<f32>::splat(0.0))
        .select(*b_feature_id, Simd::splat(0));
    let feature_id_candidate = a_feature_contribution | b_feature_contribution;

    ab_t = ab_t
        .simd_max(Vector::<f32>::splat(0.0))
        .simd_min(Vector::<f32>::splat(1.0));
    let mut closest_on_ab = Vector3Wide::default();
    Vector3Wide::scale_to(ab, &ab_t, &mut closest_on_ab);
    closest_on_ab += *a;
    let mut ab_on_a = Vector3Wide::default();
    Vector3Wide::subtract(b_on_a, a_on_a, &mut ab_on_a);
    let mut closest_on_ab_on_a = Vector3Wide::default();
    Vector3Wide::scale_to(&ab_on_a, &ab_t, &mut closest_on_ab_on_a);
    closest_on_ab_on_a += *a_on_a;
    let mut distance_squared_candidate = Vector::<f32>::default();
    Vector3Wide::length_squared_to(&closest_on_ab, &mut distance_squared_candidate);
    select(
        mask,
        distance_squared,
        closest,
        closest_a,
        feature_id,
        &distance_squared_candidate,
        &closest_on_ab,
        &closest_on_ab_on_a,
        &feature_id_candidate,
    );
}

fn try_remove(simplex: &mut Simplex, index: i32, feature_id: &Vector<i32>, active_mask: &Vector<i32>) {
    let should_remove = *active_mask & (*feature_id & Simd::splat(1 << index)).simd_eq(Simd::splat(0)).to_int();
    let last_slot = simplex.count - Simd::splat(1);
    simplex.count = Mask::from_int(should_remove).select(last_slot, simplex.count);
    let should_pull_last_slot = should_remove & Simd::splat(index).simd_lt(last_slot).to_int();
    if should_pull_last_slot.simd_ne(Simd::splat(0)).any() {
        for i in 0..VECTOR_WIDTH {
            if should_pull_last_slot[i] < 0 {
                let last = last_slot[i];
                unsafe {
                    // Copy last slot data into index slot
                    let get_v = |s: &Simplex, idx: i32| -> (f32, f32, f32) {
                        let src = match idx {
                            0 => &s.a,
                            1 => &s.b,
                            2 => &s.c,
                            3 => &s.d,
                            _ => unreachable!(),
                        };
                        (src.x[i], src.y[i], src.z[i])
                    };
                    let get_va = |s: &Simplex, idx: i32| -> (f32, f32, f32) {
                        let src = match idx {
                            0 => &s.a_on_a,
                            1 => &s.b_on_a,
                            2 => &s.c_on_a,
                            3 => &s.d_on_a,
                            _ => unreachable!(),
                        };
                        (src.x[i], src.y[i], src.z[i])
                    };
                    let (vx, vy, vz) = get_v(&*simplex, last);
                    let (vax, vay, vaz) = get_va(&*simplex, last);

                    let set_v = |s: &mut Simplex, idx: i32, x: f32, y: f32, z: f32| {
                        let dst = match idx {
                            0 => &mut s.a,
                            1 => &mut s.b,
                            2 => &mut s.c,
                            3 => &mut s.d,
                            _ => unreachable!(),
                        };
                        *((&mut dst.x) as *mut _ as *mut f32).add(i) = x;
                        *((&mut dst.y) as *mut _ as *mut f32).add(i) = y;
                        *((&mut dst.z) as *mut _ as *mut f32).add(i) = z;
                    };
                    set_v(simplex, index, vx, vy, vz);

                    let set_va = |s: &mut Simplex, idx: i32, x: f32, y: f32, z: f32| {
                        let dst = match idx {
                            0 => &mut s.a_on_a,
                            1 => &mut s.b_on_a,
                            2 => &mut s.c_on_a,
                            3 => &mut s.d_on_a,
                            _ => unreachable!(),
                        };
                        *((&mut dst.x) as *mut _ as *mut f32).add(i) = x;
                        *((&mut dst.y) as *mut _ as *mut f32).add(i) = y;
                        *((&mut dst.z) as *mut _ as *mut f32).add(i) = z;
                    };
                    set_va(simplex, index, vax, vay, vaz);
                }
            }
        }
    }
}

#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn triangle(
    a: &Vector3Wide,
    _b: &Vector3Wide,
    _c: &Vector3Wide,
    a_on_a: &Vector3Wide,
    b_on_a: &Vector3Wide,
    c_on_a: &Vector3Wide,
    ab_a: &Vector<f32>,
    ac_a: &Vector<f32>,
    ab: &Vector3Wide,
    ac: &Vector3Wide,
    abab: &Vector<f32>,
    acac: &Vector<f32>,
    feature_id_candidate: &Vector<i32>,
    mask: &Vector<i32>,
    distance_squared: &mut Vector<f32>,
    closest: &mut Vector3Wide,
    closest_a: &mut Vector3Wide,
    feature_id: &mut Vector<i32>,
) {
    // Triangle distances: project origin onto triangle plane, check barycentric coordinates.
    let mut n = Vector3Wide::default();
    unsafe { Vector3Wide::cross_without_overlap(ab, ac, &mut n); }
    let mut abxac_length_squared = Vector::<f32>::default();
    Vector3Wide::length_squared_to(&n, &mut abxac_length_squared);
    let mut a_n = Vector::<f32>::default();
    Vector3Wide::dot(a, &n, &mut a_n);
    let inverse_n_length_squared = Vector::<f32>::splat(1.0) / abxac_length_squared;
    let t = a_n * inverse_n_length_squared;
    let mut closest_candidate = Vector3Wide::default();
    Vector3Wide::scale_to(&n, &t, &mut closest_candidate);

    let mut abac = Vector::<f32>::default();
    Vector3Wide::dot(ab, ac, &mut abac);
    let c_weight = (*ab_a * abac - *ac_a * *abab) * inverse_n_length_squared;
    let b_weight = (abac * *ac_a - *acac * *ab_a) * inverse_n_length_squared;
    let a_weight = Vector::<f32>::splat(1.0) - b_weight - c_weight;
    let zero = Vector::<f32>::splat(0.0);
    let projection_in_triangle = a_weight.simd_ge(zero).to_int()
        & b_weight.simd_ge(zero).to_int()
        & c_weight.simd_ge(zero).to_int();

    let mut a_on_a_contribution = Vector3Wide::default();
    Vector3Wide::scale_to(a_on_a, &a_weight, &mut a_on_a_contribution);
    let mut b_on_a_contribution = Vector3Wide::default();
    Vector3Wide::scale_to(b_on_a, &b_weight, &mut b_on_a_contribution);
    let mut c_on_a_contribution = Vector3Wide::default();
    Vector3Wide::scale_to(c_on_a, &c_weight, &mut c_on_a_contribution);
    let mut closest_a_candidate = Vector3Wide::default();
    Vector3Wide::add(&a_on_a_contribution, &b_on_a_contribution, &mut closest_a_candidate);
    closest_a_candidate += c_on_a_contribution;
    let mut distance_squared_candidate = Vector::<f32>::default();
    Vector3Wide::length_squared_to(&closest_candidate, &mut distance_squared_candidate);
    let combined_mask = *mask & projection_in_triangle;
    select(
        &combined_mask,
        distance_squared,
        closest,
        closest_a,
        feature_id,
        &distance_squared_candidate,
        &closest_candidate,
        &closest_a_candidate,
        feature_id_candidate,
    );
}

fn find_closest_point(
    outer_loop_terminated_mask: &Vector<i32>,
    simplex: &mut Simplex,
    distance_squared: &mut Vector<f32>,
    closest_a_out: &mut Vector3Wide,
    closest_out: &mut Vector3Wide,
) {
    let active_mask = !*outer_loop_terminated_mask;

    // Vertex A:
    Vector3Wide::length_squared_to(&simplex.a, distance_squared);
    let mut feature_id = Simd::splat(1i32);
    let mut closest = simplex.a;
    let mut closest_a = simplex.a_on_a;

    let active_mask = active_mask & simplex.count.simd_ge(Simd::splat(2)).to_int();
    if active_mask.simd_eq(Simd::splat(0)).all() {
        *closest_out = closest;
        *closest_a_out = closest_a;
        return;
    }

    // Edge AB:
    let mut ab = Vector3Wide::default();
    let mut abab = Vector::<f32>::default();
    let mut ab_a = Vector::<f32>::default();
    edge(
        &simplex.a, &simplex.b, &simplex.a_on_a, &simplex.b_on_a,
        &mut ab, &mut abab, &mut ab_a,
        &Simd::splat(1), &Simd::splat(2),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    let next_active_mask = active_mask & simplex.count.simd_ge(Simd::splat(3)).to_int();
    if next_active_mask.simd_eq(Simd::splat(0)).all() {
        try_remove(simplex, 1, &feature_id, &active_mask);
        try_remove(simplex, 0, &feature_id, &active_mask);
        *closest_out = closest;
        *closest_a_out = closest_a;
        return;
    }
    let active_mask = next_active_mask;

    // Edge AC, BC:
    let mut ac = Vector3Wide::default();
    let mut acac = Vector::<f32>::default();
    let mut ac_a = Vector::<f32>::default();
    edge(
        &simplex.a, &simplex.c, &simplex.a_on_a, &simplex.c_on_a,
        &mut ac, &mut acac, &mut ac_a,
        &Simd::splat(1), &Simd::splat(4),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );
    let mut bc = Vector3Wide::default();
    let mut bcbc = Vector::<f32>::default();
    let mut bc_b = Vector::<f32>::default();
    edge(
        &simplex.b, &simplex.c, &simplex.b_on_a, &simplex.c_on_a,
        &mut bc, &mut bcbc, &mut bc_b,
        &Simd::splat(2), &Simd::splat(4),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    // Triangle ABC:
    triangle(
        &simplex.a, &simplex.b, &simplex.c,
        &simplex.a_on_a, &simplex.b_on_a, &simplex.c_on_a,
        &ab_a, &ac_a, &ab, &ac, &abab, &acac, &Simd::splat(1 | 2 | 4),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    let next_active_mask = active_mask & simplex.count.simd_ge(Simd::splat(4)).to_int();
    if next_active_mask.simd_eq(Simd::splat(0)).all() {
        try_remove(simplex, 2, &feature_id, &active_mask);
        try_remove(simplex, 1, &feature_id, &active_mask);
        try_remove(simplex, 0, &feature_id, &active_mask);
        *closest_out = closest;
        *closest_a_out = closest_a;
        return;
    }
    let active_mask = next_active_mask;

    // Edges AD, BD, CD:
    let mut ad = Vector3Wide::default();
    let mut adad = Vector::<f32>::default();
    let mut ad_a = Vector::<f32>::default();
    edge(
        &simplex.a, &simplex.d, &simplex.a_on_a, &simplex.d_on_a,
        &mut ad, &mut adad, &mut ad_a,
        &Simd::splat(1), &Simd::splat(8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );
    let mut bd = Vector3Wide::default();
    let mut bdbd = Vector::<f32>::default();
    let mut bd_b = Vector::<f32>::default();
    edge(
        &simplex.b, &simplex.d, &simplex.b_on_a, &simplex.d_on_a,
        &mut bd, &mut bdbd, &mut bd_b,
        &Simd::splat(2), &Simd::splat(8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );
    let mut cd = Vector3Wide::default();
    let mut cdcd = Vector::<f32>::default();
    let mut cd_c = Vector::<f32>::default();
    edge(
        &simplex.c, &simplex.d, &simplex.c_on_a, &simplex.d_on_a,
        &mut cd, &mut cdcd, &mut cd_c,
        &Simd::splat(4), &Simd::splat(8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    // Triangle ACD:
    triangle(
        &simplex.a, &simplex.c, &simplex.d,
        &simplex.a_on_a, &simplex.c_on_a, &simplex.d_on_a,
        &ac_a, &ad_a, &ac, &ad, &acac, &adad, &Simd::splat(1 | 4 | 8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    // Triangle ABD:
    triangle(
        &simplex.a, &simplex.b, &simplex.d,
        &simplex.a_on_a, &simplex.b_on_a, &simplex.d_on_a,
        &ab_a, &ad_a, &ab, &ad, &abab, &adad, &Simd::splat(1 | 2 | 8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    // Triangle BCD:
    triangle(
        &simplex.b, &simplex.c, &simplex.d,
        &simplex.b_on_a, &simplex.c_on_a, &simplex.d_on_a,
        &bc_b, &bd_b, &bc, &bd, &bcbc, &bdbd, &Simd::splat(2 | 4 | 8),
        &active_mask, distance_squared, &mut closest, &mut closest_a, &mut feature_id,
    );

    // Tetrahedron:
    // Test the plane of each triangle against the origin.
    let mut nabc = Vector3Wide::default();
    unsafe { Vector3Wide::cross_without_overlap(&bc, &ab, &mut nabc); }
    let mut nabd = Vector3Wide::default();
    unsafe { Vector3Wide::cross_without_overlap(&ad, &bd, &mut nabd); }
    let mut nacd = Vector3Wide::default();
    unsafe { Vector3Wide::cross_without_overlap(&cd, &ac, &mut nacd); }
    let mut nbdc = Vector3Wide::default();
    unsafe { Vector3Wide::cross_without_overlap(&bd, &cd, &mut nbdc); }

    let mut calibration_dot_abc = Vector::<f32>::default();
    Vector3Wide::dot(&ad, &nabc, &mut calibration_dot_abc);
    let flip_required = calibration_dot_abc.simd_ge(Vector::<f32>::splat(0.0)).to_int();

    let mut abc_dot = Vector::<f32>::default();
    Vector3Wide::dot(&nabc, &simplex.a, &mut abc_dot);
    let mut abd_dot = Vector::<f32>::default();
    Vector3Wide::dot(&nabd, &simplex.a, &mut abd_dot);
    let mut acd_dot = Vector::<f32>::default();
    Vector3Wide::dot(&nacd, &simplex.a, &mut acd_dot);
    let mut bdc_dot = Vector::<f32>::default();
    Vector3Wide::dot(&nbdc, &simplex.b, &mut bdc_dot);

    let zero = Vector::<f32>::splat(0.0);
    let abc_inside = abc_dot.simd_ge(zero).to_int() ^ flip_required;
    let abd_inside = abd_dot.simd_ge(zero).to_int() ^ flip_required;
    let acd_inside = acd_dot.simd_ge(zero).to_int() ^ flip_required;
    let bdc_inside = bdc_dot.simd_ge(zero).to_int() ^ flip_required;
    let tetrahedron_contains = (abc_inside & abd_inside) & (acd_inside & bdc_inside);
    let use_tetrahedral_result = tetrahedron_contains & active_mask;
    // Note that we don't guarantee correct closest points in the intersecting case.
    select(
        &use_tetrahedral_result,
        distance_squared,
        &mut closest,
        &mut closest_a,
        &mut feature_id,
        &Vector::<f32>::splat(0.0),
        &Vector3Wide::default(),
        &Vector3Wide::default(),
        &Simd::splat(1 | 2 | 4 | 8),
    );

    try_remove(simplex, 3, &feature_id, &active_mask);
    try_remove(simplex, 2, &feature_id, &active_mask);
    try_remove(simplex, 1, &feature_id, &active_mask);
    try_remove(simplex, 0, &feature_id, &active_mask);

    *closest_out = closest;
    *closest_a_out = closest_a;
}

impl<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
    IPairDistanceTester<TShapeWideA, TShapeWideB>
    for GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
where
    TShapeA: IConvexShape,
    TShapeWideA: IShapeWide<TShapeA>,
    TSupportFinderA: ISupportFinder<TShapeWideA> + Default,
    TShapeB: IConvexShape,
    TShapeWideB: IShapeWide<TShapeB>,
    TSupportFinderB: ISupportFinder<TShapeWideB> + Default,
{
    fn test(
        &self,
        a: &TShapeWideA,
        b: &TShapeWideB,
        offset_b: &Vector3Wide,
        orientation_a: &QuaternionWide,
        orientation_b: &QuaternionWide,
        inactive_lanes: &Vector<i32>,
        intersected: &mut Vector<i32>,
        distance: &mut Vector<f32>,
        closest_a: &mut Vector3Wide,
        normal: &mut Vector3Wide,
    ) {
        let mut r_a = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_a, &mut r_a);
        let mut r_b = Matrix3x3Wide::default();
        Matrix3x3Wide::create_from_quaternion(orientation_b, &mut r_b);
        let support_finder_a = TSupportFinderA::default();
        let support_finder_b = TSupportFinderB::default();
        let mut simplex = Simplex {
            a: Vector3Wide::default(),
            b: Vector3Wide::default(),
            c: Vector3Wide::default(),
            d: Vector3Wide::default(),
            a_on_a: Vector3Wide::default(),
            b_on_a: Vector3Wide::default(),
            c_on_a: Vector3Wide::default(),
            d_on_a: Vector3Wide::default(),
            count: Simd::splat(1),
        };
        sample_minkowski_difference(
            a, &r_a, &support_finder_a, b, &r_b, &support_finder_b, offset_b, offset_b,
            inactive_lanes, &mut simplex.a_on_a, &mut simplex.a,
        );

        let mut terminated_mask = *inactive_lanes;
        *intersected = Simd::splat(0);
        let _iteration_count = 0i32;
        // Note that we use hardcoded defaults if this is default constructed.
        let epsilon = Vector::<f32>::splat(if self.termination_epsilon > 0.0 {
            self.termination_epsilon
        } else {
            Self::TERMINATION_EPSILON_DEFAULT
        });
        let mut containment_epsilon = Vector::<f32>::splat(if self.containment_epsilon > 0.0 {
            self.containment_epsilon
        } else {
            Self::CONTAINMENT_EPSILON_DEFAULT
        });
        // Use the JIT's ability to optimize away branches with generic type knowledge to set appropriate containment epsilons for shapes that have actual margins.
        if TSupportFinderA::has_margin() && TSupportFinderB::has_margin() {
            let mut margin_a = Vector::<f32>::default();
            TSupportFinderA::get_margin(a, &mut margin_a);
            let mut margin_b = Vector::<f32>::default();
            TSupportFinderB::get_margin(b, &mut margin_b);
            containment_epsilon = containment_epsilon.simd_max(margin_a + margin_b);
        } else if TSupportFinderA::has_margin() {
            let mut margin_a = Vector::<f32>::default();
            TSupportFinderA::get_margin(a, &mut margin_a);
            containment_epsilon = containment_epsilon.simd_max(margin_a);
        } else if TSupportFinderB::has_margin() {
            let mut margin_b = Vector::<f32>::default();
            TSupportFinderB::get_margin(b, &mut margin_b);
            containment_epsilon = containment_epsilon.simd_max(margin_b);
        }
        let containment_epsilon_squared = containment_epsilon * containment_epsilon;
        let mut distance_squared = Vector::<f32>::splat(f32::MAX);
        let all_ones = Simd::splat(-1i32);
        loop {
            let mut new_distance_squared = Vector::<f32>::default();
            let mut simplex_closest_a = Vector3Wide::default();
            let mut simplex_closest = Vector3Wide::default();
            find_closest_point(
                &terminated_mask,
                &mut simplex,
                &mut new_distance_squared,
                &mut simplex_closest_a,
                &mut simplex_closest,
            );

            let simplex_contains_origin = new_distance_squared.simd_le(containment_epsilon_squared).to_int();
            *intersected = *intersected | (simplex_contains_origin & !terminated_mask);
            terminated_mask = terminated_mask | simplex_contains_origin;
            if terminated_mask.simd_eq(all_ones).all() {
                break;
            }

            let no_progress_made = new_distance_squared.simd_ge(distance_squared).to_int();
            distance_squared = distance_squared.simd_min(new_distance_squared);
            let about_to_terminate = no_progress_made & !terminated_mask;
            *closest_a = Vector3Wide::conditional_select(&about_to_terminate, &simplex_closest_a, closest_a);
            *normal = Vector3Wide::conditional_select(&about_to_terminate, &simplex_closest, normal);
            terminated_mask = terminated_mask | about_to_terminate;
            if terminated_mask.simd_eq(all_ones).all() {
                break;
            }

            let mut sample_direction = Vector3Wide::default();
            Vector3Wide::negate(&simplex_closest, &mut sample_direction);
            let mut v_a = Vector3Wide::default();
            let mut v = Vector3Wide::default();
            sample_minkowski_difference(
                a, &r_a, &support_finder_a, b, &r_b, &support_finder_b, offset_b,
                &sample_direction, &terminated_mask, &mut v_a, &mut v,
            );

            // If the newly sampled is no better than the simplex-identified closest point, the lane is done.
            let mut progress_offset = Vector3Wide::default();
            Vector3Wide::subtract(&v, &simplex_closest, &mut progress_offset);
            let mut progress = Vector::<f32>::default();
            Vector3Wide::dot(&progress_offset, &sample_direction, &mut progress);
            // The epsilon against which the progress is measured is scaled by the largest simplex vertex squared distance from the origin.
            let mut a_squared = Vector::<f32>::default();
            Vector3Wide::length_squared_to(&simplex.a, &mut a_squared);
            let mut b_squared = Vector::<f32>::default();
            Vector3Wide::length_squared_to(&simplex.b, &mut b_squared);
            let mut c_squared = Vector::<f32>::default();
            Vector3Wide::length_squared_to(&simplex.c, &mut c_squared);
            let mut d_squared = Vector::<f32>::default();
            Vector3Wide::length_squared_to(&simplex.d, &mut d_squared);
            let mut max = Mask::from_int(simplex.count.simd_gt(Simd::splat(1)).to_int() & b_squared.simd_gt(a_squared).to_int())
                .select(b_squared, a_squared);
            max = Mask::from_int(simplex.count.simd_gt(Simd::splat(2)).to_int() & c_squared.simd_gt(max).to_int())
                .select(c_squared, max);
            max = Mask::from_int(simplex.count.simd_gt(Simd::splat(3)).to_int() & d_squared.simd_gt(max).to_int())
                .select(d_squared, max);
            let no_progress_made = progress.simd_le(max * epsilon).to_int();
            let about_to_terminate = no_progress_made & !terminated_mask;
            *closest_a = Vector3Wide::conditional_select(&about_to_terminate, &simplex_closest_a, closest_a);
            *normal = Vector3Wide::conditional_select(&about_to_terminate, &simplex_closest, normal);
            terminated_mask = terminated_mask | about_to_terminate;
            if terminated_mask.simd_eq(all_ones).all() {
                break;
            }

            append(&terminated_mask, &mut simplex, &v_a, &v);
        }
        // The normal gathered during the loop was the unnormalized offset.
        *distance = distance_squared.sqrt();
        let inverse_distance = Vector::<f32>::splat(1.0) / *distance;
        // The containment epsilon acts as a margin around shapes.
        *distance = *distance - containment_epsilon;
        *normal *= inverse_distance;

        // For shapes with substantial margins, compensate for the margin.
        let mut closest_a_margin_offset = Vector3Wide::default();
        Vector3Wide::scale_to(normal, &(-containment_epsilon), &mut closest_a_margin_offset);
        *closest_a += closest_a_margin_offset;
    }
}
