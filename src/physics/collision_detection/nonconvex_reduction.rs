// Translated from BepuPhysics/CollisionDetection/NonconvexReduction.cs

use crate::physics::collision_detection::collision_batcher_continuations::{
    ICollisionTestContinuation, PairContinuation,
};
use crate::physics::collision_detection::contact_manifold::{
    Contact, ConvexContact, ConvexContactManifold, NonconvexContactManifold,
};
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use glam::Vec3;

/// Child of a nonconvex reduction containing a convex manifold and child metadata.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct NonconvexReductionChild {
    pub manifold: ConvexContactManifold,
    /// Offset from the origin of the first shape's parent to the child in world space.
    pub offset_a: Vec3,
    pub child_index_a: i32,
    /// Offset from the origin of the second shape's parent to the child in world space.
    pub offset_b: Vec3,
    pub child_index_b: i32,
}

impl Default for NonconvexReductionChild {
    fn default() -> Self {
        Self {
            manifold: ConvexContactManifold::default(),
            offset_a: Vec3::ZERO,
            child_index_a: 0,
            offset_b: Vec3::ZERO,
            child_index_b: 0,
        }
    }
}

/// Accumulates child manifolds and reduces them into a single nonconvex contact manifold.
#[repr(C)]
pub struct NonconvexReduction {
    pub child_count: i32,
    pub completed_child_count: i32,
    pub children: Buffer<NonconvexReductionChild>,
}

impl ICollisionTestContinuation for NonconvexReduction {
    fn create(&mut self, child_manifold_count: i32, pool: &mut BufferPool) {
        self.child_count = child_manifold_count;
        self.completed_child_count = 0;
        self.children = pool.take(child_manifold_count);
    }
}

impl Default for NonconvexReduction {
    fn default() -> Self {
        Self {
            child_count: 0,
            completed_child_count: 0,
            children: Buffer::default(),
        }
    }
}

impl Copy for NonconvexReduction {}
impl Clone for NonconvexReduction {
    fn clone(&self) -> Self {
        *self
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RemainingCandidate {
    child_index: i32,
    contact_index: i32,
    distinctiveness: f32,
}

impl NonconvexReduction {
    /// Adds a contact from a child manifold to the target nonconvex manifold.
    #[inline(always)]
    pub unsafe fn add_contact(
        child: &NonconvexReductionChild,
        contact_index_in_child: i32,
        target_manifold: *mut NonconvexContactManifold,
    ) {
        let contact = &*(&child.manifold.contact0 as *const ConvexContact)
            .offset(contact_index_in_child as isize);
        let target = NonconvexContactManifold::allocate(&mut *target_manifold);
        target.offset = contact.offset;
        target.depth = contact.depth;
        target.normal = child.manifold.normal;
        // Mix the convex-generated feature id with the child indices.
        target.feature_id =
            contact.feature_id ^ ((child.child_index_a << 8) ^ (child.child_index_b << 16));
    }

    #[inline(always)]
    unsafe fn use_contact(
        remaining_candidates: &mut QuickList<RemainingCandidate>,
        index: i32,
        children: &Buffer<NonconvexReductionChild>,
        target_manifold: *mut NonconvexContactManifold,
    ) {
        let candidate = *remaining_candidates.get(index);
        Self::add_contact(&*children.get(candidate.child_index), candidate.contact_index, target_manifold);
        remaining_candidates.fast_remove_at(index);
    }

    #[inline(always)]
    fn compute_distinctiveness(
        candidate: &ConvexContact,
        contact_normal: Vec3,
        reduced_contact: &Contact,
        distance_squared_interpolation_min: f32,
        inverse_distance_squared_interpolation_span: f32,
        depth_scale: f32,
    ) -> f32 {
        // Normal distinctiveness: acos(0.99999) is about 0.25 degrees.
        let normal_dot = contact_normal.dot(reduced_contact.normal);
        const NORMAL_INTERPOLATION_SPAN: f32 = -0.99999;
        let normal_distinctiveness = (normal_dot - 0.99999) * (1.0 / NORMAL_INTERPOLATION_SPAN);

        // Offset distinctiveness
        let offset_distinctiveness = ((reduced_contact.offset - candidate.offset).length_squared()
            - distance_squared_interpolation_min)
            * inverse_distance_squared_interpolation_span;

        // Three-way max: normal, offset, normal*offset
        let combined_distinctiveness = normal_distinctiveness * offset_distinctiveness;
        let mut distinctiveness = if offset_distinctiveness > normal_distinctiveness {
            offset_distinctiveness
        } else {
            normal_distinctiveness
        };
        if distinctiveness < combined_distinctiveness {
            distinctiveness = combined_distinctiveness;
        }

        if distinctiveness <= 0.0 {
            0.0
        } else {
            // Deeper contacts are more important to the manifold.
            let mut depth_multiplier = 1.0 + candidate.depth * depth_scale;
            if depth_multiplier < 0.01 {
                depth_multiplier = 0.01;
            }
            distinctiveness * depth_multiplier
        }
    }

    unsafe fn choose_most_distinct(
        &self,
        manifold: *mut NonconvexContactManifold,
        pool: &mut BufferPool,
    ) {
        // Find the approximate extents of the manifold for calibration.
        let extent_axis = Vec3::new(0.280454652, 0.55873544499, 0.7804869574);
        let mut minimum_extent = f32::MAX;
        let mut minimum_extent_position = Vec3::ZERO;
        for i in 0..self.child_count {
            let child = &*self.children.get(i);
            for j in 0..child.manifold.count {
                let position = &(*(&child.manifold.contact0 as *const ConvexContact).add(j as usize)).offset;
                let extent = position.dot(extent_axis);
                if extent < minimum_extent {
                    minimum_extent = extent;
                    minimum_extent_position = *position;
                }
            }
        }
        let mut maximum_distance_squared: f32 = 0.0;
        for i in 0..self.child_count {
            let child = &*self.children.get(i);
            for j in 0..child.manifold.count {
                let distance_squared = ((*(&child.manifold.contact0 as *const ConvexContact).add(j as usize)).offset
                    - minimum_extent_position)
                    .length_squared();
                if distance_squared > maximum_distance_squared {
                    maximum_distance_squared = distance_squared;
                }
            }
        }
        let maximum_distance = maximum_distance_squared.sqrt();
        let mut initial_best_score = -f32::MAX;
        let mut initial_best_score_index: i32 = 0;
        let maximum_allocated_candidate_count = self.child_count * 4;
        const HEAP_ALLOCATION_THRESHOLD: i32 = 8192;
        let mut remaining_contacts_buffer: Buffer<RemainingCandidate>;
        let heap_allocated = maximum_allocated_candidate_count > HEAP_ALLOCATION_THRESHOLD;
        if heap_allocated {
            remaining_contacts_buffer = pool.take(maximum_allocated_candidate_count);
        } else {
            // Use a Vec for stack-like allocation in Rust (C# uses stackalloc)
            let mut vec = Vec::<RemainingCandidate>::with_capacity(maximum_allocated_candidate_count as usize);
            vec.set_len(maximum_allocated_candidate_count as usize);
            remaining_contacts_buffer = Buffer::new(
                vec.as_mut_ptr(),
                maximum_allocated_candidate_count,
                -1, // No pool id for stack allocation
            );
            std::mem::forget(vec);
        }
        let mut remaining_contacts = QuickList::new(remaining_contacts_buffer);
        let extremity_scale = maximum_distance * 5e-3;
        for child_index in 0..self.child_count {
            let child = &*self.children.get(child_index);
            for contact_index in 0..child.manifold.count {
                let contact = &*(&child.manifold.contact0 as *const ConvexContact).add(contact_index as usize);
                let candidate_score: f32;
                if contact.depth >= 0.0 {
                    let extent = contact.offset.dot(extent_axis) - minimum_extent;
                    candidate_score = contact.depth + extent * extremity_scale;
                } else {
                    candidate_score = contact.depth;
                }
                if candidate_score > initial_best_score {
                    initial_best_score = candidate_score;
                    initial_best_score_index = remaining_contacts.count;
                }
                let indices = remaining_contacts.allocate_unsafely();
                indices.child_index = child_index;
                indices.contact_index = contact_index;
                indices.distinctiveness = f32::MAX;
            }
        }

        debug_assert!(remaining_contacts.count > 0, "This function should only be called when there are populated manifolds.");

        Self::use_contact(
            &mut remaining_contacts,
            initial_best_score_index,
            &self.children,
            manifold,
        );

        let distance_squared_interpolation_min = maximum_distance_squared * (1e-3 * 1e-3);
        let inverse_distance_squared_interpolation_span = 1.0 / maximum_distance_squared;
        let depth_scale = 400.0 / maximum_distance;

        // Incrementally search for contacts which expand the manifold as much as possible.
        let reduced_contacts = &mut (*manifold).contact0 as *mut Contact;
        while remaining_contacts.count > 0
            && (*manifold).count < NonconvexContactManifold::MAXIMUM_CONTACT_COUNT
        {
            let last_contact_index = (*manifold).count - 1;
            let reduced_contact = &*reduced_contacts.add(last_contact_index as usize);

            let mut best_score: f32 = -1.0;
            let mut best_score_index: i32 = -1;
            let mut i = 0i32;
            while i < remaining_contacts.count {
                let remaining_contact = remaining_contacts.get_mut(i);
                let child_manifold = &(*self.children.get(remaining_contact.child_index)).manifold;
                let child_contact = &*(&child_manifold.contact0 as *const ConvexContact)
                    .add(remaining_contact.contact_index as usize);
                let distinctiveness = Self::compute_distinctiveness(
                    child_contact,
                    child_manifold.normal,
                    reduced_contact,
                    distance_squared_interpolation_min,
                    inverse_distance_squared_interpolation_span,
                    depth_scale,
                );
                if distinctiveness <= 0.0 {
                    // Fully redundant.
                    remaining_contacts.fast_remove_at(i);
                    // Don't increment i — the swapped element is now at index i.
                } else {
                    if distinctiveness < remaining_contact.distinctiveness {
                        remaining_contact.distinctiveness = distinctiveness;
                    }
                    if remaining_contact.distinctiveness > best_score {
                        best_score = remaining_contact.distinctiveness;
                        best_score_index = i;
                    }
                    i += 1;
                }
            }
            if best_score_index >= 0 {
                Self::use_contact(
                    &mut remaining_contacts,
                    best_score_index,
                    &self.children,
                    manifold,
                );
            } else {
                break;
            }
        }

        if heap_allocated {
            remaining_contacts.dispose(pool);
        } else {
            // For stack-like allocation, reconstruct the Vec to drop it.
            // Vec was forgotten above — reconstruct to drop.
            let _ = Vec::from_raw_parts(
                remaining_contacts.span.as_mut_ptr(),
                0,
                maximum_allocated_candidate_count as usize,
            );
        }
    }

    /// Flushes accumulated child manifolds into a final nonconvex (or convex) manifold.
    /// Called when all child manifolds have been completed.
    pub unsafe fn flush(&mut self, pair_id: i32, pool: &mut BufferPool) -> FlushResult {
        debug_assert!(self.child_count > 0);
        debug_assert!(self.completed_child_count == self.child_count);

        let mut populated_child_manifolds = 0i32;
        let mut sample_populated_child_index = 0i32;
        let mut total_contact_count = 0i32;
        for i in 0..self.child_count {
            let child = &mut *self.children.get_mut(i);
            let child_manifold_count = child.manifold.count;
            if child_manifold_count > 0 {
                total_contact_count += child_manifold_count;
                populated_child_manifolds += 1;
                sample_populated_child_index = i;
                for j in 0..child.manifold.count {
                    // Push all contacts into the space of the parent object.
                    let contact = &mut *(&mut child.manifold.contact0 as *mut ConvexContact).add(j as usize);
                    contact.offset = contact.offset + child.offset_a;
                }
            }
        }
        let sample_child = &*self.children.get(sample_populated_child_index);

        if populated_child_manifolds > 1 {
            // Multiple contributing child manifolds — produce a nonconvex manifold.
            let mut reduced_manifold = std::mem::MaybeUninit::<NonconvexContactManifold>::uninit();
            (*reduced_manifold.as_mut_ptr()).count = 0;

            if total_contact_count <= NonconvexContactManifold::MAXIMUM_CONTACT_COUNT {
                // No reduction required; we can fit every contact.
                for i in 0..self.child_count {
                    let child = &*self.children.get(i);
                    for j in 0..child.manifold.count {
                        Self::add_contact(child, j, reduced_manifold.as_mut_ptr());
                    }
                }
            } else {
                self.choose_most_distinct(reduced_manifold.as_mut_ptr(), pool);
            }

            // The manifold offsetB is the offset from shapeA origin to shapeB origin.
            (*reduced_manifold.as_mut_ptr()).offset_b =
                sample_child.manifold.offset_b - sample_child.offset_b + sample_child.offset_a;

            pool.return_buffer(&mut self.children);

            FlushResult::Nonconvex(reduced_manifold.assume_init())
        } else {
            // Single or zero populated child manifolds — report convex directly.
            let mut result_manifold = sample_child.manifold;
            result_manifold.offset_b =
                sample_child.manifold.offset_b - sample_child.offset_b + sample_child.offset_a;

            pool.return_buffer(&mut self.children);

            FlushResult::Convex(result_manifold)
        }
    }

    /// Records a completed child manifold.
    #[inline(always)]
    pub unsafe fn on_child_completed(
        &mut self,
        report: &PairContinuation,
        manifold: &ConvexContactManifold,
    ) {
        (*self.children.get_mut(report.child_index())).manifold = *manifold;
        self.completed_child_count += 1;
    }

    /// Records an untested child as having zero contacts.
    #[inline(always)]
    pub unsafe fn on_untested_child_completed(&mut self, report: &PairContinuation) {
        (*self.children.get_mut(report.child_index())).manifold.count = 0;
        self.completed_child_count += 1;
    }

    /// Checks if the reduction is complete and can be flushed.
    pub fn is_complete(&self) -> bool {
        self.completed_child_count == self.child_count
    }

    /// Tries to flush if all children are completed.
    pub unsafe fn try_flush(&mut self, pair_id: i32, pool: &mut BufferPool) -> Option<FlushResult> {
        if self.completed_child_count == self.child_count {
            Some(self.flush(pair_id, pool))
        } else {
            None
        }
    }

    /// Disposes resources used by the reduction.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.children);
    }
}

/// Result of flushing a nonconvex reduction.
pub enum FlushResult {
    /// Multiple child manifolds produced a nonconvex manifold.
    Nonconvex(NonconvexContactManifold),
    /// A single child manifold was reported directly as convex.
    Convex(ConvexContactManifold),
}
