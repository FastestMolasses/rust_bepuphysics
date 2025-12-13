// Translated from BepuPhysics/Trees/Tree_RefinementScheduling.cs

use super::node::NodeChild;
use super::tree::Tree;
use crate::utilities::bounding_box::BoundingBox;
use crate::utilities::collections::quicklist::QuickList;
use crate::utilities::memory::buffer_pool::BufferPool;

impl Tree {
    fn refit_and_measure(&self, child: &mut NodeChild) -> f32 {
        unsafe {
            let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                .add(child.index as usize);

            debug_assert!(self.leaf_count >= 2);

            let premetric = Self::compute_bounds_metric_vecs(&child.min, &child.max);
            let mut child_change = 0.0f32;

            if node.a.index >= 0 {
                child_change += self.refit_and_measure(&mut node.a);
            }
            if node.b.index >= 0 {
                child_change += self.refit_and_measure(&mut node.b);
            }

            BoundingBox::create_merged(
                node.a.min,
                node.a.max,
                node.b.min,
                node.b.max,
                &mut child.min,
                &mut child.max,
            );

            let postmetric = Self::compute_bounds_metric_vecs(&child.min, &child.max);
            postmetric - premetric + child_change
        }
    }

    fn refit_and_mark(
        &self,
        child: &mut NodeChild,
        leaf_count_threshold: i32,
        refinement_candidates: &mut QuickList<i32>,
        pool: &mut BufferPool,
    ) -> f32 {
        debug_assert!(leaf_count_threshold > 1);

        unsafe {
            let node = &mut *(self.nodes.as_ptr() as *mut super::node::Node)
                .add(child.index as usize);
            debug_assert!(
                self.metanodes.get(child.index).cost_or_flag.refine_flag == 0
            );
            let mut child_change = 0.0f32;

            let premetric = Self::compute_bounds_metric_vecs(&child.min, &child.max);

            // The wavefront of internal nodes is defined by the transition from
            // more than threshold to less than threshold.
            if node.a.index >= 0 {
                if node.a.leaf_count <= leaf_count_threshold {
                    refinement_candidates.add(node.a.index, pool);
                    child_change += self.refit_and_measure(&mut node.a);
                } else {
                    child_change += self.refit_and_mark(
                        &mut node.a,
                        leaf_count_threshold,
                        refinement_candidates,
                        pool,
                    );
                }
            }

            if node.b.index >= 0 {
                if node.b.leaf_count <= leaf_count_threshold {
                    refinement_candidates.add(node.b.index, pool);
                    child_change += self.refit_and_measure(&mut node.b);
                } else {
                    child_change += self.refit_and_mark(
                        &mut node.b,
                        leaf_count_threshold,
                        refinement_candidates,
                        pool,
                    );
                }
            }

            BoundingBox::create_merged(
                node.a.min,
                node.a.max,
                node.b.min,
                node.b.max,
                &mut child.min,
                &mut child.max,
            );

            let postmetric = Self::compute_bounds_metric_vecs(&child.min, &child.max);
            postmetric - premetric + child_change
        }
    }

    fn refit_and_mark_root(
        &self,
        leaf_count_threshold: i32,
        refinement_candidates: &mut QuickList<i32>,
        pool: &mut BufferPool,
    ) -> f32 {
        debug_assert!(
            self.leaf_count > 2,
            "There's no reason to refit a tree with 2 or less elements. Nothing would happen."
        );

        unsafe {
            let children = &mut (*(self.nodes.as_ptr() as *mut super::node::Node)).a
                as *mut NodeChild;

            let mut child_change = 0.0f32;
            let mut merged = BoundingBox {
                min: glam::Vec3::splat(f32::MAX),
                max: glam::Vec3::splat(f32::MIN),
            };
            for i in 0..2 {
                let child = &mut *children.add(i);
                if child.index >= 0 {
                    if child.leaf_count <= leaf_count_threshold {
                        refinement_candidates.add(child.index, pool);
                        child_change += self.refit_and_measure(child);
                    } else {
                        child_change += self.refit_and_mark(
                            child,
                            leaf_count_threshold,
                            refinement_candidates,
                            pool,
                        );
                    }
                }
                BoundingBox::create_merged(
                    child.min,
                    child.max,
                    merged.min,
                    merged.max,
                    &mut merged.min,
                    &mut merged.max,
                );
            }

            let postmetric = Self::compute_bounds_metric_bb(&merged);

            // The root's own change is not included since refines cannot change
            // the volume of the root.
            if postmetric >= 1e-10 {
                child_change / postmetric
            } else {
                0.0
            }
        }
    }

    #[cfg(debug_assertions)]
    fn validate_refine_flags(&self, index: i32) {
        unsafe {
            let metanode = self.metanodes.get(index);
            if metanode.cost_or_flag.refine_flag != 0 {
                eprintln!("Bad refine flag");
            }

            let children = &self.nodes.get(index).a as *const NodeChild;
            for i in 0..2 {
                let child = &*children.add(i);
                if child.index >= 0 {
                    self.validate_refine_flags(child.index);
                }
            }
        }
    }

    fn get_refit_and_mark_tuning(
        &self,
    ) -> (
        i32, /* maximum_subtrees */
        i32, /* estimated_refinement_candidate_count */
        i32, /* refinement_leaf_count_threshold */
    ) {
        let maximum_subtrees = ((self.leaf_count as f64).sqrt() * 3.0) as i32;
        let estimated_refinement_candidate_count =
            (self.leaf_count * 2) / maximum_subtrees;
        let refinement_leaf_count_threshold =
            self.leaf_count.min(maximum_subtrees);
        (
            maximum_subtrees,
            estimated_refinement_candidate_count,
            refinement_leaf_count_threshold,
        )
    }

    fn get_refine_tuning(
        &self,
        frame_index: i32,
        refinement_candidates_count: i32,
        refine_aggressiveness_scale: f32,
        cost_change: f32,
    ) -> (
        i32, /* target_refinement_count */
        i32, /* refinement_period */
        i32, /* refinement_offset */
    ) {
        if cost_change.is_nan() || cost_change.is_infinite() {
            panic!(
                "The change in tree cost is an invalid value, strongly implying the tree bounds \
                 have been corrupted by infinites or NaNs."
            );
        }
        let refine_aggressiveness =
            (cost_change * refine_aggressiveness_scale).max(0.0);
        let refine_portion = (refine_aggressiveness * 0.25).min(1.0);

        let target_refinement_scale = (self.node_count as f32).min(
            (refinement_candidates_count as f32 * refine_aggressiveness_scale * 0.03)
                .ceil()
                .max(2.0)
                + refinement_candidates_count as f32 * refine_portion,
        );

        let refinement_period = (1i32).max(
            (refinement_candidates_count as f32 / target_refinement_scale) as i32,
        );
        let refinement_offset = ((frame_index as i64 * 236887691i64 + 104395303i64)
            % (1i64.max(refinement_candidates_count as i64))) as i32;
        let target_refinement_count =
            refinement_candidates_count.min(target_refinement_scale as i32);

        (
            target_refinement_count,
            refinement_period,
            refinement_offset,
        )
    }

    pub fn refit_and_refine(
        &mut self,
        pool: &mut BufferPool,
        frame_index: i32,
        refine_aggressiveness_scale: f32,
    ) {
        // Don't proceed if the tree has no refitting or refinement required.
        if self.leaf_count <= 2 {
            return;
        }
        let (maximum_subtrees, estimated_refinement_candidate_count, leaf_count_threshold) =
            self.get_refit_and_mark_tuning();
        let mut refinement_candidates =
            QuickList::<i32>::with_capacity(estimated_refinement_candidate_count, pool);

        // Collect the refinement candidates.
        let cost_change = self.refit_and_mark_root(
            leaf_count_threshold,
            &mut refinement_candidates,
            pool,
        );

        let (target_refinement_count, period, offset) = self.get_refine_tuning(
            frame_index,
            refinement_candidates.count,
            refine_aggressiveness_scale,
            cost_change,
        );

        let mut refinement_targets =
            QuickList::<i32>::with_capacity(target_refinement_count, pool);

        let mut index = offset;
        unsafe {
            for _ in 0..target_refinement_count - 1 {
                index += period;
                if index >= refinement_candidates.count {
                    index -= refinement_candidates.count;
                }
                refinement_targets.add_unsafely(refinement_candidates[index]);
                debug_assert!(
                    self.metanodes
                        .get(refinement_candidates[index])
                        .cost_or_flag
                        .refine_flag
                        == 0,
                    "Refinement target search shouldn't run into the same node twice!"
                );
                self.metanodes
                    .get_mut(refinement_candidates[index])
                    .cost_or_flag
                    .refine_flag = 1;
            }
        }
        refinement_candidates.dispose(pool);

        unsafe {
            if self.metanodes.get(0).cost_or_flag.refine_flag == 0 {
                refinement_targets.add_unsafely(0);
                self.metanodes.get_mut(0).cost_or_flag.refine_flag = 1;
            }
        }

        // Refine all marked targets.
        let mut subtree_references =
            QuickList::<i32>::with_capacity(maximum_subtrees, pool);
        let mut treelet_internal_nodes =
            QuickList::<i32>::with_capacity(maximum_subtrees, pool);

        let (mut buffer, mut resources) =
            self.create_binned_resources(pool, maximum_subtrees);

        for i in 0..refinement_targets.count {
            subtree_references.count = 0;
            treelet_internal_nodes.count = 0;
            self.binned_refine(
                refinement_targets[i],
                &mut subtree_references,
                maximum_subtrees,
                &mut treelet_internal_nodes,
                &mut resources,
                pool,
            );
            unsafe {
                self.metanodes
                    .get_mut(refinement_targets[i])
                    .cost_or_flag
                    .refine_flag = 0;
            }
        }

        pool.return_buffer(&mut buffer);
        subtree_references.dispose(pool);
        treelet_internal_nodes.dispose(pool);
        refinement_targets.dispose(pool);
    }
}
