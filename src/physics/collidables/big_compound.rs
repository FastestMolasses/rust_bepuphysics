use glam::{Quat, Vec3};

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::for_each_ref::IBreakableForEach;

use super::compound::{Compound, CompoundChild, IOverlapCollector};
use super::shape::{IShape, IShapeRayHitHandler, IDisposableShape, ICompoundShape};
use super::shapes::Shapes;
use super::mesh::{ShapeTreeOverlapEnumerator, ShapeTreeSweepLeafTester};
use super::compound_builder::CompoundBuilder;

use crate::physics::body_properties::{BodyInertia, RigidPose};
use crate::physics::collision_detection::ray_batchers::RayData;
use crate::physics::trees::tree_ray_cast::IRayLeafTester;
use crate::utilities::matrix3x3::Matrix3x3;

use crate::physics::collision_detection::collision_tasks::convex_compound_overlap_finder::IBoundsQueryableCompound;
use crate::physics::collision_detection::collision_tasks::convex_compound_task_overlaps::ConvexCompoundTaskOverlaps;
use crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::{
    ICollisionTaskOverlaps, ICollisionTaskSubpairOverlaps, OverlapQueryForPair,
};
use crate::physics::trees::node::NodeChild;
use crate::physics::trees::tree::Tree;
use crate::physics::trees::tree_sweep::ISweepLeafTester;
use crate::utilities::bounding_box::BoundingBox;

/// Compound shape containing a bunch of shapes accessible through a tree acceleration structure.
/// Useful for compounds with lots of children.
#[repr(C)]
pub struct BigCompound {
    /// Acceleration structure for the compound children.
    pub tree: Tree,
    /// Buffer of children within this compound.
    pub children: Buffer<CompoundChild>,
}

impl BigCompound {
    /// Type id of big compound shapes.
    pub const ID: i32 = 7;

    /// Gets the number of children in the compound.
    #[inline(always)]
    pub fn child_count(&self) -> i32 {
        self.children.len()
    }

    /// Gets a reference to a child by index.
    #[inline(always)]
    pub fn get_child(&self, compound_child_index: i32) -> &CompoundChild {
        &self.children[compound_child_index as usize]
    }

    /// Gets a mutable reference to a child by index.
    #[inline(always)]
    pub fn get_child_mut(&mut self, compound_child_index: i32) -> &mut CompoundChild {
        &mut self.children[compound_child_index as usize]
    }

    /// Creates a BigCompound shape instance, but leaves the Tree in an unbuilt state.
    /// The Tree must be built before the compound can be used.
    pub fn create_without_tree_build(
        children: Buffer<CompoundChild>,
        pool: &mut BufferPool,
    ) -> Self {
        debug_assert!(
            children.len() > 0,
            "Compounds must have a nonzero number of children."
        );
        let child_len = children.len();
        let mut tree = Tree::new(pool, child_len);
        tree.node_count = 1i32.max(child_len - 1);
        tree.leaf_count = child_len;
        Self { tree, children }
    }

    /// Fills a buffer of subtrees according to a buffer of compound children.
    pub fn fill_subtrees_for_children(
        children: &[CompoundChild],
        shapes: &Shapes,
        subtrees: &mut [NodeChild],
    ) {
        assert_eq!(
            subtrees.len(),
            children.len(),
            "Children and subtrees span lengths should match."
        );
        for i in 0..children.len() {
            let subtree = &mut subtrees[i];
            Compound::compute_child_bounds(
                &children[i],
                Quat::IDENTITY,
                shapes,
                &mut subtree.min,
                &mut subtree.max,
            );
            subtree.leaf_count = 1;
            subtree.index = Tree::encode(i as i32);
        }
    }

    /// Creates a compound shape instance and builds an acceleration structure using a sweep builder.
    pub unsafe fn create_with_sweep_build(
        children: Buffer<CompoundChild>,
        shapes: &Shapes,
        pool: &mut BufferPool,
    ) -> Self {
        let child_len = children.len();
        let mut compound = Self::create_without_tree_build(children, pool);
        let mut subtrees: Buffer<NodeChild> = pool.take_at_least(child_len);
        Self::fill_subtrees_for_children(
            std::slice::from_raw_parts(compound.children.as_ptr(), child_len as usize),
            shapes,
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), child_len as usize),
        );
        // NodeChild intentionally shares the same byte size as BoundingBox.
        let leaf_bounds = subtrees.cast::<BoundingBox>();
        compound.tree.sweep_build(pool, &leaf_bounds);
        pool.return_buffer(&mut subtrees);
        compound
    }

    /// Creates a compound shape with an acceleration structure built using the binned builder.
    #[inline(always)]
    pub unsafe fn new(
        children: Buffer<CompoundChild>,
        shapes: &Shapes,
        pool: &mut BufferPool,
    ) -> Self {
        let child_len = children.len();
        let mut compound = Self::create_without_tree_build(children, pool);
        let mut subtrees: Buffer<NodeChild> = pool.take_at_least(child_len);
        Self::fill_subtrees_for_children(
            std::slice::from_raw_parts(compound.children.as_ptr(), child_len as usize),
            shapes,
            std::slice::from_raw_parts_mut(subtrees.as_mut_ptr(), child_len as usize),
        );
        compound
            .tree
            .binned_build(subtrees, Some(pool as *mut BufferPool), None, None, 0, -1, -1, 16, 64, 1.0 / 16.0, 64, false);
        pool.return_buffer(&mut subtrees);
        compound
    }

    /// Computes the bounding box for the big compound given an orientation.
    pub fn compute_bounds(
        &self,
        orientation: Quat,
        shape_batches: &Shapes,
        min: &mut Vec3,
        max: &mut Vec3,
    ) {
        Compound::compute_child_bounds(&self.children[0], orientation, shape_batches, min, max);
        for i in 1..self.children.len() as usize {
            let mut child_min = Vec3::ZERO;
            let mut child_max = Vec3::ZERO;
            Compound::compute_child_bounds(
                &self.children[i],
                orientation,
                shape_batches,
                &mut child_min,
                &mut child_max,
            );
            *min = min.min(child_min);
            *max = max.max(child_max);
        }
    }

    // NOTE: add_child_bounds_to_batcher — requires full SIMD BoundingBoxBatcher wide path

    /// Tests a ray against the big compound shape using BVH acceleration.
    pub unsafe fn ray_test<TRayHitHandler: IShapeRayHitHandler>(
        &self,
        pose: &RigidPose,
        ray: &RayData,
        maximum_t: &mut f32,
        shapes: &Shapes,
        hit_handler: &mut TRayHitHandler,
    ) {
        let mut orientation = Matrix3x3::default();
        Matrix3x3::create_from_quaternion(&pose.orientation, &mut orientation);

        let offset = ray.origin - pose.position;
        let mut local_origin = Vec3::ZERO;
        Matrix3x3::transform_transpose(&offset, &orientation, &mut local_origin);
        let mut local_direction = Vec3::ZERO;
        Matrix3x3::transform_transpose(&ray.direction, &orientation, &mut local_direction);

        struct LeafTester<'a, THandler: IShapeRayHitHandler> {
            children: *const CompoundChild,
            shapes: &'a Shapes,
            handler: &'a mut THandler,
            orientation: Matrix3x3,
            original_ray: RayData,
        }

        impl<THandler: IShapeRayHitHandler> IRayLeafTester for LeafTester<'_, THandler> {
            unsafe fn test_leaf(
                &mut self,
                leaf_index: i32,
                ray_data: *mut crate::physics::trees::ray_batcher::RayData,
                maximum_t: *mut f32,
            ) {
                if self.handler.allow_test(leaf_index) {
                    let child = &*self.children.add(leaf_index as usize);

                    // Use a simple non-generic hit handler to capture the result,
                    // avoiding infinite generic recursion in AOT.
                    let mut hit_t: f32 = -1.0;
                    let mut hit_normal = Vec3::ZERO;

                    struct ChildShapeTester {
                        t: *mut f32,
                        normal: *mut Vec3,
                    }

                    impl IShapeRayHitHandler for ChildShapeTester {
                        fn allow_test(&self, _child_index: i32) -> bool {
                            true
                        }
                        fn on_ray_hit(
                            &mut self,
                            _ray: &RayData,
                            _maximum_t: &mut f32,
                            t: f32,
                            normal: Vec3,
                            _child_index: i32,
                        ) {
                            unsafe {
                                *self.t = t;
                                *self.normal = normal;
                            }
                        }
                    }

                    let mut tester = ChildShapeTester {
                        t: &mut hit_t,
                        normal: &mut hit_normal,
                    };

                    if let Some(batch) = self.shapes.get_batch(child.shape_index.type_id() as usize) {
                        let child_pose = RigidPose {
                            position: child.local_position,
                            orientation: child.local_orientation,
                        };
                        // trees::RayData and collision_detection::RayData have identical field layout.
                        let cd_ray = &*(ray_data as *const RayData);
                        batch.ray_test(
                            child.shape_index.index() as usize,
                            &child_pose,
                            cd_ray,
                            &mut *maximum_t,
                            &mut tester,
                        );
                    }

                    if hit_t >= 0.0 {
                        // Rotate the normal back to world space.
                        let mut rotated_normal = Vec3::ZERO;
                        Matrix3x3::transform(&hit_normal, &self.orientation, &mut rotated_normal);
                        self.handler.on_ray_hit(
                            &self.original_ray,
                            &mut *maximum_t,
                            hit_t,
                            rotated_normal,
                            leaf_index,
                        );
                    }
                }
            }
        }

        let mut leaf_tester = LeafTester {
            children: self.children.as_ptr() as *const CompoundChild,
            shapes,
            handler: hit_handler,
            orientation,
            original_ray: *ray,
        };

        self.tree.ray_cast(
            local_origin,
            local_direction,
            maximum_t,
            &mut leaf_tester,
            ray.id,
        );
    }

    /// Adds a child to the compound.
    pub fn add(
        &mut self,
        child: CompoundChild,
        pool: &mut BufferPool,
        shapes: &Shapes,
    ) {
        let old_len = self.children.len();
        pool.resize_to_at_least(&mut self.children, old_len + 1, old_len);
        self.children[old_len as usize] = child;

        let mut bounds = BoundingBox::default();
        unsafe {
            shapes.update_bounds(child.as_pose(), &child.shape_index, &mut bounds);
        }
        let child_index = self.tree.add(bounds, pool);
        debug_assert!(
            child_index == old_len,
            "Adding to a tree acts like appending to the list."
        );
    }

    /// Removes a child from the compound by index.
    /// The last child is pulled to fill the gap left by the removed child.
    pub fn remove_at(&mut self, child_index: i32, pool: &mut BufferPool) {
        let moved_child_index = self.tree.remove_at(child_index);
        if moved_child_index >= 0 {
            self.children[child_index as usize] = self.children[moved_child_index as usize];
            debug_assert!(
                moved_child_index == self.children.len() - 1,
                "The child moved by the tree is expected to be the last one."
            );
        }
        // Shrinking the buffer takes care of 'removing' the now-empty last slot.
        let new_len = self.children.len() - 1;
        pool.resize_to_at_least(&mut self.children, new_len, new_len);
    }

    /// Finds overlapping children for pairs of bounding boxes against the compound tree.
    pub unsafe fn find_local_overlaps<
        TOverlaps: ICollisionTaskOverlaps<TSubpairOverlaps>,
        TSubpairOverlaps: ICollisionTaskSubpairOverlaps,
    >(
        &self,
        pairs: &Buffer<OverlapQueryForPair>,
        pool: &mut BufferPool,
        overlaps: &mut TOverlaps,
    ) {
        let mut enumerator = ShapeTreeOverlapEnumerator::<TSubpairOverlaps> {
            pool,
            overlaps: std::ptr::null_mut(),
        };
        for i in 0..pairs.len() {
            let pair = &pairs[i as usize];
            enumerator.overlaps = overlaps.get_overlaps_for_pair(i) as *mut TSubpairOverlaps;
            let compound = &*(pair.container as *const BigCompound);
            compound
                .tree
                .get_overlaps_minmax(pair.min, pair.max, &mut enumerator);
        }
    }

    /// Finds overlapping children for a swept bounding box against the compound tree.
    pub unsafe fn find_local_overlaps_sweep<TOverlaps: ICollisionTaskSubpairOverlaps>(
        &self,
        min: Vec3,
        max: Vec3,
        sweep: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        overlaps: *mut TOverlaps,
    ) {
        let mut enumerator = ShapeTreeSweepLeafTester::<TOverlaps> { pool, overlaps };
        self.tree.sweep(min, max, sweep, maximum_t, &mut enumerator);
    }

    /// Computes the inertia of this compound using the shapes collection.
    /// Does not recenter children.
    pub fn compute_inertia(
        &self,
        child_masses: &[f32],
        shapes: &Shapes,
    ) -> BodyInertia {
        CompoundBuilder::compute_inertia(&self.children, child_masses, shapes)
    }

    /// Computes the inertia of this compound using the shapes collection.
    /// Recenters children around the calculated center of mass and shifts tree nodes.
    pub fn compute_inertia_recentered(
        &mut self,
        child_masses: &[f32],
        shapes: &Shapes,
    ) -> (BodyInertia, glam::Vec3) {
        let (inertia, center_of_mass) =
            CompoundBuilder::compute_inertia_recentered(&mut self.children, child_masses, shapes);
        // Shift the tree nodes to match the recentered children.
        for i in 0..self.tree.node_count as usize {
            let node = &mut self.tree.nodes[i];
            node.a.min -= center_of_mass;
            node.a.max -= center_of_mass;
            node.b.min -= center_of_mass;
            node.b.max -= center_of_mass;
        }
        (inertia, center_of_mass)
    }

    /// Returns the compound's resources to a buffer pool.
    pub fn dispose(&mut self, pool: &mut BufferPool) {
        pool.return_buffer(&mut self.children);
        self.tree.dispose(pool);
    }
}

impl IShape for BigCompound {
    #[inline(always)]
    fn type_id() -> i32 {
        Self::ID
    }
}

impl IDisposableShape for BigCompound {
    fn dispose(&mut self, pool: &mut BufferPool) {
        self.dispose(pool);
    }
}

impl ICompoundShape for BigCompound {
    fn child_count(&self) -> i32 {
        self.child_count()
    }

    fn get_child(&self, child_index: i32) -> &CompoundChild {
        self.get_child(child_index)
    }

    fn compute_bounds(&self, _orientation: Quat, _min: &mut Vec3, _max: &mut Vec3) {
        panic!("BigCompound::compute_bounds requires Shapes; use the inherent method with shape_batches parameter.");
    }

    fn find_local_overlaps<TOverlaps: IOverlapCollector>(
        &self,
        local_min: &Vec3,
        local_max: &Vec3,
        overlaps: &mut TOverlaps,
    ) {
        // Adapter: bridge IOverlapCollector to IBreakableForEach<i32> for tree query.
        struct OverlapAdapter<'a, T: IOverlapCollector>(&'a mut T);
        impl<T: IOverlapCollector> IBreakableForEach<i32> for OverlapAdapter<'_, T> {
            fn loop_body(&mut self, i: i32) -> bool {
                self.0.add(i);
                true // always continue
            }
        }
        let mut adapter = OverlapAdapter(overlaps);
        self.tree.get_overlaps_minmax(*local_min, *local_max, &mut adapter);
    }
}

impl IBoundsQueryableCompound for BigCompound {
    fn child_count(&self) -> i32 {
        self.child_count()
    }

    fn find_local_overlaps<TOverlaps, TSubpairOverlaps>(
        &self,
        query_bounds: &Buffer<OverlapQueryForPair>,
        pool: &mut BufferPool,
        _shapes: &Shapes,
        overlaps: &mut TOverlaps,
    ) where
        TSubpairOverlaps: ICollisionTaskSubpairOverlaps,
        TOverlaps: ICollisionTaskOverlaps<TSubpairOverlaps>,
    {
        // For each pair, query the tree to find overlapping leaf children.
        for i in 0..query_bounds.len() {
            let pair = &query_bounds[i as usize];
            let overlaps_for_pair = overlaps.get_overlaps_for_pair(i);
            let compound = unsafe { &*(pair.container as *const BigCompound) };

            // Use tree overlap query with an adapter that stores results
            struct BigCompoundOverlapAdapter<'a, T: ICollisionTaskSubpairOverlaps> {
                overlaps: &'a mut T,
                pool: &'a mut BufferPool,
            }
            impl<T: ICollisionTaskSubpairOverlaps> IBreakableForEach<i32> for BigCompoundOverlapAdapter<'_, T> {
                fn loop_body(&mut self, leaf_index: i32) -> bool {
                    *self.overlaps.allocate(self.pool) = leaf_index;
                    true
                }
            }
            let mut adapter = BigCompoundOverlapAdapter { overlaps: overlaps_for_pair, pool };
            compound.tree.get_overlaps_minmax(pair.min, pair.max, &mut adapter);
        }
    }

    unsafe fn find_local_overlaps_sweep(
        &self,
        min: Vec3,
        max: Vec3,
        sweep: Vec3,
        maximum_t: f32,
        pool: &mut BufferPool,
        _shapes: &Shapes,
        overlaps: *mut u8,
    ) {
        // BigCompound uses tree sweep. The overlaps pointer is type-erased.
        let mut enumerator = ShapeTreeSweepLeafTester::<crate::physics::collision_detection::collision_tasks::compound_pair_overlaps::ChildOverlapsCollection> {
            pool,
            overlaps: overlaps as *mut _,
        };
        self.tree.sweep(min, max, sweep, maximum_t, &mut enumerator);
    }
}
