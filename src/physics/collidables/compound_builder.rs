use glam::{Quat, Vec3};

use crate::utilities::memory::buffer::Buffer;
use crate::utilities::memory::buffer_pool::BufferPool;
use crate::utilities::symmetric3x3::Symmetric3x3;

use super::compound::CompoundChild;
use super::shapes::Shapes;
use super::typed_index::TypedIndex;
use crate::physics::body_properties::{BodyInertia, RigidPose};
use crate::physics::pose_integration::PoseIntegration;

/// A child entry in the compound builder.
pub struct CompoundBuilderChild {
    /// Local pose of the child in the compound's local space.
    pub local_pose: RigidPose,
    /// Shape index in the shapes collection.
    pub shape_index: TypedIndex,
    /// Weight of the child (mass for dynamics, pose weight for kinematics).
    pub weight: f32,
    /// Inverse inertia tensor of the child in its local space.
    pub local_inverse_inertia: Symmetric3x3,
}

/// Reusable convenience type for incrementally building compound shapes.
pub struct CompoundBuilder {
    pub pool: *mut BufferPool,
    pub shapes: *mut Shapes,
    pub children: Vec<CompoundBuilderChild>,
}

impl CompoundBuilder {
    /// Creates a compound builder.
    pub fn new(
        pool: &mut BufferPool,
        shapes: &mut Shapes,
        initial_builder_capacity: usize,
    ) -> Self {
        Self {
            pool: pool as *mut BufferPool,
            shapes: shapes as *mut Shapes,
            children: Vec::with_capacity(initial_builder_capacity),
        }
    }

    /// Adds a new shape to the accumulator with explicit inverse inertia and weight.
    pub fn add(
        &mut self,
        shape: TypedIndex,
        local_pose: &RigidPose,
        local_inverse_inertia: &Symmetric3x3,
        weight: f32,
    ) {
        debug_assert!(
            Symmetric3x3::determinant(local_inverse_inertia) > 0.0,
            "Child inertia tensors should be invertible."
        );
        self.children.push(CompoundBuilderChild {
            local_pose: *local_pose,
            shape_index: shape,
            weight,
            local_inverse_inertia: *local_inverse_inertia,
        });
    }

    /// Adds a new shape with body inertia (inverse mass is used as the inverse weight).
    pub fn add_with_inertia(
        &mut self,
        shape: TypedIndex,
        local_pose: &RigidPose,
        child_inertia: &BodyInertia,
    ) {
        debug_assert!(
            child_inertia.inverse_mass > 0.0,
            "Child masses should be finite."
        );
        self.add(
            shape,
            local_pose,
            &child_inertia.inverse_inertia_tensor,
            1.0 / child_inertia.inverse_mass,
        );
    }

    /// Adds a new shape assuming infinite inertia (kinematic child).
    pub fn add_for_kinematic(&mut self, shape: TypedIndex, local_pose: &RigidPose, weight: f32) {
        self.children.push(CompoundBuilderChild {
            local_pose: *local_pose,
            shape_index: shape,
            weight,
            local_inverse_inertia: Symmetric3x3::default(),
        });
    }

    /// Gets the contribution to an inertia tensor of a point mass at the given offset from the center of mass.
    #[inline(always)]
    pub fn get_offset_inertia_contribution(offset: Vec3, mass: f32) -> Symmetric3x3 {
        let inner_product = offset.dot(offset);
        Symmetric3x3 {
            xx: mass * (inner_product - offset.x * offset.x),
            yx: -mass * (offset.y * offset.x),
            yy: mass * (inner_product - offset.y * offset.y),
            zx: -mass * (offset.z * offset.x),
            zy: -mass * (offset.z * offset.y),
            zz: mass * (inner_product - offset.z * offset.z),
        }
    }

    /// Computes the uninverted inertia contribution of a child.
    #[inline(always)]
    pub fn compute_inertia_for_child(
        position: Vec3,
        orientation: Quat,
        inverse_local_inertia: &Symmetric3x3,
        mass: f32,
    ) -> Symmetric3x3 {
        let offset_contribution = Self::get_offset_inertia_contribution(position, mass);
        let mut rotated_inverse_inertia = Symmetric3x3::default();
        PoseIntegration::rotate_inverse_inertia(
            inverse_local_inertia,
            orientation,
            &mut rotated_inverse_inertia,
        );
        let mut inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&rotated_inverse_inertia, &mut inertia);
        inertia = offset_contribution + inertia;
        inertia
    }

    /// Builds a buffer of compound children from the accumulated set for a dynamic compound.
    /// Computes a center of mass and recenters child shapes relative to it.
    pub fn build_dynamic_compound_recentered(&self) -> (Buffer<CompoundChild>, BodyInertia, Vec3) {
        let pool = unsafe { &mut *self.pool };
        let mut center = Vec3::ZERO;
        let mut total_weight = 0.0f32;
        for child in &self.children {
            center += child.local_pose.position * child.weight;
            total_weight += child.weight;
        }
        debug_assert!(
            total_weight > 0.0,
            "The compound must have nonzero weight for recentering."
        );
        let inverse_mass = 1.0 / total_weight;
        center *= inverse_mass;

        let mut children: Buffer<CompoundChild> = pool.take(self.children.len() as i32);

        let mut summed_inertia = Symmetric3x3::default();
        for (i, source) in self.children.iter().enumerate() {
            let target = &mut children[i];
            target.local_position = source.local_pose.position - center;
            target.local_orientation = source.local_pose.orientation;
            target.shape_index = source.shape_index;
            summed_inertia = summed_inertia
                + Self::compute_inertia_for_child(
                    target.local_position,
                    target.local_orientation,
                    &source.local_inverse_inertia,
                    source.weight,
                );
        }
        let mut inverse_inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&summed_inertia, &mut inverse_inertia);
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = inverse_mass;
        inertia.inverse_inertia_tensor = inverse_inertia;
        (children, inertia, center)
    }

    /// Builds a buffer of compound children for a dynamic compound without recentering.
    pub fn build_dynamic_compound(&self) -> (Buffer<CompoundChild>, BodyInertia) {
        let pool = unsafe { &mut *self.pool };
        let mut total_weight = 0.0f32;
        for child in &self.children {
            total_weight += child.weight;
        }
        debug_assert!(total_weight > 0.0, "The compound must have nonzero weight.");
        let inverse_mass = 1.0 / total_weight;

        let mut children: Buffer<CompoundChild> = pool.take(self.children.len() as i32);

        let mut summed_inertia = Symmetric3x3::default();
        for (i, source) in self.children.iter().enumerate() {
            let target = &mut children[i];
            target.local_position = source.local_pose.position;
            target.local_orientation = source.local_pose.orientation;
            target.shape_index = source.shape_index;
            summed_inertia = summed_inertia
                + Self::compute_inertia_for_child(
                    source.local_pose.position,
                    source.local_pose.orientation,
                    &source.local_inverse_inertia,
                    source.weight,
                );
        }
        let mut inverse_inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&summed_inertia, &mut inverse_inertia);
        let mut inertia = BodyInertia::default();
        inertia.inverse_mass = inverse_mass;
        inertia.inverse_inertia_tensor = inverse_inertia;
        (children, inertia)
    }

    /// Builds a buffer of compound children for a kinematic compound with recentering.
    pub fn build_kinematic_compound_recentered(&self) -> (Buffer<CompoundChild>, Vec3) {
        let pool = unsafe { &mut *self.pool };
        let mut center = Vec3::ZERO;
        let mut total_weight = 0.0f32;
        for child in &self.children {
            center += child.local_pose.position * child.weight;
            total_weight += child.weight;
        }
        debug_assert!(
            total_weight > 0.0,
            "The compound must have nonzero weight for recentering."
        );
        let inverse_weight = 1.0 / total_weight;
        center *= inverse_weight;

        let mut children: Buffer<CompoundChild> = pool.take(self.children.len() as i32);

        for (i, source) in self.children.iter().enumerate() {
            let target = &mut children[i];
            target.local_position = source.local_pose.position - center;
            target.local_orientation = source.local_pose.orientation;
            target.shape_index = source.shape_index;
        }
        (children, center)
    }

    /// Builds a buffer of compound children for a kinematic compound without recentering.
    pub fn build_kinematic_compound(&self) -> Buffer<CompoundChild> {
        let pool = unsafe { &mut *self.pool };
        let mut children: Buffer<CompoundChild> = pool.take(self.children.len() as i32);

        for (i, source) in self.children.iter().enumerate() {
            let target = &mut children[i];
            target.local_position = source.local_pose.position;
            target.local_orientation = source.local_pose.orientation;
            target.shape_index = source.shape_index;
        }
        children
    }

    /// Computes the center of mass of a compound from its children.
    pub fn compute_center_of_mass(
        children: &Buffer<CompoundChild>,
        child_masses: &[f32],
    ) -> (Vec3, f32) {
        let mut sum = Vec3::ZERO;
        let mut mass_sum = 0.0f32;
        for i in 0..children.len() as usize {
            sum += child_masses[i] * children[i].local_position;
            mass_sum += child_masses[i];
        }
        let inverse_mass = 1.0 / mass_sum;
        (sum * inverse_mass, inverse_mass)
    }

    /// Computes inverse inertia for a set of compound children (no recentering).
    pub fn compute_inverse_inertia(
        children: &Buffer<CompoundChild>,
        inverse_local_inertias: &[Symmetric3x3],
        child_masses: &[f32],
    ) -> BodyInertia {
        let mut summed_inertia = Symmetric3x3::default();
        let mut mass_sum = 0.0f32;
        for i in 0..children.len() as usize {
            let child = &children[i];
            summed_inertia = summed_inertia
                + Self::compute_inertia_for_child(
                    child.local_position,
                    child.local_orientation,
                    &inverse_local_inertias[i],
                    child_masses[i],
                );
            mass_sum += child_masses[i];
        }
        let mut inverse_inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&summed_inertia, &mut inverse_inertia);
        let mut result = BodyInertia::default();
        result.inverse_mass = 1.0 / mass_sum;
        result.inverse_inertia_tensor = inverse_inertia;
        result
    }

    /// Computes inverse inertia for a set of compound children with recentering.
    /// Child positions will have the center of mass subtracted.
    pub fn compute_inverse_inertia_recentered(
        children: &mut Buffer<CompoundChild>,
        inverse_local_inertias: &[Symmetric3x3],
        child_masses: &[f32],
    ) -> (BodyInertia, Vec3) {
        let (center_of_mass, inverse_mass) = Self::compute_center_of_mass(children, child_masses);
        let mut summed_inertia = Symmetric3x3::default();
        for i in 0..children.len() as usize {
            children[i].local_position -= center_of_mass;
            let child = &children[i];
            summed_inertia = summed_inertia
                + Self::compute_inertia_for_child(
                    child.local_position,
                    child.local_orientation,
                    &inverse_local_inertias[i],
                    child_masses[i],
                );
        }
        let mut inverse_inertia = Symmetric3x3::default();
        Symmetric3x3::invert(&summed_inertia, &mut inverse_inertia);
        let mut result = BodyInertia::default();
        result.inverse_mass = inverse_mass;
        result.inverse_inertia_tensor = inverse_inertia;
        (result, center_of_mass)
    }

    /// Computes the inertia of a compound using the shapes collection to look up
    /// per-child inertia tensors. Does not recenter the children.
    pub fn compute_inertia(
        children: &Buffer<CompoundChild>,
        child_masses: &[f32],
        shapes: &Shapes,
    ) -> BodyInertia {
        let len = children.len() as usize;
        let mut local_inverse_inertias = vec![Symmetric3x3::default(); len];
        for i in 0..len {
            let child = &children[i];
            if let Some(batch) = shapes.get_batch(child.shape_index.type_id() as usize) {
                if let Some(inertia) =
                    batch.try_compute_inertia(child.shape_index.index() as usize, child_masses[i])
                {
                    local_inverse_inertias[i] = inertia.inverse_inertia_tensor;
                }
            }
        }
        Self::compute_inverse_inertia(children, &local_inverse_inertias, child_masses)
    }

    /// Computes the inertia of a compound using the shapes collection to look up
    /// per-child inertia tensors. Recenters children around the calculated center of mass.
    pub fn compute_inertia_recentered(
        children: &mut Buffer<CompoundChild>,
        child_masses: &[f32],
        shapes: &Shapes,
    ) -> (BodyInertia, Vec3) {
        let len = children.len() as usize;
        let mut local_inverse_inertias = vec![Symmetric3x3::default(); len];
        for i in 0..len {
            let child = &children[i];
            if let Some(batch) = shapes.get_batch(child.shape_index.type_id() as usize) {
                if let Some(inertia) =
                    batch.try_compute_inertia(child.shape_index.index() as usize, child_masses[i])
                {
                    local_inverse_inertias[i] = inertia.inverse_inertia_tensor;
                }
            }
        }
        Self::compute_inverse_inertia_recentered(children, &local_inverse_inertias, child_masses)
    }

    /// Empties out the accumulated children.
    pub fn reset(&mut self) {
        self.children.clear();
    }
}

impl Drop for CompoundBuilder {
    fn drop(&mut self) {
        self.children.clear();
    }
}
