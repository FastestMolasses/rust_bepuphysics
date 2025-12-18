// Translated from BepuPhysics/Constraints/Contact/ContactNonconvexTypes.cs
// Nonconvex contact constraint description structs — scalar form for scatter/gather to/from SIMD type batches.

use crate::physics::collision_detection::narrow_phase_callbacks::PairMaterialProperties;
use crate::physics::constraints::contact::contact_constraint_description::{
    NonconvexConstraintContactData, NonconvexOneBodyManifoldConstraintProperties,
    NonconvexTwoBodyManifoldConstraintProperties,
};
use crate::physics::constraints::contact::contact_nonconvex_common::NonconvexContactPrestepData;
use crate::physics::constraints::contact::contact_nonconvex_types::*;
use crate::physics::constraints::contact::contact_convex_common::MaterialPropertiesWide;
use crate::physics::constraints::spring_settings::SpringSettingsWide;
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::memory::buffer::Buffer;
use crate::utilities::vector::Vector;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;

// ============================================================================
// Helper: copy nonconvex contact data from description into SIMD prestep
// ============================================================================

/// Copies per-contact nonconvex data from scalar description contacts into SIMD prestep contacts.
#[inline(always)]
unsafe fn copy_contact_data_to_prestep(
    contact_count: i32,
    source_contacts: *const NonconvexConstraintContactData,
    target_contacts: *mut NonconvexContactPrestepData,
) {
    for i in 0..contact_count as isize {
        let source = &*source_contacts.offset(i);
        let target = &mut *target_contacts.offset(i);
        Vector3Wide::write_first(source.offset_a, &mut target.offset);
        *GatherScatter::get_first_mut(&mut target.depth) = source.penetration_depth;
        Vector3Wide::write_first(source.normal, &mut target.normal);
    }
}

/// Copies per-contact nonconvex data from SIMD prestep contacts into scalar description contacts.
#[inline(always)]
unsafe fn copy_contact_data_from_prestep(
    contact_count: i32,
    source_contacts: *const NonconvexContactPrestepData,
    target_contacts: *mut NonconvexConstraintContactData,
) {
    for i in 0..contact_count as isize {
        let source = &*source_contacts.offset(i);
        let target = &mut *target_contacts.offset(i);
        Vector3Wide::read_first(&source.offset, &mut target.offset_a);
        target.penetration_depth = *GatherScatter::get_first(&source.depth);
        Vector3Wide::read_first(&source.normal, &mut target.normal);
    }
}

// ============================================================================
// Macros for generating nonconvex contact descriptions
// ============================================================================

/// Generate a nonconvex two-body contact constraint description struct.
macro_rules! impl_nonconvex_two_body_description {
    ($name:ident, $prestep:ty, $type_processor:ty, $count:expr, [$($contact_field:ident),+]) => {
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            pub common: NonconvexTwoBodyManifoldConstraintProperties,
            $(pub $contact_field: NonconvexConstraintContactData,)+
        }

        impl $name {
            pub const CONSTRAINT_TYPE_ID: i32 = <$type_processor>::BATCH_TYPE_ID;
            pub const CONTACT_COUNT: i32 = $count;

            /// Scatters this scalar description into a SIMD-width prestep data bundle.
            pub fn apply_description(
                &self,
                prestep_bundle: &mut $prestep,
                _bundle_index: usize,
                inner_index: usize,
            ) {
                let target = unsafe {
                    GatherScatter::get_offset_instance_mut(prestep_bundle, inner_index)
                };
                // Copy common two-body properties
                Vector3Wide::write_first(self.common.offset_b, &mut target.offset_b);
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.friction_coefficient) =
                        self.common.friction_coefficient;
                }
                SpringSettingsWide::write_first(
                    &self.common.spring_settings,
                    &mut target.material_properties.spring_settings,
                );
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.maximum_recovery_velocity) =
                        self.common.maximum_recovery_velocity;
                }
                // Copy per-contact data
                unsafe {
                    copy_contact_data_to_prestep(
                        $count,
                        &self.contact0 as *const NonconvexConstraintContactData,
                        &mut target.contact0 as *mut NonconvexContactPrestepData,
                    );
                }
            }

            /// Reads a scalar description from a SIMD-width prestep data bundle.
            pub fn build_description(
                prestep_bundle: &$prestep,
                _bundle_index: usize,
                inner_index: usize,
                description: &mut $name,
            ) {
                let source = unsafe {
                    GatherScatter::get_offset_instance(prestep_bundle, inner_index)
                };
                Vector3Wide::read_first(&source.offset_b, &mut description.common.offset_b);
                description.common.friction_coefficient =
                    unsafe { *GatherScatter::get_first(&source.material_properties.friction_coefficient) };
                SpringSettingsWide::read_first(
                    &source.material_properties.spring_settings,
                    &mut description.common.spring_settings,
                );
                description.common.maximum_recovery_velocity =
                    unsafe { *GatherScatter::get_first(&source.material_properties.maximum_recovery_velocity) };
                unsafe {
                    copy_contact_data_from_prestep(
                        $count,
                        &source.contact0 as *const NonconvexContactPrestepData,
                        &mut description.contact0 as *mut NonconvexConstraintContactData,
                    );
                }
            }

            /// Copies shared manifold-wide properties (offsetB, material) into this description.
            #[inline(always)]
            pub fn copy_manifold_wide_properties(&mut self, offset_b: &Vec3, material: &PairMaterialProperties) {
                self.common.offset_b = *offset_b;
                self.common.friction_coefficient = material.friction_coefficient;
                self.common.spring_settings = material.spring_settings;
                self.common.maximum_recovery_velocity = material.maximum_recovery_velocity;
            }

            #[inline(always)]
            pub fn get_first_contact(&self) -> &NonconvexConstraintContactData {
                &self.contact0
            }
            #[inline(always)]
            pub fn get_first_contact_mut(&mut self) -> &mut NonconvexConstraintContactData {
                &mut self.contact0
            }
        }
    };
}

/// Generate a nonconvex one-body contact constraint description struct.
macro_rules! impl_nonconvex_one_body_description {
    ($name:ident, $prestep:ty, $type_processor:ty, $count:expr, [$($contact_field:ident),+]) => {
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            pub common: NonconvexOneBodyManifoldConstraintProperties,
            $(pub $contact_field: NonconvexConstraintContactData,)+
        }

        impl $name {
            pub const CONSTRAINT_TYPE_ID: i32 = <$type_processor>::BATCH_TYPE_ID;
            pub const CONTACT_COUNT: i32 = $count;

            /// Scatters this scalar description into a SIMD-width prestep data bundle.
            pub fn apply_description(
                &self,
                prestep_bundle: &mut $prestep,
                _bundle_index: usize,
                inner_index: usize,
            ) {
                let target = unsafe {
                    GatherScatter::get_offset_instance_mut(prestep_bundle, inner_index)
                };
                // Copy common one-body properties (no offsetB)
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.friction_coefficient) =
                        self.common.friction_coefficient;
                }
                SpringSettingsWide::write_first(
                    &self.common.spring_settings,
                    &mut target.material_properties.spring_settings,
                );
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.maximum_recovery_velocity) =
                        self.common.maximum_recovery_velocity;
                }
                // Copy per-contact data
                unsafe {
                    copy_contact_data_to_prestep(
                        $count,
                        &self.contact0 as *const NonconvexConstraintContactData,
                        &mut target.contact0 as *mut NonconvexContactPrestepData,
                    );
                }
            }

            /// Reads a scalar description from a SIMD-width prestep data bundle.
            pub fn build_description(
                prestep_bundle: &$prestep,
                _bundle_index: usize,
                inner_index: usize,
                description: &mut $name,
            ) {
                let source = unsafe {
                    GatherScatter::get_offset_instance(prestep_bundle, inner_index)
                };
                description.common.friction_coefficient =
                    unsafe { *GatherScatter::get_first(&source.material_properties.friction_coefficient) };
                SpringSettingsWide::read_first(
                    &source.material_properties.spring_settings,
                    &mut description.common.spring_settings,
                );
                description.common.maximum_recovery_velocity =
                    unsafe { *GatherScatter::get_first(&source.material_properties.maximum_recovery_velocity) };
                unsafe {
                    copy_contact_data_from_prestep(
                        $count,
                        &source.contact0 as *const NonconvexContactPrestepData,
                        &mut description.contact0 as *mut NonconvexConstraintContactData,
                    );
                }
            }

            /// Copies shared manifold-wide properties (material only, no offsetB for one body) into this description.
            #[inline(always)]
            pub fn copy_manifold_wide_properties(&mut self, material: &PairMaterialProperties) {
                self.common.friction_coefficient = material.friction_coefficient;
                self.common.spring_settings = material.spring_settings;
                self.common.maximum_recovery_velocity = material.maximum_recovery_velocity;
            }

            #[inline(always)]
            pub fn get_first_contact(&self) -> &NonconvexConstraintContactData {
                &self.contact0
            }
            #[inline(always)]
            pub fn get_first_contact_mut(&mut self) -> &mut NonconvexConstraintContactData {
                &mut self.contact0
            }
        }
    };
}

// ============================================================================
// Concrete description types — Nonconvex Two-Body
// ============================================================================

impl_nonconvex_two_body_description!(
    Contact2Nonconvex, Contact2NonconvexPrestepData, Contact2NonconvexTypeProcessor, 2,
    [contact0, contact1]
);

impl_nonconvex_two_body_description!(
    Contact3Nonconvex, Contact3NonconvexPrestepData, Contact3NonconvexTypeProcessor, 3,
    [contact0, contact1, contact2]
);

impl_nonconvex_two_body_description!(
    Contact4Nonconvex, Contact4NonconvexPrestepData, Contact4NonconvexTypeProcessor, 4,
    [contact0, contact1, contact2, contact3]
);

// ============================================================================
// Concrete description types — Nonconvex One-Body
// ============================================================================

impl_nonconvex_one_body_description!(
    Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData, Contact2NonconvexOneBodyTypeProcessor, 2,
    [contact0, contact1]
);

impl_nonconvex_one_body_description!(
    Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData, Contact3NonconvexOneBodyTypeProcessor, 3,
    [contact0, contact1, contact2]
);

impl_nonconvex_one_body_description!(
    Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyTypeProcessor, 4,
    [contact0, contact1, contact2, contact3]
);
