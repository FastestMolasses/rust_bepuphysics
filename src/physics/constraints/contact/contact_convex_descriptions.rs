// Translated from BepuPhysics/Constraints/Contact/ContactConvexTypes.cs
// Contact constraint description structs — scalar form for scatter/gather to/from SIMD type batches.

use crate::physics::collision_detection::narrow_phase_callbacks::PairMaterialProperties;
use crate::physics::constraints::contact::contact_constraint_description::ConstraintContactData;
use crate::physics::constraints::contact::contact_convex_common::ConvexContactWide;
use crate::physics::constraints::contact::contact_convex_types::*;
use crate::physics::constraints::spring_settings::{SpringSettings, SpringSettingsWide};
use crate::utilities::gather_scatter::GatherScatter;
use crate::utilities::vector3_wide::Vector3Wide;
use glam::Vec3;

// ============================================================================
// Macros for generating convex contact descriptions
// ============================================================================

/// Generate a convex one-body contact constraint description struct.
macro_rules! impl_convex_one_body_description {
    ($name:ident, $prestep:ty, $type_processor:ty, $count:expr, [$($contact_field:ident),+]) => {
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            $(pub $contact_field: ConstraintContactData,)+
            pub normal: Vec3,
            pub friction_coefficient: f32,
            pub spring_settings: SpringSettings,
            pub maximum_recovery_velocity: f32,
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &mut *(&mut target.contact0 as *mut ConvexContactWide).add(_i) };
                        Vector3Wide::write_first(self.$contact_field.offset_a, &mut contact.offset_a);
                        unsafe { *GatherScatter::get_first_mut(&mut contact.depth) = self.$contact_field.penetration_depth };
                        _i += 1;
                    }
                )+
                Vector3Wide::write_first(self.normal, &mut target.normal);
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.friction_coefficient) =
                        self.friction_coefficient;
                }
                SpringSettingsWide::write_first(&self.spring_settings, &mut target.material_properties.spring_settings);
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.maximum_recovery_velocity) =
                        self.maximum_recovery_velocity;
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &*(&source.contact0 as *const ConvexContactWide).add(_i) };
                        Vector3Wide::read_first(&contact.offset_a, &mut description.$contact_field.offset_a);
                        description.$contact_field.penetration_depth = unsafe { *GatherScatter::get_first(&contact.depth) };
                        _i += 1;
                    }
                )+
                Vector3Wide::read_first(&source.normal, &mut description.normal);
                description.friction_coefficient = unsafe { *GatherScatter::get_first(&source.material_properties.friction_coefficient) };
                SpringSettingsWide::read_first(&source.material_properties.spring_settings, &mut description.spring_settings);
                description.maximum_recovery_velocity = unsafe { *GatherScatter::get_first(&source.material_properties.maximum_recovery_velocity) };
            }

            /// Copies shared manifold-wide properties (normal, material) into this description.
            #[inline(always)]
            pub fn copy_manifold_wide_properties(&mut self, normal: &Vec3, material: &PairMaterialProperties) {
                self.normal = *normal;
                self.friction_coefficient = material.friction_coefficient;
                self.spring_settings = material.spring_settings;
                self.maximum_recovery_velocity = material.maximum_recovery_velocity;
            }

            /// Returns a pointer to the first contact data field.
            #[inline(always)]
            pub fn get_first_contact(&self) -> &ConstraintContactData {
                &self.contact0
            }
            #[inline(always)]
            pub fn get_first_contact_mut(&mut self) -> &mut ConstraintContactData {
                &mut self.contact0
            }
        }
    };
}

/// Generate a convex two-body contact constraint description struct.
macro_rules! impl_convex_two_body_description {
    ($name:ident, $prestep:ty, $type_processor:ty, $count:expr, [$($contact_field:ident),+]) => {
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            $(pub $contact_field: ConstraintContactData,)+
            pub offset_b: Vec3,
            pub normal: Vec3,
            pub friction_coefficient: f32,
            pub spring_settings: SpringSettings,
            pub maximum_recovery_velocity: f32,
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &mut *(&mut target.contact0 as *mut ConvexContactWide).add(_i) };
                        Vector3Wide::write_first(self.$contact_field.offset_a, &mut contact.offset_a);
                        unsafe { *GatherScatter::get_first_mut(&mut contact.depth) = self.$contact_field.penetration_depth };
                        _i += 1;
                    }
                )+
                Vector3Wide::write_first(self.offset_b, &mut target.offset_b);
                Vector3Wide::write_first(self.normal, &mut target.normal);
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.friction_coefficient) =
                        self.friction_coefficient;
                }
                SpringSettingsWide::write_first(&self.spring_settings, &mut target.material_properties.spring_settings);
                unsafe {
                    *GatherScatter::get_first_mut(&mut target.material_properties.maximum_recovery_velocity) =
                        self.maximum_recovery_velocity;
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &*(&source.contact0 as *const ConvexContactWide).add(_i) };
                        Vector3Wide::read_first(&contact.offset_a, &mut description.$contact_field.offset_a);
                        description.$contact_field.penetration_depth = unsafe { *GatherScatter::get_first(&contact.depth) };
                        _i += 1;
                    }
                )+
                Vector3Wide::read_first(&source.offset_b, &mut description.offset_b);
                Vector3Wide::read_first(&source.normal, &mut description.normal);
                description.friction_coefficient = unsafe { *GatherScatter::get_first(&source.material_properties.friction_coefficient) };
                SpringSettingsWide::read_first(&source.material_properties.spring_settings, &mut description.spring_settings);
                description.maximum_recovery_velocity = unsafe { *GatherScatter::get_first(&source.material_properties.maximum_recovery_velocity) };
            }

            /// Copies shared manifold-wide properties (offsetB, normal, material) into this description.
            #[inline(always)]
            pub fn copy_manifold_wide_properties(&mut self, offset_b: &Vec3, normal: &Vec3, material: &PairMaterialProperties) {
                self.offset_b = *offset_b;
                self.normal = *normal;
                self.friction_coefficient = material.friction_coefficient;
                self.spring_settings = material.spring_settings;
                self.maximum_recovery_velocity = material.maximum_recovery_velocity;
            }

            /// Returns a reference to the first contact data field.
            #[inline(always)]
            pub fn get_first_contact(&self) -> &ConstraintContactData {
                &self.contact0
            }
            #[inline(always)]
            pub fn get_first_contact_mut(&mut self) -> &mut ConstraintContactData {
                &mut self.contact0
            }
        }
    };
}

// ============================================================================
// Concrete description types — Convex One-Body
// ============================================================================

impl_convex_one_body_description!(
    Contact1OneBody,
    Contact1OneBodyPrestepData,
    Contact1OneBodyTypeProcessor,
    1,
    [contact0]
);

impl_convex_one_body_description!(
    Contact2OneBody,
    Contact2OneBodyPrestepData,
    Contact2OneBodyTypeProcessor,
    2,
    [contact0, contact1]
);

impl_convex_one_body_description!(
    Contact3OneBody,
    Contact3OneBodyPrestepData,
    Contact3OneBodyTypeProcessor,
    3,
    [contact0, contact1, contact2]
);

impl_convex_one_body_description!(
    Contact4OneBody,
    Contact4OneBodyPrestepData,
    Contact4OneBodyTypeProcessor,
    4,
    [contact0, contact1, contact2, contact3]
);

// ============================================================================
// Concrete description types — Convex Two-Body
// ============================================================================

impl_convex_two_body_description!(
    Contact1TwoBody,
    Contact1PrestepData,
    Contact1TypeProcessor,
    1,
    [contact0]
);

impl_convex_two_body_description!(
    Contact2TwoBody,
    Contact2PrestepData,
    Contact2TypeProcessor,
    2,
    [contact0, contact1]
);

impl_convex_two_body_description!(
    Contact3TwoBody,
    Contact3PrestepData,
    Contact3TypeProcessor,
    3,
    [contact0, contact1, contact2]
);

impl_convex_two_body_description!(
    Contact4TwoBody,
    Contact4PrestepData,
    Contact4TypeProcessor,
    4,
    [contact0, contact1, contact2, contact3]
);
