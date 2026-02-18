// Translated from BepuPhysics/Constraints/Contact/ContactConvexTypes.cs
// Contact constraint description structs — scalar form for scatter/gather to/from SIMD type batches.

use crate::physics::collision_detection::narrow_phase_callbacks::PairMaterialProperties;
use crate::physics::constraints::contact::contact_constraint_description::ConstraintContactData;
use crate::physics::constraints::contact::contact_convex_common::ConvexContactWide;
use crate::physics::constraints::contact::contact_convex_types::*;
use crate::physics::constraints::spring_settings::SpringSettings;
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &mut *(&mut prestep_bundle.contact0 as *mut ConvexContactWide).add(_i) };
                        Vector3Wide::write_slot(self.$contact_field.offset_a, inner_index, &mut contact.offset_a);
                        unsafe { *GatherScatter::get_mut(&mut contact.depth, inner_index) = self.$contact_field.penetration_depth };
                        _i += 1;
                    }
                )+
                Vector3Wide::write_slot(self.normal, inner_index, &mut prestep_bundle.normal);
                unsafe {
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.friction_coefficient, inner_index) =
                        self.friction_coefficient;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.spring_settings.angular_frequency, inner_index) =
                        self.spring_settings.angular_frequency;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.spring_settings.twice_damping_ratio, inner_index) =
                        self.spring_settings.twice_damping_ratio;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.maximum_recovery_velocity, inner_index) =
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &*(&prestep_bundle.contact0 as *const ConvexContactWide).add(_i) };
                        Vector3Wide::read_slot(&contact.offset_a, inner_index, &mut description.$contact_field.offset_a);
                        description.$contact_field.penetration_depth = unsafe { *GatherScatter::get(&contact.depth, inner_index) };
                        _i += 1;
                    }
                )+
                Vector3Wide::read_slot(&prestep_bundle.normal, inner_index, &mut description.normal);
                unsafe {
                    description.friction_coefficient = *GatherScatter::get(&prestep_bundle.material_properties.friction_coefficient, inner_index);
                    description.spring_settings.angular_frequency = *GatherScatter::get(&prestep_bundle.material_properties.spring_settings.angular_frequency, inner_index);
                    description.spring_settings.twice_damping_ratio = *GatherScatter::get(&prestep_bundle.material_properties.spring_settings.twice_damping_ratio, inner_index);
                    description.maximum_recovery_velocity = *GatherScatter::get(&prestep_bundle.material_properties.maximum_recovery_velocity, inner_index);
                }
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &mut *(&mut prestep_bundle.contact0 as *mut ConvexContactWide).add(_i) };
                        Vector3Wide::write_slot(self.$contact_field.offset_a, inner_index, &mut contact.offset_a);
                        unsafe { *GatherScatter::get_mut(&mut contact.depth, inner_index) = self.$contact_field.penetration_depth };
                        _i += 1;
                    }
                )+
                Vector3Wide::write_slot(self.offset_b, inner_index, &mut prestep_bundle.offset_b);
                Vector3Wide::write_slot(self.normal, inner_index, &mut prestep_bundle.normal);
                unsafe {
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.friction_coefficient, inner_index) =
                        self.friction_coefficient;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.spring_settings.angular_frequency, inner_index) =
                        self.spring_settings.angular_frequency;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.spring_settings.twice_damping_ratio, inner_index) =
                        self.spring_settings.twice_damping_ratio;
                    *GatherScatter::get_mut(&mut prestep_bundle.material_properties.maximum_recovery_velocity, inner_index) =
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
                let mut _i = 0;
                $(
                    {
                        let contact = unsafe { &*(&prestep_bundle.contact0 as *const ConvexContactWide).add(_i) };
                        Vector3Wide::read_slot(&contact.offset_a, inner_index, &mut description.$contact_field.offset_a);
                        description.$contact_field.penetration_depth = unsafe { *GatherScatter::get(&contact.depth, inner_index) };
                        _i += 1;
                    }
                )+
                Vector3Wide::read_slot(&prestep_bundle.offset_b, inner_index, &mut description.offset_b);
                Vector3Wide::read_slot(&prestep_bundle.normal, inner_index, &mut description.normal);
                unsafe {
                    description.friction_coefficient = *GatherScatter::get(&prestep_bundle.material_properties.friction_coefficient, inner_index);
                    description.spring_settings.angular_frequency = *GatherScatter::get(&prestep_bundle.material_properties.spring_settings.angular_frequency, inner_index);
                    description.spring_settings.twice_damping_ratio = *GatherScatter::get(&prestep_bundle.material_properties.spring_settings.twice_damping_ratio, inner_index);
                    description.maximum_recovery_velocity = *GatherScatter::get(&prestep_bundle.material_properties.maximum_recovery_velocity, inner_index);
                }
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
