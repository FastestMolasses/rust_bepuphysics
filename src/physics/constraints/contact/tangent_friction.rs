// Translated from BepuPhysics/Constraints/Contact/TangentFriction.cs

use crate::out;
use crate::physics::body_properties::{BodyInertiaWide, BodyVelocityWide};
use crate::utilities::matrix2x3_wide::Matrix2x3Wide;
use crate::utilities::symmetric2x2_wide::Symmetric2x2Wide;
use crate::utilities::symmetric3x3_wide::Symmetric3x3Wide;
use crate::utilities::vector::{HwMinMax, Vector};
use crate::utilities::vector2_wide::Vector2Wide;
use crate::utilities::vector3_wide::Vector3Wide;

/// Handles the tangent friction implementation.
pub struct TangentFriction;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct TangentFrictionJacobians {
    pub linear_a: Matrix2x3Wide,
    pub angular_a: Matrix2x3Wide,
    pub angular_b: Matrix2x3Wide,
}

impl TangentFriction {
    //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
    #[inline(always)]
    pub fn compute_jacobians(
        tangent_x: &Vector3Wide,
        tangent_y: &Vector3Wide,
        offset_a: &Vector3Wide,
        offset_b: &Vector3Wide,
        jacobians: &mut TangentFrictionJacobians,
    ) {
        //Two velocity constraints:
        //dot(velocity(p, A), tangentX) = dot(velocity(p, B), tangentX)
        //dot(velocity(p, A), tangentY) = dot(velocity(p, B), tangentY)
        //jLinearA = [ tangentX ]
        //           [ tangentY ]
        //jAngularA = [ offsetA x tangentX ]
        //            [ offsetA x tangentY ]
        //jLinearB = [ -tangentX ]
        //           [ -tangentY ]
        //jAngularB = [ tangentX x offsetB ]
        //            [ tangentY x offsetB ]
        jacobians.linear_a.x = *tangent_x;
        jacobians.linear_a.y = *tangent_y;
        unsafe {
            Vector3Wide::cross_without_overlap(offset_a, tangent_x, &mut jacobians.angular_a.x);
            Vector3Wide::cross_without_overlap(offset_a, tangent_y, &mut jacobians.angular_a.y);
            Vector3Wide::cross_without_overlap(tangent_x, offset_b, &mut jacobians.angular_b.x);
            Vector3Wide::cross_without_overlap(tangent_y, offset_b, &mut jacobians.angular_b.y);
        }
    }

    /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
    #[inline(always)]
    pub fn apply_impulse(
        jacobians: &TangentFrictionJacobians,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        corrective_impulse: &Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let linear_impulse_a = out!(Matrix2x3Wide::transform(
            corrective_impulse,
            &jacobians.linear_a
        ));
        let angular_impulse_a = out!(Matrix2x3Wide::transform(
            corrective_impulse,
            &jacobians.angular_a
        ));
        let angular_impulse_b = out!(Matrix2x3Wide::transform(
            corrective_impulse,
            &jacobians.angular_b
        ));

        let corrective_velocity_a_linear = out!(Vector3Wide::scale_to(
            &linear_impulse_a,
            &inertia_a.inverse_mass
        ));
        let corrective_velocity_a_angular = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_a,
            &inertia_a.inverse_inertia_tensor
        ));
        let corrective_velocity_b_linear = out!(Vector3Wide::scale_to(
            &linear_impulse_a,
            &inertia_b.inverse_mass
        ));
        let corrective_velocity_b_angular = out!(Symmetric3x3Wide::transform_without_overlap(
            &angular_impulse_b,
            &inertia_b.inverse_inertia_tensor
        ));
        let temp = wsv_a.linear;
        Vector3Wide::add(&temp, &corrective_velocity_a_linear, &mut wsv_a.linear);
        let temp = wsv_a.angular;
        Vector3Wide::add(&temp, &corrective_velocity_a_angular, &mut wsv_a.angular);
        let temp = wsv_b.linear;
        Vector3Wide::subtract(&temp, &corrective_velocity_b_linear, &mut wsv_b.linear); //note subtract- we based it on the LinearA jacobian.
        let temp = wsv_b.angular;
        Vector3Wide::add(&temp, &corrective_velocity_b_angular, &mut wsv_b.angular);
    }

    #[inline(always)]
    pub fn compute_corrective_impulse(
        wsv_a: &BodyVelocityWide,
        wsv_b: &BodyVelocityWide,
        effective_mass: &Symmetric2x2Wide,
        jacobians: &TangentFrictionJacobians,
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector2Wide,
        corrective_csi: &mut Vector2Wide,
    ) {
        let csva_linear = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_a.linear,
            &jacobians.linear_a
        ));
        let csva_angular = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_a.angular,
            &jacobians.angular_a
        ));
        let csvb_linear = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_b.linear,
            &jacobians.linear_a
        ));
        let csvb_angular = out!(Matrix2x3Wide::transform_by_transpose_without_overlap(
            &wsv_b.angular,
            &jacobians.angular_b
        ));
        //Note that the velocity in constraint space is (csvaLinear - csvbLinear + csvaAngular + csvbAngular).
        //The subtraction there is due to sharing the linear jacobian between both bodies.
        //In the following, we need to compute the constraint space *violating* velocity- which is the negation of the above velocity in constraint space.
        //So, (csvbLinear - csvaLinear - (csvaAngular + csvbAngular)).
        let csv_linear = out!(Vector2Wide::subtract(&csvb_linear, &csva_linear));
        let csv_angular = out!(Vector2Wide::add(&csva_angular, &csvb_angular));
        let csv = out!(Vector2Wide::subtract(&csv_linear, &csv_angular));

        let csi = out!(Symmetric2x2Wide::transform_without_overlap(
            &csv,
            effective_mass
        ));

        let previous_accumulated = *accumulated_impulse;
        let temp = *accumulated_impulse;
        Vector2Wide::add(&temp, &csi, accumulated_impulse);
        //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
        let accumulated_magnitude = out!(Vector2Wide::length(accumulated_impulse));
        //Note division by zero guard.
        let scale = Vector::<f32>::splat(1.0)
            .hw_min(*maximum_impulse / Vector::<f32>::splat(1e-16).hw_max(accumulated_magnitude));
        let temp = *accumulated_impulse;
        Vector2Wide::scale(&temp, &scale, accumulated_impulse);

        Vector2Wide::subtract(accumulated_impulse, &previous_accumulated, corrective_csi);
    }

    #[inline(always)]
    pub fn warm_start(
        tangent_x: &Vector3Wide,
        tangent_y: &Vector3Wide,
        offset_to_manifold_center_a: &Vector3Wide,
        offset_to_manifold_center_b: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        accumulated_impulse: &Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let jacobians = out!(Self::compute_jacobians(
            tangent_x,
            tangent_y,
            offset_to_manifold_center_a,
            offset_to_manifold_center_b
        ));
        //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
        //To compensate for this, the accumulated impulse should be scaled if dt changes.
        Self::apply_impulse(
            &jacobians,
            inertia_a,
            inertia_b,
            accumulated_impulse,
            wsv_a,
            wsv_b,
        );
    }

    #[inline(always)]
    pub fn solve(
        tangent_x: &Vector3Wide,
        tangent_y: &Vector3Wide,
        offset_to_manifold_center_a: &Vector3Wide,
        offset_to_manifold_center_b: &Vector3Wide,
        inertia_a: &BodyInertiaWide,
        inertia_b: &BodyInertiaWide,
        maximum_impulse: &Vector<f32>,
        accumulated_impulse: &mut Vector2Wide,
        wsv_a: &mut BodyVelocityWide,
        wsv_b: &mut BodyVelocityWide,
    ) {
        let jacobians = out!(Self::compute_jacobians(
            tangent_x,
            tangent_y,
            offset_to_manifold_center_a,
            offset_to_manifold_center_b
        ));
        //Compute effective mass matrix contributions.
        let linear_contribution_a = out!(Symmetric2x2Wide::sandwich_scale(
            &jacobians.linear_a,
            &inertia_a.inverse_mass
        ));
        let linear_contribution_b = out!(Symmetric2x2Wide::sandwich_scale(
            &jacobians.linear_a,
            &inertia_b.inverse_mass
        ));

        let angular_contribution_a = out!(Symmetric3x3Wide::matrix_sandwich(
            &jacobians.angular_a,
            &inertia_a.inverse_inertia_tensor
        ));
        let angular_contribution_b = out!(Symmetric3x3Wide::matrix_sandwich(
            &jacobians.angular_b,
            &inertia_b.inverse_inertia_tensor
        ));

        //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
        let linear = out!(Symmetric2x2Wide::add(
            &linear_contribution_a,
            &linear_contribution_b
        ));
        let angular = out!(Symmetric2x2Wide::add(
            &angular_contribution_a,
            &angular_contribution_b
        ));
        let inverse_effective_mass = out!(Symmetric2x2Wide::add(&linear, &angular));
        let effective_mass = out!(Symmetric2x2Wide::invert_without_overlap(
            &inverse_effective_mass
        ));

        let corrective_csi = out!(Self::compute_corrective_impulse(
            wsv_a,
            wsv_b,
            &effective_mass,
            &jacobians,
            maximum_impulse,
            accumulated_impulse
        ));
        Self::apply_impulse(
            &jacobians,
            inertia_a,
            inertia_b,
            &corrective_csi,
            wsv_a,
            wsv_b,
        );
    }
}
