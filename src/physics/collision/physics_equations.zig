const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Core physics equations for collision response
pub const CollisionPhysics = struct {
    //------------------------------------------------------------------------------
    // POSITION CORRECTION EQUATIONS
    //------------------------------------------------------------------------------

    /// d' = d * factor
    /// Calculate corrected collision depth with bias factor
    pub fn calculateCorrectedDepth(depth: f32, correction_factor: f32) f32 {
        return depth * correction_factor;
    }

    /// position_correction = normal * depth * correction_factor
    /// Calculate position correction vector from normal and depth
    pub fn calculatePositionCorrection(normal: Vector2, depth: f32, correction_factor: f32) Vector2 {
        return normal.scale(depth * correction_factor);
    }

    /// Calculate position correction ratio based on mass
    /// ratio_a = m_a / (m_a + m_b)
    pub fn calculateCorrectionRatio(inv_mass_a: f32, inv_mass_b: f32) f32 {
        const total_inv_mass = inv_mass_a + inv_mass_b;
        if (total_inv_mass <= 0.0001) return 0.0;
        return inv_mass_a / total_inv_mass;
    }

    //------------------------------------------------------------------------------
    // IMPULSE RESOLUTION EQUATIONS
    //------------------------------------------------------------------------------

    /// j = -(1 + e) * v_rel_n / (1/m_a + 1/m_b)
    /// Calculate impulse magnitude from velocity and masses
    pub fn calculateImpulseMagnitude(normal_velocity: f32, restitution: f32, inv_mass_sum: f32) f32 {
        if (inv_mass_sum <= 0.0001) return 0.0;
        return -(1.0 + restitution) * normal_velocity / inv_mass_sum;
    }

    /// impulse = normal * j
    /// Calculate impulse vector from normal and magnitude
    pub fn calculateImpulseVector(normal: Vector2, magnitude: f32) Vector2 {
        return normal.scale(magnitude);
    }

    /// Calculate velocity change from impulse (v' = v + j/m)
    pub fn calculateVelocityChange(impulse: Vector2, inverse_mass: f32) Vector2 {
        return impulse.scale(inverse_mass);
    }

    /// Calculate angular velocity change from impulse and lever arm
    pub fn calculateAngularVelocityChange(impulse: Vector2, r: Vector2, inverse_inertia: f32) f32 {
        return r.cross(impulse) * inverse_inertia;
    }

    /// Calculate restitution for different collision scenarios
    /// For top collisions or low-speed impacts, reduce restitution
    pub fn adjustRestitution(normal: Vector2, normal_velocity: f32, base_restitution: f32) f32 {
        var adjusted_restitution = base_restitution;

        // For top collisions (normal pointing up), apply extra damping
        if (normal.y < -0.9) {
            // Apply more damping for low velocities to help come to rest
            if (@abs(normal_velocity) < 2.0) {
                adjusted_restitution *= 0.2; // Greatly reduce bounciness
            }
        } else {
            // For other collisions, reduce restitution for low-speed collisions
            if (@abs(normal_velocity) < 10.0) {
                adjusted_restitution *= @abs(normal_velocity) / 10.0;
            }
        }

        return adjusted_restitution;
    }

    //------------------------------------------------------------------------------
    // FRICTION EQUATIONS
    //------------------------------------------------------------------------------

    /// Calculate tangent vector perpendicular to normal vector
    /// tangent = relative_velocity - (relative_velocity • normal) * normal
    pub fn calculateFrictionTangent(relative_velocity: Vector2, normal: Vector2, normal_velocity: f32) Vector2 {
        return relative_velocity.sub(normal.scale(normal_velocity));
    }

    /// Normalize tangent vector if length is significant
    pub fn normalizeTangentVector(tangent: Vector2) Vector2 {
        const tangent_length = tangent.length();
        if (tangent_length > 0.0001) {
            return tangent.scale(1.0 / tangent_length);
        }
        return tangent;
    }

    /// Calculate effective friction coefficient
    /// μ = μ_a * μ_b
    pub fn calculateFrictionCoefficient(friction_a: f32, friction_b: f32) f32 {
        return friction_a * friction_b;
    }

    /// Clamp friction impulse magnitude to maximum value
    /// |j_t| ≤ μ * |j_n|
    pub fn clampFrictionImpulse(j_t: f32, normal_impulse: f32, friction_coef: f32) f32 {
        const max_friction = @abs(normal_impulse * friction_coef);
        if (@abs(j_t) > max_friction) {
            return if (j_t > 0.0) max_friction else -max_friction;
        }
        return j_t;
    }

    /// Calculate relative velocity at a contact point
    /// vrel = vb + ωb×rb - va - ωa×ra
    pub fn calculateRelativeVelocity(vel_a: Vector2, vel_b: Vector2, angular_vel_a: f32, angular_vel_b: f32, ra: Vector2, rb: Vector2) Vector2 {
        // Calculate velocity at point considering both linear and angular components
        var point_vel_a = vel_a;
        var point_vel_b = vel_b;

        // Add angular velocity contribution
        const perp_a = Vector2.init(-ra.y, ra.x);
        point_vel_a = point_vel_a.add(perp_a.scale(angular_vel_a));

        const perp_b = Vector2.init(-rb.y, rb.x);
        point_vel_b = point_vel_b.add(perp_b.scale(angular_vel_b));

        // Return relative velocity
        return point_vel_b.sub(point_vel_a);
    }
};
