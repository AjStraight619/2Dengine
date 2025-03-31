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

    /// Calculate friction impulse magnitude
    /// j_f = -tangent_velocity * μ
    pub fn calculateFrictionImpulseMagnitude(tangent_velocity: f32, friction_coefficient: f32) f32 {
        return -tangent_velocity * friction_coefficient;
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
            if (@abs(normal_velocity) < 1.0) {
                adjusted_restitution *= @abs(normal_velocity);
            }
        }

        return adjusted_restitution;
    }
};
