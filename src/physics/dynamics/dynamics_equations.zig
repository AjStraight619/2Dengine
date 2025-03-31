const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Core physics equations for dynamics calculations
pub const DynamicsEquations = struct {
    //------------------------------------------------------------------------------
    // FORCE EQUATIONS
    //------------------------------------------------------------------------------

    /// F = m * g
    /// Calculate gravitational force from mass and gravity
    pub fn gravitationalForce(mass: f32, gravity: Vector2) Vector2 {
        return gravity.scale(mass);
    }

    /// F = -c * v
    /// Calculate drag/air resistance force from velocity
    pub fn dragForce(velocity: Vector2, coefficient: f32) Vector2 {
        return velocity.scale(-coefficient);
    }

    /// F = -k * x
    /// Calculate spring force using Hooke's law
    pub fn springForce(displacement: Vector2, stiffness: f32) Vector2 {
        return displacement.scale(-stiffness);
    }

    /// F = -c * v
    /// Calculate damping force for springs
    pub fn dampingForce(velocity: Vector2, damping: f32) Vector2 {
        return velocity.scale(-damping);
    }

    /// F = μ * N * direction
    /// Calculate friction force
    pub fn frictionForce(normal_force: f32, friction_coef: f32, direction: Vector2) Vector2 {
        return direction.normalize().scale(-normal_force * friction_coef);
    }

    /// F = m * a → a = F/m
    /// Calculate acceleration from force and mass
    pub fn accelerationFromForce(force: Vector2, mass: f32) Vector2 {
        return force.scale(1.0 / mass);
    }

    //------------------------------------------------------------------------------
    // INTEGRATION EQUATIONS
    //------------------------------------------------------------------------------

    /// v = v₀ + a * dt
    /// Update velocity using Euler integration
    pub fn updateVelocityEuler(velocity: Vector2, acceleration: Vector2, dt: f32) Vector2 {
        return velocity.add(acceleration.scale(dt));
    }

    /// p = p₀ + v * dt
    /// Update position using Euler integration
    pub fn updatePositionEuler(position: Vector2, velocity: Vector2, dt: f32) Vector2 {
        return position.add(velocity.scale(dt));
    }

    /// ω = ω₀ + α * dt
    /// Update angular velocity using Euler integration
    pub fn updateAngularVelocityEuler(angular_velocity: f32, angular_acceleration: f32, dt: f32) f32 {
        return angular_velocity + angular_acceleration * dt;
    }

    /// θ = θ₀ + ω * dt
    /// Update rotation angle using Euler integration
    pub fn updateRotationEuler(rotation: f32, angular_velocity: f32, dt: f32) f32 {
        return rotation + angular_velocity * dt;
    }

    /// Verlet position update
    /// p = 2*p₀ - p₋₁ + a*dt²
    pub fn updatePositionVerlet(current_pos: Vector2, prev_pos: Vector2, acceleration: Vector2, dt: f32) Vector2 {
        return current_pos.scale(2).sub(prev_pos).add(acceleration.scale(dt * dt));
    }

    /// Velocity calculation from positions (for Verlet)
    /// v = (p - p₋₁) / dt
    pub fn velocityFromPositions(current_pos: Vector2, prev_pos: Vector2, dt: f32) Vector2 {
        return current_pos.sub(prev_pos).scale(1.0 / dt);
    }
};
