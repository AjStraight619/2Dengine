const RigidBody = @import("../body/body.zig").RigidBody;
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Forces = @import("forces.zig").Forces;

/// Physics integrators for updating body position, velocity, etc.
pub const Integrator = struct {
    /// Simple Euler integration
    pub fn euler(body: *RigidBody, dt: f32, gravity: Vector2) void {
        if (body.body_type == .static) return;

        // Apply gravity
        body.applyForce(gravity.scale(body.mass));

        // Calculate acceleration from forces
        const acceleration = body.force.scale(body.inverse_mass);

        // Update linear velocity
        body.velocity = body.velocity.add(acceleration.scale(dt));

        // Apply angular acceleration
        body.angular_velocity += body.torque * body.inverse_inertia * dt;

        // Update position and rotation
        body.position = body.position.add(body.velocity.scale(dt));
        body.rotation += body.angular_velocity * dt;

        // Reset forces for next frame
        body.force = Vector2.zero();
        body.torque = 0;

        // Update AABB
        body.updateAABB();
    }

    /// Semi-implicit Euler integration (better stability)
    pub fn semiImplicitEuler(body: *RigidBody, dt: f32, gravity: Vector2) void {
        if (body.body_type == .static) return;
        if (body.is_sleeping) return; // Skip integration for sleeping bodies

        // Apply gravity
        body.applyForce(gravity.scale(body.mass));

        // Calculate acceleration from forces
        const acceleration = body.force.scale(body.inverse_mass);

        // Update linear velocity first (different from standard Euler)
        body.velocity = body.velocity.add(acceleration.scale(dt));

        // Apply angular acceleration
        body.angular_velocity += body.torque * body.inverse_inertia * dt;

        // Update position using the updated velocity
        body.position = body.position.add(body.velocity.scale(dt));
        body.rotation += body.angular_velocity * dt;

        // Reset forces for next frame
        body.force = Vector2.zero();
        body.torque = 0;

        // Update AABB
        body.updateAABB();
    }

    /// Verlet integration (better energy conservation)
    pub fn verlet(body: *RigidBody, dt: f32, gravity: Vector2) void {
        if (body.body_type == .static) return;

        // Store last position for Verlet integration
        const last_position = body.position;

        // Apply gravity
        body.applyForce(gravity.scale(body.mass));

        // Calculate acceleration
        const acceleration = body.force.scale(body.inverse_mass);

        // Verlet position update using current position, last position, and acceleration
        if (body.last_position.x == 0 and body.last_position.y == 0) {
            // First frame, use Euler
            body.position = body.position.add(body.velocity.scale(dt));
        } else {
            // Verlet integration
            const new_position = body.position.scale(2).sub(body.last_position).add(acceleration.scale(dt * dt));
            body.last_position = body.position;
            body.position = new_position;
        }

        // Calculate velocity from positions
        if (dt > 0) {
            body.velocity = body.position.sub(last_position).scale(1 / dt);
        }

        // Angular motion (simple Euler for now)
        body.angular_velocity += body.torque * body.inverse_inertia * dt;
        body.rotation += body.angular_velocity * dt;

        // Reset forces
        body.force = Vector2.zero();
        body.torque = 0;

        // Update AABB
        body.updateAABB();
    }
};
