const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const Collision = @import("detector.zig").Collision;
const std = @import("std");

/// Collision resolution system
pub const CollisionResolver = struct {
    /// Resolve a collision between two bodies
    pub fn resolveCollision(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip resolution if both bodies are static (not just either one)
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        std.debug.print("\n>>> RESOLVING COLLISION <<<\n", .{});
        std.debug.print("A: type={}, static={}, pos=({d},{d}), vel=({d},{d})\n", .{ a.shape, a.body_type == .static, a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        std.debug.print("B: type={}, static={}, pos=({d},{d}), vel=({d},{d})\n", .{ b.shape, b.body_type == .static, b.position.x, b.position.y, b.velocity.x, b.velocity.y });
        std.debug.print("Collision normal=({d},{d}) - THIS POINTS FROM A TO B\n", .{ collision.normal.x, collision.normal.y });

        // Position correction (prevents sinking)
        resolvePositions(a, b, collision);

        // Impulse resolution (bounce and friction)
        resolveVelocities(a, b, collision);

        std.debug.print("After resolution - A: pos=({d},{d}), vel=({d},{d})\n", .{ a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        std.debug.print("After resolution - B: pos=({d},{d}), vel=({d},{d})\n", .{ b.position.x, b.position.y, b.velocity.x, b.velocity.y });
    }

    /// Resolve positions to prevent objects from sinking into each other
    fn resolvePositions(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip if both bodies are static
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        // Use different correction factors based on collision orientation
        var correction_factor: f32 = 1.01; // Default small bias

        // For top collisions (normal pointing up), use stronger correction
        if (collision.normal.y < -0.9) {
            correction_factor = 1.1; // 10% stronger correction for top collisions
            std.debug.print("TOP collision - using stronger position correction: {d}\n", .{correction_factor});
        }

        const correction_depth = collision.depth * correction_factor;
        const correction = collision.normal.scale(correction_depth);

        std.debug.print("Position correction: depth={d}, adjusted={d}\n", .{ collision.depth, correction_depth });

        // Apply the correction based on mass properties
        if (a.body_type == .static) {
            // Only push body B (static A can't move)
            b.position = b.position.add(correction);
            std.debug.print("Static A: moved B to ({d},{d})\n", .{ b.position.x, b.position.y });
        } else if (b.body_type == .static) {
            // Only push body A (static B can't move)
            a.position = a.position.sub(correction);
            std.debug.print("Static B: moved A to ({d},{d})\n", .{ a.position.x, a.position.y });
        } else {
            // Push both bodies based on their relative masses
            const total_inv_mass = a.inverse_mass + b.inverse_mass;

            if (total_inv_mass <= 0.0001) return;

            const a_ratio = a.inverse_mass / total_inv_mass;
            const b_ratio = b.inverse_mass / total_inv_mass;

            a.position = a.position.sub(correction.scale(a_ratio));
            b.position = b.position.add(correction.scale(b_ratio));
            std.debug.print("Moved both: A to ({d},{d}), B to ({d},{d})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        }

        // Update AABBs after position changes
        a.updateAABB();
        b.updateAABB();
    }

    /// Resolve velocities using impulses
    fn resolveVelocities(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip resolution if both bodies are static
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        // Get the contact point
        const contact = collision.contact_points[0];

        // Calculate relative velocity at contact point
        const ra = contact.sub(a.position);
        const rb = contact.sub(b.position);
        const relative_velocity = calculateRelativeVelocity(a, ra, b, rb);

        // Project relative velocity onto the collision normal
        const normal_velocity = relative_velocity.dot(collision.normal);

        std.debug.print("Relative velocity along normal: {d}\n", .{normal_velocity});

        // Skip if objects are moving away from each other faster than a small threshold
        const separation_threshold = 0.1; // Small positive threshold to allow slight separation
        if (normal_velocity > separation_threshold) {
            std.debug.print("Objects separating, skipping impulse\n", .{});
            return;
        }

        // Calculate impulse factors
        const ra_cross_n = ra.cross(collision.normal);
        const rb_cross_n = rb.cross(collision.normal);

        const inv_mass_sum = a.inverse_mass + b.inverse_mass +
            (ra_cross_n * ra_cross_n) * a.inverse_inertia +
            (rb_cross_n * rb_cross_n) * b.inverse_inertia;

        if (inv_mass_sum <= 0.0001) return;

        // SPECIAL CASE: If normal is pointing up (0,-1), apply extra damping to help rest on top
        var restitution = collision.restitution;
        if (collision.normal.y < -0.9) {
            std.debug.print("TOP collision detected - applying extra damping\n", .{});

            // Apply more damping for low velocities to help come to rest
            if (@abs(normal_velocity) < 2.0) {
                restitution *= 0.2; // Greatly reduce bounciness for top collisions

                // Also directly dampen vertical velocity when close to coming to rest
                if (@abs(normal_velocity) < 0.5 and @abs(a.velocity.y) < 1.0) {
                    std.debug.print("Near rest - strong damping applied\n", .{});
                    if (a.body_type != .static) {
                        a.velocity.y *= 0.5; // Directly reduce vertical velocity
                    }
                }
            }
        } else {
            // For other collisions, reduce restitution for low-speed collisions
            if (@abs(normal_velocity) < 1.0) {
                restitution *= @abs(normal_velocity);
            }
        }

        std.debug.print("Using restitution: {d}\n", .{restitution});

        // Calculate impulse magnitude
        const j = -(1.0 + restitution) * normal_velocity / inv_mass_sum;

        std.debug.print("Applying impulse: {d}\n", .{j});

        // Apply the impulse
        const impulse = collision.normal.scale(j);
        applyImpulse(a, b, impulse, ra, rb);

        // Apply friction (simplified)
        const friction_factor = collision.friction * 0.8; // Reduce friction slightly for stability

        // Calculate tangent vector (perpendicular to normal)
        var tangent = relative_velocity.sub(collision.normal.scale(normal_velocity));
        const tangent_length = tangent.length();

        // Skip friction if minimal tangential velocity
        if (tangent_length < 0.0001) return;

        // Normalize tangent
        tangent = tangent.scale(1.0 / tangent_length);

        // Apply a simplified friction impulse
        const friction_impulse = tangent.scale(-relative_velocity.dot(tangent) * friction_factor);
        applyImpulse(a, b, friction_impulse, ra, rb);
    }

    /// Calculate relative velocity at a contact point
    fn calculateRelativeVelocity(a: *RigidBody, ra: Vector2, b: *RigidBody, rb: Vector2) Vector2 {
        // Calculate velocity at contact point
        var vel_a = a.velocity;
        var vel_b = b.velocity;

        // Add angular velocity contribution
        if (a.body_type != .static) {
            const perp_a = Vector2.init(-ra.y, ra.x);
            vel_a = vel_a.add(perp_a.scale(a.angular_velocity));
        }

        if (b.body_type != .static) {
            const perp_b = Vector2.init(-rb.y, rb.x);
            vel_b = vel_b.add(perp_b.scale(b.angular_velocity));
        }

        // Return relative velocity
        return vel_b.sub(vel_a);
    }

    /// Apply an impulse to both bodies
    fn applyImpulse(a: *RigidBody, b: *RigidBody, impulse: Vector2, ra: Vector2, rb: Vector2) void {
        // Apply linear impulse
        if (a.body_type != .static) {
            a.velocity = a.velocity.sub(impulse.scale(a.inverse_mass));
        }

        if (b.body_type != .static) {
            b.velocity = b.velocity.add(impulse.scale(b.inverse_mass));
        }

        // Apply angular impulse
        if (a.body_type != .static) {
            a.angular_velocity -= ra.cross(impulse) * a.inverse_inertia;
        }

        if (b.body_type != .static) {
            b.angular_velocity += rb.cross(impulse) * b.inverse_inertia;
        }
    }
};
