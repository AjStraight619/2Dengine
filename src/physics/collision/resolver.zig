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

        std.debug.print(">>> POSITION CORRECTION <<<\n", .{});
        std.debug.print("A body type: {s}, B body type: {s}\n", .{ @tagName(a.body_type), @tagName(b.body_type) });
        std.debug.print("Current A pos: ({d:.4},{d:.4}), B pos: ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        std.debug.print("Collision normal: ({d:.4},{d:.4}), depth: {d:.4}\n", .{ collision.normal.x, collision.normal.y, collision.depth });

        // Check if this is a horizontal collision between rectangles
        const is_horizontal = @abs(collision.normal.x) > 0.8;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // Use different correction factors based on collision orientation and types
        var correction_factor: f32 = 1.01; // Default small bias

        // For horizontal rect-rect collisions, use stronger correction to prevent sticking
        if (horizontal_rect_collision) {
            correction_factor = 1.2; // 20% stronger correction
            std.debug.print("HORIZONTAL rect-rect collision - using stronger position correction: {d}\n", .{correction_factor});
        }
        // For top collisions (normal pointing up), use stronger correction
        else if (collision.normal.y < -0.8) {
            correction_factor = 1.1; // 10% stronger correction for top collisions
            std.debug.print("TOP collision - using stronger position correction: {d}\n", .{correction_factor});
        }

        const correction_depth = collision.depth * correction_factor;
        const correction = collision.normal.scale(correction_depth);
        std.debug.print("Correction vector: ({d:.4},{d:.4})\n", .{ correction.x, correction.y });

        // Apply the correction based on mass properties
        if (a.body_type == .static) {
            // Only push body B (static A can't move)
            b.position = b.position.add(correction);
            std.debug.print("Static A: moved B to ({d:.4},{d:.4})\n", .{ b.position.x, b.position.y });
        } else if (b.body_type == .static) {
            // Only push body A (static B can't move)
            a.position = a.position.sub(correction);
            std.debug.print("Static B: moved A to ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y });
        } else {
            // Push both bodies based on their relative masses
            const total_inv_mass = a.inverse_mass + b.inverse_mass;

            if (total_inv_mass <= 0.0001) return;

            const a_ratio = a.inverse_mass / total_inv_mass;
            const b_ratio = b.inverse_mass / total_inv_mass;

            std.debug.print("Dynamic-Dynamic: A ratio: {d:.4}, B ratio: {d:.4}\n", .{ a_ratio, b_ratio });

            a.position = a.position.sub(correction.scale(a_ratio));
            b.position = b.position.add(correction.scale(b_ratio));
            std.debug.print("Moved both: A to ({d:.4},{d:.4}), B to ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
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

        std.debug.print(">>> VELOCITY RESOLUTION <<<\n", .{});
        std.debug.print("Before: A vel=({d:.4},{d:.4}), B vel=({d:.4},{d:.4})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Get the contact point
        const contact = collision.contact_points[0];
        std.debug.print("Contact point: ({d:.4},{d:.4})\n", .{ contact.x, contact.y });

        // Calculate relative velocity at contact point
        const ra = contact.sub(a.position);
        const rb = contact.sub(b.position);
        const relative_velocity = calculateRelativeVelocity(a, ra, b, rb);
        std.debug.print("Relative velocity: ({d:.4},{d:.4})\n", .{ relative_velocity.x, relative_velocity.y });

        // Project relative velocity onto the collision normal
        const normal_velocity = relative_velocity.dot(collision.normal);
        std.debug.print("Relative velocity along normal: {d:.4}\n", .{normal_velocity});

        // If objects are moving away fast enough, skip impulse
        const separation_tolerance = 0.01; // Very small tolerance
        if (normal_velocity > separation_tolerance) {
            std.debug.print("Objects DEFINITELY separating, skipping impulse\n", .{});
            return;
        }

        // Check if this is a horizontal collision between rectangles (for special handling)
        const is_horizontal = @abs(collision.normal.x) > 0.8;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // Calculate impulse factors
        const ra_cross_n = ra.cross(collision.normal);
        const rb_cross_n = rb.cross(collision.normal);

        const inv_mass_sum = a.inverse_mass + b.inverse_mass +
            (ra_cross_n * ra_cross_n) * a.inverse_inertia +
            (rb_cross_n * rb_cross_n) * b.inverse_inertia;

        if (inv_mass_sum <= 0.0001) return;

        // Calculate restitution based on collision type
        var restitution = collision.restitution;

        // Enhance restitution for horizontal rectangle collisions to improve bouncing
        if (horizontal_rect_collision) {
            // Increase bounce effect for horizontal collisions
            restitution *= 1.2;
            std.debug.print("Horizontal rect-rect collision: enhancing restitution to {d:.4}\n", .{restitution});
        }

        // Reduce bounciness for low-speed collisions (linear damping)
        const velocity_threshold = 10.0;
        if (@abs(normal_velocity) < velocity_threshold) {
            restitution *= @abs(normal_velocity) / velocity_threshold;
        }

        // Calculate impulse magnitude
        var j = -(1.0 + restitution) * normal_velocity / inv_mass_sum;

        // Cap impulse to avoid numerical explosions
        const max_impulse = 1000.0;
        if (@abs(j) > max_impulse) {
            std.debug.print("WARNING: Capping excessive impulse from {d:.4} to {d:.4}\n", .{ j, std.math.sign(j) * max_impulse });
            j = std.math.sign(j) * max_impulse;
        }

        std.debug.print("Impulse magnitude: {d:.4}\n", .{j});

        // Apply the impulse along the normal only
        const impulse = collision.normal.scale(j);
        std.debug.print("Impulse vector: ({d:.4},{d:.4})\n", .{ impulse.x, impulse.y });

        // Log before applying impulse
        std.debug.print("Before impulse: A vel=({d:.4},{d:.4}), B vel=({d:.4},{d:.4})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Apply impulse
        applyImpulse(a, b, impulse, ra, rb);

        // Log after applying impulse
        std.debug.print("After impulse: A vel=({d:.4},{d:.4}), B vel=({d:.4},{d:.4})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Apply friction
        applyFriction(a, b, collision, relative_velocity, normal_velocity, j, ra, rb, horizontal_rect_collision);

        // Final velocities after all impulses
        std.debug.print("Final velocities: A=({d:.4},{d:.4}), B=({d:.4},{d:.4})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });
    }

    /// Apply friction impulse
    fn applyFriction(a: *RigidBody, b: *RigidBody, collision: Collision, relative_velocity: Vector2, normal_velocity: f32, normal_impulse: f32, ra: Vector2, rb: Vector2, is_horizontal_rect_collision: bool) void {

        // Calculate tangent vector (perpendicular to normal)
        var tangent = relative_velocity.sub(collision.normal.scale(normal_velocity));
        const tangent_length = tangent.length();

        // Skip friction if negligible tangential velocity
        // Use different thresholds based on collision type
        if (is_horizontal_rect_collision) {
            // Lower threshold for horizontal rect collisions
            if (tangent_length < 0.01) {
                std.debug.print("Horizontal rect collision: skipping friction due to minimal tangential velocity\n", .{});
                return;
            }
        } else {
            // Standard threshold for other collisions
            if (tangent_length < 0.1) {
                std.debug.print("Skipping friction due to minimal tangential velocity\n", .{});
                return;
            }
        }

        // Normalize tangent
        tangent = tangent.scale(1.0 / tangent_length);

        // Adjust friction coefficient for horizontal rect-rect collisions
        var friction = collision.friction;
        if (is_horizontal_rect_collision) {
            // Reduce friction for horizontal collisions to prevent sticking
            friction *= 0.7;
            std.debug.print("Horizontal rect-rect collision: reducing friction to {d:.4}\n", .{friction});
        }

        // Calculate and apply friction impulse with adjusted magnitude
        var friction_j = -relative_velocity.dot(tangent) * friction * 0.5 * @abs(normal_impulse);

        // Cap friction impulse
        if (@abs(friction_j) > @abs(normal_impulse) * friction) {
            friction_j = -@abs(normal_impulse) * friction * std.math.sign(friction_j);
        }

        // Convert to vector and apply
        const friction_impulse = tangent.scale(friction_j);
        std.debug.print("Friction impulse: ({d:.4},{d:.4})\n", .{ friction_impulse.x, friction_impulse.y });

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
