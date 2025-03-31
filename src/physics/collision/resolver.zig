const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const Collision = @import("detector.zig").Collision;
const std = @import("std");
const detector = @import("detector.zig");

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
        const is_horizontal = @abs(collision.normal.x) > 0.9; // Increased threshold for more accurate detection
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // Check for vertical collision with ground
        const is_vertical = @abs(collision.normal.y) > 0.9;
        const is_ground_collision = is_vertical and collision.normal.y < 0;
        const vertical_rect_ground_collision = is_ground_collision and is_rect_vs_rect;

        // Use different correction factors based on collision orientation and types
        var correction_factor: f32 = 1.01; // Default small bias

        // For horizontal rect-rect collisions, use stable correction
        if (horizontal_rect_collision) {
            correction_factor = 1.05; // Reduced from 1.2 for more stable behavior
            std.debug.print("HORIZONTAL rect-rect collision - using moderate position correction: {d}\n", .{correction_factor});
        }
        // For ground collisions (normal pointing up), use stronger correction
        else if (vertical_rect_ground_collision) {
            correction_factor = 1.1; // 10% stronger correction for ground collisions
            std.debug.print("GROUND collision - using stronger position correction: {d}\n", .{correction_factor});
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

        // Check for extremely high velocities and log them
        const vel_a_len = a.velocity.length();
        const vel_b_len = b.velocity.length();
        const velocity_threshold: f32 = 500.0;

        if (vel_a_len > velocity_threshold or vel_b_len > velocity_threshold) {
            std.debug.print("\n🚨 EXTREME VELOCITY DURING RESOLUTION:\n", .{});
            std.debug.print("  A: vel={d:.2} at ({d:.2},{d:.2})\n", .{ vel_a_len, a.position.x, a.position.y });
            std.debug.print("  B: vel={d:.2} at ({d:.2},{d:.2})\n", .{ vel_b_len, b.position.x, b.position.y });
            std.debug.print("  Normal: ({d:.2},{d:.2}), depth: {d:.2}\n", .{ collision.normal.x, collision.normal.y, collision.depth });
        }

        std.debug.print(">>> VELOCITY RESOLUTION <<<\n", .{});
        std.debug.print("Before: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Check if this is a vertical collision between rectangles with very small velocity
        // This is likely a "resting" scenario that causes jittering
        const is_vertical = @abs(collision.normal.y) > 0.9;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const vertical_rect_collision = is_vertical and is_rect_vs_rect;

        // DEBUG: Special logging for near-rest vertical rect collisions
        if (vertical_rect_collision) {
            const velocity_near_zero = @abs(b.velocity.y) < 0.1;
            if (velocity_near_zero) {
                std.debug.print("JITTER-DEBUG: Near-rest vertical rect collision detected!\n", .{});
                std.debug.print("JITTER-DEBUG: Velocity before resolution: {d:.8}\n", .{b.velocity.y});
                std.debug.print("JITTER-DEBUG: Gravity applied this frame: {d:.8}\n", .{b.force.y});
            }
        }

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

        // Better separation handling - only skip if objects are DEFINITELY separating
        // Increase separation tolerance for horizontal collisions to prevent sticking
        var separation_tolerance: f32 = 0.001; // Default very small tolerance

        // Check if this is a horizontal collision between rectangles
        const is_horizontal = @abs(collision.normal.x) > 0.9;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        if (horizontal_rect_collision) {
            // Much higher tolerance for horizontal rectangle collisions
            separation_tolerance = 0.05; // 50x higher tolerance for horizontal rect collisions
        }

        if (normal_velocity > separation_tolerance) {
            std.debug.print("Objects DEFINITELY separating, skipping impulse\n", .{});
            return;
        }

        // Calculate impulse factors
        const ra_cross_n = ra.cross(collision.normal);
        const rb_cross_n = rb.cross(collision.normal);

        const inv_mass_sum = a.inverse_mass + b.inverse_mass +
            (ra_cross_n * ra_cross_n) * a.inverse_inertia +
            (rb_cross_n * rb_cross_n) * b.inverse_inertia;

        if (inv_mass_sum <= 0.0001) return;

        // Calculate restitution based on collision type
        var restitution = collision.restitution;

        // For horizontal rectangles, drastically reduce restitution to prevent bouncing
        if (horizontal_rect_collision) {
            // Near-zero restitution for horizontal collisions
            restitution *= 0.1;
            std.debug.print("Horizontal rect-rect collision: reducing restitution to {d:.4}\n", .{restitution});
        }

        // Reduce restitution for vertical collisions as well
        if (vertical_rect_collision) {
            // Objects resting on top of each other should have less bounce
            if (collision.normal.y < 0 and @abs(normal_velocity) < 1.0) {
                restitution *= 0.06; // Reduced to 6% of original restitution
                std.debug.print("Vertical rect-rect collision (top): reducing restitution to {d:.4}\n", .{restitution});
            }
        }

        // Reduce bounciness for low-speed collisions (linear damping)
        const velocity_threshold_for_damping = 10.0;
        if (@abs(normal_velocity) < velocity_threshold_for_damping) {
            restitution *= @abs(normal_velocity) / velocity_threshold_for_damping;
        }

        // Calculate impulse magnitude
        var j = -(1.0 + restitution) * normal_velocity / inv_mass_sum;

        // Cap impulse to avoid numerical explosions - REDUCED MAX IMPULSE FOR STABILITY
        const max_impulse = 500.0; // Significantly reduced from 1000.0
        if (@abs(j) > max_impulse) {
            std.debug.print("⚠️ EXCESSIVE IMPULSE CAPPED: {d:.4} → {d:.4}\n", .{ j, std.math.sign(j) * max_impulse });
            j = std.math.sign(j) * max_impulse;
        }

        std.debug.print("Impulse magnitude: {d:.4}\n", .{j});

        // Apply the impulse along the normal only
        const impulse = collision.normal.scale(j);
        std.debug.print("Impulse vector: ({d:.4},{d:.4})\n", .{ impulse.x, impulse.y });

        // Log before applying impulse
        std.debug.print("Before impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Apply impulse
        applyImpulse(a, b, impulse, ra, rb);

        // Log after applying impulse
        std.debug.print("After impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Check for sudden large changes in velocity - could indicate instability
        const new_vel_a_len = a.velocity.length();
        const new_vel_b_len = b.velocity.length();
        const velocity_change_a = @abs(new_vel_a_len - vel_a_len);
        const velocity_change_b = @abs(new_vel_b_len - vel_b_len);

        if (velocity_change_a > 100.0 or velocity_change_b > 100.0) {
            std.debug.print("⚠️ LARGE VELOCITY CHANGE DETECTED!\n", .{});
            std.debug.print("  A: {d:.2} → {d:.2} (change: {d:.2})\n", .{ vel_a_len, new_vel_a_len, velocity_change_a });
            std.debug.print("  B: {d:.2} → {d:.2} (change: {d:.2})\n", .{ vel_b_len, new_vel_b_len, velocity_change_b });
        }

        // Apply friction
        applyFriction(a, b, collision, relative_velocity, normal_velocity, j, ra, rb, horizontal_rect_collision);

        // DEBUG: Check for small vertical velocity changes that might cause jitter
        if (vertical_rect_collision and b.body_type != .static) {
            if (@abs(b.velocity.y) < 0.05) {
                std.debug.print("JITTER-DEBUG: After resolution, very small velocity: {d:.8}\n", .{b.velocity.y});

                // DEBUG: Track position and velocity over frames to analyze jitter
                std.debug.print("JITTER-DEBUG: POSITION: {d:.8}, VELOCITY: {d:.8}\n", .{ b.position.y, b.velocity.y });
            }
        }

        // Final velocities after all impulses
        std.debug.print("Final velocities: A=({d:.6},{d:.6}), B=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });
    }

    /// Apply friction impulse
    fn applyFriction(a: *RigidBody, b: *RigidBody, collision: Collision, relative_velocity: Vector2, normal_velocity: f32, normal_impulse: f32, ra: Vector2, rb: Vector2, is_horizontal_rect_collision: bool) void {
        // Check for vertical collision condition (for resting objects)
        const is_vertical = @abs(collision.normal.y) > 0.9;

        // Calculate tangent vector (perpendicular to normal)
        var tangent = relative_velocity.sub(collision.normal.scale(normal_velocity));
        const tangent_length = tangent.length();

        // Skip friction if negligible tangential velocity
        var tangent_threshold: f32 = 0.1; // Default threshold

        // Horizontal rectangle collisions need smaller threshold for better responsiveness
        if (is_horizontal_rect_collision) {
            tangent_threshold = 0.01; // 10x more sensitive for horizontal collisions
        }

        if (tangent_length < tangent_threshold) {
            std.debug.print("Skipping friction due to minimal tangential velocity: {d:.4}\n", .{tangent_length});
            return;
        }

        // Normalize tangent vector
        tangent = tangent.scale(1.0 / tangent_length);

        // Calculate friction impulse factor
        // Coulomb's Law of Friction (max_friction = mu * normal_force)
        var friction_coef = (a.friction * b.friction);

        // Reduce friction for horizontal collisions (helps prevent sticking)
        if (is_horizontal_rect_collision) {
            friction_coef *= 0.7; // 30% reduction for horizontal collisions
        }

        // Reduce friction for vertical resting contacts to reduce jitter
        if (is_vertical and @abs(relative_velocity.y) < 0.05) {
            friction_coef *= 0.5; // 50% reduction for resting contacts
        }

        // Calculate friction impulse magnitude (limited by normal impulse)
        var j_t = -(relative_velocity.dot(tangent)) / (a.inverse_mass + b.inverse_mass +
            (ra.cross(tangent) * ra.cross(tangent)) * a.inverse_inertia +
            (rb.cross(tangent) * rb.cross(tangent)) * b.inverse_inertia);

        // Clamp friction impulse magnitude
        const max_friction = @abs(normal_impulse * friction_coef);
        if (@abs(j_t) > max_friction) {
            j_t = if (j_t > 0.0) max_friction else -max_friction;
        }

        // Calculate friction impulse vector
        const friction_impulse = tangent.scale(j_t);

        // Store initial velocities for debugging
        const a_vel_before = a.velocity;
        const b_vel_before = b.velocity;

        // Debug print the friction impulse
        std.debug.print("Friction impulse: ({d:.4},{d:.4})\n", .{ friction_impulse.x, friction_impulse.y });

        // CRITICAL FIX: Don't apply friction in the y-direction if it would cause the object to move upward
        var safe_friction_impulse = friction_impulse;

        // For objects on the ground (vertical collision with normal pointing up)
        if (is_vertical and collision.normal.y < 0) {
            // If friction would cause object to move upward, zero out the y component
            if ((a.body_type == .dynamic and (a.velocity.y < 0 and friction_impulse.y * a.inverse_mass > @abs(a.velocity.y))) or
                (b.body_type == .dynamic and (b.velocity.y < 0 and friction_impulse.y * b.inverse_mass < -@abs(b.velocity.y))))
            {
                std.debug.print("⚠️ PREVENTING INVALID UPWARD FRICTION!\n", .{});
                safe_friction_impulse.y = 0;
            }
        }

        // Apply friction impulse
        if (a.body_type == .dynamic) {
            a.velocity = a.velocity.add(safe_friction_impulse.scale(-a.inverse_mass));
            a.angular_velocity -= ra.cross(safe_friction_impulse) * a.inverse_inertia;
        }

        if (b.body_type == .dynamic) {
            b.velocity = b.velocity.add(safe_friction_impulse.scale(b.inverse_mass));
            b.angular_velocity += rb.cross(safe_friction_impulse) * b.inverse_inertia;
        }

        // Check if friction caused significant vertical velocity changes (could indicate instability)
        if (a.body_type == .dynamic and b.body_type == .dynamic) {
            // Check if signs flipped (negative to positive or vice versa) - indicates unstable friction
            const a_y_sign_changed = (a_vel_before.y < 0 and a.velocity.y > 0) or (a_vel_before.y > 0 and a.velocity.y < 0);
            const b_y_sign_changed = (b_vel_before.y < 0 and b.velocity.y > 0) or (b_vel_before.y > 0 and b.velocity.y < 0);

            if (a_y_sign_changed or b_y_sign_changed) {
                std.debug.print("🚨 UNSTABLE FRICTION DETECTED: Vertical velocity direction changed!\n", .{});
                std.debug.print("  A: y-vel {d:.6} → {d:.6}\n", .{ a_vel_before.y, a.velocity.y });
                std.debug.print("  B: y-vel {d:.6} → {d:.6}\n", .{ b_vel_before.y, b.velocity.y });
                detector.unstable_friction_count += 1;
            }
        }
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
