const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;

/// Physics stability safeguards to prevent simulation blowups
pub const StabilitySafeguards = struct {
    /// Apply emergency velocity cap to prevent simulation explosions
    pub fn applyVelocityCaps(a: *RigidBody, b: *RigidBody) void {
        const emergency_velocity_cap = 50.0;

        // Cap body A velocity if needed
        if (a.velocity.length() > emergency_velocity_cap) {
            const original_vel = a.velocity;
            a.velocity = a.velocity.scale(emergency_velocity_cap / a.velocity.length());
            std.debug.print("⚠️ EMERGENCY VELOCITY CAP: Object A vel={d:.1} capped to {d:.1}\n", .{ original_vel.length(), a.velocity.length() });
        }

        // Cap body B velocity if needed
        if (b.velocity.length() > emergency_velocity_cap) {
            const original_vel = b.velocity;
            b.velocity = b.velocity.scale(emergency_velocity_cap / b.velocity.length());
            std.debug.print("⚠️ EMERGENCY VELOCITY CAP: Object B vel={d:.1} capped to {d:.1}\n", .{ original_vel.length(), b.velocity.length() });
        }
    }

    /// Check for extreme collision condition and log a warning
    pub fn checkExtremeCollision(a: *const RigidBody, b: *const RigidBody, depth: f32) bool {
        const velocity_threshold: f32 = 500.0;
        const depth_threshold: f32 = 10.0;

        const a_velocity = a.velocity.length();
        const b_velocity = b.velocity.length();

        const extreme_velocity = (a_velocity > velocity_threshold) or
            (b_velocity > velocity_threshold);

        const extreme_penetration = depth > depth_threshold;

        if (extreme_velocity or extreme_penetration) {
            std.debug.print("🚨 EXTREME VELOCITY DURING RESOLUTION:\n", .{});
            std.debug.print("  A: vel={d:.2} at ({d:.2},{d:.2})\n", .{ a_velocity, a.position.x, a.position.y });
            std.debug.print("  B: vel={d:.2} at ({d:.2},{d:.2})\n", .{ b_velocity, b.position.x, b.position.y });
            std.debug.print("  Normal: ({d:.2},{d:.2}), depth: {d:.2}\n", .{ a.velocity.x, a.velocity.y, depth });
            return true;
        }

        return false;
    }

    /// Prevent infinite velocities by validating values
    pub fn sanitizeVelocity(body: *RigidBody) void {
        // Check for NaN or infinity
        if (std.math.isNan(body.velocity.x) or std.math.isInf(body.velocity.x)) {
            body.velocity.x = 0;
        }
        if (std.math.isNan(body.velocity.y) or std.math.isInf(body.velocity.y)) {
            body.velocity.y = 0;
        }
    }
};
