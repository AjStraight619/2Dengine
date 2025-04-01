const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const BodyType = @import("../body/body.zig").BodyType;
const PhysicsWorld = @import("../world.zig").PhysicsWorld;

/// Sleep management system for physics engines
pub const SleepManager = struct {
    /// Force sleep for nearly-at-rest objects to prevent perpetual wake-up chain reactions
    pub fn forceCollectiveSleep(world: *PhysicsWorld) void {
        // First, count how many dynamic bodies are awake with very low velocity
        var count_low_vel_awake: usize = 0;
        var count_dynamic: usize = 0;

        const velocity_threshold = 0.1; // Threshold for "barely moving"

        for (world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                count_dynamic += 1;

                // Count bodies that are awake but with negligible velocity
                if (!body.is_sleeping and body.velocity.lengthSquared() < velocity_threshold * velocity_threshold) {
                    count_low_vel_awake += 1;
                }
            }
        }

        // If more than 3 objects are in this barely-moving state, force them all to sleep
        if (count_low_vel_awake >= 3 and count_low_vel_awake >= count_dynamic / 2) {
            std.debug.print("COLLECTIVE SLEEP ENFORCED: {d} barely-moving objects put to sleep\n", .{count_low_vel_awake});

            for (world.bodies.items) |body| {
                if (body.body_type == .dynamic and
                    !body.is_sleeping and
                    body.velocity.lengthSquared() < velocity_threshold * velocity_threshold)
                {
                    // Force sleep
                    body.is_sleeping = true;
                    body.velocity = Vector2.zero();
                    body.angular_velocity = 0.0;
                    body.low_velocity_frames = 20; // High value to prevent immediate wake-up
                    body.sleep_time = 2.0;
                }
            }
        }
    }

    /// Monitor pair sleeping for objects in contact
    pub fn handleContactSleeping(a: *RigidBody, b: *RigidBody) void {
        // Only handle dynamic-dynamic interactions
        if (a.body_type != .dynamic or b.body_type != .dynamic) return;

        const sleep_velocity_threshold: f32 = 0.05;
        const a_low_vel = a.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold;
        const b_low_vel = b.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold;

        // Both objects have low velocity - increase their sleep counters
        if (a_low_vel and b_low_vel) {
            // Increment both counters
            a.low_velocity_frames += 1;
            b.low_velocity_frames += 1;

            // If either object has enough frames to sleep, make BOTH sleep
            if (a.low_velocity_frames > 8 or b.low_velocity_frames > 8) { // Lower threshold for contacting objects
                if (a.is_sleeping or b.is_sleeping) {
                    // If either is already sleeping, make both sleep to prevent wake-ups
                    a.is_sleeping = true;
                    b.is_sleeping = true;
                    a.velocity = Vector2.zero();
                    b.velocity = Vector2.zero();
                    std.debug.print("CONTACT SLEEP: Both objects put to sleep (one was already sleeping)\n", .{});
                } else {
                    // Normal case - both should sleep now
                    a.is_sleeping = true;
                    b.is_sleeping = true;
                    a.velocity = Vector2.zero();
                    b.velocity = Vector2.zero();
                    std.debug.print("CONTACT SLEEP: Both objects put to sleep\n", .{});
                }
            }
        } else {
            // Only wake up if a substantial velocity is detected
            const wake_velocity_threshold = 0.15; // 3x higher than sleep threshold

            // If either has significant velocity, wake up both
            const a_significant_vel = a.velocity.lengthSquared() > wake_velocity_threshold * wake_velocity_threshold;
            const b_significant_vel = b.velocity.lengthSquared() > wake_velocity_threshold * wake_velocity_threshold;

            if (a_significant_vel or b_significant_vel) {
                a.low_velocity_frames = 0;
                b.low_velocity_frames = 0;
                // Only wake up if there's substantial movement, otherwise leave sleeping objects asleep
                if (a.is_sleeping and a_significant_vel) a.is_sleeping = false;
                if (b.is_sleeping and b_significant_vel) b.is_sleeping = false;
            } else {
                // Neither has significant velocity - leave sleeping objects asleep
                std.debug.print("PRESERVING SLEEP: Objects near each other with small velocities\n", .{});
            }
        }
    }

    /// Handle wall collision sleep behavior
    pub fn handleWallCollisionSleep(body: *RigidBody, is_wall_collision: bool) void {
        const sleep_velocity_threshold: f32 = 0.05;

        if (body.body_type != .dynamic) return;

        if (body.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold) {
            // Wall collisions make objects sleep faster
            if (is_wall_collision) {
                body.low_velocity_frames += 3; // Triple increment for wall contacts
                std.debug.print("Using horizontal wall collision with higher bias\n", .{});

                // Even faster sleep for objects right against a wall
                if (body.velocity.lengthSquared() < 0.01 * 0.01) {
                    body.low_velocity_frames += 3;
                    // Immediately zero horizontal velocity for wall contact
                    if (is_wall_collision) {
                        body.velocity.x = 0;

                        // Force to sleep immediately if velocity is essentially zero
                        if (body.velocity.lengthSquared() < 0.001) {
                            body.is_sleeping = true;
                            body.velocity = Vector2.zero();
                            std.debug.print("IMMEDIATE WALL SLEEP: Object velocity essentially zero\n", .{});
                        }
                    }
                }
            } else {
                body.low_velocity_frames += 1;
            }

            if (body.low_velocity_frames > 5 and is_wall_collision) {
                body.is_sleeping = true;
                body.velocity = Vector2.zero();
                std.debug.print("WALL RESTING: Object put to sleep\n", .{});
            } else if (body.low_velocity_frames > 10) {
                body.is_sleeping = true;
                body.velocity = Vector2.zero();
            }
        } else {
            body.low_velocity_frames = 0;
        }
    }
};
