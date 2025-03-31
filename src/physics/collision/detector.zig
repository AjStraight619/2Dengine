const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Shape = @import("../shapes/shape.zig").Shape;
const Circle = @import("../shapes/shape.zig").Circle;
const Rectangle = @import("../shapes/shape.zig").Rectangle;
const RigidBody = @import("../body/body.zig").RigidBody;
const testCollision = @import("sat.zig").testCollision;
const SATResult = @import("sat.zig").SATResult;

/// Global counter for collision statistics
pub var total_collision_count: usize = 0;
pub var unstable_friction_count: usize = 0;
pub var significant_collision_count: usize = 0; // New counter for impacts only
pub var resting_contact_count: usize = 0; // Counter for inactive/resting contacts

/// Reference to the physics world for debug access
pub var physics_world_ref: ?*anyopaque = null;

/// A map to track recently processed collision pairs to avoid duplicates
var last_collision_pairs = std.AutoHashMap(u64, f32).init(std.heap.page_allocator);

/// Generate a unique hash for a body pair
fn generatePairHash(a: *const RigidBody, b: *const RigidBody) u64 {
    const ptr_a = @intFromPtr(a);
    const ptr_b = @intFromPtr(b);

    // Always use the same order regardless of input order
    return if (ptr_a < ptr_b)
        (ptr_a << 32) | ptr_b
    else
        (ptr_b << 32) | ptr_a;
}

/// Collision information
pub const Collision = struct {
    /// Direction of collision (points from body A to body B)
    normal: Vector2,

    /// Penetration depth
    depth: f32,

    /// Contact points in world space (up to 2 for 2D shapes)
    contact_points: [2]Vector2,

    /// Number of valid contact points
    contact_count: u8,

    /// Restitution coefficient for this collision
    restitution: f32,

    /// Friction coefficient for this collision
    friction: f32,
};

/// Collision detection system
pub const CollisionDetector = struct {
    /// Detect collision between two rigid bodies
    pub fn detectCollision(a: *RigidBody, b: *RigidBody) ?Collision {
        // First check AABBs for early out
        if (!aabbOverlap(a.aabb, b.aabb)) {
            return null;
        }

        // Check if this is a resting collision pair (recent collision with low relative velocity)
        const pair_hash = generatePairHash(a, b);
        if (last_collision_pairs.get(pair_hash)) |last_time| {
            // Get current time approximation (we don't have actual time, so use a counter)
            const current_time: f32 = @floatFromInt(total_collision_count % 1000);

            // If we processed this pair very recently, check relative velocity
            if (@abs(current_time - last_time) < 10.0) {
                // Skip if both bodies are sleeping
                if (a.is_sleeping and b.is_sleeping) {
                    resting_contact_count += 1;
                    return null;
                }

                // Calculate relative velocity
                const rel_vel = if (a.body_type == .dynamic and b.body_type == .dynamic)
                    b.velocity.sub(a.velocity)
                else if (a.body_type == .dynamic)
                    a.velocity.scale(-1)
                else
                    b.velocity;

                // Skip if relative velocity is very small (objects at rest)
                if (rel_vel.lengthSquared() < 0.1) {
                    resting_contact_count += 1;
                    return null;
                }
            }
        }

        // Perform detailed collision detection using SAT
        const sat_result = testCollision(a, b);

        // If no collision detected, return null
        if (!sat_result.collision) {
            return null;
        }

        // Skip collisions with extremely tiny penetration - likely numerical jitter
        const min_penetration_depth: f32 = 0.002;
        if (sat_result.depth < min_penetration_depth) {
            return null;
        }

        // Check if both bodies have nearly zero velocity - possible resting contact
        const is_a_nearly_stationary = a.body_type != .dynamic or a.velocity.lengthSquared() < 0.01;
        const is_b_nearly_stationary = b.body_type != .dynamic or b.velocity.lengthSquared() < 0.01;

        // For horizontal wall collisions, use a higher velocity threshold since we want to
        // detect when objects have come to rest against a wall
        const is_horizontal_collision = @abs(sat_result.mtv.x) > 0.9;
        if (is_horizontal_collision) {
            // For horizontal collisions, specifically check horizontal velocity
            if (is_a_nearly_stationary and is_b_nearly_stationary) {
                // If the relative horizontal velocity is very small, consider them at rest
                const rel_horiz_vel = if (a.body_type == .dynamic and b.body_type == .dynamic)
                    @abs(b.velocity.x - a.velocity.x)
                else if (a.body_type == .dynamic)
                    @abs(a.velocity.x)
                else
                    @abs(b.velocity.x);

                // If horizontal velocity is very small, increment resting counters
                if (rel_horiz_vel < 0.1) { // Increased sensitivity for wall detection
                    // If this is a very small penetration, just skip it
                    if (sat_result.depth < 0.1) { // Higher threshold for horizontal collisions
                        resting_contact_count += 1;

                        // Put bodies to sleep if they've been nearly stationary for multiple frames
                        if (a.body_type == .dynamic) {
                            a.low_velocity_frames += 2; // Increment faster for wall contacts
                            if (a.low_velocity_frames > 5) {
                                a.is_sleeping = true;
                                a.velocity.x = 0;
                                std.debug.print("WALL RESTING: Object put to sleep\n", .{});
                            }
                        }

                        if (b.body_type == .dynamic) {
                            b.low_velocity_frames += 2; // Increment faster for wall contacts
                            if (b.low_velocity_frames > 5) {
                                b.is_sleeping = true;
                                b.velocity.x = 0;
                                std.debug.print("WALL RESTING: Object put to sleep\n", .{});
                            }
                        }

                        return null;
                    }
                }
            }

            // Special case: any sleeping object near a wall with tiny velocity should stay sleeping
            const a_is_sleeping_dynamic = a.is_sleeping and a.inverse_mass > 0;
            const b_is_sleeping_dynamic = b.is_sleeping and b.inverse_mass > 0;

            if (a_is_sleeping_dynamic or b_is_sleeping_dynamic) {
                // If penetration is small enough, don't wake them up
                if (sat_result.depth < 0.2) {
                    std.debug.print("SLEEPING AT WALL: Keeping objects asleep despite small overlap\n", .{});
                    return null;
                }
            }
        }

        // Increment global collision counter for statistics
        total_collision_count += 1;

        // Calculate the relative velocity for impact detection
        const relative_velocity = if (a.body_type == .dynamic and b.body_type == .dynamic)
            b.velocity.sub(a.velocity)
        else if (a.body_type == .dynamic)
            a.velocity.scale(-1)
        else
            b.velocity;

        // Project relative velocity onto collision normal
        const normal_velocity = relative_velocity.dot(sat_result.mtv);

        // Wake up sleeping bodies only if they're experiencing a significant collision
        if (a.is_sleeping and a.body_type == .dynamic) {
            // Wake up with velocity if the impact is significant
            if (normal_velocity < -1.0) {
                // Stronger collisions give the body some velocity
                a.wakeUpWithMinVelocity(0.5);
                std.debug.print("WAKING BODY WITH VELOCITY - impact strength: {d:.2}\n", .{-normal_velocity});
            } else {
                // Minor collisions just wake it up
                a.wakeUp();
            }
        }

        if (b.is_sleeping and b.body_type == .dynamic) {
            // Wake up with velocity if the impact is significant
            if (normal_velocity < -1.0) {
                // Stronger collisions give the body some velocity
                b.wakeUpWithMinVelocity(0.5);
                std.debug.print("WAKING BODY WITH VELOCITY - impact strength: {d:.2}\n", .{-normal_velocity});
            } else {
                // Minor collisions just wake it up
                b.wakeUp();
            }
        }

        // Record this collision pair to avoid redundant processing
        const current_time: f32 = @floatFromInt(total_collision_count % 1000);
        last_collision_pairs.put(pair_hash, current_time) catch {};

        // Only count as significant collision if objects are moving toward each other
        // with some minimum velocity (actual impacts, not resting contact)
        const significant_velocity_threshold = 5.0; // Increased from 1.0 to only count real impacts
        if (normal_velocity < -significant_velocity_threshold) {
            significant_collision_count += 1;
        }

        // Calculate material properties for the collision
        const materials = calculateMaterialProperties(a, b);

        // Convert SAT result to Collision format
        return Collision{
            .normal = sat_result.mtv,
            .depth = sat_result.depth,
            .contact_points = sat_result.contact_points,
            .contact_count = sat_result.contact_count,
            .restitution = materials.restitution,
            .friction = materials.friction,
        };
    }

    /// Check if two AABBs overlap (for broadphase)
    fn aabbOverlap(a: anytype, b: anytype) bool {
        return !(a.max_x < b.min_x or
            a.min_x > b.max_x or
            a.max_y < b.min_y or
            a.min_y > b.max_y);
    }

    /// Calculate material properties for collision response
    fn calculateMaterialProperties(a: *const RigidBody, b: *const RigidBody) struct { restitution: f32, friction: f32 } {
        return .{
            // Use the lower of the two restitution values for a more stable simulation
            .restitution = @min(a.restitution, b.restitution),

            // Use the geometric mean for friction
            .friction = @sqrt(a.friction * b.friction),
        };
    }
};

// For backward compatibility
pub const Detector = CollisionDetector;
