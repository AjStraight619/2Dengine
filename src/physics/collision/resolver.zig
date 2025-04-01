const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const Collision = @import("detector.zig").Collision;
const std = @import("std");
const detector = @import("detector.zig");
const CollisionPhysics = @import("physics_equations.zig").CollisionPhysics;
const logger = @import("../logger.zig");

/// Constants used throughout the collision resolver
const Constants = struct {
    // Velocity thresholds
    pub const EXTREME_VELOCITY_THRESHOLD: f32 = 500.0;
    pub const VELOCITY_THRESHOLD_FOR_DAMPING: f32 = 10.0;
    pub const LOW_VELOCITY_THRESHOLD: f32 = 0.05;
    pub const VERY_LOW_VELOCITY_THRESHOLD: f32 = 0.03;
    pub const SLOW_VELOCITY_THRESHOLD: f32 = 0.2;
    pub const TINY_VELOCITY_THRESHOLD: f32 = 0.01;
    pub const WALL_VELOCITY_THRESHOLD: f32 = 5.0;

    // Collision detection thresholds
    pub const VERTICAL_NORMAL_THRESHOLD: f32 = 0.9;
    pub const HORIZONTAL_NORMAL_THRESHOLD: f32 = 0.9;
    pub const WALL_NORMAL_THRESHOLD: f32 = 0.95;

    // Position correction factors
    pub const DEFAULT_CORRECTION_FACTOR: f32 = 1.01;
    pub const HORIZONTAL_CORRECTION_FACTOR: f32 = 1.05;
    pub const GROUND_CORRECTION_FACTOR: f32 = 1.1;

    // Separation tolerance values
    pub const DEFAULT_SEPARATION_TOLERANCE: f32 = 0.001;
    pub const HORIZONTAL_SEPARATION_TOLERANCE: f32 = 0.05;
    pub const VERTICAL_SEPARATION_TOLERANCE: f32 = 0.1;

    // Impulse related
    pub const MAXIMUM_IMPULSE: f32 = 500.0;
    pub const DIRECT_FRICTION_FACTOR: f32 = -0.05;
    pub const FRICTION_AMPLIFIER: f32 = 2.0;
    pub const IMPULSE_CLAMP_FACTOR: f32 = 1.5;

    // Sleep related
    pub const SLEEP_VELOCITY_THRESHOLD: f32 = 0.05;
    pub const WALL_SLEEP_FRAME_INCREMENT: i32 = 2;
    pub const WALL_SLEEP_THRESHOLD: i32 = 6;
    pub const GENERAL_SLEEP_THRESHOLD: i32 = 10;
    pub const DYNAMIC_PAIR_SLEEP_THRESHOLD: i32 = 8;
};

/// Collision resolution system
pub const CollisionResolver = struct {
    /// Resolve a collision between two bodies
    pub fn resolveCollision(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip resolution if both bodies are static (not just either one)
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        logger.info(.collision, ">>> RESOLVING COLLISION <<<", .{});
        logger.info(.collision, "A: type={}, static={}, pos=({d},{d}), vel=({d},{d})", .{ a.shape, a.body_type == .static, a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        logger.info(.collision, "B: type={}, static={}, pos=({d},{d}), vel=({d},{d})", .{ b.shape, b.body_type == .static, b.position.x, b.position.y, b.velocity.x, b.velocity.y });
        logger.info(.collision, "Collision normal=({d},{d}) - THIS POINTS FROM A TO B", .{ collision.normal.x, collision.normal.y });

        // Position correction (prevents sinking)
        resolvePositions(a, b, collision);

        // Impulse resolution (bounce and friction)
        resolveVelocities(a, b, collision);

        logger.info(.collision, "After resolution - A: pos=({d},{d}), vel=({d},{d})", .{ a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        logger.info(.collision, "After resolution - B: pos=({d},{d}), vel=({d},{d})", .{ b.position.x, b.position.y, b.velocity.x, b.velocity.y });
    }

    /// Resolve positions to prevent objects from sinking into each other
    fn resolvePositions(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip if both bodies are static
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        logger.info(.collision, ">>> POSITION CORRECTION <<<", .{});
        logger.info(.collision, "A body type: {s}, B body type: {s}", .{ @tagName(a.body_type), @tagName(b.body_type) });
        logger.info(.collision, "Current A pos: ({d:.4},{d:.4}), B pos: ({d:.4},{d:.4})", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        logger.info(.collision, "Collision normal: ({d:.4},{d:.4}), depth: {d:.4}", .{ collision.normal.x, collision.normal.y, collision.depth });

        // Check if this is a horizontal collision between rectangles
        const is_horizontal = @abs(collision.normal.x) > Constants.HORIZONTAL_NORMAL_THRESHOLD;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // Check for vertical collision with ground
        const is_vertical = @abs(collision.normal.y) > Constants.VERTICAL_NORMAL_THRESHOLD;
        const is_ground_collision = is_vertical and collision.normal.y < 0;
        const vertical_rect_ground_collision = is_ground_collision and is_rect_vs_rect;

        // Use different correction factors based on collision orientation and types
        var correction_factor: f32 = Constants.DEFAULT_CORRECTION_FACTOR; // Default small bias

        // For horizontal rect-rect collisions, use stable correction
        if (horizontal_rect_collision) {
            correction_factor = Constants.HORIZONTAL_CORRECTION_FACTOR;
            logger.info(.collision, "HORIZONTAL rect-rect collision - using moderate position correction: {d}", .{correction_factor});
        }
        // For ground collisions (normal pointing up), use stronger correction
        else if (vertical_rect_ground_collision) {
            correction_factor = Constants.GROUND_CORRECTION_FACTOR;
            logger.info(.collision, "GROUND collision - using stronger position correction: {d}", .{correction_factor});
        }

        const correction = CollisionPhysics.calculatePositionCorrection(collision.normal, collision.depth, correction_factor);
        logger.info(.collision, "Correction vector: ({d:.4},{d:.4})", .{ correction.x, correction.y });

        // Apply the correction based on mass properties
        if (a.body_type == .static) {
            // Only push body B (static A can't move)
            b.position = b.position.add(correction);
            logger.info(.collision, "Static A: moved B to ({d:.4},{d:.4})", .{ b.position.x, b.position.y });
        } else if (b.body_type == .static) {
            // Only push body A (static B can't move)
            a.position = a.position.sub(correction);
            logger.info(.collision, "Static B: moved A to ({d:.4},{d:.4})", .{ a.position.x, a.position.y });
        } else {
            // Push both bodies based on their relative masses
            const total_inv_mass = a.inverse_mass + b.inverse_mass;

            if (total_inv_mass <= 0.0001) return;

            const a_ratio = CollisionPhysics.calculateCorrectionRatio(a.inverse_mass, b.inverse_mass);
            const b_ratio = CollisionPhysics.calculateCorrectionRatio(b.inverse_mass, a.inverse_mass);

            logger.info(.collision, "Dynamic-Dynamic: A ratio: {d:.4}, B ratio: {d:.4}", .{ a_ratio, b_ratio });

            a.position = a.position.sub(correction.scale(a_ratio));
            b.position = b.position.add(correction.scale(b_ratio));
            logger.info(.collision, "Moved both: A to ({d:.4},{d:.4}), B to ({d:.4},{d:.4})", .{ a.position.x, a.position.y, b.position.x, b.position.y });
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

        // 1. Initial checks and setup
        checkForExtremeVelocities(a, b);
        logger.info(.collision, ">>> VELOCITY RESOLUTION <<<", .{});
        logger.info(.collision, "Before: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // 2. Detect collision types based on normal direction
        const collision_context = detectCollisionType(collision, a, b);

        // 3. Handle jitter debugging for vertical rectangle collisions
        if (collision_context.vertical_rect_collision) {
            detectAndLogPotentialJitter(b);
        }

        // 4. Calculate collision properties (contact point, relative velocity, etc.)
        var collision_data = calculateCollisionData(a, b, collision);

        // 5. Check if objects are separating (skip impulse for separating objects)
        if (shouldSkipImpulseForSeparatingObjects(collision_data, collision_context)) {
            return;
        }

        // 6. Calculate and apply normal impulse
        applyNormalImpulse(a, b, collision, &collision_data, collision_context);

        // 7. Apply friction
        applyFriction(a, b, collision, collision_data.relative_velocity, collision_data.normal_velocity, collision_data.impulse_magnitude, collision_data.ra, collision_data.rb, collision_context.horizontal_rect_collision);

        // 8. Handle sleep state for objects with very low velocities
        updateSleepStates(a, b, collision, collision_context);

        // 9. Log final velocities
        logger.info(.collision, "Final velocities: A=({d:.6},{d:.6}), B=({d:.6},{d:.6})", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });
    }

    /// Structure to hold information about the type of collision
    const CollisionContext = struct {
        is_vertical: bool, // True if normal is mostly vertical
        is_horizontal: bool, // True if normal is mostly horizontal
        vertical_rect_collision: bool, // Vertical collision between rectangles
        horizontal_rect_collision: bool, // Horizontal collision between rectangles
        is_wall_collision: bool, // Collision with a wall (horizontal normal)
        is_ground_collision: bool, // Collision with ground (normal pointing up)
    };

    /// Structure to hold calculated collision data
    const CollisionData = struct {
        ra: Vector2, // Vector from A's center to contact point
        rb: Vector2, // Vector from B's center to contact point
        relative_velocity: Vector2, // Relative velocity at contact point
        normal_velocity: f32, // Velocity along the normal direction
        inv_mass_sum: f32, // Sum of inverse masses with rotational factors
        restitution: f32, // Effective restitution for this collision
        impulse_magnitude: f32, // Calculated impulse magnitude
    };

    /// Check for extreme velocities and log them
    fn checkForExtremeVelocities(a: *RigidBody, b: *RigidBody) void {
        const vel_a_len = a.velocity.length();
        const vel_b_len = b.velocity.length();

        if (vel_a_len > Constants.EXTREME_VELOCITY_THRESHOLD or vel_b_len > Constants.EXTREME_VELOCITY_THRESHOLD) {
            logger.warning(.stability, "EXTREME VELOCITY DURING RESOLUTION:", .{});
            logger.warning(.stability, "  A: vel={d:.2} at ({d:.2},{d:.2})", .{ vel_a_len, a.position.x, a.position.y });
            logger.warning(.stability, "  B: vel={d:.2} at ({d:.2},{d:.2})", .{ vel_b_len, b.position.x, b.position.y });
            logger.warning(.stability, "  Normal: ({d:.2},{d:.2})", .{ a.velocity.x, a.velocity.y });
        }
    }

    /// Detect the type of collision based on normal direction and shapes
    fn detectCollisionType(collision: Collision, a: *RigidBody, b: *RigidBody) CollisionContext {
        const is_vertical = @abs(collision.normal.y) > Constants.VERTICAL_NORMAL_THRESHOLD;
        const is_horizontal = @abs(collision.normal.x) > Constants.HORIZONTAL_NORMAL_THRESHOLD;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;

        const vertical_rect_collision = is_vertical and is_rect_vs_rect;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;
        const is_wall_collision = @abs(collision.normal.x) > Constants.WALL_NORMAL_THRESHOLD;
        const is_ground_collision = is_vertical and collision.normal.y < 0;

        return CollisionContext{
            .is_vertical = is_vertical,
            .is_horizontal = is_horizontal,
            .vertical_rect_collision = vertical_rect_collision,
            .horizontal_rect_collision = horizontal_rect_collision,
            .is_wall_collision = is_wall_collision,
            .is_ground_collision = is_ground_collision,
        };
    }

    /// Log details for potential jitter in vertical rect collisions
    fn detectAndLogPotentialJitter(b: *RigidBody) void {
        const velocity_near_zero = @abs(b.velocity.y) < 0.1;
        if (velocity_near_zero) {
            logger.debug(.stability, "JITTER-DEBUG: Near-rest vertical rect collision detected!", .{});
            logger.debug(.stability, "JITTER-DEBUG: Velocity before resolution: {d:.8}", .{b.velocity.y});
            logger.debug(.stability, "JITTER-DEBUG: Gravity applied this frame: {d:.8}", .{b.force.y});
        }
    }

    /// Calculate collision related data needed for impulse resolution
    fn calculateCollisionData(a: *RigidBody, b: *RigidBody, collision: Collision) CollisionData {
        // Get the contact point
        const contact = collision.contact_points[0];
        logger.info(.collision, "Contact point: ({d:.4},{d:.4})", .{ contact.x, contact.y });

        // Calculate relative vectors from centers to contact point
        const ra = contact.sub(a.position);
        const rb = contact.sub(b.position);

        // Calculate relative velocity at contact point
        const relative_velocity = CollisionPhysics.calculateRelativeVelocity(a.velocity, b.velocity, a.angular_velocity, b.angular_velocity, ra, rb);
        logger.info(.collision, "Relative velocity: ({d:.4},{d:.4})", .{ relative_velocity.x, relative_velocity.y });

        // Project relative velocity onto the collision normal
        const normal_velocity = relative_velocity.dot(collision.normal);
        logger.info(.collision, "Relative velocity along normal: {d:.4}", .{normal_velocity});

        // Calculate impulse factors
        const ra_cross_n = ra.cross(collision.normal);
        const rb_cross_n = rb.cross(collision.normal);

        const inv_mass_sum = a.inverse_mass + b.inverse_mass +
            (ra_cross_n * ra_cross_n) * a.inverse_inertia +
            (rb_cross_n * rb_cross_n) * b.inverse_inertia;

        // Just create with default values that will be filled in later
        return CollisionData{
            .ra = ra,
            .rb = rb,
            .relative_velocity = relative_velocity,
            .normal_velocity = normal_velocity,
            .inv_mass_sum = inv_mass_sum,
            .restitution = collision.restitution,
            .impulse_magnitude = 0.0,
        };
    }

    /// Determine if we should skip impulse resolution for separating objects
    fn shouldSkipImpulseForSeparatingObjects(collision_data: CollisionData, context: CollisionContext) bool {
        if (collision_data.inv_mass_sum <= 0.0001) return true;

        // Better separation handling - only skip if objects are DEFINITELY separating
        // Increase separation tolerance for horizontal collisions to prevent sticking
        var separation_tolerance: f32 = Constants.DEFAULT_SEPARATION_TOLERANCE;

        // For horizontal wall collisions, we use higher tolerance
        if (context.horizontal_rect_collision) {
            // Much higher tolerance for horizontal rectangle collisions
            separation_tolerance = Constants.HORIZONTAL_SEPARATION_TOLERANCE;
        }

        // Increased resting contact detection for vertical collisions
        if (context.is_vertical) {
            // For vertical collisions, increase tolerance for resting contacts
            if (@abs(collision_data.normal_velocity) < 0.1) {
                separation_tolerance = Constants.VERTICAL_SEPARATION_TOLERANCE;
            }
        }

        if (collision_data.normal_velocity > separation_tolerance) {
            logger.info(.collision, "Objects DEFINITELY separating, skipping impulse", .{});
            return true;
        }

        return false;
    }

    /// Apply special handling for wall collisions
    fn handleWallCollision(a: *RigidBody, b: *RigidBody, collision: Collision, collision_data: CollisionData) bool {
        // Only apply this for horizontal wall collisions with low velocity
        const is_horizontal = @abs(collision.normal.x) > Constants.HORIZONTAL_NORMAL_THRESHOLD;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        if (horizontal_rect_collision) {
            // For ANY wall collision with low velocity, completely stop horizontal motion
            if (@abs(collision_data.normal_velocity) < Constants.WALL_VELOCITY_THRESHOLD) {
                logger.info(.collision, "WALL COLLISION: Complete horizontal stop applied", .{});

                // For RIGHT wall collision (normal pointing left), zero X velocity
                if (collision.normal.x < 0 and b.body_type == .dynamic) {
                    b.velocity.x = 0;
                    // Force sleeping sooner to prevent further collisions
                    b.low_velocity_frames += 8;
                    if (b.low_velocity_frames > Constants.GENERAL_SLEEP_THRESHOLD) {
                        b.is_sleeping = true;
                        logger.info(.sleep, "FORCED SLEEP: Object at wall is now sleeping", .{});
                    }
                }

                // For LEFT wall collision (normal pointing right), zero X velocity
                if (collision.normal.x > 0 and a.body_type == .dynamic) {
                    a.velocity.x = 0;
                    // Force sleeping sooner
                    a.low_velocity_frames += 8;
                    if (a.low_velocity_frames > Constants.GENERAL_SLEEP_THRESHOLD) {
                        a.is_sleeping = true;
                        logger.info(.sleep, "FORCED SLEEP: Object at wall is now sleeping", .{});
                    }
                }

                // Don't even apply any impulse for wall collisions - just stop
                if (collision_data.normal_velocity < 0) {
                    logger.warning(.collision, "WALL IMPACT: Skipping normal impulse resolution", .{});
                    return true; // Skip further impulse calculation
                }
            }
        }

        return false; // Continue with normal impulse calculation
    }

    /// Calculate and apply normal impulse for collision response
    fn applyNormalImpulse(a: *RigidBody, b: *RigidBody, collision: Collision, collision_data: *CollisionData, context: CollisionContext) void {
        // Special handling for wall collisions
        if (handleWallCollision(a, b, collision, collision_data.*)) {
            return;
        }

        // Calculate restitution based on collision type
        const restitution = adjustRestitutionForCollisionType(collision.restitution, context, collision_data.normal_velocity);

        // Calculate impulse magnitude
        var j = CollisionPhysics.calculateImpulseMagnitude(collision_data.normal_velocity, restitution, collision_data.inv_mass_sum);

        // Cap impulse to avoid numerical explosions
        if (@abs(j) > Constants.MAXIMUM_IMPULSE) {
            logger.warning(.stability, "EXCESSIVE IMPULSE CAPPED: {d:.4} → {d:.4}", .{ j, std.math.sign(j) * Constants.MAXIMUM_IMPULSE });
            j = std.math.sign(j) * Constants.MAXIMUM_IMPULSE;
        }

        logger.info(.collision, "Impulse magnitude: {d:.4}", .{j});
        collision_data.impulse_magnitude = j;
        collision_data.restitution = restitution;

        // Apply the impulse along the normal only
        const impulse = CollisionPhysics.calculateImpulseVector(collision.normal, j);
        logger.info(.collision, "Impulse vector: ({d:.4},{d:.4})", .{ impulse.x, impulse.y });

        // Log before applying impulse
        logger.info(.collision, "Before impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Store initial velocities for stability checking
        const vel_a_len = a.velocity.length();
        const vel_b_len = b.velocity.length();

        // Apply impulse
        applyImpulse(a, b, impulse, collision_data.ra, collision_data.rb);

        // Log after applying impulse
        logger.info(.collision, "After impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Check for sudden large changes in velocity - could indicate instability
        checkForVelocityInstability(a, b, vel_a_len, vel_b_len);
    }

    /// Adjust restitution coefficient based on collision type
    fn adjustRestitutionForCollisionType(base_restitution: f32, context: CollisionContext, normal_velocity: f32) f32 {
        var restitution = base_restitution;

        // For horizontal rectangles, drastically reduce restitution to prevent bouncing
        if (context.horizontal_rect_collision) {
            // ZERO restitution for horizontal collisions - no bounce at all
            restitution = 0.0;
            logger.info(.collision, "Horizontal rect-rect collision: ZERO restitution applied", .{});
        }

        // Reduce restitution for vertical collisions as well
        if (context.vertical_rect_collision) {
            // Objects resting on top of each other should have less bounce
            if (normal_velocity < 0 and @abs(normal_velocity) < 1.0) {
                restitution *= 0.06; // Reduced to 6% of original restitution
                logger.info(.collision, "Vertical rect-rect collision (top): reducing restitution to {d:.4}", .{restitution});
            }
        }

        // Create a normalized collision normal based on context
        var collision_normal = Vector2.init(0, 0);
        if (context.is_vertical) {
            // For vertical collisions
            collision_normal = Vector2.init(0, if (normal_velocity < 0) -1.0 else 1.0);
        } else if (context.is_horizontal) {
            // For horizontal collisions
            collision_normal = Vector2.init(if (normal_velocity < 0) -1.0 else 1.0, 0);
        } else {
            // Default 45 degree collision if we can't determine
            collision_normal = Vector2.init(0.7071, 0.7071);
        }

        // Adjust restitution using physics equations, passing the proper normal vector
        restitution = CollisionPhysics.adjustRestitution(collision_normal, normal_velocity, restitution);

        // Reduce bounciness for low-speed collisions (linear damping)
        if (@abs(normal_velocity) < Constants.VELOCITY_THRESHOLD_FOR_DAMPING) {
            restitution *= @abs(normal_velocity) / Constants.VELOCITY_THRESHOLD_FOR_DAMPING;
        }

        return restitution;
    }

    /// Check for potential instability in the velocity changes
    fn checkForVelocityInstability(a: *RigidBody, b: *RigidBody, old_vel_a_len: f32, old_vel_b_len: f32) void {
        const new_vel_a_len = a.velocity.length();
        const new_vel_b_len = b.velocity.length();
        const velocity_change_a = @abs(new_vel_a_len - old_vel_a_len);
        const velocity_change_b = @abs(new_vel_b_len - old_vel_b_len);

        if (velocity_change_a > 100.0 or velocity_change_b > 100.0) {
            logger.warning(.stability, "LARGE VELOCITY CHANGE DETECTED!", .{});
            logger.warning(.stability, "  A: {d:.2} → {d:.2} (change: {d:.2})", .{ old_vel_a_len, new_vel_a_len, velocity_change_a });
            logger.warning(.stability, "  B: {d:.2} → {d:.2} (change: {d:.2})", .{ old_vel_b_len, new_vel_b_len, velocity_change_b });
        }
    }

    /// Update sleep states for objects with low velocities
    fn updateSleepStates(a: *RigidBody, b: *RigidBody, collision: Collision, context: CollisionContext) void {
        _ = collision; // Unused parameter, kept for API consistency

        if (a.body_type == .dynamic and b.body_type == .dynamic) {
            updateDynamicDynamicSleepState(a, b);
        } else {
            // Handle dynamic vs static sleep states
            updateDynamicStaticSleepState(a, b, context.is_wall_collision);
        }
    }

    /// Update sleep state for two dynamic bodies in contact
    fn updateDynamicDynamicSleepState(a: *RigidBody, b: *RigidBody) void {
        // For objects in contact with each other:
        // 1. They should have similar sleep states to avoid one being awake while pushing a sleeping one
        // 2. They should have similar velocity thresholds to sleep together

        const a_low_vel = a.velocity.lengthSquared() < Constants.SLEEP_VELOCITY_THRESHOLD * Constants.SLEEP_VELOCITY_THRESHOLD;
        const b_low_vel = b.velocity.lengthSquared() < Constants.SLEEP_VELOCITY_THRESHOLD * Constants.SLEEP_VELOCITY_THRESHOLD;

        // Both objects have low velocity - increase their sleep counters
        if (a_low_vel and b_low_vel) {
            // Increment both counters
            a.low_velocity_frames += 1;
            b.low_velocity_frames += 1;

            // If either object has enough frames to sleep, make BOTH sleep
            if (a.low_velocity_frames > Constants.DYNAMIC_PAIR_SLEEP_THRESHOLD or
                b.low_velocity_frames > Constants.DYNAMIC_PAIR_SLEEP_THRESHOLD)
            {
                a.is_sleeping = true;
                b.is_sleeping = true;
                a.velocity = Vector2.zero();
                b.velocity = Vector2.zero();
                logger.info(.sleep, "CONTACT SLEEP: Both objects put to sleep", .{});
            }
        } else {
            // If either has significant velocity, wake up both
            if (!a_low_vel or !b_low_vel) {
                a.low_velocity_frames = 0;
                b.low_velocity_frames = 0;
                a.is_sleeping = false;
                b.is_sleeping = false;
            }
        }
    }

    /// Update sleep state for dynamic body against static body
    fn updateDynamicStaticSleepState(a: *RigidBody, b: *RigidBody, is_wall_collision: bool) void {
        // Process sleep state for body A if it's dynamic
        if (a.body_type == .dynamic) {
            updateSingleBodySleepState(a, is_wall_collision);
        }

        // Process sleep state for body B if it's dynamic
        if (b.body_type == .dynamic) {
            updateSingleBodySleepState(b, is_wall_collision);
        }
    }

    /// Update sleep state for a single dynamic body
    fn updateSingleBodySleepState(body: *RigidBody, is_wall_collision: bool) void {
        if (body.velocity.lengthSquared() < Constants.SLEEP_VELOCITY_THRESHOLD * Constants.SLEEP_VELOCITY_THRESHOLD) {
            // Wall collisions make objects sleep faster
            if (is_wall_collision) {
                body.low_velocity_frames += Constants.WALL_SLEEP_FRAME_INCREMENT;

                // Even faster sleep for objects right against a wall
                if (body.velocity.lengthSquared() < Constants.TINY_VELOCITY_THRESHOLD * Constants.TINY_VELOCITY_THRESHOLD) {
                    body.low_velocity_frames += Constants.WALL_SLEEP_FRAME_INCREMENT;
                    // Immediately zero horizontal velocity for wall contact
                    if (@abs(body.velocity.x) > 0.9) {
                        body.velocity.x = 0;
                    }
                }
            } else {
                body.low_velocity_frames += 1;
            }

            // Put to sleep if counter is high enough
            if (body.low_velocity_frames > Constants.WALL_SLEEP_THRESHOLD and is_wall_collision) {
                body.is_sleeping = true;
                body.velocity = Vector2.zero();
                logger.info(.sleep, "WALL RESTING: Object put to sleep", .{});
            } else if (body.low_velocity_frames > Constants.GENERAL_SLEEP_THRESHOLD) {
                body.is_sleeping = true;
                body.velocity = Vector2.zero();
            }
        } else {
            body.low_velocity_frames = 0;
        }
    }

    /// Apply friction impulse
    fn applyFriction(a: *RigidBody, b: *RigidBody, collision: Collision, relative_velocity: Vector2, normal_velocity: f32, normal_impulse: f32, ra: Vector2, rb: Vector2, is_horizontal_rect_collision: bool) void {
        _ = is_horizontal_rect_collision; // Parameter still needed for API compatibility

        // Check for vertical collision condition (for resting objects)
        const is_vertical = @abs(collision.normal.y) > Constants.VERTICAL_NORMAL_THRESHOLD;
        const is_ground_collision = is_vertical and collision.normal.y < 0;

        // Track if we zeroed velocity due to very low speed
        var velocity_zeroed = false;

        // IMPROVEMENT: Stop objects with very low horizontal velocity
        // This helps prevent objects from sliding indefinitely at low speeds
        if (is_ground_collision) {
            if (a.body_type == .dynamic and @abs(a.velocity.x) < Constants.VERY_LOW_VELOCITY_THRESHOLD) {
                a.velocity.x = 0.0;
                a.low_velocity_frames += 1;
                velocity_zeroed = true;
                logger.info(.collision, "Small velocity zeroed on ground (A): {d:.6} -> 0.0", .{a.velocity.x});
            }

            if (b.body_type == .dynamic and @abs(b.velocity.x) < Constants.VERY_LOW_VELOCITY_THRESHOLD) {
                b.velocity.x = 0.0;
                b.low_velocity_frames += 1;
                velocity_zeroed = true;
                logger.info(.collision, "Small velocity zeroed on ground (B): {d:.6} -> 0.0", .{b.velocity.x});
            }
        }

        // Exit early if we've already zeroed the velocities
        if (velocity_zeroed and @abs(relative_velocity.x) < Constants.VERY_LOW_VELOCITY_THRESHOLD) {
            return;
        }

        // Calculate tangent vector (perpendicular to normal)
        var tangent = CollisionPhysics.calculateFrictionTangent(relative_velocity, collision.normal, normal_velocity);
        const tangent_length = tangent.length();

        // Only normalize if tangent has meaningful length
        if (tangent_length > 0.0001) {
            tangent = CollisionPhysics.normalizeTangentVector(tangent);
        } else {
            // For objects resting (almost zero tangent vel), assume horizontal friction direction
            if (is_ground_collision) {
                // Create a horizontal friction direction for rolling objects
                tangent = Vector2.init(1.0, 0.0);
                logger.info(.collision, "GROUND CONTACT: Using horizontal friction for rolling objects", .{});
            } else {
                // For other cases, try to get a perpendicular vector to the normal
                if (@abs(collision.normal.x) > @abs(collision.normal.y)) {
                    tangent = Vector2.init(0.0, 1.0);
                } else {
                    tangent = Vector2.init(1.0, 0.0);
                }
            }
        }

        // Calculate friction impulse factor
        // Coulomb's Law of Friction (max_friction = mu * normal_force)
        var friction_coef = CollisionPhysics.calculateFrictionCoefficient(a.friction, b.friction);

        // IMPROVEMENT: Increase rolling friction coefficients to slow objects down faster
        // Special case: Increase friction for ground contacts to properly handle rolling
        if (is_ground_collision) {
            // Both static and kinetic friction apply to rolling objects on ground
            if (tangent_length < 0.2) {
                // Static friction (rolling resistance) - STRONGER than before
                friction_coef *= Constants.FRICTION_AMPLIFIER;
                logger.info(.collision, "INCREASED ROLLING RESISTANCE: Applying stronger static friction (coef={d:.4})", .{friction_coef});
            }

            // FRICTION FIX: Apply direct horizontal damping for ground contacts
            // This directly reduces horizontal velocity proportional to friction coefficient
            if (b.body_type == .dynamic and @abs(b.velocity.x) > Constants.VERY_LOW_VELOCITY_THRESHOLD) {
                // Calculate a direct horizontal friction impulse
                const direct_friction = b.velocity.x * b.mass * Constants.DIRECT_FRICTION_FACTOR * friction_coef;

                // Apply the friction directly to the horizontal velocity
                b.velocity.x += direct_friction * b.inverse_mass;
            }

            if (a.body_type == .dynamic and @abs(a.velocity.x) > Constants.VERY_LOW_VELOCITY_THRESHOLD) {
                // Calculate a direct horizontal friction impulse
                const direct_friction = a.velocity.x * a.mass * Constants.DIRECT_FRICTION_FACTOR * friction_coef;

                // Apply the friction directly to the horizontal velocity
                a.velocity.x += direct_friction * a.inverse_mass;
            }
        }

        // Calculate friction impulse magnitude (limited by normal impulse)
        var j_t = -(relative_velocity.dot(tangent)) / (a.inverse_mass + b.inverse_mass +
            (ra.cross(tangent) * ra.cross(tangent)) * a.inverse_inertia +
            (rb.cross(tangent) * rb.cross(tangent)) * b.inverse_inertia);

        // IMPROVEMENT: For very slow objects, calculate exact friction needed to stop them
        // Ensure minimum friction for very slow-moving objects on ground
        if (is_ground_collision) {
            if (a.body_type == .dynamic and @abs(a.velocity.x) > 0.0 and @abs(a.velocity.x) < Constants.SLOW_VELOCITY_THRESHOLD) {
                // Calculate the impulse needed to stop A completely
                const impulse_to_stop_a = -a.velocity.x * a.mass;
                const original_j_t = j_t;
                j_t = std.math.clamp(j_t, -@abs(impulse_to_stop_a) * Constants.IMPULSE_CLAMP_FACTOR, @abs(impulse_to_stop_a) * Constants.IMPULSE_CLAMP_FACTOR);
                logger.info(.collision, "STOPPING FRICTION: Object A vel={d:.6}, using j_t={d:.6} (was {d:.6})", .{ a.velocity.x, j_t, original_j_t });
            }

            if (b.body_type == .dynamic and @abs(b.velocity.x) > 0.0 and @abs(b.velocity.x) < Constants.SLOW_VELOCITY_THRESHOLD) {
                // Calculate the impulse needed to stop B completely
                const impulse_to_stop_b = -b.velocity.x * b.mass;
                const original_j_t = j_t;
                j_t = std.math.clamp(j_t, -@abs(impulse_to_stop_b) * Constants.IMPULSE_CLAMP_FACTOR, @abs(impulse_to_stop_b) * Constants.IMPULSE_CLAMP_FACTOR);
                logger.info(.collision, "STOPPING FRICTION: Object B vel={d:.6}, using j_t={d:.6} (was {d:.6})", .{ b.velocity.x, j_t, original_j_t });
            }
        }

        // Clamp friction impulse magnitude
        j_t = CollisionPhysics.clampFrictionImpulse(j_t, normal_impulse, friction_coef);

        // Calculate friction impulse vector
        const friction_impulse = tangent.scale(j_t);

        // Store initial velocities for debugging
        const a_vel_before = a.velocity;
        const b_vel_before = b.velocity;

        // CRITICAL FIX: Don't apply friction in the y-direction if it would cause the object to move upward
        var safe_friction_impulse = friction_impulse;

        // For objects on the ground (vertical collision with normal pointing up)
        if (is_vertical and collision.normal.y < 0) {
            // If friction would cause object to move upward, zero out the y component
            if ((a.body_type == .dynamic and (a.velocity.y < 0 and friction_impulse.y * a.inverse_mass > @abs(a.velocity.y))) or
                (b.body_type == .dynamic and (b.velocity.y < 0 and friction_impulse.y * b.inverse_mass < -@abs(b.velocity.y))))
            {
                logger.warning(.collision, "PREVENTING INVALID UPWARD FRICTION!", .{});
                safe_friction_impulse.y = 0;
            }
        }

        // Apply friction impulse
        if (a.body_type == .dynamic) {
            a.velocity = a.velocity.add(safe_friction_impulse.scale(-a.inverse_mass));
            a.angular_velocity -= ra.cross(safe_friction_impulse) * a.inverse_inertia;

            // IMPROVEMENT: Prevent velocity sign change due to friction (friction shouldn't reverse movement)
            if (a_vel_before.x > 0 and a.velocity.x < 0) {
                a.velocity.x = 0.0;
                logger.info(.collision, "PREVENTING FRICTION OVERSHOOT: A x-vel {d:.6} → 0.0 (was going to {d:.6})", .{ a_vel_before.x, a.velocity.x });
            } else if (a_vel_before.x < 0 and a.velocity.x > 0) {
                a.velocity.x = 0.0;
                logger.info(.collision, "PREVENTING FRICTION OVERSHOOT: A x-vel {d:.6} → 0.0 (was going to {d:.6})", .{ a_vel_before.x, a.velocity.x });
            }
        }

        if (b.body_type == .dynamic) {
            b.velocity = b.velocity.add(safe_friction_impulse.scale(b.inverse_mass));
            b.angular_velocity += rb.cross(safe_friction_impulse) * b.inverse_inertia;

            // IMPROVEMENT: Prevent velocity sign change due to friction (friction shouldn't reverse movement)
            if (b_vel_before.x > 0 and b.velocity.x < 0) {
                b.velocity.x = 0.0;
                logger.info(.collision, "PREVENTING FRICTION OVERSHOOT: B x-vel {d:.6} → 0.0 (was going to {d:.6})", .{ b_vel_before.x, b.velocity.x });
            } else if (b_vel_before.x < 0 and b.velocity.x > 0) {
                b.velocity.x = 0.0;
                logger.info(.collision, "PREVENTING FRICTION OVERSHOOT: B x-vel {d:.6} → 0.0 (was going to {d:.6})", .{ b_vel_before.x, b.velocity.x });
            }
        }

        // IMPROVEMENT: Increment the low velocity frames counter for very slow objects
        if (a.body_type == .dynamic and @abs(a.velocity.x) < Constants.VERY_LOW_VELOCITY_THRESHOLD * 2.0) {
            a.low_velocity_frames += 1;
        }

        if (b.body_type == .dynamic and @abs(b.velocity.x) < Constants.VERY_LOW_VELOCITY_THRESHOLD * 2.0) {
            b.low_velocity_frames += 1;
        }
    }

    /// Apply an impulse to both bodies
    fn applyImpulse(a: *RigidBody, b: *RigidBody, impulse: Vector2, ra: Vector2, rb: Vector2) void {
        // Apply linear impulse
        if (a.body_type != .static) {
            const vel_change = CollisionPhysics.calculateVelocityChange(impulse, -a.inverse_mass);
            a.velocity = a.velocity.add(vel_change);
        }

        if (b.body_type != .static) {
            const vel_change = CollisionPhysics.calculateVelocityChange(impulse, b.inverse_mass);
            b.velocity = b.velocity.add(vel_change);
        }

        // Apply angular impulse
        if (a.body_type != .static) {
            a.angular_velocity += CollisionPhysics.calculateAngularVelocityChange(impulse.scale(-1), ra, a.inverse_inertia);
        }

        if (b.body_type != .static) {
            b.angular_velocity += CollisionPhysics.calculateAngularVelocityChange(impulse, rb, b.inverse_inertia);
        }
    }
};
