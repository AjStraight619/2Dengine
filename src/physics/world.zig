const std = @import("std");
const RigidBody = @import("body/body.zig").RigidBody;
const Vector2 = @import("geometry/vector2.zig").Vector2;
const Shape = @import("shapes/shape.zig").Shape;
const BodyType = @import("body/body.zig").BodyType;
const collision = @import("collision/mod.zig");
const factory = @import("body/factory.zig");
const options = @import("body/options.zig");
const Integrator = @import("dynamics/integration.zig").Integrator;
const ShapeProperties = @import("shapes/properties.zig").ShapeProperties;
const stability = @import("stability/mod.zig");
const BodyCreator = @import("body/body.zig").BodyCreator;
const logger = @import("logger.zig");

pub const PhysicsWorld = struct {
    allocator: std.mem.Allocator,
    gravity: Vector2,
    bodies: std.ArrayList(*RigidBody),

    // Collision detection and resolution systems
    broadphase: collision.Broadphase,
    detector: collision.Detector = .{},
    resolver: collision.Resolver = .{},

    // Debug options
    debug_draw_collisions: bool = false,
    debug_draw_contacts: bool = false,
    collision_logging: bool = true,

    // Collision data for debug visualization
    recent_collisions: std.ArrayList(collision.Collision),
    contact_points: std.ArrayList(Vector2),
    collision_count: usize = 0,

    // Physics tuning settings
    velocity_iterations: usize = 4,
    position_iterations: usize = 3,

    // Force diagnostics flag
    force_diagnostics: bool = false,

    // For generating unique body IDs
    next_body_id: u64 = 1,

    pub fn init(allocator: std.mem.Allocator) PhysicsWorld {
        return PhysicsWorld{
            .allocator = allocator,
            .gravity = Vector2.init(0, 100),
            .bodies = std.ArrayList(*RigidBody).init(allocator),
            .broadphase = collision.Broadphase.init(allocator),
            .recent_collisions = std.ArrayList(collision.Collision).init(allocator),
            .contact_points = std.ArrayList(Vector2).init(allocator),
            .velocity_iterations = 4,
            .position_iterations = 3,
            .next_body_id = 1,
        };
    }

    pub fn deinit(self: *PhysicsWorld) void {
        // Free all allocated rigid bodies
        for (self.bodies.items) |body| {
            self.allocator.destroy(body);
        }
        self.bodies.deinit();

        // Free collision debug data
        self.recent_collisions.deinit();
        self.contact_points.deinit();
    }

    // Logging configuration methods

    /// Configure the physics logger
    pub fn configureLogging(self: *PhysicsWorld, enabled: bool, min_level: logger.LogLevel) void {
        _ = self; // Unused
        logger.global.setEnabled(enabled);
        logger.global.setMinLevel(min_level);
    }

    /// Enable or disable a specific logging category
    pub fn setLogCategory(self: *PhysicsWorld, category: logger.LogCategory, enabled: bool) void {
        _ = self; // Unused
        logger.global.setCategoryEnabled(category, enabled);
    }

    /// Enable or disable all performance logging
    pub fn setPerformanceLogging(self: *PhysicsWorld, enabled: bool) void {
        _ = self; // Unused
        logger.global.setCategoryEnabled(.performance, enabled);
    }

    /// Enable or disable all collision logging
    pub fn setCollisionLogging(self: *PhysicsWorld, enabled: bool) void {
        self.collision_logging = enabled;
        logger.global.setCategoryEnabled(.collision, enabled);
    }

    // Simplified body creation methods that use the BodyCreator

    /// Create a body with the specified type (internal method)
    pub fn createBodyWithType(self: *PhysicsWorld, body_type: BodyType, shape: Shape, position: Vector2) !*RigidBody {
        return BodyCreator.createBodyWithType(self.allocator, body_type, shape, position, &self.next_body_id, &self.bodies);
    }

    // Add a pre-existing body to the world
    pub fn addBody(self: *PhysicsWorld, body: *RigidBody) !void {
        try self.bodies.append(body);
    }

    // Remove and destroy a body from the world
    pub fn removeBody(self: *PhysicsWorld, body: *RigidBody) void {
        for (self.bodies.items, 0..) |b, i| {
            if (b == body) {
                _ = self.bodies.orderedRemove(i);
                self.allocator.destroy(body);
                return;
            }
        }
    }

    // Remove a body at a specific index
    pub fn removeBodyAtIndex(self: *PhysicsWorld, index: usize) void {
        if (index >= self.bodies.items.len) return;

        const body = self.bodies.orderedRemove(index);
        self.allocator.destroy(body);
    }

    pub fn update(self: *PhysicsWorld, dt: f32) void {
        const update_start = std.time.nanoTimestamp();
        var phase_time: i64 = 0;

        // Clear debug collision data from previous frame
        self.recent_collisions.clearRetainingCapacity();
        self.contact_points.clearRetainingCapacity();
        self.collision_count = 0;

        // 1. Update body positions using integration
        const integration_start = std.time.nanoTimestamp();
        self.integrateForces(dt);
        const integration_end = std.time.nanoTimestamp();
        phase_time = @as(i64, @intCast(integration_end - integration_start));

        if (phase_time > 5_000_000 or self.force_diagnostics) { // 5ms
            logger.logPerformance("INTEGRATION", phase_time, .{});
        }

        // Sanitize all bodies to prevent numerical instability
        const sanitize_start = std.time.nanoTimestamp();
        for (self.bodies.items) |body| {
            body.sanitizeState();

            // Update sleep state
            body.updateSleepState(dt);
        }
        const sanitize_end = std.time.nanoTimestamp();
        phase_time = @as(i64, @intCast(sanitize_end - sanitize_start));

        if (phase_time > 5_000_000 or self.force_diagnostics) { // 5ms
            logger.logPerformance("SANITIZE", phase_time, .{ .bodies_count = self.bodies.items.len });
        }

        // 2. Detect and resolve collisions with multiple iterations for stability
        var collision_time: i64 = 0;
        for (0..self.position_iterations) |iteration| {
            const collision_start = std.time.nanoTimestamp();
            self.detectAndResolveCollisions() catch {};
            const collision_end = std.time.nanoTimestamp();
            phase_time = @as(i64, @intCast(collision_end - collision_start));
            collision_time += phase_time;

            if ((phase_time > 10_000_000 or self.force_diagnostics) and iteration == 0) { // 10ms for a single iteration, only log first iteration when forced
                logger.logPerformance("COLLISION ITERATION", phase_time, .{ .iteration = iteration, .collisions = self.collision_count });
            }
        }

        if (collision_time > 15_000_000 or self.force_diagnostics) { // 15ms total for all iterations
            logger.logPerformance("COLLISION TOTAL", collision_time, .{ .iterations = self.position_iterations, .collisions = self.collision_count });
        }

        // 3. Update rigid body AABBs
        const aabb_start = std.time.nanoTimestamp();
        for (self.bodies.items) |body| {
            body.updateAABB();
        }
        const aabb_end = std.time.nanoTimestamp();
        phase_time = @as(i64, @intCast(aabb_end - aabb_start));

        if (phase_time > 5_000_000 or self.force_diagnostics) { // 5ms
            logger.logPerformance("AABB UPDATE", phase_time, .{ .bodies_count = self.bodies.items.len });
        }

        // 4. Reset forces for next frame
        const reset_start = std.time.nanoTimestamp();
        for (self.bodies.items) |body| {
            if (body.body_type == .dynamic and !body.is_sleeping) {
                body.force = Vector2.zero();
                body.torque = 0.0;
            }
        }
        const reset_end = std.time.nanoTimestamp();
        phase_time = @as(i64, @intCast(reset_end - reset_start));

        if (phase_time > 1_000_000 or self.force_diagnostics) { // 1ms (this should be very fast)
            logger.logPerformance("FORCE RESET", phase_time, .{ .bodies_count = self.bodies.items.len });
        }

        // 5. NEW: Force collective sleep for chains of low-velocity objects
        self.forceCollectiveSleep();

        // Total physics update time
        const update_end = std.time.nanoTimestamp();
        const total_time = @as(i64, @intCast(update_end - update_start));

        if (total_time > 15_000_000 or self.force_diagnostics) { // 15ms
            logger.logPerformance("PHYSICS UPDATE TOTAL", total_time, .{});
        }

        // Log collision count if it's large or diagnostics are forced
        if (self.collision_count > 50 or self.force_diagnostics) {
            logger.info(.collision, "COLLISION COUNT: {d} collisions this frame", .{self.collision_count});
        }

        // Reset force_diagnostics flag after this frame
        self.force_diagnostics = false;
    }

    /// Apply physics integration to update positions based on forces
    fn integrateForces(self: *PhysicsWorld, dt: f32) void {
        // Use the specified integrator for simulation
        for (self.bodies.items) |body| {
            // Using semi-implicit Euler for better stability
            Integrator.semiImplicitEuler(body, dt, self.gravity);
        }
    }

    /// Detect and resolve collisions between bodies
    fn detectAndResolveCollisions(self: *PhysicsWorld) !void {
        // 1. Broadphase - find potential collision pairs
        const broadphase_start = std.time.nanoTimestamp();
        var potential_pairs = try self.broadphase.findPotentialCollisions(self.bodies.items);
        const broadphase_end = std.time.nanoTimestamp();
        const broadphase_time = @as(i64, @intCast(broadphase_end - broadphase_start));

        if (broadphase_time > 5_000_000 or self.force_diagnostics) { // 5ms
            logger.logPerformance("BROADPHASE", broadphase_time, .{ .bodies_count = self.bodies.items.len, .pairs_count = potential_pairs.items.len });
        }

        defer potential_pairs.deinit();

        // 2. Narrowphase - detailed collision detection for each pair
        var pairs_resolved: usize = 0;
        var max_resolution_time: i64 = 0;
        var total_resolution_time: i64 = 0;

        for (potential_pairs.items) |pair| {
            const detection_start = std.time.nanoTimestamp();
            const collision_result = collision.Detector.detectCollision(pair.a, pair.b);
            const detection_end = std.time.nanoTimestamp();
            const detection_time = @as(i64, @intCast(detection_end - detection_start));

            if (detection_time > 1_000_000 or self.force_diagnostics) { // 1ms
                logger.logPerformance("DETECTION", detection_time, .{ .body_a = .{ .pos = pair.a.position }, .body_b = .{ .pos = pair.b.position } });
            }

            if (collision_result) |collision_info| {
                // Store collision info for debugging if enabled
                if (self.debug_draw_collisions) {
                    try self.recent_collisions.append(collision_info);
                }

                // Store contact points for debugging if enabled
                if (self.debug_draw_contacts) {
                    for (0..collision_info.contact_count) |i| {
                        try self.contact_points.append(collision_info.contact_points[i]);
                    }
                }

                // Log collision if enabled
                if (self.collision_logging) {
                    // We can add detailed logging here if needed
                    //logger.debug(.collision, "Collision detected: depth={d}, normal=({d},{d})",
                    //    .{ collision_info.depth, collision_info.normal.x, collision_info.normal.y });
                }

                // Increment collision counter for statistics
                self.collision_count += 1;

                // 3. Resolve the collision
                const resolution_start = std.time.nanoTimestamp();
                collision.Resolver.resolveCollision(pair.a, pair.b, collision_info);
                const resolution_end = std.time.nanoTimestamp();
                const resolution_time = @as(i64, @intCast(resolution_end - resolution_start));

                total_resolution_time += resolution_time;
                if (resolution_time > max_resolution_time) {
                    max_resolution_time = resolution_time;
                }

                if (resolution_time > 5_000_000 or self.force_diagnostics) { // 5ms
                    logger.logPerformance("RESOLUTION", resolution_time, .{ .body_a = .{ .pos = pair.a.position }, .body_b = .{ .pos = pair.b.position } });
                }

                pairs_resolved += 1;
            }
        }

        // Log if overall collision resolution was slow
        if (total_resolution_time > 10_000_000 or self.force_diagnostics) { // 10ms
            logger.logPerformance("COLLISION RESOLUTION TOTAL", total_resolution_time, .{ .pairs_resolved = pairs_resolved, .total_pairs = potential_pairs.items.len });
        }
    }

    /// Enable collision visualization for debugging
    pub fn setDebugDrawCollisions(self: *PhysicsWorld, enabled: bool) void {
        self.debug_draw_collisions = enabled;
    }

    /// Enable contact point visualization for debugging
    pub fn setDebugDrawContacts(self: *PhysicsWorld, enabled: bool) void {
        self.debug_draw_contacts = enabled;
    }

    // New method to configure the broadphase
    pub fn configureBroadphase(self: *PhysicsWorld, cell_size: f32) void {
        self.broadphase.setCellSize(cell_size);
    }

    /// Force complete performance diagnostics for the next frame
    pub fn forceDiagnostics(self: *PhysicsWorld) void {
        logger.info(.general, "FORCING DETAILED PHYSICS DIAGNOSTICS", .{});
        // Set temporary override flags that will cause all performance metrics to be logged
        // These will be reset after one frame
        self.force_diagnostics = true;
    }

    // User-friendly body API methods

    /// Get a body by its unique ID
    pub fn getBodyById(self: *PhysicsWorld, id: u64) ?*RigidBody {
        for (self.bodies.items) |body| {
            if (body.id == id) {
                return body;
            }
        }
        return null;
    }

    /// Remove a body by its unique ID
    pub fn removeBodyById(self: *PhysicsWorld, id: u64) bool {
        for (self.bodies.items, 0..) |body, i| {
            if (body.id == id) {
                _ = self.bodies.orderedRemove(i);
                self.allocator.destroy(body);
                return true;
            }
        }
        return false;
    }

    /// Remove all bodies of a specific type (e.g., all dynamic bodies)
    pub fn removeAllBodiesOfType(self: *PhysicsWorld, body_type: BodyType) usize {
        var count: usize = 0;
        var i: usize = 0;
        while (i < self.bodies.items.len) {
            const body = self.bodies.items[i];
            if (body.body_type == body_type) {
                _ = self.bodies.orderedRemove(i);
                self.allocator.destroy(body);
                count += 1;
            } else {
                i += 1;
            }
        }
        return count;
    }

    /// Apply a force to a body specified by ID
    pub fn applyForceToBody(self: *PhysicsWorld, id: u64, force: Vector2) bool {
        if (self.getBodyById(id)) |body| {
            body.applyForce(force);
            return true;
        }
        return false;
    }

    /// Apply torque to a body specified by ID
    pub fn applyTorqueToBody(self: *PhysicsWorld, id: u64, torque: f32) bool {
        if (self.getBodyById(id)) |body| {
            body.applyTorque(torque);
            return true;
        }
        return false;
    }

    /// Apply impulse to a body specified by ID
    pub fn applyImpulseToBody(self: *PhysicsWorld, id: u64, impulse: Vector2, contact_point: ?Vector2) bool {
        if (self.getBodyById(id)) |body| {
            body.applyImpulse(impulse, contact_point);
            return true;
        }
        return false;
    }

    /// Add a circle body with properties - wrapper around BodyCreator.createCircle
    pub fn addCircle(self: *PhysicsWorld, properties: struct {
        type: BodyType,
        position: Vector2,
        radius: f32,
        mass: ?f32 = null,
        restitution: ?f32 = null,
        friction: ?f32 = null,
        velocity: ?Vector2 = null,
        angular_velocity: ?f32 = null,
        rotation: ?f32 = null,
        initial_force: ?Vector2 = null,
    }) !*RigidBody {
        return BodyCreator.createCircleFromWorld(self.allocator, properties, &self.next_body_id, &self.bodies);
    }

    /// Add a rectangle body with properties - wrapper around BodyCreator.createRectangle
    /// Note: This method now accepts angle in degrees for better usability
    pub fn addRectangle(
        self: *PhysicsWorld,
        properties: struct {
            type: BodyType,
            position: Vector2,
            width: f32,
            height: f32,
            angle_degrees: ?f32 = null, // Using angle_degrees consistently
            mass: ?f32 = null,
            restitution: ?f32 = null,
            friction: ?f32 = null,
            velocity: ?Vector2 = null,
            angular_velocity: ?f32 = null,
            rotation: ?f32 = null,
        },
    ) !*RigidBody {
        return BodyCreator.createRectangleFromWorld(self.allocator, properties, &self.next_body_id, &self.bodies);
    }

    /// Force sleep for nearly-at-rest objects to prevent perpetual wake-up chain reactions
    pub fn forceCollectiveSleep(self: *PhysicsWorld) void {
        // Delegate to the stability module's implementation
        stability.forceCollectiveSleep(self);
    }
};
