const std = @import("std");
const RigidBody = @import("body/body.zig").RigidBody;
const Vector2 = @import("geometry/vector2.zig").Vector2;
const Shape = @import("shapes/shape.zig").Shape;
const BodyType = @import("body/body.zig").BodyType;
const collision = @import("collision/mod.zig");
const factory = @import("body/factory.zig");
const options = @import("body/options.zig");
const Integrator = @import("dynamics/integration.zig").Integrator;

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

    // New member for force diagnostics
    force_diagnostics: bool = false,

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

    // Factory access points for different body types
    pub fn static(self: *PhysicsWorld) factory.StaticBodyFactory {
        return factory.StaticBodyFactory{ .world = self };
    }

    pub fn dynamic(self: *PhysicsWorld) factory.DynamicBodyFactory {
        return factory.DynamicBodyFactory{ .world = self };
    }

    pub fn kinematic(self: *PhysicsWorld) factory.KinematicBodyFactory {
        return factory.KinematicBodyFactory{ .world = self };
    }

    // Internal method to create a body with the specified type
    pub fn createBodyWithType(self: *PhysicsWorld, body_type: BodyType, shape: Shape, position: Vector2) !*RigidBody {
        const body = try self.allocator.create(RigidBody);
        const mass: f32 = switch (body_type) {
            .static => std.math.floatMax(f32),
            .dynamic => 1.0, // Default mass for dynamic bodies
            .kinematic => 0.0, // Kinematic bodies don't respond to forces
        };

        body.* = RigidBody.init(body_type, position, shape, mass);
        try self.bodies.append(body);
        return body;
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
            std.debug.print("⏱️ INTEGRATION: {d:.2}ms\n", .{@as(f64, @floatFromInt(phase_time)) / 1_000_000.0});
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
            std.debug.print("⏱️ SANITIZE: {d:.2}ms for {d} bodies\n", .{ @as(f64, @floatFromInt(phase_time)) / 1_000_000.0, self.bodies.items.len });
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
                std.debug.print("⏱️ COLLISION ITERATION {d}: {d:.2}ms with {d} collisions\n", .{ iteration, @as(f64, @floatFromInt(phase_time)) / 1_000_000.0, self.collision_count });
            }
        }

        if (collision_time > 15_000_000 or self.force_diagnostics) { // 15ms total for all iterations
            std.debug.print("⏱️ COLLISION TOTAL: {d:.2}ms for {d} iterations with {d} collisions\n", .{ @as(f64, @floatFromInt(collision_time)) / 1_000_000.0, self.position_iterations, self.collision_count });
        }

        // 3. Update rigid body AABBs
        const aabb_start = std.time.nanoTimestamp();
        for (self.bodies.items) |body| {
            body.updateAABB();
        }
        const aabb_end = std.time.nanoTimestamp();
        phase_time = @as(i64, @intCast(aabb_end - aabb_start));

        if (phase_time > 5_000_000 or self.force_diagnostics) { // 5ms
            std.debug.print("⏱️ AABB UPDATE: {d:.2}ms for {d} bodies\n", .{ @as(f64, @floatFromInt(phase_time)) / 1_000_000.0, self.bodies.items.len });
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
            std.debug.print("⏱️ FORCE RESET: {d:.2}ms for {d} bodies\n", .{ @as(f64, @floatFromInt(phase_time)) / 1_000_000.0, self.bodies.items.len });
        }

        // Total physics update time
        const update_end = std.time.nanoTimestamp();
        const total_time = @as(i64, @intCast(update_end - update_start));

        if (total_time > 15_000_000 or self.force_diagnostics) { // 15ms
            std.debug.print("⏱️ PHYSICS UPDATE TOTAL: {d:.2}ms\n", .{@as(f64, @floatFromInt(total_time)) / 1_000_000.0});
        }

        // Log collision count if it's large or diagnostics are forced
        if (self.collision_count > 50 or self.force_diagnostics) {
            std.debug.print("📊 COLLISION COUNT: {d} collisions this frame\n", .{self.collision_count});
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
            std.debug.print("⏱️ BROADPHASE: {d:.2}ms for {d} bodies, found {d} potential pairs\n", .{ @as(f64, @floatFromInt(broadphase_time)) / 1_000_000.0, self.bodies.items.len, potential_pairs.items.len });
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
                std.debug.print("⏱️ DETECTION: {d:.2}ms between bodies at ({d:.1},{d:.1}) and ({d:.1},{d:.1})\n", .{ @as(f64, @floatFromInt(detection_time)) / 1_000_000.0, pair.a.position.x, pair.a.position.y, pair.b.position.x, pair.b.position.y });
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
                    // Log only critical collisions or disable by default to improve performance
                    // std.debug.print("Collision detected: depth={d}, normal=({d},{d}), body_a_pos=({d},{d}), body_b_pos=({d},{d})\n", .{ collision_info.depth, collision_info.normal.x, collision_info.normal.y, pair.a.position.x, pair.a.position.y, pair.b.position.x, pair.b.position.y });
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
                    std.debug.print("⏱️ RESOLUTION: {d:.2}ms for collision between bodies at ({d:.1},{d:.1}) and ({d:.1},{d:.1})\n", .{ @as(f64, @floatFromInt(resolution_time)) / 1_000_000.0, pair.a.position.x, pair.a.position.y, pair.b.position.x, pair.b.position.y });
                }

                pairs_resolved += 1;
            }
        }

        // Log if overall collision resolution was slow
        if (total_resolution_time > 10_000_000 or self.force_diagnostics) { // 10ms
            std.debug.print("⏱️ COLLISION RESOLUTION TOTAL: {d:.2}ms for {d}/{d} pairs\n", .{ @as(f64, @floatFromInt(total_resolution_time)) / 1_000_000.0, pairs_resolved, potential_pairs.items.len });
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

    /// Enable collision logging to console
    pub fn setCollisionLogging(self: *PhysicsWorld, enabled: bool) void {
        self.collision_logging = enabled;
        // If we're turning off collision logging, also turn off broadphase logging
        if (!enabled) {
            // TODO: Add a flag in broadphase to disable excessive logging
        }
    }

    /// Get the number of collisions from the last frame
    pub fn getCollisionCount(self: PhysicsWorld) usize {
        return self.collision_count;
    }

    // New method to configure the broadphase
    pub fn configureBroadphase(self: *PhysicsWorld, cell_size: f32) void {
        self.broadphase.setCellSize(cell_size);
    }

    /// Force complete performance diagnostics for the next frame
    pub fn forceDiagnostics(self: *PhysicsWorld) void {
        std.debug.print("\n--- FORCING DETAILED PHYSICS DIAGNOSTICS ---\n", .{});
        // Set temporary override flags that will cause all performance metrics to be logged
        // These will be reset after one frame
        self.force_diagnostics = true;
    }
};
