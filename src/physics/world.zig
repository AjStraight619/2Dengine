const std = @import("std");
const RigidBody = @import("body/body.zig").RigidBody;
const Vector2 = @import("geometry/vector2.zig").Vector2;
const Shape = @import("shapes/shape.zig").Shape;
const BodyType = @import("body/body.zig").BodyType;
const collision = @import("collision/mod.zig");
const factory = @import("body/factory.zig");
const options = @import("body/options.zig");

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
    velocity_iterations: usize = 8,
    position_iterations: usize = 6,

    pub fn init(allocator: std.mem.Allocator) PhysicsWorld {
        return PhysicsWorld{
            .allocator = allocator,
            .gravity = Vector2.init(0, 100),
            .bodies = std.ArrayList(*RigidBody).init(allocator),
            .broadphase = collision.Broadphase.init(allocator),
            .recent_collisions = std.ArrayList(collision.Collision).init(allocator),
            .contact_points = std.ArrayList(Vector2).init(allocator),
            .velocity_iterations = 8,
            .position_iterations = 6,
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
        // Clear debug collision data from previous frame
        self.recent_collisions.clearRetainingCapacity();
        self.contact_points.clearRetainingCapacity();
        self.collision_count = 0;

        // Limit the timestep to prevent tunneling at very low framerates
        // This helps with stability when framerate drops
        const max_dt = 1.0 / 30.0; // Cap at 30Hz minimum
        const current_dt = @min(dt, max_dt);

        // Use small substeps to improve collision detection for fast-moving objects
        // More substeps = more accurate but more CPU intensive
        const substeps: usize = 32; // Reduced from 128 for better performance
        const substep_dt = current_dt / @as(f32, @floatFromInt(substeps));

        // Run physics simulation with substeps
        for (0..substeps) |_| {
            // 1. Update body positions using integration (with fractional dt)
            self.integrateForces(substep_dt);

            // 2. Detect and resolve collisions with multiple iterations for stability
            for (0..self.position_iterations) |_| {
                self.detectAndResolveCollisions() catch {};
            }
        }

        // 3. Update rigid body AABBs
        for (self.bodies.items) |body| {
            body.updateAABB();
        }

        // 4. Reset forces for next frame
        for (self.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.force = Vector2.zero();
                body.torque = 0.0;
            }
        }
    }

    /// Apply physics integration to update positions based on forces
    fn integrateForces(self: *PhysicsWorld, dt: f32) void {
        // Use the specified integrator for simulation
        const Integrator = @import("dynamics/integration.zig").Integrator;

        for (self.bodies.items) |body| {
            // Using semi-implicit Euler for better stability
            Integrator.semiImplicitEuler(body, dt, self.gravity);
        }
    }

    /// Detect and resolve collisions between bodies
    fn detectAndResolveCollisions(self: *PhysicsWorld) !void {
        // 1. Broadphase - find potential collision pairs
        var potential_pairs = try self.broadphase.findPotentialCollisions(self.bodies.items);
        defer potential_pairs.deinit();

        // 2. Narrowphase - detailed collision detection for each pair
        for (potential_pairs.items) |pair| {
            if (collision.Detector.detectCollision(pair.a, pair.b)) |collision_info| {
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
                collision.Resolver.resolveCollision(pair.a, pair.b, collision_info);
            }
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
    }

    /// Get the number of collisions from the last frame
    pub fn getCollisionCount(self: PhysicsWorld) usize {
        return self.collision_count;
    }
};
