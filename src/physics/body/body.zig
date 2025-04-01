const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Shape = @import("../shapes/shape.zig").Shape;
const AABB = @import("../geometry/aabb.zig").AABB;
const ShapeProperties = @import("../shapes/properties.zig").ShapeProperties;
const BoundsCalculator = @import("../shapes/bounds.zig").BoundsCalculator;
const std = @import("std");

pub const BodyType = enum {
    static,
    dynamic,
    kinematic,
};

/// Body creation functionality - extracted from PhysicsWorld
pub const BodyCreator = struct {
    /// Helper function to convert degrees to radians for more intuitive API
    pub fn degreesToRadians(degrees: f32) f32 {
        return std.math.degreesToRadians(degrees);
    }

    /// Create a new body with the specified type
    pub fn createBodyWithType(
        allocator: std.mem.Allocator,
        body_type: BodyType,
        shape: Shape,
        position: Vector2,
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        const body = try allocator.create(RigidBody);
        const mass: f32 = switch (body_type) {
            .static => std.math.floatMax(f32),
            .dynamic => 1.0, // Default mass for dynamic bodies
            .kinematic => 0.0, // Kinematic bodies don't respond to forces
        };

        body.* = RigidBody.init(body_type, position, shape, mass);

        // Assign a unique ID and increment the counter
        body.id = next_body_id.*;
        next_body_id.* += 1;

        try bodies.append(body);
        return body;
    }

    /// Apply properties to a body
    pub fn applyBodyProperties(
        body: *RigidBody,
        shape: Shape,
        properties: anytype,
    ) void {
        // Apply additional properties based on the body type
        if (@hasField(@TypeOf(properties), "mass") and properties.mass != null and body.body_type == .dynamic) {
            body.mass = properties.mass.?;
            body.inverse_mass = if (properties.mass.? <= 0.0) 0.0 else 1.0 / properties.mass.?;
            body.inertia = ShapeProperties.getMomentOfInertia(shape, properties.mass.?);
            body.inverse_inertia = if (body.inertia <= 0.0) 0.0 else 1.0 / body.inertia;
        }

        if (@hasField(@TypeOf(properties), "restitution") and properties.restitution != null) {
            body.restitution = properties.restitution.?;
        }

        if (@hasField(@TypeOf(properties), "friction") and properties.friction != null) {
            body.friction = properties.friction.?;
        }

        if (@hasField(@TypeOf(properties), "velocity") and properties.velocity != null) {
            body.velocity = properties.velocity.?;
        }

        if (@hasField(@TypeOf(properties), "angular_velocity") and properties.angular_velocity != null) {
            body.angular_velocity = properties.angular_velocity.?;
        }

        if (@hasField(@TypeOf(properties), "rotation") and properties.rotation != null) {
            body.rotation = properties.rotation.?;
        }
    }

    /// Create a body with any type of shape
    pub fn createBody(
        allocator: std.mem.Allocator,
        properties: struct {
            type: BodyType,
            shape: Shape,
            position: Vector2,
            mass: ?f32 = null,
            restitution: ?f32 = null,
            friction: ?f32 = null,
            velocity: ?Vector2 = null,
            angular_velocity: ?f32 = null,
            rotation: ?f32 = null,
            initial_force: ?Vector2 = null,
        },
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        const body = try createBodyWithType(allocator, properties.type, properties.shape, properties.position, next_body_id, bodies);

        applyBodyProperties(body, properties.shape, properties);

        // Apply initial force if provided
        if (properties.initial_force != null and body.body_type == .dynamic) {
            body.force = properties.initial_force.?;
        }

        return body;
    }

    /// Create a circle body with properties
    pub fn createCircle(
        allocator: std.mem.Allocator,
        properties: struct {
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
        },
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        const shape = Shape{ .circle = .{ .radius = properties.radius } };

        // Create the body with basic properties
        const body = try createBody(allocator, .{
            .type = properties.type,
            .shape = shape,
            .position = properties.position,
            .mass = properties.mass,
            .restitution = properties.restitution,
            .friction = properties.friction,
            .velocity = properties.velocity,
            .angular_velocity = properties.angular_velocity,
            .rotation = properties.rotation,
            .initial_force = properties.initial_force,
        }, next_body_id, bodies);

        // Apply initial force if provided
        if (properties.initial_force != null and body.body_type == .dynamic) {
            body.force = properties.initial_force.?;
        }

        return body;
    }

    /// Wrapper function that accepts any struct type for creating circles
    pub fn createCircleFromWorld(
        allocator: std.mem.Allocator,
        world_properties: anytype,
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        // Create circle shape
        const shape = Shape{ .circle = .{ .radius = world_properties.radius } };

        // Create the body
        const body = try createBodyWithType(allocator, world_properties.type, shape, world_properties.position, next_body_id, bodies);

        // Apply properties
        applyBodyProperties(body, shape, world_properties);

        // Apply initial force if provided
        if (@hasField(@TypeOf(world_properties), "initial_force") and
            world_properties.initial_force != null and
            body.body_type == .dynamic)
        {
            body.force = world_properties.initial_force.?;
        }

        return body;
    }

    /// Create a rectangle body with properties, accepting angle in degrees for better usability
    pub fn createRectangle(
        allocator: std.mem.Allocator,
        properties: struct {
            type: BodyType,
            position: Vector2,
            width: f32,
            height: f32,
            angle_degrees: ?f32 = null, // Angle in degrees for better usability
            mass: ?f32 = null,
            restitution: ?f32 = null,
            friction: ?f32 = null,
            velocity: ?Vector2 = null,
            angular_velocity: ?f32 = null,
            rotation: ?f32 = null,
        },
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        // Convert degrees to radians for internal use
        const angle_degrees = properties.angle_degrees orelse 0.0;
        const angle_radians = degreesToRadians(angle_degrees);

        const shape = Shape{ .rectangle = .{ .width = properties.width, .height = properties.height, .angle = angle_radians } };

        return try createBody(allocator, .{
            .type = properties.type,
            .shape = shape,
            .position = properties.position,
            .mass = properties.mass,
            .restitution = properties.restitution,
            .friction = properties.friction,
            .velocity = properties.velocity,
            .angular_velocity = properties.angular_velocity,
            .rotation = properties.rotation,
            .initial_force = null,
        }, next_body_id, bodies);
    }

    /// Wrapper function that accepts any struct type for creating rectangles
    pub fn createRectangleFromWorld(
        allocator: std.mem.Allocator,
        world_properties: anytype,
        next_body_id: *u64,
        bodies: *std.ArrayList(*RigidBody),
    ) !*RigidBody {
        // Convert degrees to radians for internal use
        const angle_degrees = if (@hasField(@TypeOf(world_properties), "angle_degrees"))
            (world_properties.angle_degrees orelse 0.0)
        else
            0.0;

        const angle_radians = degreesToRadians(angle_degrees);

        // Create rectangle shape
        const shape = Shape{ .rectangle = .{ .width = world_properties.width, .height = world_properties.height, .angle = angle_radians } };

        // Create the body
        const body = try createBodyWithType(allocator, world_properties.type, shape, world_properties.position, next_body_id, bodies);

        // Apply properties
        applyBodyProperties(body, shape, world_properties);

        // Apply initial force if provided
        if (@hasField(@TypeOf(world_properties), "initial_force") and
            world_properties.initial_force != null and
            body.body_type == .dynamic)
        {
            body.force = world_properties.initial_force.?;
        }

        return body;
    }
};

pub const RigidBody = struct {
    // Unique identifier
    id: u64 = 0,

    // Basic properties
    body_type: BodyType,
    position: Vector2,
    velocity: Vector2,
    acceleration: Vector2,
    rotation: f32,

    // Angular motion
    angular_velocity: f32 = 0.0,
    angular_acceleration: f32 = 0.0,

    // Mass properties
    mass: f32,
    inverse_mass: f32,
    inertia: f32,
    inverse_inertia: f32,

    // Material properties
    restitution: f32 = 0.2, // Bounciness (0-1)
    friction: f32 = 0.1, // Friction coefficient

    // Force accumulators
    force: Vector2 = Vector2.zero(),
    torque: f32 = 0.0,

    // Collision properties
    shape: Shape,
    aabb: AABB,

    // Utility
    last_position: Vector2,

    // Sleeping mechanism
    is_sleeping: bool = false,
    sleep_time: f32 = 0.0,
    low_velocity_frames: u32 = 0, // Track consecutive frames with low velocity

    pub fn init(body_type: BodyType, position: Vector2, shape: Shape, mass: f32) RigidBody {
        var body = RigidBody{
            .body_type = body_type,
            .position = position,
            .last_position = position,
            .velocity = Vector2.zero(),
            .acceleration = Vector2.zero(),
            .rotation = 0.0,
            .mass = mass,
            .inverse_mass = if (body_type == .static or mass <= 0.0) 0.0 else 1.0 / mass,
            .shape = shape,
            .aabb = BoundsCalculator.computeAABB(shape, position),
            .inertia = ShapeProperties.getMomentOfInertia(shape, mass),
            .inverse_inertia = 0.0, // Will be calculated below
        };

        body.inverse_inertia = if (body_type == .static or body.inertia <= 0.0)
            0.0
        else
            1.0 / body.inertia;

        return body;
    }

    pub fn updateAABB(self: *RigidBody) void {
        self.aabb = BoundsCalculator.computeAABB(self.shape, self.position);
    }

    pub fn applyForce(self: *RigidBody, force: Vector2) void {
        if (self.body_type != .dynamic) return;
        self.force = self.force.add(force);
    }

    pub fn applyTorque(self: *RigidBody, torque: f32) void {
        if (self.body_type != .dynamic) return;
        self.torque += torque;
    }

    pub fn applyImpulse(self: *RigidBody, impulse: Vector2, contact_point: ?Vector2) void {
        if (self.body_type != .dynamic) return;

        // Linear impulse
        self.velocity = self.velocity.add(impulse.scale(self.inverse_mass));

        // Angular impulse if contact point provided
        if (contact_point) |point| {
            const r = point.sub(self.position);
            const torque = r.cross(impulse);
            self.angular_velocity += torque * self.inverse_inertia;
        }
    }

    // Fluent interface methods for configuration

    /// Set the velocity of the body and return self for chaining
    pub fn withVelocity(self: *RigidBody, vel: Vector2) *RigidBody {
        self.velocity = vel;
        return self;
    }

    /// Set the angular velocity of the body and return self for chaining
    pub fn withAngularVelocity(self: *RigidBody, ang_vel: f32) *RigidBody {
        self.angular_velocity = ang_vel;
        return self;
    }

    /// Set the restitution (bounciness) of the body and return self for chaining
    pub fn withRestitution(self: *RigidBody, rest: f32) *RigidBody {
        self.restitution = rest;
        return self;
    }

    /// Set the friction coefficient of the body and return self for chaining
    pub fn withFriction(self: *RigidBody, fric: f32) *RigidBody {
        self.friction = fric;
        return self;
    }

    /// Set the mass of the body and update inverse mass and inertia
    pub fn withMass(self: *RigidBody, new_mass: f32) *RigidBody {
        if (self.body_type == .static) return self;

        self.mass = new_mass;
        self.inverse_mass = if (new_mass <= 0.0) 0.0 else 1.0 / new_mass;

        // Recalculate moment of inertia
        self.inertia = ShapeProperties.getMomentOfInertia(self.shape, new_mass);
        self.inverse_inertia = if (self.inertia <= 0.0) 0.0 else 1.0 / self.inertia;

        return self;
    }

    /// Apply an initial force to the body
    pub fn withInitialForce(self: *RigidBody, force_vector: Vector2) *RigidBody {
        if (self.body_type != .dynamic) return self;

        self.force = force_vector;
        return self;
    }

    /// Apply an initial torque to the body
    pub fn withInitialTorque(self: *RigidBody, torque_value: f32) *RigidBody {
        if (self.body_type != .dynamic) return self;

        self.torque = torque_value;
        return self;
    }

    /// Set the rotation angle of the body
    pub fn withRotation(self: *RigidBody, angle: f32) *RigidBody {
        self.rotation = angle;
        return self;
    }

    /// Cap velocity to prevent numerical instability
    pub fn capVelocity(self: *RigidBody, max_velocity: f32) void {
        const vel_length = self.velocity.length();
        if (vel_length > max_velocity) {
            // Scale down velocity to maximum
            self.velocity = self.velocity.scale(max_velocity / vel_length);
            // Log the capping
            const logger = @import("../logger.zig");
            logger.warning(.stability, "VELOCITY CAPPED: body at ({d:.2},{d:.2}) - Original: {d:.2}, Capped to: {d:.2}", .{ self.position.x, self.position.y, vel_length, max_velocity });
        }
    }

    /// Check if position or velocity have invalid values
    pub fn hasInvalidState(self: *RigidBody) bool {
        // Check for NaN or infinity
        if (std.math.isNan(self.position.x) or std.math.isNan(self.position.y) or
            std.math.isNan(self.velocity.x) or std.math.isNan(self.velocity.y) or
            std.math.isInf(self.position.x) or std.math.isInf(self.position.y) or
            std.math.isInf(self.velocity.x) or std.math.isInf(self.velocity.y))
        {
            return true;
        }

        // Check for unreasonably large values
        const max_position = 10000.0;
        const max_velocity = 10000.0;

        if (@abs(self.position.x) > max_position or @abs(self.position.y) > max_position or
            @abs(self.velocity.x) > max_velocity or @abs(self.velocity.y) > max_velocity)
        {
            return true;
        }

        return false;
    }

    /// Sanitize the physics state by capping velocity and fixing invalid positions
    pub fn sanitizeState(self: *RigidBody) void {
        // Reasonable maximum values for a stable simulation
        const max_velocity: f32 = 1000.0;
        const max_angular_velocity: f32 = 50.0;
        const logger = @import("../logger.zig");

        // Cap linear velocity
        self.capVelocity(max_velocity);

        // Cap angular velocity
        if (@abs(self.angular_velocity) > max_angular_velocity) {
            self.angular_velocity = std.math.sign(self.angular_velocity) * max_angular_velocity;
        }

        // Check for invalid state
        if (self.hasInvalidState()) {
            logger.err(.stability, "INVALID PHYSICS STATE DETECTED AND FIXED", .{});
            logger.err(.stability, "Position: ({d},{d}), Velocity: ({d},{d})", .{ self.position.x, self.position.y, self.velocity.x, self.velocity.y });

            // Reset to a safe state
            if (std.math.isNan(self.position.x) or std.math.isInf(self.position.x)) self.position.x = 0;
            if (std.math.isNan(self.position.y) or std.math.isInf(self.position.y)) self.position.y = 0;
            if (std.math.isNan(self.velocity.x) or std.math.isInf(self.velocity.x)) self.velocity.x = 0;
            if (std.math.isNan(self.velocity.y) or std.math.isInf(self.velocity.y)) self.velocity.y = 0;

            // Cap extreme but valid values
            const safe_pos_limit = 5000.0;
            if (@abs(self.position.x) > safe_pos_limit) self.position.x = std.math.sign(self.position.x) * safe_pos_limit;
            if (@abs(self.position.y) > safe_pos_limit) self.position.y = std.math.sign(self.position.y) * safe_pos_limit;

            // Reset all motion
            self.velocity = Vector2.zero();
            self.angular_velocity = 0;
            self.force = Vector2.zero();
            self.torque = 0;
        }
    }

    /// Check if body should be put to sleep (has very low velocity for some time)
    pub fn updateSleepState(self: *RigidBody, dt: f32) void {
        if (self.body_type != .dynamic) return;
        const logger = @import("../logger.zig");

        // More aggressive sleep thresholds for better stability
        const sleep_velocity_threshold = 0.02; // Reduced from 0.05
        const sleep_angular_threshold = 0.01; // Reduced from 0.02
        const time_until_sleep = 0.5; // Reduced from 1.0 second
        const consecutive_frames_for_sleep = 10; // New: require several consecutive frames of low velocity

        // Check if velocity is below sleeping threshold
        const velocity_sq = self.velocity.lengthSquared();
        const is_below_threshold =
            velocity_sq < sleep_velocity_threshold * sleep_velocity_threshold and
            @abs(self.angular_velocity) < sleep_angular_threshold;

        if (is_below_threshold) {
            // Track consecutive low velocity frames
            self.low_velocity_frames += 1;

            // Accumulate sleep time
            self.sleep_time += dt;

            // If extremely low velocity, force sleep faster
            if (velocity_sq < 0.0001) {
                self.sleep_time += dt * 2.0; // Accelerate sleep time
            }

            // If below threshold for long enough OR enough consecutive low-velocity frames, put to sleep
            if (self.sleep_time > time_until_sleep or self.low_velocity_frames >= consecutive_frames_for_sleep) {
                self.is_sleeping = true;
                self.velocity = Vector2.zero(); // Explicitly zero out velocity
                self.angular_velocity = 0.0;

                // Log when we put a body to sleep
                logger.debug(.sleep, "Body at ({d:.1}, {d:.1}) PUT TO SLEEP (v={d:.4}, frames={d})", .{ self.position.x, self.position.y, @sqrt(velocity_sq), self.low_velocity_frames });
            }
        } else {
            // Reset sleep counter if moving significantly
            self.sleep_time = 0.0;
            self.low_velocity_frames = 0;

            // Only wake up if previously sleeping and now substantial movement
            if (self.is_sleeping and velocity_sq > sleep_velocity_threshold * sleep_velocity_threshold * 4.0) {
                self.is_sleeping = false;
                logger.debug(.sleep, "Body at ({d:.1}, {d:.1}) WOKE UP (v={d:.4})", .{ self.position.x, self.position.y, @sqrt(velocity_sq) });
            }
        }
    }

    /// Wake up a sleeping body
    pub fn wakeUp(self: *RigidBody) void {
        const logger = @import("../logger.zig");

        // Don't wake up bodies with barely any force/velocity - this prevents jitter
        const min_force_for_wakeup = 0.5;
        const min_velocity_for_wakeup = 0.1;

        // Only wake up if there's sufficient force or velocity to justify it
        if (self.force.lengthSquared() < min_force_for_wakeup * min_force_for_wakeup and
            self.velocity.lengthSquared() < min_velocity_for_wakeup * min_velocity_for_wakeup)
        {
            logger.debug(.sleep, "IGNORED WAKE: Force/velocity too small to wake body", .{});
            return;
        }

        self.is_sleeping = false;
        self.sleep_time = 0.0;
        self.low_velocity_frames = 0; // Reset the frame counter too
    }

    /// Wake up a body with a minimum velocity to ensure it actually moves
    pub fn wakeUpWithMinVelocity(self: *RigidBody, min_velocity: f32) void {
        self.wakeUp();

        // Ensure a minimum velocity for the awakened body
        const vel_len_sq = self.velocity.lengthSquared();
        if (vel_len_sq < min_velocity * min_velocity) {
            if (vel_len_sq > 0.001) {
                // Body has some velocity - scale it up to minimum
                const current_len = @sqrt(vel_len_sq);
                self.velocity = self.velocity.scale(min_velocity / current_len);
            } else {
                // Body has no significant velocity - give it a small push downward
                self.velocity = Vector2.init(0, min_velocity);
            }
        }
    }

    /// Wake a connected group of bodies (for chain reactions)
    pub fn wakeConnected(self: *RigidBody, bodies: []RigidBody, min_distance: f32) void {
        self.wakeUp();

        if (bodies.len == 0) return;

        // Wake up any bodies in close proximity
        for (bodies) |*other| {
            if (other.body_type != .dynamic or other == self) continue;

            // If bodies are close enough, wake the other one too
            const dist_sq = self.position.distanceToSquared(other.position);
            if (dist_sq < min_distance * min_distance) {
                other.wakeUp();
            }
        }
    }
};
