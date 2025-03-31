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

pub const RigidBody = struct {
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
            std.debug.print("🛑 VELOCITY CAPPED: body at ({d:.2},{d:.2}) - Original: {d:.2}, Capped to: {d:.2}\n", .{ self.position.x, self.position.y, vel_length, max_velocity });
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

        // Cap linear velocity
        self.capVelocity(max_velocity);

        // Cap angular velocity
        if (@abs(self.angular_velocity) > max_angular_velocity) {
            self.angular_velocity = std.math.sign(self.angular_velocity) * max_angular_velocity;
        }

        // Check for invalid state
        if (self.hasInvalidState()) {
            std.debug.print("⚠️ INVALID PHYSICS STATE DETECTED AND FIXED:\n", .{});
            std.debug.print("  Position: ({d},{d}), Velocity: ({d},{d})\n", .{ self.position.x, self.position.y, self.velocity.x, self.velocity.y });

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

        const sleep_velocity_threshold = 0.05;
        const sleep_angular_threshold = 0.02;
        const time_until_sleep = 1.0; // seconds

        // Check if velocity is below sleeping threshold
        if (self.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold and
            @abs(self.angular_velocity) < sleep_angular_threshold)
        {
            // Accumulate sleep time
            self.sleep_time += dt;

            // If below threshold for long enough, put to sleep
            if (self.sleep_time > time_until_sleep) {
                self.is_sleeping = true;
                self.velocity = Vector2.zero();
                self.angular_velocity = 0.0;
            }
        } else {
            // Reset sleep counter if moving
            self.sleep_time = 0.0;
            self.is_sleeping = false;
        }
    }

    /// Wake up a sleeping body
    pub fn wakeUp(self: *RigidBody) void {
        self.is_sleeping = false;
        self.sleep_time = 0.0;
    }
};
