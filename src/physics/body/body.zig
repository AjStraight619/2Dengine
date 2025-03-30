const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Shape = @import("../shapes/shape.zig").Shape;
const AABB = @import("../geometry/aabb.zig").AABB;
const ShapeProperties = @import("../shapes/properties.zig").ShapeProperties;
const BoundsCalculator = @import("../shapes/bounds.zig").BoundsCalculator;

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
};
