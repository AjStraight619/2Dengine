const Shape = @import("../shapes/shape.zig").Shape;
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("body.zig").RigidBody;
const BodyType = @import("body.zig").BodyType;
const options = @import("options.zig");

// Factory for creating static bodies
pub const StaticBodyFactory = struct {
    world: *anyopaque, // Opaque pointer to PhysicsWorld to avoid circular dependencies

    // Create a circle with options that include position and radius
    pub fn circle(self: StaticBodyFactory, circle_options: options.StaticCircleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .circle = .{ .radius = circle_options.radius } };
        const body = try world.createBodyWithType(.static, shape, circle_options.position);
        return applyStaticBodyOptions(body, circle_options);
    }

    // Create a rectangle with options that include position, width, height, and angle
    pub fn rectangle(self: StaticBodyFactory, rect_options: options.StaticRectangleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .rectangle = .{ .width = rect_options.width, .height = rect_options.height, .angle = rect_options.angle } };
        const body = try world.createBodyWithType(.static, shape, rect_options.position);
        return applyStaticBodyOptions(body, rect_options);
    }
};

// Factory for creating dynamic bodies
pub const DynamicBodyFactory = struct {
    world: *anyopaque, // Opaque pointer to PhysicsWorld

    // Create a circle with options that include position and radius
    pub fn circle(self: DynamicBodyFactory, circle_options: options.DynamicCircleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .circle = .{ .radius = circle_options.radius } };
        const body = try world.createBodyWithType(.dynamic, shape, circle_options.position);
        return applyDynamicBodyOptions(body, circle_options);
    }

    // Create a rectangle with options that include position, width, height, and angle
    pub fn rectangle(self: DynamicBodyFactory, rect_options: options.DynamicRectangleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .rectangle = .{ .width = rect_options.width, .height = rect_options.height, .angle = rect_options.angle } };
        const body = try world.createBodyWithType(.dynamic, shape, rect_options.position);
        return applyDynamicBodyOptions(body, rect_options);
    }
};

// Factory for creating kinematic bodies
pub const KinematicBodyFactory = struct {
    world: *anyopaque, // Opaque pointer to PhysicsWorld

    // Create a circle with options that include position and radius
    pub fn circle(self: KinematicBodyFactory, circle_options: options.KinematicCircleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .circle = .{ .radius = circle_options.radius } };
        const body = try world.createBodyWithType(.kinematic, shape, circle_options.position);
        return applyKinematicBodyOptions(body, circle_options);
    }

    // Create a rectangle with options that include position, width, height, and angle
    pub fn rectangle(self: KinematicBodyFactory, rect_options: options.KinematicRectangleOptions) !*RigidBody {
        const world = @as(*PhysicsWorld, @ptrCast(@alignCast(self.world)));
        const shape = Shape{ .rectangle = .{ .width = rect_options.width, .height = rect_options.height, .angle = rect_options.angle } };
        const body = try world.createBodyWithType(.kinematic, shape, rect_options.position);
        return applyKinematicBodyOptions(body, rect_options);
    }
};

// Apply static body options
pub fn applyStaticBodyOptions(body: *RigidBody, opts: anytype) *RigidBody {
    if (@hasField(@TypeOf(opts), "restitution") and opts.restitution != null) {
        body.restitution = opts.restitution.?;
    }

    if (@hasField(@TypeOf(opts), "friction") and opts.friction != null) {
        body.friction = opts.friction.?;
    }

    return body;
}

// Apply dynamic body options
pub fn applyDynamicBodyOptions(body: *RigidBody, opts: anytype) *RigidBody {
    if (@hasField(@TypeOf(opts), "mass") and opts.mass != null) {
        body.mass = opts.mass.?;
        body.inverse_mass = if (opts.mass.? <= 0.0) 0.0 else 1.0 / opts.mass.?;
    }

    if (@hasField(@TypeOf(opts), "restitution") and opts.restitution != null) {
        body.restitution = opts.restitution.?;
    }

    if (@hasField(@TypeOf(opts), "friction") and opts.friction != null) {
        body.friction = opts.friction.?;
    }

    if (@hasField(@TypeOf(opts), "velocity") and opts.velocity != null) {
        body.velocity = opts.velocity.?;
    }

    if (@hasField(@TypeOf(opts), "angular_velocity") and opts.angular_velocity != null) {
        body.angular_velocity = opts.angular_velocity.?;
    }

    if (@hasField(@TypeOf(opts), "initial_force") and opts.initial_force != null) {
        body.force = opts.initial_force.?;
    }

    if (@hasField(@TypeOf(opts), "initial_torque") and opts.initial_torque != null) {
        body.torque = opts.initial_torque.?;
    }

    if (@hasField(@TypeOf(opts), "rotation") and opts.rotation != null) {
        body.rotation = opts.rotation.?;
    }

    return body;
}

// Apply kinematic body options
pub fn applyKinematicBodyOptions(body: *RigidBody, opts: anytype) *RigidBody {
    if (@hasField(@TypeOf(opts), "restitution") and opts.restitution != null) {
        body.restitution = opts.restitution.?;
    }

    if (@hasField(@TypeOf(opts), "friction") and opts.friction != null) {
        body.friction = opts.friction.?;
    }

    if (@hasField(@TypeOf(opts), "velocity") and opts.velocity != null) {
        body.velocity = opts.velocity.?;
    }

    if (@hasField(@TypeOf(opts), "angular_velocity") and opts.angular_velocity != null) {
        body.angular_velocity = opts.angular_velocity.?;
    }

    if (@hasField(@TypeOf(opts), "rotation") and opts.rotation != null) {
        body.rotation = opts.rotation.?;
    }

    return body;
}

// Forward declaration of PhysicsWorld
const PhysicsWorld = @import("../world.zig").PhysicsWorld;
