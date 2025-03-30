const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Options for static circle bodies
pub const StaticCircleOptions = struct {
    position: Vector2,
    radius: f32,
    restitution: ?f32 = null,
    friction: ?f32 = null,
};

/// Options for static rectangle bodies
pub const StaticRectangleOptions = struct {
    position: Vector2,
    width: f32,
    height: f32,
    angle: f32 = 0.0,
    restitution: ?f32 = null,
    friction: ?f32 = null,
};

/// Options for dynamic circle bodies
pub const DynamicCircleOptions = struct {
    position: Vector2,
    radius: f32,
    mass: ?f32 = null,
    restitution: ?f32 = null,
    friction: ?f32 = null,
    velocity: ?Vector2 = null,
    angular_velocity: ?f32 = null,
    initial_force: ?Vector2 = null,
    initial_torque: ?f32 = null,
    rotation: ?f32 = null,
};

/// Options for dynamic rectangle bodies
pub const DynamicRectangleOptions = struct {
    position: Vector2,
    width: f32,
    height: f32,
    angle: f32 = 0.0,
    mass: ?f32 = null,
    restitution: ?f32 = null,
    friction: ?f32 = null,
    velocity: ?Vector2 = null,
    angular_velocity: ?f32 = null,
    initial_force: ?Vector2 = null,
    initial_torque: ?f32 = null,
    rotation: ?f32 = null,
};

/// Options for kinematic circle bodies
pub const KinematicCircleOptions = struct {
    position: Vector2,
    radius: f32,
    restitution: ?f32 = null,
    friction: ?f32 = null,
    velocity: ?Vector2 = null,
    angular_velocity: ?f32 = null,
    rotation: ?f32 = null,
};

/// Options for kinematic rectangle bodies
pub const KinematicRectangleOptions = struct {
    position: Vector2,
    width: f32,
    height: f32,
    angle: f32 = 0.0,
    restitution: ?f32 = null,
    friction: ?f32 = null,
    velocity: ?Vector2 = null,
    angular_velocity: ?f32 = null,
    rotation: ?f32 = null,
};
