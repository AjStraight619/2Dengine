const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Shape types for physics bodies
pub const Shape = union(enum) {
    circle: Circle,
    rectangle: Rectangle,
};

/// Circle shape definition
pub const Circle = struct {
    radius: f32,
};

/// Rectangle shape definition
pub const Rectangle = struct {
    width: f32,
    height: f32,
    angle: f32 = 0.0, // rotation in radians, default 0

    pub fn isAxisAligned(self: Rectangle) bool {
        return @abs(self.angle) < 0.0001;
    }
};
