// Geometry module re-exports
pub const Vector2 = @import("vector2.zig").Vector2;
pub const AABB = @import("aabb.zig").AABB;
pub const ShapeTransform = @import("transform.zig").ShapeTransform;

// Re-export transform utilities
pub const rotateVector = @import("transform.zig").rotateVector;
pub const rotateVectorByAngle = @import("transform.zig").rotateVectorByAngle;
pub const localToWorld = @import("transform.zig").localToWorld;
pub const worldToLocal = @import("transform.zig").worldToLocal;

// Helper functions for geometry
pub fn getCircleNormal(center: Vector2, point: Vector2) Vector2 {
    const dir = point.sub(center);
    return dir.normalize();
}

pub fn getRectangleNormal(edge_index: u8) Vector2 {
    return switch (edge_index) {
        0 => Vector2.init(0, -1), // Top edge normal (up)
        1 => Vector2.init(1, 0), // Right edge normal (right)
        2 => Vector2.init(0, 1), // Bottom edge normal (down)
        3 => Vector2.init(-1, 0), // Left edge normal (left)
        else => Vector2.init(0, 0),
    };
}
