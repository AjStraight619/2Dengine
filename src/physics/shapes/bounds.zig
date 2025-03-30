const Vector2 = @import("../geometry/vector2.zig").Vector2;
const AABB = @import("../geometry/aabb.zig").AABB;
const Shape = @import("shape.zig").Shape;
const Circle = @import("shape.zig").Circle;
const Rectangle = @import("shape.zig").Rectangle;
const transform = @import("../geometry/transform.zig");

/// AABB calculation utility for different shape types
pub const BoundsCalculator = struct {
    /// Calculate AABB for any shape
    pub fn computeAABB(shape: Shape, position: Vector2) AABB {
        return switch (shape) {
            .circle => |c| computeCircleAABB(c, position),
            .rectangle => |r| computeRectangleAABB(r, position),
        };
    }

    /// Calculate AABB for a circle
    pub fn computeCircleAABB(circle: Circle, position: Vector2) AABB {
        return AABB{
            .min_x = position.x - circle.radius,
            .min_y = position.y - circle.radius,
            .max_x = position.x + circle.radius,
            .max_y = position.y + circle.radius,
        };
    }

    /// Calculate AABB for a rectangle
    pub fn computeRectangleAABB(rect: Rectangle, position: Vector2) AABB {
        if (rect.isAxisAligned()) {
            // Simple case for axis-aligned rectangle
            return AABB{
                .min_x = position.x - rect.width / 2.0,
                .min_y = position.y - rect.height / 2.0,
                .max_x = position.x + rect.width / 2.0,
                .max_y = position.y + rect.height / 2.0,
            };
        } else {
            // Use the transform utilities for rotated rectangles
            const corners = transform.ShapeTransform.getRectangleCorners(rect, position);
            return transform.ShapeTransform.aabbFromCorners(corners);
        }
    }

    /// Check if a point is inside an AABB
    pub fn pointInAABB(aabb: AABB, point: Vector2) bool {
        return point.x >= aabb.min_x and
            point.x <= aabb.max_x and
            point.y >= aabb.min_y and
            point.y <= aabb.max_y;
    }

    /// Check if two AABBs overlap
    pub fn aabbOverlap(a: AABB, b: AABB) bool {
        return a.min_x <= b.max_x and
            a.max_x >= b.min_x and
            a.min_y <= b.max_y and
            a.max_y >= b.min_y;
    }
};
