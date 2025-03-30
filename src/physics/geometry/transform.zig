const std = @import("std");
const Vector2 = @import("vector2.zig").Vector2;
const Rectangle = @import("../shapes/shape.zig").Rectangle;
const Circle = @import("../shapes/shape.zig").Circle;
const AABB = @import("aabb.zig").AABB;

/// COLLISION HELPER FUNCTIONS
/// Find closest point on axis-aligned rectangle to a point
pub fn closestPointOnRectangle(point: Vector2, half_width: f32, half_height: f32) Vector2 {
    const closest_x = std.math.clamp(point.x, -half_width, half_width);
    const closest_y = std.math.clamp(point.y, -half_height, half_height);
    return Vector2.init(closest_x, closest_y);
}

/// Convert a point from world space to a rectangle's local space (accounting for rotation)
pub fn worldToRectangleSpace(world_point: Vector2, rect_position: Vector2, rect_angle: f32) Vector2 {
    // Translate to rectangle space
    const translated = world_point.sub(rect_position);

    // Rotate if necessary
    if (rect_angle != 0.0) {
        return rotateVectorByAngle(translated, -rect_angle);
    }
    return translated;
}

/// Check if a circle and rectangle are colliding (in rectangle's local space)
pub fn circleRectangleOverlap(
    circle_center: Vector2, // Circle center in rectangle's local space
    circle_radius: f32,
    rect_half_width: f32,
    rect_half_height: f32,
) bool {
    // First, expand the rectangle by the circle radius (Minkowski sum approach)
    const expanded_half_width = rect_half_width + circle_radius;
    const expanded_half_height = rect_half_height + circle_radius;

    // Then test if the circle center (now treated as a point) is inside the expanded rectangle
    return (circle_center.x >= -expanded_half_width and
        circle_center.x <= expanded_half_width and
        circle_center.y >= -expanded_half_height and
        circle_center.y <= expanded_half_height);
}

/// Calculate collision info for circle and rectangle (in rectangle's local space)
pub fn circleRectangleCollisionInfo(
    circle_center: Vector2, // Circle center in rectangle's local space
    circle_radius: f32,
    rect_half_width: f32,
    rect_half_height: f32,
) struct { normal: Vector2, depth: f32, is_inside: bool } {
    // Check if circle center is inside the rectangle
    const is_inside = (circle_center.x >= -rect_half_width and
        circle_center.x <= rect_half_width and
        circle_center.y >= -rect_half_height and
        circle_center.y <= rect_half_height);

    // Find closest point on rectangle to circle center
    const closest_point = closestPointOnRectangle(circle_center, rect_half_width, rect_half_height);

    // Vector from closest point to circle center
    const to_circle = circle_center.sub(closest_point);
    const distance_squared = to_circle.lengthSquared();

    // Calculate distance (with small value protection)
    const distance = if (distance_squared > 0.0001) @sqrt(distance_squared) else 0.0001;

    std.debug.print("CircleRect Info: center=({d},{d}), closest=({d},{d}), dist={d}, is_inside={}\n", .{ circle_center.x, circle_center.y, closest_point.x, closest_point.y, distance, is_inside });

    var normal: Vector2 = undefined;
    var depth: f32 = 0;

    if (is_inside) {
        // Case 1: Circle center is inside rectangle
        // Find the nearest edge to push out towards
        const dx_left = circle_center.x + rect_half_width;
        const dx_right = rect_half_width - circle_center.x;
        const dy_top = circle_center.y + rect_half_height;
        const dy_bottom = rect_half_height - circle_center.y;

        // Find minimum penetration axis
        var min_dist = dx_left;
        normal = Vector2.init(-1, 0);

        if (dx_right < min_dist) {
            min_dist = dx_right;
            normal = Vector2.init(1, 0);
        }

        if (dy_top < min_dist) {
            min_dist = dy_top;
            normal = Vector2.init(0, -1);
        }

        if (dy_bottom < min_dist) {
            min_dist = dy_bottom;
            normal = Vector2.init(0, 1);
        }

        // For inside case, penetration depth is edge distance + radius
        depth = min_dist + circle_radius;
        std.debug.print("Inside case: normal=({d},{d}), depth={d}\n", .{ normal.x, normal.y, depth });
    } else if (distance < circle_radius) {
        // Case 2: Circle overlaps rectangle but center is outside
        normal = to_circle.scale(1.0 / distance);
        depth = circle_radius - distance;
        std.debug.print("Outside case: normal=({d},{d}), depth={d}\n", .{ normal.x, normal.y, depth });
    } else {
        // No collision
        normal = Vector2.zero();
        depth = 0.0;
        std.debug.print("No collision case\n", .{});
    }

    // Ensure a minimum depth to prevent numerical issues
    if (depth > 0 and depth < 0.01) {
        depth = 0.01;
    }

    return .{ .normal = normal, .depth = depth, .is_inside = is_inside };
}

/// Provides transformation utilities for physics shapes
pub const ShapeTransform = struct {
    /// Get the corners of a rectangle in world space
    pub fn getRectangleCorners(rect: Rectangle, position: Vector2) [4]Vector2 {
        const half_width = rect.width / 2.0;
        const half_height = rect.height / 2.0;

        if (rect.isAxisAligned()) {
            // Simple case for axis-aligned rectangles
            return [4]Vector2{
                // Top-left
                Vector2.init(position.x - half_width, position.y - half_height),
                // Top-right
                Vector2.init(position.x + half_width, position.y - half_height),
                // Bottom-right
                Vector2.init(position.x + half_width, position.y + half_height),
                // Bottom-left
                Vector2.init(position.x - half_width, position.y + half_height),
            };
        } else {
            // Rotated rectangle
            const sin_val = @sin(rect.angle);
            const cos_val = @cos(rect.angle);

            var corners: [4]Vector2 = undefined;

            // Calculate local corners (centered at 0,0)
            const local_corners = [4]Vector2{
                Vector2.init(-half_width, -half_height), // Top-left
                Vector2.init(half_width, -half_height), // Top-right
                Vector2.init(half_width, half_height), // Bottom-right
                Vector2.init(-half_width, half_height), // Bottom-left
            };

            // Rotate and translate to world position
            for (local_corners, 0..) |corner, i| {
                corners[i] = rotateVector(corner, sin_val, cos_val);
                corners[i] = corners[i].add(position);
            }

            return corners;
        }
    }

    /// Get points evenly distributed on a circle's circumference
    pub fn getCirclePoints(circle: Circle, position: Vector2, count: usize) []Vector2 {
        const angle_step = 2.0 * std.math.pi / @as(f32, @floatFromInt(count));

        // Allocate memory for the points
        var points = std.heap.page_allocator.alloc(Vector2, count) catch |err| {
            std.debug.print("Failed to allocate memory for circle points: {}\n", .{err});
            return &[_]Vector2{};
        };

        // Calculate points
        for (0..count) |i| {
            const angle = @as(f32, @floatFromInt(i)) * angle_step;
            points[i] = Vector2.init(position.x + circle.radius * @cos(angle), position.y + circle.radius * @sin(angle));
        }

        return points;
    }

    /// Calculate AABB from rectangle corners
    pub fn aabbFromCorners(corners: [4]Vector2) AABB {
        var min_x = std.math.inf(f32);
        var min_y = std.math.inf(f32);
        var max_x = -std.math.inf(f32);
        var max_y = -std.math.inf(f32);

        for (corners) |corner| {
            min_x = @min(min_x, corner.x);
            min_y = @min(min_y, corner.y);
            max_x = @max(max_x, corner.x);
            max_y = @max(max_y, corner.y);
        }

        return AABB{
            .min_x = min_x,
            .min_y = min_y,
            .max_x = max_x,
            .max_y = max_y,
        };
    }

    /// Calculate the edge midpoints of a rectangle
    pub fn getRectangleEdgeMidpoints(rect: Rectangle, position: Vector2) [4]Vector2 {
        const half_width = rect.width / 2.0;
        const half_height = rect.height / 2.0;

        if (rect.isAxisAligned()) {
            // Simple case for axis-aligned rectangles
            return [4]Vector2{
                Vector2.init(position.x, position.y - half_height), // Top
                Vector2.init(position.x + half_width, position.y), // Right
                Vector2.init(position.x, position.y + half_height), // Bottom
                Vector2.init(position.x - half_width, position.y), // Left
            };
        } else {
            // Rotated rectangle
            const sin_val = @sin(rect.angle);
            const cos_val = @cos(rect.angle);

            var midpoints: [4]Vector2 = undefined;

            // Calculate local midpoints (centered at 0,0)
            const local_midpoints = [4]Vector2{
                Vector2.init(0, -half_height), // Top
                Vector2.init(half_width, 0), // Right
                Vector2.init(0, half_height), // Bottom
                Vector2.init(-half_width, 0), // Left
            };

            // Rotate and translate to world position
            for (local_midpoints, 0..) |midpoint, i| {
                midpoints[i] = rotateVector(midpoint, sin_val, cos_val);
                midpoints[i] = midpoints[i].add(position);
            }

            return midpoints;
        }
    }
};

/// A helper function for rotating a vector by pre-calculated sin and cos values
pub fn rotateVector(v: Vector2, sin_val: f32, cos_val: f32) Vector2 {
    return Vector2.init(v.x * cos_val - v.y * sin_val, v.x * sin_val + v.y * cos_val);
}

/// A helper function to rotate a vector by an angle
pub fn rotateVectorByAngle(v: Vector2, angle: f32) Vector2 {
    const sin_val = @sin(angle);
    const cos_val = @cos(angle);
    return rotateVector(v, sin_val, cos_val);
}

/// A helper function to transform a local point to world space
pub fn localToWorld(local_point: Vector2, position: Vector2, angle: f32) Vector2 {
    return rotateVectorByAngle(local_point, angle).add(position);
}

/// A helper function to transform a world point to local space
pub fn worldToLocal(world_point: Vector2, position: Vector2, angle: f32) Vector2 {
    const translated = world_point.sub(position);
    return rotateVectorByAngle(translated, -angle); // Rotate in the opposite direction
}
