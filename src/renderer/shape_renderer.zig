const std = @import("std");
const rl = @import("raylib");
const phys = @import("../physics/mod.zig");

/// Handles rendering of physics shapes (circles, rectangles)
pub const ShapeRenderer = struct {
    // Color settings
    circle_color: rl.Color = rl.Color.red,
    rectangle_color: rl.Color = rl.Color.blue,
    static_color: rl.Color = rl.Color.gray,
    dynamic_color: rl.Color = rl.Color.red,
    circle_segments: i32 = 36, // Smoothness of circle drawing

    // Draw a circle shape
    pub fn drawCircle(self: ShapeRenderer, position: phys.Vector2, circle: phys.Circle, is_static: bool) void {
        // Convert coordinates to integer for drawing
        const pos_x = @as(i32, @intFromFloat(position.x));
        const pos_y = @as(i32, @intFromFloat(position.y));

        // Choose color based on whether body is static or dynamic
        const shape_color = if (is_static) self.static_color else self.dynamic_color;

        // Convert radius to integer for drawing
        const radius = @as(i32, @intFromFloat(circle.radius));

        // Draw filled circle
        rl.drawCircle(pos_x, pos_y, @floatFromInt(radius), shape_color);

        // Draw outline for better visibility
        rl.drawCircleLines(pos_x, pos_y, @floatFromInt(radius), rl.Color.black);
    }

    // Draw a rectangle shape
    pub fn drawRectangle(self: ShapeRenderer, position: phys.Vector2, rectangle: phys.Rectangle, is_static: bool) void {
        // Convert coordinates to integer for drawing
        const pos_x = @as(i32, @intFromFloat(position.x));
        const pos_y = @as(i32, @intFromFloat(position.y));

        // Choose color based on whether body is static or dynamic
        const shape_color = if (is_static) self.static_color else self.dynamic_color;

        // Calculate dimensions
        const half_width = rectangle.width / 2.0;
        const half_height = rectangle.height / 2.0;

        if (rectangle.isAxisAligned()) {
            // Regular axis-aligned rectangle
            const rect_x = pos_x - @as(i32, @intFromFloat(half_width));
            const rect_y = pos_y - @as(i32, @intFromFloat(half_height));
            const rect_width = @as(i32, @intFromFloat(rectangle.width));
            const rect_height = @as(i32, @intFromFloat(rectangle.height));

            // Draw filled rectangle
            rl.drawRectangle(rect_x, rect_y, rect_width, rect_height, shape_color);

            // Draw outline for better visibility
            rl.drawRectangleLines(rect_x, rect_y, rect_width, rect_height, rl.Color.black);
        } else {
            // Rotated rectangle
            const pos_x_f = @as(f32, @floatFromInt(pos_x));
            const pos_y_f = @as(f32, @floatFromInt(pos_y));

            // Draw rotated rectangle using raylib's Pro function
            rl.drawRectanglePro(rl.Rectangle{
                .x = pos_x_f,
                .y = pos_y_f,
                .width = rectangle.width,
                .height = rectangle.height,
            }, rl.Vector2{ .x = half_width, .y = half_height }, rectangle.angle * 180.0 / std.math.pi, // Convert radians to degrees
                shape_color);

            // Get and draw the corners for the outline
            const transform = phys.geometry.ShapeTransform;
            const corners = transform.getRectangleCorners(rectangle, position);

            // Draw the outline
            for (0..4) |i| {
                const j = (i + 1) % 4;
                rl.drawLine(@intFromFloat(corners[i].x), @intFromFloat(corners[i].y), @intFromFloat(corners[j].x), @intFromFloat(corners[j].y), rl.Color.black);
            }
        }
    }

    // Draw any physics body based on its shape type
    pub fn drawBody(self: ShapeRenderer, body: *phys.RigidBody) void {
        const is_static = body.body_type == .static;

        switch (body.shape) {
            .circle => |circle| self.drawCircle(body.position, circle, is_static),
            .rectangle => |rect| self.drawRectangle(body.position, rect, is_static),
        }
    }
};
