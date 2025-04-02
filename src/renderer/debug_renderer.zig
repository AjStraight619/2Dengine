const std = @import("std");
const rl = @import("raylib");
const phys = @import("../physics/mod.zig");
const geom = @import("../physics/geometry/mod.zig");

/// Handles rendering of debug information like forces, velocities, normals, and AABBs
pub const DebugRenderer = struct {
    // Configuration
    velocity_scale: f32 = 3.0,
    force_scale: f32 = 0.1,
    normal_scale: f32 = 25.0,
    min_vector_length: f32 = 0.01,

    // Colors
    velocity_color: rl.Color = rl.Color.blue,
    force_color: rl.Color = rl.Color.green,
    normal_color: rl.Color = rl.Color.magenta,
    aabb_color: rl.Color = rl.Color{ .r = 255, .g = 255, .b = 0, .a = 120 },

    // Draw an AABB for a physics body
    pub fn drawAABB(self: DebugRenderer, body: *phys.RigidBody) void {
        // Get the AABB from the body (already calculated by physics system)
        const aabb = body.aabb;

        // Calculate width and height
        const width = aabb.max_x - aabb.min_x;
        const height = aabb.max_y - aabb.min_y;

        // Draw a semi-transparent rectangle
        rl.drawRectangle(@intFromFloat(aabb.min_x), @intFromFloat(aabb.min_y), @intFromFloat(width), @intFromFloat(height), self.aabb_color);

        // Draw the outline
        rl.drawRectangleLines(@intFromFloat(aabb.min_x), @intFromFloat(aabb.min_y), @intFromFloat(width), @intFromFloat(height), rl.Color.black);
    }

    // Draw velocity vector for a body
    pub fn drawVelocityVector(self: DebugRenderer, body: *phys.RigidBody) void {
        if (body.velocity.length() > 0.1) {
            const pos_x = @as(i32, @intFromFloat(body.position.x));
            const pos_y = @as(i32, @intFromFloat(body.position.y));

            // Calculate end point of vector using physics Vector2
            const velocity_vec = body.velocity.scale(self.velocity_scale);
            const end_pos = body.position.add(velocity_vec);

            const end_x = @as(i32, @intFromFloat(end_pos.x));
            const end_y = @as(i32, @intFromFloat(end_pos.y));

            rl.drawLine(pos_x, pos_y, end_x, end_y, self.velocity_color);

            // Draw arrowhead
            const normalized_vel = body.velocity.normalize();
            drawArrowhead(end_x, end_y, normalized_vel, 10.0, self.velocity_color);
        }
    }

    // Draw force vector for a body
    pub fn drawForceVector(self: DebugRenderer, body: *phys.RigidBody) void {
        if (body.force.length() > self.min_vector_length) {
            const pos_x = @as(i32, @intFromFloat(body.position.x));
            const pos_y = @as(i32, @intFromFloat(body.position.y));

            var force_vec = body.force.scale(self.force_scale);

            // Ensure force vector has a minimum visible length
            const min_visual_length: f32 = 10.0; // Minimum pixel length for visibility
            const force_len = force_vec.length();
            if (force_len < min_visual_length and force_len > 0) {
                force_vec = force_vec.scale(min_visual_length / force_len);
            }

            const end_pos = body.position.add(force_vec);

            const end_x = @as(i32, @intFromFloat(end_pos.x));
            const end_y = @as(i32, @intFromFloat(end_pos.y));

            rl.drawLine(pos_x, pos_y, end_x, end_y, self.force_color);

            const normalized_force = body.force.normalize();
            drawArrowhead(end_x, end_y, normalized_force, 10.0, self.force_color);

            // Draw force magnitude text with background for better readability
            var force_text_buf: [32]u8 = undefined;
            const force_text = std.fmt.bufPrintZ(&force_text_buf, "F: {d:.0}", .{body.force.length()}) catch "?";

            // Draw background for text
            const text_bg_color = rl.Color{ .r = 230, .g = 255, .b = 230, .a = 220 };
            const text_width = 50;
            const text_height = 20;
            rl.drawRectangle(end_x + 2, end_y - 10, text_width, text_height, text_bg_color);

            // Draw text with black for better visibility
            rl.drawText(@ptrCast(force_text), end_x + 5, end_y - 5, 15, rl.Color.black);
        }
    }

    // Draw normal vectors for a body
    pub fn drawNormalVectors(self: DebugRenderer, body: *phys.RigidBody) void {
        const pos = body.position;

        switch (body.shape) {
            .circle => |circle| {
                // For circles, draw normals at 8 points around the circumference
                const segments = 8;
                const angle_step = 2.0 * std.math.pi / @as(f32, @floatFromInt(segments));

                for (0..segments) |i| {
                    const angle = @as(f32, @floatFromInt(i)) * angle_step;

                    // Point on circle
                    const point_on_circle = phys.Vector2.init(pos.x + circle.radius * @cos(angle), pos.y + circle.radius * @sin(angle));

                    // Calculate normal at this point
                    const normal = geom.getCircleNormal(pos, point_on_circle);
                    self.drawNormalVector(point_on_circle, normal);
                }
            },
            .rectangle => |rect| {
                const half_width = rect.width / 2.0;
                const half_height = rect.height / 2.0;

                if (rect.isAxisAligned()) {
                    // For axis-aligned rectangles, draw normals at the center of each edge
                    // Center points of each edge
                    const edge_centers = [4]phys.Vector2{
                        // Top edge
                        phys.Vector2.init(pos.x, pos.y - half_height),
                        // Right edge
                        phys.Vector2.init(pos.x + half_width, pos.y),
                        // Bottom edge
                        phys.Vector2.init(pos.x, pos.y + half_height),
                        // Left edge
                        phys.Vector2.init(pos.x - half_width, pos.y),
                    };

                    // Draw normal at each edge center
                    for (0..4) |i| {
                        const normal = geom.getRectangleNormal(@intCast(i));
                        self.drawNormalVector(edge_centers[i], normal);
                    }
                } else {
                    // For rotated rectangles, use transformation utilities
                    const transform = phys.geometry.ShapeTransform;
                    const edge_midpoints = transform.getRectangleEdgeMidpoints(rect, pos);

                    // Draw normal at each edge
                    for (0..4) |i| {
                        // Get non-rotated normal and rotate it
                        const base_normal = phys.geometry.getRectangleNormal(@intCast(i));
                        const rotated_normal = phys.geometry.rotateVectorByAngle(base_normal, rect.angle);

                        self.drawNormalVector(edge_midpoints[i], rotated_normal);
                    }
                }
            },
        }
    }

    // Helper to draw a single normal vector
    fn drawNormalVector(self: DebugRenderer, start: phys.Vector2, normal: phys.Vector2) void {
        const scaled_normal = normal.scale(self.normal_scale);
        const end = start.add(scaled_normal);

        rl.drawLine(@intFromFloat(start.x), @intFromFloat(start.y), @intFromFloat(end.x), @intFromFloat(end.y), self.normal_color);

        // Draw arrowhead
        drawArrowhead(@intFromFloat(end.x), @intFromFloat(end.y), normal, 10.0, self.normal_color);
    }

    // Draw a coordinate grid
    pub fn drawGrid(width: i32, height: i32, cell_size: i32, color: rl.Color) void {
        const fade_amount: f32 = 0.4;

        // Draw horizontal lines
        var y: i32 = 0;
        while (y <= height) : (y += cell_size) {
            rl.drawLine(0, y, width, y, rl.fade(color, fade_amount));
        }

        // Draw vertical lines
        var x: i32 = 0;
        while (x <= width) : (x += cell_size) {
            rl.drawLine(x, 0, x, height, rl.fade(color, fade_amount));
        }
    }
};

// Draw an arrowhead at the specified position and angle
pub fn drawArrowhead(x: i32, y: i32, direction: phys.Vector2, size: f32, color: rl.Color) void {
    const arrow_angle = std.math.pi / 6.0; // 30 degrees
    const pos = phys.Vector2.init(@floatFromInt(x), @floatFromInt(y));

    // Calculate arrow vectors by rotating the direction vector
    // First arrow at -150 degrees (counter-clockwise from direction)
    const arrow1 = direction.rotate(-std.math.pi + arrow_angle).scale(size);

    // Second arrow at 150 degrees (clockwise from direction)
    const arrow2 = direction.rotate(std.math.pi - arrow_angle).scale(size);

    // Calculate end points
    const point1 = pos.add(arrow1);
    const point2 = pos.add(arrow2);

    // Draw the arrowhead lines
    rl.drawLine(x, y, @intFromFloat(point1.x), @intFromFloat(point1.y), color);
    rl.drawLine(x, y, @intFromFloat(point2.x), @intFromFloat(point2.y), color);
}
