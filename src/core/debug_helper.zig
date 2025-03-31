const std = @import("std");
const rl = @import("raylib");
const debug = @import("debug.zig");
const ze = @import("../mod.zig");

/// Simple debug helper for examples
/// This module provides convenient functions to add standard debug features to any example
pub const ExampleDebugHelper = struct {
    // Engine reference
    engine: *ze.Engine,

    // Debug flags
    show_forces: bool = true,
    show_velocities: bool = true,

    // Create a new debug helper
    pub fn init(engine: *ze.Engine) ExampleDebugHelper {
        return ExampleDebugHelper{
            .engine = engine,
        };
    }

    // Initialize default debug settings for an example
    pub fn setupDebugDefaults(self: *ExampleDebugHelper) void {
        // Enable debug mode by default in example
        self.engine.debug_mode = true;
        self.engine.debug_system.debug_mode = true;
        self.engine.physics_renderer.setDebugMode(true);

        // Show basic debug info
        self.engine.show_debug_overlay = true;

        // Always show forces in examples by default
        self.engine.debug_system.draw_forces = true;
        self.engine.physics_renderer.draw_forces = true;

        // Always show velocities in examples by default
        self.engine.debug_system.draw_velocities = true;
        self.engine.physics_renderer.draw_velocities = true;
    }

    // Process example-specific input
    pub fn processInput(self: *ExampleDebugHelper) void {
        // Force visualization - F key
        if (rl.isKeyPressed(rl.KeyboardKey.f)) {
            self.toggleForceVisualization();
        }

        // Velocity visualization - V key
        if (rl.isKeyPressed(rl.KeyboardKey.v)) {
            self.toggleVelocityVisualization();
        }

        // Reset forces - R key
        if (rl.isKeyPressed(rl.KeyboardKey.r)) {
            self.resetAllForces();
        }
    }

    // Toggle force visualization
    pub fn toggleForceVisualization(self: *ExampleDebugHelper) void {
        self.show_forces = !self.show_forces;
        self.engine.debug_system.draw_forces = self.show_forces;
        self.engine.physics_renderer.draw_forces = self.show_forces;
        std.debug.print("Force visualization: {}\n", .{self.show_forces});
    }

    // Toggle velocity visualization
    pub fn toggleVelocityVisualization(self: *ExampleDebugHelper) void {
        self.show_velocities = !self.show_velocities;
        self.engine.debug_system.draw_velocities = self.show_velocities;
        self.engine.physics_renderer.draw_velocities = self.show_velocities;
        std.debug.print("Velocity visualization: {}\n", .{self.show_velocities});
    }

    // Reset all forces on dynamic bodies
    pub fn resetAllForces(self: *ExampleDebugHelper) void {
        for (self.engine.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.force = ze.physics.Vector2.zero();
                body.torque = 0.0;
            }
        }
        std.debug.print("Reset all forces\n", .{});
    }

    // Apply a test force to all dynamic bodies
    pub fn applyTestForce(self: *ExampleDebugHelper, force: ze.math.Vector2) void {
        for (self.engine.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(force);
            }
        }
        std.debug.print("Applied force: ({d:.2}, {d:.2})\n", .{ force.x, force.y });
    }

    // Draw common debug controls help text
    pub fn drawDebugControls(self: *ExampleDebugHelper, x: i32, y: i32) void {
        rl.drawText("Debug Controls:", x, y, 20, rl.Color.dark_gray);
        rl.drawText("D: Debug Mode | F: Forces | V: Velocities | O: Overlay", x, y + 25, 15, rl.Color.dark_gray);
        rl.drawText("R: Reset Forces | P: Performance | +/-: Force Scale", x, y + 45, 15, rl.Color.dark_gray);

        // Show current debug state
        const state_y = y + 75;
        rl.drawText("Current Debug State:", x, state_y, 18, rl.Color.black);

        var state_text: [64]u8 = undefined;
        _ = std.fmt.bufPrintZ(&state_text, "Forces: {s} | Velocities: {s}", .{
            if (self.show_forces) "ON" else "OFF",
            if (self.show_velocities) "ON" else "OFF",
        }) catch "Error";

        rl.drawText(@ptrCast(&state_text), x, state_y + 25, 15, rl.Color.dark_green);
    }

    // Helper to draw force vectors on objects
    pub fn drawForceOnBody(body: *ze.physics.RigidBody, force: ze.math.Vector2, color: rl.Color) void {
        const pos_x = @as(i32, @intFromFloat(body.position.x));
        const pos_y = @as(i32, @intFromFloat(body.position.y));

        // Scale force for visualization
        const scale: f32 = 0.05;
        const scaled_force = force.scale(scale);
        const end_x = @as(i32, @intFromFloat(body.position.x + scaled_force.x));
        const end_y = @as(i32, @intFromFloat(body.position.y + scaled_force.y));

        // Draw the force vector
        rl.drawLine(pos_x, pos_y, end_x, end_y, color);

        // Draw an arrowhead
        debug.drawArrowhead(end_x, end_y, force.normalize(), 10.0, color);

        // Draw force magnitude
        var text_buf: [32]u8 = undefined;
        _ = std.fmt.bufPrintZ(&text_buf, "{d:.1}", .{force.length()}) catch "??";
        rl.drawText(@ptrCast(&text_buf), end_x + 5, end_y - 10, 15, color);
    }
};

// Helper function to draw an arrowhead (copied from debug_renderer for convenience)
pub fn drawArrowhead(x: i32, y: i32, direction: ze.math.Vector2, size: f32, color: rl.Color) void {
    const arrow_angle = std.math.pi / 6.0; // 30 degrees
    const pos = ze.math.Vector2.init(@floatFromInt(x), @floatFromInt(y));

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
