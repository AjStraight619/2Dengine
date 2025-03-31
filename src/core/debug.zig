const std = @import("std");
const rl = @import("raylib");
const physics = @import("../physics/mod.zig");

/// Debug utilities for the ZigEngine
pub const DebugUtils = struct {
    /// Display performance information when requested
    pub fn displayPerformanceInfo(engine: anytype) void {
        const object_count = engine.physics_world.bodies.items.len;
        var sleeping_count: usize = 0;

        // Count sleeping bodies
        for (engine.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic and body.is_sleeping) {
                sleeping_count += 1;
            }
        }

        // Print performance metrics
        std.debug.print("\n--- Performance Info ---\n", .{});
        std.debug.print("Objects: {d}\n", .{object_count});
        std.debug.print("Sleeping: {d}/{d} dynamic objects\n", .{ sleeping_count, countDynamicBodies(engine) });
        std.debug.print("FPS: {d}\n", .{rl.getFPS()});
        std.debug.print("Target physics rate: {d}Hz\n", .{@round(1.0 / engine.fixed_time_step)});
        std.debug.print("Collisions per frame: {d}\n", .{engine.physics_world.collision_count});

        // Force detailed diagnostics for next frame
        engine.physics_world.forceDiagnostics();
    }

    /// Process debug key inputs (P key for performance, etc.)
    pub fn processDebugKeys(engine: anytype) void {
        // Performance info (P key)
        if (rl.isKeyPressed(rl.KeyboardKey.p)) {
            displayPerformanceInfo(engine);
        }

        // Toggle debug mode (D key)
        if (rl.isKeyPressed(rl.KeyboardKey.d)) {
            engine.debug_mode = !engine.debug_mode;
            engine.physics_renderer.setDebugMode(engine.debug_mode);
            std.debug.print("Debug mode: {}\n", .{engine.debug_mode});
        }

        // Toggle debug rendering features when in debug mode
        if (engine.debug_mode) {
            // Toggle velocity vectors (V key)
            if (rl.isKeyPressed(rl.KeyboardKey.v)) {
                engine.physics_renderer.draw_velocities = !engine.physics_renderer.draw_velocities;
                std.debug.print("Velocity vectors: {}\n", .{engine.physics_renderer.draw_velocities});
            }

            // Toggle force vectors (F key)
            if (rl.isKeyPressed(rl.KeyboardKey.f)) {
                engine.physics_renderer.draw_forces = !engine.physics_renderer.draw_forces;
                std.debug.print("Force vectors: {}\n", .{engine.physics_renderer.draw_forces});
            }

            // Toggle normal vectors (N key)
            if (rl.isKeyPressed(rl.KeyboardKey.n)) {
                engine.physics_renderer.draw_normals = !engine.physics_renderer.draw_normals;
                std.debug.print("Normal vectors: {}\n", .{engine.physics_renderer.draw_normals});
            }

            // Toggle AABBs (B key)
            if (rl.isKeyPressed(rl.KeyboardKey.b)) {
                engine.physics_renderer.draw_aabbs = !engine.physics_renderer.draw_aabbs;
                std.debug.print("AABBs: {}\n", .{engine.physics_renderer.draw_aabbs});
            }
        }

        // Toggle overlay (O key)
        if (rl.isKeyPressed(rl.KeyboardKey.o)) {
            engine.show_debug_overlay = !engine.show_debug_overlay;
            std.debug.print("Debug overlay: {}\n", .{engine.show_debug_overlay});
        }
    }

    /// Count total dynamic bodies in the physics world
    fn countDynamicBodies(engine: anytype) usize {
        var count: usize = 0;
        for (engine.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                count += 1;
            }
        }
        return count;
    }

    /// Draw on-screen debug information
    pub fn drawDebugInfo(engine: anytype, x: i32, y: i32, color: rl.Color) void {
        const object_count = engine.physics_world.bodies.items.len;
        const dynamic_count = countDynamicBodies(engine);
        var sleeping_count: usize = 0;

        // Count sleeping bodies
        for (engine.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic and body.is_sleeping) {
                sleeping_count += 1;
            }
        }

        // Draw performance metrics as text overlay
        var y_pos = y;
        const line_height = 20;

        // Use direct text for raylib since it expects null-terminated strings
        var text_buf: [100]u8 = undefined;

        // FPS display
        _ = std.fmt.bufPrintZ(&text_buf, "FPS: {d}", .{rl.getFPS()}) catch {
            rl.drawText("FPS: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Objects count
        _ = std.fmt.bufPrintZ(&text_buf, "Objects: {d} ({d} dynamic, {d} sleeping)", .{ object_count, dynamic_count, sleeping_count }) catch {
            rl.drawText("Objects: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Physics rate
        _ = std.fmt.bufPrintZ(&text_buf, "Physics rate: {d}Hz", .{@round(1.0 / engine.fixed_time_step)}) catch {
            rl.drawText("Physics rate: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Collisions
        _ = std.fmt.bufPrintZ(&text_buf, "Collisions: {d}", .{engine.physics_world.collision_count}) catch {
            rl.drawText("Collisions: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);

        // Debug visualization status (if in debug mode)
        if (engine.debug_mode) {
            y_pos += line_height;

            // Draw current debug visualization settings
            _ = std.fmt.bufPrintZ(&text_buf, "Debug Mode (D): ON", .{}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, rl.Color.green);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  Velocities (V): {s}", .{if (engine.physics_renderer.draw_velocities) "ON" else "OFF"}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  Forces (F): {s}", .{if (engine.physics_renderer.draw_forces) "ON" else "OFF"}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  Normals (N): {s}", .{if (engine.physics_renderer.draw_normals) "ON" else "OFF"}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  AABBs (B): {s}", .{if (engine.physics_renderer.draw_aabbs) "ON" else "OFF"}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        } else {
            y_pos += line_height;
            _ = std.fmt.bufPrintZ(&text_buf, "Debug Mode (D): OFF", .{}) catch return;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        }
    }
};
