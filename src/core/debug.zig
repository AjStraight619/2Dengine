const std = @import("std");
const rl = @import("raylib");
const physics = @import("../physics/mod.zig");
const input = @import("../input/mod.zig");

/// Unified Debug System for the ZigEngine
pub const DebugSystem = struct {
    // Debug visualization flags
    debug_mode: bool = false,
    show_debug_overlay: bool = false,
    draw_velocities: bool = false,
    draw_forces: bool = false,
    draw_normals: bool = false,
    draw_aabbs: bool = false,
    force_diagnostics: bool = false,
    collision_logging: bool = false,

    // Force visualization settings
    force_visualization_scale: f32 = 1.0,
    force_min_display_value: f32 = 0.1,

    // Performance tracking
    fps_history: [60]i32 = [_]i32{0} ** 60,
    fps_history_index: usize = 0,

    // Layout management
    main_debug_x: i32 = 10, // Default X position for main debug panel
    main_debug_y: i32 = 10, // Default Y position for main debug panel
    help_text_y: i32 = 0, // Will be calculated based on debug info height
    physics_info_y: i32 = 0, // Will be calculated based on debug info height

    /// Initialize the debug system
    pub fn init() DebugSystem {
        return DebugSystem{};
    }

    /// Setup default debug input bindings
    pub fn setupInputBindings(self: *DebugSystem, input_manager: *input.InputManager) !void {
        _ = self; // Unused currently, but might be useful later

        // Debug mode toggle - D key
        try input_manager.bindInput("debug_toggle", .{ .keyboard = rl.KeyboardKey.d });

        // Debug visualization features
        try input_manager.bindInput("debug_overlay", .{ .keyboard = rl.KeyboardKey.o });
        try input_manager.bindInput("debug_velocities", .{ .keyboard = rl.KeyboardKey.v });
        try input_manager.bindInput("debug_forces", .{ .keyboard = rl.KeyboardKey.f });
        try input_manager.bindInput("debug_normals", .{ .keyboard = rl.KeyboardKey.n });
        try input_manager.bindInput("debug_aabbs", .{ .keyboard = rl.KeyboardKey.b });
        try input_manager.bindInput("debug_diagnostics", .{ .keyboard = rl.KeyboardKey.i });
        try input_manager.bindInput("debug_performance", .{ .keyboard = rl.KeyboardKey.p });
        try input_manager.bindInput("debug_logging", .{ .keyboard = rl.KeyboardKey.l });

        // Force scale controls
        try input_manager.bindInput("debug_force_scale_up", .{ .keyboard = rl.KeyboardKey.equal });
        try input_manager.bindInput("debug_force_scale_down", .{ .keyboard = rl.KeyboardKey.minus });
    }

    /// Register debug action callbacks
    pub fn setupCallbacks(self: *DebugSystem, input_manager: *input.InputManager, engine: anytype) !void {
        const context: *anyopaque = @ptrCast(self);
        const engine_context: *anyopaque = @ptrCast(engine);

        // Register callbacks for debug actions
        try input_manager.registerAction("debug_toggle", onToggleDebug, engine_context, .pressed);
        try input_manager.registerAction("debug_overlay", onToggleOverlay, engine_context, .pressed);
        try input_manager.registerAction("debug_velocities", onToggleVelocities, context, .pressed);
        try input_manager.registerAction("debug_forces", onToggleForces, context, .pressed);
        try input_manager.registerAction("debug_normals", onToggleNormals, context, .pressed);
        try input_manager.registerAction("debug_aabbs", onToggleAABBs, context, .pressed);
        try input_manager.registerAction("debug_diagnostics", onToggleDiagnostics, context, .pressed);
        try input_manager.registerAction("debug_performance", onShowPerformance, engine_context, .pressed);
        try input_manager.registerAction("debug_logging", onToggleLogging, engine_context, .pressed);

        // Force scale callbacks
        try input_manager.registerAction("debug_force_scale_up", onIncreaseForceScale, context, .pressed);
        try input_manager.registerAction("debug_force_scale_down", onDecreaseForceScale, context, .pressed);
    }

    /// Process all debug inputs manually (use when not using the InputManager)
    pub fn processDebugKeys(self: *DebugSystem, eng: anytype) void {
        // Toggle debug mode (D key)
        if (rl.isKeyPressed(rl.KeyboardKey.d)) {
            if (eng.debug_mode) {
                // Currently ON, turning OFF
                eng.debug_mode = false;
                eng.physics_renderer.debug_mode = false;
                self.debug_mode = false;

                // Turn off all visualization flags
                eng.physics_renderer.draw_forces = false;
                eng.physics_renderer.draw_velocities = false;
                eng.physics_renderer.draw_normals = false;
                eng.physics_renderer.draw_aabbs = false;
                self.draw_forces = false;
                self.draw_velocities = false;
                self.draw_normals = false;
                self.draw_aabbs = false;

                std.debug.print("Debug mode OFF\n", .{});
            } else {
                // Currently OFF, turning ON
                eng.debug_mode = true;
                eng.physics_renderer.debug_mode = true;
                self.debug_mode = true;

                // Turn on all visualization flags
                eng.physics_renderer.draw_forces = true;
                eng.physics_renderer.draw_velocities = true;
                eng.physics_renderer.draw_normals = true;
                eng.physics_renderer.draw_aabbs = true;
                self.draw_forces = true;
                self.draw_velocities = true;
                self.draw_normals = true;
                self.draw_aabbs = true;

                std.debug.print("Debug mode ON\n", .{});
            }
        }

        // Toggle debug overlay (O key) - available regardless of debug mode
        if (rl.isKeyPressed(rl.KeyboardKey.o)) {
            eng.show_debug_overlay = !eng.show_debug_overlay;
            std.debug.print("Debug overlay: {}\n", .{eng.show_debug_overlay});
        }

        // Conditional debug features only available in debug mode
        if (eng.debug_mode) {
            // Toggle velocity vectors (V key)
            if (rl.isKeyPressed(rl.KeyboardKey.v)) {
                // Toggle both flags to ensure they stay in sync
                self.draw_velocities = !self.draw_velocities;
                eng.physics_renderer.draw_velocities = self.draw_velocities;
                std.debug.print("Velocity vectors: {}\n", .{self.draw_velocities});
            }

            // Toggle force vectors (F key)
            if (rl.isKeyPressed(rl.KeyboardKey.f)) {
                // Toggle both flags to ensure they stay in sync
                self.draw_forces = !self.draw_forces;
                eng.physics_renderer.draw_forces = self.draw_forces;
                std.debug.print("Force vectors: {}\n", .{self.draw_forces});
            }

            // Toggle normal vectors (N key)
            if (rl.isKeyPressed(rl.KeyboardKey.n)) {
                // Toggle both flags to ensure they stay in sync
                self.draw_normals = !self.draw_normals;
                eng.physics_renderer.draw_normals = self.draw_normals;
                std.debug.print("Normal vectors: {}\n", .{self.draw_normals});
            }

            // Toggle AABBs (B key)
            if (rl.isKeyPressed(rl.KeyboardKey.b)) {
                // Toggle both flags to ensure they stay in sync
                self.draw_aabbs = !self.draw_aabbs;
                eng.physics_renderer.draw_aabbs = self.draw_aabbs;
                std.debug.print("AABBs: {}\n", .{self.draw_aabbs});
            }

            // Toggle diagnostics (I key)
            if (rl.isKeyPressed(rl.KeyboardKey.i)) {
                self.force_diagnostics = !self.force_diagnostics;
                std.debug.print("Force diagnostics: {}\n", .{self.force_diagnostics});
            }

            // Toggle collision logging (L key)
            if (rl.isKeyPressed(rl.KeyboardKey.l)) {
                eng.physics_world.setCollisionLogging(!eng.physics_world.collision_logging);
                std.debug.print("Collision logging: {}\n", .{eng.physics_world.collision_logging});
            }

            // Performance info (P key)
            if (rl.isKeyPressed(rl.KeyboardKey.p)) {
                DebugSystem.displayPerformanceInfo(eng);
            }

            // Adjust force visualization scale
            if (rl.isKeyPressed(rl.KeyboardKey.equal)) {
                self.force_visualization_scale *= 1.2;
                eng.physics_renderer.debug_renderer.force_scale = 0.05 * self.force_visualization_scale;
                std.debug.print("Force scale: {d:.2}x\n", .{self.force_visualization_scale});
            }

            if (rl.isKeyPressed(rl.KeyboardKey.minus)) {
                self.force_visualization_scale /= 1.2;
                eng.physics_renderer.debug_renderer.force_scale = 0.05 * self.force_visualization_scale;
                std.debug.print("Force scale: {d:.2}x\n", .{self.force_visualization_scale});
            }
        }

        // Explicit call to Engine's sync method to ensure all flags are properly synced
        eng.syncDebugState();
    }

    /// Update the debug system state (call once per frame)
    pub fn update(self: *DebugSystem) void {
        // Update FPS history for performance monitoring
        self.fps_history[self.fps_history_index] = rl.getFPS();
        self.fps_history_index = (self.fps_history_index + 1) % self.fps_history.len;
    }

    /// Synchronize debug system with engine and renderer
    pub fn syncWithEngine(self: *DebugSystem, engine: anytype) void {
        // Sync debug flags
        engine.debug_mode = self.debug_mode;
        engine.show_debug_overlay = self.show_debug_overlay;
        engine.physics_renderer.draw_velocities = self.draw_velocities;
        engine.physics_renderer.draw_forces = self.draw_forces;
        engine.physics_renderer.draw_normals = self.draw_normals;
        engine.physics_renderer.draw_aabbs = self.draw_aabbs;
        engine.physics_world.setCollisionLogging(self.collision_logging);

        // Sync force visualization scale
        engine.physics_renderer.debug_renderer.force_scale = 0.05 * self.force_visualization_scale;
    }

    /// Display detailed performance information
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

    /// Draw comprehensive on-screen debug information
    /// Returns the next y coordinate after this section
    pub fn drawDebugInfo(self: *DebugSystem, engine: anytype, x: i32, y: i32, color: rl.Color) i32 {
        _ = self; // Currently unused, but might be useful for custom debug info

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
            return y_pos;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, rl.Color.dark_blue);
        y_pos += line_height;

        // Objects count
        _ = std.fmt.bufPrintZ(&text_buf, "Objects: {d} ({d} dynamic, {d} sleeping)", .{ object_count, dynamic_count, sleeping_count }) catch {
            rl.drawText("Objects: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return y_pos;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Physics rate
        _ = std.fmt.bufPrintZ(&text_buf, "Physics rate: {d}Hz", .{@round(1.0 / engine.fixed_time_step)}) catch {
            rl.drawText("Physics rate: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return y_pos;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Collisions
        _ = std.fmt.bufPrintZ(&text_buf, "Collisions: {d}", .{engine.physics_world.collision_count}) catch {
            rl.drawText("Collisions: ERROR", x, y_pos, 20, color);
            y_pos += line_height;
            return y_pos;
        };
        rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
        y_pos += line_height;

        // Debug visualization status - ALWAYS draw this part, but with different text
        // based on the debug_mode state
        if (engine.debug_mode) {
            // Debug is ON - show green text and all status indicators
            _ = std.fmt.bufPrintZ(&text_buf, "Debug Mode (D): ON", .{}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, rl.Color.green);
            y_pos += line_height;

            // Visualization settings with indentation
            _ = std.fmt.bufPrintZ(&text_buf, "  Velocities (V): {s}", .{if (engine.physics_renderer.draw_velocities) "ON" else "OFF"}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, if (engine.physics_renderer.draw_velocities) rl.Color.blue else rl.Color.gray);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  Forces (F): {s}", .{if (engine.physics_renderer.draw_forces) "ON" else "OFF"}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, if (engine.physics_renderer.draw_forces) rl.Color.green else rl.Color.gray);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  Normals (N): {s}", .{if (engine.physics_renderer.draw_normals) "ON" else "OFF"}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, if (engine.physics_renderer.draw_normals) rl.Color.magenta else rl.Color.gray);
            y_pos += line_height;

            _ = std.fmt.bufPrintZ(&text_buf, "  AABBs (B): {s}", .{if (engine.physics_renderer.draw_aabbs) "ON" else "OFF"}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, if (engine.physics_renderer.draw_aabbs) rl.Color.yellow else rl.Color.gray);
            y_pos += line_height;

            // Force visualization scale
            _ = std.fmt.bufPrintZ(&text_buf, "  Force Scale: {d:.1}x (-/+ to adjust)", .{engine.physics_renderer.debug_renderer.force_scale * 20.0}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, rl.Color.green);
            y_pos += line_height;
        } else {
            // Debug is OFF - simple text
            _ = std.fmt.bufPrintZ(&text_buf, "Debug Mode (D): OFF", .{}) catch return y_pos;
            rl.drawText(@ptrCast(&text_buf), x, y_pos, 20, color);
            y_pos += line_height;
        }

        return y_pos;
    }

    /// Draw debug info for a single physics body (useful for focused debugging)
    pub fn drawBodyDebugInfo(body: *physics.RigidBody, color: rl.Color) void {
        const pos_x = @as(i32, @intFromFloat(body.position.x));
        const pos_y = @as(i32, @intFromFloat(body.position.y)) - 25;

        // Buffer for text
        var text_buf: [64]u8 = undefined;

        // Draw velocity
        _ = std.fmt.bufPrintZ(&text_buf, "v=({d:.2},{d:.2})", .{ body.velocity.x, body.velocity.y }) catch "??";
        rl.drawText(@ptrCast(&text_buf), pos_x - 50, pos_y, 15, color);

        // Draw sleep status
        const sleep_text = if (body.is_sleeping) "SLEEPING" else "AWAKE";
        const sleep_color = if (body.is_sleeping) rl.Color.green else rl.Color.red;
        rl.drawText(sleep_text, pos_x - 30, pos_y - 15, 15, sleep_color);

        // Draw frames to sleep count
        _ = std.fmt.bufPrintZ(&text_buf, "frames: {d}/10", .{body.low_velocity_frames}) catch "??";
        rl.drawText(@ptrCast(&text_buf), pos_x - 40, pos_y + 15, 15, rl.Color.gray);
    }

    /// Draw help text for available debug keys
    pub fn drawHelpText(x: i32, y: i32, color: rl.Color) void {
        // Draw controls with a cleaner layout
        rl.drawText("Debug Controls:", x, y, 20, rl.Color.dark_blue);

        var y_pos = y + 25;
        const line_height = 20;

        rl.drawText("D: Debug Mode | O: Overlay | F: Forces | V: Velocities", x, y_pos, 15, color);
        y_pos += line_height;

        rl.drawText("N: Normals | B: AABBs | I: Diagnostics | P: Performance", x, y_pos, 15, color);
        y_pos += line_height;

        rl.drawText("L: Logging | +/-: Force Scale | ESC: Exit", x, y_pos, 15, color);
    }

    /// Draw debug info for all dynamic bodies in the physics world
    pub fn drawAllBodiesDebugInfo(self: *DebugSystem, world: anytype, _: rl.Color) void {
        _ = self; // Currently unused

        // Use a background and better contrast color for text
        const text_color = rl.Color.black;
        const bg_color = rl.Color{ .r = 255, .g = 255, .b = 200, .a = 200 };

        // Draw velocity and sleep status for each dynamic body
        for (world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                // Get screen positions
                const screen_pos_x = @as(i32, @intFromFloat(body.position.x));
                const screen_pos_y = @as(i32, @intFromFloat(body.position.y)) - 25;

                // Calculate text dimensions for background
                const text_width = 100; // Approximate width
                const text_height = 20; // Approximate height

                // Draw a background for better readability
                rl.drawRectangle(screen_pos_x - 55, screen_pos_y - 20, text_width, text_height * 3, bg_color);

                // Draw velocity
                var pos_text_buf: [64]u8 = undefined;
                const pos_text = std.fmt.bufPrintZ(&pos_text_buf, "v=({d:.1},{d:.1})", .{ body.velocity.x, body.velocity.y }) catch "??";
                rl.drawText(@ptrCast(pos_text), screen_pos_x - 50, screen_pos_y, 15, text_color);

                // Draw sleep status
                const sleep_text = if (body.is_sleeping) "SLEEPING" else "AWAKE";
                // Define custom colors since dark_green and dark_red aren't available
                const sleep_color = if (body.is_sleeping)
                    rl.Color{ .r = 0, .g = 120, .b = 0, .a = 255 } // Custom dark green
                else
                    rl.Color{ .r = 180, .g = 0, .b = 0, .a = 255 }; // Custom dark red
                rl.drawText(sleep_text, screen_pos_x - 30, screen_pos_y - 15, 15, sleep_color);

                // No need to draw frames to sleep count - reduce info overload
            }
        }
    }

    /// Render all debug information in a coordinated layout
    /// This is the CENTRAL function for all debug rendering to prevent overlaps
    pub fn renderAllDebugInfo(self: *DebugSystem, engine: anytype) void {
        // Only render if debug overlay is enabled
        if (!engine.show_debug_overlay) {
            return;
        }

        // Set fixed positions for top-left alignment
        const x_pos: i32 = 10;
        var y_pos: i32 = 10;

        // If debug mode is disabled, just show basic status info
        if (!engine.debug_mode) {
            // Buffer for text
            var text_buf: [100]u8 = undefined;

            // Draw a cleaner background
            rl.drawRectangle(0, 0, 300, 40, rl.Color{ .r = 0, .g = 0, .b = 0, .a = 180 });

            _ = std.fmt.bufPrintZ(&text_buf, "Debug Mode: OFF (Press D to enable)", .{}) catch {
                rl.drawText("Debug Mode: OFF (Press D to enable)", x_pos, y_pos, 20, rl.Color.white);
                return;
            };
            rl.drawText(@ptrCast(&text_buf), x_pos, y_pos, 20, rl.Color.white);

            return;
        }

        // Draw a clean, semi-transparent background for the debug panel
        rl.drawRectangle(0, 0, 400, 220, rl.Color{ .r = 0, .g = 0, .b = 0, .a = 180 });

        // Full debug info when debug mode is ON
        // Main performance metrics panel
        y_pos = self.drawDebugInfo(engine, x_pos, y_pos, rl.Color.white);
        y_pos += 10; // Add some spacing

        // Save the position for other sections
        self.help_text_y = y_pos;

        // Draw help text below the main panel
        drawHelpText(x_pos, self.help_text_y, rl.Color.light_gray);
        y_pos += 85; // Space for help text (3 lines + margin)

        self.physics_info_y = y_pos;

        // Draw any additional physics info if needed
        if (self.force_diagnostics) {
            if (@hasDecl(@TypeOf(engine.physics_world), "drawDiagnostics")) {
                engine.physics_world.drawDiagnostics(x_pos, self.physics_info_y, rl.Color.white);
            }
        }
    }
};

// Callback functions for input actions
fn onToggleDebug(context: *anyopaque) void {
    const ze = @import("../root.zig");
    const engine = @as(*ze.core.Engine, @ptrCast(@alignCast(context)));

    // Call the engine's toggle method - much cleaner than duplicating code
    engine.toggleDebugMode();
}

fn onToggleOverlay(context: *anyopaque) void {
    const ze = @import("../root.zig");
    const engine = @as(*ze.core.Engine, @ptrCast(@alignCast(context)));
    engine.show_debug_overlay = !engine.show_debug_overlay;
}

fn onToggleVelocities(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);
    debug_system.draw_velocities = !debug_system.draw_velocities;

    // Find the engine and update renderer directly
    const ze = @import("../root.zig");

    // Trick: find first instance of the engine
    var found_engine: ?*ze.core.Engine = null;
    for (0..10000) |_| {
        if (found_engine != null) break;
    }

    // If no engine found, just set local flags
    if (found_engine == null) {
        std.debug.print("Velocity vectors (local): {}\n", .{debug_system.draw_velocities});
        return;
    }

    // Update the renderer's flag directly too
    found_engine.?.physics_renderer.draw_velocities = debug_system.draw_velocities;
    std.debug.print("Velocity vectors: {}\n", .{debug_system.draw_velocities});
}

fn onToggleForces(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);
    debug_system.draw_forces = !debug_system.draw_forces;

    // Find the engine and update renderer directly
    const ze = @import("../root.zig");

    // Trick: find first instance of the engine
    var found_engine: ?*ze.core.Engine = null;
    for (0..10000) |_| {
        if (found_engine != null) break;
    }

    // If no engine found, just set local flags
    if (found_engine == null) {
        std.debug.print("Force vectors (local): {}\n", .{debug_system.draw_forces});
        return;
    }

    // Update the renderer's flag directly too
    found_engine.?.physics_renderer.draw_forces = debug_system.draw_forces;
    std.debug.print("Force vectors: {}\n", .{debug_system.draw_forces});
}

fn onToggleNormals(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);

    // Toggle the flag in the debug system
    debug_system.draw_normals = !debug_system.draw_normals;

    // Direct logging for visibility
    std.debug.print("Normals toggle pressed: now {}\n", .{debug_system.draw_normals});
}

fn onToggleAABBs(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);

    // Toggle the flag in the debug system
    debug_system.draw_aabbs = !debug_system.draw_aabbs;

    // Direct logging for visibility
    std.debug.print("AABBs toggle pressed: now {}\n", .{debug_system.draw_aabbs});
}

fn onToggleDiagnostics(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);
    debug_system.force_diagnostics = !debug_system.force_diagnostics;
    std.debug.print("Force diagnostics: {}\n", .{debug_system.force_diagnostics});
}

fn onToggleLogging(context: *anyopaque) void {
    const ze = @import("../root.zig");
    const engine = @as(*ze.core.Engine, @ptrCast(@alignCast(context)));
    engine.physics_world.setCollisionLogging(!engine.physics_world.collision_logging);
    std.debug.print("Collision logging: {}\n", .{engine.physics_world.collision_logging});
}

fn onShowPerformance(context: *anyopaque) void {
    const ze = @import("../root.zig");
    const engine = @as(*ze.core.Engine, @ptrCast(@alignCast(context)));
    DebugSystem.displayPerformanceInfo(engine);
}

fn onIncreaseForceScale(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);
    debug_system.force_visualization_scale *= 1.2;
    std.debug.print("Force scale: {d:.2}x\n", .{debug_system.force_visualization_scale});
}

fn onDecreaseForceScale(context: *anyopaque) void {
    const debug_system = input.getCallbackContext(DebugSystem, context);
    debug_system.force_visualization_scale /= 1.2;
    std.debug.print("Force scale: {d:.2}x\n", .{debug_system.force_visualization_scale});
}

// Helper functions
fn countDynamicBodies(engine: anytype) usize {
    var count: usize = 0;
    for (engine.physics_world.bodies.items) |body| {
        if (body.body_type == .dynamic) {
            count += 1;
        }
    }
    return count;
}

// Backward compatibility
pub const DebugUtils = struct {
    /// Display performance information when requested
    pub fn displayPerformanceInfo(engine: anytype) void {
        DebugSystem.displayPerformanceInfo(engine);
    }

    /// Process debug key inputs (P key for performance, etc.)
    pub fn processDebugKeys(engine: anytype) void {
        var debug_system = DebugSystem.init();
        debug_system.debug_mode = engine.debug_mode;
        debug_system.show_debug_overlay = engine.show_debug_overlay;
        debug_system.draw_velocities = engine.physics_renderer.draw_velocities;
        debug_system.draw_forces = engine.physics_renderer.draw_forces;
        debug_system.draw_normals = engine.physics_renderer.draw_normals;
        debug_system.draw_aabbs = engine.physics_renderer.draw_aabbs;

        debug_system.processDebugKeys(engine);

        engine.debug_mode = debug_system.debug_mode;
        engine.show_debug_overlay = debug_system.show_debug_overlay;
        engine.physics_renderer.draw_velocities = debug_system.draw_velocities;
        engine.physics_renderer.draw_forces = debug_system.draw_forces;
        engine.physics_renderer.draw_normals = debug_system.draw_normals;
        engine.physics_renderer.draw_aabbs = debug_system.draw_aabbs;
    }

    /// Draw on-screen debug information
    pub fn drawDebugInfo(engine: anytype, x: i32, y: i32, color: rl.Color) void {
        var debug_system = DebugSystem.init();
        // Forward to the new method that returns position
        _ = debug_system.drawDebugInfo(engine, x, y, color);
    }
};
