const std = @import("std");
const rl = @import("raylib");

const physics = @import("../physics/mod.zig");
const input_module = @import("../input/mod.zig");
const renderer = @import("../renderer/mod.zig");
const args_module = @import("args.zig");
const debug_module = @import("debug.zig");

pub const Engine = struct {
    physics_world: physics.PhysicsWorld,
    allocator: std.mem.Allocator,
    debug_mode: bool = false,
    delta_time: f32 = 0.0,
    paused: bool = false,
    input_manager: *input_module.InputManager,
    physics_renderer: renderer.PhysicsRenderer,
    window_width: i32,
    window_height: i32,
    window_title: [:0]const u8,
    target_fps: i32,

    // Debug system
    debug_system: debug_module.DebugSystem,

    // Time management
    fixed_time_step: f32 = 1.0 / 60.0, // Default physics step at 60Hz
    time_accumulator: f32 = 0.0,
    time_scale: f32 = 1.0,

    // Debug flags
    show_debug_overlay: bool = false,

    pub fn init(allocator: std.mem.Allocator, window_width: i32, window_height: i32, window_title: [:0]const u8, options_opt: ?args_module.EngineOptions) !Engine {
        rl.initWindow(window_width, window_height, window_title);
        const input_manager = try input_module.InputManager.init(allocator);
        const options = options_opt orelse args_module.EngineOptions{};

        // Create the debug system
        var debug_system = debug_module.DebugSystem.init();

        var engine = Engine{
            .physics_world = physics.PhysicsWorld.init(allocator),
            .allocator = allocator,
            .input_manager = input_manager,
            .physics_renderer = renderer.PhysicsRenderer.init(),
            .window_width = window_width,
            .window_height = window_height,
            .window_title = window_title,
            .target_fps = options.target_fps,
            .debug_system = debug_system,
        };

        // Initialize debug options
        engine.debug_mode = options.debug_mode;
        engine.debug_system.debug_mode = options.debug_mode;
        engine.debug_system.draw_forces = options.draw_forces;
        engine.debug_system.draw_velocities = options.draw_velocities;
        engine.debug_system.draw_normals = options.draw_normals;
        engine.debug_system.draw_aabbs = options.draw_aabbs;

        // Set up renderer with debug options
        engine.physics_renderer.setDebugMode(options.debug_mode);
        engine.physics_renderer.draw_forces = options.draw_forces;
        engine.physics_renderer.draw_velocities = options.draw_velocities;
        engine.physics_renderer.draw_normals = options.draw_normals;
        engine.physics_renderer.draw_aabbs = options.draw_aabbs;

        // Set up debug input bindings
        try debug_system.setupInputBindings(input_manager);
        try debug_system.setupCallbacks(input_manager, &engine);

        // Make sure everything is in sync
        engine.syncDebugState();

        return engine;
    }

    pub fn update(self: *Engine) void {
        // Get delta time and apply time scale
        self.delta_time = rl.getFrameTime() * self.time_scale;

        if (!self.paused) {
            // Update physics with fixed time step
            self.updatePhysics();
        }
    }

    fn updatePhysics(self: *Engine) void {
        // Accumulate time for fixed-step updates
        self.time_accumulator += self.delta_time;

        // CRITICAL: Prevent spiral of death by clamping max accumulated time
        const max_accumulated_time = self.fixed_time_step * 5.0; // Max 5 steps per frame
        if (self.time_accumulator > max_accumulated_time) {
            std.debug.print("⚠️ WARNING: Time accumulator exceeded maximum ({d:.4} > {d:.4}), clamping to prevent spiral of death\n", .{ self.time_accumulator, max_accumulated_time });
            self.time_accumulator = max_accumulated_time;
        }

        // For diagnosing frame spikes
        var steps_taken: usize = 0;
        const start_time = std.time.nanoTimestamp();

        // Perform as many fixed steps as needed
        while (self.time_accumulator >= self.fixed_time_step) {
            const step_start = std.time.nanoTimestamp();

            self.physics_world.update(self.fixed_time_step);
            self.time_accumulator -= self.fixed_time_step;

            steps_taken += 1;

            const step_end = std.time.nanoTimestamp();
            const step_time_ms = @as(f64, @floatFromInt(step_end - step_start)) / 1_000_000.0;

            // Log if a single physics step takes too long
            if (step_time_ms > 15.0) { // 15ms is a significant part of a 16.67ms frame
                std.debug.print("🕒 SLOW PHYSICS STEP: {d:.2}ms\n", .{step_time_ms});
            }

            // Break if we're exceeding our frame budget
            if (steps_taken >= 5) {
                std.debug.print("⚠️ MAX PHYSICS STEPS ({d}) REACHED IN ONE FRAME!\n", .{steps_taken});
                break;
            }
        }

        const total_time = @as(f64, @floatFromInt(std.time.nanoTimestamp() - start_time)) / 1_000_000.0;

        // Log if physics updates take too much time in total
        if (total_time > 15.0) {
            std.debug.print("🕒 PHYSICS TOOK {d:.2}ms ({d} STEPS)\n", .{ total_time, steps_taken });
        }

        // If needed, you could also do an interpolation step here
        // based on the remaining time in accumulator
    }

    pub fn processInput(self: *Engine) void {
        // Process input
        self.input_manager.update() catch |err| {
            std.debug.print("Error processing input: {}\n", .{err});
        };

        // Process debug keys directly
        self.debug_system.processDebugKeys(self);

        // Update debug system
        self.debug_system.update();

        // Make sure debug system and engine are in sync
        self.syncDebugState();

        // Force the renderer to reflect the debug system flags
        self.physics_renderer.debug_mode = self.debug_mode;
        self.physics_renderer.draw_velocities = self.debug_system.draw_velocities;
        self.physics_renderer.draw_forces = self.debug_system.draw_forces;
        self.physics_renderer.draw_normals = self.debug_system.draw_normals;
        self.physics_renderer.draw_aabbs = self.debug_system.draw_aabbs;
    }

    // Synchronize debug flags between engine and debug system
    pub fn syncDebugState(self: *Engine) void {
        // Keep debug_mode synchronized between engine and debug system first
        self.debug_system.debug_mode = self.debug_mode;
        self.debug_system.show_debug_overlay = self.show_debug_overlay;

        // Make sure all debug visualization flags are off when debug mode is off
        if (!self.debug_mode) {
            // When debug is off, all visualizations should be off
            self.physics_renderer.draw_forces = false;
            self.physics_renderer.draw_velocities = false;
            self.physics_renderer.draw_normals = false;
            self.physics_renderer.draw_aabbs = false;
            self.debug_system.draw_forces = false;
            self.debug_system.draw_velocities = false;
            self.debug_system.draw_normals = false;
            self.debug_system.draw_aabbs = false;
        } else {
            // When debug mode is on, sync visibility flags in both directions
            // This ensures toggles work regardless of where they're triggered

            // Look at all four combinations and take the most "active" state
            // This ensures if a flag is ON anywhere, it propagates everywhere
            self.physics_renderer.draw_forces = self.physics_renderer.draw_forces or self.debug_system.draw_forces;
            self.physics_renderer.draw_velocities = self.physics_renderer.draw_velocities or self.debug_system.draw_velocities;
            self.physics_renderer.draw_normals = self.physics_renderer.draw_normals or self.debug_system.draw_normals;
            self.physics_renderer.draw_aabbs = self.physics_renderer.draw_aabbs or self.debug_system.draw_aabbs;

            // Now sync back to debug system
            self.debug_system.draw_forces = self.physics_renderer.draw_forces;
            self.debug_system.draw_velocities = self.physics_renderer.draw_velocities;
            self.debug_system.draw_normals = self.physics_renderer.draw_normals;
            self.debug_system.draw_aabbs = self.physics_renderer.draw_aabbs;
        }
    }

    pub fn render(self: *Engine) void {
        // Start the drawing
        rl.beginDrawing();
        defer rl.endDrawing();

        // Clear background
        rl.clearBackground(rl.Color.ray_white);

        // Draw grid first (background)
        self.physics_renderer.drawGrid(self.window_width, self.window_height, 10, rl.Color.gray);

        // Draw physics objects
        self.physics_renderer.drawWorld(self.physics_world);

        // Draw body debug info when in debug mode
        if (self.debug_mode) {
            self.debug_system.drawAllBodiesDebugInfo(&self.physics_world, rl.Color.white);
        }

        // Draw debug overlay if enabled - always last so it's on top
        if (self.show_debug_overlay) {
            // Use the centralized rendering function
            self.debug_system.renderAllDebugInfo(self);
        }
    }

    pub fn run(self: *Engine, user_context: *anyopaque, handle_input_fn: *const fn (ctx: *anyopaque, engine: *Engine) anyerror!void, update_fn: *const fn (ctx: *anyopaque, engine: *Engine, delta_time: f32) anyerror!void) !void {
        self.setTargetFps(self.target_fps);

        // Set up input callbacks
        try self.setupInputCallbacks();

        // Print key bindings for the user
        self.printKeyBindings();

        // Game loop
        while (!self.shouldClose()) {
            // Process input and synchronize debug state
            self.processInput();

            // Call user input handler
            try handle_input_fn(user_context, self);

            // Get delta time and apply time scale
            self.delta_time = rl.getFrameTime() * self.time_scale;

            // Update physics if not paused
            if (!self.paused) {
                // Use fixed timestep for physics
                self.updatePhysics();
            }

            // Call user update function
            try update_fn(user_context, self, self.delta_time);

            // Render
            self.render();
        }
    }

    // Set physics time step and scale
    pub fn setFixedTimeStep(self: *Engine, step: f32) void {
        self.fixed_time_step = step;
    }

    pub fn setTimeScale(self: *Engine, scale: f32) void {
        self.time_scale = scale;
    }

    pub fn deinit(self: *Engine) void {
        // Log all sleeping bodies before exiting
        std.debug.print("\n=== SLEEP STATE AT EXIT ===\n", .{});

        var total_bodies: usize = 0;
        var sleeping_bodies: usize = 0;
        var dynamic_bodies: usize = 0;

        for (self.physics_world.bodies.items) |body| {
            total_bodies += 1;

            if (body.body_type == .dynamic) {
                dynamic_bodies += 1;

                if (body.is_sleeping) {
                    sleeping_bodies += 1;
                    std.debug.print("SLEEPING: Body at ({d:.1}, {d:.1}) - vel=({d:.4}, {d:.4}), type={any}\n", .{ body.position.x, body.position.y, body.velocity.x, body.velocity.y, body.shape });
                } else {
                    std.debug.print("AWAKE: Body at ({d:.1}, {d:.1}) - vel=({d:.4}, {d:.4}), low_velocity_frames={d}\n", .{ body.position.x, body.position.y, body.velocity.x, body.velocity.y, body.low_velocity_frames });
                }
            }
        }

        std.debug.print("Sleep summary: {d}/{d} dynamic bodies sleeping ({d}% of total {d} bodies)\n", .{ sleeping_bodies, dynamic_bodies, if (dynamic_bodies > 0) sleeping_bodies * 100 / dynamic_bodies else 0, total_bodies });

        // Perform regular cleanup
        self.input_manager.deinit();
        self.physics_world.deinit();
        rl.closeWindow();
    }

    pub fn setTargetFps(self: *Engine, fps: i32) void {
        self.target_fps = fps;
        rl.setTargetFPS(fps);
    }

    pub fn togglePause(self: *Engine) void {
        self.paused = !self.paused;
    }

    pub fn toggleDebugMode(self: *Engine) void {
        std.debug.print("DEBUG MODE TOGGLE CALLED\n", .{});

        // Debug the current state
        std.debug.print("Current state: debug_mode={}, renderer.debug_mode={}\n", .{ self.debug_mode, self.physics_renderer.debug_mode });

        // Simply toggle the debug mode
        self.debug_mode = !self.debug_mode;

        // Sync to physics renderer
        self.physics_renderer.debug_mode = self.debug_mode;

        // Sync to debug system
        self.debug_system.debug_mode = self.debug_mode;

        // When turning debug mode on, also enable the debug overlay
        if (self.debug_mode) {
            self.show_debug_overlay = true;
            self.debug_system.show_debug_overlay = true;
        }

        std.debug.print("Debug mode is now: {}\n", .{self.debug_mode});
    }

    pub fn adjustGravity(self: *Engine, amount: f32) void {
        self.physics_world.gravity.y += amount;
    }

    pub fn shouldClose(_: *Engine) bool {
        return rl.windowShouldClose();
    }

    // Callback functions for input actions
    fn onTogglePause(context: *anyopaque) void {
        const self = input_module.getCallbackContext(Engine, context);
        self.togglePause();
    }

    fn onToggleCollisionLogging(context: *anyopaque) void {
        _ = context; // unused

    }

    fn onToggleDebugMode(context: *anyopaque) void {
        _ = context; // Unused now
    }

    // Register callback functions for input actions
    pub fn setupInputCallbacks(self: *Engine) !void {
        const user_ptr: *anyopaque = @ptrCast(self);

        try self.input_manager.registerAction("toggle_pause", onTogglePause, user_ptr, .pressed);
        try self.input_manager.registerAction("toggle_collision_logging", onToggleCollisionLogging, user_ptr, .pressed);
        try self.input_manager.registerAction("toggle_debug_mode", onToggleDebugMode, user_ptr, .pressed);
    }

    // Draw help text with available debug controls
    pub fn drawDebugControls(self: *Engine, x: i32, y: i32, color: rl.Color) void {
        _ = self;
        debug_module.DebugSystem.drawHelpText(x, y, color);
    }

    // Run with debug mode enabled and all debug features turned on
    pub fn runWithDebug(self: *Engine, user_context: *anyopaque, handle_input_fn: *const fn (ctx: *anyopaque, engine: *Engine) anyerror!void, update_fn: *const fn (ctx: *anyopaque, engine: *Engine, delta_time: f32) anyerror!void) !void {
        // Enable all debug features
        self.debug_mode = true;
        self.show_debug_overlay = true;
        self.physics_renderer.setDebugMode(true);

        // Enable all debug visualizations
        self.debug_system.debug_mode = true;
        self.debug_system.draw_forces = true;
        self.debug_system.draw_velocities = true;
        self.debug_system.draw_normals = true;
        self.debug_system.draw_aabbs = true;

        // Make sure renderer reflects debug settings
        self.physics_renderer.draw_forces = true;
        self.physics_renderer.draw_velocities = true;
        self.physics_renderer.draw_normals = true;
        self.physics_renderer.draw_aabbs = true;

        // Turn on physics debug info
        self.physics_world.setDebugDrawCollisions(true);
        self.physics_world.setDebugDrawContacts(true);

        // Set physics to higher detail for better debugging
        self.setFixedTimeStep(1.0 / 120.0); // 120Hz physics for smoother debugging

        self.printDebugKeyBindings();

        // Run the engine normally with debug enabled
        return self.run(user_context, handle_input_fn, update_fn);
    }

    // Display available key bindings to the console
    pub fn printKeyBindings(self: *Engine) void {
        _ = self;
        std.debug.print("\n=== CONTROLS ===\n", .{});
        std.debug.print("SPACE: Toggle pause\n", .{});
        std.debug.print("D: Toggle debug mode\n", .{});
        std.debug.print("O: Toggle debug overlay\n", .{});
        std.debug.print("ESC: Exit\n", .{});
    }

    // Display available debug key bindings to the console
    pub fn printDebugKeyBindings(self: *Engine) void {
        _ = self;
        std.debug.print("\n=== DEBUG MODE ENABLED ===\n", .{});
        std.debug.print("Debug Controls:\n", .{});
        std.debug.print("  D: Toggle debug mode\n", .{});
        std.debug.print("  V: Toggle velocity vectors\n", .{});
        std.debug.print("  F: Toggle force vectors\n", .{});
        std.debug.print("  N: Toggle normal vectors\n", .{});
        std.debug.print("  B: Toggle AABBs\n", .{});
        std.debug.print("  O: Toggle debug overlay\n", .{});
        std.debug.print("  P: Show performance info\n", .{});
        std.debug.print("  I: Show detailed diagnostics\n", .{});
        std.debug.print("  L: Toggle collision logging\n", .{});
        std.debug.print("  -/+: Adjust force scale\n", .{});
    }
};
