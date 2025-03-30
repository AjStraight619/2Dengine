const std = @import("std");
const rl = @import("raylib");

const physics = @import("../physics/mod.zig");
const input_module = @import("../input/mod.zig");
const renderer = @import("../renderer/mod.zig");
const args_module = @import("args.zig");

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

    pub fn init(allocator: std.mem.Allocator, window_width: i32, window_height: i32, window_title: [:0]const u8, options_opt: ?args_module.EngineOptions) !Engine {
        rl.initWindow(window_width, window_height, window_title);
        const input_manager = try input_module.InputManager.init(allocator);
        const options = options_opt orelse args_module.EngineOptions{};
        var engine = Engine{
            .physics_world = physics.PhysicsWorld.init(allocator),
            .allocator = allocator,
            .input_manager = input_manager,
            .physics_renderer = renderer.PhysicsRenderer.init(),
            .window_width = window_width,
            .window_height = window_height,
            .window_title = window_title,
            .target_fps = options.target_fps,
        };

        engine.physics_renderer.setDebugMode(options.debug_mode);
        engine.physics_renderer.draw_forces = options.draw_forces;
        engine.physics_renderer.draw_velocities = options.draw_velocities;
        engine.physics_renderer.draw_normals = options.draw_normals;
        engine.physics_renderer.draw_aabbs = options.draw_aabbs;

        return engine;
    }

    fn setupDefaultInputBindings(input_manager: *input_module.InputManager) !void {
        // Pause toggle - Space key
        try input_manager.bindInput("toggle_pause", .{ .keyboard = rl.KeyboardKey.space });

        // Toggle collision logging - L key
        try input_manager.bindInput("toggle_collision_logging", .{ .keyboard = rl.KeyboardKey.l });

        // Toggle debug mode - D key
        try input_manager.bindInput("toggle_debug_mode", .{ .keyboard = rl.KeyboardKey.d });
    }

    pub fn update(self: *Engine) void {
        self.delta_time = rl.getFrameTime();

        if (!self.paused) {
            // Update physics
            self.physics_world.update(self.delta_time);
        }
    }

    pub fn processInput(self: *Engine) void {
        // Process input
        self.input_manager.update() catch |err| {
            std.debug.print("Error processing input: {}\n", .{err});
        };
    }

    pub fn render(self: *Engine) void {
        // Render physics world
        rl.beginDrawing();
        defer rl.endDrawing();

        self.physics_renderer.drawGrid(self.window_width, self.window_height, 10, rl.Color.gray);

        rl.clearBackground(rl.Color.ray_white);

        self.physics_renderer.drawWorld(self.physics_world);

        renderer.PhysicsRenderer.drawDebugInfo(self.physics_world, self.paused, 20, 20, rl.Color.dark_gray);
    }

    pub fn run(self: *Engine, user_context: *anyopaque, handle_input_fn: *const fn (ctx: *anyopaque, engine: *Engine) anyerror!void, update_fn: *const fn (ctx: *anyopaque, engine: *Engine, delta_time: f32) anyerror!void) !void {
        self.setTargetFps(self.target_fps);

        // Set up input callbacks
        try self.setupInputCallbacks();

        // Game loop
        while (!self.shouldClose()) {
            // Process input
            self.processInput();

            // Call user input handler
            try handle_input_fn(user_context, self);

            // Get delta time for frame-rate independent physics
            self.delta_time = rl.getFrameTime();

            // Update physics if not paused
            if (!self.paused) {
                self.physics_world.update(self.delta_time);
            }

            // Call user update function
            try update_fn(user_context, self, self.delta_time);

            // Render
            self.render();
        }
    }

    pub fn deinit(self: *Engine) void {
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
        self.debug_mode = !self.debug_mode;
        self.physics_renderer.setDebugMode(self.debug_mode);
        std.debug.print("Debug mode: {}\n", .{self.debug_mode});
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
        const self = input_module.getCallbackContext(Engine, context);
        self.toggleDebugMode();
    }

    // Register callback functions for input actions
    pub fn setupInputCallbacks(self: *Engine) !void {
        const user_ptr: *anyopaque = @ptrCast(self);

        try self.input_manager.registerAction("toggle_pause", onTogglePause, user_ptr, .pressed);
        try self.input_manager.registerAction("toggle_collision_logging", onToggleCollisionLogging, user_ptr, .pressed);
        try self.input_manager.registerAction("toggle_debug_mode", onToggleDebugMode, user_ptr, .pressed);
    }
};
