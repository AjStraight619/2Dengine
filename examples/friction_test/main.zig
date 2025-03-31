const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

// Example: Friction Test
// This example demonstrates the friction system with objects sliding on the ground

// Context to pass both engine and debug helper to callbacks
const DebugContext = struct {
    game_engine: *ze.core.Engine,
    debug_helper: *ze.core.debug_helper.ExampleDebugHelper,
};

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "Friction Test", options);
    defer game_engine.deinit();

    // Initialize debug helper for consistent debugging across examples
    var debug_helper = ze.core.debug_helper.ExampleDebugHelper.init(&game_engine);

    // Set up game-specific input bindings
    try setupInputBindings(&game_engine);

    // Register input callbacks
    try setupInputCallbacks(&game_engine, &debug_helper);

    // Set physics update rate
    game_engine.setTargetFps(120);

    // Use normal gravity - we want objects to rest on ground
    game_engine.physics_world.gravity = ze.math.Vector2.init(0, 300);

    // Create floor with various friction values
    // Low friction section (left)
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(200, 500),
        .width = 400,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.1, // Low friction
    });

    // High friction section (right)
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(600, 500),
        .width = 400,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.9, // High friction
    });

    // Left and right walls
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(10, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.5,
    });

    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(790, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.5,
    });

    // Run the engine with our custom input and update handlers and debug enabled
    try game_engine.runWithDebug(@ptrCast(@constCast(&DebugContext{ .game_engine = &game_engine, .debug_helper = &debug_helper })), handleInput, update);
}

// Set up the game-specific input bindings
fn setupInputBindings(engine: *ze.core.Engine) !void {
    // Movement forces
    try engine.input_manager.bindInput("move_right", .{ .keyboard = rl.KeyboardKey.right });
    try engine.input_manager.bindInput("move_left", .{ .keyboard = rl.KeyboardKey.left });

    // Objects with different friction values
    try engine.input_manager.bindInput("spawn_low_friction_box", .{ .keyboard = rl.KeyboardKey.one });
    try engine.input_manager.bindInput("spawn_medium_friction_box", .{ .keyboard = rl.KeyboardKey.two });
    try engine.input_manager.bindInput("spawn_high_friction_box", .{ .keyboard = rl.KeyboardKey.three });
    try engine.input_manager.bindInput("spawn_low_friction_circle", .{ .keyboard = rl.KeyboardKey.four });
    try engine.input_manager.bindInput("spawn_high_friction_circle", .{ .keyboard = rl.KeyboardKey.five });

    // Simulation control
    try engine.input_manager.bindInput("reset_simulation", .{ .keyboard = rl.KeyboardKey.r });
    try engine.input_manager.bindInput("detailed_diagnostics", .{ .keyboard = rl.KeyboardKey.i });
}

// Callback context shared between all input callbacks
const InputContext = struct {
    engine: *ze.core.Engine,
    debug_helper: *ze.core.debug_helper.ExampleDebugHelper,
};

// Register callbacks for all input actions
fn setupInputCallbacks(engine: *ze.core.Engine, debug_helper: *ze.core.debug_helper.ExampleDebugHelper) !void {
    var context = InputContext{
        .engine = engine,
        .debug_helper = debug_helper,
    };
    const ctx_ptr: *anyopaque = @ptrCast(&context);

    // Movement
    try engine.input_manager.registerAction("move_right", onMoveRight, ctx_ptr, .down);
    try engine.input_manager.registerAction("move_left", onMoveLeft, ctx_ptr, .down);

    // Object spawning
    try engine.input_manager.registerAction("spawn_low_friction_box", onSpawnLowFrictionBox, ctx_ptr, .pressed);
    try engine.input_manager.registerAction("spawn_medium_friction_box", onSpawnMediumFrictionBox, ctx_ptr, .pressed);
    try engine.input_manager.registerAction("spawn_high_friction_box", onSpawnHighFrictionBox, ctx_ptr, .pressed);
    try engine.input_manager.registerAction("spawn_low_friction_circle", onSpawnLowFrictionCircle, ctx_ptr, .pressed);
    try engine.input_manager.registerAction("spawn_high_friction_circle", onSpawnHighFrictionCircle, ctx_ptr, .pressed);

    // Simulation control
    try engine.input_manager.registerAction("reset_simulation", onResetSimulation, ctx_ptr, .pressed);
    try engine.input_manager.registerAction("detailed_diagnostics", onDetailedDiagnostics, ctx_ptr, .pressed);
}

// Input callback functions
fn onMoveRight(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    for (ctx.engine.physics_world.bodies.items) |body| {
        if (body.body_type == .dynamic) {
            body.applyForce(ze.math.Vector2.init(200, 0));
        }
    }
}

fn onMoveLeft(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    for (ctx.engine.physics_world.bodies.items) |body| {
        if (body.body_type == .dynamic) {
            body.applyForce(ze.math.Vector2.init(-200, 0));
        }
    }
}

fn onSpawnLowFrictionBox(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Add a box with low friction
    _ = ctx.engine.physics_world.dynamic().rectangle(.{
        .position = ze.math.Vector2.init(100, 200),
        .width = 40,
        .height = 40,
        .mass = 1.0,
        .restitution = 0.1,
        .friction = 0.1, // Low friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    }) catch unreachable;
}

fn onSpawnMediumFrictionBox(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Add a box with medium friction
    _ = ctx.engine.physics_world.dynamic().rectangle(.{
        .position = ze.math.Vector2.init(100, 200),
        .width = 40,
        .height = 40,
        .mass = 1.0,
        .restitution = 0.1,
        .friction = 0.5, // Medium friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    }) catch unreachable;
}

fn onSpawnHighFrictionBox(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Add a box with high friction
    _ = ctx.engine.physics_world.dynamic().rectangle(.{
        .position = ze.math.Vector2.init(100, 200),
        .width = 40,
        .height = 40,
        .mass = 1.0,
        .restitution = 0.1,
        .friction = 0.9, // High friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    }) catch unreachable;
}

fn onSpawnLowFrictionCircle(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Add a circle with low friction
    _ = ctx.engine.physics_world.dynamic().circle(.{
        .position = ze.math.Vector2.init(100, 200),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.1,
        .friction = 0.1, // Low friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    }) catch unreachable;
}

fn onSpawnHighFrictionCircle(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Add a circle with high friction
    _ = ctx.engine.physics_world.dynamic().circle(.{
        .position = ze.math.Vector2.init(100, 200),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.1,
        .friction = 0.9, // High friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    }) catch unreachable;
}

fn onResetSimulation(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    // Clear all dynamic bodies
    var i: usize = 0;
    while (i < ctx.engine.physics_world.bodies.items.len) {
        const body = ctx.engine.physics_world.bodies.items[i];
        if (body.body_type == .dynamic) {
            _ = ctx.engine.physics_world.bodies.swapRemove(i);
            ctx.engine.physics_world.allocator.destroy(body);
        } else {
            i += 1;
        }
    }
}

fn onDetailedDiagnostics(context: *anyopaque) void {
    const ctx = @as(*InputContext, @ptrCast(@alignCast(context)));

    std.debug.print("\n=== DETAILED FRICTION DIAGNOSTICS ===\n", .{});

    // Log the state of all dynamic bodies
    for (ctx.engine.physics_world.bodies.items, 0..) |body, i| {
        if (body.body_type == .dynamic) {
            std.debug.print("Body {d}: pos=({d:.2},{d:.2}), vel=({d:.2},{d:.2}), friction={d:.2}\n", .{ i, body.position.x, body.position.y, body.velocity.x, body.velocity.y, body.friction });
        }
    }

    // Force very detailed diagnostics on physics (will print friction calculation)
    ctx.engine.physics_world.force_diagnostics = true;
}

// Input handler callback - much simpler now that we use the InputManager
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*DebugContext, @ptrCast(@alignCast(ctx)));

    // Process debug helper input
    self.debug_helper.processInput();

    // The rest of the input handling is now done through the InputManager callbacks
}

// Update callback
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = dt;
    _ = eng; // Engine parameter is provided by the engine but not used directly here
    const self = @as(*DebugContext, @ptrCast(@alignCast(ctx)));

    // Draw instructions
    rl.drawText("FRICTION TEST: Numbers 1-5: Add objects with different friction", 10, 10, 20, rl.Color.white);
    rl.drawText("LEFT/RIGHT: Apply force | R: Reset | I: Diagnostics | P: Performance", 10, 35, 20, rl.Color.white);

    // Draw floor sections with friction values
    rl.drawText("Low Friction (0.1)", 150, 520, 20, rl.Color.red);
    rl.drawText("High Friction (0.9)", 550, 520, 20, rl.Color.green);

    // Draw vertical line separating the sections
    rl.drawLine(400, 490, 400, 510, rl.Color.white);

    // Draw debug controls
    self.debug_helper.drawDebugControls(10, 60);

    // No need to manually draw body debug info - the engine handles it in debug mode
}
